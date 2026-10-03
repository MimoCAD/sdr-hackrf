//! Where a change made while receiving lands, against where the stream's
//! `Break` places it. A reading thread keeps every sample; the control
//! steps the gain between a low and a high setting every `interval_ms`
//! (the noise floor steps with it); afterwards the step is found in the
//! samples' power and compared with each break's `at_sample` and
//! `settled_at`. One JSON line per break, then a summary.
//!
//!   cargo run --release -p sdr-hackrf --example breakcheck -- [rate_sps] [steps] [interval_ms] [low_db] [high_db]

use std::io::Read;
use std::time::Duration;

use sdr_core::TuneRequest;
use sdr_hackrf::blocking::HackRf;

/// Samples per power window.
const WINDOW: usize = 64;
/// How far before `at_sample` the search for the step begins, samples.
const SEARCH_BEFORE: u64 = 200_000;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let rate: f64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(10e6);
    let steps: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(10);
    let interval_ms: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(200);
    let low_db: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let high_db: f64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(70.0);

    let mut h = HackRf::open("")?;
    h.set_rate(rate)?;
    h.set_freq(&TuneRequest::auto(300e6))?;
    h.set_gain(low_db)?;
    let (mut reader, control) = h.into_stream(0, 0).map_err(|e| e.error)?.split();
    let total_s = (steps as u64 + 2) * interval_ms;
    let want_bytes = (rate * 2.0 * total_s as f64 / 1000.0) as usize;
    let read_thread = std::thread::spawn(move || -> std::io::Result<(Vec<u8>, sdr_hackrf::blocking::ReadHalf)> {
        let mut all = vec![0u8; want_bytes];
        let mut got = 0;
        while got < want_bytes {
            got += reader.read(&mut all[got..])?;
        }
        Ok((all, reader))
    });
    let mut breaks = Vec::new();
    for k in 0..steps {
        std::thread::sleep(Duration::from_millis(interval_ms));
        let db = if k % 2 == 0 { high_db } else { low_db };
        breaks.push((db, control.set_gain(db)?));
    }
    let (bytes, reader) = read_thread.join().expect("the reading thread")?;
    let stream = sdr_hackrf::blocking::Stream::reunite(reader, control).map_err(|_| "reunite")?;
    let finished = stream.finish()?;

    // Power per window, in dB.
    let power: Vec<f64> = bytes
        .as_chunks::<{ 2 * WINDOW }>()
        .0
        .iter()
        .map(|w| {
            let e: f64 = w.iter().map(|&b| ((b as i8) as f64).powi(2)).sum();
            10.0 * (e / WINDOW as f64 + 1e-9).log10()
        })
        .collect();
    let median = |v: &mut Vec<f64>| -> f64 {
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let mut offsets_at = Vec::new();
    let mut offsets_settled = Vec::new();
    for (i, (db, b)) in breaks.iter().enumerate() {
        let from = (b.at_sample.saturating_sub(SEARCH_BEFORE) as usize) / WINDOW;
        let mid = b.at_sample as usize / WINDOW;
        let to = ((b.settled_at + SEARCH_BEFORE) as usize / WINDOW).min(power.len());
        if to <= mid + 4 || mid >= power.len() {
            println!("{{\"break\":{i},\"skipped\":\"past the samples kept\"}}");
            continue;
        }
        let before = median(&mut power[from..mid].to_vec());
        let after = median(&mut power[(b.settled_at as usize / WINDOW).min(to - 1)..to].to_vec());
        let threshold = (before + after) / 2.0;
        let rising = after > before;
        let step = (from..to).find(|&w| if rising { power[w] > threshold } else { power[w] < threshold }).map(|w| (w * WINDOW) as i64);
        let line = match step {
            Some(s) => {
                let (da, ds) = (s - b.at_sample as i64, s - b.settled_at as i64);
                offsets_at.push(da);
                offsets_settled.push(ds);
                format!("\"step_at\":{s},\"step_minus_at_sample\":{da},\"step_minus_settled_at\":{ds}")
            }
            None => "\"step_at\":null".into(),
        };
        println!(
            "{{\"break\":{i},\"to_db\":{db},\"at_sample\":{},\"settled_at\":{},\"took_us\":{},\"before_db\":{before:.1},\"after_db\":{after:.1},{line}}}",
            b.at_sample,
            b.settled_at,
            b.took.as_micros()
        );
    }
    let span = |v: &[i64]| (v.iter().min().copied(), v.iter().max().copied());
    println!(
        "{{\"summary\":true,\"rate\":{rate},\"breaks\":{},\"placed\":{},\"step_minus_at_sample\":{:?},\"step_minus_settled_at\":{:?},\"window\":{WINDOW},\"report\":\"{:?}\"}}",
        breaks.len(),
        offsets_at.len(),
        span(&offsets_at),
        span(&offsets_settled),
        finished.report
    );
    Ok(())
}
