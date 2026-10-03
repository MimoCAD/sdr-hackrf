//! Where a retune made while receiving lands, and how long the new tuning
//! takes to settle, against an external tone (a signal generator cabled to
//! the antenna port through attenuation). A reading thread keeps every
//! sample; the control retunes every `interval_ms` between two RFs
//! `step_hz` apart, so the tone sits at `tone − rf` in one and
//! `tone − rf − step` in the other. Afterwards, in windows of `WINDOW`
//! samples correlated against both offsets:
//!
//! - the takeover: the first window where the new offset outweighs the
//!   old, against the `Break`'s `at_sample` and `settled_at`;
//! - the settling: from the takeover, the frequency the windows' phase
//!   advance gives against the new offset, until it stays within
//!   `SETTLED_HZ` for `SETTLED_FOR` windows.
//!
//! One JSON line per break, then a summary. Only receives.
//!
//!   cargo run --release -p sdr-hackrf --example retunecheck -- <tone_hz> [rate_sps] [retunes] [interval_ms] [step_hz]

use std::f64::consts::TAU;
use std::io::Read;
use std::time::Duration;

use sdr_core::TuneRequest;
use sdr_hackrf::blocking::{HackRf, ReadHalf, Stream};

/// Samples per correlation window for the takeover.
const WINDOW: usize = 32;
/// Samples per window for the frequency after it: long enough that the
/// estimate's own noise sits well inside `SETTLED_HZ` (32-sample windows
/// at 4 Msps scattered by ~2 kHz).
const FREQ_WINDOW: usize = 256;
/// A window's frequency within this of the new offset counts as settled.
const SETTLED_HZ: f64 = 500.0;
/// ...for this many frequency windows in a row.
const SETTLED_FOR: usize = 8;
/// Frequency windows printed after each takeover.
const TRACE: usize = 12;
/// How far past `settled_at` the search for the settling runs, samples.
const SEARCH_AFTER: u64 = 400_000;
/// The receive gain: the LNA and VGA split (`gain::split`), amp off.
const GAIN_DB: f64 = 30.0;

/// The complex correlation of a window of wire bytes with e^{-j2π f n / fs},
/// n counted from the window's first sample at absolute index `n0`.
fn corr(bytes: &[u8], n0: usize, f: f64, fs: f64) -> (f64, f64) {
    let (mut re, mut im) = (0.0, 0.0);
    for (k, iq) in bytes.as_chunks::<2>().0.iter().enumerate() {
        let (i, q) = ((iq[0] as i8) as f64, (iq[1] as i8) as f64);
        let ph = -TAU * f * (n0 + k) as f64 / fs;
        let (c, s) = (ph.cos(), ph.sin());
        re += i * c - q * s;
        im += i * s + q * c;
    }
    (re, im)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(tone_hz) = args.first().and_then(|s| s.parse::<f64>().ok()) else {
        eprintln!("usage: retunecheck <tone_hz> [rate_sps] [retunes] [interval_ms] [step_hz]");
        std::process::exit(2);
    };
    let rate: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(4e6);
    let retunes: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(10);
    let interval_ms: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(200);
    let step_hz: f64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(100e3);
    // The tone 300 kHz above the first RF, 300 − step above the second.
    let rf = [tone_hz - 300e3, tone_hz - 300e3 + step_hz];

    let mut h = HackRf::open("")?;
    h.set_rate(rate)?;
    h.set_freq(&TuneRequest::auto(rf[0]))?;
    h.set_amp(false)?;
    h.set_gain(GAIN_DB)?;
    let (mut reader, control) = h.into_stream(0, 0).map_err(|e| e.error)?.split();
    let total_ms = (retunes as u64 + 2) * interval_ms;
    let want = (rate * 2.0 * total_ms as f64 / 1000.0) as usize;
    let read_thread = std::thread::spawn(move || -> std::io::Result<(Vec<u8>, ReadHalf)> {
        let mut all = vec![0u8; want];
        let mut got = 0;
        while got < want {
            got += reader.read(&mut all[got..])?;
        }
        Ok((all, reader))
    });
    let mut breaks = Vec::new();
    for k in 0..retunes {
        std::thread::sleep(Duration::from_millis(interval_ms));
        let to = rf[(k + 1) % 2];
        let (_, b) = control.retune(&TuneRequest::auto(to))?;
        breaks.push((k, b));
    }
    let (bytes, reader) = read_thread.join().expect("the reading thread")?;
    let report = Stream::reunite(reader, control).map_err(|_| "reunite")?.finish()?.report;

    let n_windows = bytes.len() / (2 * WINDOW);
    let window = |w: usize| &bytes[2 * WINDOW * w..2 * WINDOW * (w + 1)];
    let (mut takeovers, mut settles) = (Vec::new(), Vec::new());
    for (k, b) in &breaks {
        let old_off = tone_hz - rf[k % 2];
        let new_off = tone_hz - rf[(k + 1) % 2];
        let from = (b.at_sample as usize / WINDOW).saturating_sub(64);
        let to = ((b.settled_at + SEARCH_AFTER) as usize / WINDOW).min(n_windows);
        let mag = |w: usize, f: f64| {
            let (re, im) = corr(window(w), w * WINDOW, f, rate);
            re.hypot(im)
        };
        let takeover = (from..to).find(|&w| mag(w, new_off) > mag(w, old_off));
        let Some(t) = takeover else {
            println!("{{\"break\":{k},\"at_sample\":{},\"settled_at\":{},\"takeover\":null}}", b.at_sample, b.settled_at);
            continue;
        };
        // The frequency against the new offset in FREQ_WINDOW windows from
        // the takeover, from the phase advance between consecutive ones.
        let t_s = (t * WINDOW) as i64;
        let fw = |j: usize| {
            let n0 = t * WINDOW + j * FREQ_WINDOW;
            (n0 + FREQ_WINDOW <= bytes.len() / 2).then(|| corr(&bytes[2 * n0..2 * (n0 + FREQ_WINDOW)], n0, new_off, rate))
        };
        let last = ((to * WINDOW).saturating_sub(t * WINDOW)) / FREQ_WINDOW;
        let mut run = 0;
        let mut settled = None;
        let mut trace = Vec::new();
        let mut prev = fw(0);
        for j in 1..last {
            let (Some(p), Some(c)) = (prev, fw(j)) else { break };
            let dph = (c.1 * p.0 - c.0 * p.1).atan2(c.0 * p.0 + c.1 * p.1);
            let err_hz = dph / TAU * rate / FREQ_WINDOW as f64;
            if trace.len() < TRACE {
                trace.push(err_hz.round() as i64);
            }
            prev = Some(c);
            run = if err_hz.abs() <= SETTLED_HZ { run + 1 } else { 0 };
            if run == SETTLED_FOR && settled.is_none() {
                settled = Some(j - SETTLED_FOR);
            }
            if settled.is_some() && trace.len() >= TRACE {
                break;
            }
        }
        let settle_s = settled.map(|j| t_s + (j * FREQ_WINDOW) as i64);
        takeovers.push((t_s - b.at_sample as i64, t_s - b.settled_at as i64));
        if let Some(s) = settle_s {
            settles.push(s - b.settled_at as i64);
        }
        println!(
            "{{\"break\":{k},\"to_hz\":{},\"at_sample\":{},\"settled_at\":{},\"took_us\":{},\"takeover\":{t_s},\"takeover_minus_at_sample\":{},\"takeover_minus_settled_at\":{},\"settled\":{},\"settled_minus_settled_at\":{},\"freq_error_hz\":{trace:?}}}",
            rf[(k + 1) % 2],
            b.at_sample,
            b.settled_at,
            b.took.as_micros(),
            t_s - b.at_sample as i64,
            t_s - b.settled_at as i64,
            settle_s.map_or("null".into(), |s| s.to_string()),
            settle_s.map_or("null".into(), |s| (s - b.settled_at as i64).to_string()),
        );
    }
    let span = |v: &mut Vec<i64>| {
        v.sort();
        (v.first().copied(), v.last().copied())
    };
    let mut ta: Vec<i64> = takeovers.iter().map(|t| t.0).collect();
    let mut ts: Vec<i64> = takeovers.iter().map(|t| t.1).collect();
    println!(
        "{{\"summary\":true,\"tone_hz\":{tone_hz},\"rate\":{rate},\"breaks\":{},\"placed\":{},\"takeover_minus_at_sample\":{:?},\"takeover_minus_settled_at\":{:?},\"settled\":{},\"settled_minus_settled_at\":{:?},\"settled_hz\":{SETTLED_HZ},\"window\":{WINDOW},\"freq_window\":{FREQ_WINDOW},\"shortfalls\":{}}}",
        breaks.len(),
        takeovers.len(),
        span(&mut ta),
        span(&mut ts),
        settles.len(),
        span(&mut settles),
        report.shortfalls
    );
    Ok(())
}
