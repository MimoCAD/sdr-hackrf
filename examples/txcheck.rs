//! The transmit stream on the bench: a tone written for `seconds`, the
//! firmware's counters read as it goes, optionally a stall in the writing
//! (an underrun made on purpose), then `finish` and its report. One JSON
//! line per reading and one for the report.
//!
//!   cargo run --release -p sdr-hackrf --example txcheck -- --cabled [rate_sps] [seconds] [stall_ms]
//!
//! `--cabled` is the operator's statement that the antenna port is cabled
//! to a receiver through fixed attenuation and nothing radiates; without
//! it nothing is keyed. The transmit VGA stays at 0 dB and the +14 dB amp
//! off.

use std::f64::consts::TAU;
use std::time::{Duration, Instant};

use sdr_core::{Iq, TuneRequest};
use sdr_hackrf::M0State;
use sdr_hackrf::blocking::HackRf;

/// The tone's offset from the carrier, and its level below full scale.
const TONE_OFFSET_HZ: f64 = 400e3;
const BACKOFF_DB: f64 = -15.0;
/// Where it is sent.
const CARRIER_HZ: f64 = 299.9e6;

fn line(tag: &str, t: f64, written: u64, m: &M0State) {
    println!(
        "{{\"tag\":\"{tag}\",\"t\":{t:.4},\"written_bytes\":{written},\"m0_count\":{},\"m4_count\":{},\"shortfalls\":{},\"longest\":{},\"active_mode\":{}}}",
        m.m0_count, m.m4_count, m.num_shortfalls, m.longest_shortfall, m.active_mode
    );
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("--cabled") {
        eprintln!(
            "txcheck: refusing to key — pass --cabled only when the HackRF is cabled to a receiver through attenuation"
        );
        std::process::exit(2);
    }
    let rate: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(8e6);
    let seconds: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(2.0);
    let stall_ms: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0);

    let mut h = HackRf::open("")?;
    h.set_rate(rate)?;
    h.set_freq(&TuneRequest::auto(CARRIER_HZ))?;
    h.set_amp(false)?;
    h.set_txvga_gain(0)?;
    let mut tx = h.into_tx_stream(0, 0).map_err(|e| e.error)?;
    let spt = tx.samples_per_transfer();
    let amplitude = 32767.0 * 10f64.powf(BACKOFF_DB / 20.0);
    let step = TAU * TONE_OFFSET_HZ / rate;
    let mut phase = 0.0f64;
    let mut chunk = vec![Iq::default(); spt];
    let mut fill = |chunk: &mut [Iq]| {
        for s in chunk.iter_mut() {
            *s = Iq {
                i: (amplitude * phase.cos()) as i16,
                q: (amplitude * phase.sin()) as i16,
            };
            phase = (phase + step) % TAU;
        }
    };

    let t0 = Instant::now();
    let mut stalled = stall_ms == 0;
    let mut next_poll = Duration::from_millis(250);
    while t0.elapsed().as_secs_f64() < seconds {
        fill(&mut chunk);
        tx.send(&chunk)?;
        if !stalled && t0.elapsed() >= Duration::from_secs(1) {
            line(
                "stall-from",
                t0.elapsed().as_secs_f64(),
                tx.written() * 2,
                &tx.m0_state()?,
            );
            std::thread::sleep(Duration::from_millis(stall_ms));
            line(
                "stall-to",
                t0.elapsed().as_secs_f64(),
                tx.written() * 2,
                &tx.m0_state()?,
            );
            stalled = true;
        }
        if t0.elapsed() >= next_poll {
            line(
                "poll",
                t0.elapsed().as_secs_f64(),
                tx.written() * 2,
                &tx.m0_state()?,
            );
            next_poll += Duration::from_millis(250);
        }
    }
    let written = tx.written();
    let t_finish = Instant::now();
    let f = tx.finish()?;
    println!(
        "{{\"tag\":\"report\",\"rate\":{rate},\"written_samples\":{written},\"finish_ms\":{:.1},\"report\":\"{:?}\"}}",
        t_finish.elapsed().as_secs_f64() * 1e3,
        f.report
    );
    line(
        "after-finish",
        t0.elapsed().as_secs_f64(),
        written * 2,
        &f.hackrf.m0_state()?,
    );
    Ok(())
}
