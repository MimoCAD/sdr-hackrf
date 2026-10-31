//! The M0 core's counters while receiving: whether a mode start zeroes
//! them, whether reading them while samples flow costs samples, and how
//! the device's byte count tracks the bytes the host receives. One JSON
//! line per reading.
//!
//!   cargo run --release -p sdr-hackrf --example m0probe -- [rate_sps] [seconds] [poll_ms] [stall_ms] [depth]
//!
//! `stall_ms` stops reading for that long 1 s into each run (a loss made
//! on purpose, with `depth` transfers in flight), to see how the counters
//! record it.

use std::time::{Duration, Instant};

use sdr_core::{Iq, TuneRequest};
use sdr_hackrf::M0State;
use sdr_hackrf::blocking::HackRf;

fn line(tag: &str, t: f64, host_bytes: u64, m: &M0State) {
    println!(
        "{{\"tag\":\"{tag}\",\"t\":{t:.4},\"host_bytes\":{host_bytes},\"m0_count\":{},\"m4_count\":{},\"shortfalls\":{},\"longest\":{},\"active_mode\":{},\"error\":{}}}",
        m.m0_count, m.m4_count, m.num_shortfalls, m.longest_shortfall, m.active_mode, m.error
    );
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let rate: f64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(10e6);
    let seconds: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(3.0);
    let poll_ms: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(100);
    let stall_ms: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0);
    let depth: usize = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(0);

    let mut h = HackRf::open("")?;
    h.set_rate(rate)?;
    h.set_freq(&TuneRequest::auto(300e6))?;
    h.set_gain(16.0)?;
    line("open", 0.0, 0, &h.m0_state()?);

    for run in 0..2 {
        let mut s = h.into_stream(0, depth).map_err(|e| e.error)?;
        let mut buf = vec![Iq::default(); s.max_num_samps()];
        let mut stalled = stall_ms == 0;
        s.start()?;
        let t0 = Instant::now();
        line(&format!("run{run}-start"), 0.0, 0, &s.m0_state()?);
        let mut host_bytes = 0u64;
        let mut next_poll = Duration::from_millis(poll_ms);
        while t0.elapsed().as_secs_f64() < seconds {
            let info = s.recv_into(&mut buf, 0.5)?;
            host_bytes += 2 * info.n_samples as u64;
            if !stalled && t0.elapsed() >= Duration::from_secs(1) {
                line(
                    &format!("run{run}-stall-from"),
                    t0.elapsed().as_secs_f64(),
                    host_bytes,
                    &s.m0_state()?,
                );
                std::thread::sleep(Duration::from_millis(stall_ms));
                line(
                    &format!("run{run}-stall-to"),
                    t0.elapsed().as_secs_f64(),
                    host_bytes,
                    &s.m0_state()?,
                );
                stalled = true;
            }
            if poll_ms > 0 && t0.elapsed() >= next_poll {
                line(
                    &format!("run{run}-poll"),
                    t0.elapsed().as_secs_f64(),
                    host_bytes,
                    &s.m0_state()?,
                );
                next_poll += Duration::from_millis(poll_ms);
            }
        }
        line(
            &format!("run{run}-before-finish"),
            t0.elapsed().as_secs_f64(),
            host_bytes,
            &s.m0_state()?,
        );
        let f = s.finish()?;
        println!(
            "{{\"tag\":\"run{run}-report\",\"host_bytes\":{host_bytes},\"tail_bytes\":{},\"report\":\"{:?}\"}}",
            f.tail.len(),
            f.report
        );
        h = f.hackrf;
    }
    Ok(())
}
