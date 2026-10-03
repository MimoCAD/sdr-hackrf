//! sdr-hackrf in a browser over WebUSB: the proof page's two actions.
//!
//! `receive_check`: ask for a HackRF, stream for four seconds — the reading
//! half on this task reading the wire bytes, the control on another task
//! stepping the gain three times — then reunite and finish; the report is
//! the firmware's own count of shortfalls.
//!
//! `transmit_check`: a tone written for two seconds at the lowest transmit
//! gain, the amp off, then finish; the report is the firmware's count of
//! underruns and whether every sample was sent. Only with `cabled` set: the
//! operator's statement that the antenna port is cabled to a receiver
//! through attenuation and nothing radiates.
//!
//! Both log each step through the page's `log(text)` and resolve to a
//! one-line JSON summary. Opened as `?auto=receive` (or `?auto=transmit`)
//! the page runs one by itself — the HackRF must already be granted, since
//! only a click may open the chooser — taking `rate`, `transfer`, `depth`,
//! `seconds`, `changes` (and for transmit `cabled=1`) from the query, and
//! sends each line to the server (`POST /log`).

use std::cell::RefCell;
use std::f64::consts::TAU;
use std::rc::Rc;
use std::time::Duration;

use js_sys::Function;
use sdr_core::time::{Instant, sleep};
use sdr_core::{Iq, TuneRequest};
use sdr_hackrf::{Control, HackRf, Stream};
use wasm_bindgen::prelude::*;

const DEFAULT_RATE_SPS: f64 = 8e6;
const RX_HZ: f64 = 300e6;
const READ_FOR: Duration = Duration::from_secs(4);
const READ_BYTES: usize = 65536;
/// The gain steps the control alternates between, dB.
const GAIN_LOW_DB: f64 = 0.0;
const GAIN_HIGH_DB: f64 = 70.0;
/// The transmission: a tone this far above the carrier, this far below
/// full scale, for this long.
const TX_CARRIER_HZ: f64 = 299.9e6;
const TONE_OFFSET_HZ: f64 = 400e3;
const BACKOFF_DB: f64 = -15.0;
const TX_FOR: Duration = Duration::from_secs(2);

fn say(log: &Function, text: &str) {
    let _ = log.call1(&JsValue::NULL, &JsValue::from_str(text));
}

fn js(e: impl ToString) -> JsValue {
    JsValue::from_str(&e.to_string())
}

/// The HackRF the page was granted, or the chooser (needs the click that
/// called us).
async fn radio(log: &Function) -> Result<HackRf, JsValue> {
    let h = match HackRf::open("").await {
        Ok(h) => h,
        Err(_) => {
            say(log, "asking for a HackRF (WebUSB chooser)…");
            HackRf::request().await.map_err(js)?
        }
    };
    say(log, &format!("opened: {} · firmware {}", h.label(), h.version_string().await.map_err(js)?));
    Ok(h)
}

/// `rate` 0 for the page's 8 Msps; `transfer_bytes` and `depth` 0 for the
/// driver's defaults; `seconds` 0 for 4 s; `changes` evenly spaced gain
/// steps (0: none).
#[wasm_bindgen]
pub async fn receive_check(log: Function, rate: f64, transfer_bytes: usize, depth: usize, seconds: f64, changes: u32) -> Result<JsValue, JsValue> {
    let rate = if rate > 0.0 { rate } else { DEFAULT_RATE_SPS };
    let read_for = if seconds > 0.0 { Duration::from_secs_f64(seconds) } else { READ_FOR };
    let mut h = radio(&log).await?;
    h.set_rate(rate).await.map_err(js)?;
    h.set_freq(&TuneRequest::auto(RX_HZ)).await.map_err(js)?;
    h.set_gain(GAIN_LOW_DB).await.map_err(js)?;
    let (mut reader, control) = h.into_stream(transfer_bytes, depth).await.map_err(|e| js(e.error))?.split();
    say(&log, &format!("receiving {:.1} Msps at {:.1} MHz for {:.1} s (transfers {transfer_bytes} B × {depth}, 0 = default); stepping the gain {changes} times from another task", rate / 1e6, RX_HZ / 1e6, read_for.as_secs_f64()));

    // The control on its own task, gain steps evenly spaced.
    let back: Rc<RefCell<Option<Result<Control, String>>>> = Rc::new(RefCell::new(None));
    let slot = Rc::clone(&back);
    let task_log = log.clone();
    wasm_bindgen_futures::spawn_local(async move {
        let mut r = Ok(());
        for k in 0..changes {
            sleep(read_for / (changes + 1)).await;
            let db = if k % 2 == 0 { GAIN_HIGH_DB } else { GAIN_LOW_DB };
            match control.set_gain(db).await {
                Ok(b) => say(&task_log, &format!("gain {k}: {db} dB between samples {} and {} ({} µs)", b.at_sample, b.settled_at, b.took.as_micros())),
                Err(e) => {
                    r = Err(e.to_string());
                    break;
                }
            }
        }
        *slot.borrow_mut() = Some(r.map(|()| control));
    });

    // The reading half here, timed.
    let t = Instant::now();
    let mut buf = vec![0u8; READ_BYTES];
    let mut bytes = 0u64;
    let (mut reads, mut longest_ms) = (0u64, 0.0f64);
    let mut per_second: Vec<u64> = Vec::new();
    while t.elapsed() < read_for {
        let r0 = Instant::now();
        let n = reader.read(&mut buf).await.map_err(js)? as u64;
        longest_ms = longest_ms.max(r0.elapsed().as_secs_f64() * 1e3);
        reads += 1;
        bytes += n;
        let s = t.elapsed().as_secs() as usize;
        if per_second.len() <= s {
            per_second.resize(s + 1, 0);
        }
        per_second[s] += n;
    }
    say(&log, &format!("read: {reads} reads, the longest {longest_ms:.1} ms; bytes per second {per_second:?} ({:.1} Msps = {})", rate / 1e6, 2.0 * rate));
    let control = loop {
        if let Some(r) = back.borrow_mut().take() {
            break r.map_err(js)?;
        }
        sleep(Duration::from_millis(10)).await;
    };
    let seconds = t.elapsed().as_secs_f64();
    let stream = Stream::reunite(reader, control).map_err(|_| js("the halves did not reunite"))?;
    let fin = stream.finish().await.map_err(js)?;
    let r = &fin.report;
    let summary = format!(
        "{{\"rate\":{rate},\"transfer_bytes\":{transfer_bytes},\"depth\":{depth},\"seconds\":{seconds:.3},\"bytes_read\":{bytes},\"reads\":{reads},\"longest_read_ms\":{longest_ms:.1},\"bytes_per_second\":{per_second:?},\"samples\":{},\"tail_bytes\":{},\"shortfalls\":{},\"longest_shortfall\":{},\"left_in_device\":{},\"breaks\":{}}}",
        r.samples,
        fin.tail.len(),
        r.shortfalls,
        r.longest_shortfall,
        r.left_in_device,
        r.breaks.len(),
    );
    say(&log, &format!("finished: {} samples, {} shortfall(s) counted by the radio", r.samples, r.shortfalls));
    Ok(JsValue::from_str(&summary))
}

/// `rate` 0 for the page's 8 Msps; `seconds` 0 for 2 s. Refused unless
/// `cabled`.
#[wasm_bindgen]
pub async fn transmit_check(log: Function, cabled: bool, rate: f64, seconds: f64) -> Result<JsValue, JsValue> {
    if !cabled {
        return Err(js("refusing to key: confirm that the antenna port is cabled to a receiver through attenuation and nothing radiates"));
    }
    let rate = if rate > 0.0 { rate } else { DEFAULT_RATE_SPS };
    let send_for = if seconds > 0.0 { Duration::from_secs_f64(seconds) } else { TX_FOR };
    let mut h = radio(&log).await?;
    h.set_rate(rate).await.map_err(js)?;
    h.set_freq(&TuneRequest::auto(TX_CARRIER_HZ)).await.map_err(js)?;
    h.set_amp(false).await.map_err(js)?;
    h.set_txvga_gain(0).await.map_err(js)?;
    let mut tx = h.into_tx_stream(0, 0).await.map_err(|e| js(e.error))?;
    say(&log, &format!("transmitting a tone at {:.4} MHz, {BACKOFF_DB} dB below full scale, transmit gain 0 dB, amp off, {:.1} Msps for {:.1} s", (TX_CARRIER_HZ + TONE_OFFSET_HZ) / 1e6, rate / 1e6, send_for.as_secs_f64()));
    let amplitude = 32767.0 * 10f64.powf(BACKOFF_DB / 20.0);
    let step = TAU * TONE_OFFSET_HZ / rate;
    let mut phase = 0.0f64;
    let mut chunk = vec![Iq::default(); tx.samples_per_transfer()];
    let t = Instant::now();
    let mut longest_ms = 0.0f64;
    while t.elapsed() < send_for {
        for s in chunk.iter_mut() {
            *s = Iq { i: (amplitude * phase.cos()) as i16, q: (amplitude * phase.sin()) as i16 };
            phase = (phase + step) % TAU;
        }
        let w0 = Instant::now();
        tx.send(&chunk).await.map_err(js)?;
        longest_ms = longest_ms.max(w0.elapsed().as_secs_f64() * 1e3);
    }
    let written = tx.written();
    let f0 = Instant::now();
    let fin = tx.finish().await.map_err(js)?;
    let r = &fin.report;
    let summary = format!(
        "{{\"rate\":{rate},\"seconds\":{:.3},\"written_samples\":{written},\"longest_write_ms\":{longest_ms:.1},\"finish_ms\":{:.1},\"samples\":{},\"underruns\":{},\"longest_underrun\":{},\"sent_all\":{}}}",
        t.elapsed().as_secs_f64(),
        f0.elapsed().as_secs_f64() * 1e3,
        r.samples,
        r.underruns,
        r.longest_underrun,
        r.sent_all,
    );
    say(&log, &format!("finished: {} samples sent, {} underrun(s), every sample sent: {}", r.samples, r.underruns, r.sent_all));
    Ok(JsValue::from_str(&summary))
}
