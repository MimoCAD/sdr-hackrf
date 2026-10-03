//! The radio receiving, as a stream of samples whose losses the firmware
//! counts.
//!
//! - **Reading.** [`Stream::read`] hands out the wire bytes as the radio
//!   sends them (signed 8-bit I then Q, the layout `hackrf_transfer`
//!   writes); [`Stream::recv_into`] the same samples widened to [`Iq`].
//!   (Natively, `blocking::Stream` is a [`std::io::Read`].)
//! - **Changing.** The HackRF has no pause: a change ([`Stream::retune`],
//!   [`Stream::set_gain`], …) is made while samples flow, and placed in
//!   the stream by the firmware's own count of the bytes it has produced
//!   (`M0State::m0_count`, the host stream's byte index, read just before
//!   and just after the request). The result is a [`Break`]: every sample
//!   before `at_sample` is the old configuration, every one from
//!   `settled_at` the new. Reading and changing do not wait for each
//!   other.
//! - **Two at once.** [`Stream::split`] gives a [`ReadHalf`] (reading) and
//!   a [`Control`] (changing), for two threads natively or two tasks in a
//!   browser; [`Stream::reunite`] joins them.
//! - **Finishing.** [`Stream::finish`] turns the transceiver off, hands the
//!   last samples out ([`Finished::tail`]) and reads the firmware's
//!   counters: [`StreamReport::shortfalls`] is every time the radio found
//!   its buffer full and dropped samples.

use core::fmt;
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use std::boxed::Box;
use std::string::{String, ToString};
use std::sync::{Arc, Mutex, MutexGuard};
use std::vec::Vec;

use nusb::transfer::{Bulk, In};

use sdr_core::gain::StageReading;
use sdr_core::sync::Lock;
use sdr_core::time::Instant;
use sdr_core::usb_stream::{BulkIqStream, WireSample};
use sdr_core::{Error, Iq, RecvInfo, Result, TuneRequest, TuneResult};

use crate::proto::{EP_BULK_IN, M0_MODE_RX, MODE_OFF, MODE_RECEIVE, M0State, unwrap_count};
use crate::{BACKEND, HackRf, Stage};

/// The wire format: signed 8-bit I then Q.
pub(crate) const WIRE_FORMAT: WireSample = WireSample::I8;
/// The transfer size when 0 is given: at 20 Msps ~1.6 ms.
pub const DEFAULT_TRANSFER_BYTES: usize = 64 * 1024;
/// The transfers kept in flight when 0 is given: ~105 ms at 10 Msps.
pub const DEFAULT_QUEUE_DEPTH: usize = 32;
/// How long one `read` waits per round before trying again.
const READ_ROUND_S: f64 = 1.0;
/// After the transceiver is turned off at the finish, before the queue is
/// taken: the last bytes onto the wire.
const FINISH_DRAIN: Duration = Duration::from_millis(5);

/// One change made while samples flowed, placed by the firmware's byte
/// count.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Break {
    /// Every sample before `at_sample` (from 0) was produced before the
    /// change was asked for: the old configuration.
    pub at_sample: u64,
    /// The firmware had answered the change by sample `settled_at`: from
    /// here on, the new configuration. After a retune the synthesizer was
    /// within 500 Hz no later than ~0.1 ms past `settled_at` (40 retunes at
    /// 2, 4 and 8 Msps on two computers, a tone into the antenna port; the
    /// new tuning appeared between `at_sample` and `settled_at` in every
    /// one).
    pub settled_at: u64,
    /// The request's round trip, host clock.
    pub took: Duration,
    /// The RF the samples after the break are centred on.
    pub rf_hz: f64,
}

/// What a finished stream did.
#[derive(Clone, Debug, PartialEq)]
pub struct StreamReport {
    /// Samples handed out, the tail included.
    pub samples: u64,
    /// Times the radio found its buffer full and dropped samples
    /// (`M0State::num_shortfalls`): 0 is a run that lost nothing.
    pub shortfalls: u32,
    /// The longest of them, samples. The firmware keeps the longest, not
    /// the sum.
    pub longest_shortfall: u64,
    /// Samples the radio produced after the last one handed out: natively
    /// less than one 16 KiB piece, never sent (the M4 core sends whole
    /// pieces, `proto::M4_PIECE_BYTES`); in a browser, which cannot cancel
    /// a queued transfer, also what the last partly filled transfer held
    /// (47,840 bytes measured with 64 KiB transfers).
    pub left_in_device: u64,
    pub breaks: Vec<Break>,
}

/// A finished stream: the radio back, the report, and the samples still on
/// their way when it finished.
pub struct Finished {
    pub hackrf: HackRf,
    pub report: StreamReport,
    /// The last wire bytes (signed 8-bit I then Q), after everything a read
    /// handed out.
    pub tail: Vec<u8>,
}

impl Finished {
    /// The tail widened as [`Stream::recv_into`] widens.
    pub fn tail_samples(&self) -> Vec<Iq> {
        widen(&self.tail)
    }
}

/// Wire bytes widened as the engine widens them.
pub(crate) fn widen(bytes: &[u8]) -> Vec<Iq> {
    let (pairs, _) = bytes.as_chunks::<2>();
    pairs.iter().map(|&[i, q]| WIRE_FORMAT.widen(i, q)).collect()
}

/// A stream could not be made; the radio is handed back.
pub struct StreamError {
    pub hackrf: Box<HackRf>,
    pub error: Error,
}

impl fmt::Debug for StreamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("StreamError").field("error", &self.error).finish_non_exhaustive()
    }
}

impl fmt::Display for StreamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.error.fmt(f)
    }
}

impl core::error::Error for StreamError {}

/// An error re-labelled with the step it failed at.
pub(crate) fn at(step: &'static str) -> impl Fn(Error) -> Error {
    move |e| Error::Device { backend: BACKEND, context: step, message: e.to_string() }
}

pub(crate) fn finished_error() -> Error {
    Error::Device { backend: BACKEND, context: "stream", message: "the stream has finished".into() }
}

/// What the stream knows, readable by every handle at once.
#[derive(Clone, Debug)]
pub(crate) struct Snapshot {
    pub rate: f64,
    pub freq: f64,
    pub gain: f64,
    pub stages: Vec<StageReading<Stage>>,
    pub txvga: u16,
    pub amp: bool,
    pub label: String,
    pub breaks: Vec<Break>,
    /// Bytes handed out (received) or written (transmitted).
    pub bytes: u64,
    pub max_num_samps: usize,
    pub finished: bool,
}

impl Snapshot {
    pub fn of(h: &HackRf, max_num_samps: usize) -> Snapshot {
        Snapshot {
            rate: h.rate(),
            freq: h.freq(),
            gain: h.gain(),
            stages: h.stage_readings(),
            txvga: h.txvga_gain(),
            amp: h.amp(),
            label: h.label().to_string(),
            breaks: Vec::new(),
            bytes: 0,
            max_num_samps,
            finished: false,
        }
    }

    /// The radio's settings after a change.
    pub fn settings(&mut self, h: &HackRf) {
        self.freq = h.freq();
        self.gain = h.gain();
        self.stages = h.stage_readings();
        self.txvga = h.txvga_gain();
        self.amp = h.amp();
    }
}

/// The radio and the placement of a change, shared by both directions:
/// `f` runs between two readings of the firmware's byte count. `bytes` is
/// the host's count, at most 2³¹ behind the firmware's (its 32-bit count
/// is unwrapped against it); `mode` the M0's active mode while the stream
/// runs (before it has switched, the count is still the last run's).
pub(crate) async fn placed_change<T>(h: &mut HackRf, started: bool, bytes: u64, mode: u32, f: impl AsyncFnOnce(&mut HackRf) -> Result<T>) -> Result<(T, Break)> {
    let place = |m: &M0State| if started && m.active_mode == mode { unwrap_count(m.m0_count, bytes) / 2 } else { 0 };
    let before = if started { Some(h.m0_state().await.map_err(at("change: the count before"))?) } else { None };
    let t0 = Instant::now();
    let t = f(h).await?;
    let took = t0.elapsed();
    let after = if started { Some(h.m0_state().await.map_err(at("change: the count after"))?) } else { None };
    let at_sample = before.as_ref().map_or(0, place);
    let settled_at = after.as_ref().map_or(0, place).max(at_sample);
    Ok((t, Break { at_sample, settled_at, took, rf_hz: h.freq() }))
}

/// What a stream's handles share. Reading takes `rx`, changing takes
/// `radio`: neither waits for the other.
struct Shared {
    radio: Lock<Option<HackRf>>,
    rx: Lock<Option<BulkIqStream>>,
    snapshot: Mutex<Snapshot>,
    started: AtomicBool,
}

impl Shared {
    fn snap(&self) -> MutexGuard<'_, Snapshot> {
        self.snapshot.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The queue primed first, then the transceiver into receive: submitted
    /// transfers wait harmlessly, so no samples are produced into a buffer
    /// nobody drains (a ~23 kB shortfall at every start when the mode was
    /// set first).
    async fn start(&self, rx: &mut BulkIqStream) -> Result<()> {
        if self.started.load(Ordering::Acquire) {
            return Ok(());
        }
        rx.start().map_err(at("start: the queue"))?;
        let g = self.radio.lock().await;
        g.as_ref().ok_or_else(finished_error)?.set_transceiver_mode(MODE_RECEIVE).await.map_err(at("start: receive"))?;
        self.started.store(true, Ordering::Release);
        Ok(())
    }

    async fn recv_into(&self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
        let mut g = self.rx.lock().await;
        let rx = g.as_mut().ok_or_else(finished_error)?;
        self.start(rx).await?;
        let info = rx.recv_into_async(buf, timeout_secs).await?;
        self.snap().bytes = rx.delivered_bytes();
        Ok(info)
    }

    /// At least one byte, or 0 once the stream has finished.
    async fn read(&self, buf: &mut [u8]) -> Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        loop {
            let mut g = self.rx.lock().await;
            let Some(rx) = g.as_mut() else {
                return Ok(0);
            };
            self.start(rx).await?;
            let (n, _) = rx.recv_bytes_async(buf, READ_ROUND_S).await?;
            self.snap().bytes = rx.delivered_bytes();
            if n > 0 {
                return Ok(n);
            }
        }
    }

    async fn change<T>(&self, f: impl AsyncFnOnce(&mut HackRf) -> Result<T>) -> Result<(T, Break)> {
        let mut g = self.radio.lock().await;
        let h = g.as_mut().ok_or_else(finished_error)?;
        let bytes = self.snap().bytes;
        let r = placed_change(h, self.started.load(Ordering::Acquire), bytes, M0_MODE_RX, f).await;
        let mut s = self.snap();
        s.settings(h);
        if let Ok((_, b)) = &r {
            s.breaks.push(*b);
        }
        r
    }
}

/// The radio receiving: see the module documentation.
pub struct Stream {
    shared: Arc<Shared>,
}

/// The reading half of a [`Stream`] ([`Stream::split`]).
pub struct ReadHalf {
    shared: Arc<Shared>,
}

/// The changing half of a [`Stream`] ([`Stream::split`]). Cheap to clone;
/// every clone is the same stream's.
#[derive(Clone)]
pub struct Control {
    shared: Arc<Shared>,
}

/// What every handle reports, at once (no request, no wait).
macro_rules! reports {
    ($t:ident) => {
        impl $t {
            /// The rate (fixed for the stream's life).
            pub fn rate(&self) -> f64 {
                self.shared.snap().rate
            }

            /// The RF as last set.
            pub fn freq(&self) -> f64 {
                self.shared.snap().freq
            }

            /// The receive gain as last set ([`HackRf::gain`]).
            pub fn gain(&self) -> f64 {
                self.shared.snap().gain
            }

            /// Each receive stage as last set.
            pub fn stage_readings(&self) -> Vec<StageReading<Stage>> {
                self.shared.snap().stages.clone()
            }

            /// The radio's label (product and serial).
            pub fn label(&self) -> String {
                self.shared.snap().label.clone()
            }

            /// Samples handed out so far.
            pub fn delivered(&self) -> u64 {
                self.shared.snap().bytes / 2
            }

            /// The breaks made so far.
            pub fn breaks(&self) -> Vec<Break> {
                self.shared.snap().breaks.clone()
            }

            /// Samples one receive hands out at most.
            pub fn max_num_samps(&self) -> usize {
                self.shared.snap().max_num_samps
            }
        }
    };
}

/// What changes the radio while it receives.
macro_rules! changes {
    ($t:ident) => {
        impl $t {
            /// Run `f` on the radio while samples flow; returns what `f`
            /// returned and the [`Break`] placing it. Before the stream has
            /// started, the break is at sample 0.
            pub async fn reconfigure<T>(&self, f: impl AsyncFnOnce(&mut HackRf) -> Result<T>) -> Result<(T, Break)> {
                self.shared.change(f).await
            }

            /// [`HackRf::set_freq`] while receiving.
            pub async fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
                self.reconfigure(async |h| h.set_freq(req).await).await
            }

            /// [`HackRf::set_gain`] while receiving.
            pub async fn set_gain(&self, db: f64) -> Result<Break> {
                self.reconfigure(async |h| h.set_gain(db).await).await.map(|(_, b)| b)
            }

            /// [`HackRf::set_stage_level`] while receiving.
            pub async fn set_stage_level(&self, stage: Stage, level: usize) -> Result<Break> {
                self.reconfigure(async |h| h.set_stage_level(stage, level).await).await.map(|(_, b)| b)
            }

            /// [`HackRf::set_bandwidth`] while receiving.
            pub async fn set_bandwidth(&self, hz: f64) -> Result<Break> {
                self.reconfigure(async |h| h.set_bandwidth(hz).await).await.map(|(_, b)| b)
            }

            /// The firmware's counters now ([`HackRf::m0_state`]): reading
            /// them while samples flow cost nothing at 10 Msps polled every
            /// 10 ms.
            pub async fn m0_state(&self) -> Result<M0State> {
                self.shared.radio.lock().await.as_ref().ok_or_else(finished_error)?.m0_state().await
            }
        }
    };
}

/// What reads the samples.
macro_rules! reads {
    ($t:ident) => {
        impl $t {
            /// The wire bytes into `buf`: signed 8-bit I then Q, as the
            /// radio sends them. Waits for at least one byte; 0 once the
            /// stream has finished. Starts the stream on the first read.
            pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
                self.shared.read(buf).await
            }

            /// Samples into `buf`, widened to [`Iq`] (full scale ±32768);
            /// at most `timeout_secs` waiting (`timed_out` then set).
            /// Starts the stream on the first call.
            pub async fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
                self.shared.recv_into(buf, timeout_secs).await
            }

            /// Start the stream now rather than at the first read.
            pub async fn start(&mut self) -> Result<()> {
                let mut g = self.shared.rx.lock().await;
                let rx = g.as_mut().ok_or_else(finished_error)?;
                self.shared.start(rx).await
            }
        }
    };
}

reports!(Stream);
reports!(ReadHalf);
reports!(Control);
changes!(Stream);
changes!(Control);
reads!(Stream);
reads!(ReadHalf);

impl Stream {
    pub(crate) fn new(h: HackRf, transfer_bytes: usize, queue_depth: usize) -> core::result::Result<Stream, StreamError> {
        if h.rate() <= 0.0 {
            let error = Error::Device { backend: BACKEND, context: "stream", message: "set the sample rate before streaming".into() };
            return Err(StreamError { hackrf: Box::new(h), error });
        }
        let endpoint = match h.interface().endpoint::<Bulk, In>(EP_BULK_IN) {
            Ok(e) => e,
            Err(e) => return Err(StreamError { hackrf: Box::new(h), error: crate::usb_err("claim bulk endpoint", e) }),
        };
        let transfer_bytes = if transfer_bytes > 0 { transfer_bytes } else { DEFAULT_TRANSFER_BYTES };
        let queue_depth = if queue_depth > 0 { queue_depth } else { DEFAULT_QUEUE_DEPTH };
        let rx = BulkIqStream::new(BACKEND, endpoint, WIRE_FORMAT, transfer_bytes, queue_depth, h.rate());
        let snapshot = Mutex::new(Snapshot::of(&h, rx.max_num_samps()));
        Ok(Stream { shared: Arc::new(Shared { radio: Lock::new(Some(h)), rx: Lock::new(Some(rx)), snapshot, started: AtomicBool::new(false) }) })
    }

    /// The two halves, each usable on its own thread or task.
    pub fn split(self) -> (ReadHalf, Control) {
        (ReadHalf { shared: Arc::clone(&self.shared) }, Control { shared: self.shared })
    }

    /// The halves of one stream joined again; handed back if they are not
    /// the same stream's, or a clone of the control still exists.
    pub fn reunite(read: ReadHalf, control: Control) -> core::result::Result<Stream, (ReadHalf, Control)> {
        if !Arc::ptr_eq(&read.shared, &control.shared) || Arc::strong_count(&read.shared) != 2 {
            return Err((read, control));
        }
        drop(read);
        Ok(Stream { shared: control.shared })
    }

    /// Turn the transceiver off, hand out the samples still on their way,
    /// and read the firmware's counters: the radio back, the report, the
    /// tail.
    pub async fn finish(self) -> Result<Finished> {
        let mut rx = self.shared.rx.lock().await.take().ok_or_else(finished_error)?;
        let h = self.shared.radio.lock().await.take().ok_or_else(finished_error)?;
        self.shared.snap().finished = true;
        let mut tail = Vec::new();
        let mut m0 = None;
        if self.shared.started.load(Ordering::Acquire) {
            h.set_transceiver_mode(MODE_OFF).await.map_err(at("finish: off"))?;
            sdr_core::time::sleep(FINISH_DRAIN).await;
            rx.hold_async().await.map_err(at("finish: the queue"))?;
            let mut buf = vec![0u8; 2 * rx.max_num_samps()];
            loop {
                let (n, _) = rx.recv_bytes_async(&mut buf, 0.0).await.map_err(at("finish: the tail"))?;
                if n == 0 {
                    break;
                }
                tail.extend_from_slice(&buf[..n]);
            }
            // Read with the transceiver off: the counts stay until the
            // next mode starts.
            m0 = Some(h.m0_state().await.map_err(at("finish: the counters"))?);
        }
        let host_bytes = rx.delivered_bytes();
        rx.stop()?;
        drop(rx);
        let m0 = m0.unwrap_or_default();
        let produced = unwrap_count(m0.m0_count, host_bytes);
        let report = StreamReport {
            samples: host_bytes / 2,
            shortfalls: m0.num_shortfalls,
            longest_shortfall: m0.longest_shortfall as u64 / 2,
            left_in_device: produced.saturating_sub(host_bytes) / 2,
            breaks: self.shared.snap().breaks.clone(),
        };
        Ok(Finished { hackrf: h, report, tail })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_bytes_widen_signed() {
        assert_eq!(widen(&[0, 0, 127, 128, 7]), vec![Iq { i: 0, q: 0 }, Iq { i: 127 << 8, q: -128 << 8 }]);
    }
}
