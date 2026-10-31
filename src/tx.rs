//! The radio transmitting, as a stream of samples written to it.
//!
//! - **Writing.** [`TxStream::write`] takes wire bytes (signed 8-bit I then
//!   Q, the layout `hackrf_transfer` reads); [`TxStream::send`] takes
//!   [`Iq`] samples and narrows them (the top byte). (Natively,
//!   `blocking::TxStream` is a [`std::io::Write`].) Bytes are staged into
//!   whole transfers; while the transfer queue has room a write returns at
//!   once, and once it is full a write waits for a transfer to complete,
//!   which paces the writer at the sample rate.
//! - **Keying.** The transmitter keys up when the queue first fills (or at
//!   [`TxStream::start`]): every transfer submitted before then waits for
//!   the firmware, so key-up begins with samples already queued.
//! - **Changing.** [`TxStream::retune`], [`TxStream::set_txvga_gain`], …
//!   while transmitting, placed by the firmware's byte count as a receive
//!   change is ([`Break`]). [`TxStream::split`] gives a [`WriteHalf`] and a
//!   [`TxControl`].
//! - **Finishing.** [`TxStream::finish`] pads the last transfer with zeros,
//!   waits until the radio has sent every byte written, unkeys and reads
//!   the firmware's counters: [`TxReport::underruns`] is every time the
//!   radio found its buffer empty while samples were still to come.
//!
//! A transmission dropped without `finish` is unkeyed when its radio goes
//! (natively; the radio's own `Drop`).

use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use std::boxed::Box;
use std::string::{String, ToString};
use std::sync::{Arc, Mutex, MutexGuard};
use std::vec::Vec;

use nusb::transfer::{Bulk, Completion, Out};

use sdr_core::sync::Lock;
use sdr_core::time::Instant;
use sdr_core::{Error, Iq, Result, TuneRequest, TuneResult};

use crate::proto::{EP_BULK_OUT, M0State, MODE_OFF, MODE_TRANSMIT, unwrap_count};
use crate::session::{Break, Snapshot, StreamError, at, finished_error, placed_change};
use crate::{BACKEND, HackRf};

/// The transfer size when 0 is given.
pub const DEFAULT_TRANSFER_BYTES: usize = 32 * 1024;
/// The transfers kept in flight when 0 is given.
pub const DEFAULT_QUEUE_DEPTH: usize = 8;
/// The longest a submitted transfer may take to complete while keyed.
const SEND_TIMEOUT: Duration = Duration::from_secs(2);
/// `active_mode` while transmitting: TX_START, then TX_RUN.
const M0_MODE_TX_START: u32 = 3;
const M0_MODE_TX_RUN: u32 = 4;
/// How often the finish reads the firmware's count while the radio sends
/// what is left.
const FINISH_POLL: Duration = Duration::from_millis(1);
/// Beyond the time the bytes left take at the rate, how long the finish
/// waits for the radio to send them.
const FINISH_SLACK: Duration = Duration::from_millis(200);

/// What a finished transmission did.
#[derive(Clone, Debug, PartialEq)]
pub struct TxReport {
    /// Samples written, the finish's zero padding included.
    pub samples: u64,
    /// Times the radio found its buffer empty while samples were still to
    /// come (`M0State::num_shortfalls`, read when the last transfer had
    /// reached it): 0 is a transmission that never starved.
    pub underruns: u32,
    /// The longest of them, samples.
    pub longest_underrun: u64,
    /// The radio had sent every sample written before it was unkeyed.
    pub sent_all: bool,
    pub breaks: Vec<Break>,
}

/// A finished transmission: the radio back and the report.
pub struct TxFinished {
    pub hackrf: HackRf,
    pub report: TxReport,
}

/// The next completion within `timeout`, `None` if none came.
#[cfg(not(target_arch = "wasm32"))]
async fn next_completion(
    endpoint: &mut nusb::Endpoint<Bulk, Out>,
    timeout: Duration,
) -> Option<Completion> {
    endpoint.wait_next_complete(timeout)
}

/// The next completion within `timeout`, `None` if none came.
#[cfg(target_arch = "wasm32")]
async fn next_completion(
    endpoint: &mut nusb::Endpoint<Bulk, Out>,
    timeout: Duration,
) -> Option<Completion> {
    use core::future::{Future, poll_fn};
    use core::task::Poll;
    let mut sleep = core::pin::pin!(sdr_core::time::sleep(timeout));
    poll_fn(|cx| {
        if let Poll::Ready(c) = endpoint.poll_next_complete(cx) {
            return Poll::Ready(Some(c));
        }
        sleep.as_mut().poll(cx).map(|()| None)
    })
    .await
}

/// The OUT queue.
struct TxCore {
    endpoint: nusb::Endpoint<Bulk, Out>,
    transfer_bytes: usize,
    queue_depth: usize,
    /// Wire bytes not yet a whole transfer.
    stage: Vec<u8>,
    /// Bytes submitted.
    submitted: u64,
}

impl TxCore {
    fn submit_stage(&mut self) {
        let mut buf = self.endpoint.allocate(self.transfer_bytes);
        buf.extend_from_slice(&self.stage[..self.transfer_bytes]);
        self.stage.drain(..self.transfer_bytes);
        self.endpoint.submit(buf);
        self.submitted += self.transfer_bytes as u64;
    }

    /// One transfer completed, `Ok(false)` if none did within `timeout`.
    async fn reap(&mut self, timeout: Duration) -> Result<bool> {
        let Some(c) = next_completion(&mut self.endpoint, timeout).await else {
            return Ok(false);
        };
        c.status.map_err(|e| Error::Usb {
            backend: BACKEND,
            context: "bulk send",
            message: e.to_string(),
        })?;
        Ok(true)
    }
}

/// What a transmission's handles share. Writing takes `tx`, changing takes
/// `radio`.
struct Shared {
    radio: Lock<Option<HackRf>>,
    tx: Lock<Option<TxCore>>,
    snapshot: Mutex<Snapshot>,
    started: AtomicBool,
}

impl Shared {
    fn snap(&self) -> MutexGuard<'_, Snapshot> {
        self.snapshot.lock().unwrap_or_else(|p| p.into_inner())
    }

    async fn start(&self) -> Result<()> {
        if self.started.load(Ordering::Acquire) {
            return Ok(());
        }
        let g = self.radio.lock().await;
        g.as_ref()
            .ok_or_else(finished_error)?
            .set_transceiver_mode(MODE_TRANSMIT)
            .await
            .map_err(at("start: transmit"))?;
        self.started.store(true, Ordering::Release);
        Ok(())
    }

    /// Room for one more transfer: keyed up first if the queue is full and
    /// the radio is not yet sending.
    async fn room(&self, core: &mut TxCore) -> Result<()> {
        while core.endpoint.pending() >= core.queue_depth {
            if !self.started.load(Ordering::Acquire) {
                self.start().await?;
            }
            if !core.reap(SEND_TIMEOUT).await? {
                return Err(Error::Device {
                    backend: BACKEND,
                    context: "bulk send",
                    message: "transfer completion timed out".into(),
                });
            }
        }
        Ok(())
    }

    async fn write(&self, buf: &[u8]) -> Result<usize> {
        let mut g = self.tx.lock().await;
        let core = g.as_mut().ok_or_else(finished_error)?;
        core.stage.extend_from_slice(buf);
        while core.stage.len() >= core.transfer_bytes {
            self.room(core).await?;
            core.submit_stage();
        }
        self.snap().bytes += buf.len() as u64;
        Ok(buf.len())
    }

    /// The stage padded with zeros to a whole transfer and submitted.
    async fn flush(&self) -> Result<()> {
        let mut g = self.tx.lock().await;
        let core = g.as_mut().ok_or_else(finished_error)?;
        if !core.stage.is_empty() {
            let pad = core.transfer_bytes - core.stage.len();
            core.stage.resize(core.transfer_bytes, 0);
            self.room(core).await?;
            core.submit_stage();
            self.snap().bytes += pad as u64;
        }
        Ok(())
    }

    async fn change<T>(&self, f: impl AsyncFnOnce(&mut HackRf) -> Result<T>) -> Result<(T, Break)> {
        let mut g = self.radio.lock().await;
        let h = g.as_mut().ok_or_else(finished_error)?;
        let bytes = self.snap().bytes;
        let started = self.started.load(Ordering::Acquire);
        // Keyed, the firmware first reports TX_START, then TX_RUN.
        let mode = match h.m0_state().await {
            Ok(m) if m.active_mode == M0_MODE_TX_START => M0_MODE_TX_START,
            _ => M0_MODE_TX_RUN,
        };
        let r = placed_change(h, started, bytes, mode, f).await;
        let mut s = self.snap();
        s.settings(h);
        if let Ok((_, b)) = &r {
            s.breaks.push(*b);
        }
        r
    }
}

/// The radio transmitting: see the module documentation.
pub struct TxStream {
    shared: Arc<Shared>,
}

/// The writing half of a [`TxStream`] ([`TxStream::split`]).
pub struct WriteHalf {
    shared: Arc<Shared>,
}

/// The changing half of a [`TxStream`] ([`TxStream::split`]). Cheap to
/// clone.
#[derive(Clone)]
pub struct TxControl {
    shared: Arc<Shared>,
}

/// What every handle reports, at once.
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

            /// The transmit VGA gain in dB as last set.
            pub fn txvga_gain(&self) -> u16 {
                self.shared.snap().txvga
            }

            /// Whether the +14 dB amp is on, as last set.
            pub fn amp(&self) -> bool {
                self.shared.snap().amp
            }

            /// The radio's label (product and serial).
            pub fn label(&self) -> String {
                self.shared.snap().label.clone()
            }

            /// Samples written so far.
            pub fn written(&self) -> u64 {
                self.shared.snap().bytes / 2
            }

            /// Whether the transmitter is keyed.
            pub fn is_keyed(&self) -> bool {
                self.shared.started.load(Ordering::Acquire)
            }

            /// The breaks made so far.
            pub fn breaks(&self) -> Vec<Break> {
                self.shared.snap().breaks.clone()
            }
        }
    };
}

/// What changes the radio while it transmits.
macro_rules! changes {
    ($t:ident) => {
        impl $t {
            /// Run `f` on the radio while it transmits; returns what `f`
            /// returned and the [`Break`] placing it (in samples written).
            pub async fn reconfigure<T>(
                &self,
                f: impl AsyncFnOnce(&mut HackRf) -> Result<T>,
            ) -> Result<(T, Break)> {
                self.shared.change(f).await
            }

            /// [`HackRf::set_freq`] while transmitting.
            pub async fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
                self.reconfigure(async |h| h.set_freq(req).await).await
            }

            /// [`HackRf::set_txvga_gain`] while transmitting.
            pub async fn set_txvga_gain(&self, db: u16) -> Result<Break> {
                self.reconfigure(async |h| h.set_txvga_gain(db).await)
                    .await
                    .map(|(_, b)| b)
            }

            /// [`HackRf::set_amp`] while transmitting.
            pub async fn set_amp(&self, on: bool) -> Result<Break> {
                self.reconfigure(async |h| h.set_amp(on).await)
                    .await
                    .map(|(_, b)| b)
            }

            /// The firmware's counters now ([`HackRf::m0_state`]).
            pub async fn m0_state(&self) -> Result<M0State> {
                self.shared
                    .radio
                    .lock()
                    .await
                    .as_ref()
                    .ok_or_else(finished_error)?
                    .m0_state()
                    .await
            }
        }
    };
}

/// What writes the samples.
macro_rules! writes {
    ($t:ident) => {
        impl $t {
            /// Wire bytes to send: signed 8-bit I then Q. Takes all of
            /// `buf`; waits only while the transfer queue is full (keying
            /// up the first time it fills).
            pub async fn write(&mut self, buf: &[u8]) -> Result<usize> {
                self.shared.write(buf).await
            }

            /// Samples to send, narrowed to the wire's signed 8 bits (the
            /// top byte, as the receive side widens).
            pub async fn send(&mut self, iq: &[Iq]) -> Result<()> {
                let bytes: Vec<u8> = iq
                    .iter()
                    .flat_map(|s| [(s.i >> 8) as u8, (s.q >> 8) as u8])
                    .collect();
                self.shared.write(&bytes).await.map(|_| ())
            }

            /// What is staged, padded with zeros (silence) to a whole
            /// transfer and queued.
            pub async fn flush(&mut self) -> Result<()> {
                self.shared.flush().await
            }

            /// Key up now rather than when the queue first fills.
            pub async fn start(&mut self) -> Result<()> {
                self.shared.start().await
            }

            /// Samples one transfer carries: queue at least the queue's
            /// depth of them before keying up by hand for a clean key-up.
            pub fn samples_per_transfer(&self) -> usize {
                self.shared.snap().max_num_samps
            }
        }
    };
}

reports!(TxStream);
reports!(WriteHalf);
reports!(TxControl);
changes!(TxStream);
changes!(TxControl);
writes!(TxStream);
writes!(WriteHalf);

impl TxStream {
    pub(crate) fn new(
        h: HackRf,
        transfer_bytes: usize,
        queue_depth: usize,
    ) -> core::result::Result<TxStream, StreamError> {
        if h.rate() <= 0.0 {
            let error = Error::Device {
                backend: BACKEND,
                context: "tx stream",
                message: "set the sample rate before streaming".into(),
            };
            return Err(StreamError {
                hackrf: Box::new(h),
                error,
            });
        }
        let endpoint = match h.interface().endpoint::<Bulk, Out>(EP_BULK_OUT) {
            Ok(e) => e,
            Err(e) => {
                return Err(StreamError {
                    hackrf: Box::new(h),
                    error: crate::usb_err("claim bulk endpoint", e),
                });
            }
        };
        // Whole packets, as the firmware's DMA takes them.
        let mps = endpoint.max_packet_size().max(64);
        let transfer_bytes = if transfer_bytes > 0 {
            transfer_bytes
        } else {
            DEFAULT_TRANSFER_BYTES
        };
        let transfer_bytes = (transfer_bytes / mps).max(1) * mps;
        let queue_depth = if queue_depth > 0 {
            queue_depth
        } else {
            DEFAULT_QUEUE_DEPTH
        };
        let core = TxCore {
            endpoint,
            transfer_bytes,
            queue_depth,
            stage: Vec::new(),
            submitted: 0,
        };
        let snapshot = Mutex::new(Snapshot::of(&h, transfer_bytes / 2));
        // In a browser the page is one thread and nusb's device is not
        // Send, so clippy sees an Arc that could be an Rc; the handles
        // share it the same way on every target, by one code path.
        #[cfg_attr(target_arch = "wasm32", allow(clippy::arc_with_non_send_sync))]
        let shared = Arc::new(Shared {
            radio: Lock::new(Some(h)),
            tx: Lock::new(Some(core)),
            snapshot,
            started: AtomicBool::new(false),
        });
        Ok(TxStream { shared })
    }

    /// The two halves, each usable on its own thread or task.
    pub fn split(self) -> (WriteHalf, TxControl) {
        (
            WriteHalf {
                shared: Arc::clone(&self.shared),
            },
            TxControl {
                shared: self.shared,
            },
        )
    }

    /// The halves of one stream joined again; handed back if they are not
    /// the same stream's, or a clone of the control still exists.
    pub fn reunite(
        write: WriteHalf,
        control: TxControl,
    ) -> core::result::Result<TxStream, (WriteHalf, TxControl)> {
        if !Arc::ptr_eq(&write.shared, &control.shared) || Arc::strong_count(&write.shared) != 2 {
            return Err((write, control));
        }
        drop(write);
        Ok(TxStream {
            shared: control.shared,
        })
    }

    /// Pad the last transfer with zeros, key up if nothing has yet, wait
    /// until the radio has sent every byte, unkey, and read the firmware's
    /// counters: the radio back and the report.
    pub async fn finish(self) -> Result<TxFinished> {
        self.shared.flush().await?;
        let mut core = self
            .shared
            .tx
            .lock()
            .await
            .take()
            .ok_or_else(finished_error)?;
        let mut m0 = M0State::default();
        let mut sent_all = core.submitted == 0;
        if core.submitted > 0 {
            self.shared.start().await?;
        }
        let h = self
            .shared
            .radio
            .lock()
            .await
            .take()
            .ok_or_else(finished_error)?;
        self.shared.snap().finished = true;
        if core.submitted > 0 {
            while core.endpoint.pending() > 0 {
                if !core.reap(SEND_TIMEOUT).await? {
                    break;
                }
            }
            // Every transfer is in the radio: a shortfall so far was a
            // starved transmission, one from now on is only the end.
            m0 = h.m0_state().await.map_err(at("finish: the counters"))?;
            let left = core
                .submitted
                .saturating_sub(unwrap_count(m0.m0_count, core.submitted));
            let deadline = Instant::now()
                + Duration::from_secs_f64(left as f64 / 2.0 / h.rate())
                + FINISH_SLACK;
            loop {
                let now = h.m0_state().await.map_err(at("finish: the count"))?;
                if unwrap_count(now.m0_count, core.submitted) >= core.submitted {
                    sent_all = true;
                    break;
                }
                if Instant::now() >= deadline {
                    break;
                }
                sdr_core::time::sleep(FINISH_POLL).await;
            }
        }
        h.set_transceiver_mode(MODE_OFF)
            .await
            .map_err(at("finish: off"))?;
        #[cfg(not(target_arch = "wasm32"))]
        {
            core.endpoint.cancel_all();
            while core.endpoint.pending() > 0
                && core.reap(Duration::from_millis(200)).await.unwrap_or(false)
            {}
        }
        let report = TxReport {
            samples: core.submitted / 2,
            underruns: m0.num_shortfalls,
            longest_underrun: m0.longest_shortfall as u64 / 2,
            sent_all,
            breaks: self.shared.snap().breaks.clone(),
        };
        Ok(TxFinished { hackrf: h, report })
    }
}
