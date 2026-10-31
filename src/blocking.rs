//! The same radio, blocking: each method runs its async counterpart to
//! completion on the calling thread (`sdr_core::block_on`). Native only — a
//! browser cannot block. The receive streams are [`std::io::Read`] and
//! `sdr_core::RxSource`; the transmit streams [`std::io::Write`].
//!
//! ```no_run
//! use std::io::Read;
//! use sdr_core::TuneRequest;
//! use sdr_hackrf::blocking::HackRf;
//!
//! let mut hackrf = HackRf::open("")?;
//! hackrf.set_rate(10e6)?;
//! hackrf.set_freq(&TuneRequest::auto(446.0e6))?;
//! hackrf.set_gain(40.0)?;
//! let (mut reader, control) = hackrf.into_stream(0, 0).map_err(|e| e.error)?.split();
//! let mut buf = [0u8; 65536];
//! let n = reader.read(&mut buf)?; // I, Q, I, Q, … signed bytes
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use std::boxed::Box;
use std::io;
use std::string::{String, ToString};
use std::vec::Vec;

use sdr_core::gain::StageReading;
use sdr_core::{
    DeviceDesc, Error, Iq, RecvInfo, Result, RxSource, TuneRequest, TuneResult, block_on,
    elevate_current_thread, open_with_deadline,
};

use crate::session::{self, Break, StreamReport};
use crate::{BACKEND, M0State, Stage, TxReport};

/// Every attached HackRF ([`crate::list`]).
pub fn list() -> Vec<DeviceDesc> {
    block_on(crate::list())
}

/// An open HackRF ([`crate::HackRf`]), blocking.
pub struct HackRf(crate::HackRf);

impl From<crate::HackRf> for HackRf {
    fn from(h: crate::HackRf) -> HackRf {
        HackRf(h)
    }
}

impl core::fmt::Debug for HackRf {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl HackRf {
    /// [`crate::HackRf::open`], bounded by `sdr_core::open_with_deadline`:
    /// a device that enumerates but never answers fails within seconds
    /// instead of hanging the caller.
    pub fn open(spec: &str) -> Result<HackRf> {
        let spec = spec.to_string();
        open_with_deadline(BACKEND, move || block_on(crate::HackRf::open(&spec))).map(HackRf)
    }

    /// The async radio.
    pub fn into_async(self) -> crate::HackRf {
        self.0
    }

    pub fn label(&self) -> &str {
        self.0.label()
    }

    pub fn set_rate(&mut self, sps: f64) -> Result<()> {
        block_on(self.0.set_rate(sps))
    }

    pub fn rate(&self) -> f64 {
        self.0.rate()
    }

    pub fn set_bandwidth(&mut self, hz: f64) -> Result<()> {
        block_on(self.0.set_bandwidth(hz))
    }

    pub fn set_freq(&mut self, req: &TuneRequest) -> Result<TuneResult> {
        block_on(self.0.set_freq(req))
    }

    pub fn freq(&self) -> f64 {
        self.0.freq()
    }

    pub fn set_gain(&mut self, db: f64) -> Result<()> {
        block_on(self.0.set_gain(db))
    }

    pub fn gain(&self) -> f64 {
        self.0.gain()
    }

    pub fn stage_levels_db(stage: Stage) -> Vec<f64> {
        crate::gain::stage_levels_db(stage)
    }

    pub fn stage_readings(&self) -> Vec<StageReading<Stage>> {
        self.0.stage_readings()
    }

    pub fn set_stage_level(&mut self, stage: Stage, level: usize) -> Result<()> {
        block_on(self.0.set_stage_level(stage, level))
    }

    pub fn lna_gain(&self) -> u16 {
        self.0.lna_gain()
    }

    pub fn vga_gain(&self) -> u16 {
        self.0.vga_gain()
    }

    pub fn amp(&self) -> bool {
        self.0.amp()
    }

    pub fn txvga_gain(&self) -> u16 {
        self.0.txvga_gain()
    }

    pub fn set_lna_gain(&mut self, db: u16) -> Result<()> {
        block_on(self.0.set_lna_gain(db))
    }

    pub fn set_vga_gain(&mut self, db: u16) -> Result<()> {
        block_on(self.0.set_vga_gain(db))
    }

    pub fn set_txvga_gain(&mut self, db: u16) -> Result<()> {
        block_on(self.0.set_txvga_gain(db))
    }

    pub fn set_amp(&mut self, on: bool) -> Result<()> {
        block_on(self.0.set_amp(on))
    }

    pub fn set_bias_tee(&mut self, on: bool) -> Result<()> {
        block_on(self.0.set_bias_tee(on))
    }

    pub fn board_id(&self) -> Result<u8> {
        block_on(self.0.board_id())
    }

    pub fn version_string(&self) -> Result<String> {
        block_on(self.0.version_string())
    }

    pub fn max2837_read(&self, reg: u16) -> Result<u16> {
        block_on(self.0.max2837_read(reg))
    }

    pub fn si5351c_read(&self, reg: u16) -> Result<u8> {
        block_on(self.0.si5351c_read(reg))
    }

    pub fn rffc5071_read(&self, reg: u16) -> Result<u16> {
        block_on(self.0.rffc5071_read(reg))
    }

    pub fn m0_state(&self) -> Result<M0State> {
        block_on(self.0.m0_state())
    }

    /// [`crate::HackRf::into_stream`].
    pub fn into_stream(
        self,
        transfer_bytes: usize,
        queue_depth: usize,
    ) -> core::result::Result<Stream, StreamError> {
        block_on(self.0.into_stream(transfer_bytes, queue_depth))
            .map(Stream)
            .map_err(StreamError::from)
    }

    /// [`crate::HackRf::into_tx_stream`].
    pub fn into_tx_stream(
        self,
        transfer_bytes: usize,
        queue_depth: usize,
    ) -> core::result::Result<TxStream, StreamError> {
        block_on(self.0.into_tx_stream(transfer_bytes, queue_depth))
            .map(TxStream)
            .map_err(StreamError::from)
    }
}

/// A stream could not be made; the radio is handed back.
#[derive(Debug)]
pub struct StreamError {
    pub hackrf: Box<HackRf>,
    pub error: Error,
}

impl From<session::StreamError> for StreamError {
    fn from(e: session::StreamError) -> StreamError {
        StreamError {
            hackrf: Box::new(HackRf(*e.hackrf)),
            error: e.error,
        }
    }
}

impl core::fmt::Display for StreamError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.error.fmt(f)
    }
}

impl core::error::Error for StreamError {}

/// A finished stream ([`crate::Finished`]): the radio back, blocking.
pub struct Finished {
    pub hackrf: HackRf,
    pub report: StreamReport,
    /// The last wire bytes (signed 8-bit I then Q).
    pub tail: Vec<u8>,
}

impl Finished {
    /// The tail widened to [`Iq`].
    pub fn tail_samples(&self) -> Vec<Iq> {
        session::widen(&self.tail)
    }
}

impl From<crate::Finished> for Finished {
    fn from(f: crate::Finished) -> Finished {
        Finished {
            hackrf: HackRf(f.hackrf),
            report: f.report,
            tail: f.tail,
        }
    }
}

/// A finished transmission ([`crate::TxFinished`]): the radio back.
pub struct TxFinished {
    pub hackrf: HackRf,
    pub report: TxReport,
}

impl From<crate::TxFinished> for TxFinished {
    fn from(f: crate::TxFinished) -> TxFinished {
        TxFinished {
            hackrf: HackRf(f.hackrf),
            report: f.report,
        }
    }
}

/// [`crate::Stream`], blocking.
pub struct Stream(crate::Stream);

/// [`crate::ReadHalf`], blocking.
pub struct ReadHalf(crate::ReadHalf);

/// [`crate::Control`], blocking. Cheap to clone.
#[derive(Clone)]
pub struct Control(crate::Control);

/// [`crate::TxStream`], blocking.
pub struct TxStream(crate::TxStream);

/// [`crate::WriteHalf`], blocking.
pub struct WriteHalf(crate::WriteHalf);

/// [`crate::TxControl`], blocking. Cheap to clone.
#[derive(Clone)]
pub struct TxControl(crate::TxControl);

/// What every receive handle reports.
macro_rules! rx_reports {
    ($t:ident) => {
        impl $t {
            pub fn rate(&self) -> f64 {
                self.0.rate()
            }

            pub fn freq(&self) -> f64 {
                self.0.freq()
            }

            pub fn gain(&self) -> f64 {
                self.0.gain()
            }

            pub fn stage_readings(&self) -> Vec<StageReading<Stage>> {
                self.0.stage_readings()
            }

            pub fn label(&self) -> String {
                self.0.label()
            }

            pub fn delivered(&self) -> u64 {
                self.0.delivered()
            }

            pub fn breaks(&self) -> Vec<Break> {
                self.0.breaks()
            }

            pub fn max_num_samps(&self) -> usize {
                self.0.max_num_samps()
            }
        }
    };
}

/// What changes the radio while it receives.
macro_rules! rx_changes {
    ($t:ident) => {
        impl $t {
            /// [`crate::Stream::reconfigure`].
            pub fn reconfigure<T>(
                &self,
                f: impl AsyncFnOnce(&mut crate::HackRf) -> Result<T>,
            ) -> Result<(T, Break)> {
                block_on(self.0.reconfigure(f))
            }

            pub fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
                block_on(self.0.retune(req))
            }

            pub fn set_gain(&self, db: f64) -> Result<Break> {
                block_on(self.0.set_gain(db))
            }

            pub fn set_stage_level(&self, stage: Stage, level: usize) -> Result<Break> {
                block_on(self.0.set_stage_level(stage, level))
            }

            pub fn set_bandwidth(&self, hz: f64) -> Result<Break> {
                block_on(self.0.set_bandwidth(hz))
            }

            pub fn m0_state(&self) -> Result<M0State> {
                block_on(self.0.m0_state())
            }
        }
    };
}

/// What reads the samples.
macro_rules! reads {
    ($t:ident) => {
        impl $t {
            /// [`crate::Stream::recv_into`].
            pub fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
                block_on(self.0.recv_into(buf, timeout_secs))
            }

            /// Start the stream now rather than at the first read.
            pub fn start(&mut self) -> Result<()> {
                block_on(self.0.start())
            }
        }

        /// The wire bytes: signed 8-bit I then Q, as the radio sends them.
        /// Blocks until at least one byte; end of file once the stream has
        /// finished.
        impl io::Read for $t {
            fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
                block_on(self.0.read(buf)).map_err(io::Error::other)
            }
        }

        impl RxSource for $t {
            fn max_num_samps(&self) -> usize {
                self.0.max_num_samps()
            }

            fn start(&mut self) -> Result<()> {
                block_on(self.0.start())
            }

            fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
                block_on(self.0.recv_into(buf, timeout_secs))
            }

            fn setup_worker_thread(&self) {
                elevate_current_thread();
            }
        }
    };
}

/// What every transmit handle reports.
macro_rules! tx_reports {
    ($t:ident) => {
        impl $t {
            pub fn rate(&self) -> f64 {
                self.0.rate()
            }

            pub fn freq(&self) -> f64 {
                self.0.freq()
            }

            pub fn txvga_gain(&self) -> u16 {
                self.0.txvga_gain()
            }

            pub fn amp(&self) -> bool {
                self.0.amp()
            }

            pub fn label(&self) -> String {
                self.0.label()
            }

            pub fn written(&self) -> u64 {
                self.0.written()
            }

            pub fn is_keyed(&self) -> bool {
                self.0.is_keyed()
            }

            pub fn breaks(&self) -> Vec<Break> {
                self.0.breaks()
            }
        }
    };
}

/// What changes the radio while it transmits.
macro_rules! tx_changes {
    ($t:ident) => {
        impl $t {
            /// [`crate::TxStream::reconfigure`].
            pub fn reconfigure<T>(
                &self,
                f: impl AsyncFnOnce(&mut crate::HackRf) -> Result<T>,
            ) -> Result<(T, Break)> {
                block_on(self.0.reconfigure(f))
            }

            pub fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
                block_on(self.0.retune(req))
            }

            pub fn set_txvga_gain(&self, db: u16) -> Result<Break> {
                block_on(self.0.set_txvga_gain(db))
            }

            pub fn set_amp(&self, on: bool) -> Result<Break> {
                block_on(self.0.set_amp(on))
            }

            pub fn m0_state(&self) -> Result<M0State> {
                block_on(self.0.m0_state())
            }
        }
    };
}

/// What writes the samples.
macro_rules! writes {
    ($t:ident) => {
        impl $t {
            /// [`crate::TxStream::send`].
            pub fn send(&mut self, iq: &[Iq]) -> Result<()> {
                block_on(self.0.send(iq))
            }

            /// Key up now rather than when the queue first fills.
            pub fn start(&mut self) -> Result<()> {
                block_on(self.0.start())
            }

            pub fn samples_per_transfer(&self) -> usize {
                self.0.samples_per_transfer()
            }
        }

        /// Wire bytes to send: signed 8-bit I then Q. `flush` pads what is
        /// staged with zeros (silence) to a whole transfer.
        impl io::Write for $t {
            fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
                block_on(self.0.write(buf)).map_err(io::Error::other)
            }

            fn flush(&mut self) -> io::Result<()> {
                block_on(self.0.flush()).map_err(io::Error::other)
            }
        }
    };
}

rx_reports!(Stream);
rx_reports!(ReadHalf);
rx_reports!(Control);
rx_changes!(Stream);
rx_changes!(Control);
reads!(Stream);
reads!(ReadHalf);
tx_reports!(TxStream);
tx_reports!(WriteHalf);
tx_reports!(TxControl);
tx_changes!(TxStream);
tx_changes!(TxControl);
writes!(TxStream);
writes!(WriteHalf);

impl Stream {
    /// The async stream.
    pub fn into_async(self) -> crate::Stream {
        self.0
    }

    /// The two halves ([`crate::Stream::split`]).
    pub fn split(self) -> (ReadHalf, Control) {
        let (r, c) = self.0.split();
        (ReadHalf(r), Control(c))
    }

    /// The halves joined again ([`crate::Stream::reunite`]).
    pub fn reunite(
        read: ReadHalf,
        control: Control,
    ) -> core::result::Result<Stream, (ReadHalf, Control)> {
        crate::Stream::reunite(read.0, control.0)
            .map(Stream)
            .map_err(|(r, c)| (ReadHalf(r), Control(c)))
    }

    /// [`crate::Stream::finish`].
    pub fn finish(self) -> Result<Finished> {
        block_on(self.0.finish()).map(Finished::from)
    }
}

impl TxStream {
    /// The async stream.
    pub fn into_async(self) -> crate::TxStream {
        self.0
    }

    /// The two halves ([`crate::TxStream::split`]).
    pub fn split(self) -> (WriteHalf, TxControl) {
        let (w, c) = self.0.split();
        (WriteHalf(w), TxControl(c))
    }

    /// The halves joined again ([`crate::TxStream::reunite`]).
    pub fn reunite(
        write: WriteHalf,
        control: TxControl,
    ) -> core::result::Result<TxStream, (WriteHalf, TxControl)> {
        crate::TxStream::reunite(write.0, control.0)
            .map(TxStream)
            .map_err(|(w, c)| (WriteHalf(w), TxControl(c)))
    }

    /// [`crate::TxStream::finish`].
    pub fn finish(self) -> Result<TxFinished> {
        block_on(self.0.finish()).map(TxFinished::from)
    }
}
