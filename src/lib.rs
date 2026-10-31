//! A HackRF driver over nusb, in pure Rust: receive and transmit.
//!
//! The HackRF's ARM firmware manages the RF chips (MAX2837 transceiver,
//! RFFC5072 mixer, Si5351C clock generator); the host protocol is vendor
//! control requests plus a bulk stream of interleaved signed 8-bit I/Q.
//! The firmware exposes raw register *reads* for all three chips but
//! reports no synthesized-frequency readback, so `freq()` echoes the
//! target. Protocol per the hackrf project's documentation, cross-checked
//! against seify-hackrfone.
//!
//! A radio receives by becoming a [`Stream`] ([`HackRf::into_stream`]): it
//! reads as [`std::io::Read`] (the wire bytes, the layout `hackrf_transfer`
//! writes) or as [`sdr_core::Iq`] samples, splits into a [`ReadHalf`] and
//! a [`Control`] for two threads or tasks, and its `finish` hands the radio
//! back with the firmware's count of every shortfall. It transmits by
//! becoming a [`TxStream`] ([`HackRf::into_tx_stream`]): an
//! [`std::io::Write`] of the same wire bytes, split into a [`WriteHalf`]
//! and a [`TxControl`]. The firmware runs one transceiver mode at a time,
//! so a radio is one or the other: each stream owns it until `finish`
//! hands it back.
//!
//! Every method that waits is async: in a browser (WebUSB) it suspends;
//! natively it completes in place (nusb's native requests block), and
//! [`blocking`] wraps each one.
//!
//! `no_std` at its core (with `alloc`): the protocol's encodings
//! ([`proto`]) and the gain stages ([`gain`]) are computation only. The
//! default `std` feature adds the radio itself.

#![no_std]

extern crate alloc;
#[cfg(feature = "std")]
#[macro_use]
extern crate std;

/// A nusb operation completed: awaited in a browser (WebUSB is async),
/// waited on natively (nusb's native operations block, and awaiting them
/// would need an async runtime).
#[cfg(feature = "std")]
macro_rules! complete {
    ($op:expr) => {{
        #[cfg(target_arch = "wasm32")]
        {
            $op.await
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            nusb::MaybeFuture::wait($op)
        }
    }};
}

#[cfg(all(feature = "std", not(target_arch = "wasm32")))]
pub mod blocking;
pub mod gain;
pub mod proto;
#[cfg(feature = "std")]
pub mod session;
#[cfg(feature = "std")]
pub mod tx;

pub use gain::Stage;
pub use proto::M0State;
#[cfg(feature = "std")]
pub use session::{Break, Control, Finished, ReadHalf, Stream, StreamError, StreamReport};
#[cfg(feature = "std")]
pub use tx::{TxControl, TxFinished, TxReport, TxStream, WriteHalf};

#[cfg(feature = "std")]
use alloc::string::{String, ToString};
#[cfg(feature = "std")]
use alloc::vec::Vec;
#[cfg(feature = "std")]
use core::time::Duration;

#[cfg(feature = "std")]
use nusb::DeviceInfo;
#[cfg(feature = "std")]
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};

#[cfg(feature = "std")]
use sdr_core::gain::{StageReading, nearest_level};
#[cfg(feature = "std")]
use sdr_core::{Backend, DeviceDesc, Error, Result, TunePolicy, TuneRequest, TuneResult};

#[cfg(feature = "std")]
use crate::gain::{AMP_DB, TXVGA_MAX_DB, lna_step, vga_step};
#[cfg(feature = "std")]
use crate::proto::{
    AMP_ENABLE, ANTENNA_ENABLE, BASEBAND_FILTER_BANDWIDTH_SET, BOARD_ID_READ, GET_M0_STATE,
    KNOWN_IDS, M0_STATE_LEN, MAX2837_READ, MODE_OFF, RFFC5071_READ, SAMPLE_RATE_SET, SET_FREQ,
    SET_LNA_GAIN, SET_TRANSCEIVER_MODE, SET_TXVGA_GAIN, SET_VGA_GAIN, SI5351C_READ,
    VERSION_STRING_READ, filter_bw_for_rate,
};

/// The backend name every error carries.
pub const BACKEND: &str = "hackrf";

/// The sample rates the firmware takes, samples per second.
pub const RATE_RANGE_SPS: (f64, f64) = (1e6, 20e6);

/// The tunable RF range, Hz: 1 MHz to 6 GHz, as the HackRF One is
/// specified.
pub const FREQ_RANGE_HZ: (f64, f64) = (1e6, 6e9);

#[cfg(feature = "std")]
const CTRL_TIMEOUT: Duration = Duration::from_millis(500);

#[cfg(feature = "std")]
fn usb_err(context: &'static str, e: impl core::fmt::Display) -> Error {
    Error::Usb {
        backend: BACKEND,
        context,
        message: e.to_string(),
    }
}

#[cfg(feature = "std")]
fn rejected(context: &'static str) -> Error {
    Error::Device {
        backend: BACKEND,
        context,
        message: "firmware rejected the value".into(),
    }
}

/// The attached HackRFs this process may open (in a browser, those the
/// page has been granted, [`HackRf::request`]).
#[cfg(feature = "std")]
async fn candidates() -> Vec<DeviceInfo> {
    match complete!(nusb::list_devices()) {
        Ok(list) => list
            .filter(|d| {
                KNOWN_IDS
                    .iter()
                    .any(|&(v, p, _)| d.vendor_id() == v && d.product_id() == p)
            })
            .collect(),
        Err(_) => Vec::new(),
    }
}

#[cfg(feature = "std")]
fn label_of(d: &DeviceInfo) -> String {
    format!(
        "{} (serial {})",
        d.product_string().unwrap_or("HackRF"),
        d.serial_number().unwrap_or("?")
    )
}

/// Every attached HackRF, as `hackrf:serial=<sn>` (or `hackrf:` when the
/// device reports no serial), without opening any.
#[cfg(feature = "std")]
pub async fn list() -> Vec<DeviceDesc> {
    candidates()
        .await
        .iter()
        .map(|d| DeviceDesc {
            spec: match d.serial_number() {
                Some(sn) => format!("hackrf:serial={sn}"),
                None => "hackrf:".to_string(),
            },
            label: label_of(d),
            backend: Backend::HackRf,
        })
        .collect()
}

/// An open HackRF.
#[cfg(feature = "std")]
pub struct HackRf {
    device: nusb::Device,
    interface: nusb::Interface,
    label: String,
    rate_hz: f64,
    freq_hz: f64,
    /// What was last set: the firmware has no readback of any of these.
    lna_db: u16,
    vga_db: u16,
    txvga_db: u16,
    amp_on: bool,
}

#[cfg(feature = "std")]
impl core::fmt::Debug for HackRf {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("HackRf")
            .field("label", &self.label)
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "std")]
impl HackRf {
    /// Open by spec remainder: `""` (the first found) or `"serial=<sn>"`,
    /// with the transceiver off. In a browser the devices are those the
    /// page was granted ([`HackRf::request`]). (`blocking::HackRf::open`
    /// bounds a native open with `sdr_core::open_with_deadline`: a wedged
    /// device can enumerate and then never answer.)
    pub async fn open(rest: &str) -> Result<HackRf> {
        let not_found = || Error::NotFound {
            spec: format!("hackrf:{rest}"),
        };
        let want_serial = rest.strip_prefix("serial=");
        if !rest.is_empty() && want_serial.is_none() {
            return Err(not_found());
        }
        let info = candidates()
            .await
            .into_iter()
            .find(|d| want_serial.is_none_or(|sn| d.serial_number() == Some(sn)))
            .ok_or_else(not_found)?;
        HackRf::open_info(info).await
    }

    /// In a browser: ask the user for a HackRF (the WebUSB device chooser,
    /// filtered to [`proto::KNOWN_IDS`]; it must follow a user gesture, such
    /// as a click), then open it as [`HackRf::open`] does. The page keeps
    /// the grant, so later [`HackRf::open`] calls find it without asking.
    #[cfg(target_arch = "wasm32")]
    pub async fn request() -> Result<HackRf> {
        let selectors: Vec<nusb::DeviceSelector> = KNOWN_IDS
            .iter()
            .map(|&(v, p, _)| nusb::DeviceSelector::all().with_vid_pid(v, p))
            .collect();
        let info = nusb::request_device(&selectors)
            .await
            .map_err(|e| usb_err("request device", e))?
            .ok_or_else(|| Error::NotFound {
                spec: "hackrf: (no device chosen)".into(),
            })?;
        HackRf::open_info(info).await
    }

    async fn open_info(info: DeviceInfo) -> Result<HackRf> {
        let label = label_of(&info);
        let device = complete!(info.open()).map_err(|e| usb_err("open", e))?;
        let interface =
            complete!(device.claim_interface(0)).map_err(|e| usb_err("claim_interface", e))?;
        let h = HackRf {
            device,
            interface,
            label,
            rate_hz: 0.0,
            freq_hz: 0.0,
            lna_db: 16,
            vga_db: 20,
            txvga_db: 0,
            amp_on: false,
        };
        // Known state: the transceiver off until a stream starts it.
        h.set_transceiver_mode(MODE_OFF).await?;
        Ok(h)
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    pub(crate) fn interface(&self) -> &nusb::Interface {
        &self.interface
    }

    async fn ctrl_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        let req = ControlOut {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request,
            value,
            index,
            data,
        };
        complete!(self.device.control_out(req, CTRL_TIMEOUT)).map_err(|e| usb_err("control_out", e))
    }

    async fn ctrl_in(&self, request: u8, value: u16, index: u16, length: u16) -> Result<Vec<u8>> {
        let req = ControlIn {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request,
            value,
            index,
            length,
        };
        complete!(self.device.control_in(req, CTRL_TIMEOUT)).map_err(|e| usb_err("control_in", e))
    }

    pub(crate) async fn set_transceiver_mode(&self, mode: u16) -> Result<()> {
        self.ctrl_out(SET_TRANSCEIVER_MODE, mode, 0, &[]).await
    }

    /// Sample rate, 1–20 Msps ([`RATE_RANGE_SPS`]). The firmware takes an
    /// exact (Hz, divider) pair; the anti-alias baseband filter is set to
    /// the widest plan ≤ 75% of the rate, the policy libhackrf uses. (Below
    /// ~2 Msps the narrowest baseband filter, 1.75 MHz, is already wider
    /// than the rate.)
    pub async fn set_rate(&mut self, rate_sps: f64) -> Result<()> {
        let hz = rate_sps.round() as u32;
        if !(RATE_RANGE_SPS.0 as u32..=RATE_RANGE_SPS.1 as u32).contains(&hz) {
            return Err(Error::Unsupported {
                backend: BACKEND,
                what: format!("sample rate {hz} Hz (valid: 1–20 Msps)"),
            });
        }
        let mut buf = [0u8; 8];
        buf[..4].copy_from_slice(&hz.to_le_bytes());
        buf[4..].copy_from_slice(&1u32.to_le_bytes()); // divider 1
        self.ctrl_out(SAMPLE_RATE_SET, 0, 0, &buf).await?;
        self.rate_hz = hz as f64;
        self.write_filter(filter_bw_for_rate(hz)).await
    }

    /// The rate as set (the firmware has no readback); 0 before
    /// [`HackRf::set_rate`].
    pub fn rate(&self) -> f64 {
        self.rate_hz
    }

    async fn write_filter(&self, bw: u32) -> Result<()> {
        self.ctrl_out(
            BASEBAND_FILTER_BANDWIDTH_SET,
            (bw & 0xffff) as u16,
            (bw >> 16) as u16,
            &[],
        )
        .await
    }

    /// Baseband filter bandwidth override (after `set_rate`, which resets it
    /// to the 75% policy). Snaps to the MAX2837's plan table.
    pub async fn set_bandwidth(&mut self, bw_hz: f64) -> Result<()> {
        self.write_filter(filter_bw_for_rate((bw_hz / 0.75) as u32))
            .await
    }

    pub async fn set_freq(&mut self, req: &TuneRequest) -> Result<TuneResult> {
        let rf_target = match req.rf {
            TunePolicy::Manual(hz) => hz,
            TunePolicy::Auto | TunePolicy::Default => req.target_freq,
        };
        if let TunePolicy::Manual(x) = req.dsp
            && x != 0.0
        {
            return Err(Error::Unsupported {
                backend: BACKEND,
                what: "manual DSP tune (the HackRF has no post-LO DSP stage)".into(),
            });
        }
        let hz = rf_target.round() as u64;
        let mut buf = [0u8; 8];
        buf[..4].copy_from_slice(&((hz / 1_000_000) as u32).to_le_bytes());
        buf[4..].copy_from_slice(&((hz % 1_000_000) as u32).to_le_bytes());
        self.ctrl_out(SET_FREQ, 0, 0, &buf).await?;
        self.freq_hz = hz as f64;
        Ok(TuneResult {
            clipped_rf_freq: rf_target,
            target_rf_freq: rf_target,
            // The firmware plans RFFC5072 + MAX2837 tuning internally and
            // reports nothing back; Hz-resolution requests land exactly.
            actual_rf_freq: hz as f64,
            target_dsp_freq: req.target_freq - hz as f64,
            actual_dsp_freq: 0.0,
        })
    }

    /// The RF as set (the firmware has no readback); 0 before
    /// [`HackRf::set_freq`].
    pub fn freq(&self) -> f64 {
        self.freq_hz
    }

    /// Overall receive gain in dB, split across the LNA then the VGA
    /// ([`gain::split`]). The +14 dB RF amp stays under explicit control
    /// ([`HackRf::set_amp`]).
    pub async fn set_gain(&mut self, db: f64) -> Result<()> {
        let (lna, vga) = gain::split(db);
        self.set_lna_gain(lna).await?;
        self.set_vga_gain(vga).await
    }

    /// Overall receive gain in dB. It *includes* the +14 dB amp when it is
    /// on, while [`HackRf::set_gain`] never touches the amp — so
    /// `set_gain(gain())` with the amp on asks for 14 dB more LNA + VGA
    /// than is set.
    pub fn gain(&self) -> f64 {
        (self.lna_db + self.vga_db + if self.amp_on { AMP_DB } else { 0 }) as f64
    }

    /// [`gain::stage_levels_db`].
    pub fn stage_levels_db(stage: Stage) -> Vec<f64> {
        gain::stage_levels_db(stage)
    }

    /// Where each receive stage sits, as last set (the power-on assumption
    /// until then).
    pub fn stage_readings(&self) -> Vec<StageReading<Stage>> {
        let reading = |stage: Stage, db: u16| {
            let levels = gain::stage_levels_db(stage);
            StageReading {
                stage,
                level: nearest_level(&levels, db as f64),
                levels: levels.len(),
                db: db as f64,
            }
        };
        vec![
            reading(Stage::Amp, if self.amp_on { AMP_DB } else { 0 }),
            reading(Stage::Lna, self.lna_db),
            reading(Stage::Vga, self.vga_db),
        ]
    }

    /// One receive stage to a level of [`gain::stage_levels_db`] (clamped
    /// to the stage's range).
    pub async fn set_stage_level(&mut self, stage: Stage, level: usize) -> Result<()> {
        let levels = gain::stage_levels_db(stage);
        let db = levels[level.min(levels.len() - 1)] as u16;
        match stage {
            Stage::Amp => self.set_amp(db != 0).await,
            Stage::Lna => self.set_lna_gain(db).await,
            Stage::Vga => self.set_vga_gain(db).await,
        }
    }

    /// The LNA gain in dB as last set.
    pub fn lna_gain(&self) -> u16 {
        self.lna_db
    }

    /// The baseband VGA gain in dB as last set.
    pub fn vga_gain(&self) -> u16 {
        self.vga_db
    }

    /// Whether the +14 dB RF amp is on, as last set.
    pub fn amp(&self) -> bool {
        self.amp_on
    }

    /// The transmit VGA gain in dB as last set.
    pub fn txvga_gain(&self) -> u16 {
        self.txvga_db
    }

    /// The LNA (IF) gain: 0–40 dB, snapped down to 8 dB steps.
    pub async fn set_lna_gain(&mut self, db: u16) -> Result<()> {
        let db = lna_step(db);
        if self.ctrl_in(SET_LNA_GAIN, 0, db, 1).await?.first() != Some(&1) {
            return Err(rejected("set_lna_gain"));
        }
        self.lna_db = db;
        Ok(())
    }

    /// The baseband VGA gain: 0–62 dB, snapped down to 2 dB steps.
    pub async fn set_vga_gain(&mut self, db: u16) -> Result<()> {
        let db = vga_step(db);
        if self.ctrl_in(SET_VGA_GAIN, 0, db, 1).await?.first() != Some(&1) {
            return Err(rejected("set_vga_gain"));
        }
        self.vga_db = db;
        Ok(())
    }

    /// The transmit VGA (IF) gain: 0–47 dB in 1 dB steps. The +14 dB amp
    /// applies on transmit too and stays under explicit
    /// [`HackRf::set_amp`] control.
    pub async fn set_txvga_gain(&mut self, db: u16) -> Result<()> {
        let db = db.min(TXVGA_MAX_DB);
        if self.ctrl_in(SET_TXVGA_GAIN, 0, db, 1).await?.first() != Some(&1) {
            return Err(rejected("set_txvga_gain"));
        }
        self.txvga_db = db;
        Ok(())
    }

    /// The +14 dB RF amplifier ahead of the mixer (receive and transmit).
    pub async fn set_amp(&mut self, on: bool) -> Result<()> {
        self.ctrl_out(AMP_ENABLE, on as u16, 0, &[]).await?;
        self.amp_on = on;
        Ok(())
    }

    /// Antenna port power (bias tee).
    pub async fn set_bias_tee(&mut self, on: bool) -> Result<()> {
        self.ctrl_out(ANTENNA_ENABLE, on as u16, 0, &[]).await
    }

    /// The board ID byte (2 = HackRF One).
    pub async fn board_id(&self) -> Result<u8> {
        let r = self.ctrl_in(BOARD_ID_READ, 0, 0, 1).await?;
        r.first().copied().ok_or_else(|| Error::Device {
            backend: BACKEND,
            context: "board_id",
            message: "empty response".into(),
        })
    }

    /// The firmware's version string.
    pub async fn version_string(&self) -> Result<String> {
        let r = self.ctrl_in(VERSION_STRING_READ, 0, 0, 64).await?;
        Ok(String::from_utf8_lossy(&r)
            .trim_end_matches('\0')
            .to_string())
    }

    /// A MAX2837 register, through the firmware's debug read.
    pub async fn max2837_read(&self, reg: u16) -> Result<u16> {
        let r = self.ctrl_in(MAX2837_READ, 0, reg, 2).await?;
        Ok(u16::from_le_bytes([
            r.first().copied().unwrap_or(0),
            r.get(1).copied().unwrap_or(0),
        ]))
    }

    /// An Si5351C register, through the firmware's debug read.
    pub async fn si5351c_read(&self, reg: u16) -> Result<u8> {
        Ok(self
            .ctrl_in(SI5351C_READ, 0, reg, 1)
            .await?
            .first()
            .copied()
            .unwrap_or(0))
    }

    /// An RFFC5071/5072 register, through the firmware's debug read.
    pub async fn rffc5071_read(&self, reg: u16) -> Result<u16> {
        let r = self.ctrl_in(RFFC5071_READ, 0, reg, 2).await?;
        Ok(u16::from_le_bytes([
            r.first().copied().unwrap_or(0),
            r.get(1).copied().unwrap_or(0),
        ]))
    }

    /// The M0 core's own transfer and shortfall accounting ([`M0State`]).
    /// `Unsupported` on firmware that predates the request.
    pub async fn m0_state(&self) -> Result<M0State> {
        let old_fw = || Error::Unsupported {
            backend: BACKEND,
            what: "M0 state readback (needs firmware ≥ 2021.03)".into(),
        };
        let r = self
            .ctrl_in(GET_M0_STATE, 0, 0, M0_STATE_LEN)
            .await
            .map_err(|_| old_fw())?;
        M0State::parse(&r).ok_or_else(old_fw)
    }

    /// The radio receiving, as a [`Stream`] at the rate it holds. The stream
    /// owns the radio until `finish` hands it back. `transfer_bytes` is
    /// rounded down to whole packets (0 is 64 KiB); `queue_depth` transfers
    /// are kept in flight (0 is 32). Nothing is started until the first read
    /// (or `start`).
    pub async fn into_stream(
        self,
        transfer_bytes: usize,
        queue_depth: usize,
    ) -> core::result::Result<Stream, StreamError> {
        Stream::new(self, transfer_bytes, queue_depth)
    }

    /// The radio transmitting, as a [`TxStream`] at the rate it holds; see
    /// [`TxStream`]. `transfer_bytes` 0 is 32 KiB, `queue_depth` 0 is 8.
    pub async fn into_tx_stream(
        self,
        transfer_bytes: usize,
        queue_depth: usize,
    ) -> core::result::Result<TxStream, StreamError> {
        TxStream::new(self, transfer_bytes, queue_depth)
    }
}

#[cfg(feature = "std")]
impl Drop for HackRf {
    fn drop(&mut self) {
        // Best effort, natively (a browser cannot wait here): the
        // transceiver off whatever happened.
        #[cfg(not(target_arch = "wasm32"))]
        {
            let req = ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: SET_TRANSCEIVER_MODE,
                value: MODE_OFF,
                index: 0,
                data: &[],
            };
            let _ = nusb::MaybeFuture::wait(self.device.control_out(req, CTRL_TIMEOUT));
        }
    }
}
