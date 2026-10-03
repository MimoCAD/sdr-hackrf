//! HackRF vendor-request protocol constants.
//!
//! "M0" and "M4" are the HackRF's own two processor cores: its LPC4320
//! microcontroller has an ARM Cortex-M0, which moves samples between the
//! radio and the sample buffer, and a Cortex-M4, which runs the firmware
//! and moves the buffer to and from USB. Neither is the host's.
//!
//! Provenance: the hackrf project's documented host protocol, cross-checked
//! against seify-hackrfone (pure-Rust nusb reference implementation).

/// HackRF One (also matches the older Jawbreaker at 0x604b). A HackRF Pro
/// enumerates as 0x6089 too.
pub const KNOWN_IDS: &[(u16, u16, &str)] = &[
    (0x1d50, 0x6089, "HackRF One"),
    (0x1d50, 0x604b, "HackRF Jawbreaker"),
];

// Vendor request codes.
pub const SET_TRANSCEIVER_MODE: u8 = 1;
pub const MAX2837_READ: u8 = 3;
pub const SI5351C_READ: u8 = 5;
pub const SAMPLE_RATE_SET: u8 = 6;
pub const BASEBAND_FILTER_BANDWIDTH_SET: u8 = 7;
pub const RFFC5071_READ: u8 = 9;
pub const BOARD_ID_READ: u8 = 14;
pub const VERSION_STRING_READ: u8 = 15;
pub const SET_FREQ: u8 = 16;
pub const AMP_ENABLE: u8 = 17;
pub const SET_LNA_GAIN: u8 = 19;
pub const SET_VGA_GAIN: u8 = 20;
pub const SET_TXVGA_GAIN: u8 = 21;
pub const ANTENNA_ENABLE: u8 = 23;
/// Firmware ≥ ~2021.03: the M0 core's transfer/shortfall accounting.
pub const GET_M0_STATE: u8 = 41;
/// The length of GET_M0_STATE's answer, bytes.
pub const M0_STATE_LEN: u16 = 40;

// Transceiver modes (wValue of SET_TRANSCEIVER_MODE).
pub const MODE_OFF: u16 = 0;
pub const MODE_RECEIVE: u16 = 1;
pub const MODE_TRANSMIT: u16 = 2;

/// Bulk IQ endpoint (RX).
pub const EP_BULK_IN: u8 = 0x81;
/// Bulk IQ endpoint (TX): same signed-i8 wire format, host → device.
pub const EP_BULK_OUT: u8 = 0x02;

/// The M4 core moves samples to and from USB in pieces of this many bytes:
/// every `m4_count` read while receiving was a multiple of it, and after a
/// stop up to one piece stayed in the device, never sent (the M0 counter
/// ahead of the M4's by less than this). Measured, not documented.
pub const M4_PIECE_BYTES: u32 = 16 * 1024;

/// The MAX2837's selectable baseband filter bandwidths, Hz. The filter is
/// chosen host-side: widest value ≤ 75% of the sample rate.
pub const BASEBAND_FILTERS_HZ: [u32; 16] = [
    1_750_000, 2_500_000, 3_500_000, 5_000_000, 5_500_000, 6_000_000, 7_000_000, 8_000_000,
    9_000_000, 10_000_000, 12_000_000, 14_000_000, 15_000_000, 20_000_000, 24_000_000, 28_000_000,
];

pub fn filter_bw_for_rate(rate_hz: u32) -> u32 {
    let target = (rate_hz as u64 * 3 / 4) as u32;
    BASEBAND_FILTERS_HZ
        .iter()
        .copied()
        .rfind(|&bw| bw <= target)
        .unwrap_or(BASEBAND_FILTERS_HZ[0])
}

/// The M0 core's transfer accounting (firmware ≥ ~2021.03): the HackRF's
/// own record of whether the USB side kept pace with the sample clock.
///
/// Measured on a HackRF Pro: a mode start zeroes the counts (once the
/// firmware has switched, which is after the request returns); while
/// receiving `m4_count` is exactly the bytes the host has received and
/// `m0_count` leads it by the device's buffer fill; reading this every
/// 10 ms at 10 Msps cost nothing. A *shortfall* is the buffer found full
/// (receive) or empty (transmit): the bytes it lasted are not in
/// `m0_count`, so `m0_count` stays the host stream's byte index; only the
/// longest one's length is kept, not their sum.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct M0State {
    /// Requested mode: 0 IDLE, 1 WAIT, 2 RX, 3 TX_START, 4 TX_RUN.
    pub requested_mode: u16,
    /// Request flag: 0 = completed, non-zero = pending.
    pub request_flag: u16,
    /// Active mode (same encoding as `requested_mode`).
    pub active_mode: u32,
    /// Bytes moved by the M0 core since the mode started (wraps at 2³²).
    pub m0_count: u32,
    /// Bytes moved by the M4 core, to or from USB (wraps at 2³²).
    pub m4_count: u32,
    /// Shortfalls since the mode started.
    pub num_shortfalls: u32,
    /// The longest shortfall, bytes (÷2 = samples).
    pub longest_shortfall: u32,
    /// Shortfall limit, bytes.
    pub shortfall_limit: u32,
    /// Threshold m0_count value (bytes) for the next mode change.
    pub threshold: u32,
    /// Mode which will be switched to when the threshold is reached.
    pub next_mode: u32,
    /// Error, if any, that caused the M0 to revert to IDLE.
    pub error: u32,
}

/// `active_mode` while receiving.
pub const M0_MODE_RX: u32 = 2;

impl M0State {
    /// GET_M0_STATE's answer, little-endian; `None` if it is short.
    pub fn parse(r: &[u8]) -> Option<M0State> {
        if r.len() < M0_STATE_LEN as usize {
            return None;
        }
        let u16le = |i: usize| u16::from_le_bytes([r[i], r[i + 1]]);
        let u32le = |i: usize| u32::from_le_bytes([r[i], r[i + 1], r[i + 2], r[i + 3]]);
        Some(M0State {
            requested_mode: u16le(0),
            request_flag: u16le(2),
            active_mode: u32le(4),
            m0_count: u32le(8),
            m4_count: u32le(12),
            num_shortfalls: u32le(16),
            longest_shortfall: u32le(20),
            shortfall_limit: u32le(24),
            threshold: u32le(28),
            next_mode: u32le(32),
            error: u32le(36),
        })
    }
}

/// A 32-bit byte count that wraps, unwrapped against a 64-bit one known to
/// be within 2³¹ of it, ahead or behind.
pub fn unwrap_count(count: u32, near: u64) -> u64 {
    near.saturating_add_signed(count.wrapping_sub(near as u32) as i32 as i64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn m0_state_parses_little_endian() {
        let mut r = [0u8; 40];
        r[4] = 2;
        r[8..12].copy_from_slice(&60_045_920u32.to_le_bytes());
        r[12..16].copy_from_slice(&60_030_976u32.to_le_bytes());
        r[16] = 1;
        r[20..24].copy_from_slice(&3_947_232u32.to_le_bytes());
        let m = M0State::parse(&r).unwrap();
        assert_eq!((m.active_mode, m.m0_count, m.m4_count, m.num_shortfalls, m.longest_shortfall), (2, 60_045_920, 60_030_976, 1, 3_947_232));
        assert_eq!(M0State::parse(&r[..39]), None);
    }

    #[test]
    fn counts_unwrap_across_the_wrap() {
        assert_eq!(unwrap_count(100, 50), 100);
        assert_eq!(unwrap_count(16, (1 << 32) - 16), (1 << 32) + 16);
        assert_eq!(unwrap_count(5, (3 << 32) + 1), (3 << 32) + 5);
        assert_eq!(unwrap_count(10, 20), 10);
        assert_eq!(unwrap_count(u32::MAX - 7, (1 << 32) + 8), (1 << 32) - 8);
    }

    #[test]
    fn the_filter_is_the_widest_within_three_quarters() {
        assert_eq!(filter_bw_for_rate(10_000_000), 7_000_000);
        assert_eq!(filter_bw_for_rate(2_000_000), 1_750_000);
    }
}
