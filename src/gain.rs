//! The gain stages, as the HackRF firmware takes them
//! (`hackrf_set_lna_gain` / `hackrf_set_vga_gain` / `hackrf_set_txvga_gain`
//! / `hackrf_set_amp_enable`): the MAX2837's receive LNA 0–40 dB in 8 dB
//! steps and baseband VGA 0–62 dB in 2 dB steps, its transmit VGA 0–47 dB
//! in 1 dB steps, and the front end's switched RF amplifier, +14 dB.

use alloc::vec::Vec;

/// The receive gain stages, in signal order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Stage {
    /// The front end's switched RF amplifier: off or +14 dB.
    Amp,
    /// The MAX2837's LNA: 0–40 dB in 8 dB steps.
    Lna,
    /// The MAX2837's baseband VGA: 0–62 dB in 2 dB steps.
    Vga,
}

impl Stage {
    pub const ALL: [Stage; 3] = [Stage::Amp, Stage::Lna, Stage::Vga];
}

impl From<Stage> for sdr_core::gain::Stage {
    fn from(s: Stage) -> Self {
        match s {
            Stage::Amp => sdr_core::gain::Stage::Amp,
            Stage::Lna => sdr_core::gain::Stage::Lna,
            Stage::Vga => sdr_core::gain::Stage::Vga,
        }
    }
}

/// A shared stage name as this radio's, failing for one it does not have.
impl TryFrom<sdr_core::gain::Stage> for Stage {
    type Error = sdr_core::Error;

    fn try_from(s: sdr_core::gain::Stage) -> Result<Self, Self::Error> {
        match s {
            sdr_core::gain::Stage::Amp => Ok(Stage::Amp),
            sdr_core::gain::Stage::Lna => Ok(Stage::Lna),
            sdr_core::gain::Stage::Vga => Ok(Stage::Vga),
            other => Err(other.absent(crate::BACKEND)),
        }
    }
}

pub const LNA_MAX_DB: u16 = 40;
pub const LNA_STEP_DB: u16 = 8;
pub const VGA_MAX_DB: u16 = 62;
pub const VGA_STEP_DB: u16 = 2;
pub const TXVGA_MAX_DB: u16 = 47;
pub const AMP_DB: u16 = 14;

/// A receive stage's settable levels in dB, lowest first: the RF amp (off
/// / on), the LNA, the VGA.
pub fn stage_levels_db(stage: Stage) -> Vec<f64> {
    let steps = |max: u16, step: u16| (0..=max / step).map(|i| (i * step) as f64).collect();
    match stage {
        Stage::Amp => alloc::vec![0.0, AMP_DB as f64],
        Stage::Lna => steps(LNA_MAX_DB, LNA_STEP_DB),
        Stage::Vga => steps(VGA_MAX_DB, VGA_STEP_DB),
    }
}

/// An overall receive gain split across the LNA, then the VGA, each
/// snapped down to its step (the amp is not touched).
pub fn split(db: f64) -> (u16, u16) {
    let total = db.clamp(0.0, (LNA_MAX_DB + VGA_MAX_DB) as f64) as u16;
    let lna = lna_step(total);
    (lna, vga_step(total - lna))
}

/// An LNA gain snapped down to its step, within range.
pub fn lna_step(db: u16) -> u16 {
    db.min(LNA_MAX_DB) / LNA_STEP_DB * LNA_STEP_DB
}

/// A VGA gain snapped down to its step, within range.
pub fn vga_step(db: u16) -> u16 {
    db.min(VGA_MAX_DB) / VGA_STEP_DB * VGA_STEP_DB
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gain_fills_the_lna_first_in_its_steps() {
        assert_eq!(split(16.0), (16, 0));
        assert_eq!(split(45.0), (40, 4));
        assert_eq!(split(500.0), (40, 62));
        assert_eq!(split(-3.0), (0, 0));
        assert_eq!(stage_levels_db(Stage::Lna).len(), 6);
        assert_eq!(stage_levels_db(Stage::Vga).len(), 32);
        assert_eq!(stage_levels_db(Stage::Amp), [0.0, 14.0]);
    }

    /// The shared names convert both ways; the stages this radio lacks do
    /// not convert in.
    #[test]
    fn stages_convert_to_and_from_the_shared_names() {
        for s in Stage::ALL {
            assert_eq!(
                Stage::try_from(sdr_core::gain::Stage::from(s)).ok(),
                Some(s)
            );
        }
        assert!(Stage::try_from(sdr_core::gain::Stage::Mixer).is_err());
        assert!(Stage::try_from(sdr_core::gain::Stage::Pga).is_err());
    }
}
