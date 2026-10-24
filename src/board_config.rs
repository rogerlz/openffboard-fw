//! OpenFFBoard-specific TMC4671 hardware configuration constants.

use crate::ankyra::enumerations::Pins;

/// USB product string; trace builds are told apart by their vendor-specific interface.
pub(crate) const USB_PRODUCT: &str = "FOCI OpenFFBoard";

pub(crate) const TMC_CHANNEL_COUNT: u8 = 1;

/// OpenFFBoard v1.3 VM sense divider high-side resistor (R23).
pub(crate) const VM_DIVIDER_HIGH_OHMS: u32 = 71_500;

/// OpenFFBoard v1.3 VM sense divider low-side resistor (R24).
pub(crate) const VM_DIVIDER_LOW_OHMS: u32 = 1_500;

/// TMC4671 ADC_VM reference span in millivolts.
pub(crate) const VM_ADC_REFERENCE_MILLIVOLTS: u32 = 2_500;

/// Centered positive ADC_VM count span for the TMC4671 16-bit raw ADC.
pub(crate) const VM_ADC_CENTER_COUNTS: u32 = 32_767;

pub(crate) fn step_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::STEP0) => Some(0),
        _ => None,
    }
}

pub(crate) fn dir_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::DIR0) => Some(0),
        _ => None,
    }
}

pub(crate) fn enable_pin_channel(pin: u32) -> Option<u8> {
    let pin = u8::try_from(pin).ok()?;
    match Pins::try_from(pin) {
        Ok(Pins::ENA0) => Some(0),
        _ => None,
    }
}

/// PWM at 25 kHz: PWM_MAXCNT=3999 with fCLK=25 MHz (TMC4671 datasheet 4.9.3).
const OPENFFBOARD_V13_PWM_MAXCNT: u16 = 3999;
const OPENFFBOARD_V13_DSADC_MDEC: u16 =
    foci_firmware::config::dsadc_mdec_from_pwm_maxcnt(OPENFFBOARD_V13_PWM_MAXCNT);

const STEPPER_TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
    pwm_maxcnt: OPENFFBOARD_V13_PWM_MAXCNT,
    tmc_clock_hz: 25_000_000,
    bbm_h: 9,
    bbm_l: 9,
    dsadc_mdec_a: OPENFFBOARD_V13_DSADC_MDEC,
    dsadc_mdec_b: OPENFFBOARD_V13_DSADC_MDEC,
    // OpenFFBoard v1.3 ACS724 routing: UX=I0, WY=I1, V=I2.
    adc_i_select: foci_tmc4671::registers::AdcISelect::new()
        .with_i0_select(0)
        .with_i1_select(1)
        .with_ux_select(0)
        .with_v_select(2)
        .with_wy_select(1)
        .into_bits(),
    excitation_voltage: 1024,
    electrical_model_theta_e_us: 160,
    current_validation_flux_max_encoder_delta_counts: 4,
    current_validation_torque_max_encoder_delta_counts: 16,
    resistance_max_expected_current_count: 1600,
    resistance_voltage_scale: 1,
    resistance_settle_ms: 500,
    resistance_samples_per_window: 8,
    resistance_sample_interval_ms: 5,
    inductance_voltage: 768,
    resistance_scale_metadata_validated: true,
    current_scale_ma_per_lsb: 1.155,
    max_run_current_ma: 20_000,
    vm_divider_high_ohms: VM_DIVIDER_HIGH_OHMS,
    vm_divider_low_ohms: VM_DIVIDER_LOW_OHMS,
    vm_adc_reference_millivolts: VM_ADC_REFERENCE_MILLIVOLTS,
    vm_adc_center_counts: VM_ADC_CENTER_COUNTS,
    motor_type_n_pole_pairs: foci_firmware::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
};

#[cfg(not(feature = "hfkp-encoder"))]
pub(crate) const TMC_CONFIG: foci_firmware::config::TmcConfig = STEPPER_TMC_CONFIG;

/// Mitsubishi HF-KP053 servo: a three-phase PMSM with 3 pole pairs, ~18 ohm
/// per phase and Ld/Lq ~21/31 mH (a 200 V-class winding on 24 V), fed by
/// its 18-bit encoder (`encoder_ppr: 65536`, 262144 counts per revolution).
#[cfg(feature = "hfkp-encoder")]
pub(crate) const TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
    motor_type_n_pole_pairs: foci_tmc4671::registers::MotorTypeNPolePairs::new()
        .with_motor_type(foci_tmc4671::registers::MotorType::ThreePhase)
        .with_n_pole_pairs(3)
        .into_bits(),
    // The inductance capture rotates its drive at 1 kHz, where the winding
    // presents ~130-200 ohm, and needs ~9 V to reach its 30-count
    // quadrature-current floor. The excitation voltage caps the
    // commissioning drive, so it allows that too; the coil check then
    // drives ~0.58 A, inside its envelope.
    excitation_voltage: 12288,
    inductance_voltage: 12288,
    // The stock ladder barely moves current through this winding; 5x spans
    // ~0.23-0.68 A of d-axis current at 24 V. The ~0.75 A ceiling bounds
    // the ladder, sets its linear threshold (~0.35 A) and the gain-floor
    // torque steps (60/70/80% of it): larger steps spin the unloaded rotor
    // fast enough in 5 ms that back-EMF at 24 V eats the current.
    resistance_voltage_scale: 5,
    resistance_max_expected_current_count: 650,
    // The rotor holds still under the rotor-frame excitation, so only the
    // ~0.7 ms electrical time constant has to settle; short settles keep
    // the winding from warming past the 5% drift limit. At ~350 counts, 40
    // samples 2 ms apart keep the window noise inside it too.
    resistance_settle_ms: 100,
    resistance_samples_per_window: 40,
    resistance_sample_interval_ms: 2,
    // The stepper limits (0.36 and 1.44 degrees) rely on detent torque. The
    // unloaded rotor turns freely, so a few electrical degrees of alignment
    // error or a 2 ms torque step move it a degree or more; allow 5 and 10
    // degrees. A misaligned current loop still spins it far past that
    // within the 100 ms flux hold.
    current_validation_flux_max_encoder_delta_counts: 3641,
    current_validation_torque_max_encoder_delta_counts: 7282,
    ..STEPPER_TMC_CONFIG
};

const OPENFFBOARD_V13_BBM_REGISTER: u32 = 0x0909;
const _: () = assert!(TMC_CONFIG.bbm_register() == OPENFFBOARD_V13_BBM_REGISTER);
const OPENFFBOARD_V13_DSADC_MDEC_REGISTER: u32 =
    ((OPENFFBOARD_V13_DSADC_MDEC as u32) << 16) | OPENFFBOARD_V13_DSADC_MDEC as u32;
const _: () = assert!(TMC_CONFIG.dsadc_mdec_register() == OPENFFBOARD_V13_DSADC_MDEC_REGISTER);
const OPENFFBOARD_V13_EXCITATION_VOLTAGE: u16 = 1024;
const _: () = assert!(STEPPER_TMC_CONFIG.excitation_voltage == OPENFFBOARD_V13_EXCITATION_VOLTAGE);

pub(crate) const FEEDBACK_CONFIGS: [foci_firmware::config::FeedbackConfig;
    TMC_CHANNEL_COUNT as usize] = [foci_firmware::config::FeedbackConfig {
    phi_e_source: foci_firmware::config::FeedbackSource::Abn,
    position_source: foci_firmware::config::FeedbackSource::Abn,
}; TMC_CHANNEL_COUNT as usize];

/// The internal temperature sensor is wired to ADC1_IN16 only (RM0090).
pub(crate) type InternalTempAdcInstance = embassy_stm32::peripherals::ADC1;

pub(crate) fn analog_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::MCU_TEMP) => Some(0),
        _ => None,
    }
}

/// Factory calibration addresses (RM0090 device electronic signature): `TS_CAL1` at 30C,
/// `TS_CAL2` at 110C, both at 3.3V VDDA.
const TS_CAL1_ADDR: *const u16 = 0x1FFF_7A2C as *const u16;
const TS_CAL2_ADDR: *const u16 = 0x1FFF_7A2E as *const u16;
const TS_CAL1_TEMP_C: f32 = 30.0;
const TS_CAL2_TEMP_C: f32 = 110.0;

pub(crate) fn read_mcu_temp_calibration() -> foci_firmware::mcu_temp::FactoryCalibration {
    // SAFETY: TS_CAL1_ADDR/TS_CAL2_ADDR are ST's documented factory
    // calibration addresses in system memory, present and readable on
    // every STM32F407 device.
    let cal1 = unsafe { core::ptr::read_volatile(TS_CAL1_ADDR) };
    let cal2 = unsafe { core::ptr::read_volatile(TS_CAL2_ADDR) };
    foci_firmware::mcu_temp::FactoryCalibration {
        cal_low: (TS_CAL1_TEMP_C, cal1),
        cal_high: (TS_CAL2_TEMP_C, cal2),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analog_pin_channel_resolves_mcu_temp_only() {
        assert_eq!(analog_pin_channel(Pins::MCU_TEMP as u8), Some(0));
        assert_eq!(analog_pin_channel(Pins::STEP0 as u8), None);
    }
}
