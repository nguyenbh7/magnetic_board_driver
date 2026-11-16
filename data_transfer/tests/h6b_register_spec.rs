//! Synthetic MLX90393 checks; datasheet rev012 fig.23, tables 17 and 21.
//! No device, private measurements, file writes or calibration approval.
use data_transfer::conversions::{Axis, MagneticValue, nominal_symmetric_mlx_half_ranges_mt};
use data_transfer::memory::{Gain, HallConf, Register, Resolution, TemperatureCompensation};
use data_transfer::rpc::{BoardFrame, SENSOR_POSITIONS_MM, SENSORS_PER_BOARD};

#[test]
fn documented_register_zero_masks_and_big_endian_round_trip() {
    for gain in 0..8_u16 {
        let word = 0x018c | (gain << 4);
        let reg = Register::<0>::new(word.to_be_bytes());
        assert_eq!(reg.bytes(), word.to_be_bytes());
        assert_eq!(reg.gain() as u16, gain);
        assert_eq!(reg.hall_conf().unwrap() as usize, HallConf::FOURPHASE as usize);
        assert_eq!(reg.zseries() as usize, 1);
        assert_eq!(reg.bist() as usize, 1);
    }
    assert!(Register::<0>::new([0, 1]).hall_conf().is_none());
}

#[test]
fn documented_register_one_temperature_compensation_bit() {
    assert_eq!(Register::<1>::new(0x0400_u16.to_be_bytes()).temperature_compensation() as usize,
               TemperatureCompensation::Enabled as usize);
    assert_eq!(Register::<1>::new(0x0004_u16.to_be_bytes()).temperature_compensation() as usize,
               TemperatureCompensation::Disabled as usize);
}

#[test]
fn standalone_temperature_compensation_decoder_uses_documented_bit_ten() {
    // Separate API from Register<1>::temperature_compensation; do not infer usage.
    assert_eq!(TemperatureCompensation::from_u8_slice(&[0x04, 0x00]) as usize,
               TemperatureCompensation::Enabled as usize);
    assert_eq!(TemperatureCompensation::from_u8_slice(&[0x00, 0x04]) as usize,
               TemperatureCompensation::Disabled as usize);
}

#[test]
fn resolution_axes_decode_all_64_combinations() {
    for x in 0..4_u16 { for y in 0..4_u16 { for z in 0..4_u16 {
        let reg = Register::<2>::new(((x << 5) | (y << 7) | (z << 9)).to_be_bytes());
        let v = reg.resolution();
        assert_eq!((v.x as u16, v.y as u16, v.z as u16), (x, y, z));
    }}}
}

fn ut(raw: u16, tc: TemperatureCompensation, res: Resolution) -> Option<f64> {
    MagneticValue::from_bits(Some(&raw.to_be_bytes()), tc, Gain::SEVEN, res,
                             HallConf::FOURPHASE, Axis::X).map(|v| v.value())
}
#[test]
fn datasheet_table21_signed_unsigned_and_unsupported_modes() {
    use Resolution::*;
    use TemperatureCompensation::*;
    assert_eq!(ut(0x0000, Disabled, BIT19), Some(0.0));
    assert!((ut(0xffff, Disabled, BIT19).unwrap() + 0.150).abs() < 1e-12);
    assert_eq!(ut(0x8000, Disabled, BIT17), Some(0.0));
    assert_eq!(ut(0x4000, Disabled, BIT16), Some(0.0));
    assert_eq!(ut(0x8000, Enabled, BIT19), Some(0.0));
    assert!(ut(0x8000, Enabled, BIT17).is_none());
    assert!(ut(0x4000, Enabled, BIT16).is_none());
}
#[test]
fn nominal_range_is_digital_screen_not_saturation_proof() {
    let v = nominal_symmetric_mlx_half_ranges_mt(7, 0, 0xC).unwrap();
    assert!((v[0] - 4.9152).abs() < 1e-10);
    assert!((v[2] - 7.929856).abs() < 1e-10);
    assert!(nominal_symmetric_mlx_half_ranges_mt(7, 2, 0xC).is_none());
}
#[test]
fn repository_board_channel_order_and_frame_roundtrip() {
    // Project convention only; not a physical mapping certification.
    assert_eq!(SENSORS_PER_BOARD, 16);
    assert_eq!(SENSOR_POSITIONS_MM[0], (-6.75, -6.75, 0.0));
    assert_eq!(SENSOR_POSITIONS_MM[15], (6.75, 6.75, 0.0));
    let mut frame = BoardFrame::default();
    frame.board_id = 2; frame.frame_id = 17; frame.sensor_mask = 0x8001;
    frame.samples[0].bx_ut = -42.0; frame.samples[15].bz_ut = 13.0;
    let mut buf = [0_u8; 2048];
    let enc = postcard::to_slice(&frame, &mut buf).unwrap();
    let decoded: BoardFrame = postcard::from_bytes(enc).unwrap();
    assert_eq!(decoded, frame);
}
#[test]
fn standalone_decoder_matches_register_accessor_for_every_register_word() {
    // Exhaustively check both settings, noisy unrelated bits, and boundaries.
    for word in 0_u16..=u16::MAX {
        let bytes = word.to_be_bytes();
        let got = TemperatureCompensation::from_u8_slice(&bytes) as usize;
        let expected = usize::from(word & 0x0400 != 0);
        assert_eq!(got, expected, "word {word:04x}");
        assert_eq!(got, Register::<1>::new(bytes).temperature_compensation() as usize);
    }
}