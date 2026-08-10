use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use postcard_schema::Schema;

use crate::memory::{
    Gain, HallConf, Res3D, Resolution, TempOffset, TempRef, TemperatureCompensation,
};


pub struct MagneticBits {
    pub x: Option<[u8; 2]>,
    pub y: Option<[u8; 2]>,
    pub z: Option<[u8; 2]>,
    pub temp: Option<[u8; 2]>,
}

impl MagneticBits {
    pub fn new(
        x: Option<[u8; 2]>,
        y: Option<[u8; 2]>,
        z: Option<[u8; 2]>,
        temp: Option<[u8; 2]>,
    ) -> Self {
        Self { x, y, z, temp }
    }
}

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum Axis {
    X,
    Y,
    Z,
}

struct SensitivityPerBit {
    value: f64,
}

// Taken from the datasheet. Indices are [Gain][raw resolution 0..3][Axis].
const SENSITIVITY: [[[f64; 3]; 4]; 8] = [
    [
        [0.751, 0.751, 1.210],
        [1.502, 1.502, 2.420],
        [3.004, 3.004, 4.840],
        [6.009, 6.009, 9.680],
    ],
    [
        [0.601, 0.601, 0.968],
        [1.202, 1.202, 1.936],
        [2.403, 2.403, 3.872],
        [4.840, 4.840, 7.744],
    ],
    [
        [0.451, 0.451, 0.726],
        [0.901, 0.901, 1.452],
        [1.803, 1.803, 2.904],
        [3.605, 3.605, 5.808],
    ],
    [
        [0.376, 0.376, 0.605],
        [0.751, 0.751, 1.210],
        [1.502, 1.502, 2.420],
        [3.004, 3.004, 4.840],
    ],
    [
        [0.300, 0.300, 0.484],
        [0.601, 0.601, 0.968],
        [1.202, 1.202, 1.936],
        [2.403, 2.403, 3.872],
    ],
    [
        [0.250, 0.250, 0.403],
        [0.501, 0.501, 0.807],
        [1.001, 1.001, 1.613],
        [2.003, 2.003, 3.227],
    ],
    [
        [0.200, 0.200, 0.323],
        [0.401, 0.401, 0.645],
        [0.801, 0.801, 1.291],
        [1.602, 1.602, 2.581],
    ],
    [
        [0.150, 0.150, 0.242],
        [0.300, 0.300, 0.484],
        [0.601, 0.601, 0.968],
        [1.202, 1.202, 1.936],
    ],
];

impl SensitivityPerBit {
    pub const fn new(axis: Axis, gain: Gain, resolution: Resolution, hallconf: HallConf) -> Self {
        let hall_conf_multiplier = match hallconf {
            HallConf::TWOPHASE => 98.0 / 75.0,
            HallConf::FOURPHASE => 1.0,
        };
        Self {
            value: hall_conf_multiplier
                * SENSITIVITY[gain as usize][resolution as usize][axis as usize],
        }
    }
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Schema, Clone, Copy,  Debug, MaxSize, PartialEq)]
pub enum TempValue {
    Celsius(f64),
}
impl TempValue {
    fn from_bits(register: &[u8; 2], offset: TempOffset) -> Self {
        let t = u16::from_be_bytes(*register) as f64;
        let offset = u16::from_be_bytes(offset.offset) as f64;

        Self::Celsius(35.0 + (t - offset) / 45.2)
    }

    fn from_option_bits(register: Option<&[u8; 2]>, offset: TempRef) -> Option<Self> {
        let t = u16::from_be_bytes(*register?) as f64;
        let offset = u16::from_be_bytes(offset.offset) as f64;

        Some(Self::Celsius(35.0 + (t - offset) / 45.2))
    }

    pub fn value(&self) -> f64 {
        match self {
            Self::Celsius(val) => *val,
        }
    }
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, PartialEq, Schema, Clone, Copy,  Debug, Default, MaxSize)]
pub struct MagneticField {
    pub x: Option<MagneticValue>,
    pub y: Option<MagneticValue>,
    pub z: Option<MagneticValue>,
    pub t: Option<TempValue>,
}

impl MagneticField {
    pub fn from_bits(
        x: Option<&[u8; 2]>,
        y: Option<&[u8; 2]>,
        z: Option<&[u8; 2]>,
        temp: Option<&[u8; 2]>,
        temp_offset: TempRef,
        temp_comp: TemperatureCompensation,
        gain: Gain,
        resolution: Res3D,
        hallconf: HallConf,
    ) -> Option<Self> {
        let x_field = MagneticValue::from_bits(x, temp_comp, gain, resolution.x, hallconf, Axis::X);
        let y_field = MagneticValue::from_bits(y, temp_comp, gain, resolution.y, hallconf, Axis::Y);
        let z_field = MagneticValue::from_bits(z, temp_comp, gain, resolution.z, hallconf, Axis::Z);
        let temp = TempValue::from_option_bits(temp, temp_offset);
        Some(Self {
            x: x_field,
            y: y_field,
            z: z_field,
            t: temp,
        })
    }

    pub fn from_mbits(
        mbits: MagneticBits,
        temp_offset: TempRef,
        temp_comp: TemperatureCompensation,
        gain: Gain,
        resolution: Res3D,
        hallconf: HallConf,
    ) -> Option<Self> {
        Self::from_bits(
            mbits.x.as_ref(),
            mbits.y.as_ref(),
            mbits.z.as_ref(),
            mbits.temp.as_ref(),
            temp_offset,
            temp_comp,
            gain,
            resolution,
            hallconf,
        )
    }
}

#[derive(Debug)]
pub enum Error {
    FailedCRCSerialization,
    FailedCOBSSerialization,
    FailedWrite,
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Serialize, Deserialize, Schema, PartialEq, Clone, Copy,  Debug, MaxSize)]
#[repr(usize)]
pub enum MagneticValue {
    uT(f64),
}

impl MagneticValue {
    pub fn from_bits(
        value: Option<&[u8; 2]>,
        temp_comp: TemperatureCompensation,
        gain: Gain,
        resolution: Resolution,
        hallconf: HallConf,
        axis: Axis,
    ) -> Option<Self> {
        let raw = *value?;

        let counts: i32 = match (temp_comp, resolution) {
            // TCMP disabled, raw RES=0/1 (16/17-bit labels):
            // Datasheet: two's complement, 0 uT = 0 LSB.
            (TemperatureCompensation::Disabled, Resolution::BIT16)
            | (TemperatureCompensation::Disabled, Resolution::BIT17) => {
                i16::from_be_bytes(raw) as i32
            }

            // TCMP disabled, raw RES=2 (18-bit label):
            // Datasheet: unsigned, 0 uT = 2^15 LSB.
            (TemperatureCompensation::Disabled, Resolution::BIT18) => {
                u16::from_be_bytes(raw) as i32 - 32768
            }

            // TCMP disabled, raw RES=3 (19-bit label):
            // Datasheet: unsigned, 0 uT = 2^14 LSB.
            (TemperatureCompensation::Disabled, Resolution::BIT19) => {
                u16::from_be_bytes(raw) as i32 - 16384
            }

            // TCMP enabled, raw RES=0/1:
            // Datasheet: unsigned, 0 uT = 2^15 LSB.
            (TemperatureCompensation::Enabled, Resolution::BIT16)
            | (TemperatureCompensation::Enabled, Resolution::BIT17) => {
                u16::from_be_bytes(raw) as i32 - 32768
            }

            // Datasheet marks TCMP enabled with raw RES=2/3 as N/A.
            (TemperatureCompensation::Enabled, Resolution::BIT18)
            | (TemperatureCompensation::Enabled, Resolution::BIT19) => {
                return None;
            }
        };

        let sensitivity = SensitivityPerBit::new(axis, gain, resolution, hallconf);

        Some(Self::uT(sensitivity.value * f64::from(counts)))
    }
    pub fn value(&self) -> f64 {
        match self {
            MagneticValue::uT(val) => *val,
        }
    }
}
