use bitflags::{bitflags, Flags};
use bitmatch::bitmatch;



pub struct Register<const R: u8> {
    data: [u8; 2],
}

impl<const R: u8> Register<R> {
    pub fn new(data: [u8; 2]) -> Self {
        Self { data }
    }

    pub fn bytes(&self) -> [u8; 2] {
        self.data
    }
}

impl Register<0x00> {
    fn flags(&self) -> RegisterOneFlags {
        RegisterOneFlags::from_bits_retain(u16::from_be_bytes(self.data))
    }

    pub fn zseries(&self) -> ZSeries {
        match self.flags().contains(RegisterOneFlags::ZSeries) {
            true => ZSeries::Enabled,
            false => ZSeries::Disabled,
        }
    }

    pub fn bist(&self) -> Bist {
        match self.flags().contains(RegisterOneFlags::Bist) {
            true => Bist::Enabled,
            false => Bist::Disabled,
        }
    }

    pub fn hall_conf(&self) -> Option<HallConf> {
        HallConf::from_u8_slice(&self.data)
    }

    pub fn gain(&self) -> Gain {
        Gain::from_u8_slice(&self.data)
    }
}

pub struct BurstSel {
    pub x: bool,
    pub y: bool,
    pub z: bool,
    pub temp: bool,
}

impl Register<0x01> {
    fn flags(&self) -> RegisterTwoFlags {
        RegisterTwoFlags::from_bits_retain(u16::from_be_bytes(self.data))
    }

    pub fn burst_sel(&self) -> BurstSel {
        let x = self.flags().contains(RegisterTwoFlags::BurstSelX);
        let y = self.flags().contains(RegisterTwoFlags::BurstSelY);
        let z = self.flags().contains(RegisterTwoFlags::BurstSelZ);
        let temp = self.flags().contains(RegisterTwoFlags::BurstSelT);
        BurstSel { x, y, z, temp }
    }

    pub fn temperature_compensation(&self) -> TemperatureCompensation {
        match self.flags().contains(RegisterTwoFlags::TcmpEn) {
            true => TemperatureCompensation::Enabled,
            false => TemperatureCompensation::Disabled,
        }
    }

    pub fn external_trigger(&self) -> bool {
        self.flags().contains(RegisterTwoFlags::ExtTrig)
    }

    pub fn wake_on_change_diff(&self) -> bool {
        self.flags().contains(RegisterTwoFlags::WOCDiff)
    }

    pub fn trigger_interrupt(&self) -> bool {
        self.flags().contains(RegisterTwoFlags::TrigInt)
    }
}

impl Register<0x02> {
    pub fn resolution(&self) -> Res3D {
        Res3D::from_u8_slice(&self.data)
    }

    /// Magnetic oversampling (OSR), register 0x02 bits 0..=1.
    pub fn oversampling(&self) -> u8 {
        (u16::from_be_bytes(self.data) & 0x0003) as u8
    }

    /// Digital filter (DIG_FILT), register 0x02 bits 2..=4.
    pub fn digital_filter(&self) -> u8 {
        ((u16::from_be_bytes(self.data) >> 2) & 0x0007) as u8
    }

    /// Temperature oversampling (OSR2), register 0x02 bits 11..=12.
    pub fn temperature_oversampling(&self) -> u8 {
        ((u16::from_be_bytes(self.data) >> 11) & 0x0003) as u8
    }

    pub fn magnetic_axis_conversion_time_micro(&self) -> u64 {
        let osr = self.oversampling();
        let dig_filt = self.digital_filter();
        67 + 64 * (1 << osr) * (2 + (1 << dig_filt))
    }

    pub fn temperature_conversion_time_micro(&self) -> u64 {
        let osr2 = self.temperature_oversampling();
        67 + 192 * (1 << osr2)
    }
}

impl Register<0x03> {
    pub fn temperature_offset(&self) -> TempOffset {
        TempOffset::from_u8_slice(&self.data)
    }
}

impl Register<0x24> {
    pub fn temperature_reference(&self) -> TempRef {
        TempRef::from_u8_slice(&self.data)
    }
}

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
pub struct TempOffset {
    pub offset: [u8; 2],
}
impl TempOffset {
    pub fn from_u8_slice(offset: &[u8; 2]) -> Self {
        Self { offset: *offset }
    }
}

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
pub struct TempRef {
    pub offset: [u8; 2],
}
impl TempRef {
    pub fn from_u8_slice(offset: &[u8; 2]) -> Self {
        Self { offset: *offset }
    }
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum ZSeries {
    Disabled,
    Enabled,
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum Bist {
    Disabled,
    Enabled,
}
bitflags! {
    pub struct RegisterOneFlags: u16 {
        const ZSeries = 0b0000_0000_1000_0000;
        const Bist = 0b0000_0001_0000_0000;
    }
}

bitflags! {
    pub struct RegisterTwoFlags: u16 {
        const TrigInt = 0b1000_0000_0000_0000;
        const WOCDiff = 0b0001_0000_0000_0000;
        const ExtTrig = 0b0000_1000_0000_0000;
        const TcmpEn = 0b0000_0100_0000_0000;
        const BurstSelZ = 0b0000_0010_0000_0000;
        const BurstSelY = 0b0000_0001_0000_0000;
        const BurstSelX = 0b0000_0000_1000_0000;
        const BurstSelT = 0b0000_0000_0100_0000;
    }
}

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum TemperatureCompensation {
    Disabled,
    Enabled,
}

impl TemperatureCompensation {
    #[bitmatch]
    pub fn from_u8_slice(val: &[u8; 2]) -> Self {
        #[bitmatch]
        let "????_?t??" = val[1];
        #[bitmatch]
        match t {
            "1" => Self::Enabled,
            "0" => Self::Disabled,
        }
    }
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum Gain {
    ZERO,
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX,
    SEVEN,
}
impl Gain {
    #[bitmatch]
    pub fn from_u8_slice(val: &[u8; 2]) -> Self {
        #[bitmatch]
        match val[1] {
            "?000_????" => Self::ZERO,
            "?001_????" => Self::ONE,
            "?010_????" => Self::TWO,
            "?011_????" => Self::THREE,
            "?100_????" => Self::FOUR,
            "?101_????" => Self::FIVE,
            "?110_????" => Self::SIX,
            "?111_????" => Self::SEVEN,
        }
    }
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum Resolution {
    BIT16,
    BIT17,
    BIT18,
    BIT19,
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
#[repr(usize)]
pub enum HallConf {
    TWOPHASE,
    FOURPHASE,
}

impl HallConf {
    #[bitmatch]
    pub fn from_u8_slice(val: &[u8; 2]) -> Option<Self> {
        #[bitmatch]
        match val[1] {
            "????_0000" => Some(Self::TWOPHASE),
            "????_1100" => Some(Self::FOURPHASE),
            _ => None,
        }
    }
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Clone, Copy)]
pub struct Res3D {
    pub x: Resolution,
    pub y: Resolution,
    pub z: Resolution,
}

impl Res3D {
    #[bitmatch]
    pub fn from_u8_slice(val: &[u8; 2]) -> Self {
        #[bitmatch]
        let "yxx?_????" = val[1];
        #[bitmatch]
        let "????_??zzv" = val[0];
        let xval = #[bitmatch]
        match x {
            "00" => Resolution::BIT16,
            "01" => Resolution::BIT17,
            "10" => Resolution::BIT18,
            "11" => Resolution::BIT19,
        };
        let yval = #[bitmatch]
        match v {
            "0" =>
            {
                #[bitmatch]
                match y {
                    "0" => Resolution::BIT16,
                    "1" => Resolution::BIT17,
                }
            }
            "1" =>
            {
                #[bitmatch]
                match y {
                    "0" => Resolution::BIT18,
                    "1" => Resolution::BIT19,
                }
            }
        };
        let zval = #[bitmatch]
        match z {
            "00" => Resolution::BIT16,
            "01" => Resolution::BIT17,
            "10" => Resolution::BIT18,
            "11" => Resolution::BIT19,
        };

        Self {
            x: xval,
            y: yval,
            z: zval,
        }
    }
}

#[repr(u32)]
pub enum CustomerMemoryArea {
    Hallconf,
    GainSel,
    ZSeries,
    Bist,
    AnaReservedLow,
    BurstDataRate,
    BurstSel,
    TcmpEn,
    ExtTrg,
    WocDiff,
    CommMode,
    TrigInt,
    OSR,
    DigFilt,
    ResX,
    ResY,
    ResZ,

    OSR2,
    SensTcLT,
    SensTcHT,
    OffsetX,
    OffsetY,
    OffsetZ,
    WOxyThreshold,
    WOzThreshold,
}

struct MemoryLocation {
    register: u8,
    position: usize,
    length: usize,
}

impl CustomerMemoryArea {
    fn to_memory_location(&self) -> MemoryLocation {
        match self {
            CustomerMemoryArea::Hallconf => MemoryLocation {
                register: 0x00,
                position: 0,
                length: 4,
            },
            CustomerMemoryArea::GainSel => MemoryLocation {
                register: 0x00,
                position: 4,
                length: 3,
            },
            CustomerMemoryArea::ZSeries => MemoryLocation {
                register: 0x00,
                position: 7,
                length: 1,
            },
            CustomerMemoryArea::Bist => MemoryLocation {
                register: 0x00,
                position: 8,
                length: 1,
            },
            CustomerMemoryArea::AnaReservedLow => MemoryLocation {
                register: 0x00,
                position: 9,
                length: 7,
            },
            CustomerMemoryArea::BurstDataRate => MemoryLocation {
                register: 0x01,
                position: 0,
                length: 6,
            },
            CustomerMemoryArea::BurstSel => MemoryLocation {
                register: 0x01,
                position: 6,
                length: 4,
            },
            CustomerMemoryArea::TcmpEn => MemoryLocation {
                register: 0x01,
                position: 10,
                length: 1,
            },
            CustomerMemoryArea::ExtTrg => MemoryLocation {
                register: 0x01,
                position: 11,
                length: 1,
            },
            CustomerMemoryArea::WocDiff => MemoryLocation {
                register: 0x01,
                position: 12,
                length: 1,
            },
            CustomerMemoryArea::CommMode => MemoryLocation {
                register: 0x01,
                position: 13,
                length: 2,
            },
            CustomerMemoryArea::TrigInt => MemoryLocation {
                register: 0x01,
                position: 15,
                length: 1,
            },
            CustomerMemoryArea::OSR => MemoryLocation {
                register: 0x02,
                position: 0,
                length: 2,
            },
            CustomerMemoryArea::DigFilt => MemoryLocation {
                register: 0x02,
                position: 2,
                length: 3,
            },
            CustomerMemoryArea::ResX => MemoryLocation {
                register: 0x02,
                position: 5,
                length: 2,
            },
            CustomerMemoryArea::ResY => MemoryLocation {
                register: 0x02,
                position: 7,
                length: 2,
            },
            CustomerMemoryArea::ResZ => MemoryLocation {
                register: 0x02,
                position: 9,
                length: 2,
            },
            CustomerMemoryArea::OSR2 => MemoryLocation {
                register: 0x02,
                position: 11,
                length: 2,
            },
            CustomerMemoryArea::SensTcLT => MemoryLocation {
                register: 0x03,
                position: 0,
                length: 8,
            },
            CustomerMemoryArea::SensTcHT => MemoryLocation {
                register: 0x03,
                position: 8,
                length: 8,
            },
            CustomerMemoryArea::OffsetX => MemoryLocation {
                register: 0x04,
                position: 0,
                length: 16,
            },
            CustomerMemoryArea::OffsetY => MemoryLocation {
                register: 0x05,
                position: 0,
                length: 16,
            },
            CustomerMemoryArea::OffsetZ => MemoryLocation {
                register: 0x06,
                position: 0,
                length: 16,
            },
            CustomerMemoryArea::WOxyThreshold => MemoryLocation {
                register: 0x07,
                position: 0,
                length: 16,
            },
            CustomerMemoryArea::WOzThreshold => MemoryLocation {
                register: 0x08,
                position: 0,
                length: 16,
            },
        }
    }
}

impl Gain {
    pub fn as_u8(self) -> u8 {
        self as u8
    }
}

impl Resolution {
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    pub fn bit_depth(self) -> u8 {
        16 + self.as_u8()
    }
}

impl HallConf {
    pub fn as_raw_u8(self) -> u8 {
        match self {
            HallConf::TWOPHASE => 0x0,
            HallConf::FOURPHASE => 0xC,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Register;

    #[test]
    fn register_02_parses_old_pi_osr_filter_from_low_bits() {
        // OSR=2 (bits 0..1), DIG_FILT=4 (bits 2..4), all resolutions=0.
        let reg = Register::<0x02>::new(0x0012_u16.to_be_bytes());

        assert_eq!(reg.oversampling(), 2);
        assert_eq!(reg.digital_filter(), 4);
        assert_eq!(reg.magnetic_axis_conversion_time_micro(), 4675);
    }

    #[test]
    fn register_02_temperature_osr_uses_bits_11_through_12() {
        let reg = Register::<0x02>::new(0x1800_u16.to_be_bytes());

        assert_eq!(reg.temperature_oversampling(), 3);
        assert_eq!(reg.temperature_conversion_time_micro(), 1603);
    }

    #[test]
    fn resolution_raw_values_match_documented_bit_depths() {
        // All resolution fields zero.
        let res0 = Register::<0x02>::new(0x0000_u16.to_be_bytes()).resolution();
        assert_eq!(res0.x.as_u8(), 0);
        assert_eq!(res0.x.bit_depth(), 16);
        assert_eq!(res0.y.bit_depth(), 16);
        assert_eq!(res0.z.bit_depth(), 16);

        // All X/Y/Z resolution fields set to raw value 3.
        let res3 = Register::<0x02>::new(0x07E0_u16.to_be_bytes()).resolution();
        assert_eq!(res3.x.as_u8(), 3);
        assert_eq!(res3.x.bit_depth(), 19);
        assert_eq!(res3.y.bit_depth(), 19);
        assert_eq!(res3.z.bit_depth(), 19);
    }
}
