use bitflags::{bitflags, Flags};
use bitmatch::bitmatch;
use defmt::Format;

//use crate::mlx90393::CustomerMemoryArea;

pub struct Register {
    register: u8,
}

#[derive(Clone, Copy, Format)]
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

bitflags! {
    pub struct MagneticFieldReturnFlags: u8 {
        const X = 0b00000010;
        const Y = 0b00000100;
        const Z = 0b00001000;
        const T = 0b00000001;
    }
}

pub struct SB {
    axes: MagneticFieldReturnFlags,
}
pub struct SW {
    axes: MagneticFieldReturnFlags,
}
pub struct SM {
    axes: MagneticFieldReturnFlags,
}
pub struct RM<const X: bool, const Y: bool, const Z: bool, const TEMP: bool> {
    axes: MagneticFieldReturnFlags,
}
pub struct RR {
    location: u8,
}
pub struct WR {
    D: [u8; 2],
    location: u8,
}
pub struct EX;
pub struct HR;
pub struct HS;
pub struct RT;

pub struct CommandData<T> {
    pub command: T,
}

pub trait RunCommand<T, const M: usize, const N: usize> {
    fn write_command(&self) -> [u8; M];
    fn read_buffer(&self) -> [u8; N] {
        [0; N]
    }
}

impl RunCommand<SB, 1, 1> for CommandData<SB> {
    fn write_command(&self) -> [u8; 1] {
        [0b00010000 + self.command.axes.bits()]
    }
}

impl RunCommand<SW, 1, 1> for CommandData<SW> {
    fn write_command(&self) -> [u8; 1] {
        [0b00100000 + self.command.axes.bits()]
    }
}

impl RunCommand<SM, 1, 1> for CommandData<SM> {
    fn write_command(&self) -> [u8; 1] {
        [0b00110000 + self.command.axes.bits()]
    }
}

impl RunCommand<EX, 1, 1> for CommandData<EX> {
    fn write_command(&self) -> [u8; 1] {
        [0b10000000]
    }
}

impl RunCommand<RT, 1, 1> for CommandData<RT> {
    fn write_command(&self) -> [u8; 1] {
        [0b11110000]
    }
}

impl RunCommand<RR, 2, 3> for CommandData<RR> {
    fn write_command(&self) -> [u8; 2] {
        [0b01010000, (self.command.location << 2)]
    }
}

// firmware/microcontroller/src/mlx90393/commands.rs

impl RunCommand<WR, 4, 1> for CommandData<WR> {
    fn write_command(&self) -> [u8; 4] {
        [
            0b0110_0000,
            self.command.D[0],
            self.command.D[1],
            self.command.location << 2,
        ]
    }
}

impl RunCommand<HR, 1, 1> for CommandData<HR> {
    fn write_command(&self) -> [u8; 1] {
        [0b11010000]
    }
}

impl RunCommand<HS, 1, 1> for CommandData<HS> {
    fn write_command(&self) -> [u8; 1] {
        [0b11100000]
    }
}

impl RunCommand<RM<true, true, true, true>, 1, 9> for CommandData<RM<true, true, true, true>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, true, true, true>, 1, 7> for CommandData<RM<false, true, true, true>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<true, false, true, true>, 1, 7> for CommandData<RM<true, false, true, true>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<true, true, false, true>, 1, 7> for CommandData<RM<true, true, false, true>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<true, true, true, false>, 1, 7> for CommandData<RM<true, true, true, false>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, false, true, true>, 1, 5> for CommandData<RM<false, false, true, true>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, true, false, true>, 1, 5> for CommandData<RM<false, true, false, true>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, true, true, false>, 1, 5> for CommandData<RM<false, true, true, false>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<true, false, false, true>, 1, 5> for CommandData<RM<true, false, false, true>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<true, false, true, false>, 1, 5> for CommandData<RM<true, false, true, false>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<true, true, false, false>, 1, 5> for CommandData<RM<true, true, false, false>> {
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<true, false, false, false>, 1, 3>
    for CommandData<RM<true, false, false, false>>
{
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, true, false, false>, 1, 3>
    for CommandData<RM<false, true, false, false>>
{
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, false, true, false>, 1, 3>
    for CommandData<RM<false, false, true, false>>
{
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, false, false, true>, 1, 3>
    for CommandData<RM<false, false, false, true>>
{
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

impl RunCommand<RM<false, false, false, false>, 1, 1>
    for CommandData<RM<false, false, false, false>>
{
    fn write_command(&self) -> [u8; 1] {
        [0b01000000 + self.command.axes.bits()]
    }
}

pub struct Command;

impl Command {
    pub fn start_burst<const X: bool, const Y: bool, const Z: bool, const TEMP: bool>(
    ) -> CommandData<SB> {
        let flags = const {
            let x_flag = match X {
                true => MagneticFieldReturnFlags::X,
                false => MagneticFieldReturnFlags::empty(),
            };
            let y_flag = match Y {
                true => MagneticFieldReturnFlags::Y,
                false => MagneticFieldReturnFlags::empty(),
            };
            let z_flag = match Z {
                true => MagneticFieldReturnFlags::Z,
                false => MagneticFieldReturnFlags::empty(),
            };
            let t_flag = match TEMP {
                true => MagneticFieldReturnFlags::T,
                false => MagneticFieldReturnFlags::empty(),
            };
            (x_flag, y_flag, z_flag, t_flag)
        };

        CommandData {
            command: SB {
                axes: flags.0 | flags.1 | flags.2 | flags.3,
            },
        }
    }
    pub fn start_wake_on_change<const X: bool, const Y: bool, const Z: bool, const TEMP: bool>(
    ) -> CommandData<SW> {
        let flags = const {
            let x_flag = match X {
                true => MagneticFieldReturnFlags::X,
                false => MagneticFieldReturnFlags::empty(),
            };
            let y_flag = match Y {
                true => MagneticFieldReturnFlags::Y,
                false => MagneticFieldReturnFlags::empty(),
            };
            let z_flag = match Z {
                true => MagneticFieldReturnFlags::Z,
                false => MagneticFieldReturnFlags::empty(),
            };
            let t_flag = match TEMP {
                true => MagneticFieldReturnFlags::T,
                false => MagneticFieldReturnFlags::empty(),
            };
            (x_flag, y_flag, z_flag, t_flag)
        };

        CommandData {
            command: SW {
                axes: flags.0 | flags.1 | flags.2 | flags.3,
            },
        }
    }
    pub fn single_measurement<const X: bool, const Y: bool, const Z: bool, const TEMP: bool>(
    ) -> CommandData<SM> {
        let flags = const {
            let x_flag = match X {
                true => MagneticFieldReturnFlags::X,
                false => MagneticFieldReturnFlags::empty(),
            };
            let y_flag = match Y {
                true => MagneticFieldReturnFlags::Y,
                false => MagneticFieldReturnFlags::empty(),
            };
            let z_flag = match Z {
                true => MagneticFieldReturnFlags::Z,
                false => MagneticFieldReturnFlags::empty(),
            };
            let t_flag = match TEMP {
                true => MagneticFieldReturnFlags::T,
                false => MagneticFieldReturnFlags::empty(),
            };
            (x_flag, y_flag, z_flag, t_flag)
        };

        CommandData {
            command: SM {
                axes: flags.0 | flags.1 | flags.2 | flags.3,
            },
        }
    }
    pub fn read_measurement<const X: bool, const Y: bool, const Z: bool, const TEMP: bool>(
    ) -> CommandData<RM<X, Y, Z, TEMP>> {
        let flags = const {
            let x_flag = match X {
                true => MagneticFieldReturnFlags::X,
                false => MagneticFieldReturnFlags::empty(),
            };
            let y_flag = match Y {
                true => MagneticFieldReturnFlags::Y,
                false => MagneticFieldReturnFlags::empty(),
            };
            let z_flag = match Z {
                true => MagneticFieldReturnFlags::Z,
                false => MagneticFieldReturnFlags::empty(),
            };
            let t_flag = match TEMP {
                true => MagneticFieldReturnFlags::T,
                false => MagneticFieldReturnFlags::empty(),
            };
            (x_flag, y_flag, z_flag, t_flag)
        };

        CommandData {
            command: RM {
                axes: flags.0 | flags.1 | flags.2 | flags.3,
            },
        }
    }
    pub fn read_register(location: u8) -> CommandData<RR> {
        CommandData {
            command: RR { location },
        }
    }
    pub fn write_register(data: [u8; 2], location: u8) -> CommandData<WR> {
        CommandData {
            command: WR { D: data, location },
        }
    }
    pub fn exit() -> CommandData<EX> {
        CommandData { command: EX }
    }
    pub fn memory_recall() -> CommandData<HR> {
        todo!()
    }
    pub fn memory_store() -> CommandData<HS> {
        todo!()
    }
    pub fn reset() -> CommandData<RT> {
        CommandData { command: RT }
    }
}
