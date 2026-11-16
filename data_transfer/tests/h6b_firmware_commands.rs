//! Exercises the actual MCU command encoder without hardware.
#![cfg(feature = "use-defmt")]
#[allow(dead_code, non_snake_case, unused_imports)]
#[path = "../../firmware/microcontroller/src/mlx90393/commands.rs"]
mod firmware_commands;
use firmware_commands::{Command, RunCommand};

#[test]
fn register_word_address_is_shifted_in_read_and_write_commands() {
    // Datasheet rev012 section 15.3.1.4–5: address is shifted left twice.
    // Neither word is asserted to be a temperature-reference register.
    assert_eq!(Command::read_register(0x18).write_command(), [0x50, 0x60]);
    assert_eq!(Command::read_register(0x24).write_command(), [0x50, 0x90]);
    assert_eq!(Command::write_register([0x12, 0x34], 0x18).write_command(),
               [0x60, 0x12, 0x34, 0x60]);
    assert_eq!(Command::write_register([0x12, 0x34], 0x24).write_command(),
               [0x60, 0x12, 0x34, 0x90]);
}