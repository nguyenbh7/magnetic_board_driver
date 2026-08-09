# WSL firmware flash and desktop-app workflow

This document records the normal development workflow for `magnetic_board_driver` when the Rust toolchain, STM32 firmware flashing, and desktop app are run from WSL.

It is written against branch `agent-live-fit-fixes`, but the directory/command structure comes from the repository itself and should remain applicable to related development branches unless their Cargo configuration changes.

## Repository layout relevant to running the system

```text
magnetic_board_driver/
├── Cargo.toml                 # host workspace: app, data_transfer, magnetic-data-parser
├── app/                       # desktop GUI / serial RPC client
├── data_transfer/             # shared wire types and conversions
└── firmware/
    ├── Cargo.toml             # embedded workspace
    ├── .cargo/config.toml     # target + probe-rs runner + linker flags
    └── microcontroller/       # STM32WBA55CG firmware crate
```

The root workspace contains the desktop `app`. The `firmware/` directory is a separate Cargo workspace whose only member is `microcontroller`.

## 1. Select/update the development branch

From the repository root:

```bash
git fetch origin
git switch agent-live-fit-fixes
git pull --ff-only origin agent-live-fit-fixes
```

## 2. Optional software checks before connecting hardware

From the repository root, run the host-side shared-code tests:

```bash
cargo test -p data_transfer
```

Then check the firmware using the firmware workspace's own target/linker configuration:

```bash
cd firmware
cargo check
```

Do not normally add an explicit `--target` here. `firmware/.cargo/config.toml` already sets:

```text
thumbv8m.main-none-eabi
```

and supplies the embedded linker flags used by the real firmware run.

## 3. Make the hardware visible inside WSL

For flashing, the debug probe must be visible to WSL. For running the desktop app, the controller's serial interface must also be visible to WSL.

Useful checks are:

```bash
probe-rs list
lsusb
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

If the devices are already listed, no additional USB forwarding step is needed. If they are visible in Windows but not WSL, attach/pass the corresponding USB device(s) through to WSL using the Windows/WSL USB setup used on this machine before continuing.

## 4. Build, flash, and monitor the STM32 firmware

From the repository root:

```bash
cd firmware
cargo run
```

No normal command-line modifier is required. `firmware/.cargo/config.toml` defines:

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "probe-rs run --chip STM32WBA55CG"

[build]
target = "thumbv8m.main-none-eabi"

[env]
DEFMT_LOG = "debug"
```

Therefore `cargo run` from `firmware/` does all of the following:

1. Builds the `microcontroller` firmware for the configured Cortex-M target.
2. Runs the configured `probe-rs` runner.
3. Programs the STM32WBA55CG.
4. Attaches to defmt output at debug level.

For normal development, leave this terminal open so firmware/defmt messages remain visible and use a second WSL terminal for the desktop app.

### Item-2 acquisition-baseline verification

On `agent-live-fit-fixes`, each MLX90393 is programmed after reset to the Old-Pi comparison baseline:

```text
gain                = 4
resolution register = 0
HALLCONF             = 0x0C
OSR                  = 2
DIG_FILT             = 4
```

The firmware reads those registers back before `configure_old_pi_baseline()` succeeds. A successful sensor now prints a startup line of the form:

```text
MLX addr=<address> Old-Pi baseline verified gain=4 resolution=0 hall_conf=12 osr=2 dig_filt=4
```

`hall_conf=12` is decimal `0x0C`.

A mismatch prints:

```text
MLX addr=<address> failed Old-Pi acquisition baseline verification
```

For the item-2 hardware check, verify that every connected sensor prints the successful five-field baseline and that none prints the failure line. Item 3 will expose the same information properly through the RPC/UI; until then, the app's cached sensitivity display should not be treated as authoritative startup readback.

## 5. Run the desktop app

Open a second WSL terminal and return to the repository root:

```bash
cd /path/to/magnetic_board_driver
cargo run -p app
```

`-p app` is the preferred explicit form because the root is a workspace. The desktop program is the `app` package. The current `magnetic-data-parser` package is a library rather than another runnable desktop binary, so plain `cargo run` from the repository root may also resolve to the app; `cargo run -p app` is less ambiguous and will remain correct if more binaries are added later.

The app discovers serial ports and opens the selected controller port at 115200 baud.

In the GUI:

1. Refresh/update the serial-port list if necessary.
2. Select the controller's serial port.
3. Start the field stream.
4. Confirm the expected board/sensor presence is reported.
5. Use the live-fit / sensor-trace views as needed.

## 6. Normal two-terminal workflow

Terminal 1 — firmware / defmt:

```bash
cd /path/to/magnetic_board_driver/firmware
cargo run
```

Terminal 2 — desktop app:

```bash
cd /path/to/magnetic_board_driver
cargo run -p app
```

This is the recommended day-to-day workflow while debugging acquisition and live fitting.

## 7. Quick troubleshooting

### `cargo run` in `firmware/` cannot find a probe

Check:

```bash
probe-rs list
lsusb
```

If the probe is present in Windows but not WSL, USB forwarding/attachment to WSL is the first thing to fix.

### The app shows no serial ports

Check:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

The desktop app uses the serial port selected in its UI at 115200 baud.

### Firmware builds with an explicit target but `cargo run` fails

Use the firmware workspace configuration as the source of truth:

```bash
cd firmware
cargo check
cargo run
```

The configured target is `thumbv8m.main-none-eabi`; avoid carrying a different explicit target into normal flash commands unless the repository configuration is intentionally changed.

### Item-2 baseline verification fails on an MLX sensor

Preserve the full firmware startup log and note the MLX address that failed. Do not move on to fitter tuning: the sensor acquisition configuration should be fixed first.
