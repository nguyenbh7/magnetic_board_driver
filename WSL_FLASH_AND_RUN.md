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
cargo check --release
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
cargo run --release
```

**Use the release profile for firmware flashing.** The unoptimized dev profile has been observed to HardFault during the 16-sensor async construction path, while the same source boots normally in `--release`.

`firmware/.cargo/config.toml` defines:

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "probe-rs run --chip STM32WBA55CG"

[build]
target = "thumbv8m.main-none-eabi"

[env]
DEFMT_LOG = "debug"
```

Therefore `cargo run --release` from `firmware/` does all of the following:

1. Builds the `microcontroller` firmware for the configured Cortex-M target using the optimized release profile.
2. Runs the configured `probe-rs` runner.
3. Programs the STM32WBA55CG.
4. Attaches to defmt output at debug level.

For normal development, leave this terminal open so firmware/defmt messages remain visible and use a second WSL terminal for the desktop app.

### Why `--release` matters

The firmware constructs 16 sensor initialization futures for a board. In the unoptimized dev profile, this async state is large enough to HardFault during startup on the STM32WBA55CG. The original `brandon/mlx-sensitivity-live-fit` branch was hardware-tested on the same setup and showed this behavior:

- `cargo run` -> HardFault during sensor-group construction.
- `cargo run --release` -> boots normally.

Treat `cargo run --release` as the canonical firmware command.

### MLX90393 acquisition-baseline behavior

On `agent-live-fit-fixes`, each MLX90393 is programmed after reset to the Old-Pi comparison baseline:

```text
gain                = 4
resolution register = 0
HALLCONF             = 0x0C
OSR                  = 2
DIG_FILT             = 4
```

The firmware reads those registers back before accepting the configuration. Successful sensors are intentionally silent so normal firmware startup remains similar to the original branch. A mismatch prints:

```text
MLX addr=<address> failed Old-Pi acquisition baseline verification
```

A normal release-mode startup with no baseline-failure messages means the startup readback passed. Item 3 will expose the live sensor configuration properly through the RPC/UI; until then, the app's cached sensitivity display should not be treated as authoritative hardware readback.

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
cargo run --release
```

Terminal 2 — desktop app:

```bash
cd /path/to/magnetic_board_driver
cargo run -p app
```

This is the recommended day-to-day workflow while debugging acquisition and live fitting.

## 7. Quick troubleshooting

### `cargo run --release` in `firmware/` cannot find a probe

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

### Firmware HardFaults when run without `--release`

This is a known behavior of the current embedded startup path. Use:

```bash
cd firmware
cargo run --release
```

Do not use a successful unoptimized `cargo check` as proof that the dev-profile firmware will run safely on the MCU.

### Firmware builds with an explicit target but `cargo run --release` fails

Use the firmware workspace configuration as the source of truth:

```bash
cd firmware
cargo check --release
cargo run --release
```

The configured target is `thumbv8m.main-none-eabi`; avoid carrying a different explicit target into normal flash commands unless the repository configuration is intentionally changed.

### MLX baseline verification fails

Preserve the full firmware startup log and note the MLX address that failed. Do not move on to fitter tuning: the sensor acquisition configuration should be fixed first.
