# Live-fit remediation plan

This branch (`agent-live-fit-fixes`) tracks fixes identified by comparing the current STM32/Rust live-fit path against the older Raspberry Pi live-fitting implementation and the validated offline `gk_analysis` workflows.

The order is intentional: make the acquisition geometry and sensor data trustworthy before changing the fitter.

## Current progress

- Completed: **1. sensor geometry/address ordering**.
- Completed: **2. known-good MLX90393 acquisition configuration**.
- Completed: **3. sensitivity/configuration reporting and resolution naming**.
- Completed: **4. HALLCONF 0x00 magnetic-field scaling validation**.
- Current next item: **5. robust per-sensor offset/background calibration**.
- Firmware hardware workflow is documented in `WSL_FLASH_AND_RUN.md`.
- Canonical firmware flash command is `cd firmware && cargo run --release`.
- The unoptimized firmware dev profile is not suitable for this hardware path: both the untouched baseline branch and this branch HardFault during the existing 16-sensor async construction path when run without `--release`.
- Item 2 hardware validation passed in release mode: startup completed through the RPC server with no MLX baseline-verification failure.
- Startup now probes for sensor presence before applying the Old-Pi baseline, so absent addresses are skipped instead of incurring repeated I2C timeout/configuration cycles.
- Temporary boot diagnostics and per-sensor success logs have been removed; normal firmware startup is back to the original two `Hello World!` messages, while MLX baseline failures remain visible.
- High-frequency streamed-field and raw-MLX debug `info!` output is disabled during normal operation.
- Item 3 hardware validation passed: `Read from board` returned a consistent actual register readback of gain 4, raw resolution 0 (16-bit), and HALLCONF 0x0C across the detected sensors.
- Item 3 desktop validation passed: `cargo check -p app` succeeded and the running UI displayed raw resolution 0 explicitly as 16-bit.
- Item 4 software validation passed with explicit HALLCONF scaling regression tests and `cargo test -p data_transfer`; firmware `cargo check --release` also passed.
- Item 4 hardware A/B validation passed in a strong-field linear-stage setup: HALLCONF 0x0C gave approximately (-880, 620, 1630) uT and HALLCONF 0x00 gave approximately (-870, 650, 1680) uT, i.e. about 1.1%, 4.8%, and 3.1% axis differences rather than a common ~30.7% scaling error.

## Acquisition / sensor pipeline

- [x] **1. Fix the firmware 4x4 sensor geometry and address ordering.**
  - Use the verified 13.5 mm square with 4.5 mm pitch.
  - Address order `0x0C..0x1B` maps to rows with `y = -6.75, -2.25, 2.25, 6.75 mm` and, within each row, `x = 6.75, 2.25, -2.25, -6.75 mm`.
  - Boards A/B/C share the same physical index-to-position mapping; Board C's address bit remapping does not change its physical sensor index.
  - Implemented in `firmware/microcontroller/src/main.rs` via `sensor_grid_position_mm()` and a 4.5 mm pitch derived from the 13.5 mm side length.

- [x] **2. Establish and enforce a known-good MLX90393 acquisition configuration.**
  - Start from the Old-Pi baseline for comparison: gain 4, resolution register value 0, HALLCONF 0x0C, OSR 2, DIG_FILT 4.
  - Program these values after sensor reset instead of relying on retained/default register state.
  - Read back OSR and DIG_FILT along with gain/resolution/HALLCONF before accepting the startup configuration.
  - Correct register `0x02` OSR/DIG_FILT/OSR2 parsing so conversion delays use the actual 16-bit field locations.
  - Added host-side register parsing tests for the Old-Pi OSR/filter combination and temperature OSR field.
  - WSL software verification passed: `cargo test -p data_transfer` and firmware compilation both passed.
  - Hardware verification passed in the firmware release profile: the controller completed sensor initialization and reached the RPC server without an MLX baseline-verification failure.
  - Firmware must be run with `cargo run --release`; the unoptimized dev profile HardFaults during the existing 16-sensor async construction path even on the untouched baseline branch.
  - Startup now probes each sensor before applying the baseline, avoiding long configuration retries on absent addresses.
  - Successful per-sensor baseline messages were removed after validation to keep normal startup output concise; baseline verification failures are still logged.

- [x] **3. Fix sensitivity/configuration reporting and naming.**
  - `GetMlxSensitivity` now re-detects connected sensors and reads the actual MLX90393 registers instead of returning only the cached startup struct.
  - A successful status requires at least one detected sensor and a consistent gain/resolution/HALLCONF readback across all detected sensors.
  - Resolution variants and raw mappings now follow the MLX90393 register convention: 0=16-bit, 1=17-bit, 2=18-bit, 3=19-bit.
  - The desktop status keeps raw values explicit and displays the corresponding bit depth, e.g. `resolution=0 (16-bit)`.
  - Host/shared software verification passed with `cargo test -p data_transfer`; firmware `cargo check --release` and desktop `cargo check -p app` also passed.
  - Hardware readback verification passed: the app returned `ok=true`, gain 4, raw resolution 0 (16-bit), and HALLCONF 0x0C from the connected sensor board.
  - The updated desktop app was run successfully and displayed the expected 16-bit label.

- [x] **4. Validate HALLCONF 0x00 magnetic-field scaling.**
  - Inspection showed that the existing conversion formula already used the MLX90393 datasheet factor `98/75` for HALLCONF 0x00 relative to the HALLCONF 0x0C sensitivity table, so no production scaling change was required.
  - Added regression tests that pin known sensitivity-table values and verify that the HALLCONF factor reaches final converted magnetic-field values.
  - `cargo test -p data_transfer` passed with the new tests; firmware `cargo check --release` also passed.
  - A low-field ambient test was intentionally treated as inconclusive because mode-dependent offset/noise was comparable to the measured field.
  - Strong-field hardware A/B verification passed in the linear-stage setup: HALLCONF 0x0C measured approximately `(-880, 620, 1630)` uT and HALLCONF 0x00 approximately `(-870, 650, 1680)` uT with the sensor/magnet stationary.
  - Those changes are approximately 1.1% (X), 4.8% (Y), and 3.1% (Z), which is consistent with the physical field remaining comparable across modes and rules out a missing/reversed common `98/75` scale factor.
  - Keep HALLCONF 0x0C as the acquisition baseline after testing.

- [ ] **5. Add robust per-sensor offset/background calibration.**
  - Replace one-frame background capture with a multi-frame average.
  - Preserve calibration per sensor and XYZ component.
  - Decide whether calibration should be persisted in firmware/app state and whether calibrated fields should also be written to logs.

- [ ] **6. Make streamed board frames explicit and atomic.**
  - Add a frame/sweep identifier to streamed measurements or publish a complete board frame as one message.
  - Prevent a desktop fit from combining the tail of sweep N with the head of sweep N+1 after subscription startup or a dropped packet.
  - Update raw logging/decoding so frame boundaries are unambiguous.

## Live fitting / display

- [ ] **7. Replace per-frame free-moment grid fitting with the validated constrained pose model.**
  - Use known magnet magnitude/geometry and fit position plus orientation continuously.
  - Use the previous result as a warm start.
  - Keep a coarse/global/grid search only for initialization/reacquisition.
  - Port the useful behavior from the validated `gk_analysis` temporal/reacquisition workflow rather than duplicating notebook code literally.

- [ ] **8. Add controlled temporal filtering for the live display.**
  - Reproduce the useful smoothing effect of the Old-Pi buffered averaging with a small configurable frame average or EMA.
  - Keep latency measurable and configurable rather than hiding it.
  - Prefer filtering fields before fitting when comparing directly with the Old-Pi behavior.

## Validation

- [ ] **9. Add live-fit regression/diagnostic tests.**
  - Verify sensor index -> position mapping.
  - Verify MLX conversion/configuration readback.
  - Compare Rust fit results against `gk_analysis` on recorded frames.
  - Quantify stationary position jitter, displacement noise, RMSE, and reacquisition behavior before/after each major change.
