# Live-fit remediation plan

This branch (`agent-live-fit-fixes`) tracks fixes identified by comparing the current STM32/Rust live-fit path against the older Raspberry Pi live-fitting implementation and the validated offline `gk_analysis` workflows.

The order is intentional: make the acquisition geometry and sensor data trustworthy before changing the fitter.

## Acquisition / sensor pipeline

- [ ] **1. Fix the firmware 4x4 sensor geometry and address ordering.**
  - Use the verified 13.5 mm square with 4.5 mm pitch.
  - Address order `0x0C..0x1B` must map to rows with `y = -6.75, -2.25, 2.25, 6.75 mm` and, within each row, `x = 6.75, 2.25, -2.25, -6.75 mm`.
  - Boards A/B/C must share the same physical index-to-position mapping; Board C's address bit remapping must not change its physical sensor index.

- [ ] **2. Establish and enforce a known-good MLX90393 acquisition configuration.**
  - Start from the Old-Pi baseline for comparison: gain 4, resolution register value 0, HALLCONF 0x0C, OSR 2, DIG_FILT 4.
  - Program these values after sensor reset instead of relying on retained/default register state.
  - Expose/read back OSR and DIG_FILT along with gain/resolution/HALLCONF.

- [ ] **3. Fix sensitivity/configuration reporting and naming.**
  - Make `GetMlxSensitivity` query actual sensor state rather than returning only the cached startup struct.
  - Rename resolution variants/UI labels to match the MLX90393 register convention (0=16-bit, 1=17-bit, 2=18-bit, 3=19-bit).
  - Keep raw register values explicit in status output to avoid future ambiguity.

- [ ] **4. Fix HALLCONF 0x00 magnetic-field scaling.**
  - Replace the current approximate multiplier with the correct gain/resolution/axis coefficients for HALLCONF 0x00.
  - Add conversion tests against known reference-table values.

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
