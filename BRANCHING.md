# Branching workflow

This repository uses two active measurement branches with different roles:

- `gk_measurement` — production/lab branch. This is the validated branch to hand to the lab for routine measurements.
- `mlx-live-fit` — calibration/development branch. This keeps the richer MLX90393 controls, timing/cadence diagnostics, copy-to-clipboard helpers, compact logging, UI diagnostics, and other instrumentation used while developing and validating the measurement system.

## Promotion rule

1. Use `mlx-live-fit` for calibration experiments, diagnostics, and development.
2. Measurement-correctness fixes that are validated on the development branch should be deliberately promoted into `gk_measurement`.
3. `gk_measurement` should remain simpler and stable enough to distribute to the lab; development-only diagnostics do not need to be promoted wholesale.
4. When a geometry, calibration, field-frame, or sensor-correction change affects both branches, keep the two implementations synchronized explicitly rather than allowing them to drift silently.

## Sensor orientation convention

For the physical calibration view, the JST connectors point toward the user/front. Sensor 0 is the bottom-right sensor in that view. The KiCad-verified acquisition numbering runs up a column first and then moves leftward across columns.

## 2026-09-27 consolidation

The earlier `brandon/mlx-sensitivity-live-fit` history was preserved at:

`archive/mlx-sensitivity-live-fit-pre-consolidation-2026-09-27`

The active development branch is now named simply `mlx-live-fit`. The old `brandon/mlx-sensitivity-live-fit` name is deprecated and should not be used for new work.

`gk_measurement` remains the production/lab branch and is not replaced by the calibration/development branch.
