# Branching workflow

This repository uses two active measurement branches with different roles:

- `gk_measurement` — production/lab branch. This is the validated branch to hand to the lab for routine measurements.
- `mlx-live-fit` — calibration/development branch. This keeps the richer MLX90393 controls, timing/cadence diagnostics, copy-to-clipboard helpers, compact logging, UI diagnostics, and other instrumentation used while developing and validating the measurement system.

## Promotion rule

1. Use `mlx-live-fit` for calibration experiments, diagnostics, and development.
2. Measurement-correctness fixes that are validated on the development branch should be deliberately promoted into `gk_measurement`.
3. `gk_measurement` should remain simpler and stable enough to distribute to the lab; development-only diagnostics do not need to be promoted wholesale.
4. When a geometry, calibration, field-frame, or sensor-correction change affects both branches, keep the two implementations synchronized explicitly rather than allowing them to drift silently.

## Logging overwrite safeguards (2026-10-27)

The `mlx-live-fit` and `feature/live-rigid-body-fit` branches now contain
exclusive new-file creation for `MLXBF002` logs, guarded file selection,
explicit flush/error notices, and in-app logging workflow instructions.
See [app/LOGGING_WORKFLOW.md](app/LOGGING_WORKFLOW.md).

The separate `gk_measurement` production branch still has its older raw +
A/B companion logger and uses `File::create`. Do **not** assume it has
the new overwrite protection. Port that branch deliberately and test both
output files before deploying it for routine lab work.

Code review and static source checks are complete. Cargo tests and a
Windows GUI smoke test are still required on a development machine before
lab deployment.

## Sensor orientation convention

For the physical calibration view, the JST connectors point toward the user/front. Sensor 0 is the bottom-right sensor in that view. The KiCad-verified acquisition numbering runs up a column first and then moves leftward across columns.

## Branch naming

The active development branch is `mlx-live-fit`.

The old `brandon/mlx-sensitivity-live-fit` name is deprecated. Its pre-consolidation history is preserved at `archive/mlx-sensitivity-live-fit-pre-consolidation-2026-09-27`.
