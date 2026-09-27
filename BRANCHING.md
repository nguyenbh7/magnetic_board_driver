# Branching workflow

This repository uses the following measurement branches:

- `gk_measurement` — production branch for validated measurement, calibration, logging, and live-fit behavior used for experiments.
- `mlx-live-fit` — development branch for experimental MLX90393 sensitivity, live-fit, logging, diagnostics, and related measurement changes. New work should start here and be promoted into `gk_measurement` only after validation.

## Promotion rule

1. Start development from the current `gk_measurement` tip by updating/rebasing `mlx-live-fit` onto production before new work.
2. Develop and test on `mlx-live-fit`.
3. Merge validated changes into `gk_measurement` through a focused review/PR.
4. After promotion, bring `mlx-live-fit` forward to the new `gk_measurement` tip so the two branches do not accumulate independent long-running histories.

## 2026-09-27 consolidation

The historical `brandon/mlx-sensitivity-live-fit` branch had diverged substantially from `gk_measurement`. Its pre-consolidation tip is preserved at:

`archive/mlx-sensitivity-live-fit-pre-consolidation-2026-09-27`

That archive is retained only as a source for selectively recovering old experimental work. It should not be used as a measurement/production branch.

The KiCad-verified sensor mapping and the A1 calibrated live-fit/raw-background pipeline on `gk_measurement` are the baseline that future development must preserve.
