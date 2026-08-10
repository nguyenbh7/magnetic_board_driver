# Item 7 stage A/B validation

This note records the current validation direction for the constrained live pose fitter.

## Production requirement

The production/clinical tracker must fit magnet orientation continuously.  A fixed-orientation displacement tracker is **not** an acceptable production solution because magnet orientation cannot be guaranteed clinically.

The linear stage is used only as translation ground truth for validation.  A commanded pure-axis stage move lets us measure displacement scale, cross-axis leakage, hysteresis, and distance dependence while the fitter remains free to estimate orientation.

## Current hardware observation

With the constrained five-parameter live fit running and the stage used as translation ground truth:

- X: about 0.9 mm live displacement per 1 mm physical travel;
- Y: about 1.3 mm live displacement per 1 mm physical travel;
- Z: about 1.2-1.5 mm live displacement per 1 mm physical travel;
- a 5 mm Z move produced about 6.5 mm total fitted displacement;
- as Z separation increased, more of the fitted motion appeared in X/Y instead of Z.

This is not consistent with a single scalar displacement calibration error.  A pure-axis move leaking into other fitted axes indicates coupling among translation and orientation/model parameters.

Do not add empirical independent X/Y/Z gains before the model/optimizer comparison is resolved.

## Rust vs validated Python A/B

Use the same raw magnetic stage-run data for both implementations:

1. Rust live constrained pose fit (`agent-live-fit-fixes`).
2. Validated Python free-orientation fits in `gk_analysis/agent-dev`.

The Python A/B tooling is documented in `gk_analysis/LIVE_STAGE_AB.md` and implemented by `scripts/run_live_stage_ab.py`.

The new `gk_analysis` raw decoder supports both archived 66-byte SensorField logs and the current 71-byte `frame_id` SensorField logs.  New logs reconstruct board frames by explicit firmware `frame_id`, so the offline analysis uses the same atomic sweep boundaries as the live application.

Primary decision rule:

- Python correct / Rust wrong -> fix Rust optimizer/temporal implementation.
- Python and Rust show similar scale/leakage -> investigate magnetic model, magnet strength, background/sensor calibration, or other shared assumptions before changing the optimizer.

## Magnet-strength calibration caveat

The old `Calibrate magnet / set zero` behavior was designed around the free-moment fitter, where a fitted moment magnitude could be compared with the nominal magnet model.

Under the current constrained 5p solver, moment magnitude is supplied to the optimizer as a fixed input and the returned `PoseFit.moment_norm` is that same supplied value.  Therefore the old calibration calculation cannot independently estimate magnet strength; it effectively compares the model magnitude with itself.

Do not treat that button as a validated magnet-strength calibration in item 7.  Strength calibration/estimation needs a separate design after the Rust-vs-Python A/B test establishes whether the present displacement bias is optimizer-specific or shared with the physical model.

## Current test sequence

Start with a Z-axis continuous run because the physical displacement is unambiguous:

`0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 0 mm`

Hold each plateau for roughly 3-5 seconds.  Record a separate no-magnet raw background file first.  Record live Rust signed delta X/Y/Z, total displacement, and RMS at each plateau.  The Python runner automatically detects stage transitions from raw-field jumps and reports plateau displacement error and off-axis leakage; explicit transition markers can override automatic segmentation if necessary.
