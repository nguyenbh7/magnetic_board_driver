# Live rigid-body fitting

Experimental desktop-app feature branch: feature/live-rigid-body-fit.

## Build and run (Windows PowerShell)

    git fetch origin
    git switch feature/live-rigid-body-fit
    cargo test -p app
    cargo run -p app --release

The joint six-DOF fitter runs alongside the unchanged independent single-board fits. It interpolates three complete BoardFrame streams to a common bracketed timestamp and jointly fits XYZ translation (mm) and rotation-vector components (radians). All three finite-cylinder magnet fields contribute at all 48 Hall sensors. Relative magnet centers/axes, strengths, sensor positions, board rotations, gains, and backgrounds are fixed by the loaded geometry profile. Fits run off the UI thread; new fits are skipped while a worker is busy.

## A1 reference setup

Press **Use A1 reference profile**. This uses the 110-mm triangle and per-magnet calibration recorded in hall-effect-motion-tracking/data/reference/a1_v1_rigid_constellation_2026-10-05.json. The field convention is model = [By,-Bx,Bz]; Board 2 is right, Board 0 upper-left, Board 1 lower-left. This is an effective fitted calibration, not independently verified physical magnet offsets.

With all magnets REMOVED, stream complete frames from all three boards and press **Capture A1 no-magnet background**. The current version captures one complete frame per board and is not a long-duration background average. Then install the magnets, fit the pose, and press **Set rigid zero** to reference displacement and angular motion to the current pose. Do not capture a no-magnet background while magnets are installed.

## Gamma Knife CT setup

Run hall-effect-motion-tracking/scripts/fit_gk_05152026_ct_rigid.py with the May-15 CT workbook and original raw dataset, producing one dataset folder containing manifest.json and baseline_calibration.csv. Then from magnetic_board_driver run:

    python .\tools\export_gk_ct_live_profile.py `
      --analysis-repo F:\GitHub\hall-effect-motion-tracking `
      --ct-xlsx "PATH_TO_CT_WORKBOOK.xlsx" `
      --fit-results "PATH_TO_RESULTS\gk_05152026_ct_rigid\gk_data_05152026" `
      --output .\app\config\gk_ct_live_profile.json

Choose **Load rigid geometry (.json)** in the app and select this exported file. The exporter verifies CT sensor index correspondence identity,identity,flip_rot180 and physical-to-raw Left<-1, Top<-0, Right<-2. It rotates historical raw Hall fields into CT coordinates, rather than applying the A1 [By,-Bx,Bz] transform. The fixed per-board CT backgrounds are already part of the profile; do NOT capture an A1 no-magnet background.

A May-15 profile cannot be assumed valid after remounting a headrest, changing magnet orientation, moving sensors or changing the background field. Regenerate the geometry/calibration for the actual apparatus.

## Geometry JSON and computation

Profile fields: name, origin_mm, boards (raw_board_id, 16 sensors_mm in firmware order, 3x3 raw_to_world, background_mt, optional 16 sensor_gains), magnets (center_mm, signed axis, diameter_mm, thickness_mm, signed remanence_mt), initial_pose, lower_bounds, upper_bounds, max_rmse_mt. The body transform is center_i = origin + t + R*(center_i0-origin), axis_i = R*axis_i0.

Each frame is timestamped using sample offsets; interpolation requires bracketing and <=120 ms gap on either side, with at least eight usable sensors per board. The solver uses the Python model's 4x12 magnetic-charge face quadrature but an independent six-parameter damped least-squares optimizer, NOT the SciPy trust-region-reflective implementation. It warm-starts from the previous pose, rejects fits above the configured RMS threshold, and cancels stale completions by generation.

The UI displays absolute translation and rotation vector, zero-relative translation and angular change, RMS field residual, sensor count, worker duration, counts and plots. Copy rigid diagnostics captures these results. Existing logging and legacy per-board plots are unchanged.

## Validation required

1. Build/test and run the app in the user's Rust toolchain. A build could not be executed in this environment.
2. Verify raw board IDs, sensor correspondence and orientation at the physical installation.
3. Run stationary, known-translation and known-rotation trials, and compare fits and field residuals against offline Python with the same profile and measured frames.
4. Benchmark release-build worker elapsed time and the actual live update rate; long fits cause skipped intermediate snapshots but do not intentionally block acquisition.
5. Confirm port changes, stream restarts, profile loads and background recaptures invalidate old fits.

Research results from the offline SciPy solver must not be attributed to the live solver until the parity and hardware tests pass.
