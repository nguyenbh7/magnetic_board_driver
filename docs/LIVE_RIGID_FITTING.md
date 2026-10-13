# Live rigid-body fitting

Experimental desktop-app feature branch: feature/live-rigid-body-fit.

## Build and run (Windows PowerShell)

    git fetch origin
    git switch feature/live-rigid-body-fit
    cargo test -p app
    cargo run -p app --release

The joint six-DOF fitter runs alongside the unchanged independent single-board fits. It interpolates three complete BoardFrame streams to a common bracketed timestamp and jointly fits XYZ translation (mm) and rotation-vector components (radians). All three finite-cylinder magnet fields contribute at all 48 Hall sensors. Relative magnet centers/axes, strengths, sensor positions, board rotations, gains, and backgrounds are fixed by the loaded geometry profile. Fits run off the UI thread; new fits are skipped while a worker is busy.

## A1 V1 live-demo setup (Zoom-ready)

Choose **Tracking setup → A1 V1 — calibrated demo** from the drop-down
above the rigid-fit graphs. This automatically loads the bundled reference
profile; no CT workbook or custom JSON is required. A1 uses the 110-mm
triangle and per-magnet calibration in
`hall-effect-motion-tracking/data/reference/a1_v1_rigid_constellation_2026-10-05.json`.
The field convention is model = [By,-Bx,Bz], with Board 2 on the right,
Board 0 upper-left, and Board 1 lower-left. Magnet offsets are **effective
calibrated locations**, not independently CT-measured locations.

**Prepare for a screen-shared demo:**

1. Start the app and three-board sensor streaming; confirm Boards 0/1/2
   are present and providing data. Select **A1 V1 — calibrated demo**.
2. With **all three magnets physically removed**, wait for complete frames
   and press **Capture A1 no-magnet background**. The UI should show
   **A1 background: captured (3 boards)**. This snapshots one complete
   frame per board, **not** a long-duration noise average. Never click
   this button while magnets are installed.
3. Attach all three magnets in the same A1 V1 geometry and polarity used
   for calibration. Wait until the rigid fitting status says **Tracking**
   (accepted 6-DOF pose, not a fit rejection).
4. At the demonstration's stationary reference pose, press **Set rigid zero**.
   The UI shows **Rigid zero: set**, and the translation and rotation
   graphs now reference that pose.
5. Move the calibrated rigid A1 toolhead in small deliberate translations
   (and only physically safe small rotations). Show the **3-D translation
   magnitude** and **relative rotation angle** live; check completed-fit
   count and fitted field RMSE in the status. Rotation is the fitted
   estimate and is **not** independently validated by an angular
   ground-truth instrument yet.

Re-selecting A1 **reloads the profile and requires a fresh background
capture**. Changing the serial connection also clears its capture. If the
A1 mount, magnet orientations, board placements, or magnet type changed,
the bundled calibration is not automatically valid—recalibrate before
making quantitative claims. The independent per-board view/logging is
left unchanged.

The **Enable/Disable rigid fit**, **Set rigid zero** and **Copy rigid
diagnostics** controls continue to work in A1 mode. The no-magnet capture
button only appears when A1 is selected.

## Default Gamma Knife stage magnet locations

The Live Fits dashboard now offers a **Tracking setup** dropdown:
**A1 V1** (complete bundled demo calibration), **Linear stage — May-15 CT**
(reference centers), and **Rotation stage** (reference centers). Only
the two Gamma Knife stage selections are magnet-center-only references.

- **Linear stage — May-15 CT**: in the May-15 CT world frame, Left/M1
  (180.20, 114.40, 114.30) mm, Top/M2 (104.10, 95.20, -14.30) mm,
  Right/M3 (17.70, 105.00, 112.40) mm. Source: the
  `markers_051526` sheet of `sensor_position_data_05152026.xlsx`.
  The matching offline CT fit uses Left<-raw1, Top<-raw0, Right<-raw2
  and right-board sensor permutation `flip_rot180`.
- **Rotation stage — measured centers**: relative to the rotation-stage
  origin, Top (126.89, 0.00, -90.59) mm, Left (0.00, 82.03, -94.15) mm,
  Right (0.00, -82.03, -94.15) mm. Source:
  `hall-effect-motion-tracking/src/gk_analysis/rotation_stage.py`
  constant `MEASURED_MAGNET_CENTERS_STAGE_MM` on branch
  `magnet-simulations`. The 25-mm tangent board geometry there is a
  **simulation assumption**, not independently measured sensor poses.

Selecting either **Gamma Knife** dropdown reference **does not fabricate sensor positions,
magnet axes, board orientations, or per-board field corrections**. If the
currently loaded full profile is incompatible, live rigid fitting is disabled;
the selected reference points are shown and the app requests a complete
calibration profile. The user can still run independent per-board fits.

To activate a matching stage fit, press **Load rigid geometry (.json)** and
choose a full profile with the corresponding coordinate-frame key:

- `"coordinate_frame": "gk_may15_ct"` for the May-15 CT linear reference.
- `"coordinate_frame": "gk_rotation_stage"` for a calibrated rotation-stage setup.

The magnet centers must match the chosen reference within 0.05 mm, in the
displayed order. A custom profile without one of these explicit frame tags
remains supported, but is **not** identified as either built-in stage preset.
The CT export utility now writes `gk_may15_ct` and checks the measured
magnet locations. Re-export any older May-15 JSON profile to enable
automatic dropdown identification.

Selecting **A1 V1 — calibrated demo** switches back to the bundled A1
profile, clears the previous background and rigid zero, and lets you
capture a fresh no-magnet background before tracking.

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
