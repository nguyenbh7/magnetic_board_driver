# October 30 Gamma Knife — recorded Hall time-series replay (research only)

**Goal:** Resume the already recorded Gamma Knife experiment, NOT restart DICOM research. Run the original Oct30 **1,382 frames × 3 boards** through the Rust app's *existing* `RigidLiveFit::receive_frame` → `RigidFitJob::run` → `RigidLiveFit::apply_completion` path and compare the endpoint 6-DOF transform with pre/post CBCT. Geometry **A** is the already retained optimized three-board CT geometry plus authenticated KiCad superior PCB/JST prior. Geometry **B**, if supplied, uses the same 48 primary CT chip positions, magnets and BB but the hybrid superior PCB-center review candidate. No new scan, magnet model, board detection, UI panel or tracking enablement.

## Implemented, not yet run on original Oct30 Hall data

- `hall-effect-motion-tracking/tools/cbct_bridge/export_oct30_hall_rust_replay.py`: input is the *previously provided* ZIP containing three `*_complete.npz` files, raw firmware order 0..15, `B_uT` and per-chip `time_us`. Its private output preserves **all 4,146 16-sensor frames and their timestamps** (convert to Rust `BoardFrame` with u16 per-chip time offsets). It refuses wrong dimensions, missing boards, mismatched source hashes, nonmonotonic frames, out-of-range offsets or rewriting outputs. It also derives the independent **CT motion reference** from the same original pre/post optimized+KiCad detector JSONs, aligning 32 stationary side-board chips first using `compare_oct30_prepost.stationary_registration`; then fits the 3 magnet centers. This yields **signed LPS translation vector and 3D rotation vector**, not just 1.713° and 0.516 mm scalar magnitudes.
- Fixed, predeclared motion channel exclusions: **any raw Hall sensor** with an interframe component jump **>8 mT** or first-vs-last plateau component change **>10 mT** is masked across the replay. This is a conservative suspected-wrap QC policy; no inferred modulo period, fabricated unwrapped field or gain correction. The archive has **no independently recorded no-magnet per-sensor background**, so the Rust app's existing `fit_strengths_with_board_offsets` stationary-field calibration is used with the initial **1.0–4.5-s** plateau. All 48 baseline channels contribute to calibration exactly as that method currently does; the exclusion applies to the **motion pose fits**. Calibrated signed remanence plausibility 300–2200 mT and stationary 0.5-mT RMSE must be **reported**; failing these never activates tracking.
- Opt-in Rust test `experimental_oct30_recorded_gk_motion_time_series_replay` in `app/src/rigid_fit.rs`: loads private recorded data and A (required), B (optional), assembles unchanged Rust `BoardFrame` samples and sensor masks, ingests **every raw frame**, and solves every tenth board frame by default (fixed offline 10-frame stride, approximately 3–4 fitted poses/sec; override via `GK_OCT30_FIT_STRIDE`). This is *offline solver subsampling*, NOT a measurement of actual live execution cadence. Does not modify live tracking flags or alter optimizer tolerances.
- Primary A uses `experimental_candidate_profile_with_reviewed_cad()`. B only runs if the unchanged `register_ct_boards_v2()` gate passes **and the firmware-sensor ordering matches A**. Otherwise B is recorded as a deliberate failure while A continues. Sensor centers, normals, magnets, original raw Hall measurements, exclusions and independent CT truth remain fixed. Both arms recalibrate signed finite-cylinder strengths/board-constant offsets using their own frozen geometry (as the app would for either model); the result must **not** be interpreted as a geometry-only magnetic error comparison, because changes in mapping axes alter fitted nuisance parameters. No CT motion target is passed into calibration or optimization.
- Reports accepted/rejected frame counts, motion fit RMSE, stationary strength/offset calibration QC, and **full geodesic rotation disagreement plus LPS translation-vector disagreement** against independent CT truth. Pre pose is the median of *accepted* fits in **1.0–4.5 s**; final pose is the median accepted fit in the **last 8 s**. The relative orientation is `R_post @ R_pre.T`. If either plateau lacks three accepted poses, the run explicitly reports **insufficient fits** instead of inventing a motion result. The optional private trajectory contains attempted times, pose, error and acceptance. The shareable **sanitized summary** contains no raw Hall fields, absolute CT feature positions or DICOM UIDs; it still represents potentially research-sensitive derived data.
- The original CT truth is a **single pre/post endpoint**. There is no independent mid-motion CT truth and no HDMM comparison from this archive. Original point-dipole differential fits (~1.76°) use a different physical field model and are only context, not proof the finite-cylinder Rust solver matches.

## One-time commands — Windows, use existing downloaded files

**1) Build the private input from the prior NPZ archive and the newly saved original pre/post CT detections:**

```powershell
cd F:\GitHub\hall-effect-motion-tracking
git switch a1-vgm-validation
git pull --ff-only origin a1-vgm-validation
cd .\tools\cbct_bridge
.\.venv\Scripts\python.exe -m unittest -v test_export_oct30_hall_rust_replay.py

$archive="$HOME\Downloads\GK_manequinn measurement_10_30_2026_2_board2_complete.zip"
$primary="$HOME\Downloads\GK_CBCT_Results\oct30_prepost_primary_for_hybrid"
$replay="$HOME\Downloads\GK_CBCT_Results\oct30_motion_replay_PRIVATE.json"

foreach($path in @($archive,
  "$primary\pre\detections_with_cad_estimates.json",
  "$primary\post\detections_with_cad_estimates.json")) {
  if(-not (Test-Path -LiteralPath $path -PathType Leaf)) {
    throw "Original Oct30 input not found: $path. Use the existing saved file; do not substitute another acquisition."
  }
}

.\.venv\Scripts\python.exe .\export_oct30_hall_rust_replay.py `
  --archive "$archive" `
  --ct-pre "$primary\pre\detections_with_cad_estimates.json" `
  --ct-post "$primary\post\detections_with_cad_estimates.json" `
  --output "$replay"
if($LASTEXITCODE -ne 0){throw "Oct30 private export failed"}
```

If the original archive is saved elsewhere, **change only `$archive` to its actual existing location**. Do not rerun CT scanning/detection or collect any new Hall data. The exporter refuses to overwrite `$replay`, so choose a fresh output filename if reprocessing is intentional.

**2) Replay through the actual Rust optimizer (no app GUI needed):**

```powershell
cd F:\GitHub\magnetic_board_driver
git switch feature/live-rigid-body-fit
git pull --ff-only origin feature/live-rigid-body-fit

$env:GK_OCT30_REPLAY_IN="$replay"
$env:GK_OCT30_PRIMARY_PRE="$primary\pre\detections_with_cad_estimates.json"
$env:GK_OCT30_HYBRID_PRE="$HOME\Downloads\GK_CBCT_Results\oct30_hybrid_PCB_only_for_gk_replay\Oct30_pre\detections_hybrid_PCB_only_REVIEW.json"
$env:GK_OCT30_SUMMARY_OUT="$HOME\Downloads\GK_CBCT_Results\oct30_RUST_motion_summary_sanitized.json"
$env:GK_OCT30_TRAJECTORY_OUT="$HOME\Downloads\GK_CBCT_Results\oct30_RUST_motion_trajectory_PRIVATE.json"
$env:GK_OCT30_FIT_STRIDE="10"
cargo test -p app --release experimental_oct30_recorded_gk_motion_time_series_replay -- --nocapture
```

The opt-in replay test runs only when `GK_OCT30_REPLAY_IN` is defined; ordinary `cargo test -p app --release` stays fast. Existing saved output paths are **not overwritten**. If B fails registration, it remains a clearly flagged optional case; continue analysis of A. **Do not loosen Rust CT-mapping or fit thresholds to obtain a passing B.**

**Upload only `oct30_RUST_motion_summary_sanitized.json`** and the console test summary after verifying contents. Retain full Hall frames, private trajectories, and original detector coordinates locally. The next task is interpreting finite-cylinder fitting quality, wrap exclusions and CT endpoint errors—not more DICOM geometry engineering.

## Validation state

- Prior: Windows app `cargo test -p app --release` **43/43 passed** before adding the opt-in replay test; pre/post hybrid JSON preparation and 11 Python geometry tests succeeded.
- New Oct30 NPZ exporter, CT target synthetic tests and opt-in Rust recorded-motion test are **committed but not yet exercised on the user's local original data or Windows toolchain**. Do not claim a successful 6-DOF replay or measured accuracy until that run is available.
