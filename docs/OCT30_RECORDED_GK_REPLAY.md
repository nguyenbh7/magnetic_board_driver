# October 30 Gamma Knife — recorded Hall time-series replay (research only)

**Goal:** Resume the already recorded Gamma Knife experiment, NOT restart DICOM research. Run the original Oct30 **1,382 frames × 3 boards** through the Rust app's *existing* `RigidLiveFit::receive_frame` → `RigidFitJob::run` → `RigidLiveFit::apply_completion` path and compare the endpoint 6-DOF transform with pre/post CBCT. Geometry **A** is the already retained optimized three-board CT geometry plus authenticated KiCad superior PCB/JST prior. Geometry **B**, if supplied, uses the same 48 primary CT chip positions, magnets and BB but the hybrid superior PCB-center review candidate. No new scan, magnet model, board detection, UI panel or tracking enablement.

## Implemented, not yet run on original Oct30 Hall data

- `hall-effect-motion-tracking/tools/cbct_bridge/export_oct30_hall_rust_replay.py`: input is the *previously provided* ZIP containing three `*_complete.npz` files, raw firmware order 0..15, `B_uT` and per-chip `time_us`. Its private output preserves **all 4,146 16-sensor frames and their timestamps** (convert to Rust `BoardFrame` with u16 per-chip time offsets). It refuses wrong dimensions, missing boards, mismatched source hashes, nonmonotonic frames, out-of-range offsets or rewriting outputs. It also derives the independent **CT motion reference** from the same original pre/post optimized+KiCad detector JSONs, aligning 32 stationary side-board chips first using `compare_oct30_prepost.stationary_registration`; then fits the 3 magnet centers. This yields **signed LPS translation vector and 3D rotation vector**, not just 1.713° and 0.516 mm scalar magnitudes.
- Fixed, predeclared motion channel exclusions: **any raw Hall sensor** with an interframe component jump **>8 mT** or first-vs-last plateau component change **>10 mT** is masked across the replay. This is a conservative suspected-wrap QC policy; no inferred modulo period, fabricated unwrapped field or gain correction. The movement NPZ has no embedded no-magnet background. Supply the original matched background MLXBF002 recording when available to use `fit_magnet_strengths`; otherwise the Rust app explicitly falls back to its lower-confidence `fit_strengths_with_board_offsets` calibration. Both use the initial **1.0–4.5-s** stationary plateau. All 48 baseline channels contribute to calibration exactly as that method currently does; the exclusion applies to the **motion pose fits**. Calibrated signed remanence plausibility 300–2200 mT and stationary 0.5-mT RMSE must be **reported**; failing these never activates tracking.
- Opt-in Rust test `experimental_oct30_recorded_gk_motion_time_series_replay` in `app/src/rigid_fit.rs`: loads private recorded data and A (required), B (optional), assembles unchanged Rust `BoardFrame` samples and sensor masks, ingests **every raw frame**, and solves every tenth board frame by default (fixed offline 10-frame stride, approximately 3–4 fitted poses/sec; override via `GK_OCT30_FIT_STRIDE`). This is *offline solver subsampling*, NOT a measurement of actual live execution cadence. Does not modify live tracking flags or alter optimizer tolerances.
- Primary A uses `experimental_candidate_profile_with_reviewed_cad()`. B only runs if the unchanged `register_ct_boards_v2()` gate passes **and the firmware-sensor ordering matches A**. Otherwise B is recorded as a deliberate failure while A continues. Sensor centers, normals, magnets, original raw Hall measurements, exclusions and independent CT truth remain fixed. Both arms recalibrate signed finite-cylinder strengths/board-constant offsets using their own frozen geometry (as the app would for either model); the result must **not** be interpreted as a geometry-only magnetic error comparison, because changes in mapping axes alter fitted nuisance parameters. No CT motion target is passed into calibration or optimization.
- Reports accepted/rejected frame counts, motion fit RMSE, stationary strength/offset calibration QC, and **full geodesic rotation disagreement plus LPS translation-vector disagreement** against independent CT truth. Pre pose is the median of *accepted* fits in **1.0–4.5 s**; final pose is the median accepted fit in the **last 8 s**. The relative orientation is `R_post @ R_pre.T`. If either plateau lacks three accepted poses, the run explicitly reports **insufficient fits** instead of inventing a motion result. The optional private trajectory contains attempted times, pose, error and acceptance. The shareable **sanitized summary** contains no raw Hall fields, absolute CT feature positions or DICOM UIDs; it still represents potentially research-sensitive derived data.
- **Previously recorded background:** The earlier October30 research analysis used a separate no-magnet MLXBF002 background recording. When this **same-session background .bin** is available, provide `--background-bin`: the exporter retains the per-sensor raw median and Rust calls the unchanged `fit_magnet_strengths` plus `set_experimental_gk_profile(profile,Some(background))`, matching the live app's measured-background path. Without it, the app's existing `fit_strengths_with_board_offsets` nuisance-field alternative is explicitly reported as lower confidence. Never substitute a background recorded with a different Hall sensitivity configuration or hardware layout. Neither route changes the pose solver.

The original CT truth is a **single pre/post endpoint**. There is no independent mid-motion CT truth and no HDMM comparison from this archive. Original point-dipole differential fits (~1.76°) use a different physical field model and are only context, not proof the finite-cylinder Rust solver matches.

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

# Optional, strongly preferred IF the existing Oct30 no-magnet background
# MLXBF002 .bin file is available. Use the ORIGINAL matched recording.
# Leave unset to use the app's lower-confidence board-offset fit.
$backgroundCandidate="$HOME\Downloads\GK_manequin_measurement_10_30_2026.bin"
$backgroundBin=if(Test-Path -LiteralPath $backgroundCandidate -PathType Leaf) {
  $backgroundCandidate
} else {
  $null
}
# If the actual saved original background is elsewhere, set $backgroundBin
# to that file's exact local path. Do not substitute a different setup.
$backgroundArgs=@()
if($backgroundBin) {
  if(-not (Test-Path -LiteralPath $backgroundBin -PathType Leaf)) {
    throw "Matched Oct30 background log missing: $backgroundBin"
  }
  $backgroundArgs=@("--background-bin",$backgroundBin)
}

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
  @backgroundArgs `
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
# First run baseline A alone; do not introduce the optional hybrid arm yet.
Remove-Item Env:GK_OCT30_HYBRID_PRE -ErrorAction SilentlyContinue
$env:GK_OCT30_SUMMARY_OUT="$HOME\Downloads\GK_CBCT_Results\oct30_RUST_motion_summary_sanitized.json"
$env:GK_OCT30_TRAJECTORY_OUT="$HOME\Downloads\GK_CBCT_Results\oct30_RUST_motion_trajectory_PRIVATE.json"
$env:GK_OCT30_FIT_STRIDE="10"
cargo test -p app --release experimental_oct30_recorded_gk_motion_time_series_replay -- --nocapture
```

The opt-in replay test runs only when `GK_OCT30_REPLAY_IN` is defined; ordinary `cargo test -p app --release` stays fast. Existing saved output paths are **not overwritten**. First inspect baseline A's stationary calibration QC, accepted-fit count, and full CT vector disagreement. **Only if A yields interpretable results** is a B sensitivity run warranted: set `$env:GK_OCT30_HYBRID_PRE="$HOME\\Downloads\\GK_CBCT_Results\\oct30_hybrid_PCB_only_for_gk_replay\\Oct30_pre\\detections_hybrid_PCB_only_REVIEW.json"`, change `GK_OCT30_SUMMARY_OUT` and `GK_OCT30_TRAJECTORY_OUT` to **fresh B filenames**, then repeat the same Rust test. If B fails registration it remains clearly flagged; do **not** relax CT-mapping or fit thresholds to obtain a nominal pass.

**Upload only `oct30_RUST_motion_summary_sanitized.json`** and the console test summary after verifying contents. Retain full Hall frames, private trajectories, and original detector coordinates locally. The next task is interpreting finite-cylinder fitting quality, wrap exclusions and CT endpoint errors—not more DICOM geometry engineering.

## Validation state

- Prior: Windows app `cargo test -p app --release` **43/43 passed** before adding the opt-in replay test; pre/post hybrid JSON preparation and 11 Python geometry tests succeeded.
- New Oct30 NPZ exporter, CT target synthetic tests and opt-in Rust recorded-motion test are **committed but not yet exercised on the user's local original data or Windows toolchain**. Do not claim a successful 6-DOF replay or measured accuracy until that run is available.

## FIRST REAL RUN — Original Oct30 Rust baseline A results (2026-11-04)

**Actual user-produced `oct30_RUST_motion_summary_sanitized.json` received and analyzed.** The opt-in `experimental_oct30_recorded_gk_motion_time_series_replay` completed baseline **A_original_CT_KiCad_prior**: original optimized CT source, 16 superior chip-first sensor estimates, authenticated KiCad/JST orientation. The output states research-only and `live_fitting_activated=false`; neither real-time live throughput nor HDMM accuracy is established.

| Oct30 endpoint (independently registered CT) | CT | Rust Hall A | Error |
| --- | ---: | ---: | ---: |
| Rotation angle | 1.71272924° | 1.83516697° | 0.12243773° magnitude |
| Relative 3D rotation | registered CT orientation | Rust estimated orientation | **0.16340485° geodesic** |
| Magnet-centroid translation magnitude | 0.51569266 mm | 0.45902421 mm | 0.05666844 mm magnitude |
| Full magnet-centroid translation vector | [−0.031424,−0.037253,+0.513385] mm LPS | [−0.065886,−0.020790,+0.453795] mm LPS | **0.0707783 mm Euclidean** |

Hall minus CT translation components: **[−0.034462,+0.016462,−0.059589] mm**, displacement direction error approximately **5.0°**. All errors refer to the **magnet centroid** and the registered CT LPS frame, not nose or HDMM marker motion.

**Runtime/fit counts:** 4146 recorded raw frames (1382 per raw board), 10-frame fixed fit stride, **414 attempted, 414 accepted, zero rejected or solver failures**. 36 fitted initial-plateau poses and 84 fitted final-plateau poses enter medians. Mean accepted finite-cylinder field residual RMSE **0.3496333 mT**. The `>8-mT` transient or `>10-mT` endpoint jump rule excluded board0 sensors **12/13** and board2 sensors **14/15**, leaving **44** sensor vectors for pose fitting; **all 48** still contribute to initial stationary calibration by current method. No digital unwrapping performed.

**Calibration limitations:** The run did **not** use the separately recorded no-magnet background (`measured_background_used=false`); it used `fit_strengths_with_board_offsets`. Stationary residual **0.4364452 mT** with **0.5000 mT** QC threshold; signed strength estimates **[−1667.72,+1557.10,+1328.48] mT**, in the screening range 300..2200 mT. Hence the basic research QC passes but is near its residual limit; fitted field offsets may absorb geometry, background or axis biases. Do not claim independent chip-center calibration. One pre/post CT pair cannot validate the motion trajectory between endpoints or predict clinical accuracy.

**Narrow follow-up:** Baseline A has yielded a valid, interpretable retrospective endpoint result, so an **optional B** run is now justified as a controlled sensitivity test. Use the same private replay input, masks, fitted model, and fixed stride; set `GK_OCT30_HYBRID_PRE` to the pre-existing hybrid PCB-center-only review JSON and select fresh filenames for summary and trajectory. B must pass unchanged CT-to-firmware registration and must preserve A's exact firmware index order; if not, record the fail, don't tune it. A supplementary background-aware run is worth considering only with a *verified*, matched original no-magnet log: the earlier suggested name in Downloads was a guess and the current summary confirms no measured background was used. Do not confuse A/B geometry sensitivity with background method sensitivity, and do not re-open DICOM localization experiments.

## Actual Oct30 Rust replay A/B outcome — hybrid PCB center does not improve independent CT motion (2026-11-04)

The user ran the **real Oct30 A/B time-series Rust replay** and supplied `oct30_RUST_motion_A_B_summary_sanitized.json` (`gk_oct30_rust_recorded_motion_replay_summary_v1`). Both arms passed the existing CT→firmware registration (including unchanged sensor mapping), stationary calibration QC and **414/414** motion fits, with **zero** rejected fits/solver failures, identical 10-frame stride and source data. This is **replay performance on one recorded movement**, not an independently validated live tracking system.

| Quantity | Original optimized CT + KiCad superior prior (A) | Hybrid superior PCB center only (B) | B minus A |
| --- | ---: | ---: | ---: |
| Independent CT rotation target | 1.71272924° | same | — |
| Estimated rotation magnitude | 1.83516697° | 1.83667276° | +0.00150579° |
| **Full 3D rotation geodesic error vs CT** | **0.16340485°** | **0.16355757°** | **+0.00015273°** |
| Independent CT centroid translation magnitude | 0.51569266 mm | same | — |
| Estimated translation magnitude | 0.45902421 mm | 0.45985303 mm | +0.00082882 mm |
| **Full translation vector error vs CT** | **0.07077830 mm** | **0.07123993 mm** | **+0.00046163 mm** |
| Translation magnitude-only error | 0.05666844 mm | 0.05583963 mm | −0.00082882 mm |
| Stationary field RMSE | 0.43644522 mT | 0.43363725 mT | −0.00280797 mT |
| Mean accepted motion field RMSE | 0.34963326 mT | 0.34879918 mT | −0.00083408 mT |
| Fits accepted/attempted | 414/414 | 414/414 | no change |
| Measured original no-magnet background used | **No** | **No** | no change |

Both arms share raw board motion exclusions **board0 sensors 12/13; board2 sensors 14/15; none for board1**, giving **44 Hall sensor vectors** for motion fitting. No digital unwrapping was attempted. All original 4146 frames were ingested. The full signed magnet-strength fits differ mainly in the third coefficient: A **[−1667.7208, +1557.1006, +1328.4798] mT**, B **[−1667.7266,+1557.1020,+1331.0448] mT**, showing that the magnetic nuisance calibration partly adjusts to the geometry alternative. The two estimated LPS translation vectors differ by only **0.004985 mm** Euclidean. Original A's LPS translation is **[−0.0658863, −0.0207902, +0.4537951] mm**; B is **[−0.0659267, −0.0159126, +0.4548244] mm**.

**Decision:** The hybrid PCB midpoint **does not provide a measurable practical accuracy improvement** for the Oct30 test movement. B reduces field residuals slightly (<0.003 mT) but is **very marginally worse on both independent 3D CT comparison metrics** (0.00015° geodesic; 0.00046 mm translation-vector difference). These tiny differences are **not statistically or physically meaningful** in a single-movement test. The increased precision of output decimals is computational, not validated measurement resolution. **Keep A as the default experimental GK geometry; archive B solely as a completed sensitivity analysis. Do not integrate the hybrid PCB center into the default live geometry, and do not restart CT localization research.** B passed same-channel correspondence, so this is a meaningful controlled research comparison, although each arm independently recalibrates magnet strengths/board offsets and the source PCB midpoint remains unvalidated.

**Next limited GK experiment work:** Analyze temporal stability and residuals from the **already exported original Oct30 trajectory** and, only if the correct original session background log is verified, repeat **A with measured background** as a separate calibration-method comparison. Keep the fixed CT 3D endpoint reference, signed axis mapping, exclusions, and solver thresholds; do not tune to the CT target. One movement with only pre/post CT cannot establish intermediate-time tracking accuracy, robustness across poses, or HDMM equivalence. No new hardware acquisitions required to complete retrospective analysis.

