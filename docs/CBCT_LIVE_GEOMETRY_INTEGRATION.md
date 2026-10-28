# CBCT-assisted geometry for the live six-DOF fitter — integration design

**Status:** proposed architecture, 2026-10-27. No CBCT import, detector
subprocess, or automatic profile generation is implemented in the desktop
app by this note.

## Inputs and evidence

The collaborator's `find_magnets_arrays.ipynb` takes a DICOM CT **series**
(not necessarily a single DICOM file). It reads the CT volume using
SimpleITK/TorchIO and detects three magnets, three rectangular sensor
boards with sixteen chip locations each, and one glass BB. It writes
`objects.csv`, `sensors.csv`, PNG/segmentation QC, and retains
locations in DICOM patient **LPS millimeters**. It contains an
independent `run()` function suitable for refactoring into a Python
library and CLI.

Saved notebook output for 2026-10-23 CBCT phantom scans (not
independently re-executed during this design review) reports:
- Two CT volumes of 448^3 at approximately 0.5 mm isotropic voxels.
- Three boards per scan with chip-grid pitch 4.50 mm, grid-fit RMS
  0.04–0.22 mm.
- Board-array centroid displacement across the two scans approximately
  0.03–0.07 mm.
- Three magnet centroid displacements approximately 1.33–2.99 mm.
- One 6 mm glass BB, moving approximately 1.19 mm between scans.

These are **detector repeatability observations on this phantom**, not
independent physical-accuracy validation. Two scans have different
DICOM Frame of Reference UIDs, so numerical coordinate agreement alone
is not proof of a valid registration. Do not conflate the detected
glass BB with the actual HDMM reflective sticker unless it is
physically confirmed to represent that marker.

## Recommended architecture

```text
DICOM CT series/folder (or extracted ZIP)
            |
 Python CBCT detector (separate process)
            |
 detections.json + visual QC + DICOM provenance
            |
 board/magnet identity & physical calibration
 + optional manual review
            |
 validated RigidProfile JSON
            |
 Rust desktop app: Load geometry (.json)
            |
 real-time Rust six-DOF fitting
```

Keep CBCT image processing in Python: its SimpleITK, SciPy, TorchIO
dependencies and minute-scale processing do not belong inside
the real-time Rust optimizer or UI event handler. Initially run a
separate Python CLI and use the existing **Load geometry (.json)**
button in `app/src/main.rs`. Subsequently add **Import CBCT**
which spawns the Python CLI off the UI thread, shows progress, displays
the detections and validation report, and requires confirmation before
replacing the active calibrated profile.

Implement the reusable detector/CLI in
`hall-effect-motion-tracking` (e.g. `src/gk_analysis/cbct_geometry/`);
keep all Rust integration under `magnetic_board_driver`.
Use a versioned interchange JSON and a separately versioned validated
profile, never an unreviewed CSV-to-live-fit conversion.

## Required profile fields and the missing calibration bridge

The existing Rust `RigidProfile` is already deserialized by
`RigidProfile::from_json()` in `app/src/rigid_fit.rs`. Its inputs are:
- `name`, `coordinate_frame`, `origin_mm` (magnet centroid),
  `initial_pose`, six lower and upper bounds, `max_rmse_mt`;
- three boards with `raw_board_id`, exactly 16 `sensors_mm` in
  **firmware sensor-index order**, 3x3 `raw_to_world`,
  `background_mt`, and optional `sensor_gains`;
- three magnets with baseline `center_mm`, **signed** unit
  magnetization `axis`, physical `diameter_mm` and
  `thickness_mm`, and signed effective `remanence_mt`.

A CBCT detector can directly propose positions of magnet bodies and
board/chip packages, plus approximate board normals and unsigned
magnet long axes. It cannot by itself provide:
1. The mapping from anatomical Left/Top/Right boards to **raw board
   firmware IDs** and the 16 chip locations to **firmware indices**;
   the square 4x4 grid is symmetric under rotation and reflection.
   Validate using PCB layout/JST orientation, fiducials, and/or
   controlled sensor perturbation.
2. The actual Hall sensitive element's center relative to the imaged
   chip package: preferably fit each board's rigid pose to the detected
   grid, then transform CAD-defined sensing element coordinates.
3. The `raw_to_world` signed Hall-vector triad convention. A
   detected plane normal and grid do not on their own establish
   electrical sign conventions.
4. Magnet **polarity**, signed axis, accurate physical dimensions or
   remanence. CT metal blooming distorts sizes, and an unsigned
   geometric axis is not a magnetic polarity measurement. Use known
   magnet specifications, controlled polarity check and/or calibrated
   gaussmeter data.
5. Per-board field offsets, measured no-magnet per-sensor
   backgrounds, sensor gains. Select one documented background model
   consistent with live subtraction; avoid subtracting it twice.

### Coordinate frames

CT outputs DICOM LPS positions. Keep explicit frame metadata,
including Series Instance UID, Frame of Reference UID,
spatial transforms and algorithm version. Do not quietly reuse
`gk_may15_ct` or `gk_rotation_stage` for a new scan. Use a
new custom frame tag and convert vector components consistently
so sensor positions, magnet centers and magnetic fields all agree.

The existing setup preset is **not** a full calibrated reference for
a newly CT-scanned headrest; the live UI must not silently transfer
old presets' magnet positions into a new scan.

## QC and acceptance gates

- Require exactly three magnets, three physical boards, 16 detectable
  sensors per board and, if requested, one explicitly identified marker.
- Check each board's rigid 4x4 pitch, plane/fit residual and reconstructed
  CAD-to-image residual; compare pairwise magnet separations to plausible
  fixture geometry. Report every rejected candidate and why.
- Reject ambiguous board IDs, mirrored/rotated 4x4 index mapping,
  unverified field axis/polarity, mismatched frame UID or contradictory
  metadata. Require operator confirmation and saved overrides where
  needed. Detector confidence alone must not bypass these checks.
- Overlay candidate magnet centers, board indices 0–15 and marker
  coordinates on image slices / 3-D QC. No automatic activation
  before review.
- With a stationary real Hall recording, evaluate field residuals,
  per-board and per-sensor errors, correct magnet polarity and
  plausible pose/strength; compare offline and Rust fit on the same
  frames.
- Test known stage translations/rotations and a holdout scan, including
  remounts, reversed boards, missing chips, bright implants and
  multiple CT series. Treat subvoxel grid RMS as an internal
  consistency metric, not ground-truth positional accuracy.

## Glass BB, HDMM and predicted nose

The notebook finds a **glass BB**; its identity as the HDMM sticker is
not established by segmentation. The marker must be verified as
attached to the **moving magnet-bearing rigid body**. Where it
corresponds to the desired nose/HDMM point, extend the profile with an
optional `marker_mm` in the same CT LPS frame and compute

```text
marker(t) = origin + translation(t) + R(t) * (marker_0 - origin)
```

Also distinguish absolute CT position, displacement relative to
rigid zero, and HDMM coordinates. Comparing signed positions in HDMM
space requires an independently verified CT-to-HDMM transform, and
the two scans' separate DICOM reference UIDs require registration
before direct position comparison.

## October-23 real CBCT comparison (Windows validation, 2026-10-27)

The Python prototype in `hall-effect-motion-tracking/tools/cbct_bridge`
was run on both original October-23 scans. It detected 3 magnets,
3 boards × 16 chip positions, and one inferior glass BB per scan.
Annotated board and overview images passed a preliminary qualitative
visual check. Both scans' detected grids had 4.5 × 4.5 mm pitch and
in-plane grid RMS <= 0.222 mm.

The full-precision `compare_cbct_scans.py` run using actual
`detections.json` outputs gave **0.0075 mm stationary-board
registration RMS**, **1.5962° magnet rigid rotation**,
**0.0243 mm magnet rigid-fit RMS**, and **0.0150 mm held-out glass
BB prediction residual** (BB displacement **1.209 mm**). Two scans
had distinct DICOM Frame of Reference UIDs; three stationary-board
centers were explicitly registered before magnet motion estimation.
This is **internal geometric consistency, not CT/HDMM spatial
accuracy**, and the BB is not yet verified as the HDMM sticker.

**Next Rust integration gate:** do not auto-activate detected
geometry until October-specific raw board IDs, image-chip-to-firmware
index correspondence, signed field transforms, Hall element offsets,
magnet polarity/remanence and field backgrounds have all been verified.
The old May-15 assignment `raw1=Left, raw0=Top, raw2=Right` and
right-board `flip_rot180` are *historical*, not established for this
October mounting. The existing JSON-profile loader is ready for
reviewed data; the GUI DICOM import is still not implemented. See
the research-repo bridge README for reproducibility and provenance.

## October-23 stationary Hall mapping verification screen (2026-10-27)

A prototype `screen_hall_board_mapping.py` is now in the research
repository under `tools/cbct_bridge/`. It reads the October-23
`MLXBF002` recording (~4.46 s, 461 board frames, 16 sensors/board),
computes per-sensor median field magnitudes, and ranks all six
raw-board-to-CBCT-array assignments and all eight D4 4×4 sensor
permutations per board. It produces **diagnostic candidates only**:
the `AMBIGUOUS_UNVERIFIED` status is deliberate.

A first local exploratory comparison using the notebook's printed
Scan 1 chip coordinates found that the best two complete
assignments were essentially tied (sum of squared pattern
correlations around 1.79 and 1.78). At least one board had a weak
maximum match (~0.48). Thus **no October raw board mapping,
grid permutation or magnetic-field transform is verified** by
this recording alone. This is not evidence that the original May-15
mapping is wrong, or that the October detector failed.
A short stationary recording and an approximate dipole-field
magnitude screen do not account for uncertain CBCT-to-Hall
acquisition timing, backgrounds, gains, near-field cylinder
effects or polarity. Test full-precision Scan 1 and Scan 2
detections on the Windows machine; inspect candidates and
compare physical cabling/JST orientation and controlled
one-board-at-a-time perturbations. Only independently verified
calibrations should be exported to the live Rust profile.

## Two Hall-to-CBCT mapping screens: ambiguous, not approved (2026-10-27)

Using the **same** 461-frame October-23 Hall recording with the
separate Scan 1 and Scan 2 geometry exports, candidate raw board
assignments ranked first in both:
`raw0→Superior` (r=0.948/0.972),
`raw1→Right` (r=0.483/0.449),
`raw2→Left` (r=0.804/0.795).
Scan 1 top vs runner-up objective scores were nearly tied
(1.778724 / 1.778077); Scan 2 separated them more clearly
(1.776955 / 1.460707). Raw 1 remains weak and its
Scan 2 top two D4 sensor mappings are also effectively tied.

The proposed left-board D4 mapping differs between scans:
`rot90_flip` (Scan 1), `rot270` (Scan 2).
This could be a DICOM detector grid-ordering ambiguity rather
than physical board motion; do **not** approve either transform
yet. A new research-repo
`tools/cbct_bridge/audit_cbct_sensor_indexing.py`
compares both scans' 16 physical chip coordinates after board
registration to distinguish detector relabeling from inconsistent
candidate maps. Four synthetic tests have been committed but
the new script has not yet been run on the real scans.
The previous 18 bridge tests passed in Windows.

No mapping has been approved for the Rust live profile.
The magnetic pattern screen ignores full three-magnet
finite-cylinder vector fields, backgrounds, sensor gains,
field-axis transforms and physical polarity. The acquisition
is a short stationary recording, so both CBCT screening runs
are not independent confirmation. Physical board identification
and calibration remain necessary before direct CBCT import
can safely activate tracking.

## Staged implementation

1. **Python refactor:** isolate `load_scan`, magnet/board/BB detection
   and export functions from notebook into a tested library and
   `detect_cbct_geometry` CLI. Accept a DICOM series directory,
   optionally archive input with safe extraction, or a NIfTI file.
   Output JSON detections, QC, and a manifest; retain scan locally.
2. **Profile builder:** convert reviewed detections + board ID/index
   mapping + magnetic calibration to a fully validated `RigidProfile`
   using the existing Rust JSON schema. Reject incomplete inputs.
3. **Offline replay:** run the current historical raw-field
   measurements through both Python and Rust models using an
   equivalent generated profile, compare residuals/poses and test
   incorrect orientation/polarity as negative controls.
4. **GUI:** add **Import CBCT series** and asynchronous detector
   invocation, review, and **Approve / Load for fitting**. Never
   block the sensor stream or activate unvalidated detection.
5. **Marker:** add optional marker metadata and nose/HDMM lever-arm
   prediction, then independent HDMM registration and validation.

## Data governance

CBCT images and DICOM headers may contain patient identifiers.
Keep raw scans, intermediate images and detector results outside
public repositories unless properly approved/de-identified.
Record only appropriate provenance and reproduceable algorithms in Git.


## Implemented Python bridge (2026-10-27)

The Python bridge source is now maintained in
[`hall-effect-motion-tracking/tools/cbct_bridge/`](https://github.com/nguyenbh7/hall-effect-motion-tracking/tree/a1-calibration/tools/cbct_bridge)
on `a1-calibration`, **not** only as a ChatGPT ZIP. The module
`detector_core.py` contains the 50 importable functions extracted from the
collaborator's notebook; the folder also includes `cbct_detect.py`,
`make_calibration_template.py`, `build_rigid_profile.py`,
`generate_detector.py`, `requirements.txt`, `tests/test_bridge.py`,
and an operator README. Nine synthetic adapter tests passed locally.
The detector has **not** yet been validated by rerunning the two CBCT ZIPs
with the required SimpleITK/TorchIO dependencies.

The Rust GUI integration remains proposed; the current application already
supports manually loading the exported, reviewed `RigidProfile.json`.
No raw DICOM, scan outputs, calibration containing patient identifiers,
or unapproved geometry was committed. Next test on the lab machine:
`python -m unittest discover -s tests -v` from
`tools/cbct_bridge`, followed by both CBCT scan CLI runs and image QC.
