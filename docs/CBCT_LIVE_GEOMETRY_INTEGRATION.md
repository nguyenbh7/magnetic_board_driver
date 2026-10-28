# Implemented desktop DICOM/CBCT geometry import (2026-10-28)

**This section supersedes the original proposed-architecture status
farther down this historical document.** The target scope is **INITIAL
GEOMETRY** (not magnetic calibration). New live-app code:

- `app/src/cbct_geometry.rs`: validates the established
  `gk_cbct_detections_v1` Python output. Computes the **centroid
  of all three detected magnets as the initial rigid-body origin**
  in DICOM LPS millimeters, stores each magnet's relative
  center and unsigned CT axis, stores **all 48 CT chip positions**
  in detector order, and stores the detected glass BB
  position/offset from the magnet centroid.
- `app/src/main.rs`: adds three controls to the **Live
  Tracking → Rigid 6-DOF** panel:
  **Import DICOM ZIP**, **Import DICOM folder** and
  **Open CT detections.json**.
  A fourth **Copy all CT positions** button exports a
  tabular clipboard listing of all 48 detector-indexed
  chips, magnets, and glass BB in DICOM LPS mm, with
  offsets relative to the three-magnet origin.
- Importing a DICOM series/ZIP launches the maintained
  `hall-effect-motion-tracking/tools/cbct_bridge/cbct_detect.py`
  as a **separate Python subprocess in a blocking worker**,
  never on the iced UI event loop or the real-time
  fitting thread. The detector writes original QC
  images/JSON into a **unique, local** directory
  `%LOCALAPPDATA%\magnetic_board_driver\cbct_imports\import_*`
  and its output is immediately parsed into the
  initial-geometry preview.
- **Open CT detections.json** avoids CT processing
  entirely, and is the fastest way to see the
  **already processed October 23 scans** inside the app.
  Example: `%USERPROFILE%\Documents\CBCT_Results\scan1\detections.json`.
  It loads positions, origin and BB without changing
  the selected live magnetic profile. The second scan
  can be opened separately (do not merge its
  different DICOM Frame of Reference UID).
- The detector expects **three magnets, three 16-chip
  arrays, and exactly one glass BB**, reflecting the
  established October phantom detector. It requires a
  nonempty DICOM FrameOfReferenceUID, valid/finite
  positions, normalizable magnet axes, and each board
  grid fit RMS ≤1 mm, refusing incomplete scans.
  Sensor indices 0…15 remain **CT DETECTOR order**,
  not firmware-indexed; do not infer board wiring,
  magnet polarity or field-axis calibration here.
- **HDMM sticker caveat:** The existing detector
  identifies a **glass BB**, NOT an independently
  detected/reflected HDMM nose sticker. The app
  explicitly labels the BB as unverified for HDMM
  and shows its relative offset without automatically
  assigning it to the HDMM target. A sticker-specific
  detection/identification (or operator-confirmed
  BB surrogate) is a *separate follow-up*.
- This is **read-only CT geometry initialization**:
  importing does NOT call
  `RigidLiveFit::set_profile()`, does NOT generate
  an approved `RigidProfile`, and does NOT
  replace/reset the A1/other calibrated live profiles.
  The rigid body's magnetic **6-DOF tracking**
  requires independently verified signed field
  axes/background/magnet strengths and remains
  deliberately separate from today's goal.

**Initial orientation and BB lever arm:** The app also constructs a
proper, right-handed **rigid-body coordinate frame** from
the named CT magnet centers: +X points from the Left
magnet to the Right magnet; +Y points toward Superior
after Gram–Schmidt orthogonalization; +Z is +X × +Y.
The origin is the mean of all three magnet centers.
This yields a well-defined baseline orientation
and each magnet/BB's offset in **both DICOM LPS
and body-local coordinates**. A collinear
or ambiguously named magnet constellation is
rejected. BB-local offsets are *candidate lever
arms*, **not a validated HDMM nose point**.

## Build / demo on the Windows development PC

```powershell
Set-Location 'F:\GitHub\magnetic_board_driver'
git switch feature/live-rigid-body-fit
git pull --ff-only
cargo test -p app
cargo run -p app --release
```

Select **Live Tracking → Rigid 6-DOF →
Open CT detections.json**, then choose
`C:\Users\Brandon Nguyen\Documents\CBCT_Results\scan1\detections.json`.
The panel should show three detected magnets,
their three-magnet center of mass (geometric
centroid, not a mass-weighted center of mass),
the three boards and 48 chip locations, the
glass BB and BB-to-rigid-origin offset.
Click **Copy all CT positions** to inspect
all individual sensor positions externally.

To use the ZIP/folder buttons, the two
sibling repositories must be at
`F:\GitHub\hall-effect-motion-tracking` and
`F:\GitHub\magnetic_board_driver`.
If not, set `CBCT_DETECT_SCRIPT` to the
full path of `cbct_detect.py`. The Python
environment must have the detector's
dependencies from
`hall-effect-motion-tracking/tools/cbct_bridge/requirements.txt`;
if needed set `CBCT_PYTHON` to a specific
virtual-environment `python.exe`. The DICOM
folder should contain the complete series;
individual `.dcm` files are not accepted
as stand-alone imports. The output is
**not committed to either Git repository**.

There are **seven new Rust unit tests** for
CBCT import schema, 3/3/1 object counts,
origin/initial coordinate-frame and marker local offsets, all-48-chip
clipboard export, missing DICOM frame,
incomplete detection, collinear magnets, and invented HDMM
object rejection. **Build and GUI smoke
testing on Windows are still pending.**
Do not report this as compiled or
end-to-end tested until the user runs
`cargo test -p app` and verifies an import.

**Acceptance test for today's scoped goal:**
the app imports the user's actual
`scan1/detections.json`, displays finite
three-magnet centroid plus three relative
magnet centers, all 48 CT chip coordinates
and one BB offset. It labels sticker
identification unverified and leaves any
active magnetic `RigidProfile` unchanged.
The ZIP/folder buttons are an optional
full-detector end-to-end test.

## Windows CBCT full-field static screen tests: 69/69 pass (2026-10-28)

The user confirmed **69/69 passing CBCT-bridge tests
in 0.171 seconds** on Windows, including the nine
new tests for static three-magnet finite-cylinder
field evaluation, `MLXBF002` parsing, corrected v2
geometry and field-axis transformation guards and
spatially held-out board backgrounds.

**No October real-field residual report has been supplied yet.**
These tests cannot verify October magnetic polarity, axis
signs, board identity, Hall-to-CT pose pairing or a live
`RigidProfile`. Await the unapproved two-CBCT real-data
`three_magnet_static_screen_v1.json` results for
evaluation against the background-only reference and
historical May15 raw-board assignment. Keep all
calibration approval gates false.


# CBCT-assisted geometry for the live six-DOF fitter — integration design

**Historical design note (2026-10-27):** The import UI and detector subprocess are now implemented as an uncompiled initial-geometry preview; magnetic profile generation remains separate and unavailable.

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

## Real cross-scan chip mapping audit: detector order stable (2026-10-27)

The user ran the research-repo `audit_cbct_sensor_indexing.py`
on both real CBCT geometries and the two candidate Hall-mapping
reports; **22/22 bridge tests passed on Windows**.
After registration of assumed stationary board centers (RMS
**0.0075 mm**), best CT-chip index correspondence was
identity on all three boards, with physical chip RMS
**0.1789 mm Right**, **0.1518 mm Left**, **0.0200 mm Superior**.
The nearest competing D4 mapping was approximately **10 mm**
RMS for each board. The CT detector's chip numbering therefore
does **not** explain raw2's discrepant candidate firmware mapping.

Raw0→Superior and raw1→Right *candidate* per-sensor indices
agree across the two magnetic screens, but raw1 magnetic
correlation is weak (~0.45–0.48). Raw2→Left ranked index
mappings disagree by **10.044 mm RMS**: `rot90_flip` in
Scan 1 versus `rot270` in Scan 2. Importantly, `rot270`
was Scan 1's second-ranked raw2/Left permutation (r≈0.777
versus best r≈0.804), making it a *plausible* common
candidate, not verified firmware indexing.

**No Rust live profile is approved.** The same ~4.46 s
stationary Hall log was compared against CBCT scans with
slightly different magnet poses. Confirm which scan
corresponds to the Hall acquisition before attempting
a full vector-field calibration. Independently verify
board IDs and firmware indexing against hardware
connectors, then signed magnetic axes/polarity/background,
finite-cylinder model and static/moving fit parity.
The cross-scan index audit is reproducible in the
research-repo bridge README; keep scan-derived reports
outside public Git.

## October 23 CT acquisition chronology / Hall pairing gate (2026-10-27)

Source CT DICOM `(0008,0022)` and `(0008,0032)` tags,
checked on multiple CT slices, report **Scan 1 at 15:07:10**
and **Scan 2 at 15:15:20** on 2026-10-23, an
**8-minute-10-second** interval. Timezone and any drift
between the scanner and Hall-recording computer have not
been verified. The `MLXBF002` Hall log contains only
device-relative microsecond timestamps (duration
~4.46 s), not an independently proven scanner-relative
wall-clock acquisition time; a DICOM-to-Hall pairing is
still **unresolved**. The scans' ~1.6° rigid magnet
pose difference means that screening the same raw Hall
frames against both geometries is a sensitivity check,
not two calibrated field-validations.

**Do not mark either scan as a matched Hall baseline or
load an approved `RigidProfile` until the log's physical
pose and capture time are corroborated**, firmware board
identity/chip indexing is verified, and the three-magnet
finite-cylinder vector-field forward model is validated
against an appropriate baseline with signed Hall axes,
magnet strengths and offset handling. File-system
creation/modification times of the original Hall binary
may be useful supporting evidence but are not reliable
after copy/restore. Details, exact acquisition times
and follow-up actions are recorded in the research-repo
`tools/cbct_bridge/README.md`.

## Oct27 local Hall timestamp check cannot establish Oct23 pairing

Windows `Get-Item` on the user’s downloaded copy of
`mannequinn measurement 10_23_2026.bin` reported
CreationTime and LastWriteTime both **2026-10-27 13:35:03**.
These times postdate the two original Oct23 DICOM acquisitions
and are consistent with copying or downloading; they are not
valid measurement timestamps. The scanner-to-Hall pairing
remains unestablished. Do not infer CBCT match from this
file’s filesystem metadata or auto-approve a live profile.
Source-machine logs/metadata and operator notes would be
appropriate provenance; meanwhile verify physical firmware
board identities/JST orientation independently.

## Overwritten October Hall recording: return to validated May-15 hardware prior (2026-10-27)

The user reports that the latest October-23 Hall recording was
**overwritten** and that the physical board connections should be
unchanged from May 15. The May-15 physically validated assignment is
**raw0=Top/Superior, raw1=Left, raw2=Right**. Treat this as a
*provisional prior* for October, NOT a reconfirmed mapping.
The October one-magnet magnitude-screen ranking
`raw0→Superior, raw1→Right, raw2→Left` is unapproved,
depends on Hall file content with unresolved provenance, and
must **not** override the original physically verified mapping.

No Hall log is currently established as a contemporaneous,
uncorrupted, correctly associated October scan baseline.
The CBCT detector geometry and two-scan consistency remain valid,
but background, magnet strength/polarity, board vector transforms
and raw-to-CT sensor indices are **not approved for live fitting**.
Preserve the generated calibration template with false verification
flags. Verify the October physical cabling/JST and KiCad sensor
index orientation independently, then calibrate with an
appropriate matched Hall acquisition.

**Recording-loss safety:** The `mlx-live-fit` and
`feature/live-rigid-body-fit` branches now have an
atomic create-new `MLXBF002` logger and explicit
stop-drain/flush behavior (see `app/LOGGING_WORKFLOW.md`);
testing on the actual Windows acquisition PC is still required.
The separate lab-production `gk_measurement` branch still
uses the older `File::create` raw/A-B logger and must not
be represented as overwrite-protected. Port/test that
branch's raw and companion file protection before
future acquisition, or use the verified newer logger.
For future paired CT-Hall datasets, record a unique
run ID and actual acquisition time, retain the raw file
and an independent backup, and do not reuse a selected
filename.

## CT-only candidate firmware chip indexing / May-15 board prior (2026-10-27)

The user clarified the surviving Hall measurements are from the
**October 23 CBCT experimental session**; the intended latest
recording was overwritten, however, so the surviving file's
exact baseline pose and relationship to the two CT images
are not established. User expects **physical cabling unchanged
from May 15**: raw0=Superior, raw1=Left, raw2=Right.
This remains an explicit provisional October hardware prior,
not automatically verified physical wiring.

The research repo now contains:
`tools/cbct_bridge/propose_ct_sensor_order.py` to infer
**unverified** 4×4 chip-index candidates geometrically from
the CBCT chip-side normal and PCB center versus chip centroid,
using the documented KiCad convention (connector toward
viewer/front; sensor0 bottom-right; up a column then left).
For existing October `detections.json`, the missing
PCB center is read from the corresponding `objects.csv`.
Future detector output includes `board_center_lps_mm`
directly in the JSON.

`tools/cbct_bridge/stage_index_candidates.py` optionally copies
these geometrical permutations and the explicit May15 board
IDs into a **new** `calibration.candidate.json` (outside Git);
it never modifies the source calibration review, never overwrites
an existing candidate file, and retains
`reviewed_and_approved=false` and
`firmware_mapping_verified=false`. Other magnetic
calibrations (signed field-axis transform, offsets,
remanence/polarity, effective background) are not fabricated.
The Rust `RigidProfile` export must continue to fail closed.

**Physical verification still required:** Determine that the
specific October PCB orientation, chip-side normal and actual
JST connector end match the geometry-based approximation,
check firmware board 0/1/2 physical wiring, and assess
the true field vectors using an appropriate matched
three-magnet calibration. The old May15 sample permutation
is not automatically transferred to a separate CT scan.
The follow-up `compare_geometric_sensor_orders.py` independently
registers the CT board centroids (Scan 1 to Scan 2) and checks that
each proposed firmware sensor number resolves to the same physical
3-D chip position, reporting per-board RMS/max positional differences.
The candidate staging script never modifies the original calibration
and uses exclusive file creation for the new review copy. These
checks verify **cross-scan consistency**, not physical board ID.

Six synthetic ordering, five staging, and three cross-scan
candidate-comparison tests were added and require Windows
execution; the previous 22-test suite passed. **The new
expected count is 36 tests.** See the research repo README
for the complete Windows PowerShell commands and safety rules.

## Oct23 CT/JST index proposals consistent in both scans (2026-10-27)

The user executed the research-repo `propose_ct_sensor_order.py`
against Scan 1 and Scan 2, then the cross-scan
`compare_geometric_sensor_orders.py`. **All 36 Windows
bridge tests passed in 0.073 s.** Identical candidate
16-chip firmware-to-detection mappings emerged in both
CBCT scans:

- Right: `[12,13,14,15,8,9,10,11,4,5,6,7,0,1,2,3]`
- Left: `[3,2,1,0,7,6,5,4,11,10,9,8,15,14,13,12]`
- Superior: `[12,13,14,15,8,9,10,11,4,5,6,7,0,1,2,3]`

With three assumed-stationary board centers registered
between scans (registration RMS **0.0075 mm**), chip
RMS differences are **0.1789 mm Right**,
**0.1518 mm Left**, and **0.0200 mm Superior**;
max differences 0.3808/0.3821/0.0372 mm.
These measurements reflect repeatability, **not absolute
CT positioning accuracy or verified firmware indexing**.

Original face-on CBCT QC images show the
connector/cable end opposite the chip cluster on each
board, supporting the board-centroid-to-chip-centroid
direction used for the provisional index calculation.
They do **not** show actual firmware sensor-zero
or determine the signed raw Hall axes.

Next: the research-repo `stage_index_candidates.py`
can produce a **separate** `calibration.candidate.json`
with these unverified chip permutations and provisional
May15 physical board IDs (raw0 Superior, raw1 Left,
raw2 Right). The original `calibration.review.json`
is unchanged and all required approval flags remain
false, so `RigidProfile` export must still fail.
Verify the physical JST/PCB index convention, signed
raw-to-LPS field transform, Hall-element offset,
magnet polarity/strength, and background against
a matched measurement before live fitting.

## Unapproved Scan 1 candidate staged; distinguish current vs legacy 4×4 axes (2026-10-28)

User confirmed successful creation of
`CBCT_Results/scan1/calibration.candidate.json`
with provisional physical IDs **Right raw2, Left
raw1, Superior raw0**; all `firmware_mapping_verified`
flags remain **false**. Original
`calibration.review.json` is intact and the
live Rust `RigidProfile` builder remains gated.

A source audit uncovered a significant convention
difference between **this repo's current**
`data_transfer/src/rpc.rs:SENSOR_POSITIONS_MM`
(sensor0 (-6.75,-6.75), sensor1 (-6.75,-2.25),
sensor4 (-2.25,-6.75) mm; +Y by sensor,
+X by four sensors, column-first) and the
historical May15 `INTENDED_OLD_4X4_POSITIONS_MM`
(sensor0 (+6.75,-6.75), sensor1
(+2.25,-6.75), sensor4 (+6.75,-2.25)
mm; row-first).

The two centered planar layouts differ by a **proper
90° in-plane rotation**, so a geometric CT Kabsch
fit alone cannot disambiguate them. The Oct23
candidate geometry/indices were constructed from
the firmware's **column-first** convention, but
the October signed `raw_to_world` Hall-field
matrix is **still unknown** and MUST NOT be copied
from historical May15 profile transforms without
addressing this frame convention. Raw chip vector
axes can additionally differ from nominal board-local
axes; verify against firmware/raw-field instrumentation.

The research-repo
`tools/cbct_bridge/audit_ct_local_frames.py`
now computes both candidate local-to-LPS matrices,
residuals, relative rotation and board-normal dot
products as an explicitly unverified report.
Seven new unit tests are committed; 43 tests total,
with 36 previously passed on Windows. This utility
does not set calibration approvals or load the Rust
fitter. Field-axis parity and full three-magnet
baseline validation remain necessary.

## Windows CT local-frame regression verification: 43 passed

The user confirmed the complete research-repository CBCT bridge
suite passed on Windows: **43 tests in 0.066 seconds**,
including the seven local firmware-versus-May15 frame-convention
unit tests and fail-closed calibration checks. This is **software
validation, not physical magnetic-field calibration**. The
real-data `audit_ct_local_frames.py` report on October Scan 1
has not yet been supplied, so its candidate rotation matrices
and CT chip fitting residuals have not been reviewed.
The Rust live profile must remain unapproved until the actual
signed raw Hall axes, chip-active-element offsets, background,
magnets, and recording/geometry correspondence are validated.

## Scan 1 board-local frame candidates audited; magnetic axes remain unknown (2026-10-28)

The user's real Scan 1 `audit_ct_local_frames.py` output
returned a **90.000°** current-firmware-versus-legacy
May15 local-grid rotation on all three boards.
Both conventions have identical geometry RMS:
**Right (raw2) 0.1791 mm; Left (raw1) 0.2397 mm;
Superior (raw0) 0.1635 mm**. The provisional
current-grid +Z directions have dot product
**−1.000** with the CT-reported chip-facing normals.

This negative sign is **expected by construction**:
with JST down in the chip-facing view the current
firmware's +X points picture-left, +Y up, and
right-handed +Z away from the chips. The indexing
proposal explicitly used the CT normal to define
picture-right, so `−1` is NOT independent PCB
orientation validation or an MLX90393 `B_z`
sign verification.

The first report printed its numerical matrices
but raised `FileExistsError` when saving because
`local_frame_candidates.json` already existed
and the tool uses exclusive output creation.
Source-tool error reporting was improved; no
original file was overwritten. The research repo
now contains three more local-normal sign tests:
**46 expected**, with **43 previously passed**
on Windows. The candidate rotation matrices
are **geometric basis matrices**, not approved
`raw_to_world` signed Hall-axis transforms.
The current Rust fitter and its calibration
gates must not accept them as verified.

Next validate actual mounted PCB/chip-axis conventions,
signal polarity, magnet strengths/background and
pose-matched Hall frame predictions before live fit.

## 46 passing CBCT tests; traced raw MLX field components (2026-10-28)

All **46** research-repo CBCT bridge regression tests passed
on Windows in **0.069 s**, including new board-local axis
interpretation and protection tests.

The raw `MLXBF002` logging data path was audited in
the current Rust firmware:
`firmware/microcontroller/src/mlx90393/sensor.rs`
reads sensor X/Y/Z measurement bytes into distinct
MLX field components;
`data_transfer/src/conversions.rs` converts each
component to µT with its corresponding `Axis::X/Y/Z`
sensitivity, and
`firmware/microcontroller/src/handlers.rs` stores those
values directly in `BoardFrameSample.bx_ut/by_ut/bz_ut`.
There is **no extra component permutation or 3×3
rotation in the inspected logging path**. This finding
depends on which firmware was actually flashed on
the October 23 rig.

The physically mounted MLX chip/package axes relative
to the JST/PCB reference frame remain **unverified**;
the local 4×4 grid-to-CT geometric rotation does not
alone establish the sensor-reported magnetic field
axes. The historical May15 nominal coordinate
system differs by 90° in plane; never transfer
historical `raw_to_world` rotations without
reconciling coordinate frames and physically
verifying signed Hall axes. Record independent
PCB pin-1 / chip-axis evidence and check a known
signed field direction before calibrating the
full three-magnet finite-cylinder model.

Research repository method and limitations:
`hall-effect-motion-tracking/tools/cbct_bridge/HALL_AXIS_PROVENANCE.md`.
The October `calibration.candidate.json` remains
unapproved and the Rust `RigidProfile` validator
must remain fail-closed.

## Existing KiCad library source recovered (2026-10-28)

Earlier documentation incorrectly implied that the
user had not shared the physical KiCad PCB design.
ChatGPT Library actually contains user-uploaded
`/Hall Effect Motion Tracking/MLX90393_board3-2025-06-02_225333.zip`
(September 22) and `/MLX90393_board4.zip`
(September 25). The **later** ZIP contains a complete
`MLX90393_board3.kicad_pcb` with **16 placed
MLX90393-011 QFN footprints** in a 4×4, 4.5-mm
grid, all 180° on F.Cu, plus three JST and four
mounting-hole footprints. It also contains schematics
and production footprints. The inner project retained
the `board3` name despite the outer archive being
`board4.zip`. The earlier older ZIP lacked placed
sensor footprints when inspected previously.

These original board files are in **Library**, not
committed to Git; Library uploads do not automatically
synchronize to either repo. Do not publish electronics
CAD/manufacturing files to public Git without
the user's direction. This source now permits a
KiCad pin-1/footprint-versus-datasheet MLX magnetic
axis audit, but exact October manufactured board
revision and sensor index/address correspondence
still require confirmation. See research repo
`tools/cbct_bridge/HALL_AXIS_PROVENANCE.md`.

## Critical correction: KiCad confirms v1 firmware-index mirror (2026-10-28)

The archived populated `MLX90393_board4.zip`
and current driver `data_transfer/src/rpc.rs`
conclusively expose a **software convention error**
in the research-repo `propose_ct_sensor_order.py`
v1: firmware sensor **0 is bottom-LEFT**
(JST down, facing the chips) rather than
bottom-right. The current firmware diagram is:

```text
 3   7  11  15
 2   6  10  14
 1   5   9  13
 0   4   8  12
```

The older v1 CT candidate mappings
looked repeatable between two CT scans
but were **mirrored across the chip grid**.
Their `-1` local +Z vs CT normal was a
consequence of the mirrored indexing;
in the corrected proper firmware frame,
local +Z is expected to align **+1**
with the CT chip-facing normal, largely
by construction. The v1 indexed
`calibration.candidate.json` and
`local_frame_candidates.json` **must not be
approved or fed to any magnetic model**.

The research bridge now generates an
explicit versioned **v2** proposal with
firmware-verified bottom-left ordering.
`stage_index_candidates.py`,
`compare_geometric_sensor_orders.py`
and `audit_ct_local_frames.py`
reject the obsolete v1/missing metadata.
The corrected total is **51 new/current
regression tests**, pending a Windows run.

The source KiCad project's 16 MLX90393 QFN
footprints are all on F.Cu at 180°, with
pad 1 in the pin-1 position shown in the
Melexis Figure 26 top view. Matching
signed axes to actual PCB orientation
yields the **provisional** chip-to-board
rotation `[By,-Bx,Bz]` (proper rotation,
det +1), consistent with the A1 vector
convention already used in this Rust code.
The raw **Hall-to-DICOM** transform is
still unapproved and must use fresh CT
v2 grid-fitting matrices and independent
magnetic validation.

Editable project source has been packaged
into 21 files, and this repository now
contains `hardware/kicad/MLX90393_board4/README.md`
and `.gitignore`. **Actual KiCad source**
requires local extraction / Git push
from the delivered source-only ZIP;
GitHub connector source text write access
cannot transmit the large PCB source.
The source ZIP includes no autosave/backups
or generated Gerbers. Do not mistake the
README existing for successful asset upload.

## Windows 51/51 corrected-index regression tests passed (2026-10-28)

User ran the research-repo CBCT bridge test suite
after correcting firmware sensor 0 to **bottom-left**
in the JST-down chip-facing view, versioning the
unapproved proposal as v2, and rejecting the
superseded mirrored v1 outputs. **51 tests passed
in 0.079 seconds** on Windows.

This is software regression verification only:
the actual October Scan 1/Scan 2 v2 positional
comparison and v2 local-frame output have
not yet been supplied or reviewed; raw Hall
field-axis transformation and magnetic profile
approval remain unverified and fail-closed.
The KiCad source ZIP is prepared for user
local import, but the editable PCB source has
not yet been confirmed committed/pushed to
this repository.

## Real corrected v2 Oct23 CT results; unverified signed-axis hypothesis (2026-10-28)

User-supplied **real Oct23 v2** Scan1/Scan2
CT sensor-order comparison confirms both scans
assign the same candidate indices to all 48 chips:
**Right raw2 identity, Left raw1 [15…0], Superior
raw0 identity**. Stationary-board centroid
registration RMS **0.00752885 mm**.
Cross-scan chip RMS/max differences:
**Right 0.17887/0.38077 mm**, **Left
0.15180/0.38210 mm**, **Superior
0.01999/0.03721 mm**.

Scan1 corrected **grid +Z versus detector chip-normal**
dots are **+0.999973 (Right), +0.999998 (Left),
+0.999683 (Superior)**, versus the old v1
mirror's -1; this agreement is partly a
construction result, not an independent
verification of magnetic field-axis signs.
Current nominal-grid CT RMS: Right
**0.17914 mm**, Left **0.23974 mm**,
Superior **0.16347 mm**; historical May15
local layout fits equally but rotates 90°.

Research repo now includes
`tools/cbct_bridge/propose_hall_field_axes.py`,
which combines the corrected v2 scan-specific
proper grid-to-LPS rotation with the **provisional
board4/Melexis chip-to-grid transform
`[By,-Bx,Bz]`**, refusing v1 files,
scan-UID mismatch, invalid/reflected rotations,
wrong physical raw IDs, contradictory board
normals and verified flags. It writes an
**unverified standalone diagnostic JSON**, not
a loadable Rust `RigidProfile`. Original
geometry/calibration files stay unchanged,
live export stays blocked. Nine new synthetic
tests bring the expected test count to **60**
(51 previously passed on Windows).

Remaining blockers: KiCad source files still
require the separate hardware-repo local
Git push, the fabricated Oct23 board revision
and firmware build must be checked, and the
preserved Oct23 Hall data require a physically
matched pose plus independently calibrated
background/magnet polarity and strength for
full three-cylinder vector-field validation.

## Hall-axis candidate regression suite passed 60/60 (2026-10-28)

User reported **60/60 passing tests in 0.094 seconds on Windows**
for the research-repo CBCT bridge, now including the guarded
KiCad/Melexis `[By,-Bx,Bz]` axis-hypothesis generator.
Synthetic tests check proper rotations, source/board IDs, UID
agreement, mirrored-v1 rejection and continued unapproved status.
They do **not** physically calibrate the signed raw Hall axes.

The actual October Scan1 `hall_axis_candidates_v2.json`
result has not yet been supplied for review. The earlier v2
CBCT chip matching and local-grid matrices were internally
consistent, but the original October Hall recording's exact
pose match, manufactured board revision, sensor offsets and
field-background/polarity evidence remain unverified. Do
not load or approve a live `RigidProfile` from those
candidate-only artifacts.

The editable KiCad PCB files were packaged for a local Git
import; the actual assets are not confirmed pushed merely
because their README and .gitignore exist.

## Candidate raw-to-LPS matrices and next Oct23 three-magnet screen (2026-10-28)

The real Scan 1 corrected-v2 KiCad-derived Hall-axis JSON
has proper 3×3 matrices (determinants ≈+1,
orthogonality residual <4e-15), with identities
for Right raw2 and Superior raw0, reversed
index order for Left raw1. Its chip-to-grid
candidate is `[By,-Bx,Bz]`. **Every magnetic
verification and review flag is false**;
proper rotation tests alone do not imply
physically correct signed axes.

The surviving October `MLXBF002` file was
independently decoded: raw0/raw1/raw2 contain
**153/154/154 frames** with all 16 chips
recorded. Median per-chip field magnitudes:
**4.66 / 7.03 / 5.36 mT**. The overwrite
means the binary is still not independently
pose-paired with either CBCT scan.

Research-repo
`tools/cbct_bridge/screen_three_magnet_static.py`
now evaluates the full signed vector field of
three finite cylindrical magnets across all
48 CT-localized chips, for each CBCT pose,
raw-board ID hypothesis, unsigned CT magnet
axis sign hypothesis and prespecified
remanence sensitivity, using spatially held-out
board-background nuisance estimates and a
background-only reference. **No independent
magnetic residual result exists yet.**
Nine synthetic tests bring the pending suite
to **69**; 60 previously passed on Windows.
Full method: `tools/cbct_bridge/STATIC_THREE_MAGNET_SCREEN.md`.

This screen remains *exploratory* and cannot
create or approve a Rust `RigidProfile`.
The editable KiCad PCB source still awaits
confirmation of the user's separate local
Git import/push.

## High-gain output-range hazard discovered in Oct23 Hall recording (2026-10-28)

The user's real fixed-CT-pose finite-cylinder screen has
**poor magnetic agreement**, despite corrected v2
PCB chip indexing and provisional `[By,-Bx,Bz]`
axis rotation: held-out RMSE **3.2172 mT** for
Scan1, **3.2269 mT** for Scan2, versus
**3.2901 mT** board-background-only.
The weak 1.9–2.2% RMSE improvement does
not support changing historically known
physical Right2/Left1/Superior0 board IDs,
magnet polarity, or CT pose correspondence.

**New high-priority warning from direct binary data:**
All **22,128** XYZ component values in the original
MLXBF002 file agree with **gain7, resolution0,
four-phase HALLCONF0xC** conversion factors
(95th-percentile nearest-count error 0.000968),
even though the current firmware `main.rs`
declares `mlx_sensitivity: {gain:0,
resolution:0,hall_conf:0xC}` as a default.
This suggests the runtime/app selection or
acquisition firmware differed from current
default and MUST be checked against register
readback, not guessed.

At gain7/res0/four-phase, signed digital output
limits are just **±4.9152 mT (XY) and
±7.929856 mT (Z)**. Adjacent chips on raw1
and raw2 show abrupt ~8–14 mT component
jumps consistent with a 16-bit wrap hypothesis.
Raw1 has **0/7/9** suspicious adjacent X/Y/Z
component edges, raw2 **7/6/4**, raw0 **0/0/0**.
A periodic nearest-difference calculation
reduces neighboring component RMS
raw1 **5.573→2.399 mT**, raw2
**4.296→2.393 mT**, without fitting any
magnet field. This is **not proof of actual
silicon wraparound** (clipping and analog
range behavior must be checked), nor a
unique physically valid unwrapping.
The previous static field comparison could
therefore be heavily distorted by **digital
range/aliasing issues** rather than incorrect
hardware geometry.

**Required firmware/application change for future
GK scans:** explicitly set a sufficiently
wide MLX digital range, preferably start at
`GAIN_SEL=0,RES_XYZ=0,HALLCONF=0xC`
(nominal ±24.61mT XY / ±39.65mT Z),
read back and save actual gain/RES/HALLCONF/
TCMP_EN per board/chip in acquisition
metadata, and reject or clearly flag
components near digital limits. Preserve
raw 16-bit measurements/settings when
possible, alongside converted µT data.
Any new static CT/Hall baseline must
have confirmed pose correspondence.
Do not silently unwrap the old recording
or approve any `RigidProfile`.

Research evidence:
`tools/cbct_bridge/OCT23_HALL_OUTPUT_RANGE_DIAGNOSIS.md`.
New `audit_mlx_output_range.py` flags this risk;
eight synthetic tests bring the expected
bridge count to **77**, not yet run.
Melexis Datasheet Rev012, Table 17/21.

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


## Live detector progress and cancellation (2026-10-28)

The Windows Live Tracking DICOM import now runs the Python detector with unbuffered stdout
(`python -u`). The detector emits `CBCT_PROGRESS:` stage messages for archive extraction,
CT volume loading, magnet and sensor-array search, glass BB search, and QC/output writing.
The Rust app reads those messages from the subprocess stdout on a separate thread and
updates a one-line status approximately four times per second. The UI also shows elapsed
time and an expandable log (including ordinary detector terminal output), retained after
completion or failure. The detector stages are indeterminate: **no artificial percentage**
is displayed, because heavy segmentation steps have no reliable work denominator.

**Cancel detection** requests termination of the Python child process and prevents importing
partial geometry. The app waits for the child to exit before enabling a subsequent import.
A canceled scan may leave incomplete QC files under the import-specific local results
folder; they are not loaded as live geometry. As before, even successful CT geometry
remains an **unreviewed preview** and never enables magnetic calibration or a rigid profile.

**Validation still required on Windows:** build the Rust app from
`feature/live-rigid-body-fit`, run a real scan using the adjacent
`hall-effect-motion-tracking` checkout on `a1-calibration`, verify stage updates
and log visibility, then cancel a second scan during volume processing and verify
that no geometry is imported. These changes have not yet been compile-tested or
replayed with the real DICOM archive in this environment.
