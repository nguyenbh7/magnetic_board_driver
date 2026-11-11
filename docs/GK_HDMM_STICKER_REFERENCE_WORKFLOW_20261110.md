# GK HDMM sticker reference, CT detection and cropped-scan recovery

**Status:** source implementation committed; synthetic and desktop real-scan
acceptance tests are still required. The output is experimental geometry,
not validated autonomous sticker identification or clinical motion tracking.
No raw DICOM, Hall data, detector JSON or presentation plots are edited.

The standalone live app now includes app/python_cbct/hdmm_sticker_geometry.py
and an explicit **Load reviewed sticker JSON** button in the HDMM section.

## Three operations

1. **inspect:** Given an operator-defined approximate nose/reflector CT-LPS
   region, search its volume for compact *bright AND dark* local CT features.
   Report several possible centers. NONE is automatically called a sticker.
   The optical marker may not be sufficiently CT-contrasting to appear.
2. **approve:** After *independent source CT slice review*, record the
   actual optical reflector's reference CT-LPS center, scan detector hash,
   the three CT magnet locations and a written identification note.
   A claimed visible center that is cut off at the voxel-grid boundary
   is refused. Independent physical marker-to-magnet metrology is
   allowed as an alternate evidence source, but must first establish
   the CT-LPS reference point externally.
3. **transfer:** Predict that confirmed rigid point in a later CT using
   registered stationary side receiver chips and measured 3-magnet
   head rotation/translation. The predicted target point may be outside
   the later CT FOV and is explicitly labeled **not directly detected**.

**Hard limit:** Without a previously confirmed reference point or
physical marker-to-magnet measurement, an entirely cropped marker
cannot be localized from that CT, regardless of candidate matching.

## Before the next GK experiment

Obtain one intact reference CBCT that includes the sticker, all three
magnets and stationary side boards, without repositioning sticker or
magnets between reference and motion scans. If the optical sticker is
CT-invisible, attach an appropriate co-visible fiducial or measure a
rigid relationship to the magnets independently. Record the actual
sticker attachment, detector output provenance and source DICOM.

The script's CT FOV status only describes the *oriented voxel grid*.
A cylindrical CBCT reconstruction may contain padded pixels or have
less usable support than the apparent rectangular grid. The optional
4-mm support radius is an assumption, NOT a measured sticker radius.
Confirm actual coverage on native DICOM slices and scanner metadata.

## Commands on Windows: standalone app repository

Replace every quoted placeholder path and L P S with the actual paths
and CT-LPS millimetre coordinates. Each output must have a fresh name;
existing outputs are never overwritten.

~~~powershell
cd F:\GitHub\magnetic_board_driver

$py = ".\.venv\Scripts\python.exe"
$tool = ".\app\python_cbct\hdmm_sticker_geometry.py"

# Independently approximate nose ROI in an intact reference scan:
& $py $tool inspect --reference-scan "REF_SCAN_FOLDER_OR_ZIP" --search-center-lps-mm L P S --radius-mm 25 --out ".\hdmm_candidates_NEW.json"

# REVIEW native DICOM slices and independently identify the actual sticker.
# Replace L P S with its reviewed center, not first bright candidate.
& $py $tool approve --reference-detections "REF_DETECTIONS_JSON" --reference-scan "REF_SCAN_FOLDER_OR_ZIP" --marker-lps-mm L P S --evidence CT_VISIBLE_OPERATOR_REVIEWED --note "Optical reflector center identified in source CT slices, reviewer and scan documented." --out ".\hdmm_reference_NEW.json"

# Propagate reviewed point to a later CT even if the sticker is cropped.
& $py $tool transfer --reference-json ".\hdmm_reference_NEW.json" --reference-detections "REF_DETECTIONS_JSON" --target-detections "TARGET_DETECTIONS_JSON" --target-scan "TARGET_SCAN_FOLDER_OR_ZIP" --out ".\hdmm_transfer_NEW.json"

# Code tests, using the app's Python environment (pytest additionally needed):
& $py -m pytest -q .\app\python_cbct\test_hdmm_sticker_geometry.py
cargo test -p app hdmm_sticker
cargo check -p app
~~~

For an independent physical measurement converted into CT-LPS geometry,
use --evidence INDEPENDENT_PHYSICAL_RIGID_OFFSET instead of
CT_VISIBLE_OPERATOR_REVIEWED and record the metrology in --note.
The script does NOT calculate CT coordinates from an arbitrary ruler
measurement or estimate the marker location from optical motion curves.

## Registration and QC

Two stationary 4x4 side boards (32 CT chip locations, including possible
grid symmetries) register the scan coordinate frames. Three named
magnets independently fit the head-moving rigid transform:

    p_target = (p_reference @ R_station + t_station) @ R_head + t_head

The transfer rejects fixed-board registration RMS over 0.75 mm,
magnet head-rigid RMS over 0.75 mm, and any change in magnet-to-magnet
distance over 1.0 mm. The numbers are preliminary *failure gates*,
not the uncertainty in the sticker coordinate. Passing a magnet fit
does not prove the sticker stayed fixed relative to those magnets.

The original reference JSON embeds the source detector SHA-256,
magnet positions, and CT frame UID when available. The transfer JSON
stores matching target magnet centers and target UID plus FOV status.
All estimated points are labeled RIGID_TRANSFER_ESTIMATE_NOT_DIRECT_CT_DETECTION.

## Use in the live application

First import the same reference/target CT geometry into the app.
In the HDMM field group, click **Load reviewed sticker JSON**. Use the
reference JSON with reference CT; transfer JSON with target CT.
The app checks all three current CT magnet locations (within 0.75 mm),
the frame UID if present, valid source provenance and transfer QC.
It refuses an unreviewed brightness candidate, mismatched CT, or
unchecked target image-voxel FOV. Accepted files populate the
editable CT-LPS XYZ field and display the physical evidence note.

**The import does not itself enable motion fitting or set the HDMM
target.** Inspect the value and click **Set HDMM point** explicitly,
after separately approved CT mapping and valid experimental
stationary magnetic calibration. The plotted value is still a
Hall-rigid-pose prediction, not an optical HDMM measurement.

## What this cannot solve from the November 6 scans

The old source search found 97/96 raw CT candidates, 72/71
de-duplicated and 33 lateral moving pairs, but no confirmed optical
nose reflector. The reported crop may explain the failure, but is
not proven from the original candidate shortlist. Without another
independently registered image or physical measurement, the original
sticker CT position remains unknown.

Even with a spatially located sticker, independent Hall/HDMM clock
registration, vendor-axis/CT-LPS registration and non-wrapped sensor
measurements remain necessary before publishing same-point tracking
accuracy.
