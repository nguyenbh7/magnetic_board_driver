# Next Gamma Knife measurement — live Hall prediction at the optical HDMM sticker

## Goal

Display the Hall **predicted** scalar displacement of the physical HDMM
reflective sticker next to raw Hall sensor traces and its prediction graph.
The vendor optical HDMM remains a **separate visual display**;
there is NO optical input or time synchronization in the desktop app.
Both displays need the same stationary physical zero for a meaningful
immediate qualitative comparison. This remains phantom/research use only.

The numeric readout next to the Hall prediction graph is:

- **LIVE**: current accepted Hall 6-DOF fit while receiving fresh frames;
- **HOLD**: last accepted value after stream stop or subsequent fit rejection,
  **not** an updated position;
- **INVALID**: latched sensor input/range QC fault; hide numeric prediction;
- **REPLAY**: predicted from previously recorded raw Hall data, not live.

Numeric displacement is in mm and includes rigid-body translation and
rotation about the CT-defined origin. It is NOT the vendor HDMM signal,
not a measured optical marker center, and not a clinical motion-tracking
validation.

## Current in-app CT sticker workflow (2026-11-12 UI cleanup)

The GUI now has one **HDMM optical sticker — CT detection and review**
panel directly after **DICOM Geometry — CT preview**. Its controls have
different responsibilities; no second Load/Set buttons appear next to the
live displacement graph:

1. Import the **original** CBCT ZIP or DICOM folder in the DICOM panel;
   the ordinary detector finds magnets/48 chips/optional glass BB and
   explicitly reports **HDMM NOT SEARCHED**.
2. Click **Find HDMM candidates from CT**. The app runs its own
   `app/python_cbct/hdmm_anatomy_first.py` with slice-profile-v2 and a
   25-mm experimental search radius in a worker; JSON and PNG are saved
   alongside the imported CT detection report. No separate research repo
   or Jupyter notebook is required.
3. Click **Open CT review image** and check the **original reconstructed
   CT slices**, including the FOV near the nasal tip. Candidate IDs are
   unverified; the preview focus is not an automatic marker detection.
4. Select the verified candidate from the dropdown, write at least 20
   characters documenting the physical CT evidence, and explicitly click
   **Approve operator-reviewed CT candidate**. The Python review bridge
   checks original scan hash, affine, detector report hash, FrameOfReferenceUID,
   FOV support, candidate ID, and operator note. Cropped/ROI-limited components
   are rejected. If the anatomy detector abstains or the sticker isn't visible,
   do not approve a guessed position.
5. A successfully reviewed JSON is **staged** into the CT LPS input, not
   activated. Inspect provenance and coordinates, then click **Set HDMM
   point**. The graph shows a Hall-predicted *point* movement, not optical
   HDMM readings or a calibrated optical centroid.
6. **Load existing reviewed sticker JSON** and raw manual CT LPS coordinates
   remain separate, clearly labeled fallback inputs inside that same panel.
   Changing a CT source or detector invalidates any previous sticker review.
7. Finally, after a valid rigid Hall fit, **Set zero** in magnetic calibration
   at the same stationary physical pose as the vendor optical HDMM zero.

The **Live Tracking** monitor keeps the large numeric readout beside raw Hall
traces but contains no duplicate marker selection buttons. **Sensor Traces**
uses the same axis-specific all-sensor limits as Live Tracking; the conflicting
legacy manual scalar warning input has been removed.

The legacy command-line walkthrough below is optional for offline research
or reproducibility. The GUI now runs both proposal and approval workflows.
The GUI and Rust build must still pass Windows testing after this integration.

---

## 1. Build the app and analyze its tests

```powershell
cd F:\GitHub\magnetic_board_driver
git switch feature/live-rigid-body-fit
git pull --ff-only origin feature/live-rigid-body-fit
python -m pytest -q .\app\python_cbct\test_approve_anatomy_hdmm_sticker.py .\app\python_cbct\test_hdmm_sticker_geometry.py .\app\python_cbct\test_hdmm_anatomy_first.py .\app\python_cbct\test_hdmm_nose_slice_profile.py
if ($LASTEXITCODE -ne 0) { throw 'HDMM CT review/approval tests failed' }
cargo test -p app
if ($LASTEXITCODE -ne 0) { throw 'Live app tests failed' }
cargo run -p app --release
```

## 2. Obtain the initial current-session CT and matching detections

On the phantom, place the optical HDMM sticker at the nose and confirm
the **entire sticker lies in the reconstructed FOV**. Use the app's
**DICOM Geometry → Import ZIP / Import folder** to process this CT.
Review all three magnet centers and 3x16 sensor-chip positions before
approving calibration. The app's **CT details / detector report folder**
provides the path to the matching `detections.json` and review images.
Only this SAME scan/detection report may be used for the sticker reference.

Keep the exact original CT folder or ZIP available and copy these two
paths into PowerShell. The following are **placeholders** to replace
with paths actually produced by the next measurement:

```powershell
$scan = 'C:\REPLACE_WITH_NEXT_GK_DICOM_FOLDER_OR_ZIP'
$det = 'C:\REPLACE_WITH_CT_OUTPUT\detections.json'
if (-not (Test-Path $scan) -or -not (Test-Path $det)) {
    throw 'Set actual CT and matching detector paths first'
}
```

## 3. Run v2 detection and independently review the sticker

The anatomy-first v2 detector is now bundled directly in
`magnetic_board_driver/app/python_cbct/`. No research checkout or
collaborator notebook is needed. This command currently runs from
PowerShell rather than an in-app automatic "Detect sticker" button.
It produces **unreviewed** candidates and a PNG; it never activates tracking.

```powershell
cd F:\GitHub\magnetic_board_driver
$outDir = '.\analysis_outputs\hdmm_sticker_validation'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$outDir = (Resolve-Path $outDir).Path
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$proposal = "$outDir\next_gk_anatomy_first_$stamp.json"
python .\app\python_cbct\hdmm_anatomy_first.py propose `
    --scan $scan --detections $det `
    --landmark-method slice-profile-v2 --radius-mm 25 --out $proposal
if ($LASTEXITCODE -ne 0) { throw 'Anatomy-first CT candidate proposal failed' }
Get-Content $proposal -Raw | ConvertFrom-Json |
    Select-Object -ExpandProperty candidates |
    Select-Object candidate_id,threshold_raw_ct,voxel_count,centroid_lps_mm,touches_scan_boundary,touches_search_roi_boundary | Format-Table -AutoSize
Write-Host "Inspect the original CT and $([IO.Path]::ChangeExtension($proposal,'.png'))"
```

**Do not treat N01 or any first-listed candidate as automatically verified.**\nCurrent v2 sorts unverified components by distance to an anatomical seed and\nROI-boundary flags. The actual distal-nose feature may appear later in the\nlist; inspect every plausible nasal-tip candidate and reconstructed FOV.\nThe v2 CT-localization method has not been verified to identify an optical\nsticker automatically. The 25-mm search radius is the existing method's
allowed maximum and is used for **new experimental review** because a
November 12 distal feature was flagged at the former 23-mm spherical
search boundary while appearing intact within the expanded ROI. This
does NOT relax CT field-of-view checking or establish sticker identity.
Review the original CT and overlay image to identify the actual physical
HDMM reflective attachment. The thresholded CT component centroid
will generally not equal the calibrated optical tracking center.
A fully cropped/absent/ambiguous sticker must not be approved.

## 4. Export a strictly matching, operator-reviewed reference

The bridge requires a reviewed candidate ID, original scan/detections
SHA matches, DICOM LPS agreement, and an intact 4-mm assumed support.
It writes the existing format accepted by the live app's JSON importer.

```powershell
cd F:\GitHub\magnetic_board_driver
$candidate = 'N01'  # REPLACE only after physically reviewing the CT overlay
$note = 'Original GK CT slices confirm the reflective HDMM sticker at the mannequin nose tip; threshold component center is approximate, not optical center.'
$reviewed = Join-Path $HOME "Downloads\next_gk_reviewed_hdmm_$stamp.json"
python .\app\python_cbct\approve_anatomy_hdmm_sticker.py `
    --proposal $proposal `
    --candidate-id $candidate `
    --reference-detections $det `
    --reference-scan $scan `
    --operator-confirmed --note $note `
    --out $reviewed
if ($LASTEXITCODE -ne 0) { throw 'CT sticker reference approval failed' }
Write-Host "Reviewed sticker JSON: $reviewed"
```

The app will **reject** a raw candidate file, a mismatched CT magnet
constellation, or a reference from a different CT FrameOfReferenceUID.
The new bridge refuses cropped candidates. If the sticker is cropped
in the current scan, reacquire an intact CT or use the separately
reviewed same-session transfer process; do not use an edge fragment
as a direct measurement.

## 5. Activate and compare during the experiment

1. In the app, import/review the current-session CT geometry and
   complete the experimental Hall calibration and board mapping.
2. Next to the **Hall-predicted HDMM sticker displacement** plot,
   click **Load reviewed sticker**, choose `$reviewed`, and inspect
   the loaded LPS coordinates and source status.
3. Click **Set HDMM point** only after confirming it is the correct
   physical sticker and current CBCT. A new CT or calibration
   invalidates the old point and requires a new review/import.
4. With the phantom stationary, wait for an accepted Hall fit,
   click **Set zero** in the app, and zero the vendor optical HDMM
   on the **same physical stationary pose**.
5. Compare the app's large Hall **predicted mm** number against the
   separate optical HDMM display while moving the head. Both need
   the same movement definition (e.g., 3D displacement magnitude),
   mm units, and zero location. Clock alignment is NOT available.
6. Watch raw Hall traces and range/step warnings. If the readout
   says **HOLD**, **INVALID**, or the fit is rejected, do not compare
   that old value to the optical HDMM as a contemporaneous measurement.
7. Save raw Hall logs, the CT detection/approval JSON, and
   independently collected optical HDMM logs for later analysis.

Neither the app nor this workflow displays or records the actual
optical HDMM readout. No sensor calibration, rigid fit, patient
tracking, clinical release, or absolute sticker accuracy is implied.
