# Gamma Knife CT mapping review: UI and calibration boundary

Updated 2026-10-30 (branch `feature/live-rigid-body-fit`).

## Decision and current result

The October 30/31 seven-scan detector output provides three magnets and 48 CT-detected sensor chips. The superior PCB centroid/JST connector is not independently CT-measured. Consequently, **strict CT-only v2 registration remains unavailable** for the superior board; a separately reviewed KiCad PCB/JST mounting prior is used for a **provisional** three-board CT-to-firmware mapping.

The user explicitly approved the CAD mapping for **experimental calibration only**. Approval does not measure the superior JST in CT, validate the raw Hall-to-LPS transform against independent experiments, or establish live 6-DOF motion accuracy. Current UI status is "Mapping approved for experimental calibration. Tracking remains off until Hall calibration succeeds."

Known example mapping checks from the user's October 30 report:

| Board | Raw ID | Grid RMS (mm) | Source of JST/PCB orientation |
|---|---:|---:|---|
| Superior | 0 | 0.465 | KiCad estimate, NOT CT-measured |
| Left | 1 | 0.134 | CT-detected PCB/JST proxy |
| Right | 2 | 0.203 | CT-detected PCB/JST proxy |

Verify raw board cabling (0 superior, 1 left, 2 right), chip faces, superior JST toward patient right, side JSTs inferior, and the KiCad PCB revision with the actual setup.

## UI/report decisions

- The *CT sensor mapping* panel shows a compact three-board summary. **Show details** opens a larger scrollable review with one section per board. Index mappings are broken into four-sensor lines, and candidate Hall-to-LPS matrices into labeled L/P/S rows.
- The detailed textual report rounds matrix values for visual review. Exported calibration JSON retains full numeric precision in `registration_v2_candidates`.
- **Copy calibration report** is a concise status/summary, distinguishing the strict CT-only warning from the separately reviewed CAD-assisted candidate. **Copy mapping** copies the complete human-readable board mapping for scientific review.
- The approval button remains a separate explicit action after preview. Approving neither captures Hall fields nor enables tracking.
- User-facing button labels favor short natural language (Preview mapping, Show details, Copy mapping, Approve for calibration) instead of hyphen-heavy technical nomenclature. Hyphens in coordinate terminology and code identifiers remain meaningful.

## Calibration sequence and limitations

1. Import the matching CT detections and preview the CAD-assisted mapping. Confirm physical board identities and connector directions, then explicitly approve **for experimental calibration only**.
2. Without magnets and with fixed boards, capture a no-magnet Hall background. Then install the stationary magnets and capture all 48 Hall channels.
3. Run the experimental stationary-field calibration and save full data and raw logs. A good field RMSE alone does not establish live-motion accuracy.
4. Validate direction, repeatability, timing, and 6-DOF displacement against independent motion/HDMM reference measurements before treating tracking as validated.

Never silently substitute CAD-predicted PCB/JST positions for CT measurements or use unverified mapping to enable live fitting.

## Test / validation status

- Prior compilation regression fixed (`Option::map_or` callback).
- Detector status consistency fixed: truly incomplete missing magnet or array is rejected; missing *optional* glass BB can be accepted with three complete boards.
- User verified **42/42 app tests passing** immediately before this UI/report change.
- A formatting regression assertion was added for the CAD mapping preview. The updated source has been checked in GitHub, but Rust tests and GUI rendering for this change must still be rerun on Windows:
  `cargo test -p app` and `cargo run -p app --release`.

The main remaining scientific dependency is new GK stationary calibration data followed by independent motion validation, not additional reinterpretation of the existing CAD estimates.
