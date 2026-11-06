# Native in-app GK CBCT detection — implementation contract

Status: **NOT IMPLEMENTED** as of 2026-11-06. This document records the user's requirement so a Python module or packaged Python worker is not presented as a "native app" completion.

## Intended end state

The existing Rust desktop application must directly import the selected DICOM ZIP or folder, run the selected detector in Rust in a cancelable background job, and return the existing `CbctGeometry` review-only structure. **No installed Python, standalone `cbct_detect.py`, local research repository, Jupyter notebook, vendored notebook-derived Python module, or Python subprocess is permitted at runtime.** Detector choices remain **Original**, **Hybrid**, **Optimized**. A method failure must be explicit; do not silently use geometry from another method.

## Current architecture and gap

- UI: `app/src/main.rs` DICOM selection, start/cancel/progress, safety resets and preview already exist.
- Geometry validation: `app/src/cbct_geometry.rs` already enforces three named magnets, three 16-chip arrays, finite CT geometry and review-only registration.
- **All current detectors still invoke Python via `std::process::Command`.** The code discovers `CBCT_DETECT_SCRIPT`, resolves a Python interpreter and invokes `cbct_detect.py`. Removing the notebook picker or vendoring Python functions is not a native implementation.
- The optimized maintained Python detector is in the separate `hall-effect-motion-tracking` repository at `tools/cbct_bridge/detector_core.py` (~1,725 lines). It includes volume I/O, three-magnet detection, board identification, slab reslicing, 4×4 chip grid and BB detection, and seven speed switches. This is a **porting reference**, not a runtime dependency.
- The previously tested Hybrid board detector's exact five-function source (`_reslice`, `_side_level`, `_contrast_cut`, `_footprint`, `_measure_board`) is **only in the reviewed source notebook `find_magnets_arrays(1).ipynb` on the experiment workstation**, not in the repositories. Until those original bodies are obtained, do not claim exact Hybrid behavior can be recreated or validated.
- The previous one-time Python vendoring script and missing `hybrid_board_detector.py` are an **interim, non-native implementation**, not the requested deliverable.

## Implementation units

1. **Rust DICOM input**: read zipped or directory multi-slice CT using a maintained Rust DICOM library with explicit compressed transfer-syntax support/clear errors; validate Series/Frame UID, LPS orientation, spacing and consistent pixel arrays. Reject mixed/ambiguous CT series and ZIP extraction traversal. Keep raw scans private; don't log DICOM identifiers.
2. **Rust image operations**: connected components/threshold masks, point-cloud geometry, morphology, EDT or geometrically equivalent guarded fallback, principal-axis/plane fits, trilinear/affine oblique reslice, Otsu/outline footprint and 4×4 chip-grid search. Preserve physical-mm conventions and detector/QC acceptance thresholds; avoid CPU or memory blocking on UI thread.
3. **Rust detectors**: named magnets and glass BB, plus three interchangeable board methods. Original must match the unaccelerated reviewed baseline; Optimized must preserve the same acceptance geometry with seven speedups; Hybrid must use the *actual reviewed alternate board algorithms* rather than an invented approximation. Avoid automatic algorithm fallback.
4. **Rust geometry adapter**: feed existing `CbctGeometry` and mapping-review gates directly; record mode/source version and QC, including explicit unreviewed superior PCB/JST provenance.
5. **UI migration**: replace `run_cbct_import_off_thread` Python launcher with Rust worker; remove `CBCT_DETECT_SCRIPT` and `discover_cbct_python` only after all three native modes work. Keep import cancellation, logs, geometry validation, calibration disablement and source labeling intact.
6. **Regression**: synthetic unit tests for geometry, DICOM coordinate transforms and all three modes; matched real October 30 seven-scan regression for 3 magnets/3 arrays/48 chips/optional BB, raw LPS point differences, chip grids, source tags, CT-to-firmware review and speed; verify switching detector invalidates prior calibration. Execute `cargo test -p app --release` and interactive Windows app checks.

## Explicit acceptance criteria

- Running the released app on a computer with **no Python installation** can import the same DICOM ZIP in each of the three modes, and the app never launches a Python interpreter or requires a notebook.
- The Hybrid method matches previously reviewed Hybrid real-data results within scientifically justified tolerances. If exact reviewed source is unavailable, Hybrid must be clearly unavailable, not silently approximated.
- Original/Optimized geometry and speed are compared to the frozen seven-scan results, not assumed equivalent from labels.
- No imported CT geometry automatically enables or approves live magnetic tracking.

**Do not mark this action item complete merely because the existing Python detector is callable from Rust or because a source `.py` file is packaged with the executable.**
