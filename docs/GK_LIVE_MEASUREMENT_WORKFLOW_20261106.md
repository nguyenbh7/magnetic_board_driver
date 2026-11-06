# Gamma Knife live app — background, HDMM proxy, repeat acquisition, raw sensor QC, DICOM packaging

Status: **implemented in source on `feature/live-rigid-body-fit`, not yet compiled or experimentally validated on the user's Windows machine**. The app is for **experimental**, not clinical, tracking.

## Acquisition sequence

1. **No magnets / fixed receiver mounting**: select serial port, Start Field Stream, click **Capture background** and collect the full 3-second/48-channel capture. As soon as complete, click **Save background only**. A new standalone `gk_raw_hall_background_v1` JSON contains raw firmware board IDs 0–2, 16 sensors per board, XYZ **mT**, and an acquisition time. No CT, marker, or stationary reference is required. The Save dialog refuses overwriting an existing file.
2. **If reopening the app or switching ports**: click **Load background** and select that JSON. The file is validated but **NOT automatically applied**. After verifying same three receivers, exact mounting orientation and gain/resolution, click **Confirm same setup and use background**. Importing clears stale stationary calibration and active experimental fit, requiring a new stationary reference. Never combine a saved background from different sensor/board mounting or changed magnetic environment.
3. Acquire CT with the sensors fixed and the magnets in the intended initial position. Use **Import ZIP/folder** or reviewed **Load JSON** as appropriate, independently inspect CT/JST geometry, and explicitly review/approve only for experimental calibration. Import does not enable tracking.
4. With the magnets stationary after CT, click **Capture stationary fields** and **Calibrate tracking**. Review fit RMSE/strength bounds before enabling experimental 6-DOF. The retained background is used if present; otherwise the existing fitted board-constant offsets carry lower confidence.
5. **Start/stop/restart during motion**: stopping freezes the last history. The next **Start Field Stream** clears that history and relative zero, keeps the calibrated CT model and last accepted *absolute pose only as an optimizer starting seed*, and takes the first new accepted fit as a new relative zero. A new "Set zero" can still reset the reference within a recording. This does **not** create a new CT-to-firmware mapping: it is valid only when the boards and mounting remain fixed. If the receiver, support structure, magnet identity, wiring, or overall setup moved, obtain/review new registration and calibration. The GK experimental pose bounds remain ±20 mm and ±30° relative to that CT baseline; movements beyond the calibration search region are not recoverable by merely resetting zero.
6. **Predicted HDMM displacement**: if the reflective sticker location has been **independently measured** in the same CT LPS frame, enter the sticker's L,P,S coordinates in millimetres and click **Set HDMM point**. The rigid fitter projects `p(t)=origin + translation(t) + R(t)(sticker-origin)` for the current and zero poses, including rotational motion, and plots the scalar 3-D displacement from the new zero. The app does *not* import an actual HDMM stream. Only compare against the displayed HDMM measurement when the reference zero and timing are aligned. A detected glass BB is **not automatically** the HDMM reflective marker; no plot is generated without a manually confirmed point.
7. **Monitor raw Hall alongside the rigid fit**: Live Tracking shows fitted XYZ/rotation, projected HDMM marker motion, and each receiver's selected raw Bx/By/Bz field traces (mT). The **Both** tab shows the separate sensor trace panel side-by-side instead of duplicating those plots. The peak absolute *individual component* is calculated across **all sensors with trace history** on each receiver, including unselected sensors; missing/nonfinite samples are counted separately. Enter the verified **per-axis full-scale in mT** for the actual MLX gain/resolution configuration. Values ≥90% trigger **possible clipping** warnings. Changing gain/resolution or board connections clears that warning threshold. No firmware saturation flag is available, so **the absence of a warning does not establish no saturation**. Retain raw logs for offline confirmation.

## Repository-independent source (interim)

The `magnetic_board_driver` repo now contains the maintained `app/python_cbct/` detector scripts and helper modules plus `requirements.txt`. `app/src/cbct_geometry.rs` resolves the default script **inside this repository**, not in a neighboring `hall-effect-motion-tracking` checkout. You can install the third-party Python image libraries in this repo's own `.venv`:

```powershell
cd C:\Users\nguye\Documents\Github\magnetic_board_driver
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\app\python_cbct\requirements.txt
Remove-Item Env:\CBCT_DETECT_SCRIPT -ErrorAction SilentlyContinue
```

This achieves **source checkout independence for Original/Optimized**, not a fully native executable. It still launches Python, and **Hybrid remains unavailable until the reviewed alternate PCB algorithms are ported/committed**. The exact prior source notebook was not present in the repositories. We intentionally fail closed rather than substitute non-equivalent detector code. Follow `docs/CBCT_NATIVE_DETECTOR_PLAN.md` for the true Rust DICOM port and real-CT validation. Do not declare that task complete.

## Local tests and checks required before measurement

```powershell
cd C:\Users\nguye\Documents\Github\magnetic_board_driver
git switch feature/live-rigid-body-fit
git pull --ff-only origin feature/live-rigid-body-fit
cargo test -p app --release
cargo run -p app --release
```

Then perform a short, stationary and moving **offline-reproducible** 3-board test and verify:

- Background export, reopening, import pending confirmation, and use with a later CT.
- Saving twice to the same filename does not overwrite the earlier background.
- Stop/start: new zero, previous pose as seed, no old history mixed with new fit or trace times; check independent physical displacement reference and what happens at pose bounds.
- Actual CT-measured HDMM sticker point, full rigid projected displacement, aligned reference zero versus external HDMM display; no assumed BB identity.
- Sensor traces and full-board peaks at relevant field ranges; change MLX gain/resolution and verify configured full-scale warning is cleared.
- Original/Optimized DICOM imports using only app-local Python script, and failure of Hybrid is explicit. All CT geometry remains review-only; never automatically enable tracking.

**Validation status:** GitHub source commits and Rust unit tests have been written, but Windows `cargo test`, real CBCT detection, sensor saturation metrology, and HDMM comparison have **not been executed by the assistant**.

## Lab laptop dependencies and built-in diagnostics (2026-11-06)

The DICOM Python dependency manifest was **already** bundled under
`app/python_cbct/requirements.txt`: NumPy, SciPy, Matplotlib, pandas,
SimpleITK, PyTorch, TorchIO, IPython, and contourpy. No Python is needed for
Hall logging or the native Rust six-DOF solver.

A new one-time `scripts/setup_dicom_windows.ps1` creates an isolated `.venv`,
installs exactly that requirements file, and executes the standard-library-only
`app/python_cbct/check_dicom_environment.py`. It reports each dependency,
the selected interpreter, DICOM CLI compatibility and stale environment
overrides. The app's **DICOM Geometry** panel also has a nonblocking
**Check DICOM packages**, **Copy setup commands**, and **Copy package report**.
No GUI action automatically installs software or alters an environment.

A Windows developer can prepare a portable source+binaries ZIP via
`scripts/package_lab_laptop.ps1` (includes `app.exe`, `app/python_cbct/`,
the requirements, one-time installer, and lab usage instructions; **does not**
bundle the downloaded Python wheels or experimental/patient data). The app can
find Python detector code beside the portable executable; no other Git
checkout is needed. Python still must be installed on the laptop once.

```powershell
# Lab laptop, inside extracted app folder or this repo:
powershell -ExecutionPolicy Bypass -File .\scripts\setup_dicom_windows.ps1
# No-install recheck:
powershell -ExecutionPolicy Bypass -File .\scripts\setup_dicom_windows.ps1 -CheckOnly
# Dev PC: package app + Python sources for the laptop:
powershell -ExecutionPolicy Bypass -File .\scripts\package_lab_laptop.ps1
```

The detector remains *Python-backed* despite repo-local source. Hybrid's
actual reviewed variant has not been ported into the app and must not be
reported as available merely because its Python dependencies pass.

