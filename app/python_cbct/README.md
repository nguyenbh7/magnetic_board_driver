## Lab laptop installation and package diagnostics (2026-11-06)

**Yes, this folder already has `requirements.txt`.** It is the authoritative
Python package list for the current DICOM importer:

`numpy`, `scipy`, `matplotlib`, `pandas`, `SimpleITK`, `torch`,
`torchio`, `IPython`, and `contourpy`.

These packages are only needed for **DICOM processing**. Raw Hall sensor
streaming, logging, and Rust rigid 6-DOF fitting do not require Python.

For a **new Windows lab laptop** with Python **3.11** installed:

```powershell
# From the magnetic_board_driver repository or the extracted lab-app bundle:
powershell -ExecutionPolicy Bypass -File .\scripts\setup_dicom_windows.ps1

# Future checks without reinstalling:
powershell -ExecutionPolicy Bypass -File .\scripts\setup_dicom_windows.ps1 -CheckOnly
```

This installs into `./.venv`, not system Python, and validates real imports and
the detector CLI. A missing dependency is reported by name. You can also use
the in-app **DICOM Geometry → Check DICOM packages** button, **Copy setup
commands**, and **Copy package report**. These GUI buttons do **not** run pip,
install packages or change the active detector.

For manual troubleshooting:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\app\python_cbct\requirements.txt
.\.venv\Scripts\python.exe .\app\python_cbct\check_dicom_environment.py
```

To prepare an entire lab-app ZIP on the Windows development PC (close running
`app.exe` windows first):

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\package_lab_laptop.ps1
```

The output is `dist/GK_Live_Fitting_Lab_Laptop.zip`. It contains the compiled
app (if the release build succeeds), bundled Python source, requirements file,
setup script and a short laptop README. **It does not bundle the large Python
packages**; install them on the laptop once, with internet access. It does
not bundle private DICOM or Hall data. Keep the whole extracted directory:
an `app.exe` file by itself is insufficient for DICOM imports.

If you have an old external `CBCT_DETECT_SCRIPT` or `CBCT_PYTHON` environment
variable, remove it so the app uses its local detector and `.venv`. Hybrid
requires a separately reviewed board algorithm and remains unavailable even
when all dependencies are installed; Original and Optimized work without it.

The Rust build itself uses `Cargo.toml`/`Cargo.lock` for dependencies.
The setup script does not install Rust or cargo; neither is required for a
precompiled `app.exe` on the laptop.

# Bundled CBCT detector (transitional Python backend)

The app repository contains the DICOM detector's maintained source directly in
`app/python_cbct/` so **no sibling `hall-effect-motion-tracking` checkout is
required** for Original/Optimized DICOM imports. This directory is a pinned
source snapshot from `nguyenbh7/hall-effect-motion-tracking` branch
`a1-vgm-validation` (2026-11-06). Source files copied unchanged:

- `cbct_detect.py`, `detector_core.py`, `cbct_superior_roi.py`
- `chip_grid_recovery.py`, `superior_board_rescue.py`
- `superior_chip_first_index.py`, `pcb_cad_geometry.py`
- `requirements.txt`

**This is NOT native Rust DICOM processing.** The GUI still launches a Python
interpreter and requires open-source image libraries, but it no longer loads
our detection code from another Git repository. The longer-term true-native
Rust port is tracked in `docs/CBCT_NATIVE_DETECTOR_PLAN.md`.

On a machine with Python 3.11, create a dedicated environment from the
`magnetic_board_driver` repository root:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\app\python_cbct\requirements.txt
```

The app automatically probes this `.venv` when starting a DICOM import.
Alternatively set `CBCT_PYTHON` to its interpreter path.

If `CBCT_DETECT_SCRIPT` points to an older external script, **unset it** so the
app uses its own `app/python_cbct/cbct_detect.py`; a preflight check prevents
quietly using an incompatible version.

**Hybrid is still blocked until its reviewed alternative board functions are
available in this repository.** The exact five functions previously existed
only inside an external source notebook. We intentionally do not substitute
a different or invented board detector and call it the validated Hybrid.
The code fails explicitly rather than silently falling back. Original and
Optimized do not require Hybrid.

The CT geometry remains **unreviewed**, is never authorized for clinical
tracking, and cannot automatically approve the CT-to-firmware mapping or
activate a magnetic fit. Real GK DICOM performance and application builds
must be checked on Windows after this source consolidation.
