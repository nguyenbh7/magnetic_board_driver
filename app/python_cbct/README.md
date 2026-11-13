## Anatomy-first HDMM sticker proposals (2026-11-12)

The experimental `hdmm_anatomy_first.py`, `hdmm_nose_slice_profile.py`,
and supporting local CT feature code are now bundled in this repository.
No separate research checkout or notebook is needed **at runtime**.
They are **not yet wired to an automatic in-app Find sticker button**.
Only the operator-reviewed reference JSON is accepted by the live app.

Run from repository root with the SAME imported CT and its matching
`detections.json`, with your DICOM Python venv active:

```powershell
python .\app\python_cbct\hdmm_anatomy_first.py propose `
  --scan "C:\PATH\TO\CBCT.zip" `
  --detections "C:\PATH\TO\detections.json" `
  --landmark-method slice-profile-v2 `
  --out "$HOME\Downloads\hdmm_anatomy_review_new.json"
```

The script saves a same-name `.png` CT preview. It now chooses a separate
**inferior nasal review focus** where plausible, because the anatomically
estimated bridge can be about 19 mm superior to the tip; this is only a
visualization preference, **not automatic optical sticker identification**,
not a change to any original candidate or threshold, and not proof of
complete CT sticker coverage. Review all plausible candidates in the original
CT and assess the reconstructed FOV. To create an importable *reviewed*
reference, see
[the standalone review and approval workflow](../../docs/GK_NEXT_HDMM_STICKER_LIVE_COMPARISON.md).

---


## HDMM sticker reference and cropped-scan geometry (2026-11-10)

The bundled **hdmm_sticker_geometry.py** provides three independent,
read-only commands: **inspect** an approximate nose/reflector CT region
for local positive/negative contrast candidates, **approve** an
independently reviewed reference optical sticker point from an intact
scan, and **transfer** that CT-LPS point to a later cropped scan using
stationary receiver registration and three-magnet head motion.

This is NOT an autonomous optical sticker classifier. An optical
reflector can be invisible in CBCT or cut off by the reconstruction FOV.
No valid reference or physical measurement means **no recovered point**.
The live app's **Load reviewed sticker JSON** validates the geometry
against the currently loaded CT and only populates a coordinate field
for separate manual Set HDMM point approval; no tracking is activated.

Full commands and scan/measurement prerequisites:
[GK HDMM sticker reference protocol](../../docs/GK_HDMM_STICKER_REFERENCE_WORKFLOW_20261110.md).
The lab-package script already copies all Python files in this folder,
so this module is included in portable app ZIPs without another Git
checkout. Synthetic tests are in test_hdmm_sticker_geometry.py
(install pytest separately for developer validation).

---

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
is available only when the separately reviewed vendored
`hybrid_board_detector.py` module is present; Original and Optimized do not
require it.

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

**Hybrid is now bundled** in `hybrid_board_detector.py`, extracted from
the previously reviewed notebook by `vendor_hybrid_board_detector.py`.
The module is ordinary Python source committed on the live app branch;
no notebook is needed on the lab computer. Its source digest remains
recorded for detector provenance. Original and Optimized remain available.

The CT geometry remains **unreviewed**, is never authorized for clinical
tracking, and cannot automatically approve the CT-to-firmware mapping or
activate a magnetic fit. Real GK DICOM performance and application builds
must be checked on Windows after this source consolidation.
