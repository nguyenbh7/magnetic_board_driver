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
