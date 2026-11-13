"""Check the actual Python environment used by this app's CBCT importer.

Standard library only. Does not read scans, process DICOM, or modify packages.
Run with: python app/python_cbct/check_dicom_environment.py
"""
from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import os
from pathlib import Path
import subprocess
import sys

DEPENDENCIES = {
    "numpy": "numpy",
    "scipy": "scipy",
    "matplotlib": "matplotlib",
    "pandas": "pandas",
    "SimpleITK": "SimpleITK",
    "torch": "torch",
    "torchio": "torchio",
    "IPython": "ipython",
    "contourpy": "contourpy",
}


def diagnose(script: Path) -> tuple[bool, str]:
    lines = [
        "GK DICOM importer dependency check",
        f"Python executable: {sys.executable}",
        f"Python version: {sys.version.split()[0]}",
        f"Detector script: {script.resolve()}",
        f"CBCT_DETECT_SCRIPT override: {os.environ.get('CBCT_DETECT_SCRIPT') or '(not set)'}",
        f"CBCT_PYTHON override: {os.environ.get('CBCT_PYTHON') or '(not set)'}",
    ]
    success = True
    if sys.version_info < (3, 11):
        lines.append("FAIL: Python 3.11 or newer is needed for this lab setup.")
        success = False
    for module, package in DEPENDENCIES.items():
        try:
            importlib.import_module(module)
            version = importlib.metadata.version(package)
            lines.append(f"PASS: {package} {version}")
        except Exception as exc:
            success = False
            lines.append(f"FAIL: {package}: {type(exc).__name__}: {exc}")
    if not script.is_file():
        success = False
        lines.append(f"FAIL: missing bundled detector: {script}")
    else:
        try:
            completed = subprocess.run(
                [sys.executable, str(script), "--help"],
                capture_output=True, text=True, timeout=60, check=False,
            )
            if completed.returncode != 0 or "--detector-variant" not in completed.stdout:
                success = False
                lines.append("FAIL: detector CLI is outdated or --help failed; cannot select Original/Optimized.")
                lines.append(completed.stderr[-500:])
            else:
                lines.append("PASS: detector CLI supports Original/Hybrid/Optimized variants.")
        except (OSError, subprocess.TimeoutExpired) as exc:
            success = False
            lines.append(f"FAIL: cannot check detector CLI: {exc}")
    hybrid_present = script.with_name("hybrid_board_detector.py").is_file()
    if hybrid_present:
        lines.append("PASS: Reviewed Hybrid source is bundled alongside the CT detector.")
    else:
        lines.append("NOTE: Hybrid source is NOT bundled; Original/Optimized remain available.")
    lines.append(
        ("RESULT: DICOM Python environment ready for Original/Optimized/Hybrid."
         if hybrid_present else
         "RESULT: DICOM Python environment ready for Original/Optimized.")
        if success else "RESULT: DICOM Python environment is NOT ready. Install requirements.txt."
    )
    return success, "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--detector-script", type=Path,
                        default=Path(__file__).with_name("cbct_detect.py"))
    args = parser.parse_args(argv)
    success, message = diagnose(args.detector_script)
    print(message, flush=True)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
