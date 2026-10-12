#!/usr/bin/env python3
"""Export the validated May-15 Gamma Knife CT rigid-body calibration for live use.

This intentionally IMPORTS the existing analysis's CT geometry loader, including
its corrected sensor-grid transforms. It reads the saved baseline_calibration.csv
from the same analysis run rather than re-fitting magnetic nuisance parameters.

Example (PowerShell, from magnetic_board_driver):
  python .\tools\export_gk_ct_live_profile.py \
      --analysis-repo F:\GitHub\hall-effect-motion-tracking \
      --ct-xlsx "PATH_TO_051526_CT_WORKBOOK.xlsx" \
      --fit-results "PATH_TO_results\gk_05152026_ct_rigid\gk_data_05152026" \
      --output .\app\config\gk_ct_live_profile.json

Only valid when the live board geometry, magnet axes, polarity, electronics,
and background environment match the calibrated CT setup. Recalibrate after
moving or remounting the hardware.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

PHYSICAL = (0, 1, 2)
TRANSFORMS = ("identity", "identity", "flip_rot180")
PHYSICAL_TO_RAW = (1, 0, 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis-repo", required=True, type=Path)
    parser.add_argument("--ct-xlsx", required=True, type=Path)
    parser.add_argument("--fit-results", required=True, type=Path,
                        help="One output dataset folder with manifest.json and baseline_calibration.csv")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-rmse-mt", type=float, default=0.5)
    args = parser.parse_args()

    for required in (
        args.analysis_repo / "scripts" / "fit_gk_05152026_ct_rigid.py",
        args.ct_xlsx, args.fit_results / "manifest.json",
        args.fit_results / "baseline_calibration.csv",
    ):
        if not required.is_file():
            parser.error(f"Required input missing: {required}")

    sys.path.insert(0, str(args.analysis_repo / "scripts"))
    sys.path.insert(0, str(args.analysis_repo / "src"))
    import fit_gk_05152026_ct_rigid as gk  # exact maintained analysis path

    manifest = json.loads((args.fit_results / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("analysis") != "gk_05152026_ct_rigid_6dof":
        parser.error("Calibration manifest is not the validated May-15 CT rigid fit")
    if manifest.get("grid_transforms") != ",".join(TRANSFORMS):
        parser.error("Grid transforms do not match validated identity,identity,flip_rot180 mapping")
    if manifest.get("raw_board_map_physical_left_top_right") != list(PHYSICAL_TO_RAW):
        parser.error("Raw board assignment does not match Left<-raw1, Top<-raw0, Right<-raw2")
    if manifest.get("axis_mode") != "baseline-dipole":
        parser.error("The profile requires fixed baseline-dipole magnet axes")
    if manifest["field_model"]["field_component_convention"] != (
        "historical May-15 raw [Bx, By, Bz] rotated directly to CT"
    ):
        parser.error("Unrecognized field-component convention")

    with (args.fit_results / "baseline_calibration.csv").open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    calibration = {int(r["board_id"]): r for r in rows}
    if set(calibration) != set(PHYSICAL):
        parser.error("baseline_calibration.csv must contain exactly physical boards 0, 1, 2")
    for b in PHYSICAL:
        if calibration[b]["axis_mode"] != "baseline-dipole":
            parser.error(f"Board {b} is not from baseline-dipole calibration")

    geom = gk.load_ct_geometry(args.ct_xlsx, TRANSFORMS)
    centers = np.vstack([geom["magnet_centers"][b] for b in PHYSICAL])
    # Same CT reference as the desktop app's Linear-stage preset. Fail loudly
    # if another CT workbook/session is accidentally exported under this ID.
    reference_centers = np.array([
        [180.2, 114.4, 114.3],
        [104.1, 95.2, -14.3],
        [17.7, 105.0, 112.4],
    ], dtype=float)
    max_diff = float(np.max(np.abs(centers - reference_centers)))
    if max_diff > 0.05:
        parser.error(
            "CT centers differ from the May-15 linear-stage preset by "
            f"up to {max_diff:.3f} mm. Verify CT workbook, magnet assignment and "
            "coordinate convention; do not relabel this as the May-15 profile."
        )
    field_model = manifest["field_model"]
    diameter = float(field_model["diameter_mm_nominal"]) if (
        "diameter_mm_nominal" in field_model
    ) else float(field_model["diameter_mm"])
    thicknesses = [float(x) for x in field_model["thicknesses_mm_left_top_right"]]

    boards = []
    magnets = []
    for physical, raw in enumerate(PHYSICAL_TO_RAW):
        row = calibration[physical]
        boards.append({
            "raw_board_id": int(raw),
            "sensors_mm": np.asarray(geom["sensor_ct"][physical], float).tolist(),
            "raw_to_world": np.asarray(geom["board_rotations"][physical], float).tolist(),
            "background_mt": [
                float(row["background_x_ct_mT"]),
                float(row["background_y_ct_mT"]),
                float(row["background_z_ct_mT"]),
            ],
        })
        magnets.append({
            "center_mm": np.asarray(geom["magnet_centers"][physical], float).tolist(),
            "axis": [
                float(row["axis_x_ct"]),
                float(row["axis_y_ct"]),
                float(row["axis_z_ct"]),
            ],
            "diameter_mm": diameter,
            "thickness_mm": thicknesses[physical],
            "remanence_mt": float(row["effective_remanence_mT"]),
        })

    bound = float(manifest["optimizer"]["translation_bound_mm"])
    radians = np.deg2rad(float(manifest["optimizer"]["rotation_component_bound_deg"]))
    profile = {
        "name": f"GK CT May-15 {manifest['dataset']} (frozen geometry/calibration)",
        "coordinate_frame": "gk_may15_ct",
        "origin_mm": np.mean(centers, axis=0).tolist(),
        "boards": boards,
        "magnets": magnets,
        "initial_pose": [0.0] * 6,
        "lower_bounds": [-bound] * 3 + [-float(radians)] * 3,
        "upper_bounds": [+bound] * 3 + [+float(radians)] * 3,
        "max_rmse_mt": float(args.max_rmse_mt),
    }
    if args.max_rmse_mt <= 0:
        parser.error("--max-rmse-mt must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote: {args.output}")
    print("Physical-to-raw: Left<-1, Top<-0, Right<-2")
    print("Sensor-to-CT: identity, identity, flip_rot180")
    print("Hall vector transform: historical raw-to-CT rotation (NOT A1 By,-Bx,Bz)")
    print("Magnet centers/axes/scales/backgrounds frozen from CT and baseline calibration.")
    print("Use only with the same rigid mounting geometry and acquisition conventions.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
