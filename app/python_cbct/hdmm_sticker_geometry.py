"""CT HDMM sticker reference/recovery, with no unreviewed automatic identity.

Three explicit steps:
  inspect: candidate CT-contrast structures near an operator-specified nose ROI;
  approve: record a physically verified / visually CT-reviewed reference center;
  transfer: propagate that reference to a cropped CT via stationary receiver
            registration followed by independent three-magnet head motion.

CT candidate detection never auto-approves a sticker. No old Nov06 scan can
recover a completely out-of-FOV marker without independent reference geometry.
Input detections and CT volumes remain unchanged; output cannot overwrite.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from itertools import product

import numpy as np
from scipy import ndimage as ndi

# Self-contained module: this is also bundled under the standalone
# magnetic_board_driver/app/python_cbct path for the next GK experiment.
# Algorithm and safety checks match the existing CT A/B board and magnet
# registration; there is NO run-time import from the research checkout.
MAGNETS = ("magnet_left", "magnet_right", "magnet_superior")
BOARDS = ("array_left", "array_right")


def _objects(report):
    if report.get("format") != "gk_cbct_detections_v1" or \
       report.get("coordinate_frame") != "DICOM_LPS_mm":
        raise ValueError("expected original DICOM-LPS CT detector report")
    source = report.get("objects")
    if not isinstance(source, list):
        raise ValueError("CT detector report has no object list")
    names = [o.get("name") for o in source]
    if len(set(names)) != len(names):
        raise ValueError("ambiguous duplicate CT object names")
    objects = {o["name"]: o for o in source}
    for name in MAGNETS:
        if name not in objects or objects[name].get("kind") != "magnet":
            raise ValueError("missing named CT magnet " + name)
        finite_point(objects[name]["center_lps_mm"], name)
    for name in BOARDS:
        if name not in objects or objects[name].get("kind") != "array":
            raise ValueError("missing stationary receiver grid " + name)
        grid = np.asarray(objects[name].get("chip_grid_lps_mm"), float)
        if grid.shape != (16,3) or not np.isfinite(grid).all():
            raise ValueError("invalid 16-chip receiver grid " + name)
    return objects


def fit_points(a,b):
    """Proper row-vector least-squares 3D rigid transform a@R+t=b."""
    a, b = np.asarray(a,float), np.asarray(b,float)
    if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 3 or len(a) < 3:
        raise ValueError("rigid fit requires >=3 matching three-dimensional points")
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("nonfinite rigid reference geometry")
    ac, bc = a.mean(axis=0), b.mean(axis=0)
    U, singular, Vt = np.linalg.svd((a-ac).T @ (b-bc))
    if singular[1] <= 1e-7 or not np.isfinite(singular).all():
        raise ValueError("CT rigid geometry is collinear/degenerate")
    R = U @ np.diag([1.,1.,np.linalg.det(U @ Vt)]) @ Vt
    t = bc - ac @ R
    errors = np.linalg.norm(a @ R + t - b, axis=1)
    return {"rotation":R,"translation":t,
            "rms_mm":float(np.sqrt(np.mean(errors**2)))}


def _grid_permutations():
    ids = np.arange(16).reshape((4,4))
    result = []
    for turns in range(4):
        rotated = np.rot90(ids, turns)
        for option in (rotated, np.fliplr(rotated)):
            perm = option.ravel().tolist()
            if perm not in result:
                result.append(perm)
    return result


def stationary_registration(a,b):
    """32 physical chips on stationary side boards, including 4x4 symmetries."""
    old = np.concatenate([np.asarray(a[n]["chip_grid_lps_mm"],float)
                          for n in BOARDS])
    permutations = _grid_permutations()
    best = None
    for left,right in product(permutations,repeat=2):
        new = np.concatenate([
            np.asarray(b[BOARDS[0]]["chip_grid_lps_mm"],float)[left],
            np.asarray(b[BOARDS[1]]["chip_grid_lps_mm"],float)[right],
        ])
        try:
            fit = fit_points(old,new)
        except ValueError as exc:
            # Some *incorrect 4x4 index permutations* have rank-deficient
            # cross-covariance even when the physical boards are valid.
            # Reject only that pairing; do not reject the entire CT scan.
            if "collinear/degenerate" not in str(exc):
                raise
            continue
        if best is None or fit["rms_mm"] < best["fit"]["rms_mm"]:
            best = {"fit":fit, "permutations":{"left":left,"right":right}}
    if best is None:
        raise ValueError("no nondegenerate stationary board registration")
    return best


FORMAT = "gk_hdmm_sticker_reference_v1"
TRANSFER_FORMAT = "gk_hdmm_sticker_transfer_v1"
SCAN_FORMAT = "gk_hdmm_sticker_roi_candidates_v1"
LPS_FROM_RAS = np.diag([-1., -1., 1., 1.])


def finite_point(value, label="position"):
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,) or not np.isfinite(arr).all():
        raise ValueError(f"{label} must be three finite LPS millimetre coordinates")
    return arr


def affine_lps(affine_ras):
    affine = np.asarray(affine_ras, float)
    if affine.shape != (4, 4) or not np.isfinite(affine).all():
        raise ValueError("invalid finite 4x4 RAS affine")
    result = LPS_FROM_RAS @ affine
    if abs(np.linalg.det(result[:3, :3])) < 1e-8:
        raise ValueError("non-invertible CT physical affine")
    return result


def fov_status(shape, affine_ras, point_lps, *, marker_radius_mm=4.0):
    """Continuous voxel coordinates & signed closest FOV-face distance (mm).

    The distance uses each oriented voxel axis normal, not voxel-space
    coordinate distances; this works for oblique DICOM acquisitions.
    Positive = contained within the imaged parallelepiped, negative = outside.
    4 mm is a configurable *support region*, not an inferred sticker size.
    """
    shape = tuple(int(k) for k in shape)
    if len(shape) != 3 or min(shape) < 2:
        raise ValueError("expected nonempty 3D CT shape")
    if not np.isfinite(marker_radius_mm) or marker_radius_mm <= 0:
        raise ValueError("marker support radius must be finite and positive")
    physical = affine_lps(affine_ras)
    pt = finite_point(point_lps)
    ijk = (np.linalg.inv(physical) @ np.r_[pt, 1.])[:3]
    inverse = np.linalg.inv(physical[:3, :3])
    # For each i/j/k face the reciprocal row norm gives distance in mm
    # to the physical plane. Center-of-outermost-voxel is considered the
    # physical support boundary; we do not extrapolate unseen anatomy.
    plane_mm = 1. / np.linalg.norm(inverse, axis=1)
    near = ijk * plane_mm
    far = (np.asarray(shape, float) - 1 - ijk) * plane_mm
    signed = float(np.min(np.minimum(near, far)))
    where = ("OUTSIDE_FOV" if signed < 0 else
             "CROPPED_OR_EDGE_LIMITED" if signed < marker_radius_mm else
             "CENTER_AND_SUPPORT_INSIDE_FOV")
    return {
        "status": where,
        "center_in_fov": bool(signed >= 0),
        "support_fully_in_fov": bool(signed >= marker_radius_mm),
        "closest_scan_face_distance_mm": round(signed, 4),
        "assumed_support_radius_mm": float(marker_radius_mm),
        "voxel_position_ijk": ijk.tolist(),
        "scan_shape_ijk": list(shape),
        "only_checks_reconstructed_voxel_grid_bounds": True,
        "actual_cylindrical_cbct_or_masked_reconstruction_support_unverified": True,
        "not_direct_sticker_detection": True,
    }


def _contrast_candidates(volume, affine_ras, center_lps, *,
                         roi_radius_mm=25.0,
                         percentiles=(97., 99., 99.7),
                         min_vol_mm3=0.3,
                         max_vol_mm3=250.0):
    """Both bright and dark local features; never a trained sticker classifier.

    An independently approximate nose/reflector region is required because
    isolated CT features across the whole FOV are not identifiable as an
    optical sticker. Return several plausible compact features for review.
    """
    vol = np.asarray(volume)
    if vol.ndim != 3 or not np.isfinite(vol).all():
        raise ValueError("CT volume must contain finite 3D samples")
    if not np.isfinite(roi_radius_mm) or not 5 <= roi_radius_mm <= 60:
        raise ValueError("ROI radius must be 5–60 mm")
    physical = affine_lps(affine_ras)
    center = finite_point(center_lps, "search center")
    roi_ijk = (np.linalg.inv(physical) @ np.r_[center, 1.])[:3]
    spacing = np.linalg.norm(physical[:3, :3], axis=0)
    margin = np.ceil(roi_radius_mm / spacing).astype(int) + 2
    bounds = [
        slice(max(0, int(np.floor(roi_ijk[i])) - int(margin[i])),
              min(vol.shape[i], int(np.ceil(roi_ijk[i])) + int(margin[i]) + 1))
        for i in range(3)
    ]
    if any(sl.start >= sl.stop for sl in bounds):
        return []
    roi = vol[tuple(bounds)]
    if roi.size > 8_000_000:
        raise ValueError("search ROI exceeds 8 million voxels; use smaller radius")
    data = roi.astype(np.float32, copy=False)
    smooth = ndi.gaussian_filter(data, sigma=np.maximum(.35 / spacing, .25))
    background = ndi.gaussian_filter(data, sigma=np.maximum(3. / spacing, 1.))
    contrast = smooth - background
    # Use broadcasted coordinate axes, not an N-by-3 world-coordinate
    # allocation (which is expensive for a clinical CT volume).
    idx = [np.arange(b.start, b.stop, dtype=np.float32)
           for b in bounds]
    ii, jj, kk = np.ix_(*idx)
    diff0 = (physical[0, 0] * ii + physical[0, 1] * jj +
             physical[0, 2] * kk + physical[0, 3] - center[0])
    diff1 = (physical[1, 0] * ii + physical[1, 1] * jj +
             physical[1, 2] * kk + physical[1, 3] - center[1])
    diff2 = (physical[2, 0] * ii + physical[2, 1] * jj +
             physical[2, 2] * kk + physical[2, 3] - center[2])
    in_roi = diff0 ** 2 + diff1 ** 2 + diff2 ** 2 <= roi_radius_mm ** 2
    del diff0, diff1, diff2
    vox_mm3 = abs(float(np.linalg.det(physical[:3, :3])))
    items = []
    for sign in (1., -1.):
        values = (sign * contrast)[in_roi]
        positives = values[values > 0]
        if len(positives) < 4:
            continue
        for percentile in percentiles:
            cut = float(np.percentile(positives, percentile))
            if cut <= 0:
                continue
            mask = (sign * contrast >= cut) & in_roi
            labeled, _ = ndi.label(mask)
            counts = np.bincount(labeled.ravel())
            for component_id, slices in enumerate(ndi.find_objects(labeled), 1):
                if slices is None:
                    continue
                n = int(counts[component_id])
                physical_vol = n * vox_mm3
                if not min_vol_mm3 <= physical_vol <= max_vol_mm3:
                    continue
                extent = np.array([sl.stop - sl.start for sl in slices]) * spacing
                if extent.max() > 18.0:
                    continue
                part = labeled[slices] == component_id
                pts = np.argwhere(part).astype(float)
                local_center = pts.mean(axis=0) + np.array([sl.start for sl in slices])
                image_ijk = local_center + np.array([sl.start for sl in bounds])
                xyz = (physical @ np.r_[image_ijk, 1.])[:3]
                peak = float(np.max((sign * contrast)[slices][part]))
                items.append({
                    "center_lps_mm": xyz.tolist(),
                    "distance_to_search_center_mm": float(np.linalg.norm(xyz - center)),
                    "signed_contrast": "bright" if sign > 0 else "dark",
                    "local_contrast_peak": peak,
                    "component_volume_mm3": round(physical_vol, 4),
                    "extent_mm": extent.tolist(),
                    "percentile": percentile,
                    "nearest_scan_face": fov_status(vol.shape, affine_ras, xyz),
                    "identity": "UNVERIFIED_CT_STICKER_CANDIDATE",
                })
    # Collapse duplicate detections across percentiles; rank by distance to
    # an approximate independently specified nose point and local contrast.
    items.sort(key=lambda x: (x["distance_to_search_center_mm"],
                              -x["local_contrast_peak"]))
    output = []
    for item in items:
        pt = np.asarray(item["center_lps_mm"])
        if any(np.linalg.norm(pt - np.asarray(prev["center_lps_mm"])) < 2.0
               for prev in output):
            continue
        output.append(item)
        if len(output) >= 25:
            break
    return output


def candidate_report(volume, affine_ras, search_center_lps_mm, *, radius_mm=25):
    volume = np.asarray(volume)
    candidates = _contrast_candidates(volume, affine_ras, search_center_lps_mm,
                                      roi_radius_mm=radius_mm)
    return {
        "format": SCAN_FORMAT,
        "coordinate_frame": "DICOM_LPS_mm",
        "status": "CANDIDATES_ONLY_REQUIRES_PHYSICAL_CT_REVIEW",
        "search_center_lps_mm": finite_point(search_center_lps_mm).tolist(),
        "search_center_fov": fov_status(volume.shape, affine_ras,
                                       search_center_lps_mm,
                                       marker_radius_mm=radius_mm),
        "candidate_count": len(candidates),
        "candidates": candidates,
        "automatic_hdmm_identity": False,
        "warning": (
            "The HDMM sticker is an optical object; it may be CT-invisible. "
            "Candidates are NOT approved even if bright, midline or unique. "
            "A truncated nose ROI may exclude the actual marker."
        ),
    }


def _json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reviewed_reference(reference_detections_path, point_lps_mm, *,
                       evidence, note, volume_shape=None, affine_ras=None,
                       marker_radius_mm=4.):
    """Only an operator-confirmed CT reference or physical measurement.

    Do not silently convert an unreviewed candidate into an accepted marker.
    """
    if evidence not in ("CT_VISIBLE_OPERATOR_REVIEWED",
                        "INDEPENDENT_PHYSICAL_RIGID_OFFSET"):
        raise ValueError("evidence must be operator-reviewed CT or independent physical metrology")
    if not note or len(note.strip()) < 12:
        raise ValueError("include an independently documented sticker-identification note")
    src = _json(reference_detections_path)
    objects = _objects(src)
    centers = [finite_point(objects[name]["center_lps_mm"]) for name in MAGNETS]
    if min(np.linalg.norm(centers[i] - centers[j])
           for i in range(3) for j in range(i + 1, 3)) < 15:
        raise ValueError("reference magnet constellation degenerate")
    pt = finite_point(point_lps_mm, "reviewed sticker")
    coverage = (fov_status(volume_shape, affine_ras, pt,
                           marker_radius_mm=marker_radius_mm)
                if volume_shape is not None and affine_ras is not None else None)
    if evidence == "CT_VISIBLE_OPERATOR_REVIEWED":
        if coverage is None:
            raise ValueError("CT-visible evidence requires checking original scan FOV")
        if not coverage["support_fully_in_fov"]:
            raise ValueError("CT-visible marker reference is cropped or outside scan: use intact reference")
    return {
        "format": FORMAT,
        "coordinate_frame": "DICOM_LPS_mm",
        "status": "OPERATOR_REVIEWED_REFERENCE_NOT_ALGORITHM_VERIFIED",
        "hdmm_marker_lps_mm": pt.tolist(),
        "evidence_type": evidence,
        "evidence_note": note.strip(),
        "reference_detections_sha256": _sha_file(reference_detections_path),
        "reference_frame_of_reference_uid": src.get("frame_of_reference_uid"),
        "reference_magnets_lps_mm": {name: centers[i].tolist()
                                     for i, name in enumerate(MAGNETS)},
        "reference_fov": coverage,
        "immutable_reference": True,
        "reference_to_head_rigidity_is_an_assumption": True,
        "must_reconfirm_marker_not_moved_relative_to_magnets": True,
        "independently_validated_absolute_marker_accuracy": False,
    }


def transfer_reference(reference_record, reference_detections, target_detections,
                       *, target_volume_shape=None, target_affine_ras=None,
                       marker_radius_mm=4.0,
                       max_stationary_rms_mm=0.75,
                       max_magnet_rms_mm=0.75,
                       max_constellation_change_mm=1.0):
    """CT reference->target: stationary board registration then head motion.

    No optical measurements, Hall fit, target BB, or approximate clock are
    used to choose transform, so marker prediction remains independent of
    the intended HDMM validation.
    """
    if reference_record.get("format") != FORMAT:
        raise ValueError("required reviewed HDMM reference record")
    if reference_record.get("coordinate_frame") != "DICOM_LPS_mm":
        raise ValueError("reference is not CT LPS")
    expected_hash = reference_record.get("reference_detections_sha256")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise ValueError("reference detection provenance hash is missing")
    for label, data in [("reference", reference_detections), ("target", target_detections)]:
        if data.get("format") != "gk_cbct_detections_v1":
            raise ValueError(f"{label} requires accepted CT detection data")
        if data.get("coordinate_frame") != "DICOM_LPS_mm":
            raise ValueError(f"{label} is not DICOM LPS")
    original = _objects(reference_detections)
    target = _objects(target_detections)
    if not reference_record.get("evidence_note") or not reference_record.get("evidence_type"):
        raise ValueError("unreviewed reference has no physical sticker identification evidence")
    if reference_record.get("status") != "OPERATOR_REVIEWED_REFERENCE_NOT_ALGORITHM_VERIFIED":
        raise ValueError("cannot transfer unreviewed candidate as reflector")
    for key, bound in [("max_stationary_rms_mm", max_stationary_rms_mm),
                       ("max_magnet_rms_mm", max_magnet_rms_mm),
                       ("max_constellation_change_mm", max_constellation_change_mm)]:
        if not np.isfinite(bound) or not 0.0 < bound <= 5.0:
            raise ValueError(f"invalid {key}")
    old_magnets = np.stack([finite_point(original[n]["center_lps_mm"]) for n in MAGNETS])
    new_magnets = np.stack([finite_point(target[n]["center_lps_mm"]) for n in MAGNETS])
    for i, name in enumerate(MAGNETS):
        expected = finite_point(reference_record["reference_magnets_lps_mm"][name])
        if not np.allclose(old_magnets[i], expected, atol=1e-5, rtol=0):
            raise ValueError("reference detector magnets changed since physical sticker approval")
    if min(np.linalg.norm(old_magnets[i] - old_magnets[j])
           for i in range(3) for j in range(i + 1, 3)) < 15:
        raise ValueError("degenerate magnet configuration")
    constellation_delta = max(
        abs(np.linalg.norm(old_magnets[i] - old_magnets[j]) -
            np.linalg.norm(new_magnets[i] - new_magnets[j]))
        for i in range(3) for j in range(i + 1, 3)
    )
    if constellation_delta > max_constellation_change_mm:
        raise ValueError(f"magnet rigid constellation changed {constellation_delta:.3f} mm")
    registered = stationary_registration(original, target)["fit"]
    if registered["rms_mm"] > max_stationary_rms_mm:
        raise ValueError(f"fixed-board CT registration RMS {registered['rms_mm']:.3f} mm too high")
    moving_old = old_magnets @ registered["rotation"] + registered["translation"]
    head = fit_points(moving_old, new_magnets)
    if head["rms_mm"] > max_magnet_rms_mm:
        raise ValueError(f"head magnet rigid fit RMS {head['rms_mm']:.3f} mm too high")
    position = finite_point(reference_record["hdmm_marker_lps_mm"])
    predicted = ((position @ registered["rotation"] + registered["translation"])
                 @ head["rotation"] + head["translation"])
    fov = (fov_status(target_volume_shape, target_affine_ras, predicted,
                       marker_radius_mm=marker_radius_mm)
           if target_volume_shape is not None and target_affine_ras is not None else {
               "status": "TARGET_FOV_UNCHECKED",
               "not_direct_sticker_detection": True,
           })
    return {
        "format": TRANSFER_FORMAT,
        "coordinate_frame": "DICOM_LPS_mm",
        "status": "RIGID_TRANSFER_ESTIMATE_NOT_DIRECT_CT_DETECTION",
        "reference_hdmm_marker_lps_mm": position.tolist(),
        "predicted_target_hdmm_marker_lps_mm": predicted.tolist(),
        "reference_evidence_type": reference_record["evidence_type"],
        "reference_evidence_note": reference_record["evidence_note"],
        "reference_detections_sha256": reference_record["reference_detections_sha256"],
        "reference_frame_of_reference_uid": reference_record.get("reference_frame_of_reference_uid"),
        "target_frame_of_reference_uid": target_detections.get("frame_of_reference_uid"),
        "target_magnets_lps_mm": {
            name: finite_point(target[name]["center_lps_mm"]).tolist()
            for name in MAGNETS
        },
        "stationary_receiver_registration_rms_mm": round(float(registered["rms_mm"]), 6),
        "head_magnet_rigid_fit_rms_mm": round(float(head["rms_mm"]), 6),
        "magnet_constellation_max_distance_change_mm": round(float(constellation_delta), 6),
        "target_fov": fov,
        "marker_was_directly_identified_in_target": False,
        "geometric_model":"CT stationary board registration then proper 3D three-magnet rigid transform",
        "transfer_must_be_approved_by_operator": True,
        "marker_relative_to_magnets_must_be_rigid": True,
        "uncertainty_not_equal_to_fit_rms": True,
        "not_independent_validation_of_hdmm_or_hall": True,
    }


def review_transferred_marker_target(
    transfer, volume, affine_ras, *, search_radius_mm=12.0,
):
    """Inspect a transfer in target CT without promoting a blob to a sticker.

    Uses the target prediction established WITHOUT optical HDMM data; a
    nearby bright/dark spot is only a candidate. Partial or missing FOV
    must remain explicitly unconfirmed. No source data are modified.
    """
    if transfer.get("format") != TRANSFER_FORMAT or (
        transfer.get("status") != "RIGID_TRANSFER_ESTIMATE_NOT_DIRECT_CT_DETECTION"
    ):
        raise ValueError("review requires a gated rigid-transfer report")
    data = np.asarray(volume)
    if data.ndim != 3 or not np.isfinite(data).all():
        raise ValueError("target CT must be a finite 3D volume")
    center = finite_point(transfer["predicted_target_hdmm_marker_lps_mm"])
    target_fov = fov_status(
        data.shape, affine_ras, center,
        marker_radius_mm=(
            transfer.get("target_fov", {}).get("assumed_support_radius_mm", 4.0)
        ),
    )
    output = {
        "format": "gk_hdmm_sticker_transfer_target_review_v1",
        "coordinate_frame": "DICOM_LPS_mm",
        "predicted_target_hdmm_marker_lps_mm": center.tolist(),
        "target_fov": target_fov,
        "transferred_from_reviewed_reference": True,
        "direct_target_sticker_identification": False,
        "optical_hdmm_data_used_for_candidate_ranking": False,
        "source_data_modified": False,
        "nearby_ct_candidates": [],
        "status": "PREDICTED_TARGET_OUTSIDE_RECONSTRUCTED_FOV",
    }
    if target_fov["status"] == "OUTSIDE_FOV":
        output["note"] = (
            "Target center lies outside the reconstructed volume. No target "
            "sticker coordinates were measured; keep the rigid transfer as "
            "a model-derived estimate only."
        )
        return output
    nearby = candidate_report(
        data, affine_ras, center, radius_mm=search_radius_mm
    )
    output["nearby_ct_candidates"] = nearby["candidates"]
    output["nearby_candidate_count"] = nearby["candidate_count"]
    if target_fov["status"] == "CROPPED_OR_EDGE_LIMITED":
        output["status"] = "TARGET_EDGE_CROPPED_UNCONFIRMED"
        output["note"] = (
            "Sticker support intersects the reconstructed-volume edge. "
            "Visible fragments might be among the candidates; without a "
            "reviewed shape/point correspondence the complete center is "
            "NOT measured from this scan."
        )
    else:
        output["status"] = "TARGET_FULL_FOV_CANDIDATES_ONLY"
        output["note"] = (
            "Review these predicted-location CT candidates in the actual "
            "images against an independently documented sticker appearance. "
            "The transfer remains unverified by this scan; neither nearest "
            "blob nor strongest contrast is automatically accepted."
        )
    return output


def load_volume(scan_path):
    from cbct_detect import scan_path as resolve_scan
    import detector_core as core
    with resolve_scan(Path(scan_path)) as resolved:
        image = core.load_scan(resolved)
        data = image.data[0].numpy()
        return data, image.affine.copy()


def write_no_overwrite(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as file:
        json.dump(record, file, indent=2, allow_nan=False)
        file.write("\n")


def cli(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    commands = ap.add_subparsers(dest="mode", required=True)
    inspect = commands.add_parser("inspect", help="locally propose bright/dark CT sticker candidates")
    inspect.add_argument("--reference-scan", required=True, type=Path)
    inspect.add_argument("--search-center-lps-mm", nargs=3, type=float, required=True, metavar=("L","P","S"))
    inspect.add_argument("--radius-mm", type=float, default=25.)
    inspect.add_argument("--out", required=True, type=Path)

    approve = commands.add_parser("approve", help="record independent operator-reviewed reference")
    approve.add_argument("--reference-detections", required=True, type=Path)
    approve.add_argument("--reference-scan", type=Path)
    approve.add_argument("--marker-lps-mm", nargs=3, type=float, required=True, metavar=("L","P","S"))
    approve.add_argument("--evidence", choices=["CT_VISIBLE_OPERATOR_REVIEWED",
                                                "INDEPENDENT_PHYSICAL_RIGID_OFFSET"], required=True)
    approve.add_argument("--note", required=True)
    approve.add_argument("--out", required=True, type=Path)

    transfer = commands.add_parser("transfer", help="predict reviewed marker position in target CT")
    transfer.add_argument("--reference-json", required=True, type=Path)
    transfer.add_argument("--reference-detections", required=True, type=Path)
    transfer.add_argument("--target-detections", required=True, type=Path)
    transfer.add_argument("--target-scan", type=Path)
    transfer.add_argument("--review-target-radius-mm", type=float, default=None,
                          help="if target CT is provided, inspect +/- ROI around transferred sticker; leave unset to skip")
    transfer.add_argument("--out", required=True, type=Path)

    args = ap.parse_args(argv)
    if args.mode == "inspect":
        volume, affine = load_volume(args.reference_scan)
        record = candidate_report(volume, affine, args.search_center_lps_mm,
                                  radius_mm=args.radius_mm)
    elif args.mode == "approve":
        dims, affine = (None, None)
        if args.reference_scan:
            volume, affine = load_volume(args.reference_scan)
            dims = volume.shape
        record = reviewed_reference(
            args.reference_detections, args.marker_lps_mm,
            evidence=args.evidence, note=args.note,
            volume_shape=dims, affine_ras=affine,
        )
    else:
        reference = _json(args.reference_json)
        source = _json(args.reference_detections)
        if _sha_file(args.reference_detections) != reference.get("reference_detections_sha256"):
            raise ValueError("reference detections file changed since marker approval")
        target = _json(args.target_detections)
        dims, affine = (None, None)
        if args.target_scan:
            volume, affine = load_volume(args.target_scan)
            dims = volume.shape
        record = transfer_reference(reference, source, target,
                                    target_volume_shape=dims, target_affine_ras=affine)
        if args.review_target_radius_mm is not None:
            if not args.target_scan:
                raise ValueError("--review-target-radius-mm requires --target-scan")
            record["target_review"] = review_transferred_marker_target(
                record, volume, affine,
                search_radius_mm=args.review_target_radius_mm,
            )
    write_no_overwrite(args.out, record)
    print(f"{record['status']}: {args.out}")
    if args.mode == "inspect":
        print(f"Candidates: {record['candidate_count']}; identity is NOT automatic")
    if args.mode == "transfer":
        print("Predicted target LPS mm:", record["predicted_target_hdmm_marker_lps_mm"])
        print("Target FOV:", record["target_fov"]["status"])
        if "target_review" in record:
            review = record["target_review"]
            print("Target CT review:", review["status"],
                  "candidates:", len(review["nearby_ct_candidates"]))
            print("DO NOT treat transfer/candidate as directly detected sticker.")


if __name__ == "__main__":
    cli()
