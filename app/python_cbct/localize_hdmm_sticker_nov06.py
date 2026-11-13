"""November 6 HDMM sticker: local CT feature proposal, review, and same-session transfer.

propose: Search only near the independently predicted B sticker position, propose
  actual CT high-density connected components, and save a review contact sheet.
approve: An operator selects and visually checks one component in original B CT.
transfer: Propagate that reviewed B reference to A using the independently
  measured stationary-board and three-magnet CT geometry.

Candidate existence/shape and proximity to an Oct30-derived prediction do NOT
independently validate that prediction. Never alter original DICOM, detector
reports, earlier references or live tracking. Outputs refuse overwrite.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from scipy import ndimage as ndi

from hdmm_sticker_geometry import (
    _objects, finite_point, fov_status, reviewed_reference,
    transfer_reference, review_transferred_marker_target,
    load_volume, write_no_overwrite, LPS_FROM_RAS,
)

FORMAT = "gk_hdmm_nov06B_ct_sticker_candidates_v1"
TRANSFER_FORMAT = "gk_hdmm_nov06B_to_A_sticker_transfer_v1"
THRESHOLDS = (1200., 1600., 2000., 2400.)


def sha_file(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as fd:
        for part in iter(lambda: fd.read(2**20), b""):
            h.update(part)
    return h.hexdigest()


def volume_digest(volume, affine):
    """Stable source-image fingerprint, streamed over slices (no 3D copy)."""
    v = np.asarray(volume)
    a = np.asarray(affine, dtype=np.float64)
    if v.ndim != 3 or a.shape != (4, 4):
        raise ValueError("Expected 3D CT volume and 4x4 affine")
    if not np.isfinite(a).all() or not np.isfinite(v).all():
        raise ValueError("Nonfinite CT input")
    h = hashlib.sha256()
    h.update(str(v.dtype).encode("ascii"))
    h.update(json.dumps(list(v.shape)).encode("ascii"))
    h.update(a.tobytes())
    for slab in v:
        h.update(np.ascontiguousarray(slab).tobytes())
    return h.hexdigest()


def lps_to_ijk(affine, lps):
    a = LPS_FROM_RAS @ np.asarray(affine, float)
    if abs(np.linalg.det(a[:3, :3])) < 1e-8:
        raise ValueError("Singular CT affine")
    return (np.linalg.inv(a) @ np.r_[finite_point(lps), 1.])[:3]


def ijk_to_lps(affine, ijk):
    return (LPS_FROM_RAS @ np.asarray(affine, float) @ np.r_[ijk, 1.])[:3]


def prediction_from_prior(record, target_detections):
    """Require an Oct30->B report matching this exact B magnet constellation."""
    if record.get("format") != "gk_hdmm_sticker_conditional_transfer_bundle_v1":
        raise ValueError("Expected Oct30->B conditional sticker transfer bundle")
    prediction = record.get("prediction", {})
    if prediction.get("format") != "gk_hdmm_sticker_transfer_v1":
        raise ValueError("Invalid conditional transfer prediction")
    target = _objects(target_detections)
    if prediction.get("target_frame_of_reference_uid") != target_detections.get(
        "frame_of_reference_uid"
    ):
        raise ValueError("Prior target FrameOfReference UID does not match B report")
    original = prediction.get("target_magnets_lps_mm", {})
    for name in ("magnet_left", "magnet_right", "magnet_superior"):
        expected = finite_point(original.get(name), name)
        current = finite_point(target[name]["center_lps_mm"], name)
        if not np.allclose(expected, current, rtol=0, atol=1e-4):
            raise ValueError("Prior B magnet positions differ from current B detections")
    return finite_point(prediction["predicted_target_hdmm_marker_lps_mm"])


def find_candidates(volume, affine, predicted_lps, *, radius_mm=12.,
                    thresholds=THRESHOLDS, min_voxels=8, max_voxels=5000):
    """Contrast components inside a physically spherical ROI, without identity claims."""
    v = np.asarray(volume)
    if v.ndim != 3 or not np.isfinite(v).all():
        raise ValueError("CT volume must be finite 3D")
    if not 5 <= radius_mm <= 25 or not np.isfinite(radius_mm):
        raise ValueError("Search radius must be 5..25 mm")
    prediction = finite_point(predicted_lps)
    prior_fov = fov_status(v.shape, affine, prediction)
    if not prior_fov["center_in_fov"]:
        raise ValueError("Predicted center outside original B CT; do not segment")
    voxel = lps_to_ijk(affine, prediction)
    physical = LPS_FROM_RAS @ np.asarray(affine, float)
    spacing = np.linalg.norm(physical[:3, :3], axis=0)
    half = np.ceil(radius_mm / spacing).astype(int) + 2
    # Convert numpy integer half-widths to native ints. Otherwise slice
    # boundaries leak np.int64 into the saved JSON search_region_ijk.
    bounds = [slice(max(0, int(np.floor(voxel[k])) - int(half[k])),
                    min(int(v.shape[k]), int(np.ceil(voxel[k])) + int(half[k]) + 1))
              for k in range(3)]
    if any(s.start >= s.stop for s in bounds):
        raise ValueError("Empty local B CT ROI")
    sub = v[tuple(bounds)]
    idx = [np.arange(s.start, s.stop) for s in bounds]
    i, j, k = np.ix_(*idx)
    # Physical spherical ROI even with oblique acquisition axes.
    diff = np.zeros(sub.shape, np.float64)
    for d in range(3):
        delta = (physical[d, 0]*i + physical[d, 1]*j +
                 physical[d, 2]*k + physical[d, 3] - prediction[d])
        diff += delta*delta
    within = diff <= radius_mm*radius_mm
    del diff
    structured = np.ones((3, 3, 3), dtype=np.uint8)
    proposals = []
    vox_mm3 = abs(float(np.linalg.det(physical[:3, :3])))
    for raw_cut in thresholds:
        if not np.isfinite(raw_cut):
            raise ValueError("Threshold must be finite")
        labels, n = ndi.label((sub >= raw_cut) & within, structure=structured)
        counts = np.bincount(labels.ravel(), minlength=n+1)
        for component_id, sl in enumerate(ndi.find_objects(labels), 1):
            if sl is None:
                continue
            count = int(counts[component_id])
            if not min_voxels <= count <= max_voxels:
                continue
            piece = labels[sl] == component_id
            pts = np.argwhere(piece).astype(float)
            pts += np.asarray([x.start for x in sl])
            pts += np.asarray([x.start for x in bounds])
            center = pts.mean(axis=0)
            center_lps = ijk_to_lps(affine, center)
            dist = float(np.linalg.norm(center_lps - prediction))
            bbox_min, bbox_max = pts.min(axis=0), pts.max(axis=0)
            component_fov = fov_status(v.shape, affine, center_lps)
            touches_scan_boundary = bool(np.any(bbox_min <= 0) or np.any(
                bbox_max >= np.asarray(v.shape)-1))
            touches_local_roi = any(
                x.start == 0 or x.stop == sub.shape[d]
                for d, x in enumerate(sl)
            )
            # Flag ROI truncation; do not silently accept pieces cut by it.
            # A component touching the spherical search boundary is also
            # considered incomplete even if it does not touch the box.
            patch_distances = np.linalg.norm(
                pts @ physical[:3, :3].T + physical[:3, 3] - prediction, axis=1
            )
            touches_radius = bool(np.max(patch_distances) >= radius_mm - max(spacing))
            peak = float(np.max(sub[sl][piece]))
            proposals.append({
                "threshold_raw_ct": float(raw_cut),
                "voxel_count": count,
                "component_volume_mm3": float(count*vox_mm3),
                "centroid_ijk": center.tolist(),
                "centroid_lps_mm": center_lps.tolist(),
                "bbox_ijk_inclusive": [bbox_min.astype(int).tolist(),
                                       bbox_max.astype(int).tolist()],
                "distance_from_oct30_prediction_mm": dist,
                "maximum_raw_ct_value": peak,
                "full_support_inside_reconstructed_box": bool(
                    component_fov["support_fully_in_fov"]),
                "component_center_clearance_mm": component_fov[
                    "closest_scan_face_distance_mm"
                ],
                "touches_scan_boundary": touches_scan_boundary,
                "touches_search_roi_boundary": bool(touches_local_roi or touches_radius),
                "status": "UNVERIFIED_HIGH_DENSITY_CT_COMPONENT",
            })
    # With threshold/component size not calibrated, the nearest component
    # is a review candidate, not an automatic sticker identity decision.
    proposals.sort(key=lambda p: (
        p["touches_scan_boundary"] or p["touches_search_roi_boundary"],
        p["distance_from_oct30_prediction_mm"],
        -p["voxel_count"],
        p["threshold_raw_ct"],
    ))
    for n, p in enumerate(proposals):
        p["candidate_id"] = f"C{n+1:02d}"
    return {
        "predicted_center_lps_mm": prediction.tolist(),
        "predicted_center_ijk": voxel.tolist(),
        "prior_fov": prior_fov,
        "search_radius_mm": float(radius_mm),
        "thresholds_raw_ct": [float(t) for t in thresholds],
        "candidate_count": len(proposals),
        "candidates": proposals,
        "search_region_ijk": [[int(s.start), int(s.stop)] for s in bounds],
        "minimum_component_voxels": min_voxels,
        "maximum_component_voxels": max_voxels,
    }


def draw_review(volume, affine, result, png_path):
    """Display candidate and prior positions; never depict an approved marker."""
    if not result["candidates"]:
        # Still save explanatory image; do not invent a candidate.
        center = np.asarray(result["predicted_center_ijk"], float)
        chosen = None
    else:
        chosen = result["candidates"][0]
        center = np.asarray(chosen["centroid_ijk"], float)
    prior = np.asarray(result["predicted_center_ijk"], float)
    fig, axes = plt.subplots(2, 3, figsize=(14, 9), constrained_layout=True)
    for axis in range(3):
        dims = [d for d in range(3) if d != axis]
        index = int(np.clip(round(center[axis]), 0, volume.shape[axis]-1))
        cut = np.take(volume, index, axis=axis).T
        for row, (label, lo, hi) in enumerate((
            ("soft", -650, 500), ("high density", -650, 3000)
        )):
            ax = axes[row, axis]
            ax.imshow(cut, origin="lower", cmap="gray",
                      vmin=lo, vmax=hi, interpolation="nearest")
            ax.plot(prior[dims[0]], prior[dims[1]], marker="x",
                    color="yellow", markersize=10, markeredgewidth=2,
                    label="Oct30 conditional prediction")
            if chosen is not None:
                ax.plot(center[dims[0]], center[dims[1]], marker="+",
                        color="cyan", markersize=14, markeredgewidth=1.8,
                        label=f"{chosen['candidate_id']} CT component centroid")
                lower, upper = chosen["bbox_ijk_inclusive"]
                if lower[axis] <= index <= upper[axis]:
                    ax.add_patch(Rectangle(
                        (lower[dims[0]]-.5, lower[dims[1]]-.5),
                        upper[dims[0]]-lower[dims[0]]+1,
                        upper[dims[1]]-lower[dims[1]]+1,
                        fill=False, linewidth=1.5, linestyle="--",
                        edgecolor="cyan",
                    ))
            ax.set_xlim(max(-.5, center[dims[0]] - 35),
                        min(volume.shape[dims[0]]-.5, center[dims[0]] + 35))
            ax.set_ylim(max(-.5, center[dims[1]] - 35),
                        min(volume.shape[dims[1]]-.5, center[dims[1]] + 35))
            ax.set_title(f"B voxel axis {axis}, index {index}; {label}")
            ax.set_xlabel(f"voxel axis {dims[0]}")
            ax.set_ylabel(f"voxel axis {dims[1]}")
            if axis == 0 and row == 0:
                ax.legend(loc="upper right", fontsize=7)
    fig.suptitle(
        "Nov06B candidate review — yellow X = conditional Oct30 prediction; "
        "cyan + = unverified CT component (box = thresholded extent)"
    )
    out = Path(png_path)
    if out.exists():
        raise FileExistsError(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def build_proposal(volume, affine, detections, prior, *,
                   radius_mm=12., thresholds=THRESHOLDS):
    predicted = prediction_from_prior(prior, detections)
    features = find_candidates(volume, affine, predicted,
                               radius_mm=radius_mm, thresholds=thresholds)
    return {
        "format": FORMAT,
        "coordinate_frame": "DICOM_LPS_mm",
        "status": "LOCAL_CT_CANDIDATES_REQUIRE_OPERATOR_REVIEW",
        "source_scan_role": "NOV06B_OBSERVED_CT_NOT_PRIOR_MODEL",
        "prior_oct30_prediction_is_cross_session_conditional": True,
        "prior_is_not_a_ground_truth_label": True,
        "automatic_marker_identity": False,
        "source_detections_sha256": None,
        "source_volume_sha256": None,
        "prior_transfer_sha256": None,
        "source_frame_of_reference_uid": detections.get("frame_of_reference_uid"),
        "source_volume_shape_ijk": list(volume.shape),
        "source_affine_ras": np.asarray(affine).tolist(),
        "review": features,
        "warning": (
            "Connected bright components may include the shell or reconstruction "
            "artifacts. Candidate-to-prior distance is NOT independent accuracy. "
            "Confirm actual sticker center in original B CT before approving."
        ),
    }


def reviewed_reference_from_candidate(proposal, candidate_id,
                                      detections_path, volume, affine, *,
                                      note, explicitly_confirmed):
    if proposal.get("format") != FORMAT:
        raise ValueError("Expected November 6 B candidate proposal")
    if not explicitly_confirmed:
        raise ValueError("Explicit operator review flag required")
    if not note or len(note.strip()) < 20:
        raise ValueError("Document how the marker center was visually identified")
    if proposal.get("source_detections_sha256") != sha_file(detections_path):
        raise ValueError("B detections changed after candidate proposal")
    if proposal.get("source_volume_sha256") != volume_digest(volume, affine):
        raise ValueError("B CT volume changed after candidate proposal")
    if proposal.get("source_volume_shape_ijk") != list(volume.shape):
        raise ValueError("B CT shape changed")
    options = [p for p in proposal["review"]["candidates"]
               if p.get("candidate_id") == candidate_id]
    if len(options) != 1:
        raise ValueError("Requested candidate ID not found or ambiguous")
    selected = options[0]
    if (selected["touches_scan_boundary"] or selected["touches_search_roi_boundary"]
            or not selected["full_support_inside_reconstructed_box"]):
        raise ValueError("Proposed B component cropped or ROI-edge limited; cannot approve")
    point = finite_point(selected["centroid_lps_mm"])
    record = reviewed_reference(
        detections_path, point,
        evidence="CT_VISIBLE_OPERATOR_REVIEWED",
        note=note, volume_shape=volume.shape, affine_ras=affine
    )
    record.update({
        "marker_center_estimation": "NOV06B_OPERATOR_REVIEWED_HIGH_DENSITY_CONNECTED_COMPONENT",
        "marker_center_exact_optical_center_established": False,
        "source_ct_volume_sha256": proposal["source_volume_sha256"],
        "nov06B_component_candidate_id": candidate_id,
        "nov06B_candidate_report_source_sha256": proposal["prior_transfer_sha256"],
        "component_threshold_raw_ct": selected["threshold_raw_ct"],
        "component_bbox_ijk": selected["bbox_ijk_inclusive"],
        "component_centroid_ijk": selected["centroid_ijk"],
        "component_volume_mm3": selected["component_volume_mm3"],
        "oct30_prior_prediction_residual_mm_descriptive": selected[
            "distance_from_oct30_prediction_mm"
        ],
        "confirmed_sticker_same_physical_attachment_between_B_and_A": False,
        "absolute_sticker_center_uncertainty_measured": False,
    })
    return record


def transfer_B_to_A(reference, detections_B, detections_A,
                    volume_A, affine_A, *, review_radius_mm=12.):
    if reference.get("marker_center_estimation") != (
        "NOV06B_OPERATOR_REVIEWED_HIGH_DENSITY_CONNECTED_COMPONENT"
    ):
        raise ValueError("Only operator-reviewed Nov06B CT reference may transfer to A")
    if not 5 <= review_radius_mm <= 25:
        raise ValueError("Invalid A review radius")
    geometry = transfer_reference(
        reference, detections_B, detections_A,
        target_volume_shape=volume_A.shape,
        target_affine_ras=affine_A,
    )
    local = review_transferred_marker_target(
        geometry, volume_A, affine_A, search_radius_mm=review_radius_mm
    )
    return {
        "format": TRANSFER_FORMAT,
        "coordinate_frame": "DICOM_LPS_mm",
        "status": "NOV06B_DIRECT_CT_REFERENCE_TO_A_RIGID_PREDICTION",
        "same_session_sticker_attachment_rigid_assumption": True,
        "same_session_sticker_attachment_independently_confirmed": False,
        "sticker_center_is_approximate": True,
        "direct_sticker_observation_in_A": False,
        "may_be_partially_or_fully_cropped_in_A": True,
        "B_reference_oct30_cross_session_residual_is_not_tracking_error": True,
        "prediction": geometry,
        "target_review": local,
        "no_live_calibration": True,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    cmds = parser.add_subparsers(dest="mode", required=True)
    p = cmds.add_parser("propose")
    p.add_argument("--ct-b", type=Path, required=True)
    p.add_argument("--detections-b", type=Path, required=True)
    p.add_argument("--oct30-to-b-transfer", type=Path, required=True)
    p.add_argument("--radius-mm", type=float, default=12.)
    p.add_argument("--out", type=Path, required=True)
    a = cmds.add_parser("approve")
    a.add_argument("--proposal", type=Path, required=True)
    a.add_argument("--ct-b", type=Path, required=True)
    a.add_argument("--detections-b", type=Path, required=True)
    a.add_argument("--candidate-id", required=True)
    a.add_argument("--operator-reviewed", action="store_true")
    a.add_argument("--review-note", required=True)
    a.add_argument("--out", type=Path, required=True)
    t = cmds.add_parser("transfer")
    t.add_argument("--reference-b", type=Path, required=True)
    t.add_argument("--detections-b", type=Path, required=True)
    t.add_argument("--detections-a", type=Path, required=True)
    t.add_argument("--ct-a", type=Path, required=True)
    t.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error("Refuse to overwrite previous artifact")
    if args.mode == "propose":
        if args.out.with_suffix(".png").exists():
            parser.error("Refuse to overwrite prior candidate image")
        b_report = json.loads(args.detections_b.read_text(encoding="utf-8"))
        prior = json.loads(args.oct30_to_b_transfer.read_text(encoding="utf-8"))
        vol, affine = load_volume(args.ct_b)
        result = build_proposal(vol, affine, b_report, prior,
                                radius_mm=args.radius_mm)
        result["source_detections_sha256"] = sha_file(args.detections_b)
        result["source_volume_sha256"] = volume_digest(vol, affine)
        result["prior_transfer_sha256"] = sha_file(args.oct30_to_b_transfer)
        # Validate the complete report before rendering the PNG, so an
        # unsupported scalar cannot leave an orphan review image behind.
        json.dumps(result, indent=2, allow_nan=False)
        draw_review(vol, affine, result["review"], args.out.with_suffix(".png"))
        print("Candidate count:", result["review"]["candidate_count"])
        for item in result["review"]["candidates"][:12]:
            print(item["candidate_id"], "threshold", item["threshold_raw_ct"],
                  "nvox", item["voxel_count"],
                  "distance from conditional Oct30 prediction (mm)",
                  f"{item['distance_from_oct30_prediction_mm']:.3f}",
                  "ROI edge", item["touches_search_roi_boundary"],
                  "FOV support", item["full_support_inside_reconstructed_box"])
        print("Review image:", args.out.with_suffix(".png"))
        print("Do not approve without visual review of candidate center and identity")
    elif args.mode == "approve":
        prop = json.loads(args.proposal.read_text(encoding="utf-8"))
        volume, affine = load_volume(args.ct_b)
        result = reviewed_reference_from_candidate(
            prop, args.candidate_id, args.detections_b, volume, affine,
            note=args.review_note, explicitly_confirmed=args.operator_reviewed,
        )
        print("Reviewed B reference LPS mm:", result["hdmm_marker_lps_mm"])
        print("This is a CT component centroid, not calibrated optical center")
    else:
        ref = json.loads(args.reference_b.read_text(encoding="utf-8"))
        if ref.get("reference_detections_sha256") != sha_file(args.detections_b):
            raise ValueError("B source detector report changed since approval")
        b_report = json.loads(args.detections_b.read_text(encoding="utf-8"))
        a_report = json.loads(args.detections_a.read_text(encoding="utf-8"))
        avol, aaff = load_volume(args.ct_a)
        result = transfer_B_to_A(ref, b_report, a_report, avol, aaff)
        print("A predicted marker LPS mm:",
              result["prediction"]["predicted_target_hdmm_marker_lps_mm"])
        print("A reconstructed-box FOV:", result["prediction"]["target_fov"]["status"])
        print("A direct CT review status:", result["target_review"]["status"])
        print("No sticker automatically accepted in A")
    write_no_overwrite(args.out, result)
    print("Saved:", args.out)


if __name__ == "__main__":
    main()
