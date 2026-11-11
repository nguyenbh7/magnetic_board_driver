"""Operator-reviewed anatomy-first CT sticker candidate -> live-app reference.

Run the research anatomy-first detector separately on this EXACT same CT
and detections.json. Review its CT PNG and identify the actual HDMM sticker.
This tool never selects a candidate automatically and never establishes the
true optical marker center; it records a reviewed CT component centroid.

Output is an existing gk_hdmm_sticker_reference_v1 accepted by the desktop
app's "Load reviewed sticker" button *only with the same imported CT*.
An edge-limited candidate or mismatched CT/provenance is rejected.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from hdmm_sticker_geometry import (
    LPS_FROM_RAS, finite_point, load_volume, reviewed_reference,
    write_no_overwrite,
)

FORMAT = "gk_hdmm_anatomy_first_candidates_v1"
CANDIDATE_STATUS = "UNVERIFIED_ANATOMY_GUIDED_HIGH_DENSITY_COMPONENT"


def sha_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(2**20), b""):
            digest.update(block)
    return digest.hexdigest()


def volume_digest(volume, affine):
    """Exact fingerprint convention used by the research anatomy-first code."""
    v = np.asarray(volume)
    a = np.asarray(affine, dtype=np.float64)
    if v.ndim != 3 or a.shape != (4, 4):
        raise ValueError("Expected 3D CT and 4x4 affine")
    if not np.isfinite(a).all() or not np.isfinite(v).all():
        raise ValueError("Nonfinite CT volume/affine")
    digest = hashlib.sha256()
    digest.update(str(v.dtype).encode("ascii"))
    digest.update(json.dumps(list(v.shape)).encode("ascii"))
    digest.update(a.tobytes())
    for slab in v:
        digest.update(np.ascontiguousarray(slab).tobytes())
    return digest.hexdigest()


def reviewed_reference_from_anatomy(
    proposal, selected_id, detections_path, volume, affine, *,
    note, operator_confirmed,
):
    if not operator_confirmed:
        raise ValueError("Explicit operator confirmation of physical sticker identity required")
    if not note or len(note.strip()) < 20:
        raise ValueError("Document how the HDMM sticker was identified in the original CT")
    if proposal.get("format") != FORMAT:
        raise ValueError("Expected anatomy-first sticker candidate report")
    if proposal.get("automatic_sticker_identity") is not False:
        raise ValueError("Proposal cannot claim automatic sticker identification")
    if proposal.get("no_prior_sticker_coordinates_used") is not True:
        raise ValueError("Proposal must be independent anatomy-first CT search")
    if proposal.get("status") != "ANATOMY_GUIDED_CANDIDATES_REQUIRE_CT_REVIEW":
        raise ValueError("Proposal lacks reviewable anatomy-first CT candidates")
    if proposal.get("landmark_method") != "slice-profile-v2":
        raise ValueError("Next-GK preparation requires experimental slice-profile-v2 report")

    if proposal.get("source_detections_sha256") != sha_file(detections_path):
        raise ValueError("CT detections changed or candidate proposal was not tied to this report")
    if proposal.get("source_volume_sha256") != volume_digest(volume, affine):
        raise ValueError("Source CT volume differs from anatomy-first proposal")
    if proposal.get("source_shape_ijk") != list(volume.shape):
        raise ValueError("Candidate proposal CT shape differs")
    a = np.asarray(affine, dtype=np.float64)
    source_affine = np.asarray(proposal.get("source_affine_ras"), dtype=float)
    if (source_affine.shape != (4, 4) or
        not np.array_equal(a, source_affine)):
        raise ValueError("Candidate proposal CT physical affine differs")
    detections = json.loads(Path(detections_path).read_text(encoding="utf-8"))
    if (proposal.get("source_frame_of_reference_uid") !=
            detections.get("frame_of_reference_uid")):
        raise ValueError("CT FrameOfReferenceUID mismatch")
    candidates = [c for c in proposal.get("candidates", [])
                  if c.get("candidate_id") == selected_id]
    if len(candidates) != 1:
        raise ValueError("Candidate ID not found or ambiguous")
    candidate = candidates[0]
    if candidate.get("status") != CANDIDATE_STATUS:
        raise ValueError("Candidate is not an unverified high-density CT component")
    if (candidate.get("touches_scan_boundary") or
        candidate.get("touches_search_roi_boundary") or
        candidate.get("full_support_inside_reconstructed_box") is not True):
        raise ValueError("Component appears cropped or search-ROI-limited; use intact CT")
    lps = finite_point(candidate.get("centroid_lps_mm"), "candidate centroid")
    ijk = finite_point(candidate.get("centroid_ijk"), "candidate voxel centroid")
    physical = LPS_FROM_RAS @ a
    transformed = (physical @ np.r_[ijk, 1.])[:3]
    if float(np.linalg.norm(transformed - lps)) > 0.02:
        raise ValueError("Candidate voxel centroid and physical LPS disagree")
    if (int(candidate.get("voxel_count", 0)) < 8 or
        float(candidate.get("threshold_raw_ct", -1)) not in
        (1200.0, 1600.0, 2000.0, 2400.0)):
        raise ValueError("Unexpected candidate component count or raw CT threshold")
    note = note.strip()
    record = reviewed_reference(
        detections_path,
        lps,
        evidence="CT_VISIBLE_OPERATOR_REVIEWED",
        note=note,
        volume_shape=volume.shape,
        affine_ras=affine,
    )
    record.update({
        "reviewed_candidate_method": "EXPLICIT_CT_REVIEW_SLICE_PROFILE_V2",
        "reviewed_candidate_id": selected_id,
        "reviewed_component_threshold_raw_ct": candidate["threshold_raw_ct"],
        "reviewed_component_voxels": int(candidate["voxel_count"]),
        "reviewed_component_centroid_ijk": ijk.tolist(),
        "source_ct_volume_sha256": proposal["source_volume_sha256"],
        "ct_marker_estimate_is_component_centroid_not_optical_center": True,
        "optical_hdmm_center_calibrated": False,
        "automatic_sticker_identity": False,
        "sticker_motion_requires_same_head_attachment": True,
    })
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proposal", type=Path, required=True)
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--reference-detections", type=Path, required=True)
    parser.add_argument("--reference-scan", type=Path, required=True)
    parser.add_argument("--operator-confirmed", action="store_true")
    parser.add_argument("--note", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        parser.error("Refusing to overwrite existing CT sticker reference")
    proposal = json.loads(args.proposal.read_text(encoding="utf-8"))
    volume, affine = load_volume(args.reference_scan)
    record = reviewed_reference_from_anatomy(
        proposal, args.candidate_id,
        args.reference_detections, volume, affine,
        note=args.note, operator_confirmed=args.operator_confirmed,
    )
    record["anatomy_first_proposal_sha256"] = sha_file(args.proposal)
    record["reference_scan_name"] = args.reference_scan.name
    if args.reference_scan.is_file() and proposal.get("source_archive_sha256"):
        if proposal["source_archive_sha256"] != sha_file(args.reference_scan):
            raise ValueError("Original CT archive hash mismatch")
    json.dumps(record, allow_nan=False)
    write_no_overwrite(args.out, record)
    print("Reviewed reference:", args.out)
    print("CT LPS centroid (NOT calibrated optical center):", record["hdmm_marker_lps_mm"])
    print("Import this JSON with the matching CT in the live app, review, then Set HDMM point.")


if __name__ == "__main__":
    main()
