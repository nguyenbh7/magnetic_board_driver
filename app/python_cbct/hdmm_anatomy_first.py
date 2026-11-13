"""CT anatomy-first HDMM sticker proposal: no prior sticker coordinate.

Use the principal connected head-shell component, CT LPS anterior direction,
and geometric midline to propose the anterior nose region. Search this region
for compact high-density connected components, but NEVER label any one an
HDMM optical sticker automatically.

The head-shell and nose heuristics must be assessed on original, intact CBCTs.
No Oct30 marker coordinate or interscan transfer is used in detection.
The optional 'evaluate' mode compares outputs to an existing operator-reviewed
CT reference *after* proposal; it does not influence any detection step.

Outputs are source-checked, created exclusively, and never enter live fitting.
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
    LPS_FROM_RAS, fov_status, _objects, finite_point, load_volume,
    write_no_overwrite,
)
from localize_hdmm_sticker_nov06 import (
    find_candidates, volume_digest, sha_file,
)
from hdmm_nose_slice_profile import find_nose_slice_profile_landmark

FORMAT = "gk_hdmm_anatomy_first_candidates_v1"
EVAL_FORMAT = "gk_hdmm_anatomy_first_retrospective_evaluation_v1"
HEAD_MIN_VALUE = 300.
HEAD_MAX_VALUE = 2300.
STRIDE = 3
SEARCH_RADIUS_MM = 23.
LANDMARK_METHODS = ("global-shell-v1", "slice-profile-v2")


def ijk_to_lps_batch(affine, points):
    physical = LPS_FROM_RAS @ np.asarray(affine, float)
    return np.asarray(points, float) @ physical[:3, :3].T + physical[:3, 3]


def find_anterior_shell_landmark(volume, affine, *, stride=STRIDE,
                                 shell_min=HEAD_MIN_VALUE,
                                 shell_max=HEAD_MAX_VALUE):
    """Locate probable central anterior shell surface without knowing sticker.

    This is an anatomical HEURISTIC, not a trained nose landmark estimator.
    Reject ambiguous primary components; never silently accept other hardware.
    """
    v = np.asarray(volume)
    if v.ndim != 3 or min(v.shape) < 40 or not np.isfinite(v).all():
        raise ValueError("Expected finite, reasonably sized 3D CBCT")
    if not 2 <= stride <= 5 or shell_max <= shell_min:
        raise ValueError("Invalid anatomy sampling or intensity range")
    physical = LPS_FROM_RAS @ np.asarray(affine, float)
    if physical.shape != (4, 4) or not np.isfinite(physical).all():
        raise ValueError("Invalid physical CT affine")
    if abs(np.linalg.det(physical[:3, :3])) < 1e-8:
        raise ValueError("Singular affine")

    sample = v[::stride, ::stride, ::stride]
    mask = (sample >= shell_min) & (sample <= shell_max)
    labels, n = ndi.label(mask, structure=np.ones((3,3,3), np.uint8))
    sizes = np.bincount(labels.ravel(), minlength=n+1)
    sizes[0] = 0
    order = np.argsort(sizes)[::-1]
    if not len(order) or sizes[order[0]] < 650:
        return {"status":"HEAD_SHELL_NOT_FOUND",
                "reason":"No sufficiently large CT shell component",
                "candidate_count":0}
    if len(order) > 1 and sizes[order[1]] >= 0.70*sizes[order[0]]:
        return {"status":"HEAD_SHELL_AMBIGUOUS",
                "reason":"Two near-equal large CT components; manual review required",
                "candidate_count":0,
                "component_sizes_downsampled": [int(sizes[x]) for x in order[:3]]}

    primary = labels == int(order[0])
    points = np.argwhere(primary).astype(float) * stride
    lps = ijk_to_lps_batch(affine, points)
    # Reject large non-head objects by requiring broad physical extents.
    span = np.percentile(lps, 98, axis=0)-np.percentile(lps, 2, axis=0)
    if np.any(span < [45., 30., 40.]):
        return {
            "status":"HEAD_SHELL_SHAPE_UNSUPPORTED",
            "reason":"Largest component lacks expected head spatial extent",
            "largest_component_span_lps_mm": span.tolist(),
            "candidate_count":0,
        }
    # DICOM LPS: anatomical anterior is toward NEGATIVE P.
    # Estimate mid-sagittal plane from the broad connected shell rather
    # than from a sticker or a stored CT landmark.
    l_mid = float(.5*(np.percentile(lps[:,0], 5)+np.percentile(lps[:,0], 95)))
    midline = np.abs(lps[:,0]-l_mid) <= 20.
    if int(midline.sum()) < 60:
        return {"status":"NO_MIDLINE_SHELL",
                "reason":"Insufficient central anterior shell", "candidate_count":0}
    front_p = np.percentile(lps[midline,1], 1)
    anterior = midline & (lps[:,1] <= front_p + 7.)
    if anterior.sum() < 12:
        return {"status":"NO_ANTERIOR_SHELL",
                "reason":"No stable midline front surface", "candidate_count":0}
    # Prefer the anterior-most quartile and robust median, not an isolated
    # noisy extreme voxel. S is NOT constrained from earlier CT scans.
    front = lps[anterior]
    cutoff = np.percentile(front[:,1], 25)
    focus = front[front[:,1] <= cutoff]
    if len(focus) < 8:
        focus = front
    landmark_lps = np.median(focus, axis=0)
    ijk = np.linalg.inv(physical) @ np.r_[landmark_lps, 1.]
    fov = fov_status(v.shape, affine, landmark_lps)
    return {
        "status":"ANATOMY_HEURISTIC_REVIEW_REQUIRED",
        "head_component_downsampled_voxels":int(sizes[order[0]]),
        "runner_up_component_downsampled_voxels":(
            int(sizes[order[1]]) if len(order)>1 else 0
        ),
        "largest_component_span_lps_mm":span.tolist(),
        "estimated_medial_L_mm":l_mid,
        "midline_width_mm":40.,
        "anterior_surface_band_width_mm":7.,
        "nose_region_landmark_lps_mm":landmark_lps.tolist(),
        "nose_region_landmark_ijk":ijk[:3].tolist(),
        "nose_region_fov":fov,
        "does_not_use_previous_sticker_coordinate":True,
        "anterior_surface_is_not_confirmed_nose":True,
    }


def anatomy_proposals(volume, affine, *, stride=STRIDE,
                      search_radius_mm=SEARCH_RADIUS_MM,
                      landmark_method="global-shell-v1"):
    if landmark_method not in LANDMARK_METHODS:
        raise ValueError("Unknown anatomy landmark method")
    head = (
        find_nose_slice_profile_landmark(volume, affine, stride=stride)
        if landmark_method == "slice-profile-v2" else
        find_anterior_shell_landmark(volume, affine, stride=stride)
    )
    head["landmark_method"] = landmark_method
    if head["status"] != "ANATOMY_HEURISTIC_REVIEW_REQUIRED":
        return {
            "format":FORMAT,
            "status":"ABSTAIN_ANATOMY_NOT_RELIABLY_IDENTIFIED",
            "automatic_sticker_identity":False,
            "anatomy":head,
            "landmark_method":landmark_method,
            "candidate_count":0, "candidates":[],
            "no_prior_sticker_coordinates_used":True,
        }
    search = find_candidates(
        volume, affine, head["nose_region_landmark_lps_mm"],
        radius_mm=search_radius_mm,
        thresholds=(1200.,1600.,2000.,2400.),
        min_voxels=8, max_voxels=5000,
    )
    candidates = []
    for i, src in enumerate(search["candidates"], 1):
        # Remove semantically misleading references to Oct30:
        candidate = {
            key: value for key, value in src.items()
            if key not in ("candidate_id","distance_from_oct30_prediction_mm")
        }
        candidate["candidate_id"] = f"N{i:02d}"
        candidate["distance_from_anatomical_landmark_mm"] = src[
            "distance_from_oct30_prediction_mm"
        ]
        candidate["status"] = "UNVERIFIED_ANATOMY_GUIDED_HIGH_DENSITY_COMPONENT"
        candidates.append(candidate)
    return {
        "format":FORMAT,
        "status":("ANATOMY_GUIDED_CANDIDATES_REQUIRE_CT_REVIEW"
                  if candidates else "NO_PLAUSIBLE_HIGH_DENSITY_COMPONENT"),
        "anatomy":head,
        "landmark_method":landmark_method,
        "search_radius_mm":search_radius_mm,
        "search_region_ijk":search["search_region_ijk"],
        "candidate_count":len(candidates),
        "candidates":candidates,
        "distal_review_focus_candidate_id":choose_distal_review_focus(head, candidates),
        "distal_review_focus_is_not_sticker_identification":True,
        "automatic_sticker_identity":False,
        "no_prior_sticker_coordinates_used":True,
        "limitations":[
            "Anatomical nose estimate is unvalidated and may select forehead/chin/hardware.",
            "No prior sticker coordinates are used; candidate distance is not accuracy.",
            "Connected bright component can contain phantom shell or artifacts.",
            "Search is limited by the reconstructed voxel box, not actual CBCT support.",
            "Cropped marker cannot be reconstructed without a reference scan.",
        ],
    }



def choose_distal_review_focus(anatomy, candidates):
    """Choose a *PNG review focus*, NEVER infer optical sticker identity.

    The slice-profile seed is often superior to the distal nasal tip. Show
    an additional inferior, near-midline dense component prominently, even
    when it is farther from the seed or touches the search-sphere edge.
    No thresholds, segmentation, candidate order, or live approvals change.
    A None result means there is no anatomically plausible review focus.
    """
    if anatomy.get("status") != "ANATOMY_HEURISTIC_REVIEW_REQUIRED":
        return None
    center = anatomy.get("nose_region_landmark_lps_mm")
    if center is None:
        return None
    eligible = []
    for c in candidates:
        p = c.get("centroid_lps_mm")
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            continue
        inferior = center[2] - p[2]
        if (8. <= inferior <= 30. and abs(p[0] - center[0]) <= 18.
            and -20. <= p[1] - center[1] <= 10.
            and c.get("threshold_raw_ct", 0) >= 1200.
            and c.get("voxel_count", 0) >= 8):
            eligible.append(c)
    if not eligible:
        return None
    # Only prioritize for HUMAN review. The largest compact dense nasal-tip
    # component can still be unrelated phantom shell or a partial sticker.
    selected = max(eligible, key=lambda c: (
        float(c["threshold_raw_ct"]), int(c["voxel_count"])
    ))
    return selected["candidate_id"]


def render_anatomy_preview(volume, proposal, path):
    """Three orthogonal views with the anatomy seed and best unverified CT proposal."""
    head = proposal["anatomy"]
    center = head.get("nose_region_landmark_ijk")
    if center is None:
        raise ValueError("No anatomical seed to preview")
    center = np.asarray(center, float)
    focus_id = proposal.get("distal_review_focus_candidate_id")
    candidate = next(
        (c for c in proposal.get("candidates", [])
         if c.get("candidate_id") == focus_id), None
    )
    if candidate is None:
        candidate = proposal["candidates"][0] if proposal["candidates"] else None
    target = np.asarray(candidate["centroid_ijk"], float) if candidate else center
    fig, axes = plt.subplots(2, 3, figsize=(14, 9), constrained_layout=True)
    for axis in range(3):
        dims = [d for d in range(3) if d != axis]
        index = int(np.clip(round(target[axis]),0,volume.shape[axis]-1))
        sl = np.take(volume, index, axis=axis).T
        for row, (label,lo,hi) in enumerate((("soft",-650.,500.),
                                              ("dense",-650.,3000.))):
            ax = axes[row,axis]
            ax.imshow(sl, origin="lower", cmap="gray", vmin=lo, vmax=hi,
                      interpolation="nearest")
            ax.plot(center[dims[0]],center[dims[1]], marker="x",
                    color="yellow",markersize=12,markeredgewidth=1.8)
            if candidate is not None:
                ax.plot(target[dims[0]],target[dims[1]], marker="+",
                        color="cyan",markersize=12,markeredgewidth=1.8)
                bounds = candidate["bbox_ijk_inclusive"]
                if bounds[0][axis]<=index<=bounds[1][axis]:
                    ax.add_patch(Rectangle(
                        (bounds[0][dims[0]]-.5,bounds[0][dims[1]]-.5),
                        bounds[1][dims[0]]-bounds[0][dims[0]]+1,
                        bounds[1][dims[1]]-bounds[0][dims[1]]+1,
                        fill=False,edgecolor="cyan",linestyle="--"))
            ax.set_xlim(max(-.5,target[dims[0]]-48),
                        min(volume.shape[dims[0]]-.5,target[dims[0]]+48))
            ax.set_ylim(max(-.5,target[dims[1]]-48),
                        min(volume.shape[dims[1]]-.5,target[dims[1]]+48))
            ax.set_title(f"Voxel axis {axis}, slice {index}; {label}")
            ax.set_xlabel(f"voxel axis {dims[0]}")
            ax.set_ylabel(f"voxel axis {dims[1]}")
    fig.suptitle(
        "ANATOMY-FIRST / NOT STICKER ID — yellow X = estimated anterior shell, "
        "cyan + = unverified distal review focus (or first candidate)")
    output = Path(path)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=140)
    plt.close(fig)


def evaluate_after_detection(proposal, reviewed, *,
                             detection_sha256):
    """Holdout audit only: no new CT candidates, and no feedback into detector."""
    if proposal.get("format") != FORMAT:
        raise ValueError("Expected anatomy-first proposal")
    if reviewed.get("reference_detections_sha256") != detection_sha256:
        raise ValueError("Reviewed reference not from identical CT detections")
    center = finite_point(reviewed["hdmm_marker_lps_mm"])
    rows = []
    for candidate in proposal.get("candidates",[]):
        xyz = finite_point(candidate["centroid_lps_mm"])
        rows.append({
            "candidate_id":candidate["candidate_id"],
            "raw_ct_threshold":candidate["threshold_raw_ct"],
            "distance_to_reviewed_CT_centroid_mm":float(np.linalg.norm(xyz-center)),
        })
    return {
        "format":EVAL_FORMAT,
        "status":"RETROSPECTIVE_HOLDOUT_COMPARISON_NOT_ABSOLUTE_ACCURACY",
        "reference_center_lps_mm":center.tolist(),
        "reference_is_exact_optical_center":False,
        "uses_reference_during_detection":False,
        "candidates_compared":rows,
        "no_prior_sticker_coordinates_used_during_proposal":True,
    }


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    sub=ap.add_subparsers(dest="mode", required=True)
    p=sub.add_parser("propose")
    p.add_argument("--scan",type=Path,required=True)
    p.add_argument("--detections",type=Path,required=True)
    p.add_argument("--stride",type=int,default=STRIDE)
    p.add_argument("--radius-mm",type=float,default=SEARCH_RADIUS_MM)
    p.add_argument("--landmark-method",choices=LANDMARK_METHODS,
                   default="global-shell-v1",
                   help="Old baseline or experimental bilateral slice-profile detector")
    p.add_argument("--out",type=Path,required=True)
    e=sub.add_parser("evaluate")
    e.add_argument("--proposal",type=Path,required=True)
    e.add_argument("--reviewed-reference",type=Path,required=True)
    e.add_argument("--detections",type=Path,required=True)
    e.add_argument("--out",type=Path,required=True)
    a=ap.parse_args(argv)
    if a.out.exists():
        ap.error("Refusing to overwrite output")
    if a.mode=="evaluate":
        report=json.loads(a.proposal.read_text(encoding="utf-8"))
        reviewed=json.loads(a.reviewed_reference.read_text(encoding="utf-8"))
        det_hash=sha_file(a.detections)
        if report.get("source_detections_sha256") != det_hash:
            raise ValueError("Proposal not from identical CT detections")
        output=evaluate_after_detection(report,reviewed,
                                        detection_sha256=det_hash)
    else:
        if a.out.with_suffix(".png").exists():
            ap.error("Refusing to overwrite review PNG")
        detections=json.loads(a.detections.read_text(encoding="utf-8"))
        _objects(detections)  # Require same original CT detector provenance.
        volume,affine=load_volume(a.scan)
        output=anatomy_proposals(volume,affine,stride=a.stride,
                                 search_radius_mm=a.radius_mm,
                                 landmark_method=a.landmark_method)
        output["source_scan"]=a.scan.name
        output["source_volume_sha256"]=volume_digest(volume,affine)
        output["source_shape_ijk"]=list(volume.shape)
        output["source_affine_ras"]=np.asarray(affine).tolist()
        output["source_detections_sha256"]=sha_file(a.detections)
        output["source_frame_of_reference_uid"]=detections.get("frame_of_reference_uid")
        if Path(a.scan).is_file() and detections.get("source_archive_sha256"):
            actual=sha_file(a.scan)
            if actual.lower()!=detections["source_archive_sha256"].lower():
                raise ValueError("Detection report and source scan archive hash differ")
        json.dumps(output,allow_nan=False)
        if output["anatomy"]["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED":
            render_anatomy_preview(volume,output,a.out.with_suffix(".png"))
        print("Anatomy method:",output["landmark_method"])
        print("Anatomy:",output["anatomy"]["status"])
        print("Anatomical LPS landmark:",output["anatomy"].get(
            "nose_region_landmark_lps_mm"))
        print("Unverified local CT candidates:",output["candidate_count"])
        for c in output["candidates"][:8]:
            print(c["candidate_id"],c["threshold_raw_ct"],c["voxel_count"],
                  "nose distance mm",round(c["distance_from_anatomical_landmark_mm"],3),
                  "edge?",c["touches_scan_boundary"] or c["touches_search_roi_boundary"])
        if output["anatomy"]["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED":
            print("Review image:",a.out.with_suffix(".png"))
        print("No sticker identity or calibrated optical center accepted.")
    write_no_overwrite(a.out,output)
    print("Saved:",a.out)


if __name__=="__main__":
    main()
