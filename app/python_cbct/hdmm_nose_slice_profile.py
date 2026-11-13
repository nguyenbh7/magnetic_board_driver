"""Experimental slice-wise nose-surface locator in DICOM LPS, no sticker prior.

Unlike the earlier global-most-anterior-shell heuristic, this method tests
whether the central head surface is locally anterior to *bilateral* flanking
surface at the SAME physical superior-inferior (LPS S) position. Peaks must
persist in consecutive physical slices. If no plausible stable protrusion is
present, abstain. A nose-like bump does NOT identify an HDMM sticker.

This is a development method informed by Oct30 scans 4/5, not an independent
validation, not calibrated for general head orientations, and never authorized
to initiate live tracking.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage as ndi

from hdmm_sticker_geometry import LPS_FROM_RAS, fov_status


METHOD = "slice-profile-v2"
SLAB_MM = 3.0
CENTER_HALF_WIDTH_MM = 5.0
INNER_FLANK_MM = (9.0, 19.0)
OUTER_FLANK_MM = (19.0, 36.0)
CENTER_OFFSETS_MM = tuple(float(x) for x in range(-12, 13, 3))
MIN_PROTRUSION_MM = 5.0
MIN_PERSISTENT_SLABS = 2


def _front_p(p, minimum_count):
    if len(p) < minimum_count:
        return None
    return float(np.percentile(p, 12.))


def _slice_profile(lps, median_l, slab_s):
    """Best bilateral-local convexity around physical head midline at this S."""
    if len(lps) < 45:
        return None
    x, p = lps[:, 0], lps[:, 1]
    results = []
    for offset in CENTER_OFFSETS_MM:
        c = median_l + offset
        center = (np.abs(x-c) <= CENTER_HALF_WIDTH_MM)
        left = (x >= c-INNER_FLANK_MM[1]) & (x <= c-INNER_FLANK_MM[0])
        right = (x >= c+INNER_FLANK_MM[0]) & (x <= c+INNER_FLANK_MM[1])
        left_outer = (x >= c-OUTER_FLANK_MM[1]) & (x <= c-OUTER_FLANK_MM[0])
        right_outer = (x >= c+OUTER_FLANK_MM[0]) & (x <= c+OUTER_FLANK_MM[1])
        fronts = (
            _front_p(p[center], 3), _front_p(p[left], 3),
            _front_p(p[right], 3), _front_p(p[left_outer], 3),
            _front_p(p[right_outer], 3),
        )
        if any(f is None for f in fronts):
            continue
        mid, l_in, r_in, l_out, r_out = fronts
        # A bilateral peak must be ahead of *both* near flanks. The outer
        # flanks penalize a broad, rounded anterior shell/forehead.
        prominence = min(l_in, r_in) - mid
        broad = min(l_out, r_out) - mid
        asymmetry = abs(l_in-r_in)
        if broad < prominence * 0.75:
            continue
        score = prominence - 0.15*asymmetry - 0.045*abs(offset)
        results.append({
            "physical_s_mm": float(slab_s),
            "candidate_medial_l_mm": float(c),
            "anterior_surface_p_mm": float(mid),
            "near_flank_left_front_p_mm": float(l_in),
            "near_flank_right_front_p_mm": float(r_in),
            "prominence_mm": float(prominence),
            "broad_prominence_mm": float(broad),
            "flank_asymmetry_mm": float(asymmetry),
            "score_mm": float(score),
        })
    if not results:
        return None
    return max(results, key=lambda x: x["score_mm"])


def find_nose_slice_profile_landmark(volume, affine, *, stride=3,
                                     shell_min=300., shell_max=2300.,
                                     slab_mm=SLAB_MM,
                                     min_prominence_mm=MIN_PROTRUSION_MM):
    """Return a review-only physical LPS nose search seed or explicit abstention.

    Only geometry and moderate-intensity CT shell voxels are inputs. A target
    sticker threshold, older CT marker center, and image-wise registration
    are NOT supplied to this function.
    """
    v = np.asarray(volume)
    aff = np.asarray(affine, dtype=float)
    if v.ndim != 3 or min(v.shape) < 40 or not np.isfinite(v).all():
        raise ValueError("Expected finite, reasonably sized 3D CBCT")
    if aff.shape != (4,4) or not np.isfinite(aff).all():
        raise ValueError("Expected finite 4x4 affine")
    physical = LPS_FROM_RAS @ aff
    if abs(np.linalg.det(physical[:3, :3])) < 1e-8:
        raise ValueError("Singular CT affine")
    if (stride < 2 or stride > 5 or shell_max <= shell_min
        or not 2. <= slab_mm <= 6.
        or not 2. <= min_prominence_mm <= 25.):
        raise ValueError("Unsupported nose profile parameters")

    sample = v[::stride, ::stride, ::stride]
    labels, num = ndi.label(
        (sample >= shell_min) & (sample <= shell_max),
        structure=np.ones((3,3,3), dtype=np.uint8),
    )
    counts = np.bincount(labels.ravel(), minlength=num+1)
    counts[0] = 0
    order = np.argsort(counts)[::-1]
    context = {
        "landmark_method": METHOD,
        "source_only_CT_geometry": True,
        "does_not_use_previous_sticker_coordinate": True,
        "nose_profile_slab_width_mm": float(slab_mm),
        "minimum_bilateral_protrusion_mm": float(min_prominence_mm),
        "minimum_consecutive_slab_count": MIN_PERSISTENT_SLABS,
        "head_component_downsampled_voxels": int(counts[order[0]]) if len(order) else 0,
        "runner_up_component_downsampled_voxels": (
            int(counts[order[1]]) if len(order)>1 else 0
        ),
    }
    def abstain(status, reason, **extra):
        return dict(context, status=status, reason=reason,
                    candidate_count=0, **extra)
    if not len(order) or counts[order[0]] < 650:
        return abstain("HEAD_SHELL_NOT_FOUND", "No large moderate-density shell")
    if len(order)>1 and counts[order[1]] >= .70*counts[order[0]]:
        return abstain("HEAD_SHELL_AMBIGUOUS", "Competing large shell components")

    pts_ijk = np.argwhere(labels == int(order[0])).astype(float)*stride
    xyz = pts_ijk @ physical[:3,:3].T + physical[:3,3]
    span = np.percentile(xyz,98,axis=0)-np.percentile(xyz,2,axis=0)
    if np.any(span < np.array([45., 30., 40.])):
        return abstain("HEAD_SHELL_SHAPE_UNSUPPORTED",
                       "Head shell is too small in an anatomical dimension",
                       largest_component_span_lps_mm=span.tolist())
    median_l = float((np.percentile(xyz[:,0],5)+np.percentile(xyz[:,0],95))/2.)
    # Anchor the slab lattice to the measured CT shell rather than world
    # S=0; moving the affine origin must not silently change which sampled
    # voxels share a physical slice.
    s_min = float(xyz[:,2].min())
    s_max = float(xyz[:,2].max())
    s_edges = np.arange(s_min - 1e-6, s_max + slab_mm, slab_mm)
    profiles=[]
    for s0,s1 in zip(s_edges[:-1],s_edges[1:]):
        layer = xyz[(xyz[:,2]>=s0)&(xyz[:,2]<s1)]
        row=_slice_profile(layer,median_l,.5*(s0+s1))
        if row is not None:
            row["shell_points_in_slab"]=int(len(layer))
            profiles.append(row)

    eligible = [
        r for r in profiles
        if r["prominence_mm"] >= min_prominence_mm
        and r["score_mm"] >= min_prominence_mm*.7
        and r["flank_asymmetry_mm"] <= 16.
    ]
    eligible.sort(key=lambda r: r["physical_s_mm"])
    runs=[]
    for row in eligible:
        if (runs and abs(row["physical_s_mm"]-runs[-1][-1]["physical_s_mm"]
                         -slab_mm) < .01
            and abs(row["candidate_medial_l_mm"] -
                    runs[-1][-1]["candidate_medial_l_mm"]) <= 9.):
            runs[-1].append(row)
        else:
            runs.append([row])
    stable=[r for r in runs if len(r)>=MIN_PERSISTENT_SLABS]
    context.update({
        "sampled_S_slab_count": int(len(s_edges)-1),
        "bilateral_profile_slab_count": len(profiles),
        "eligible_protrusion_slab_count": len(eligible),
        "stable_protrusion_run_count": len(stable),
        "estimated_medial_L_mm": median_l,
        "largest_component_span_lps_mm": span.tolist(),
        "top_profile_evidence": sorted(
            profiles,key=lambda r:r["score_mm"],reverse=True
        )[:8],
    })
    if not stable:
        return abstain("NO_STABLE_BILATERAL_NOSE_PROTRUSION",
                       "No slice-persistent central nose-like surface with two flanks")
    # Compare integrated support, not just single-slab extreme.
    ranked=sorted(stable,key=lambda run:sum(x["score_mm"] for x in run),
                  reverse=True)
    winner=ranked[0]
    quality=sum(x["score_mm"] for x in winner)
    if len(ranked)>1 and sum(x["score_mm"] for x in ranked[1]) >= 0.85*quality:
        other=ranked[1]
        if abs(np.median([x["physical_s_mm"] for x in winner])-
               np.median([x["physical_s_mm"] for x in other]))>2*slab_mm:
            return abstain(
                "AMBIGUOUS_BILATERAL_NOSE_PROTRUSIONS",
                "Two separated nose-like protrusion runs have comparable support",
            )
    weight=np.asarray([max(x["score_mm"],.01) for x in winner])
    landmark=np.average(
        np.asarray([[r["candidate_medial_l_mm"],
                     r["anterior_surface_p_mm"],
                     r["physical_s_mm"]] for r in winner]),
        axis=0, weights=weight,
    )
    ijk=(np.linalg.inv(physical) @ np.r_[landmark,1.])[:3]
    return dict(context,
        status="ANATOMY_HEURISTIC_REVIEW_REQUIRED",
        nose_region_landmark_lps_mm=landmark.tolist(),
        nose_region_landmark_ijk=ijk.tolist(),
        nose_region_fov=fov_status(v.shape,aff,landmark),
        selected_profile_s_range_mm=[
            float(winner[0]["physical_s_mm"]),
            float(winner[-1]["physical_s_mm"]),
        ],
        selected_profile_slab_count=len(winner),
        selected_profile_peak_prominence_mm=float(
            max(x["prominence_mm"] for x in winner)),
        selected_profile_integrated_score_mm=float(quality),
        selected_profile_evidence=winner,
        anterior_surface_is_not_confirmed_nose=True,
        anatomy_warning="Development-only nose-like peak; nose and sticker identity require CT review",
    )
