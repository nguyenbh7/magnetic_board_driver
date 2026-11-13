"""Synthetic development tests for the independent slice-wise CT nose prior.

No stickers are learned/identified. Oct30 scans 4/5 motivated the failure
cases, hence they are development scans, NOT blinded holdouts.
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location(
    "hdmm_nose_slice_profile", SCRIPTS / "hdmm_nose_slice_profile.py"
)
assert spec and spec.loader
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
from hdmm_anatomy_first import anatomy_proposals, find_anterior_shell_landmark

AFFINE = np.diag([-1., -1., 1., 1.])


def head(*, sticker=True, nose=True, anterior_distractor=False):
    x,y,z=np.ogrid[:128,:128,:128]
    radial=((x-64.)/34.)**2+((y-75.)/42.)**2+((z-63.)/37.)**2
    shell=(radial<=1.)&(radial>=.86)
    v=np.zeros((128,128,128),np.float32)
    v[shell]=850.
    if nose:
        taper=(y>=18)&(y<=40) & (
            ((x-64.)/(2.5 + (y-18.)*.36))**2 +
            ((z-63.)/(4. + (y-18.)*.32))**2 <= 1.
        )
        v[taper]=850.
    if sticker:
        v[60:69,17:19,59:68]=2600.
    if anterior_distractor:
        # Broad upper flap ahead of the nose; it is deliberately NOT nose-like
        # because its near-medial flanks are just as anterior as its center.
        v[50:79,9:65,89:103]=850.
    v[98:103,84:89,65:70]=4000.
    return v


def test_slice_profile_localizes_nose_without_using_prior_sticker_coordinates():
    v=head()
    a=mod.find_nose_slice_profile_landmark(v,AFFINE)
    assert a["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED"
    assert a["landmark_method"]=="slice-profile-v2"
    assert a["does_not_use_previous_sticker_coordinate"] is True
    assert a["selected_profile_slab_count"] >= 2
    assert a["selected_profile_peak_prominence_mm"] >= 5.
    assert np.linalg.norm(np.asarray(a["nose_region_landmark_lps_mm"]) -
                          [64.,24.,63.]) < 11.
    assert a["nose_region_fov"]["center_in_fov"]


def test_upper_broad_front_surface_distracts_old_global_but_not_slice_profile():
    v=head(anterior_distractor=True)
    baseline=find_anterior_shell_landmark(v,AFFINE)
    revised=mod.find_nose_slice_profile_landmark(v,AFFINE)
    assert baseline["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED"
    assert revised["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED"
    old=np.asarray(baseline["nose_region_landmark_lps_mm"])
    new=np.asarray(revised["nose_region_landmark_lps_mm"])
    # The upper flat anterior surface should dominate global anterior extrema.
    assert old[2]>80.
    assert abs(new[2]-63.)<11.
    assert np.linalg.norm(new-[64.,24.,63.])<11.


def test_rounded_head_without_nose_abstains_and_does_not_fall_back():
    # Hollow ellipsoidal shell with no central narrow protrusion.
    v=head(nose=False,sticker=False)
    report=anatomy_proposals(v,AFFINE,landmark_method="slice-profile-v2")
    assert report["status"]=="ABSTAIN_ANATOMY_NOT_RELIABLY_IDENTIFIED"
    assert report["candidate_count"]==0
    assert report["anatomy"]["status"]=="NO_STABLE_BILATERAL_NOSE_PROTRUSION"
    assert report["landmark_method"]=="slice-profile-v2"


def test_high_density_sticker_not_needed_to_locate_nose():
    a=mod.find_nose_slice_profile_landmark(head(sticker=True),AFFINE)
    b=mod.find_nose_slice_profile_landmark(head(sticker=False),AFFINE)
    assert a["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED"
    assert b["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED"
    assert np.linalg.norm(np.asarray(a["nose_region_landmark_lps_mm"]) -
                          np.asarray(b["nose_region_landmark_lps_mm"]))<3.0


def test_integrated_candidate_detection_remains_review_only_and_json_safe():
    result=anatomy_proposals(head(),AFFINE,
                            landmark_method="slice-profile-v2")
    assert result["status"]=="ANATOMY_GUIDED_CANDIDATES_REQUIRE_CT_REVIEW"
    assert result["candidate_count"]>=1
    assert result["landmark_method"]=="slice-profile-v2"
    assert result["automatic_sticker_identity"] is False
    assert result["no_prior_sticker_coordinates_used"] is True
    assert result["candidates"][0]["status"]==(
        "UNVERIFIED_ANATOMY_GUIDED_HIGH_DENSITY_COMPONENT"
    )
    json.dumps(result,allow_nan=False)


def test_translation_of_affine_changes_only_physical_landmark_not_voxel_seed():
    v=head()
    b=mod.find_nose_slice_profile_landmark(v,AFFINE)
    moved=AFFINE.copy()
    shift=np.asarray([11.,-19.,7.])
    moved[:3,3]=[-shift[0],-shift[1],shift[2]]
    a=mod.find_nose_slice_profile_landmark(v,moved)
    assert b["status"]==a["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED"
    assert np.allclose(np.asarray(a["nose_region_landmark_lps_mm"]) -
                       np.asarray(b["nose_region_landmark_lps_mm"]),shift,
                       atol=1e-6)
    assert np.allclose(a["nose_region_landmark_ijk"],
                       b["nose_region_landmark_ijk"],atol=1e-6)


def test_empty_and_invalid_ct_inputs_fail_closed():
    v=np.zeros((128,128,128),np.float32)
    out=mod.find_nose_slice_profile_landmark(v,AFFINE)
    assert out["status"]=="HEAD_SHELL_NOT_FOUND"
    assert "nose_region_landmark_lps_mm" not in out
    with pytest.raises(ValueError,match="reasonably sized"):
        mod.find_nose_slice_profile_landmark(v[:10],AFFINE)
    with pytest.raises(ValueError,match="Unsupported"):
        mod.find_nose_slice_profile_landmark(head(),AFFINE,stride=8)
    with pytest.raises(ValueError,match="Singular"):
        mod.find_nose_slice_profile_landmark(head(),np.zeros((4,4)))


def test_legacy_baseline_remains_explicit_and_unknown_method_errors():
    baseline=anatomy_proposals(head(),AFFINE)
    assert baseline["landmark_method"]=="global-shell-v1"
    with pytest.raises(ValueError,match="Unknown anatomy"):
        anatomy_proposals(head(),AFFINE,landmark_method="guess")
