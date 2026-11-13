"""Synthetic regression tests of anatomy-first HDMM sticker search and abstention."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
spec = importlib.util.spec_from_file_location(
    "hdmm_anatomy_first",SCRIPT_DIR/"hdmm_anatomy_first.py"
)
assert spec is not None and spec.loader is not None
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

LPS_IDENTITY_AFFINE = np.diag([-1., -1., 1., 1.])


def mannequin(with_sticker=True):
    """Hollow 3D ellipsoidal head and independently protruding central nose."""
    shape=(128,128,128)
    x,y,z=np.ogrid[:128,:128,:128]
    radial=((x-64.)/34.)**2+((y-75.)/42.)**2+((z-63.)/37.)**2
    shell=(radial<=1.)&(radial>=.86)
    v=np.zeros(shape,np.float32)
    v[shell]=850.
    # Sculpt a tapered nasal ridge which CONNECTS to anterior head.
    # LPS +P is posterior, so nose points toward decreasing y.
    anterior=(y>=18)&(y<=40)
    width=2.5 + (y-18.)*.36
    height=4. + (y-18.)*.32
    nose=anterior & (((x-64.)/width)**2+((z-63.)/height)**2<=1.)
    v[nose]=850.
    if with_sticker:
        # Distinct plate against tip of the nose; top intensities 2600.
        v[60:69,17:19,59:68]=2600.
    # Bright metal component far from the nose is not the target.
    v[98:103,84:89,65:70]=4000.
    return v


def test_anatomy_first_finds_nose_and_candidate_without_any_prior_sticker():
    v=mannequin()
    report=mod.anatomy_proposals(v,LPS_IDENTITY_AFFINE,
                                 search_radius_mm=23.)
    assert report["status"]=="ANATOMY_GUIDED_CANDIDATES_REQUIRE_CT_REVIEW"
    assert report["no_prior_sticker_coordinates_used"] is True
    assert report["automatic_sticker_identity"] is False
    assert report["candidate_count"]>=1
    tip=np.asarray(report["anatomy"]["nose_region_landmark_lps_mm"])
    assert np.linalg.norm(tip-[64.,18.,63.])<18.
    candidate=report["candidates"][0]
    assert candidate["status"]=="UNVERIFIED_ANATOMY_GUIDED_HIGH_DENSITY_COMPONENT"
    assert candidate["candidate_id"]=="N01"
    assert np.linalg.norm(np.asarray(candidate["centroid_lps_mm"], dtype=float) - np.array([64., 18., 63.])) < 12.
    assert "distance_from_oct30_prediction_mm" not in candidate
    assert candidate["full_support_inside_reconstructed_box"]
    assert not candidate["touches_search_roi_boundary"]
    assert not candidate["touches_scan_boundary"]


def test_no_sticker_does_not_invent_a_bright_nose_object():
    v=mannequin(with_sticker=False)
    report=mod.anatomy_proposals(v,LPS_IDENTITY_AFFINE)
    assert report["anatomy"]["status"]=="ANATOMY_HEURISTIC_REVIEW_REQUIRED"
    assert report["candidate_count"]==0
    assert report["status"]=="NO_PLAUSIBLE_HIGH_DENSITY_COMPONENT"


def test_empty_scan_abstains_instead_of_guessing():
    v=np.zeros((128,128,128),np.float32)
    out=mod.anatomy_proposals(v,LPS_IDENTITY_AFFINE)
    assert out["status"]=="ABSTAIN_ANATOMY_NOT_RELIABLY_IDENTIFIED"
    assert out["candidate_count"]==0
    assert out["anatomy"]["status"]=="HEAD_SHELL_NOT_FOUND"


def test_anatomical_seed_is_translation_covariant_in_dicom_lps():
    v=mannequin()
    base=mod.anatomy_proposals(v,LPS_IDENTITY_AFFINE)
    aff=LPS_IDENTITY_AFFINE.copy()
    shift=np.array([12.,-25.,6.])
    # LPS_FROM_RAS flips the R and A axes, so RAS offset is (-L,-P,S).
    aff[:3,3]=[-shift[0],-shift[1],shift[2]]
    moved=mod.anatomy_proposals(v,aff)
    a=np.asarray(base["anatomy"]["nose_region_landmark_lps_mm"])
    b=np.asarray(moved["anatomy"]["nose_region_landmark_lps_mm"])
    assert np.allclose(b-a,shift,atol=1e-6)
    a=np.asarray(base["candidates"][0]["centroid_lps_mm"])
    b=np.asarray(moved["candidates"][0]["centroid_lps_mm"])
    assert np.allclose(b-a,shift,atol=1e-6)


def test_review_output_and_retrospective_reference_are_strict(tmp_path):
    v=mannequin()
    output=mod.anatomy_proposals(v,LPS_IDENTITY_AFFINE)
    output["source_detections_sha256"]="source-hash"
    output["source_shape_ijk"]=list(v.shape)
    output["source_volume_sha256"]=mod.volume_digest(v,LPS_IDENTITY_AFFINE)
    p=tmp_path/"anatomy.json"
    mod.write_no_overwrite(p,output)
    assert json.loads(p.read_text())["candidate_count"]>=1
    with pytest.raises(FileExistsError):
        mod.write_no_overwrite(p,output)
    png=tmp_path/"anatomy.png"
    mod.render_anatomy_preview(v,output,png)
    assert png.exists() and png.stat().st_size>0
    ref={"reference_detections_sha256":"source-hash",
         "hdmm_marker_lps_mm":[64.,18.,63.]}
    res=mod.evaluate_after_detection(output,ref,
                                      detection_sha256="source-hash")
    assert res["uses_reference_during_detection"] is False
    assert res["candidates_compared"]
    with pytest.raises(ValueError,match="identical CT detections"):
        mod.evaluate_after_detection(output,ref,detection_sha256="changed")


def test_too_small_or_nonfinite_volume_fails_closed():
    with pytest.raises(ValueError,match="reasonably sized"):
        mod.find_anterior_shell_landmark(np.zeros((5,5,5)),LPS_IDENTITY_AFFINE)
    v=mannequin()
    v[0,0,0]=np.nan
    with pytest.raises(ValueError,match="reasonably sized"):
        mod.anatomy_proposals(v,LPS_IDENTITY_AFFINE)
