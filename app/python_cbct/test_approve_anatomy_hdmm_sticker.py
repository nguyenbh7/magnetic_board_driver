"""Safety regressions for exporting operator-reviewed anatomy-first marker to live fit."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location(
    "approve_anatomy_hdmm_sticker",
    HERE / "approve_anatomy_hdmm_sticker.py",
)
assert spec is not None and spec.loader is not None
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)

AFFINE = np.diag([-1., -1., 1., 1.])
NOTE = "Visually confirmed HDMM reflective sticker at phantom nose tip in original CT slices."


def detections(path):
    objects = [
        {"name":"magnet_left", "kind":"magnet", "center_lps_mm":[15., 15., 15.]},
        {"name":"magnet_right", "kind":"magnet", "center_lps_mm":[75., 15., 15.]},
        {"name":"magnet_superior", "kind":"magnet", "center_lps_mm":[45., 25., 75.]},
    ]
    for name,x in (("array_left",15.),("array_right",75.)):
        objects.append({
            "name":name, "kind":"array",
            "center_lps_mm":[x, 55., 25.],
            "chip_grid_lps_mm":[[x,float(10+i%4),float(40+i//4)]
                                 for i in range(16)],
        })
    report = {
        "format":"gk_cbct_detections_v1",
        "coordinate_frame":"DICOM_LPS_mm",
        "frame_of_reference_uid":"frame-gk-test",
        "objects":objects,
    }
    path.write_text(json.dumps(report),encoding="utf-8")
    return path


def sample(tmp_path, *, pos=(45.,40.,45.)):
    volume = np.zeros((96,96,96),np.float32)
    volume[43:48,38:43,43:48] = 2200.
    det = detections(tmp_path/"detections.json")
    proposal = {
        "format":"gk_hdmm_anatomy_first_candidates_v1",
        "status":"ANATOMY_GUIDED_CANDIDATES_REQUIRE_CT_REVIEW",
        "automatic_sticker_identity":False,
        "no_prior_sticker_coordinates_used":True,
        "landmark_method":"slice-profile-v2",
        "source_detections_sha256":bridge.sha_file(det),
        "source_volume_sha256":bridge.volume_digest(volume,AFFINE),
        "source_shape_ijk":list(volume.shape),
        "source_affine_ras":AFFINE.tolist(),
        "source_frame_of_reference_uid":"frame-gk-test",
        "candidates":[{
            "candidate_id":"N01",
            "status":bridge.CANDIDATE_STATUS,
            "centroid_lps_mm":list(pos),
            "centroid_ijk":list(pos),
            "voxel_count":125,
            "threshold_raw_ct":2000.,
            "touches_scan_boundary":False,
            "touches_search_roi_boundary":False,
            "full_support_inside_reconstructed_box":True,
        }],
    }
    return proposal,det,volume,AFFINE.copy()


def approve(proposal,det,volume,affine):
    return bridge.reviewed_reference_from_anatomy(
        proposal, "N01", det, volume, affine,
        note=NOTE, operator_confirmed=True,
    )


def test_reviewed_reference_is_directly_compatible_with_live_importer_format(tmp_path):
    proposal,det,volume,aff=sample(tmp_path)
    record=approve(proposal,det,volume,aff)
    assert record["format"]=="gk_hdmm_sticker_reference_v1"
    assert record["status"]=="OPERATOR_REVIEWED_REFERENCE_NOT_ALGORITHM_VERIFIED"
    assert record["immutable_reference"] is True
    assert record["coordinate_frame"]=="DICOM_LPS_mm"
    assert record["reference_frame_of_reference_uid"]=="frame-gk-test"
    assert record["hdmm_marker_lps_mm"]==[45.,40.,45.]
    assert record["reference_fov"]["support_fully_in_fov"] is True
    assert record["automatic_sticker_identity"] is False
    assert record["optical_hdmm_center_calibrated"] is False
    assert len(record["reference_magnets_lps_mm"])==3
    json.dumps(record,allow_nan=False)


def test_missing_operator_review_never_promotes_candidate(tmp_path):
    proposal,det,volume,aff=sample(tmp_path)
    with pytest.raises(ValueError,match="Explicit operator"):
        bridge.reviewed_reference_from_anatomy(
            proposal,"N01",det,volume,aff,
            note=NOTE,operator_confirmed=False,
        )
    with pytest.raises(ValueError,match="Document how"):
        bridge.reviewed_reference_from_anatomy(
            proposal,"N01",det,volume,aff,
            note="Looks fine",operator_confirmed=True,
        )
    with pytest.raises(ValueError,match="Candidate ID"):
        bridge.reviewed_reference_from_anatomy(
            proposal,"N02",det,volume,aff,
            note=NOTE,operator_confirmed=True,
        )


def test_mismatched_scan_volume_detector_file_and_affine_are_rejected(tmp_path):
    p,det,volume,aff=sample(tmp_path)
    damaged=volume.copy()
    damaged[10,10,10]=100.
    with pytest.raises(ValueError,match="Source CT volume differs"):
        approve(p,det,damaged,aff)
    wrong_aff=aff.copy()
    wrong_aff[0,3]+=1.
    with pytest.raises(ValueError,match="Source CT volume differs"):
        approve(p,det,volume,wrong_aff)
    det.write_text(det.read_text()+"\n",encoding="utf-8")
    with pytest.raises(ValueError,match="CT detections changed"):
        approve(p,det,volume,aff)


def test_cropped_or_roi_limited_candidate_never_accepted(tmp_path):
    p,det,volume,aff=sample(tmp_path)
    for key in ["touches_scan_boundary","touches_search_roi_boundary"]:
        other=json.loads(json.dumps(p))
        other["candidates"][0][key]=True
        with pytest.raises(ValueError,match="cropped"):
            approve(other,det,volume,aff)
    other=json.loads(json.dumps(p))
    other["candidates"][0]["full_support_inside_reconstructed_box"]=False
    with pytest.raises(ValueError,match="cropped"):
        approve(other,det,volume,aff)
    clipped,_,_,_=sample(tmp_path,pos=(45.,40.,2.))
    with pytest.raises(ValueError,match="cropped or outside"):
        approve(clipped,det,volume,aff)


def test_unverified_or_mutated_component_is_rejected(tmp_path):
    p,det,volume,aff=sample(tmp_path)
    for key,value in [
        ("landmark_method","global-shell-v1"),
        ("no_prior_sticker_coordinates_used",False),
        ("automatic_sticker_identity",True),
    ]:
        other=json.loads(json.dumps(p))
        other[key]=value
        with pytest.raises(ValueError):
            approve(other,det,volume,aff)
    other=json.loads(json.dumps(p))
    other["candidates"][0]["centroid_ijk"]=[46.,40.,45.]
    with pytest.raises(ValueError,match="voxel centroid and physical"):
        approve(other,det,volume,aff)
    other=json.loads(json.dumps(p))
    other["candidates"][0]["threshold_raw_ct"]=1337.
    with pytest.raises(ValueError,match="Unexpected candidate"):
        approve(other,det,volume,aff)


def test_reference_outputs_are_exclusive_not_overwritten(tmp_path):
    p,det,volume,aff=sample(tmp_path)
    record=approve(p,det,volume,aff)
    out=tmp_path/"reviewed.json"
    bridge.write_no_overwrite(out,record)
    assert json.loads(out.read_text())["format"]=="gk_hdmm_sticker_reference_v1"
    with pytest.raises(FileExistsError):
        bridge.write_no_overwrite(out,record)
