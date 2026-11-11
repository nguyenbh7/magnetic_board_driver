"""Synthetic, source-preserving tests: intact sticker, cropped scan and safe transfer."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location(
    "hdmm_sticker_geometry", SCRIPTS / "hdmm_sticker_geometry.py"
)
mod = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(mod)

AFFINE_RAS_LPS_IDENTITY = np.diag([-1., -1., 1., 1.])


def rotation_z(deg):
    theta = np.deg2rad(deg)
    c, s = np.cos(theta), np.sin(theta)
    # Proper row-vector rotation.
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def reports():
    objects = []
    for name, x in (("array_left", 7.), ("array_right", 90.)):
        grid = np.asarray([
            [x, 8. + j * 4.2, 8. + k * 4.2]
            for j in range(4) for k in range(4)
        ])
        objects.append({
            "kind": "array", "name": name,
            "center_lps_mm": grid.mean(0).tolist(),
            "chip_grid_lps_mm": grid.tolist(),
        })
    magnets = [
        ("magnet_left", [22., 30., 30.]),
        ("magnet_right", [70., 32., 33.]),
        ("magnet_superior", [48., 40., 82.]),
    ]
    for name, pt in magnets:
        objects.append({
            "kind": "magnet", "name": name,
            "center_lps_mm": pt,
        })
    pre = {
        "format": "gk_cbct_detections_v1",
        "coordinate_frame": "DICOM_LPS_mm",
        "status": "unreviewed",
        "objects": objects,
    }
    fixed_rotation = rotation_z(4.)
    fixed_translation = np.array([2., -3., 0.7])
    head_rotation = rotation_z(-2.5)
    head_translation = np.array([3.5, 1.3, 0.8])
    post = json.loads(json.dumps(pre))
    for obj in post["objects"]:
        if obj["kind"] == "array":
            points = np.asarray(obj["chip_grid_lps_mm"])
            result = points @ fixed_rotation + fixed_translation
            obj["chip_grid_lps_mm"] = result.tolist()
            obj["center_lps_mm"] = result.mean(0).tolist()
        elif obj["kind"] == "magnet":
            before = np.asarray(obj["center_lps_mm"])
            result = ((before @ fixed_rotation + fixed_translation)
                      @ head_rotation + head_translation)
            obj["center_lps_mm"] = result.tolist()
    return pre, post, fixed_rotation, fixed_translation, head_rotation, head_translation


def reference(tmp_path, point=(47., 70., 65.)):
    pre, post, fixed_R, fixed_t, head_R, head_t = reports()
    source = tmp_path / "original_reference_detections.json"
    source.write_text(json.dumps(pre), encoding="utf-8")
    before = source.read_bytes()
    reviewed = mod.reviewed_reference(
        source, point,
        evidence="CT_VISIBLE_OPERATOR_REVIEWED",
        note="Operator verified optical sticker at central nose surface in original CT.",
        volume_shape=(160, 160, 160),
        affine_ras=AFFINE_RAS_LPS_IDENTITY,
    )
    assert source.read_bytes() == before
    return pre, post, reviewed, source, (fixed_R, fixed_t, head_R, head_t)


def test_fov_distinguishes_intact_partial_and_completely_cropped():
    valid = mod.fov_status(
        (100, 100, 100), AFFINE_RAS_LPS_IDENTITY, [30, 30, 30])
    assert valid["status"] == "CENTER_AND_SUPPORT_INSIDE_FOV"
    assert valid["support_fully_in_fov"]
    edge = mod.fov_status(
        (100, 100, 100), AFFINE_RAS_LPS_IDENTITY, [1, 30, 30])
    assert edge["status"] == "CROPPED_OR_EDGE_LIMITED"
    assert edge["center_in_fov"] and not edge["support_fully_in_fov"]
    missing = mod.fov_status(
        (100, 100, 100), AFFINE_RAS_LPS_IDENTITY, [-4, 30, 30])
    assert missing["status"] == "OUTSIDE_FOV"
    assert not missing["center_in_fov"]
    # Non-uniform spacing must be measured in physical coordinates.
    affine = np.diag([-2., -3., 1., 1.])
    assert np.isclose(mod.fov_status(
        (100, 100, 100), affine, [4, 15, 30]
    )["closest_scan_face_distance_mm"], 4.0)


def test_intact_scan_roi_proposes_bright_and_dark_objects_without_identifying():
    volume = np.zeros((90, 90, 90), np.float32)
    center = np.array([40., 45., 50.])
    ii, jj, kk = np.ogrid[:90, :90, :90]
    volume += 140 * np.exp(-(
        (ii-40.)**2+(jj-45.)**2+(kk-50.)**2
    )/3.)
    volume -= 120 * np.exp(-(
        (ii-52.)**2+(jj-45.)**2+(kk-50.)**2
    )/3.)
    report = mod.candidate_report(
        volume, AFFINE_RAS_LPS_IDENTITY, center, radius_mm=18
    )
    assert report["status"] == "CANDIDATES_ONLY_REQUIRES_PHYSICAL_CT_REVIEW"
    assert not report["automatic_hdmm_identity"]
    assert report["candidate_count"] > 0
    assert report["candidate_count"] == len(report["candidates"])
    assert all(x["identity"] == "UNVERIFIED_CT_STICKER_CANDIDATE"
               for x in report["candidates"])
    assert any(x["signed_contrast"] == "bright" for x in report["candidates"])
    assert any(x["signed_contrast"] == "dark" for x in report["candidates"])


def test_transfer_uses_fixed_arrays_then_moving_magnets_without_optical_fit(tmp_path):
    pre, post, reviewed, path, transforms = reference(tmp_path)
    f_R, f_t, h_R, h_t = transforms
    pred = mod.transfer_reference(
        reviewed, pre, post,
        target_volume_shape=(160, 160, 160),
        target_affine_ras=AFFINE_RAS_LPS_IDENTITY,
    )
    pt = np.asarray(reviewed["hdmm_marker_lps_mm"])
    expected = (pt @ f_R + f_t) @ h_R + h_t
    assert np.allclose(pred["predicted_target_hdmm_marker_lps_mm"], expected, atol=1e-7)
    assert pred["status"] == "RIGID_TRANSFER_ESTIMATE_NOT_DIRECT_CT_DETECTION"
    assert pred["stationary_receiver_registration_rms_mm"] < 1e-6
    assert pred["head_magnet_rigid_fit_rms_mm"] < 1e-6
    assert pred["target_fov"]["status"] == "CENTER_AND_SUPPORT_INSIDE_FOV"
    assert not pred["marker_was_directly_identified_in_target"]
    assert pred["uncertainty_not_equal_to_fit_rms"]
    assert "Hall" not in pred["geometric_model"]
    assert path.exists()


def test_marker_absent_from_target_still_predictable_from_approved_reference(tmp_path):
    pre, post, reviewed, _, _ = reference(tmp_path)
    cropped = mod.transfer_reference(
        reviewed, pre, post,
        target_volume_shape=(40, 40, 40),
        target_affine_ras=AFFINE_RAS_LPS_IDENTITY,
    )
    assert cropped["target_fov"]["status"] == "OUTSIDE_FOV"
    assert not cropped["marker_was_directly_identified_in_target"]
    assert np.isfinite(cropped["predicted_target_hdmm_marker_lps_mm"]).all()
    unknown = mod.transfer_reference(reviewed, pre, post)
    assert unknown["target_fov"]["status"] == "TARGET_FOV_UNCHECKED"


def test_unreviewed_guess_cannot_be_approved_as_intact_sticker(tmp_path):
    pre, _, _, source, _ = reference(tmp_path)
    with pytest.raises(ValueError, match="cropped or outside"):
        mod.reviewed_reference(
            source, [1., 25., 28.],
            evidence="CT_VISIBLE_OPERATOR_REVIEWED",
            note="Operator saw reflective sticker at the cut-off edge",
            volume_shape=(160, 160, 160),
            affine_ras=AFFINE_RAS_LPS_IDENTITY,
        )
    with pytest.raises(ValueError, match="requires checking"):
        mod.reviewed_reference(
            source, [40., 45., 65.],
            evidence="CT_VISIBLE_OPERATOR_REVIEWED",
            note="The actual optical sticker was visually confirmed",
        )
    with pytest.raises(ValueError, match="identification note"):
        mod.reviewed_reference(
            source, [40., 45., 65.],
            evidence="INDEPENDENT_PHYSICAL_RIGID_OFFSET",
            note="guess",
        )
    with pytest.raises(ValueError, match="required reviewed"):
        mod.transfer_reference({"format": "candidate"},
                               pre, pre)


def test_distorted_magnet_geometry_and_board_registration_fail_closed(tmp_path):
    pre, post, reviewed, _, _ = reference(tmp_path)
    changed = json.loads(json.dumps(post))
    for obj in changed["objects"]:
        if obj["name"] == "magnet_superior":
            obj["center_lps_mm"][2] += 6.
    with pytest.raises(ValueError, match="constellation changed"):
        mod.transfer_reference(reviewed, pre, changed)
    bad_boards = json.loads(json.dumps(post))
    for obj in bad_boards["objects"]:
        if obj["name"] == "array_right":
            obj["chip_grid_lps_mm"] = (
                np.asarray(obj["chip_grid_lps_mm"]) +
                np.array([0., 0., 6.])
            ).tolist()
    with pytest.raises(ValueError, match="fixed-board CT registration RMS"):
        mod.transfer_reference(reviewed, pre, bad_boards)


def test_reference_magnet_provenance_and_no_file_overwrite(tmp_path):
    pre, post, reviewed, source, _ = reference(tmp_path)
    target = tmp_path / "approved_marker.json"
    mod.write_no_overwrite(target, reviewed)
    before = target.read_bytes()
    with pytest.raises(FileExistsError):
        mod.write_no_overwrite(target, reviewed)
    assert before == target.read_bytes()
    tampered = json.loads(json.dumps(pre))
    for obj in tampered["objects"]:
        if obj["name"] == "magnet_left":
            obj["center_lps_mm"][0] += 0.2
    with pytest.raises(ValueError, match="reference detector magnets changed"):
        mod.transfer_reference(reviewed, tampered, post)
    assert source.exists()
