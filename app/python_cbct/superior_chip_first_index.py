"""Map chip-first superior CT lattice to the documented JST-down firmware grid.

Research QC only. Physical prior for the Gamma Knife top receiver:
- chips face patient/inferior (in DICOM LPS, face normal has negative S);
- JST connectors extend toward patient's RIGHT (negative L).
This was documented for the earlier mounting. Reconfirm for October 30.
No Hall measurements, magnet fields or post-motion results enter the map.
"""
import numpy as np


def superior_index_from_mounting(detection):
    """Return deterministic firmware->chip-first index proposal, not approval.

    Requires a 4x4 superior candidate and a normal compatible with a
    patient-facing board. Reconstruct chip-facing RIGHT and UP, then order
    four bottom-to-top chips in each of four left-to-right columns.
    """
    record=detection.get("superior_chip_grid_diagnostic") or {}
    if record.get("status")!="strong_chip_grid_candidate":
        raise ValueError("No strong provisional superior 4x4 chip grid")
    points=np.asarray(record.get("chip_grid_lps_mm_not_firmware_order"),float)
    if points.shape!=(16,3) or not np.isfinite(points).all():
        raise ValueError("Expected 16 finite superior CT chip positions")
    unsigned=np.asarray(record.get("estimated_face_normal_lps"),float)
    if unsigned.shape!=(3,) or not np.isfinite(unsigned).all():
        raise ValueError("Expected finite CT superior face-normal candidate")
    if np.linalg.norm(unsigned)<0.9:
        raise ValueError("Degenerate superior plane normal")
    unsigned/=np.linalg.norm(unsigned)
    # CT PCA normal has no intrinsic face sign; physical chips face
    # toward the patient's head (inferior) in the recorded prior.
    face_normal=unsigned if unsigned[2]<0 else -unsigned
    if face_normal[2]>-0.5:
        raise ValueError("Top-board plane inconsistent with inferior-facing chip side")
    jst_lps=np.array([-1.,0.,0.])  # patient's RIGHT in LPS
    toward=jst_lps-(jst_lps@face_normal)*face_normal
    if np.linalg.norm(toward)<0.75:
        raise ValueError("JST direction approximately normal to the CT chip face")
    toward/=np.linalg.norm(toward)
    up=-toward
    right=np.cross(up,face_normal)
    right/=np.linalg.norm(right)
    origin=points.mean(axis=0)
    projected=points-origin
    x=projected@right
    y=projected@up
    order=np.argsort(x,kind="stable")
    columns=[order[i*4:(i+1)*4] for i in range(4)]
    per_firmware=[]
    pitch=[]
    for col in columns:
        local=col[np.argsort(y[col],kind="stable")]
        per_firmware.extend([int(i) for i in local])
        pitch.extend(float(v) for v in np.diff(y[local]))
    pitch.extend(float(v) for v in np.diff([x[col].mean() for col in columns]))
    # Tight spatial grid QC; no field-magnitude ranking.
    if sorted(per_firmware)!=list(range(16)):
        raise ValueError("Incomplete superior mapping")
    if not all(3.5<=v<=5.5 for v in pitch):
        raise ValueError("Superior grid projection not consistent with 4.5-mm pitch")
    # Other axis leakage in a column/row must be small to avoid
    # assigning arbitrary chip permutations to a rotated grid.
    if max(float(np.ptp(x[col])) for col in columns)>1.2:
        raise ValueError("JST prior not aligned with 4x4 grid columns")
    return {
        "format":"superior_chip_first_mounting_prior_v1",
        "status":"PROVISIONAL_PHYSICAL_MOUNTING_PRIOR",
        "firmware_index_to_chip_first_index":per_firmware,
        "face_normal_lps":np.round(face_normal,5).tolist(),
        "up_away_from_jst_lps":np.round(up,5).tolist(),
        "right_when_facing_chips_lps":np.round(right,5).tolist(),
        "min_projected_pitch_mm":round(min(pitch),3),
        "max_projected_pitch_mm":round(max(pitch),3),
        "assumption":"October 30 superior board still chips-to-patient and JST toward patient right",
        "verified":False,
        "warning":"Source prior predates October 30; no independent physical confirmation of October 30 mounting.",
    }


def check_prepost_superior_order(pre, post):
    """Check deterministic index proposals agree in pre/post CT grids.

    This tests ordering consistency, not independently true chip numbering.
    """
    a=superior_index_from_mounting(pre)
    b=superior_index_from_mounting(post)
    if a["firmware_index_to_chip_first_index"]!=b["firmware_index_to_chip_first_index"]:
        raise ValueError("Superior pre/post firmware-to-detector grid ordering changed")
    return {"pre":a,"post":b}


def main():
    import argparse
    import json
    from pathlib import Path
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pre",type=Path,required=True,
                    help="October 30 pre-scan detections.json")
    ap.add_argument("--post",type=Path,required=True,
                    help="October 30 post-scan detections.json")
    ap.add_argument("--output",type=Path,default=Path.home()/"Downloads"/
                    "GK_CBCT_Results"/"superior_chip_first_mounting_proposal.json")
    args=ap.parse_args()
    pre=json.loads(args.pre.read_text(encoding="utf-8"))
    post=json.loads(args.post.read_text(encoding="utf-8"))
    proposal=check_prepost_superior_order(pre,post)
    # Use original accepted left/right receivers as static control.
    from compare_oct30_prepost import _objects, stationary_registration
    fitted=stationary_registration(_objects(pre),_objects(post))["fit"]
    a=np.asarray(pre["superior_chip_grid_diagnostic"]
                  ["chip_grid_lps_mm_not_firmware_order"])
    b=np.asarray(post["superior_chip_grid_diagnostic"]
                  ["chip_grid_lps_mm_not_firmware_order"])
    idx=proposal["pre"]["firmware_index_to_chip_first_index"]
    diffs=np.linalg.norm(a[idx]@fitted["rotation"]+fitted["translation"]-b[idx],
                         axis=1)
    out={
        "format":"gk_superior_chip_first_mounting_audit_v1",
        "status":"PROVISIONAL_PRIOR_NOT_FIRMWARE_VERIFIED",
        "assumed_physical_mounting":"chips toward patient (inferior); JST toward patient right",
        "assumption_from":"Earlier documented GK physical mounting (not independently reconfirmed for Oct30)",
        "firmware_index_to_chip_first_index":idx,
        "prepost_grid_mapping_identical":True,
        "side_receivers_registration_rms_mm":round(float(fitted["rms_mm"]),3),
        "superior_index_matched_rms_mm":round(float(np.sqrt(np.mean(diffs**2))),3),
        "superior_index_matched_max_mm":round(float(np.max(diffs)),3),
        "pre_projected_pitch_range_mm":[
            proposal["pre"]["min_projected_pitch_mm"],
            proposal["pre"]["max_projected_pitch_mm"]],
        "post_projected_pitch_range_mm":[
            proposal["post"]["min_projected_pitch_mm"],
            proposal["post"]["max_projected_pitch_mm"]],
        "all_3_boards_calibrated":False,
        "requires_review":True,
        "remaining":"Confirm Oct30 physical JST direction and chip-face side; verify actual device firmware/hardware; calibrate Hall vector axes, backgrounds, polarities, remanence and Hall-element offsets.",
    }
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(out,indent=2)+"\n",encoding="utf-8")
    print("Mapped superior firmware-to-chip-first indices:",idx)
    print("Pre/post identical:",out["prepost_grid_mapping_identical"])
    print("Side receiver registration RMS (mm):",out["side_receivers_registration_rms_mm"])
    print("Superior 16-chip RMS / max (mm):",out["superior_index_matched_rms_mm"],
          "/",out["superior_index_matched_max_mm"])
    print("Output:",args.output)
    print("WARNING: This is a physical mounting PRIOR, NOT an approved live profile.")


if __name__=="__main__":
    main()
