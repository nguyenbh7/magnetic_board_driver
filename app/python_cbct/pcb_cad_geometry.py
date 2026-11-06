"""Nominal PCB and JST locations inferred from measured CT chip grid and KiCad.

Unlike the chip-centroid detection, this is a CAD-derived *prediction*.
It does NOT detect PCB material, connectors, or dangling wires in CT.
The board-outline midpoint is 14.875 mm toward JST from the mean of 16
sensor centers for the lab's MLX90393_board4 (internal file board3).

Source: magnetic_board_driver/hardware/kicad/MLX90393_board4/
        MLX90393_board3.kicad_pcb, Edge.Cuts and J1/J2/J3 footprints.
Native KiCad PCB is 43.400 x 60.450 mm, rectangle X=127.644766..171.044766
and Y=80.096291..140.546291; sensor grid center (149.344766,125.196291).
The chip-facing view is mirrored in X relative to raw KiCad XY.
"""
import numpy as np

BOARD_NAMES=("array_superior","array_left","array_right")
# Measurements from this PCB revision, in millimetres.
PCB_WIDTH_MM=43.400
PCB_HEIGHT_MM=60.450
SENSOR_ARRAY_CENTER_Y_KICAD_MM=125.196291
BOARD_CENTER_Y_KICAD_MM=(80.096291+140.546291)/2
SENSOR_ARRAY_CENTER_X_KICAD_MM=149.344766
JST_CENTERS_KICAD_MM=(
    ("J1",140.144766,82.921291),
    ("J2",149.235581,82.910101),
    ("J3",158.144766,82.896291),
)

# Explicit physical GK mounting: *toward* the JST connectors, in LPS.
# This supplies information that a symmetric 4x4 chip pattern does NOT.
JST_PRIOR_LPS={
    "array_superior":np.array([-1.,0.,0.]),  # patient's right
    "array_left":np.array([0.,0.,-1.]),      # inferior
    "array_right":np.array([0.,0.,-1.]),     # inferior
}
EXPECTED_FACE_AXIS={
    "array_superior":(2,-1),
    "array_left":(0,-1),
    "array_right":(0,1),
}


def predict_pcb_from_ct_chips(chip_centers_lps_mm, chip_face_normal_lps,
                              board_name):
    """Predict PCB plane center, corners and three JST footprint centers.

    Returns CAD-informed positions only. The PCB 3D thickness/true physical
    centroid and any wire endpoints remain unmeasured. This method MUST NOT
    set board_center_lps_mm or imply independent CT confirmation of connectors.
    """
    if board_name not in BOARD_NAMES:
        raise ValueError("Expected one known GK 4x4 receiver board")
    points=np.asarray(chip_centers_lps_mm,dtype=float)
    normal=np.asarray(chip_face_normal_lps,dtype=float)
    if points.shape!=(16,3) or not np.isfinite(points).all():
        raise ValueError("Expected 16 finite CT chip-center positions")
    if normal.shape!=(3,) or not np.isfinite(normal).all():
        raise ValueError("Expected finite chip-face normal")
    magnitude=float(np.linalg.norm(normal))
    if not 0.9<=magnitude<=1.1:
        raise ValueError("CT face normal is not unit length")
    normal=normal/magnitude
    axis,sign=EXPECTED_FACE_AXIS[board_name]
    if normal[axis]*sign<0.5:
        raise ValueError(f"{board_name} face normal contradicts documented mounting")
    toward=JST_PRIOR_LPS[board_name].copy()
    toward-=float(toward@normal)*normal
    if np.linalg.norm(toward)<0.7:
        raise ValueError("JST direction nearly perpendicular to board plane")
    toward/=np.linalg.norm(toward)
    up=-toward
    right=np.cross(up,normal)
    right/=np.linalg.norm(right)
    centroid=points.mean(axis=0)
    rel=points-centroid
    # Check both expected pitch AND alignment with the board's JST view.
    # Without this check, a correct 4x4 grid could be oriented diagonally
    # and a centered CAD outline would be fitted at an incorrect angle.
    xp=rel@right
    yp=rel@up
    sorted_x=np.sort(xp)
    sorted_y=np.sort(yp)
    # Each of the four projected columns/rows has four chips.
    xc=np.array([np.mean(sorted_x[i*4:(i+1)*4]) for i in range(4)])
    yc=np.array([np.mean(sorted_y[i*4:(i+1)*4]) for i in range(4)])
    rms_x=float(np.sqrt(np.mean(np.square(sorted_x-np.repeat(xc,4)))))
    rms_y=float(np.sqrt(np.mean(np.square(sorted_y-np.repeat(yc,4)))))
    if (rms_x>0.7 or rms_y>0.7 or
        not np.all((np.diff(xc)>4.1)&(np.diff(xc)<4.9)) or
        not np.all((np.diff(yc)>4.1)&(np.diff(yc)<4.9))):
        raise ValueError("CT chip grid orientation does not match confirmed JST direction")

    center=centroid+(BOARD_CENTER_Y_KICAD_MM-
                     SENSOR_ARRAY_CENTER_Y_KICAD_MM)*up
    # In KiCad coordinates +X is LEFT when physically viewed chip-face/JST-down.
    connectors={
        name:centroid-(x-SENSOR_ARRAY_CENTER_X_KICAD_MM)*right+
             (y-SENSOR_ARRAY_CENTER_Y_KICAD_MM)*up
        for name,x,y in JST_CENTERS_KICAD_MM
    }
    corners=[
        centroid+side_x*PCB_WIDTH_MM/2*right+offset_y*up
        for offset_y in (80.096291-SENSOR_ARRAY_CENTER_Y_KICAD_MM,
                         140.546291-SENSOR_ARRAY_CENTER_Y_KICAD_MM)
        for side_x in (-1,1)
    ]
    def xyz(p):
        return [round(float(v),4) for v in p]
    return {
        "status":"CAD_PREDICTION_NOT_CT_MEASUREMENT",
        "source":"MLX90393_board4/MLX90393_board3.kicad_pcb Edge.Cuts + J1-J3; known GK JST/chip-face mounting",
        "coordinate_frame":"DICOM_LPS_mm",
        "nominal_pcb_dimensions_mm":[PCB_WIDTH_MM,PCB_HEIGHT_MM],
        "pcb_center_projected_to_chip_plane_lps_mm":xyz(center),
        "board_center_offset_toward_jst_mm":round(
            SENSOR_ARRAY_CENTER_Y_KICAD_MM-BOARD_CENTER_Y_KICAD_MM,4),
        "jst_connector_footprint_centers_lps_mm":{
            name:xyz(p) for name,p in connectors.items()
        },
        "pcb_outline_corners_projected_to_chip_plane_lps_mm":[xyz(p) for p in corners],
        "chip_face_normal_from_ct_and_mounting_lps":xyz(normal),
        "jst_toward_from_documented_mounting_lps":xyz(toward),
        "plane_projection_only":True,
        "connectors_found_in_ct":False,
        "pcb_outline_found_in_ct":False,
        "requires_physical_board_revision_review":True,
        "warning":"CAD and known JST direction predict where the PCB/connectors should be; they were NOT independently segmented in the DICOM. Do not use as a verified Hall calibration.",
    }



def add_cad_estimates_to_detection(report):
    """Annotate already processed CT detections, preserving original data.

    Only chip-first rescued objects whose PCB center was NOT CT-observed
    receive nominal CAD predictions. Existing measured PCB centers are
    preserved as-is. The result remains unreviewed.
    """
    import copy
    report=copy.deepcopy(report)
    if (report.get("format")!="gk_cbct_detections_v1" or
        report.get("coordinate_frame")!="DICOM_LPS_mm"):
        raise ValueError("Expected existing DICOM-LPS CBCT detections")
    for board in report.get("objects",[]):
        if (board.get("kind")!="array"
            or board.get("name") not in BOARD_NAMES
            or not str(board.get("detection_method","")).startswith("chip_first")
            or board.get("board_center_lps_mm") is not None):
            continue
        try:
            cad=predict_pcb_from_ct_chips(
                board["chip_grid_lps_mm"],board["board_normal_lps"],
                board["name"])
        except (ValueError,KeyError) as e:
            cad={"status":"CAD_PREDICTION_UNAVAILABLE","reason":str(e),
                 "connectors_found_in_ct":False}
        board["pcb_center_cad_estimate_lps_mm"]=cad.get(
            "pcb_center_projected_to_chip_plane_lps_mm")
        board["pcb_cad_geometry"]=cad
    report["requires_review"]=True
    return report


def main():
    import argparse
    import json
    from pathlib import Path
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--detections",type=Path,required=True,
                   help="Existing complete scan's detections.json")
    p.add_argument("--output",type=Path,required=True,
                   help="New JSON output; original file is never overwritten")
    a=p.parse_args()
    if a.detections.resolve()==a.output.resolve():
        p.error("Choose a different output path; source must not be overwritten")
    report=add_cad_estimates_to_detection(
        json.loads(a.detections.read_text(encoding="utf-8")))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    for board in report.get("objects",[]):
        if board.get("pcb_cad_geometry"):
            print(board["name"],"CAD center estimate:",
                  board.get("pcb_center_cad_estimate_lps_mm"),
                  "status:",board["pcb_cad_geometry"]["status"])
    print("Saved locally:",a.output)
    print("WARNING: CAD predictions are not measured CT connectors/PCB centers.")


if __name__=="__main__":
    main()
