"""Generic quality-gated 4x4 chip-first CBCT board recovery.

For any ONE missing board, use a rejected board-like CT component as a
plane-search seed, then validate 16 real bright CT-chip centroids against
a 4x4 lattice. No hardcoded 34x9-mm board footprint or Oct30 coordinates.

Physical board orientation is separate from image-grid detection. A missing
PCB outline means no independently measured PCB centroid/JST direction.
This only yields initial CT *geometry for human review*, never a calibrated
Hall or approved magnetic tracking profile.
"""
import numpy as np
from scipy import ndimage as ndi


BOARD_NAMES=("array_left","array_right","array_superior")
MAGNET_NAMES=("magnet_left","magnet_right","magnet_superior")


def recover_missing_array(report, crop_data, crop_affine_ras, target_board):
    """Return one missing board's CT chip geometry, or fail closed.

    Accepts an arbitrary missing board site and a CT chip-lattice candidate.
    The 4x4 node QC is the SAME regardless of which board is missing.
    """
    if target_board not in BOARD_NAMES:
        raise ValueError("Unknown target board name")

    if report.get("format") != "gk_cbct_detections_v1":
        raise ValueError("Expected normal CBCT detection format")
    if report.get("coordinate_frame") != "DICOM_LPS_mm":
        raise ValueError("Expected DICOM LPS coordinate frame")
    objects = report.get("objects", [])
    magnets = {x.get("name"): x for x in objects if x.get("kind") == "magnet"}
    arrays = {x.get("name"): x for x in objects if x.get("kind") == "array"}
    if set(magnets) != set(MAGNET_NAMES):
        raise ValueError("Must have all three uniquely labeled magnets")
    if set(arrays) != set(BOARD_NAMES)-{target_board}:
        raise ValueError("Only rescue one truly missing board with two intact receivers")
    if sum(x.get("kind") == "magnet" for x in objects) != 3 or sum(
        x.get("kind") == "array" for x in objects) != 2:
        raise ValueError("Unexpected or duplicate magnet/array records")
    diagnostic = report.get("chip_grid_diagnostic") or report.get(
        "superior_chip_grid_diagnostic") or {}
    if diagnostic.get("status") != "strong_chip_grid_candidate":
        raise ValueError("Chip lattice not strong")
    if diagnostic.get("source") != f"rejected_{target_board.removeprefix('array_')}_candidate_roi":
        raise ValueError("Chip lattice was not obtained from missing-board ROI")
    if diagnostic.get("nodes_above_board_threshold") != 16:
        raise ValueError("Expected all 16 high-contrast CT chip nodes")
    if float(diagnostic.get("node_minus_interchip_contrast",0)) < 1500:
        raise ValueError("Insufficient CT chip/interchip contrast")
    if not 4.25 <= float(diagnostic.get("pitch_mm",0)) <= 4.75:
        raise ValueError("Unexpected 4x4 pitch")
    # Do not use the firmware/JST mapping as a prerequisite for finding
    # a 4x4 grid. Geometry can be detected without knowing the connector.
    template = np.asarray(diagnostic.get("chip_grid_lps_mm_not_firmware_order"),float)
    if template.shape != (16,3) or not np.isfinite(template).all():
        raise ValueError("Missing finite 16-chip prior")
    volume = np.asarray(crop_data,dtype=np.float32)
    if volume.ndim != 3 or not np.isfinite(volume).all():
        raise ValueError("Missing finite CT crop")
    affine = np.diag([-1.,-1.,1.,1.]) @ np.asarray(crop_affine_ras,float)
    if affine.shape != (4,4) or not np.isfinite(affine).all():
        raise ValueError("Invalid DICOM-LPS image affine")
    inverse = np.linalg.inv(affine)
    # The chip-first row-major order has +u along index 0->4,
    # +v along index 0->1. Refine actual chip centroids in a small
    # physical patch (<half the pitch) around each predicted node.
    u = template[4]-template[0]
    u /= np.linalg.norm(u)
    v = template[1]-template[0]
    v -= u*np.dot(u,v)
    v /= np.linalg.norm(v)
    grid = np.arange(-2.0,2.0001,0.25)
    du,dv = np.meshgrid(grid,grid,indexing="ij")
    radial2 = du**2+dv**2
    taper = np.exp(-radial2/(2*1.4**2))
    refined=[]
    offsets=[]
    for center in template:
        world=center+du[...,None]*u+dv[...,None]*v
        coords=world@inverse[:3,:3].T+inverse[:3,3]
        values=ndi.map_coordinates(volume,coords.reshape(-1,3).T,
                                   order=1,mode="constant",cval=-1000).reshape(du.shape)
        if float(values.max()) < float(report.get("board_detection_audit",{})
                                           .get("board_threshold",2000)):
            raise ValueError("Refinement patch lacks a dense CT chip")
        weights=np.maximum(values-np.percentile(values,25),0)*taper
        if float(weights.sum())<1e-6:
            raise ValueError("Chip intensity centroid could not be measured")
        offset=np.asarray([(du*weights).sum(),(dv*weights).sum()])/weights.sum()
        if np.linalg.norm(offset)>0.8:
            raise ValueError("Refined chip displaced >0.8 mm from recovered lattice")
        refined.append(center+offset[0]*u+offset[1]*v)
        offsets.append(float(np.linalg.norm(offset)))
    measured=np.asarray(refined)
    # IN-PLANE regularity only; normal/depth is inherited from the
    # detected plate plane and must not be represented as independent
    # 3D chip localization accuracy.
    ab=np.array([(i,j) for i in range(4) for j in range(4)],float)
    model=np.c_[ab,np.ones(16)]
    fit=np.linalg.lstsq(model,measured,rcond=None)[0]
    predicted=model@fit
    grid_rms=float(np.sqrt(np.mean(np.sum((measured-predicted)**2,axis=1))))
    pitch_u,pitch_v=[float(np.linalg.norm(x)) for x in fit[:2]]
    if grid_rms > 0.5 or not all(4.25<=p<=4.75 for p in (pitch_u,pitch_v)):
        raise ValueError(f"Measured chip lattice fails QC: RMS {grid_rms:.3f}, pitch {pitch_u:.3f}/{pitch_v:.3f} mm")
    center=measured.mean(axis=0)
    anchor_name="magnet_"+target_board.removeprefix("array_")
    anchor=np.asarray(magnets[anchor_name]["center_lps_mm"],float)
    distance=float(np.linalg.norm(center-anchor))
    competitors=[
        float(np.linalg.norm(center-np.asarray(magnets[k]["center_lps_mm"],float)))
        for k in MAGNET_NAMES if k != anchor_name
    ]
    if not 10<=distance<=50 or min(competitors)-distance<25:
        raise ValueError("Grid candidate cannot be confidently assigned to its magnet anchor")
    for k in arrays:
        if np.linalg.norm(center-np.asarray(arrays[k]["center_lps_mm"],float)) < 25:
            raise ValueError("Candidate overlaps an already accepted board")
    # The CT PCA plane normal is unsigned. Use documented chip-facing
    # side for the three-board GK setup, but record it as a MOUNTING
    # PRIOR, not an image-derived direction or verified Hall transform.
    raw_normal=np.asarray(diagnostic.get("estimated_face_normal_lps"),float)
    if raw_normal.shape!=(3,) or not np.isfinite(raw_normal).all():
        raise ValueError("Invalid planar CT normal")
    norm=np.linalg.norm(raw_normal)
    if not .9<=norm<=1.1:
        raise ValueError("Unexpected CT plane normal length")
    raw_normal=raw_normal/norm
    axis={"array_superior":(2,-1.0),"array_left":(0,-1.0),
          "array_right":(0,1.0)}[target_board]
    which,desired=axis
    if abs(raw_normal[which])<0.5:
        raise ValueError("Chip plane incompatible with GK board-facing mounting")
    normal=(raw_normal if raw_normal[which]*desired>0 else -raw_normal).tolist()
    # Independently validated chip-first/firmware reindexing is available
    # only for the physically confirmed superior JST mounting.
    mapping=None
    if target_board=="array_superior":
        from superior_chip_first_index import superior_index_from_mounting
        mapping=superior_index_from_mounting({
            **report,"superior_chip_grid_diagnostic":diagnostic})
    # KiCad footprint plus mounted JST direction predicts the missing
    # PCB outline and three connectors. None is labeled CT-observed.
    # An incompatible CAD/mounting prior must NOT make the image-based
    # 16-chip detection fail, but it must fail closed for CAD prediction.
    try:
        from pcb_cad_geometry import predict_pcb_from_ct_chips
        cad=predict_pcb_from_ct_chips(measured,normal,target_board)
    except ValueError as error:
        cad={"status":"CAD_PREDICTION_UNAVAILABLE",
             "reason":str(error),"connectors_found_in_ct":False}
    projected_center=cad.get("pcb_center_projected_to_chip_plane_lps_mm")
    return {
        "kind":"array","name":target_board,
        "pcb_center_cad_estimate_lps_mm":projected_center,
        "pcb_cad_geometry":cad,
        "center_lps_mm":[float(x) for x in center],
        "chip_grid_lps_mm":measured.tolist(),
        "grid_shape":[4,4],
        "grid_pitch_mm":[pitch_u,pitch_v],
        "grid_rms_mm":grid_rms,
        "board_center_lps_mm":None,
        "board_normal_lps":normal,
        "detection_method":"chip_first_generic_board_fallback",
        "review_required":True,
        "firmware_index_to_detected_index_provisional":(
            mapping["firmware_index_to_chip_first_index"] if mapping else None),
        "quality":{
            "chip_centers_from":"original_CT_intensity_weighted_local_refinement",
            "grid_rms_method":"inplane_refined_chip_centers_vs_best_affine_lattice",
            "grid_rms_is_not_absolute_3d_accuracy":True,
            "max_template_to_refined_chip_shift_mm":max(offsets),
            "contrast":float(diagnostic["node_minus_interchip_contrast"]),
            "bright_nodes":16,
            "assigned_magnet_distance_mm":distance,
            "pcb_centroid_unmeasured":True,
            "chip_depth_from_provisional_plate_plane":True,
            "chip_face_orientation_from_GK_mounting_prior":True,
            "firmware_indexing_independently_established":False,
        },
    }




def recover_superior_array(report, crop_data, crop_affine_ras):
    """Backward-compatible thin wrapper around the generic chip-grid QC."""
    return recover_missing_array(
        report,crop_data,crop_affine_ras,"array_superior")

def update_detection_tables(output_folder, rescued):
    """Keep standard CSV exports consistent with a rescued JSON board.

    The PCB outline, PCB centroid and dense connected component are not
    reconstructed: their CSV columns remain empty instead of fabricated.
    """
    import csv
    from pathlib import Path
    folder=Path(output_folder)
    objects_path=folder/"objects.csv"
    sensors_path=folder/"sensors.csv"
    if not objects_path.exists() or not sensors_path.exists():
        raise ValueError("Original detector output tables missing")
    with objects_path.open(newline="",encoding="utf-8") as handle:
        reader=csv.DictReader(handle)
        fields=list(reader.fieldnames or [])
        rows=list(reader)
    target=rescued["name"]
    if target not in BOARD_NAMES:
        raise ValueError("Not a known 4x4 board")
    if not fields or any(r.get("name")==target for r in rows):
        raise ValueError("Original object export invalid or already contains target board")
    for key in ("detection_method","chip_only_geometry"):
        if key not in fields:
            fields.append(key)
    center=rescued["center_lps_mm"]
    normal=rescued["board_normal_lps"]
    pitch=rescued["grid_pitch_mm"]
    data={
        "name":target,"kind":"array",
        "location":target.removeprefix("array_").upper()+" (chip-first)",
        "x_mm":center[0],"y_mm":center[1],"z_mm":center[2],
        "normal_x":normal[0],"normal_y":normal[1],"normal_z":normal[2],
        "n_sensors":16,"grid":"4x4",
        "pitch_1_mm":pitch[0],"pitch_2_mm":pitch[1],
        "grid_rms_mm":rescued["grid_rms_mm"],
        "grid_contrast":rescued["quality"]["contrast"],
        "detection_method":rescued["detection_method"],
        "chip_only_geometry":"true",
    }
    if any(key not in fields for key in data):
        raise ValueError("Original objects.csv lacks required fields")
    rows.append(data)
    # Validate before altering either original CSV file.
    with sensors_path.open(newline="",encoding="utf-8") as handle:
        sensor_rows=list(csv.reader(handle))
    if not sensor_rows or sensor_rows[0][:3]!=["array","row","col"]:
        raise ValueError("Original sensors.csv has unexpected format")
    if any(row and row[0]==target for row in sensor_rows[1:]):
        raise ValueError("Rescued chip CSV rows already exist")
    temp_objects=objects_path.with_suffix(".pending")
    temp_sensors=sensors_path.with_suffix(".pending")
    try:
        with temp_objects.open("w",newline="",encoding="utf-8") as handle:
            writer=csv.DictWriter(handle,fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        with temp_sensors.open("w",newline="",encoding="utf-8") as handle:
            writer=csv.writer(handle)
            writer.writerows(sensor_rows)
            for index,point in enumerate(rescued["chip_grid_lps_mm"]):
                writer.writerow([target,index//4+1,index%4+1,
                                 *[round(float(x),3) for x in point]])
        temp_objects.replace(objects_path)
        temp_sensors.replace(sensors_path)
    finally:
        temp_objects.unlink(missing_ok=True)
        temp_sensors.unlink(missing_ok=True)
