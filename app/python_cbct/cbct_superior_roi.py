"""Inspect one suspected superior board region without relaxing production detector QC.

The output is numeric JSON/text under Downloads/GK_CBCT_Results/superior_roi.
Only --save-crop saves CT voxel intensities (.npz); never commits DICOM.

Example:
  python cbct_superior_roi.py --scan "$HOME/Downloads/CBCT103026_pre" \
      --center-lps 108.1746 -78.5337 33.9655 --threshold 2290.594
"""
import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from itertools import product
from time import perf_counter

import numpy as np
from scipy import ndimage as ndi


def crop_lps_box(image, center_lps, halfwidth_mm):
    """Return cropped CT, crop-corrected affine and the source voxel bounds."""
    import detector_core as core
    corners=np.asarray([
        core.lps_to_voxel(image.affine, [center_lps[j]+sign[j]*halfwidth_mm
                                          for j in range(3)])
        for sign in product((-1,1),repeat=3)
    ],dtype=float)
    bounds_lo=np.maximum(np.floor(corners.min(axis=0)).astype(int)-2,0)
    bounds_hi=np.minimum(np.ceil(corners.max(axis=0)).astype(int)+3,
                         np.asarray(image.spatial_shape))
    if np.any(bounds_hi<=bounds_lo):
        raise ValueError("ROI lies outside the reconstructed CT field of view")
    selection=tuple(slice(int(a),int(b)) for a,b in zip(bounds_lo,bounds_hi))
    data=core._float_data(image)[selection]
    affine=np.asarray(image.affine,dtype=float).copy()
    affine[:3,3]+=affine[:3,:3]@bounds_lo
    return data,affine,bounds_lo,bounds_hi


def connected_candidates(labels,n,voxel_volume,affine,center_lps,spacing,
                         max_distance_mm=45.0,min_voxels=60):
    """Summarize components closest to the rejected board location."""
    import detector_core as core
    sizes=np.bincount(labels.ravel(),minlength=n+1)
    candidates=[]
    for idx,sl in enumerate(ndi.find_objects(labels),start=1):
        if sl is None or sizes[idx]<min_voxels:continue
        center_index=np.array([(q.start+q.stop-1)*0.5 for q in sl])
        position=core.voxel_to_lps(affine,center_index)
        distance=float(np.linalg.norm(position-center_lps))
        if distance>max_distance_mm:continue
        extent=np.array([q.stop-q.start for q in sl])*spacing
        candidates.append({
            "component":idx,"voxels":int(sizes[idx]),
            "volume_mm3":round(float(sizes[idx]*voxel_volume),1),
            "bbox_extent_mm":[round(float(v),2) for v in sorted(extent,reverse=True)],
            "bbox_center_lps_mm":[round(float(v),2) for v in position],
            "distance_to_prior_candidate_mm":round(distance,2),
        })
    candidates.sort(key=lambda c:(c["distance_to_prior_candidate_mm"],-c["voxels"]))
    return candidates


def attempt_original_board_measurement(image_data, affine, spacing, labels,
                                       component_id, threshold):
    """Diagnostic only: run exactly the original board-shape/grid QC on ROI data."""
    import detector_core as core
    coords=np.argwhere(labels==component_id)
    if len(coords)<100:return {"status":"too_few_voxels"}
    t0=perf_counter()
    try:
        board,reason=core._measure_board(
            image_data,spacing,affine,coords*spacing,
            grid=(4,4),board_size=(15.0,120.0),
            max_thickness=8.0,min_rect=0.85,slab_mm=3.0,
            pitch_mm=(2.0,10.0),min_contrast=None,dense_thr=threshold,
        )
        if board is None:
            return {"status":"rejected","reason":reason,
                    "seconds":round(perf_counter()-t0,3)}
        return {"status":"accepted_candidate_not_validated",
                "grid_rms_mm":round(float(board["grid_rms_mm"]),4),
                "grid_contrast":round(float(board["grid_contrast"]),2),
                "center_lps_mm":[round(float(x),2) for x in board["center_lps"]],
                "seconds":round(perf_counter()-t0,3)}
    except Exception as error:
        return {"status":"measurement_exception",
                "error":f"{type(error).__name__}: {error}",
                "seconds":round(perf_counter()-t0,3)}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scan",type=Path,required=True,help="DICOM folder, NIfTI or zipped DICOM")
    ap.add_argument("--center-lps",type=float,nargs=3,required=True,
                    metavar=("L","P","S"),help="Suspected failed board component center in DICOM LPS mm")
    ap.add_argument("--threshold",type=float,required=True,
                    help="Original board threshold from board_detection_audit.json")
    ap.add_argument("--radius-mm",type=float,default=48,
                    help="Halfwidth of diagnostic cubic ROI (default 48 mm)")
    ap.add_argument("--out",type=Path,help="Custom output directory; by default Downloads/GK_CBCT_Results/superior_roi")
    ap.add_argument("--probe",action="store_true",
                    help="Run slower original board/grid QC on multiple nearby components (diagnostic only)")
    ap.add_argument("--probe-top",type=int,default=3,
                    help="Maximum substantial nearby components to probe per threshold (default 3)")
    ap.add_argument("--save-crop",action="store_true",
                    help="Save cropped CT voxel values for offline analysis (requires de-identification review before sharing)")
    args=ap.parse_args()
    if args.radius_mm<=0 or args.threshold<=0:ap.error("radius and threshold must be positive")
    if args.probe_top<1 or args.probe_top>8:ap.error("--probe-top must be between 1 and 8")
    import detector_core as core
    # ZIP handling is shared with the app's detector; avoid duplicating ZIP extraction logic.
    from cbct_detect import scan_path
    output=args.out or Path.home()/"Downloads"/"GK_CBCT_Results"/"superior_roi"
    output.mkdir(parents=True,exist_ok=True)
    stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    folder=output/("roi_"+stamp)
    folder.mkdir(exist_ok=False)
    start=perf_counter()
    with scan_path(args.scan) as dicom:
        print("Loading scan and cropping superior board ROI...",flush=True)
        image=core.load_scan(dicom)
        center=np.asarray(args.center_lps,dtype=float)
        data,affine,lo,hi=crop_lps_box(image,center,args.radius_mm)
        spacing=np.asarray(image.spacing,dtype=float)
        if args.save_crop:
            np.savez_compressed(folder/"superior_roi_voxels.npz",
                                volume=data,affine_ras=affine,spacing_mm=spacing,
                                nominal_center_lps_mm=center)
    t1=perf_counter()
    smooth=ndi.gaussian_filter(data,sigma=0.5/spacing)
    factor_values=(1.15,1.0,0.9,0.8,0.7,0.6)
    readings=[]
    for factor in factor_values:
        threshold=args.threshold*factor
        labels,n=ndi.label(smooth>threshold)
        candidates=connected_candidates(labels,n,float(np.prod(spacing)),
                                        affine,center,spacing)
        entry={
            "threshold_scale":factor,"threshold":round(threshold,2),
            "components_in_ROI":int(n),
            "nearby_components":candidates[:8],
        }
        if args.probe and candidates:
            # The nearest candidate can be a connector/chip fragment, not
            # the 34 x 9 mm board fragment. Probe several substantial
            # components independently; never infer identity from proximity
            # alone. Do not change any production QC thresholds.
            pool=[c for c in candidates
                  if c["voxels"]>=500 and c["distance_to_prior_candidate_mm"]<=35]
            selected=sorted(pool,key=lambda c:c["distance_to_prior_candidate_mm"])[:args.probe_top]
            entry["board_candidate_probes"]=[]
            for c in selected:
                outcome=attempt_original_board_measurement(
                    data,affine,spacing,labels,c["component"],threshold)
                entry["board_candidate_probes"].append({
                    "component":c["component"],
                    "voxels":c["voxels"],
                    "bbox_extent_mm":c["bbox_extent_mm"],
                    "distance_to_prior_candidate_mm":c["distance_to_prior_candidate_mm"],
                    **outcome,
                })
        readings.append(entry)
        del labels
    report={
        "format":"superior_roi_diagnostic_v2",
        "warning":"Diagnostic only. Multiple distinct nearby ROI components may be probed at each threshold. Proximity alone does not identify the superior PCB. No production geometry is accepted, and magnet exclusion is not reproduced.",
        "center_lps_mm":args.center_lps,"radius_mm":args.radius_mm,
        "baseline_threshold":args.threshold,
        "cropped_shape_voxels":list(data.shape),
        "voxel_spacing_mm":[round(float(v),4) for v in spacing],
        "source_voxel_bounds":{"start":lo.tolist(),"stop":hi.tolist()},
        "timing_seconds":{"read_and_crop":round(t1-start,3),
                          "total":round(perf_counter()-start,3)},
        "threshold_sweep":readings,
    }
    path=folder/"superior_roi_report.json"
    path.write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    print("ROI voxel shape:",tuple(data.shape))
    print("Original threshold:",args.threshold)
    for entry in readings:
        candidates=entry["nearby_components"]
        print(f"  threshold {entry['threshold_scale']:.2f}x: {entry['components_in_ROI']} components;"
              f" {len(candidates)} sizeable components near superior board")
        if candidates:
            a=candidates[0]
            print(f"    closest: {a['voxels']} voxels; bbox extent {a['bbox_extent_mm']} mm;"
                  f" distance {a['distance_to_prior_candidate_mm']:.1f} mm")
            for probe in entry.get("board_candidate_probes",[]):
                print("    candidate",probe["component"],f"({probe['voxels']} voxels):",
                      probe["status"], probe.get("reason",""),
                      f"({probe.get('seconds','?')} s)")
    print("Full numeric diagnostic:",path)
    if args.save_crop:
        print("Cropped CT voxel array (review privacy before sharing):",
              folder/"superior_roi_voxels.npz")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
