"""Chip-first 4x4 sensor-grid recovery from a cropped CBCT (diagnostic only).

This does not infer JST position, firmware sensor indexing, polarity, magnetic
axes, or a usable rigid fit profile. It searches one candidate board region
without changing the production CBCT detector's acceptance criteria.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi
from scipy.signal import fftconvolve


def _grid_kernel(pitch_mm, angle_deg, step_mm=0.5):
    angle=np.deg2rad(angle_deg)
    u=np.array([np.cos(angle),np.sin(angle)])
    v=np.array([-np.sin(angle),np.cos(angle)])
    k=np.zeros((49,49),float)
    def add(position,weight):
        pixel=np.asarray(position)/step_mm+24
        lo=np.floor(pixel).astype(int)
        frac=pixel-lo
        for i in (0,1):
            for j in (0,1):
                k[lo[0]+i,lo[1]+j]+=weight*(
                    frac[0] if i else 1-frac[0])*(
                    frac[1] if j else 1-frac[1])
    for a in np.arange(4)-1.5:
        for b in np.arange(4)-1.5:
            add((a*u+b*v)*pitch_mm,1/16)
    for a in np.arange(3)-1:
        for b in np.arange(4)-1.5:
            add((a*u+b*v)*pitch_mm,-1/24)
    for a in np.arange(4)-1.5:
        for b in np.arange(3)-1:
            add((a*u+b*v)*pitch_mm,-1/24)
    return k


def recover_grid(volume, affine_ras, prior_lps, board_threshold=2290.594):
    """Locate a candidate plane and a strong 4x4 periodic bright-chip pattern.

    The prior is a suspected board-piece center from the *existing detector*.
    Reports hypothesis/quality metrics only; does not assign firmware indices.
    """
    volume=np.asarray(volume,dtype=np.float32)
    if volume.ndim!=3:raise ValueError("Expected 3D CT voxels")
    ras_to_lps=np.diag([-1.,-1.,1.,1.])
    affine=ras_to_lps@np.asarray(affine_ras,float)
    inverse=np.linalg.inv(affine)
    prior=np.asarray(prior_lps,float)
    voxel_mm=np.linalg.norm(affine[:3,:3],axis=0)
    blurred=ndi.gaussian_filter(volume,sigma=0.5/voxel_mm)
    labels,n=ndi.label(blurred>board_threshold)
    sizes=np.bincount(labels.ravel(),minlength=n+1)

    # Candidate near the rejected board-shaped region. A large, thin
    # component provides an orientation prior, not an accepted board outline.
    planes=[]
    for index in range(1,n+1):
        if sizes[index]<1500:continue
        vox=np.argwhere(labels==index)
        pts=vox@affine[:3,:3].T+affine[:3,3]
        center=pts.mean(axis=0)
        distance=float(np.linalg.norm(center-prior))
        if distance>20:continue
        eigenvalues,eigenvectors=np.linalg.eigh(np.cov(pts.T))
        if eigenvalues[0]>10 or eigenvalues[-1]<20:continue
        planes.append((distance,index,center,eigenvalues,eigenvectors))
    if not planes:
        return {"status":"no_provisional_plane","prior_lps_mm":prior.tolist(),
                "reason":"No large thin connected component near the rejected candidate"}
    distance,index,plane_center,eigenvalues,V=min(planes,key=lambda x:x[0])
    normal=V[:,0]
    tangent=np.array([1.,0.,0.])
    tangent-=normal*np.dot(tangent,normal)
    if np.linalg.norm(tangent)<0.2:
        tangent=V[:,2]
    tangent/=np.linalg.norm(tangent)
    tangent2=np.cross(normal,tangent)
    # Sample the original unsmoothed CT across a 54-mm face-on area and
    # 7-mm depth. The search is confined to this single ROI.
    step=0.5
    axes=np.arange(-27,27.01,step)
    U,W=np.meshgrid(axes,axes,indexing="ij")
    depth=np.arange(-4,3.01,step)
    slices=[]
    for t in depth:
        world=(plane_center+U[...,None]*tangent
               +W[...,None]*tangent2+t*normal)
        coords=world@inverse[:3,:3].T+inverse[:3,3]
        sl=ndi.map_coordinates(volume,coords.reshape(-1,3).T,
                               order=1,cval=-1000).reshape(U.shape)
        slices.append(ndi.gaussian_filter(sl,1.0))
    scored=[]
    for pitch in (4.25,4.5,4.75):
        for angle in range(-25,26,5):
            kernel=_grid_kernel(pitch,angle,step)
            for ti,img in enumerate(slices):
                score=fftconvolve(img,kernel,mode="same")
                # No boundary-padding hits: all 16 nodes and the grid
                # midpoints must be inside the face-on reslice.
                score[:18,:]=-np.inf
                score[-18:,:]=-np.inf
                score[:,:18]=-np.inf
                score[:,-18:]=-np.inf
                ij=np.unravel_index(int(np.argmax(score)),score.shape)
                scored.append((float(score[ij]),pitch,angle,ti,ij))
    value,pitch,angle,ti,ij=max(scored,key=lambda x:x[0])
    # Interpolate intensity at each proposed node and each inter-chip
    # midpoint. This explicitly checks the entire 16-chip grid rather
    # than accepting a bright rectangular artifact.
    grid_angle=np.deg2rad(angle)
    u2=np.array([np.cos(grid_angle),np.sin(grid_angle)])
    v2=np.array([-np.sin(grid_angle),np.cos(grid_angle)])
    origin=np.asarray(ij,float)
    image=slices[ti]
    def sample(a,b):
        px=origin+(a*u2+b*v2)*pitch/step
        return float(ndi.map_coordinates(image,[[px[0]],[px[1]]],
                                         order=1,mode="constant",cval=-1000)[0])
    nodes=[sample(a,b) for a in np.arange(4)-1.5
           for b in np.arange(4)-1.5]
    midpoint=[sample(a,b) for a in np.arange(3)-1
              for b in np.arange(4)-1.5]
    midpoint += [sample(a,b) for a in np.arange(4)-1.5
                 for b in np.arange(3)-1]
    grid_center=(plane_center+axes[ij[0]]*tangent+
                 axes[ij[1]]*tangent2+depth[ti]*normal)
    # Pixel-grid coordinate order is *not* firmware sensor order.
    points=[]
    for a in np.arange(4)-1.5:
        for b in np.arange(4)-1.5:
            uv=np.array([a,b])
            uv_rot=(u2*uv[0]+v2*uv[1])*pitch
            position=grid_center+uv_rot[0]*tangent+uv_rot[1]*tangent2
            points.append([round(float(v),3) for v in position])
    contrast=float(np.mean(nodes)-np.mean(midpoint))
    strong=(
        contrast>1200
        and min(nodes)>board_threshold
        and 4.25<=pitch<=4.75
    )
    return {
        "status":"strong_chip_grid_candidate" if strong else "weak_or_ambiguous_grid",
        "warning":"Diagnostic ONLY. CT chip coordinates are provisional; JST side, firmware indices and magnetic calibration are not established.",
        "reference_component":int(index),
        "reference_component_voxels":int(sizes[index]),
        "reference_component_pca_eigenvalues_mm2":[round(float(x),3) for x in eigenvalues],
        "reference_component_distance_to_prior_mm":round(distance,3),
        "chip_grid_center_lps_mm":[round(float(x),3) for x in grid_center],
        "estimated_face_normal_lps":[round(float(x),5) for x in normal],
        "pitch_mm":float(pitch),"in_plane_angle_deg":int(angle),
        "node_minus_interchip_score":round(float(value),1),
        "node_intensity_min":round(float(min(nodes)),1),
        "node_intensity_median":round(float(np.median(nodes)),1),
        "interchip_intensity_median":round(float(np.median(midpoint)),1),
        "node_minus_interchip_contrast":round(contrast,1),
        "nodes_above_board_threshold":int(sum(x>board_threshold for x in nodes)),
        "chip_grid_lps_mm_not_firmware_order":points,
    }



def compare_recovery_to_accepted_grid(recovery, accepted_positions_lps,
                                     accepted_normal_lps=None):
    """Order-independent research comparison to a successfully detected board.

    Uses optimal one-to-one assignment in physical LPS millimeters. Reflecting
    or rotating the 4x4 *index labels* is fine here: this comparison does not
    establish firmware ordering, JST side, or the face-normal sign.
    """
    from scipy.optimize import linear_sum_assignment
    candidate = np.asarray(
        recovery.get("chip_grid_lps_mm_not_firmware_order", []), dtype=float)
    reference = np.asarray(accepted_positions_lps, dtype=float)
    if candidate.shape != (16, 3) or reference.shape != (16, 3):
        return {"status": "no_complete_16_chip_comparison"}
    if not np.isfinite(candidate).all() or not np.isfinite(reference).all():
        return {"status": "invalid_or_nonfinite_chip_positions"}
    costs = np.linalg.norm(candidate[:, None, :] - reference[None, :, :], axis=2)
    rows, cols = linear_sum_assignment(costs)
    assigned = costs[rows, cols]
    center_error = np.linalg.norm(candidate.mean(axis=0) -
                                  reference.mean(axis=0))
    result = {
        "status": "comparison_available_not_firmware_mapping",
        "matched_chip_rms_mm": round(float(np.sqrt(np.mean(assigned**2))), 3),
        "matched_chip_p95_mm": round(float(np.percentile(assigned, 95)), 3),
        "matched_chip_max_mm": round(float(assigned.max()), 3),
        "chip_centroid_distance_mm": round(float(center_error), 3),
        "matched_chip_count": 16,
        "verified_firmware_indexing": False,
    }
    normal = recovery.get("estimated_face_normal_lps")
    if normal is not None and accepted_normal_lps is not None:
        a, b = np.asarray(normal, float), np.asarray(accepted_normal_lps, float)
        if a.shape == (3,) and b.shape == (3,) and np.isfinite(a).all() and np.isfinite(b).all():
            norm = np.linalg.norm(a) * np.linalg.norm(b)
            if norm > 1e-6:
                # sign unknown: compare unoriented planes, not chip-facing sign.
                cosine = np.clip(abs(np.dot(a,b) / norm),0,1)
                result["unsigned_normal_angle_deg"] = round(
                    float(np.degrees(np.arccos(cosine))), 2)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--crop",type=Path,required=True,
                        help="superior_roi_voxels.npz exported by cbct_superior_roi.py")
    parser.add_argument("--threshold",type=float,default=2290.594,
                        help="Board threshold from original failed detection")
    parser.add_argument("--out",type=Path,help="Optional JSON path")
    args=parser.parse_args()
    with np.load(args.crop,allow_pickle=False) as source:
        result=recover_grid(source["volume"],source["affine_ras"],
                            source["nominal_center_lps_mm"],args.threshold)
    path=args.out or (Path.home()/"Downloads"/"GK_CBCT_Results"/
                      "superior_chip_grid_candidate.json")
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print("Status:",result["status"])
    if "node_minus_interchip_score" in result:
        print("Grid pitch:",result["pitch_mm"],"mm")
        print("Grid center LPS mm:",result["chip_grid_center_lps_mm"])
        print("Node/interchip contrast:",result["node_minus_interchip_contrast"])
        print("Bright nodes:",result["nodes_above_board_threshold"],"/16")
    print("Diagnostic JSON:",path)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
