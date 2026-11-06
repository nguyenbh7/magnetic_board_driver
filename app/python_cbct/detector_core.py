"""Extracted (unmodified) detection functions from find_magnets_arrays.ipynb.

Notebook-derived code; use only with a trusted notebook. All image processing is
pre-acquisition and its outputs MUST be reviewed before any live fitting.
"""
from pathlib import Path
import csv
import gc
import os
import time
import warnings
from functools import lru_cache
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import SimpleITK as sitk
import torch
import torchio as tio
from scipy import ndimage as ndi
from scipy.spatial import ConvexHull, Delaunay
from IPython.display import Image, display
try:
    import pandas as pd
except ImportError:
    pd = None
sitk.ProcessObject_SetGlobalWarningDisplay(False)
LPS_FROM_RAS = np.diag([-1.0, -1.0, 1.0, 1.0])
GREEN = np.array([0, 255, 0], np.uint8)
YELLOW = np.array([255, 215, 0], np.uint8)
_PLANE = {"S": "axial", "I": "axial", "A": "coronal", "P": "coronal", "L": "sagittal", "R": "sagittal"}
_ORDER = ["right", "left", "anterior", "posterior", "superior", "inferior", "middle"]

# Trusted notebook SHA256: e5f1aea6b68e53f45aefdf444d9dada7317f52a32a704973ab73c90c3212c95e

def show_table(rows, digits=2):
    """Show a list of dicts as a table (pandas if installed, else plain text)."""
    if not rows:
        print("(nothing to show)")
        return
    if pd is not None:
        df = pd.DataFrame(rows).round(digits)
        df.index = range(1, len(df) + 1)
        display(df)
    else:
        keys = list(dict.fromkeys(k for r in rows for k in r))
        print("  ".join(keys))
        for r in rows:
            print("  ".join(f"{r.get(k, ''):.{digits}f}" if isinstance(r.get(k), float) else str(r.get(k, ""))
                            for k in keys))


def show_png(path, width=None):
    """Show a saved picture in the notebook."""
    display(Image(filename=str(path), width=width))


def load_scan(path):
    """Load a scan as a TorchIO ScalarImage.

    A NIfTI (or other single) file is read directly. For a folder, every DICOM series in it
    and its subfolders is listed and the CT image series is read: RT dose, plan and structure
    files exported alongside it are skipped. (Reading such a folder blindly can pick the dose
    grid instead of the CT.) If several image series qualify, the one with most slices wins.
    """
    path = Path(path)
    if path.is_file():
        return tio.ScalarImage(path)
    best = None                                          # (is CT, number of files, files, description)
    for folder in [path] + sorted(p for p in path.rglob("*") if p.is_dir()):
        for sid in sitk.ImageSeriesReader.GetGDCMSeriesIDs(str(folder)) or ():
            files = sitk.ImageSeriesReader.GetGDCMSeriesFileNames(str(folder), sid)
            reader = sitk.ImageFileReader()
            reader.SetFileName(files[0])
            try:
                reader.ReadImageInformation()
                modality = reader.GetMetaData("0008|0060").strip() if reader.HasMetaDataKey("0008|0060") else ""
            except RuntimeError:
                continue
            if modality.startswith("RT"):                # RTDOSE, RTSTRUCT, RTPLAN, RTIMAGE
                continue
            key = (modality == "CT", len(files))
            if best is None or key > best[:2]:
                best = (*key, files, f"{modality or 'unknown'} series, {len(files)} files in {folder}")
    if best is None:
        raise FileNotFoundError(f"no DICOM image series found in {path}")
    reader = sitk.ImageSeriesReader()
    reader.SetFileNames(best[2])
    print(f"reading {best[3]}")
    return tio.ScalarImage.from_sitk(reader.Execute())


def voxel_to_lps(affine_ras, ijk):
    """Voxel index (i, j, k), fractions allowed -> DICOM patient coordinates (LPS, mm)."""
    return (LPS_FROM_RAS @ affine_ras @ np.r_[np.asarray(ijk, float), 1.0])[:3]


def lps_to_voxel(affine_ras, xyz):
    """DICOM patient coordinates (LPS, mm) -> voxel index (i, j, k), fractional."""
    return (np.linalg.inv(LPS_FROM_RAS @ affine_ras) @ np.r_[np.asarray(xyz, float), 1.0])[:3]


def dir_to_lps(affine_ras, u):
    """Unit direction given in voxel-axis coordinates -> unit direction in LPS."""
    A = (LPS_FROM_RAS @ affine_ras)[:3, :3]
    d = (A / np.linalg.norm(A, axis=0)) @ u
    return d / np.linalg.norm(d)


def _display_axes(axis, orientation):
    """For the plane perpendicular to voxel `axis`: the voxel axes along the image columns (p)
    and rows (q), and whether each is flipped for the radiological view (patient's right on
    the left of the picture, anterior/superior at the top)."""
    p, q = [a for a in range(3) if a != axis]
    return p, q, orientation[p] in ("R", "A"), orientation[q] in ("S", "A")


def direction_name(d):
    """Plain-language name of a patient direction (LPS unit vector), e.g. 'left' or
    'superior-posterior'; components under 30% of the largest are left out."""
    names = (("right", "left"), ("anterior", "posterior"), ("inferior", "superior"))
    a = np.abs(d)
    order = [i for i in np.argsort(a)[::-1] if a[i] >= 0.3 * a.max()]
    return "-".join(names[i][int(d[i] > 0)] for i in order)


def name_by_location(objects, kind, ref_lps):
    """Name objects after where they are, seen from `ref_lps` (the middle of the scan):
    e.g. magnet_right, array_superior (numbered if two share a direction), and store the
    fuller description in o["location"] (e.g. 'superior-posterior'). Sorts the list in place:
    right, left, anterior, posterior, superior, inferior."""
    for o in objects:
        d = o["center_lps"] - ref_lps
        o["location"] = direction_name(d / np.linalg.norm(d)) if np.linalg.norm(d) > 5 else "middle"
    objects.sort(key=lambda o: (_ORDER.index(o["location"].split("-")[0]), o["center_lps"][2]))
    main = [o["location"].split("-")[0] for o in objects]
    seen = {}
    for o, m in zip(objects, main):
        seen[m] = seen.get(m, 0) + 1
        o["name"] = f"{kind}_{m}" if main.count(m) == 1 else f"{kind}_{m}_{seen[m]}"


def scan_center_lps(image):
    """Patient coordinates (LPS, mm) of the middle of the scanned volume."""
    return voxel_to_lps(image.affine, (np.asarray(image.spatial_shape) - 1) / 2)


def show_mips(vol, orientation, spacing, window, title=""):
    """Axial, coronal and sagittal maximum-intensity projections, radiological convention."""
    orientation = tuple(orientation)
    order = ["axial", "coronal", "sagittal"]
    planes = sorted(range(3), key=lambda a: order.index(_PLANE[orientation[a]]))
    fig, axs = plt.subplots(1, 3, figsize=(16, 5.6))
    for a, axis in zip(axs, planes):
        p, q, flip_c, flip_r = _display_axes(axis, orientation)
        img = vol.max(axis=axis).T                       # rows follow axis q, columns axis p
        if flip_c:
            img = img[:, ::-1]
        if flip_r:
            img = img[::-1]
        a.imshow(img, cmap="gray", vmin=window[0], vmax=window[1], aspect=spacing[q] / spacing[p])
        a.set_title(f"{_PLANE[orientation[axis]]} projection", fontsize=10)
        a.axis("off")
    fig.suptitle(title, fontsize=11)
    plt.tight_layout()
    plt.show()


def _ball(radius_mm, spacing):
    """Small 3D sphere (boolean array) of the given radius in mm, at least one voxel;
    the structuring element for morphological clean-up, round in mm for any voxel size."""
    r = np.maximum(radius_mm / spacing, 1.0)
    n = np.ceil(r).astype(int)
    g = np.ogrid[-n[0]:n[0] + 1, -n[1]:n[1] + 1, -n[2]:n[2] + 1]
    return (g[0] / r[0]) ** 2 + (g[1] / r[1]) ** 2 + (g[2] / r[2]) ** 2 <= 1.0


@lru_cache(maxsize=16)
def _magnet_exclusion_edt_is_exact(radius_mm, spacing_tuple):
    """Only use EDT when its discrete footprint equals the original ball.

    The original _ball() guarantees at least one voxel along each dimension
    and evaluates an ellipsoid using normalized coordinates. EDT in physical
    units can differ for some anisotropic spacings and at floating-point
    boundaries. Comparing the actual SciPy EDT kernel with the old footprint
    ensures that the fallback is taken unless they are pixel-for-pixel equal.
    """
    spacing = np.asarray(spacing_tuple, dtype=float)
    footprint = _ball(radius_mm, spacing)
    impulse = np.zeros(footprint.shape, dtype=bool)
    impulse[tuple(np.asarray(footprint.shape) // 2)] = True
    edt_footprint = ndi.distance_transform_edt(
        ~impulse, sampling=spacing) <= radius_mm
    return bool(np.array_equal(footprint, edt_footprint))


def _dilate_magnet_exclusion(mask, spacing, radius_mm=3.0, *, use_edt=False):
    """Dilate a candidate exactly as the old morphology, or use guarded EDT.

    Return (expanded mask, algorithm label) for the sanitized performance
    audit. Empty input masks are special-cased for EDT because the distance
    transform of an array with no background is not defined as a dilation.
    """
    if not use_edt:
        return ndi.binary_dilation(mask, structure=_ball(radius_mm, spacing)), "original"
    spacing_tuple = tuple(float(v) for v in spacing)
    if not _magnet_exclusion_edt_is_exact(radius_mm, spacing_tuple):
        return (ndi.binary_dilation(mask, structure=_ball(radius_mm, spacing)),
                "original_spacing_fallback")
    if not np.any(mask):
        return np.zeros_like(mask, dtype=bool), "edt_empty"
    return (ndi.distance_transform_edt(
        ~np.asarray(mask, dtype=bool), sampling=spacing) <= radius_mm, "edt")


def _magnet_exclusion_edt_enabled():
    """Return production default, permitting an explicit original-method opt-out."""
    return os.environ.get("GK_CBCT_MAGNET_EXCLUSION_EDT", "1") == "1"


def _disk(radius_px):
    """2D disk (boolean array) of the given radius in pixels."""
    n = int(np.ceil(radius_px))
    g = np.ogrid[-n:n + 1, -n:n + 1]
    return g[0] ** 2 + g[1] ** 2 <= radius_px ** 2


def principal_extents(mask, spacing):
    """Size of an object along its own three principal axes: the full extent of its voxels
    projected on each axis (plus one voxel), largest first. Works for any compact shape
    (cylinder, disc, block, sphere). Returns (sizes in mm, unit axes as columns, voxel axes)."""
    pts = np.argwhere(mask) * spacing
    if len(pts) < 4:
        return np.zeros(3), np.eye(3)
    # principal axes: eigenvectors of the covariance of the voxel positions, most spread first
    w, V = np.linalg.eigh(np.cov((pts - pts.mean(0)).T))
    V = V[:, np.argsort(w)[::-1]]
    proj = (pts - pts.mean(0)) @ V
    sizes = proj.max(0) - proj.min(0) + np.abs(V.T) @ spacing          # + one voxel along each axis
    return sizes, V


def solidity(mask):
    """Fraction of the discrete convex hull occupied by the object.

    The original method triangulates every occupied voxel, then counts lattice
    points inside the resulting convex hull. The default-on optimized method
    triangulates true convex-hull vertices, preserving the same hull
    mathematically while avoiding thousands of redundant interior vertices.
    Seven paired real-scan checks found unchanged measured CT geometry and
    aggregate candidate counts; some numerical boundary differences remain
    possible on untested images. Set GK_CBCT_SOLIDITY_HULL_VERTICES=0 to
    explicitly restore the all-voxel triangulation.
    """
    pts = np.argwhere(mask)
    if len(pts) < 10:
        return 0.0
    try:
        if os.environ.get("GK_CBCT_SOLIDITY_HULL_VERTICES", "1") == "1":
            try:
                boundary_vertices = ConvexHull(pts).vertices
                hull = Delaunay(pts[boundary_vertices])
            except Exception:
                # Construction failure must not cause a valid mask to be
                # rejected for this reason alone: fall back to original.
                hull = Delaunay(pts)
        else:
            hull = Delaunay(pts)
    except Exception:                                    # all points in one plane
        return 0.0
    lo, hi = pts.min(0), pts.max(0) + 1
    grid = np.indices(hi - lo).reshape(3, -1).T + lo
    return float(len(pts) / max(int((hull.find_simplex(grid) >= 0).sum()), 1))


def outline_paths(mask2d):
    """Perimeter of a 2D shape: closed lines of (row, col) points (marching squares at 0.5)."""
    import contourpy                                               # installed with matplotlib
    lines = contourpy.contour_generator(z=mask2d.astype(float)).lines(0.5)
    return [np.c_[ln[:, 1], ln[:, 0]] for ln in lines]


def _float_data(image):
    """The scan's voxel values as float32 (no copy when they already are)."""
    data = image.data[0].numpy()
    return data if data.dtype == np.float32 else data.astype(np.float32)


def _auto_levels(smooth, levels):
    """Thresholds a given fraction of the way from the brightest ordinary material
    (99.5th percentile of the scan) to its brightest voxel."""
    hi_anat = float(np.percentile(smooth[::2, ::2, ::2], 99.5))
    top = float(smooth.max())
    return [hi_anat + f * (top - hi_anat) for f in levels], hi_anat, top


def magnet_candidates(image, size=(4.0, 20.0), threshold=None, levels=(0.9, 0.75), min_peak_level=0.85,
                      tolerance=1.0, max_elongation=3.0, min_solidity=0.8, max_bloom_mm=2.0, verbose=True):
    """First pass of the magnet search: every compact, very dense object of magnet size.
    Dense chips on a circuit board can qualify here too; pick_magnets removes them once the
    boards are known.

    Args:
      image           TorchIO ScalarImage
      size            (min, max) magnet size in mm: the smallest dimension must reach min,
                      the largest must not exceed max (plus tolerance)
      threshold       one fixed seed threshold in scan units; None = automatic levels
      levels          automatic seed thresholds, as fractions of the way from the brightest
                      ordinary material (99.5th percentile of the scan) to the brightest voxel
      min_peak_level  a magnet's core must be at least this bright (same kind of fraction):
                      magnets are among the densest things in the scan, so bone-like or
                      ceramic objects are left out
      tolerance       mm of slack on the size range
      max_elongation  largest / smallest dimension must be at most this
      min_solidity    share of its convex hull the object must fill
      max_bloom_mm    the outline may extend at most this far beyond the dense core, so metal
                      streaks and anything the magnet touches stay out of the outline
      verbose         print the thresholds used
    Returns (candidates, rejected); each candidate is a dict with the center (LPS mm and voxel),
    size (mm, largest first), long-axis direction, mask and window (for drawing).
    """
    audit_start = time.perf_counter()
    data = _float_data(image)
    spacing = np.asarray(image.spacing, float)
    # light smoothing (0.3 mm), used only to find seeds; all measurements use the raw data
    smooth = ndi.gaussian_filter(data, sigma=0.3 / spacing)
    smooth_clock = time.perf_counter()
    (min_peak,), hi_anat, top = _auto_levels(smooth, (min_peak_level,))
    threshold_clock = time.perf_counter()
    # Reuse the same percentile/maximum for the other magnet thresholds:
    # calling _auto_levels twice previously recomputed a costly full-volume
    # percentile, even though both calls use the identical smoothed image.
    thresholds = ([hi_anat + level * (top - hi_anat) for level in levels]
                  if threshold is None else [float(threshold)])
    if verbose:
        print(f"magnets: brightest ordinary material {hi_anat:.0f}, brightest voxel {top:.0f}, "
              f"magnet cores must reach {min_peak:.0f}")
    found, rejected = [], []
    level_reports = []
    for level, thr in enumerate(thresholds, start=1):
        label_start = time.perf_counter()
        labels, n = ndi.label(smooth > thr)
        label_seconds = time.perf_counter() - label_start
        if verbose:
            print(f"  level {level}: threshold {thr:.0f} -> {n} dense objects")
        level_audit = {"component_stage_seconds": {}, "candidate_counts": {}}
        measurement_start = time.perf_counter()
        _magnet_level(data, labels, spacing, image.affine, size, tolerance, max_elongation, min_solidity,
                      max_bloom_mm, min_peak, found, rejected, level, audit=level_audit)
        measurement_seconds = time.perf_counter() - measurement_start
        level_reports.append({
            "level": level, "dense_objects": int(n),
            "candidate_counts": level_audit["candidate_counts"],
            "timing_seconds": {
                "threshold_mask_and_component_labeling": round(label_seconds, 3),
                "component_processing": round(measurement_seconds, 3),
                **{name: round(seconds, 3)
                   for name, seconds in level_audit["component_stage_seconds"].items()},
            },
        })
        del labels
    magnet_candidates.last_audit = {
        "levels": level_reports,
        "accepted_candidates_before_board_exclusion": len(found),
        "rejected_candidate_records": len(rejected),
        "timing_seconds": {
            "setup_and_gaussian_smoothing": round(smooth_clock - audit_start, 3),
            "automatic_threshold_estimation": round(threshold_clock - smooth_clock, 3),
            "threshold_mask_and_component_labeling": round(sum(
                entry["timing_seconds"]["threshold_mask_and_component_labeling"]
                for entry in level_reports), 3),
            "component_processing": round(sum(
                entry["timing_seconds"]["component_processing"]
                for entry in level_reports), 3),
            "total": round(time.perf_counter() - audit_start, 3),
        },
    }
    return found, rejected


def _outline_distance_roi(mask, spacing, radius_mm=2.5):
    """Distance field on the exact radius-padded bounding box of a mask.

    Any voxel with EDT distance <= radius_mm must lie in this box. Return
    original full-volume EDT for an empty mask: SciPy's empty-mask boundary
    convention is not interchangeable with an all-False dilation. This helper
    is used by the validated cropped magnet-outline method (default on,
    with explicit GK_CBCT_CROP_MAGNET_OUTLINE_EDT=0 rollback).
    """
    if not np.any(mask):
        roi = tuple(slice(0, dim) for dim in mask.shape)
        return roi, ndi.distance_transform_edt(~mask, sampling=spacing)
    coords = np.where(mask)
    padding = np.ceil(radius_mm / np.asarray(spacing)).astype(int)
    roi = tuple(
        slice(max(0, int(axis.min()) - int(pad)),
              min(dim, int(axis.max()) + 1 + int(pad)))
        for axis, pad, dim in zip(coords, padding, mask.shape))
    return roi, ndi.distance_transform_edt(~mask[roi], sampling=spacing)


def _core_depth_cropped_exact(core, spacing):
    """Return the original-shape foreground EDT using a safe interior crop.

    For foreground pixels, distances to background depend only on the mask's
    bounding box plus a one-voxel *false* halo. Because all original foreground
    lies inside that box, the rest of the original EDT is exactly zero.
    Fall back to the full original calculation if the foreground touches any
    volume face, since SciPy's out-of-volume boundary convention must be kept.
    Empty masks also take the original path.
    """
    occupied = np.where(core)
    if occupied[0].size == 0:
        return ndi.distance_transform_edt(core, sampling=spacing)
    bounds = [(int(points.min()), int(points.max())) for points in occupied]
    for axis, (lo, hi) in enumerate(bounds):
        if lo == 0 or hi == core.shape[axis] - 1:
            return ndi.distance_transform_edt(core, sampling=spacing)
    roi = tuple(slice(lo - 1, hi + 2) for lo, hi in bounds)
    depth = np.zeros(core.shape, dtype=np.float64)
    depth[roi] = ndi.distance_transform_edt(core[roi], sampling=spacing)
    return depth


def _magnet_level(data, labels, spacing, affine, size, tolerance, max_elongation, min_solidity,
                  max_bloom_mm, min_peak, found, rejected, level, audit=None):
    """Examine every dense object at one seed threshold (helper for magnet_candidates): clean
    its core, grow it to its half-maximum edge, check size and shape, compute the center."""
    # allowed size range, with the tolerance as slack on both ends
    lo_s, hi_s = size[0] - tolerance, size[1] + tolerance
    # Default ON after the five Oct30 + two Oct23 paired imaging-geometry
    # regressions passed. Set GK_CBCT_REUSE_INITIAL_MAGNET_EDT=0 to restore
    # the original redundant first-ring distance transform for comparison.
    reuse_initial_edt = os.environ.get("GK_CBCT_REUSE_INITIAL_MAGNET_EDT", "1") == "1"
    # Default ON after 21/21 paired CT imaging geometry and magnet-candidate
    # signature regressions passed across seven real scans with repeated
    # measured improvements in outline EDT and end-to-end processing.
    # Set GK_CBCT_CROP_MAGNET_OUTLINE_EDT=0 to restore full-window EDT.
    crop_outline_edt = (
        os.environ.get("GK_CBCT_CROP_MAGNET_OUTLINE_EDT", "1") == "1")
    # Default ON after 154 Windows tests and 21/21 paired seven-scan
    # CT-geometry, magnet- and board-candidate regressions PASS.
    # Set GK_CBCT_CROP_MAGNET_CORE_EDT=0 for the original full-window EDT.
    crop_core_edt = os.environ.get("GK_CBCT_CROP_MAGNET_CORE_EDT", "1") == "1"
    pad = np.ceil((hi_s / 2 + 5) / spacing).astype(int)            # window margin in voxels
    done = set()
    stage = (audit.setdefault("component_stage_seconds", {}) if audit is not None else None)
    counts = (audit.setdefault("candidate_counts", {}) if audit is not None else None)
    def record_stage(name, started):
        if stage is not None:
            stage[name] = stage.get(name, 0.0) + time.perf_counter() - started
    slice_start = time.perf_counter()
    slices = ndi.find_objects(labels)
    record_stage("component_slice_enumeration", slice_start)
    # one pass per dense piece; `sl` is its bounding box
    for lab, sl in enumerate(slices, start=1):
        if sl is None or lab in done:
            continue
        if counts is not None:
            counts["components_considered"] = counts.get("components_considered", 0) + 1
        extent = np.array([s.stop - s.start for s in sl]) * spacing
        if extent.max() > 2 * hi_s:                                # skull, boards, holders ...
            rejected.append((lab, f"too large ({extent.max():.0f} mm across)", None, level))
            continue
        # work in a window around the piece (its box plus a margin)
        win = tuple(slice(max(s.start - p, 0), min(s.stop + p, n_)) for s, p, n_ in zip(sl, pad, data.shape))
        local = data[win]
        core = labels[win] == lab
        if core.sum() * np.prod(spacing) < 1.0:                    # a few noisy voxels
            rejected.append((lab, "too small", None, level))
            continue
        # Metal streaks can leave saturated fans ("flares") attached to a magnet's edges. Trim
        # them from the core with an opening by a ball half as wide as the core's thickest part:
        # the magnet's body survives, thin fans do not.
        step_start = time.perf_counter()
        core_step = time.perf_counter()
        depth = (_core_depth_cropped_exact(core, spacing) if crop_core_edt
                 else ndi.distance_transform_edt(core, sampling=spacing))
        record_stage("core_distance_transform", core_step)
        if counts is not None:
            counts["core_edt_calls"] = counts.get("core_edt_calls", 0) + 1
        core_step = time.perf_counter()
        r_open = 0.5 * depth.max()
        record_stage("core_opening_radius_and_gate", core_step)
        if r_open >= 2 * spacing.min():
            core_step = time.perf_counter()
            opened = ndi.binary_opening(core, structure=_ball(r_open, spacing))
            record_stage("core_binary_opening", core_step)
            if counts is not None:
                counts["core_opening_calls"] = counts.get("core_opening_calls", 0) + 1
            core_step = time.perf_counter()
            lab_o, n_o = ndi.label(opened)
            record_stage("core_opened_component_labeling", core_step)
            if n_o:
                core_step = time.perf_counter()
                deepest = np.unravel_index(np.argmax(np.where(opened, depth, 0)), core.shape)
                core = lab_o == lab_o[deepest]
                record_stage("core_opened_component_selection", core_step)
        record_stage("core_edt_and_opening", step_start)
        step_start = time.perf_counter()
        # peak = typical value in the middle of the core
        core_pts = np.argwhere(core) * spacing
        r_core = np.linalg.norm(core_pts - core_pts.mean(0), axis=1)
        middle = r_core <= 0.5 * r_core.max() + 1e-6
        peak = float(np.median(local[core][middle])) if middle.any() else float(local[core].max())
        record_stage("peak_intensity_measurement", step_start)
        step_start = time.perf_counter()
        # half-maximum outline (background measured 1.0-2.5 mm outside the current outline),
        # at most max_bloom_mm beyond the core
        # The bloom limit and first background ring share this exact EDT:
        # mask == core on iteration zero. Reuse is validated and default-on;
        # GK_CBCT_REUSE_INITIAL_MAGNET_EDT=0 restores the original extra call.
        outline_step = time.perf_counter()
        if crop_outline_edt:
            dist_roi, dist = _outline_distance_roi(
                core, spacing, radius_mm=max(2.5, max_bloom_mm))
            near = np.zeros(core.shape, dtype=bool)
            near[dist_roi] = dist <= max_bloom_mm
        else:
            dist = ndi.distance_transform_edt(~core, sampling=spacing)
            near = dist <= max_bloom_mm
        record_stage("outline_initial_edt_and_bloom_mask", outline_step)
        if counts is not None:
            counts["outline_initial_edt_calls"] = counts.get("outline_initial_edt_calls", 0) + 1
        mask = core
        for iteration in range(3):
            if counts is not None:
                counts["outline_iterations"] = counts.get("outline_iterations", 0) + 1
            if iteration or not reuse_initial_edt:
                outline_step = time.perf_counter()
                if crop_outline_edt:
                    dist_roi, dist = _outline_distance_roi(
                        mask, spacing, radius_mm=2.5)
                else:
                    dist = ndi.distance_transform_edt(~mask, sampling=spacing)
                record_stage("outline_repeated_edt", outline_step)
                if counts is not None:
                    counts["outline_repeated_edt_calls"] = (
                        counts.get("outline_repeated_edt_calls", 0) + 1)
            outline_step = time.perf_counter()
            if crop_outline_edt:
                ring = np.zeros(mask.shape, dtype=bool)
                ring[dist_roi] = (dist > 1.0) & (dist <= 2.5)
            else:
                ring = (dist > 1.0) & (dist <= 2.5)
            background = float(np.median(local[ring])) if ring.any() else float(np.median(local))
            record_stage("outline_ring_and_background", outline_step)
            outline_step = time.perf_counter()
            hm_labels, _ = ndi.label((local > 0.5 * (peak + background)) & near)
            record_stage("outline_threshold_and_label", outline_step)
            outline_step = time.perf_counter()
            keep = np.unique(hm_labels[core & (hm_labels > 0)])
            mask = np.isin(hm_labels, keep[keep > 0])
            record_stage("outline_component_selection", outline_step)
        record_stage("iterative_half_max_outline", step_start)
        step_start = time.perf_counter()
        # clean-up: opening trims streak spikes; keep pieces touching the core; closing and
        # hole filling bridge slits cut by dark streaks
        cleanup_step = time.perf_counter()
        ball = _ball(0.6, spacing)
        opened = ndi.binary_opening(mask, structure=ball)
        op_labels, n_op = ndi.label(opened)
        record_stage("cleanup_open_and_label", cleanup_step)
        cleanup_step = time.perf_counter()
        if n_op:
            overlap = ndi.sum(core, op_labels, index=np.arange(1, n_op + 1))
            opened = np.isin(op_labels, 1 + np.flatnonzero(overlap > 0))
        if opened.sum() >= 0.6 * mask.sum():
            mask = opened
        record_stage("cleanup_keep_core_overlap", cleanup_step)
        cleanup_step = time.perf_counter()
        mask = ndi.binary_fill_holes(ndi.binary_closing(mask, structure=ball))
        record_stage("cleanup_close_and_fill", cleanup_step)
        # every dense piece inside this outline belongs to this object: do not start again from it
        cleanup_step = time.perf_counter()
        done.update(np.unique(labels[win][mask | (labels[win] == lab)]).tolist())
        record_stage("cleanup_suppress_revisited_labels", cleanup_step)
        record_stage("outline_cleanup_and_suppression", step_start)
        step_start = time.perf_counter()
        # size, shape and center
        dims, axes = principal_extents(mask, spacing)
        # center: every voxel inside the outline, weighted by how far it is above the background
        w = np.where(mask, local - background, 0).clip(min=0)
        c_vox = np.array(ndi.center_of_mass(w)) + np.array([s.start for s in win])
        c_lps = voxel_to_lps(affine, c_vox)
        record_stage("geometry_and_center", step_start)
        # already found (same magnet seen at another level, or a fragment of it)?
        if any(np.linalg.norm(c_lps - f["center_lps"]) < 1.5 for f in found):
            continue                                               # same magnet, other level or fragment
        # the tests (step 7); every failed one is recorded
        reasons = []
        if peak < min_peak:
            reasons.append(f"not dense enough ({peak:.0f})")
        if dims[2] < lo_s:
            reasons.append(f"too thin ({dims[2]:.1f} mm)")
        if dims[0] > hi_s:
            reasons.append(f"too long ({dims[0]:.1f} mm)")
        if dims[0] > max_elongation * max(dims[2], 1e-6):
            reasons.append(f"elongated ({dims[0] / dims[2]:.1f}:1)")
        if not reasons:
            solidity_start = time.perf_counter()
            sol = solidity(mask)
            record_stage("convex_hull_solidity", solidity_start)
            if sol < min_solidity:
                reasons.append(f"irregular (solidity {sol:.2f})")
        if counts is not None:
            counts["full_measurements"] = counts.get("full_measurements", 0) + 1
        # keep everything measured, plus the outline mask and its window (for pictures)
        rec = dict(kind="magnet", label=lab, level=level, center_voxel=c_vox, center_lps=c_lps, dims=dims,
                   axis_lps=dir_to_lps(affine, axes[:, 0]), peak=peak, background=background,
                   core_mm3=float(core.sum() * np.prod(spacing)), volume_mm3=float(mask.sum() * np.prod(spacing)),
                   mask=mask, window=tuple((s.start, s.stop) for s in win))
        (rejected.append((lab, ", ".join(reasons), rec, level)) if reasons else found.append(rec))


def _print_rejected(rejected, what):
    """Report each rejected object once (objects only counted when too small or large)."""
    shown, lines, counts = set(), [], {}
    for _, reason, rec, level in rejected:
        if rec is None:
            key = (level, reason.split(" (")[0])
            counts[key] = counts.get(key, 0) + 1
            continue
        key = tuple(np.round(rec["center_lps"] / 2).astype(int))
        if key in shown:
            continue
        shown.add(key)
        lines.append(f"  level {level}: {reason} at LPS {np.round(rec['center_lps'], 1)}")
    print(f"rejected (measured, not {what}-like): {len(lines)}")
    if lines:
        print("\n".join(lines[:30]) + (f"\n  ... {len(lines) - 30} more" if len(lines) > 30 else ""))
    if counts:
        print("also skipped: " + ", ".join(f"level {k[0]} {k[1]}: {c}" for k, c in sorted(counts.items())))


def _inside_board(p_mm, rec, normal_mm=6.0, edge_mm=2.0):
    """Is the point (voxel-axis mm) on the board `rec`: within its rectangle (plus edge_mm)
    and within normal_mm of its mid-plane?"""
    d = p_mm - rec["board_center_mm"]
    return (abs(d @ rec["e3"]) < normal_mm and abs(d @ rec["e1"]) < rec["dims"][0] / 2 + edge_mm
            and abs(d @ rec["e2"]) < rec["dims"][1] / 2 + edge_mm)


def _on_board(p_mm, rec):
    """Is the point inside the board itself: within its rectangle and its thickness (+1.5 mm)?
    Used to tell a chip on the board from a magnet resting on or near it."""
    return _inside_board(p_mm, rec, rec["t_half"] + 1.5, 1.0)


def _fit_plane(pts, keep_mm=3.0, rounds=3):
    """Robust plane through 3D points (mm): PCA, then refit on points within keep_mm of the
    plane, a few times. Returns (center, in-plane axes u, v (largest spread first), normal)."""
    sel = pts
    for _ in range(rounds):
        c = sel.mean(0)
        w, V = np.linalg.eigh(np.cov((sel - c).T))
        # eigh sorts eigenvalues ascending: V[:, 0] is the direction of least spread = the normal
        n = V[:, 0]
        near = np.abs((pts - c) @ n) <= keep_mm
        if near.sum() < 50:
            break
        sel = pts[near]
    c = sel.mean(0)
    w, V = np.linalg.eigh(np.cov((sel - c).T))
    return c, V[:, 2], V[:, 1], V[:, 0]


def _reslice(data, spacing, center, e1, e2, e3, u_range, v_range, t_range, step=0.25):
    """Sample the scan on a plane grid: points center + u*e1 + v*e2 + t*e3 (mm, voxel axes).
    Returns the stack of slices (t, u, v) and the grid coordinates."""
    gu = np.arange(u_range[0], u_range[1] + 1e-9, step)
    gv = np.arange(v_range[0], v_range[1] + 1e-9, step)
    gt = np.arange(t_range[0], t_range[1] + 1e-9, step)
    U, V = np.meshgrid(gu, gv, indexing="ij")
    base = center + U[..., None] * e1 + V[..., None] * e2
    stack = np.empty((len(gt), len(gu), len(gv)), np.float32)
    # Default ON after seven paired real-scan geometry regressions passed
    # with no changes in measured positions or grid fits. Set
    # GK_CBCT_CACHE_RESLICE_MIN=0 to recover the original per-slice calculation.
    cache_volume_min = os.environ.get("GK_CBCT_CACHE_RESLICE_MIN", "1") == "1"
    outside_value = float(data.min()) if cache_volume_min else None
    # Alternative batching remains experimental (OFF by default): combine a few consecutive depth planes into each
    # SciPy interpolation call. Keep EXACTLY the original physical coordinates,
    # interpolation order, constant fill and float32 output. Memory is capped
    # at 1,000,000 output samples (~24 MB just for float64 coordinates).
    # A single-plane batch uses the original loop. The minimum-cache switch
    # remains independent for legacy controlled A/B comparisons.
    # Default ON after 21/21 paired real-scan CT geometry + board-candidate
    # regression passes and repeated attributable reslice speedups.
    # GK_CBCT_AFFINE_RESLICE=0 restores the original per-plane sampler.
    affine_enabled = (os.environ.get("GK_CBCT_AFFINE_RESLICE", "1") == "1"
                      and cache_volume_min)
    batch_enabled = (os.environ.get("GK_CBCT_BATCH_RESLICE") == "1"
                     and cache_volume_min and not affine_enabled)
    points_per_plane = len(gu) * len(gv)
    batch_planes = min(4, max(1, 1_000_000 // points_per_plane)) if batch_enabled else 1
    if affine_enabled:
        # Validated on seven CT acquisitions with three paired repeats each.
        # Output indices (t,u,v) map into the same physical voxel coordinates;
        # SciPy generates them internally, avoiding large per-plane coordinate
        # tensors. Floating-point evaluation can still differ on unseen
        # acquisitions, so retain the original sampler as an explicit override.
        matrix = np.stack((step * e3 / spacing, step * e1 / spacing,
                           step * e2 / spacing), axis=1)
        offset = (center + gu[0] * e1 + gv[0] * e2 + gt[0] * e3) / spacing
        stack = ndi.affine_transform(
            data, matrix, offset=offset, output_shape=stack.shape,
            output=np.float32, order=1, mode="constant",
            cval=outside_value, prefilter=False)
    elif batch_planes > 1:
        for k in range(0, len(gt), batch_planes):
            depths = gt[k:k + batch_planes]
            coords = ((base[None, ...] + depths[:, None, None, None] * e3)
                      / spacing).reshape(-1, 3).T
            sampled = ndi.map_coordinates(data, coords, order=1, cval=outside_value)
            stack[k:k + len(depths)] = sampled.reshape((len(depths),) + U.shape)
    else:
        for k, t in enumerate(gt):
            # mm -> fractional voxel indices; trilinear interpolation; air outside the scan
            coords = ((base + t * e3) / spacing).reshape(-1, 3).T
            cval = outside_value if cache_volume_min else float(data.min())
            stack[k] = ndi.map_coordinates(data, coords, order=1, cval=cval).reshape(U.shape)
    return stack, gu, gv, gt


def _min_area_rect(points):
    """Smallest rectangle around 2D points (rotating calipers over the convex hull).
    Returns (center, side lengths (long, short), unit axes along those sides)."""
    hull = points[ConvexHull(points).vertices]
    edges = np.diff(np.vstack([hull, hull[:1]]), axis=0)
    best = None
    # one side lies along a hull edge: try each edge's angle (modulo 90 degrees)
    for a in np.unique(np.mod(np.arctan2(edges[:, 1], edges[:, 0]), np.pi / 2)):
        R = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
        p = hull @ R.T
        lo, hi = p.min(0), p.max(0)
        if best is None or np.prod(hi - lo) < best[0]:
            best = (np.prod(hi - lo), R, lo, hi)
    _, R, lo, hi = best
    center = ((lo + hi) / 2) @ R
    sides = hi - lo
    order = np.argsort(sides)[::-1]
    return center, sides[order], R[order]


def _otsu(values, bins=256):
    """Otsu's threshold: the value that best splits `values` into two groups."""
    hist, edges = np.histogram(values, bins=bins)
    mids = 0.5 * (edges[1:] + edges[:-1])
    # for every cut: share of values below/above (w0, w1) and their means (m0, m1)
    w0 = np.cumsum(hist)
    w1 = w0[-1] - w0
    m0 = np.cumsum(hist * mids) / np.maximum(w0, 1)
    m1 = (np.sum(hist * mids) - np.cumsum(hist * mids)) / np.maximum(w1, 1)
    return float(mids[np.argmax(w0 * w1 * (m0 - m1) ** 2)])


def _footprint(img, seed, step, cut=None):
    """Board footprint in a face-on image: the board's pixels (split from the surroundings
    with Otsu's threshold, or `cut` if given), holes filled, thin attachments (cables,
    wires) trimmed off. `seed` marks pixels known to be on the board.
    Returns (mask, board level, background, threshold used)."""
    cut = _otsu(img) if cut is None else cut
    background = float(np.median(img[img <= cut]))
    fp = img > cut
    lab, n = ndi.label(fp)
    keep = np.unique(lab[seed & (lab > 0)])
    fp = ndi.binary_fill_holes(np.isin(lab, keep[keep > 0]))
    fp = ndi.binary_opening(fp, structure=_disk(2.5 / step))      # trims cables narrower than 5 mm
    lab, n = ndi.label(fp)
    if n > 1:                                                      # keep the biggest piece
        fp = lab == 1 + np.argmax(ndi.sum(fp, lab, np.arange(1, n + 1)))
    fp = ndi.binary_fill_holes(fp)
    level = float(np.median(img[fp])) if fp.any() else cut
    return fp, level, background, cut


def _shift(a, du, dv, fill):
    """a shifted so that out[u, v] = a[u + du, v + dv] (integer pixels), `fill` outside."""
    out = np.full_like(a, fill)
    H, W = a.shape
    u0, u1 = max(0, -du), min(H, H - du)
    v0, v1 = max(0, -dv), min(W, W - dv)
    if u0 < u1 and v0 < v1:
        out[u0:u1, v0:v1] = a[u0 + du:u1 + du, v0 + dv:v1 + dv]
    return out


def _find_grid(img, inside, grid, step, pitch_mm):
    """Locate a rows x cols lattice of bright chips in a face-on image (axes u, v).

    Every lattice (pitch along u and v, position) is scored by the mean brightness at its
    points minus the mean brightness halfway between neighboring points; all points must
    lie on the board (`inside`). Both orientations of the grid are tried.
    Returns (score, center (u, v) px, pitch (u, v) px, (rows along v, cols along u)) or None.
    """
    best = None
    sm = ndi.gaussian_filter(img, 0.5 / step)                       # chips are ~2-3 mm across
    fill = float(sm.min())
    pitches = np.arange(pitch_mm[0], pitch_mm[1] + 1e-9, 0.25) / step
    # grid orientation: n_u chips along u and n_v along v (a 4 x 4 grid has only one)
    for n_u, n_v in {(grid[1], grid[0]), (grid[0], grid[1])}:
        a_off = np.arange(n_u) - (n_u - 1) / 2                      # lattice offsets along u (in pitches)
        b_off = np.arange(n_v) - (n_v - 1) / 2
        for pu in pitches:
            du = np.rint(a_off * pu).astype(int)
            dm = np.rint((a_off[:-1] + 0.5) * pu).astype(int)       # halfway between columns
            A = sum(_shift(sm, d, 0, fill) for d in du)             # sum over the points of each row
            Am = sum(_shift(sm, d, 0, fill) for d in dm)
            # Fu[u, v] is True when every point of that row lies on the board
            Fu = np.minimum.reduce([_shift(inside, d, 0, False) for d in du])
            for pv in pitches:
                dv = np.rint(b_off * pv).astype(int)
                dvm = np.rint((b_off[:-1] + 0.5) * pv).astype(int)
                # mean brightness at all lattice points, for every lattice center (u, v) at once
                nodes = sum(_shift(A, 0, d, fill) for d in dv) / (n_u * n_v)
                # ... and halfway between neighbors along u (mid_u) and along v (mid_v)
                mid_u = sum(_shift(Am, 0, d, fill) for d in dv) / ((n_u - 1) * n_v)
                mid_v = sum(_shift(A, 0, d, fill) for d in dvm) / (n_u * (n_v - 1))
                # lattices with every point on the board
                ok = np.minimum.reduce([_shift(Fu, 0, d, False) for d in dv])
                # high when the points sit on chips and the midpoints on the board between them
                score = np.where(ok, nodes - 0.5 * (mid_u + mid_v), -np.inf)
                k = int(np.argmax(score))
                if best is None or score.flat[k] > best[0]:
                    best = (float(score.flat[k]), np.array(np.unravel_index(k, score.shape), float),
                            np.array([pu, pv]), (n_v, n_u))
    return best if best is not None and np.isfinite(best[0]) else None


def _timed_board_operation(audit, name, func, *args, **kwargs):
    """Time an unchanged board-measurement call; only aggregate non-image data.

    This wrapper never changes function arguments, results, exceptions or the
    order of detector operations. Timers are disabled when audit is None.
    """
    if audit is None:
        return func(*args, **kwargs)
    start = time.perf_counter()
    try:
        return func(*args, **kwargs)
    finally:
        elapsed = time.perf_counter() - start
        times = audit.setdefault("substage_seconds", {})
        calls = audit.setdefault("call_counts", {})
        times[name] = times.get(name, 0.0) + elapsed
        calls[name] = calls.get(name, 0) + 1


def _measure_board(data, spacing, affine, pts, grid, board_size, max_thickness, min_rect, slab_mm,
                   pitch_mm, min_contrast, dense_thr, audit=None):
    """Measure one flat dense object as a sensor board (helper for find_arrays).
    Returns (record, None) for a board with a sensor grid, or (None, reason).
    Optional audit records only elapsed operation times and aggregate counts.
    """
    # face-on pictures are sampled at 0.25 mm pixels
    step = 0.25
    if len(pts) > 200000:                                           # plenty for a plane fit
        pts = pts[np.random.default_rng(0).choice(len(pts), 200000, replace=False)]
    # first plane, from the dense piece: c = center, u, v = in-plane axes, n = normal
    c, u, v, n = _timed_board_operation(audit, "plane_fits", _fit_plane, pts)
    near = np.abs((pts - c) @ n) <= 3.0
    pu, pv = (pts[near] - c) @ u, (pts[near] - c) @ v
    # far bigger than any board: stop early
    if pu.max() - pu.min() > 1.5 * board_size[1]:
        return None, f"too large ({pu.max() - pu.min():.0f} mm)"
    # 1) face-on slab around the plane (wide enough for a whole board even if only a piece of
    #    it was dense enough to be found) and the board's footprint in it. The plane came from
    #    that piece only, so it is refitted to the dense voxels of the whole footprint, twice,
    #    with the slab narrowing as the plane settles.
    m = 0.5 * board_size[1] + 5
    u_rng, v_rng = (pu.min() - m, pu.max() + m), (pv.min() - m, pv.max() + m)
    passes = (slab_mm + 3, slab_mm + 1.5, slab_mm)
    for k_pass, half_t in enumerate(passes):
        stack, gu, gv, gt = _timed_board_operation(
            audit, "volume_reslices", _reslice,
            data, spacing, c, u, v, n, u_rng, v_rng, (-half_t, half_t), step)
        mean = stack.mean(0)
        on = np.abs((pts - c) @ n) <= half_t
        # seed = the pixels the dense piece itself projects onto
        seed = np.zeros(mean.shape, bool)
        seed[np.clip(np.rint(((pts[on] - c) @ u - gu[0]) / step).astype(int), 0, len(gu) - 1),
             np.clip(np.rint(((pts[on] - c) @ v - gv[0]) / step).astype(int), 0, len(gv) - 1)] = True
        fp, plate, air, cut = _timed_board_operation(
            audit, "board_footprints", _footprint, mean, seed, step)
        if fp.sum() < 50:
            return None, "no flat footprint"
        if k_pass == len(passes) - 1:
            break                                                  # plane settled; measure in this frame
        # refit the plane to the whole board: its dense voxels inside the footprint
        fu, fv = np.argwhere(fp).T
        tt, kk = np.nonzero(stack[:, fu, fv] > dense_thr)
        if len(tt) < 100:
            break
        if len(tt) > 200000:
            pick = np.random.default_rng(0).choice(len(tt), 200000, replace=False)
            tt, kk = tt[pick], kk[pick]
        P = c + gu[fu[kk], None] * u + gv[fv[kk], None] * v + gt[tt, None] * n
        c, u, v, n = _timed_board_operation(
            audit, "plane_fits", _fit_plane, P, keep_mm=2.0)
        pu, pv = (P - c) @ u, (P - c) @ v
        u_rng, v_rng = (pu.min() - 15, pu.max() + 15), (pv.min() - 15, pv.max() + 15)
        pts = P
    # footprint pixels -> mm (u, v) -> the smallest rectangle around them
    fp_pts = np.argwhere(fp) * step + np.array([gu[0], gv[0]])
    rc, sides, R = _timed_board_operation(
        audit, "minimum_area_rectangles", _min_area_rect, fp_pts)
    sides = sides + step
    # rectangularity: share of that rectangle the footprint fills
    rect = fp.sum() * step ** 2 / np.prod(sides)
    if not board_size[0] <= sides[1] <= sides[0] <= board_size[1]:
        return None, f"not board-sized ({sides[0]:.0f} x {sides[1]:.0f} mm)"
    if rect < min_rect:
        return None, f"not rectangular (fills {rect:.0%} of its rectangle)"
    # board axes in voxel-axis mm: e1 along the long side, e2 along the short side, e3 normal
    e1 = R[0, 0] * u + R[0, 1] * v
    e2 = R[1, 0] * u + R[1, 1] * v
    e3 = np.cross(e1, e2)
    center = c + rc[0] * u + rc[1] * v
    # 2) re-slice aligned with the rectangle (this is also the face-on picture)
    hu, hv = sides[0] / 2 + 6, sides[1] / 2 + 6
    stack, gu, gv, gt = _timed_board_operation(
        audit, "volume_reslices", _reslice,
        data, spacing, center, e1, e2, e3, (-hu, hu), (-hv, hv), (-slab_mm, slab_mm), step)
    mean, mip = stack.mean(0), stack.max(0)
    U, V = np.meshgrid(gu, gv, indexing="ij")
    in_rect = (np.abs(U) <= sides[0] / 2) & (np.abs(V) <= sides[1] / 2)
    # (same threshold as the last pass: this picture is nearly all board, too little
    #  surroundings for a fresh split)
    fp, plate, air, _ = _timed_board_operation(
        audit, "board_footprints", _footprint,
        mean, in_rect & (mean > cut), step, cut)
    # thickness: half-maximum width of the depth profile over the board
    prof = stack[:, fp].mean(1)
    half = 0.5 * (prof.max() + prof.min())
    above = np.flatnonzero(prof >= half)
    thickness = (above[-1] - above[0] + 1) * step if len(above) else 0.0
    t_mid = float(np.sum(gt * np.clip(prof - half, 0, None)) / max(np.sum(np.clip(prof - half, 0, None)), 1e-6))
    if thickness > max_thickness:
        return None, f"too thick ({thickness:.1f} mm)"
    # 3) sensor grid
    # chips must lie at least 1 mm inside the board's edge
    inside = ndi.binary_erosion(fp, structure=_disk(1.0 / step))
    g = _timed_board_operation(
        audit, "sensor_grid_searches", _find_grid,
        mean, inside, grid, step, pitch_mm)
    if g is None:
        return None, "no room for a sensor grid"
    score, g_c, g_p, (n_v, n_u) = g
    contrast_needed = min_contrast if min_contrast is not None else 0.10 * (plate - air)
    if score < contrast_needed:
        return None, f"no sensor grid (contrast {score:.0f} < {contrast_needed:.0f})"
    # refine every chip: in-plane centroid of its brightness above the board, then its depth
    r_px = 0.45 * g_p.min()
    yy, xx = np.ogrid[:mean.shape[0], :mean.shape[1]]
    sensors_uvt = []
    for b in np.arange(n_v) - (n_v - 1) / 2:
        for a in np.arange(n_u) - (n_u - 1) / 2:
            p0 = g_c + np.array([a * g_p[0], b * g_p[1]])
            disk = (yy - p0[0]) ** 2 + (xx - p0[1]) ** 2 <= r_px ** 2
            w = np.where(disk, mean - plate, 0).clip(min=0)
            pc = np.array(ndi.center_of_mass(w)) if w.sum() > 0 else p0
            dprof = stack[:, disk].mean(1)
            dw = np.clip(dprof - 0.5 * (dprof.max() + dprof.min()), 0, None)
            t = float(np.sum(gt * dw) / max(dw.sum(), 1e-6))
            sensors_uvt.append([gu[0] + pc[0] * step, gv[0] + pc[1] * step, t])
    sensors_uvt = np.array(sensors_uvt)
    # how regular is the grid: fit an ideal lattice (center + a*col step + b*row step) to the
    # chip positions in the board plane; RMS distance of the chips from it
    ab = np.array([(a, b) for b in np.arange(n_v) - (n_v - 1) / 2 for a in np.arange(n_u) - (n_u - 1) / 2])
    M = np.c_[np.ones(len(ab)), ab]
    fit = np.linalg.lstsq(M, sensors_uvt[:, :2], rcond=None)[0]
    grid_rms = float(np.sqrt(np.mean(np.sum((M @ fit - sensors_uvt[:, :2]) ** 2, axis=1))))
    # orient the normal toward the chip side (chips sit on top of the board)
    if np.mean(sensors_uvt[:, 2]) < t_mid:
        e2, e3 = -e2, -e3
        sensors_uvt[:, 1:] *= -1
        t_mid = -t_mid
        mean, mip, fp, stack = mean[:, ::-1], mip[:, ::-1], fp[:, ::-1], stack[::-1, :, ::-1]
        gv, gt = -gv[::-1], -gt[::-1]
    # board frame (u, v, t) -> voxel-axis mm
    to_mm = lambda q: center + q[..., :1] * e1 + q[..., 1:2] * e2 + q[..., 2:3] * e3
    sensors_mm = to_mm(sensors_uvt)
    board_center_mm = center + t_mid * e3
    # chip block: rectangle around the chip centers, half a pitch beyond them
    pu_mm, pv_mm = g_p * step
    blk = (sensors_uvt[:, 0].min() - pu_mm / 2, sensors_uvt[:, 0].max() + pu_mm / 2,
           sensors_uvt[:, 1].min() - pv_mm / 2, sensors_uvt[:, 1].max() + pv_mm / 2)
    rec = dict(kind="array", center_mm=sensors_mm.mean(0), board_center_mm=board_center_mm, e1=e1, e2=e2, e3=e3,
               dims=np.array([sides[0], sides[1], thickness]), rectangularity=rect, grid_contrast=score,
               grid_rms_mm=grid_rms, depth_sd_mm=float(np.std(sensors_uvt[:, 2])),
               grid_shape=(n_v, n_u), pitch_mm=np.array([pu_mm, pv_mm]), sensors_mm=sensors_mm,
               sensors_uvt=sensors_uvt, block_uv=blk, face=dict(mean=mean, mip=mip, fp=fp, gu=gu, gv=gv, step=step),
               t_half=thickness / 2, t_mid=t_mid, board_level=plate, spacing=spacing)
    # patient coordinates
    to_lps = lambda p_mm: voxel_to_lps(affine, p_mm / spacing)
    rec["center_voxel"] = rec["center_mm"] / spacing
    rec["center_lps"] = to_lps(rec["center_mm"])
    rec["board_center_lps"] = to_lps(board_center_mm)
    rec["sensors_lps"] = np.array([to_lps(p) for p in sensors_mm])
    for key in ("e1", "e2", "e3"):
        rec[key + "_lps"] = dir_to_lps(affine, rec[key])
    rec["normal_lps"] = rec["e3_lps"]
    rec["mask"], rec["window"], rec["block_mask"] = _timed_board_operation(
        audit, "board_3d_masks", _board_masks, rec, spacing, data.shape)
    return rec, None


def _board_masks(rec, spacing, shape):
    """3D masks of the board (its footprint, board thickness) and of the chip block, in a
    window around the board. Returns (board mask, window, chip-block mask)."""
    f = rec["face"]
    corners = np.array([[a, b, t] for a in (f["gu"][0], f["gu"][-1]) for b in (f["gv"][0], f["gv"][-1])
                        for t in (rec["t_mid"] - rec["t_half"] - 1, rec["t_mid"] + rec["t_half"] + 1)])
    corners_mm = rec["board_center_mm"] - rec["t_mid"] * rec["e3"] + corners @ np.array([rec["e1"], rec["e2"], rec["e3"]])
    lo = np.maximum(np.floor(corners_mm.min(0) / spacing).astype(int) - 1, 0)
    hi = np.minimum(np.ceil(corners_mm.max(0) / spacing).astype(int) + 2, shape)
    idx = np.indices(hi - lo).reshape(3, -1).T + lo
    d = idx * spacing - (rec["board_center_mm"] - rec["t_mid"] * rec["e3"])
    u, v, t = d @ rec["e1"], d @ rec["e2"], d @ rec["e3"]
    iu = np.rint((u - f["gu"][0]) / f["step"]).astype(int)
    iv = np.rint((v - f["gv"][0]) / f["step"]).astype(int)
    ok = (iu >= 0) & (iu < len(f["gu"])) & (iv >= 0) & (iv < len(f["gv"]))
    on_fp = np.zeros(len(idx), bool)
    on_fp[ok] = f["fp"][iu[ok], iv[ok]]
    in_t = np.abs(t - rec["t_mid"]) <= max(rec["t_half"], 0.75)
    board = (on_fp & in_t).reshape(hi - lo)
    b = rec["block_uv"]
    block = ((u >= b[0]) & (u <= b[1]) & (v >= b[2]) & (v <= b[3]) & in_t).reshape(hi - lo)
    return board, tuple(zip(lo.tolist(), hi.tolist())), block


def find_arrays(image, exclude=(), expected=None, grid=(4, 4), board_threshold=None, board_level=0.35,
                board_size=(15.0, 120.0), max_thickness=8.0, min_rectangularity=0.85, slab_mm=3.0,
                pitch_mm=(2.0, 10.0), min_contrast=None, verbose=True):
    """Find flat sensor boards and the grid of sensor chips on each.

    Args:
      image              TorchIO ScalarImage
      exclude            objects to remove first (magnet candidates), so a magnet held near a
                         board never merges with it
      expected           how many arrays to keep (strongest grids first); None or 0 = all
      grid               (rows, cols) of sensor chips on each board
      board_threshold    dense-material threshold for finding boards; None = automatic
                         (board_level of the way from the 99.5th percentile to the maximum)
      board_size         (min, max) side length of a board in mm
      max_thickness      board + components must be thinner than this (mm)
      min_rectangularity footprint area / area of its bounding rectangle
      slab_mm            half-thickness of the face-on slab used to see the board (mm)
      pitch_mm           (min, max) chip spacing searched (mm)
      min_contrast       chips must be at least this much brighter than the spaces between
                         them (scan units); None = 10% of the board's own brightness above air
      verbose            print what was found and why other flat objects were rejected
    Returns a list of dicts (see _board_record), with mask/window for drawing.
    """
    start_clock = time.perf_counter()
    data = _float_data(image)
    spacing = np.asarray(image.spacing, float)
    smooth = ndi.gaussian_filter(data, sigma=0.5 / spacing)
    smooth_clock = time.perf_counter()
    if board_threshold is None:
        (board_threshold,), _, _ = _auto_levels(smooth, (board_level,))
    threshold_clock = time.perf_counter()
    dense = smooth > board_threshold
    del smooth
    dense_clock = time.perf_counter()
    # Default ON after 7/7 paired CBCT scans passed measured-geometry checks.
    # EDT is used only for voxel spacings with exactly the original 3-mm
    # discrete dilation footprint; all other spacings retain binary dilation.
    # GK_CBCT_MAGNET_EXCLUSION_EDT=0 explicitly restores original dilation.
    fast_exclusion = _magnet_exclusion_edt_enabled()
    exclusion_methods = {}
    exclusion_dilation_seconds = 0.0
    exclusion_application_seconds = 0.0
    excluded_candidate_count = 0
    for m in exclude:                                    # remove magnets (and 3 mm around them)
        win = tuple(slice(a, b) for a, b in m["window"])
        t_dilation = time.perf_counter()
        grown, method = _dilate_magnet_exclusion(
            m["mask"], spacing, use_edt=fast_exclusion)
        t_application = time.perf_counter()
        dense[win] &= ~grown
        t_end = time.perf_counter()
        exclusion_dilation_seconds += t_application - t_dilation
        exclusion_application_seconds += t_end - t_application
        exclusion_methods[method] = exclusion_methods.get(method, 0) + 1
        excluded_candidate_count += 1
    mask_clock = time.perf_counter()
    labels, n = ndi.label(dense)
    del dense
    sizes = np.bincount(labels.ravel())
    label_clock = time.perf_counter()
    if verbose:
        print(f"arrays: board threshold {board_threshold:.0f} -> {n} dense objects")
    # candidate flat objects, largest first
    min_vox = 100.0 / np.prod(spacing)                   # at least 100 mm3 of dense material
    cands = []
    prefilter = {"below_min_voxel_volume": 0, "below_min_board_extent": 0}
    for lab, sl in enumerate(ndi.find_objects(labels), start=1):
        if sl is None:
            continue
        if sizes[lab] < min_vox:
            prefilter["below_min_voxel_volume"] += 1
            continue
        ext = np.array([s.stop - s.start for s in sl]) * spacing
        if ext.max() < board_size[0] or np.sort(ext)[1] < 0.5 * board_size[0]:
            prefilter["below_min_board_extent"] += 1
            continue
        cands.append((sizes[lab], lab, sl))
    cands.sort(key=lambda c: -c[0])
    candidate_clock = time.perf_counter()
    found, rejected = [], []
    suppressed_overlaps = 0
    duplicate_boards = 0
    attempted = 0
    # Aggregate function durations across all measured candidates, including
    # early rejections. No patient voxels or per-object positions in this audit.
    measurement_times = {}
    measurement_calls = {}
    total_measurement_wall_seconds = 0.0
    for _, lab, sl in cands:
        idx = np.argwhere(labels[sl] == lab) + np.array([s.start for s in sl])
        pts = idx * spacing
        if any(_inside_board(p, f) for f in found for p in (pts.mean(0),)):
            suppressed_overlaps += 1
            continue                                     # another piece of a board already found
        attempted += 1
        candidate_audit = {}
        candidate_start = time.perf_counter()
        rec, reason = _measure_board(data, spacing, image.affine, pts, grid, board_size, max_thickness,
                                     min_rectangularity, slab_mm, pitch_mm, min_contrast, board_threshold,
                                     audit=candidate_audit)
        total_measurement_wall_seconds += time.perf_counter() - candidate_start
        for name, elapsed in candidate_audit.get("substage_seconds", {}).items():
            measurement_times[name] = measurement_times.get(name, 0.0) + elapsed
        for name, n_calls in candidate_audit.get("call_counts", {}).items():
            measurement_calls[name] = measurement_calls.get(name, 0) + n_calls
        if rec is None:
            rejected.append((lab, reason, dict(
                center_lps=voxel_to_lps(image.affine, idx.mean(0)),
                candidate_voxels=int(sizes[lab])), 1))
            continue
        if any(np.linalg.norm(rec["board_center_mm"] - f["board_center_mm"]) < 10 for f in found):
            duplicate_boards += 1
            continue
        rec["label"] = lab
        found.append(rec)
    del labels
    measurement_clock = time.perf_counter()
    found.sort(key=lambda r: -r["grid_contrast"])
    if expected:
        for r in found[expected:]:
            rejected.append((None, f"extra candidate (only {expected} expected)", r, 1))
        found = found[:expected]
    name_by_location(found, "array", scan_center_lps(image))
    # Small, JSON-safe detector audit. This preserves *why* board-sized
    # components were rejected for later diagnosis of the missing superior
    # board. It does NOT alter thresholding, candidates, or accepted geometry.
    audit_rejected = []
    for _, reason, rec, _ in rejected:
        if rec is None:
            continue
        location = rec.get("center_lps")
        audit_rejected.append({
            "reason": str(reason),
            "center_lps_mm": [float(v) for v in location] if location is not None else None,
            "candidate_voxels": int(rec.get("candidate_voxels", 0)),
        })
    find_arrays.last_audit = {
        "board_threshold": float(board_threshold),
        "connected_components": int(n),
        "prefilter": prefilter,
        "candidate_components": len(cands),
        "attempted_board_measurements": attempted,
        "board_measurement_profile": {
            # Named calls are subparts of _measure_board total. The residual
            # contains local arithmetic, chip refinement and timing overhead.
            "timing_seconds": {
                **{name: round(elapsed, 3)
                   for name, elapsed in measurement_times.items()},
                "other_measurement_processing": round(
                    max(0.0, total_measurement_wall_seconds -
                        sum(measurement_times.values())), 3),
                "measured_function_calls_total": round(
                    total_measurement_wall_seconds, 3),
            },
            "call_counts": measurement_calls,
        },
        "suppressed_overlaps": suppressed_overlaps,
        "duplicate_boards": duplicate_boards,
        "accepted_arrays": [
            {"name": a["name"],
             "center_lps_mm": [float(v) for v in a["center_lps"]],
             "grid_rms_mm": float(a["grid_rms_mm"])}
            for a in found
        ],
        "rejected_candidates": audit_rejected[:120],
        "rejected_candidate_count": len(audit_rejected),
        "magnet_exclusion": {
            "candidate_masks_processed": excluded_candidate_count,
            "method_counts": exclusion_methods,
            "experiment_enabled": fast_exclusion,
        },
        "timing_seconds": {
            "threshold_mask_and_exclusion": round(mask_clock-start_clock,3),
            # Subphases of the existing total above; do not add them again.
            "initial_setup_and_gaussian_smoothing": round(smooth_clock-start_clock,3),
            "automatic_threshold_estimation": round(threshold_clock-smooth_clock,3),
            "mask_creation_and_magnet_exclusion": round(mask_clock-threshold_clock,3),
            "dense_mask_allocation": round(dense_clock-threshold_clock,3),
            "magnet_exclusion_dilation": round(exclusion_dilation_seconds,3),
            "magnet_exclusion_mask_application": round(exclusion_application_seconds,3),
            "connected_component_labeling": round(label_clock-mask_clock,3),
            "candidate_prefilter": round(candidate_clock-label_clock,3),
            "candidate_board_measurement": round(measurement_clock-candidate_clock,3),
            "total": round(time.perf_counter()-start_clock,3),
        },
    }
    if verbose:
        print(f"\narrays found: {len(found)}")
        for r in found:
            print(f"  {r['name']}: array center LPS mm {np.round(r['center_lps'], 2)} | board center "
                  f"{np.round(r['board_center_lps'], 2)} | board {r['dims'][0]:.1f} x {r['dims'][1]:.1f} x "
                  f"{r['dims'][2]:.1f} mm | {len(r['sensors_lps'])} sensors, pitch "
                  f"{r['pitch_mm'][0]:.2f} x {r['pitch_mm'][1]:.2f} mm, chips within {r['grid_rms_mm']:.2f} mm RMS of "
                  f"a regular grid | faces LPS {np.round(r['normal_lps'], 2)}")
        _print_rejected(rejected, "sensor-board")
    return found


def pick_magnets(candidates, rejected, arrays=(), expected=None, ref_lps=None, verbose=True):
    """Second pass of the magnet search: drop candidates that sit on a sensor board (chips,
    connectors), then keep the expected number, largest dense core first.
    A magnet resting on a board's surface is kept: its center lies outside the board's slab.
    Magnets are named after their direction from ref_lps (default: the first array's center or
    the origin); run() passes the middle of the scan."""
    magnets = []
    for c in candidates:
        board = next((a for a in arrays if _on_board(c["center_voxel"] * a["spacing"], a)), None)
        if board is not None:
            rejected.append((None, f"part of sensor board {board['name']}", c, c["level"]))
        else:
            magnets.append(c)
    magnets.sort(key=lambda r: -r["core_mm3"])
    if expected:
        for r in magnets[expected:]:
            rejected.append((None, f"extra candidate (only {expected} expected; core {r['core_mm3']:.0f} mm3)",
                             r, r["level"]))
        magnets = magnets[:expected]
    name_by_location(magnets, "magnet", np.zeros(3) if ref_lps is None else ref_lps)
    if verbose:
        print(f"\nmagnets found: {len(magnets)}")
        for r in magnets:
            print(f"  {r['name']}: center LPS mm {np.round(r['center_lps'], 2)} | "
                  f"{r['dims'][0]:.1f} x {r['dims'][1]:.1f} x {r['dims'][2]:.1f} mm | long axis LPS "
                  f"{np.round(r['axis_lps'], 2)} | dense core {r['core_mm3']:.0f} mm3")
        _print_rejected(rejected, "magnet")
    return magnets


def find_magnets(image, arrays=(), expected=None, **kwargs):
    """Find magnets in one call: magnet_candidates, then pick_magnets (removing candidates
    that sit on the given sensor boards). kwargs go to magnet_candidates."""
    verbose = kwargs.get("verbose", True)
    cands, rejected = magnet_candidates(image, **kwargs)
    return pick_magnets(cands, rejected, arrays, expected, scan_center_lps(image), verbose)


def magnet_steps(image, rec, seed_threshold, max_bloom_mm=2.0):
    """Repeat the outline steps of _magnet_level for one magnet (in its window) and return
    the mask after each step, plus the window's voxel values."""
    data, spacing = _float_data(image), np.asarray(image.spacing, float)
    win = tuple(slice(a, b) for a, b in rec["window"])
    local = data[win]
    lab, _ = ndi.label(ndi.gaussian_filter(local, 0.3 / spacing) > seed_threshold)
    seed = lab == np.bincount(lab[rec["mask"] & (lab > 0)]).argmax()     # the seed piece under the outline
    depth = ndi.distance_transform_edt(seed, sampling=spacing)
    core = seed
    if 0.5 * depth.max() >= 2 * spacing.min():                            # flare trimming, as in step 2
        opened = ndi.binary_opening(seed, structure=_ball(0.5 * depth.max(), spacing))
        lab_o, n_o = ndi.label(opened)
        if n_o:
            core = lab_o == lab_o[np.unravel_index(np.argmax(np.where(opened, depth, 0)), seed.shape)]
    limit = ndi.distance_transform_edt(~core, sampling=spacing) <= max_bloom_mm
    return local, dict(seed=seed, core=core, limit=limit, final=rec["mask"])


def measure_round(mask, spacing):
    """Size and shape of one object from its outline mask (as in find_bb.py).
    Returns equivalent diameter, principal-axis diameters, elongation and surface CV."""
    pts = np.argwhere(mask) * spacing                      # voxel positions, mm
    center = pts.mean(0)
    volume = len(pts) * np.prod(spacing)
    eq_diam = (6 * volume / np.pi) ** (1 / 3)               # diameter of a sphere of the same volume
    # a solid sphere of radius r has variance r^2/5 along every axis, so 2*sqrt(5*variance)
    # recovers its diameter along each principal axis
    eig = np.clip(np.linalg.eigvalsh(np.cov(pts.T)), 0, None) if len(pts) > 3 else np.zeros(3)
    axes = 2 * np.sqrt(5 * eig)
    elongation = axes.max() / max(axes.min(), 1e-6)
    # roundness: every surface voxel should be the same distance from the center
    surface = mask & ~ndi.binary_erosion(mask)
    dist = np.linalg.norm(np.argwhere(surface) * spacing - center, axis=1)
    surface_cv = float(dist.std() / dist.mean()) if len(dist) else np.inf
    return dict(eq_diam=float(eq_diam), dims=np.sort(axes)[::-1], elongation=float(elongation),
                surface_cv=surface_cv)


def _bb_level(data, labels, spacing, affine, diameter, tolerance, max_elongation, max_surface_cv,
              found, rejected, level):
    """Measure every bright object at one seed threshold (helper for find_glass_bbs)."""
    pad = np.ceil((diameter[1] / 2 + 4) / spacing).astype(int)    # window margin in voxels
    lo_d, hi_d = diameter[0] - tolerance, diameter[1] + tolerance
    done = set()
    for lab, sl in enumerate(ndi.find_objects(labels), start=1):
        if sl is None or lab in done:
            continue
        # skip anything far too big to be a BB (boards, holders, the head shell)
        extent = np.array([s.stop - s.start for s in sl]) * spacing
        if extent.max() > 3 * hi_d:
            rejected.append((lab, f"too large ({extent.max():.0f} mm across)", None, level))
            continue
        # work in a window around the object (its box plus a margin)
        win = tuple(slice(max(s.start - p, 0), min(s.stop + p, n_)) for s, p, n_ in zip(sl, pad, data.shape))
        local = data[win]
        core = labels[win] == lab
        # peak = typical value in the middle of the object
        core_pts = np.argwhere(core) * spacing
        r_core = np.linalg.norm(core_pts - core_pts.mean(0), axis=1)
        middle = r_core <= 0.5 * r_core.max() + 1e-6
        peak = float(np.median(local[core][middle])) if middle.any() else float(local[core].max())
        # half-maximum outline; background measured in a shell 1.0-2.5 mm outside the outline,
        # repeated because the shell moves as the outline grows
        mask = core
        for _ in range(3):
            dist = ndi.distance_transform_edt(~mask, sampling=spacing)
            ring = (dist > 1.0) & (dist <= 2.5)
            background = float(np.median(local[ring])) if ring.any() else float(np.median(local))
            hm_labels, _ = ndi.label(local > 0.5 * (peak + background))
            keep = np.unique(hm_labels[core & (hm_labels > 0)])
            mask = np.isin(hm_labels, keep[keep > 0])
        # clean-up with a 0.6 mm ball: opening trims spikes and cuts thin bridges to whatever
        # the BB touches (its post); keep the pieces on this object's core; closing and hole
        # filling bridge slits
        ball = _ball(0.6, spacing)
        op_labels, n_op = ndi.label(ndi.binary_opening(mask, structure=ball))
        if n_op:
            overlap = ndi.sum(core, op_labels, index=np.arange(1, n_op + 1))
            mask = np.isin(op_labels, 1 + np.flatnonzero(overlap > 0))
        mask = ndi.binary_fill_holes(ndi.binary_closing(mask, structure=ball))
        done.update(np.unique(labels[win][mask]).tolist())
        if mask.sum() < 8:
            rejected.append((lab, "too small", None, level))
            continue
        m = measure_round(mask, spacing)
        # the tests; every failed one is recorded
        reasons = []
        if not lo_d <= m["eq_diam"] <= hi_d:
            reasons.append(f"diameter {m['eq_diam']:.2f} mm")
        if m["elongation"] > max_elongation:
            reasons.append(f"elongated {m['elongation']:.2f}")
        # voxel blockiness makes even a perfect sphere's surface uneven, more so when it is
        # only a few voxels across, so the roundness limit loosens with voxel size / radius
        cv_limit = max_surface_cv or 0.04 + 0.5 * spacing.max() / (m["eq_diam"] / 2)
        if m["surface_cv"] > cv_limit:
            reasons.append(f"not round (surface CV {m['surface_cv']:.3f})")
        # center: every voxel inside the outline, weighted by how far it is above the background
        w = np.where(mask, local - background, 0).clip(min=0)
        c_vox = np.array(ndi.center_of_mass(w)) + np.array([s.start for s in win])
        c_lps = voxel_to_lps(affine, c_vox)
        if any(np.linalg.norm(c_lps - f["center_lps"]) < 1.0 for f in found):
            continue                                    # same BB, seen at another level
        rec = dict(kind="bb", label=lab, level=level, center_voxel=c_vox, center_lps=c_lps, peak=peak,
                   background=background, mask=mask, window=tuple((s.start, s.stop) for s in win), **m)
        (rejected.append((lab, ", ".join(reasons), rec, level)) if reasons else found.append(rec))


def _in_object(c_vox, obj, spacing, margin_mm=2.0):
    """Is the voxel point within margin_mm of the object's outline (mask + window)?"""
    lo = np.array([w[0] for w in obj["window"]])
    i = np.rint(np.asarray(c_vox) - lo).astype(int)
    k = np.ceil(margin_mm / spacing).astype(int)
    sl = tuple(slice(max(a - b, 0), max(min(a + b + 1, n), 0)) for a, b, n in zip(i, k, obj["mask"].shape))
    return bool(obj["mask"][sl].any())


def find_glass_bbs(image, metal=(), arrays=(), expected=None, diameter=(4.0, 8.0), tolerance=1.0,
                   levels=(0.5, 0.3, 0.15), max_elongation=1.25, max_surface_cv=None, verbose=True):
    """Find BBs (round, solid spheres) such as a glass BB, after the magnets and boards.

    Args:
      image           TorchIO ScalarImage
      metal           objects that are not BBs (the magnet candidates, which include dense board
                      parts); a round object within 2 mm of one of their outlines is skipped
      arrays          sensor boards; a round object inside a board is skipped
      expected        how many BBs to keep (roundest first); None or 0 = all
      diameter        (min, max) BB diameter in mm, with `tolerance` of slack each way
      levels          seed thresholds, as fractions of the way from the brightest ordinary
                      material (99.5th percentile) to the brightest voxel
      max_elongation  longest / shortest principal-axis diameter must be at most this
      max_surface_cv  roundness limit (spread of center-to-surface distances / their mean);
                      None = automatic, looser for BBs only a few voxels across
    Returns a list of dicts like the magnets' (center, size, mask, window), kind "bb".
    """
    data = _float_data(image)
    spacing = np.asarray(image.spacing, float)
    smooth = ndi.gaussian_filter(data, sigma=0.5 / spacing)        # for the seeds only
    thresholds, hi_anat, top = _auto_levels(smooth, levels)
    if verbose:
        print(f"BBs: brightest ordinary material {hi_anat:.0f}, brightest voxel {top:.0f}")
    found, rejected = [], []
    for level, thr in enumerate(thresholds, start=1):
        labels, n = ndi.label(smooth > thr)
        if verbose:
            print(f"  level {level}: threshold {thr:.0f} -> {n} bright objects")
        _bb_level(data, labels, spacing, image.affine, diameter, tolerance, max_elongation, max_surface_cv,
                  found, rejected, level)
        del labels
    del smooth
    # round objects that belong to a magnet (or other dense metal part) or to a board are not BBs
    bbs = []
    for r in found:
        if any(_in_object(r["center_voxel"], m, spacing) for m in metal):
            rejected.append((None, "part of a magnet or other metal object", r, r["level"]))
            continue
        board = next((a for a in arrays if _on_board(r["center_voxel"] * spacing, a)), None)
        if board is not None:
            rejected.append((None, f"part of sensor board {board['name']}", r, r["level"]))
            continue
        bbs.append(r)
    bbs.sort(key=lambda r: r["surface_cv"])                         # roundest first
    if expected:
        for r in bbs[expected:]:
            rejected.append((None, f"extra candidate (only {expected} expected)", r, r["level"]))
        bbs = bbs[:expected]
    name_by_location(bbs, "glass_bb", scan_center_lps(image))
    if verbose:
        print(f"\nBBs found: {len(bbs)}")
        for r in bbs:
            print(f"  {r['name']}: center LPS mm {np.round(r['center_lps'], 2)} | diameter {r['eq_diam']:.2f} mm "
                  f"(axes {np.round(r['dims'], 2)}) | surface CV {r['surface_cv']:.3f} | peak {r['peak']:.0f}")
        _print_rejected(rejected, "BB")
    return bbs


def pair_magnets(objects):
    """Record, for every magnet, the nearest array and the distance between their centers."""
    arrays = [o for o in objects if o["kind"] == "array"]
    for m in (o for o in objects if o["kind"] == "magnet"):
        if arrays:
            dist = [np.linalg.norm(m["center_lps"] - a["center_lps"]) for a in arrays]
            m["nearest_array"] = arrays[int(np.argmin(dist))]["name"]
            m["nearest_array_mm"] = float(min(dist))


def oriented_slice(vol, obj, axis, orientation):
    """Slice of `vol` through the object's center, perpendicular to voxel `axis`, arranged for
    viewing. Returns (gray image, the object's outline mask in that slice, plane name)."""
    idx = int(np.clip(np.rint(obj["center_voxel"][axis]), 0, vol.shape[axis] - 1))
    p, q, flip_c, flip_r = _display_axes(axis, orientation)
    gray = np.take(vol, idx, axis=axis).T                         # rows follow q, columns follow p
    # the object's mask lives in a small window: paste its slice into a full-size image
    mask = np.zeros((vol.shape[p], vol.shape[q]), bool)
    lo = [w[0] for w in obj["window"]]
    li = idx - lo[axis]
    if 0 <= li < obj["mask"].shape[axis]:
        part = np.take(obj["mask"], li, axis=axis)
        mask[lo[p]:lo[p] + part.shape[0], lo[q]:lo[q] + part.shape[1]] = part
    mask = mask.T
    if flip_c:
        gray, mask = gray[:, ::-1], mask[:, ::-1]
    if flip_r:
        gray, mask = gray[::-1], mask[::-1]
    return gray, mask, _PLANE[orientation[axis]]


def to_display(point_vox, axis, shape, orientation):
    """(row, col) of a voxel-space point in the image made by oriented_slice."""
    p, q, flip_c, flip_r = _display_axes(axis, orientation)
    row, col = point_vox[q], point_vox[p]
    if flip_c:
        col = shape[p] - 1 - col
    if flip_r:
        row = shape[q] - 1 - row
    return row, col


def burn_in(gray, masks, dots, window, scale=3):
    """Edited image: the slice as an RGB picture (enlarged `scale` times) with outlines
    (2 pixels wide) and dots painted into the pixels.
      masks  list of (2D mask, color)       dots  list of ((row, col), radius in pixels, color)"""
    # gray values -> 0-255 through the display window, enlarged `scale` times
    g = np.clip((gray - window[0]) / (window[1] - window[0]), 0, 1)
    g = np.repeat(np.repeat((g * 255).astype(np.uint8), scale, 0), scale, 1)
    rgb = np.repeat(g[..., None], 3, axis=2)
    for mask, color in masks:
        m = np.repeat(np.repeat(mask, scale, 0), scale, 1)
        # the outline = the mask's edge, 2 pixels wide
        rgb[m & ~ndi.binary_erosion(m, iterations=2)] = color
    rr, cc = np.ogrid[:rgb.shape[0], :rgb.shape[1]]
    for (r, c), radius, color in dots:
        r0, c0 = (np.asarray([r, c], float) + 0.5) * scale - 0.5
        rgb[(rr - r0) ** 2 + (cc - c0) ** 2 <= radius ** 2] = color
    return rgb


def annotate_object(vol, obj, orientation, spacing, out_dir, window=(-1000, 4000), zoom_mm=None):
    """Outline and center of one object in the axial, coronal and sagittal planes through its
    center, painted green into full-slice images <out_dir>/<name>_<plane>.png, plus a close-up
    of all three (<name>_zoom.png; zoom_mm across, default: a little more than the object).
    Returns {"perimeter_<plane>_mm": length, ...}."""
    orientation = tuple(orientation)
    if zoom_mm is None:
        zoom_mm = max(24.0, 1.6 * obj["dims"][0]) if obj["kind"] in ("magnet", "bb") else obj["dims"][0] + 12
    order = ["axial", "coronal", "sagittal"]
    planes = sorted(range(3), key=lambda a: order.index(_PLANE[orientation[a]]))
    result = {}
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.6))
    for col, axis in enumerate(planes):
        gray, mask, plane = oriented_slice(vol, obj, axis, orientation)
        p, q, _, _ = _display_axes(axis, orientation)
        r0, c0 = to_display(obj["center_voxel"], axis, vol.shape, orientation)
        paths = outline_paths(mask)
        perim = sum(float(np.hypot(np.diff(t[:, 0]) * spacing[q], np.diff(t[:, 1]) * spacing[p]).sum()) for t in paths)
        result[f"perimeter_{plane}_mm"] = round(perim, 2)
        plt.imsave(out_dir / f"{obj['name']}_{plane}.png", burn_in(gray, [(mask, GREEN)], [((r0, c0), 3, GREEN)], window))
        hr, hc = int(round(zoom_mm / 2 / spacing[q])), int(round(zoom_mm / 2 / spacing[p]))
        r_lo, c_lo = max(int(r0) - hr, 0), max(int(c0) - hc, 0)
        crop = gray[r_lo:int(r0) + hr, c_lo:int(c0) + hc]
        a = axs[col]
        a.imshow(crop, cmap="gray", vmin=window[0], vmax=window[1])
        for t in paths:
            a.plot(t[:, 1] - c_lo, t[:, 0] - r_lo, color="lime", lw=1.5)
        a.plot(c0 - c_lo, r0 - r_lo, "o", color="lime", ms=6)
        a.set_xlim(-0.5, crop.shape[1] - 0.5)
        a.set_ylim(crop.shape[0] - 0.5, -0.5)
        a.set_title(f"{plane}: perimeter {perim:.1f} mm", fontsize=10)
        a.axis("off")
    what = {"magnet": "magnet", "bb": "glass BB"}.get(obj["kind"], "array (outline = board, dot = sensor-grid center)")
    fig.suptitle(f"{obj['name']} ({obj.get('location', '')}): {what}, center LPS "
                 f"{np.round(obj['center_lps'], 2)} mm", fontsize=10)
    fig.tight_layout()
    fig.savefig(out_dir / f"{obj['name']}_zoom.png", dpi=100)
    plt.close(fig)
    return result


def draw_board(obj, out_dir, window=(-500, 5000), scale=3):
    """Face-on picture of a sensor board (a thin slab through the board, brightest values):
    green = board outline, chip-block outline and array center; yellow = each chip and the
    board's own center. Saved as <out_dir>/<name>_board.png (painted pixels, no legend) and
    <name>_board_annotated.png (with legend and the patient directions)."""
    f = obj["face"]
    img = f["mip"].T[::-1]                                           # rows = v (up), columns = u
    fp = f["fp"].T[::-1]
    step, gu, gv = f["step"], f["gu"], f["gv"]
    to_rc = lambda uv: (len(gv) - 1 - (uv[1] - gv[0]) / step, (uv[0] - gu[0]) / step)
    U, V = np.meshgrid(gu, gv[::-1])
    b = obj["block_uv"]
    block = (U >= b[0]) & (U <= b[1]) & (V >= b[2]) & (V <= b[3])
    sens = [(to_rc(s[:2]), scale * 0.9, YELLOW) for s in obj["sensors_uvt"]]
    bc = obj["board_center_mm"] - obj["center_mm"]                  # board center in the picture
    centre_uv = obj["sensors_uvt"][:, :2].mean(0)
    board_uv = centre_uv + np.array([bc @ obj["e1"], bc @ obj["e2"]])
    dots = sens + [(to_rc(board_uv), scale * 1.6, YELLOW), (to_rc(centre_uv), scale * 2.2, GREEN)]
    rgb = burn_in(img, [(fp, GREEN), (block, GREEN)], dots, window, scale)
    plt.imsave(out_dir / f"{obj['name']}_board.png", rgb)
    # annotated version, with the patient directions of the picture's axes
    fig, a = plt.subplots(figsize=(8, 8 * img.shape[0] / img.shape[1] + 0.8))
    a.imshow(img, cmap="gray", vmin=window[0], vmax=window[1])
    for t in outline_paths(fp):
        a.plot(t[:, 1], t[:, 0], color="lime", lw=1.5)
    for t in outline_paths(block):
        a.plot(t[:, 1], t[:, 0], color="lime", lw=1, ls="--")
    for s in obj["sensors_uvt"]:
        r, c = to_rc(s[:2])
        a.plot(c, r, ".", color="gold", ms=6)
    r, c = to_rc(board_uv)
    a.plot(c, r, "x", color="gold", ms=9, mew=2, label=f"board center {np.round(obj['board_center_lps'], 1)}")
    r, c = to_rc(centre_uv)
    a.plot(c, r, "o", color="lime", ms=8, label=f"array center {np.round(obj['center_lps'], 1)}")
    a.legend(loc="lower right", fontsize=8, framealpha=0.8)
    a.set_title(f"{obj['name']} ({obj.get('location', '')}), seen face-on from the chip side\n"
                f"picture right = toward {direction_name(obj['e1_lps'])}, picture up = toward "
                f"{direction_name(obj['e2_lps'])}; LPS mm", fontsize=9)
    a.axis("off")
    fig.tight_layout()
    fig.savefig(out_dir / f"{obj['name']}_board_annotated.png", dpi=100)
    plt.close(fig)


def draw_overview(vol, objects, orientation, spacing, out_dir, window=(-1000, 4000)):
    """Maximum-intensity projections of the whole scan (axial, coronal, sagittal) with every
    object's outline and center in green and its name, saved as <out_dir>/overview.png."""
    orientation = tuple(orientation)
    order = ["axial", "coronal", "sagittal"]
    planes = sorted(range(3), key=lambda a: order.index(_PLANE[orientation[a]]))
    fig, axs = plt.subplots(1, 3, figsize=(18, 6.6))
    for a, axis in zip(axs, planes):
        p, q, flip_c, flip_r = _display_axes(axis, orientation)
        img = vol.max(axis=axis).T
        if flip_c:
            img = img[:, ::-1]
        if flip_r:
            img = img[::-1]
        a.imshow(img, cmap="gray", vmin=window[0], vmax=window[1], aspect=spacing[q] / spacing[p])
        for o in objects:
            m = np.zeros((vol.shape[p], vol.shape[q]), bool)
            lo = [w[0] for w in o["window"]]
            part = o["mask"].any(axis=axis)
            m[lo[p]:lo[p] + part.shape[0], lo[q]:lo[q] + part.shape[1]] = part
            m = m.T
            if flip_c:
                m = m[:, ::-1]
            if flip_r:
                m = m[::-1]
            for t in outline_paths(m):
                a.plot(t[:, 1], t[:, 0], color="lime", lw=1)
            r, c = to_display(o["center_voxel"], axis, vol.shape, orientation)
            a.plot(c, r, "o", color="lime", ms=4)
            a.annotate(o["name"], (c, r), xytext={"magnet": (6, 6), "bb": (-8, -4)}.get(o["kind"], (6, -14)),
                       textcoords="offset points", color="lime", fontsize=9,
                       ha="right" if o["kind"] == "bb" else "left")   # BB label left of its dot
        a.set_title(f"{_PLANE[orientation[axis]]} projection (patient's right on the left)", fontsize=10)
        a.axis("off")
    fig.tight_layout()
    fig.savefig(out_dir / "overview.png", dpi=90)
    plt.close(fig)


def write_tables(objects, out_dir):
    """objects.csv (one row per object) and sensors.csv (one row per sensor chip)."""
    rows = []
    for o in objects:
        row = dict(name=o["name"], kind=o["kind"], location=o.get("location", ""),
                   x_mm=o["center_lps"][0], y_mm=o["center_lps"][1], z_mm=o["center_lps"][2],
                   i=o["center_voxel"][0], j=o["center_voxel"][1], k=o["center_voxel"][2],
                   size_1_mm=o["dims"][0], size_2_mm=o["dims"][1], size_3_mm=o["dims"][2])
        if o["kind"] == "magnet":
            row.update(axis_x=o["axis_lps"][0], axis_y=o["axis_lps"][1], axis_z=o["axis_lps"][2],
                       nearest_array=o.get("nearest_array", ""), nearest_array_mm=o.get("nearest_array_mm", ""))
        elif o["kind"] == "bb":
            row.update(diameter_mm=o["eq_diam"], elongation=o["elongation"], surface_cv=o["surface_cv"], peak=o["peak"])
        else:
            row.update(board_x_mm=o["board_center_lps"][0], board_y_mm=o["board_center_lps"][1],
                       board_z_mm=o["board_center_lps"][2], normal_x=o["normal_lps"][0], normal_y=o["normal_lps"][1],
                       normal_z=o["normal_lps"][2], n_sensors=len(o["sensors_lps"]),
                       grid=f"{o['grid_shape'][0]}x{o['grid_shape'][1]}", pitch_1_mm=o["pitch_mm"][0],
                       pitch_2_mm=o["pitch_mm"][1], grid_rms_mm=o["grid_rms_mm"], grid_contrast=o["grid_contrast"])
        rows.append(row)
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with open(out_dir / "objects.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    with open(out_dir / "sensors.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["array", "row", "col", "x_mm", "y_mm", "z_mm"])
        for o in (o for o in objects if o["kind"] == "array"):
            n_v, n_u = o["grid_shape"]
            for s, p in enumerate(o["sensors_lps"]):
                w.writerow([o["name"], s // n_u + 1, s % n_u + 1, *np.round(p, 3)])


def save_label_map(image, objects, path):
    """Label map aligned with the scan: 1 = each object's outline (its outer surface voxels),
    2 = its center (a dot 1.5 mm across), 3 = each sensor chip (a dot 1 mm across)."""
    spacing = np.asarray(image.spacing, float)
    out = np.zeros(image.spatial_shape, np.uint8)

    def dot(c_vox, radius, value):
        m = np.ceil(radius / spacing).astype(int) + 1
        lo = np.maximum(np.floor(c_vox - m).astype(int), 0)
        hi = np.minimum(np.ceil(c_vox + m).astype(int) + 1, out.shape)
        g = np.indices(hi - lo).reshape(3, -1).T + lo
        g = g[np.linalg.norm((g - c_vox) * spacing, axis=1) <= radius]
        out[tuple(g.T)] = value

    for o in objects:
        win = tuple(slice(a, b) for a, b in o["window"])
        out[win][o["mask"] & ~ndi.binary_erosion(o["mask"])] = 1
        for s in o.get("sensors_mm", ()):
            dot(s / spacing, 0.5, 3)
        dot(o["center_voxel"], 0.75, 2)
    tio.LabelMap(tensor=torch.from_numpy(out[None]), affine=image.affine).save(path)


def run(scan, out_dir=None, n_magnets=3, n_arrays=3, magnet_size=(4.0, 20.0), grid=(4, 4),
        window=(-1000, 4000), n_bbs=1, bb_diameter=(4.0, 8.0), verbose=True):
    """Load the scan, find magnets, arrays and glass BBs, and write all outputs. Returns the objects."""
    image = load_scan(scan)
    spacing = np.asarray(image.spacing, float)
    out_dir = Path(out_dir or f"{Path(scan).resolve().name}_objects")
    out_dir.mkdir(parents=True, exist_ok=True)
    if verbose:
        print(f"scan: shape {image.spatial_shape}, spacing {np.round(spacing, 3)} mm, "
              f"orientation {''.join(image.orientation)}\n")
    candidates, rejected = magnet_candidates(image, size=magnet_size, verbose=verbose)
    arrays = find_arrays(image, exclude=candidates, expected=n_arrays or None, grid=grid, verbose=verbose)
    magnets = pick_magnets(candidates, rejected, arrays, expected=n_magnets or None,
                           ref_lps=scan_center_lps(image), verbose=verbose)
    bbs = find_glass_bbs(image, metal=candidates, arrays=arrays, expected=n_bbs or None, diameter=bb_diameter,
                         verbose=verbose)
    objects = magnets + arrays + bbs
    pair_magnets(objects)
    vol = _float_data(image)
    for o in objects:
        o.update(annotate_object(vol, o, image.orientation, spacing, out_dir, window))
        if o["kind"] == "array":
            draw_board(o, out_dir)
    draw_overview(vol, objects, image.orientation, spacing, out_dir, window)
    write_tables(objects, out_dir)
    if objects:
        save_label_map(image, objects, out_dir / "outlines.nii.gz")
    print(f"\n{'name':16s} {'location':26s} {'center x, y, z (LPS mm)':>28s}   size (mm)")
    for o in objects:
        print(f"{o['name']:16s} {o.get('location', ''):26s} {o['center_lps'][0]:9.2f} {o['center_lps'][1]:9.2f} "
              f"{o['center_lps'][2]:9.2f}   {o['dims'][0]:.1f} x {o['dims'][1]:.1f} x {o['dims'][2]:.1f}")
    if (len(magnets) != (n_magnets or len(magnets)) or len(arrays) != (n_arrays or len(arrays))
            or len(bbs) != (n_bbs or len(bbs))):
        print(f"\nnote: expected {n_magnets} magnets, {n_arrays} arrays and {n_bbs} BBs, found "
              f"{len(magnets)}, {len(arrays)} and {len(bbs)}")
    print(f"\nresults written to {out_dir}")
    return objects
