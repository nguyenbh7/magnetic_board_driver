"""Vendored GK Hybrid PCB detector. Reviewed source extracted once.
Do not edit detection thresholds or board algorithms without an A/B review.
This module is importable WITHOUT the source .ipynb being present.
"""
import warnings
import numpy as np
import scipy.ndimage as ndi
SOURCE_NOTEBOOK_SHA256 = "03b0239b2ccad19b3d7b790e807a2f4423631cadb8fd6bde2ebbb409af57900b"
SOURCE_FUNCTIONS_SHA256 = "0a581144bb2b84d8821dd44f269eda75d969b8521df9909a25acffdc63cdbf3e"

def _reslice(data, spacing, center, e1, e2, e3, u_range, v_range, t_range, step=0.25, t_step=None, cval=None):
    """Sample the scan on a plane grid: points center + u*e1 + v*e2 + t*e3 (mm, voxel axes),
    every `step` mm in u and v and every `t_step` mm (default `step`) in t. Points outside the
    scan get `cval` (default: the scan's minimum).
    Returns the stack of slices (t, u, v) and the grid coordinates."""
    gu = np.arange(u_range[0], u_range[1] + 1e-09, step)
    gv = np.arange(v_range[0], v_range[1] + 1e-09, step)
    gt = np.arange(t_range[0], t_range[1] + 1e-09, t_step or step)
    cval = float(data.min()) if cval is None else cval
    U, V = np.meshgrid(gu, gv, indexing='ij')
    base = center + U[..., None] * e1 + V[..., None] * e2
    stack = np.empty((len(gt), len(gu), len(gv)), np.float32)
    for k, t in enumerate(gt):
        coords = ((base + t * e3) / spacing).reshape(-1, 3).T
        stack[k] = ndi.map_coordinates(data, coords, order=1, cval=cval).reshape(U.shape)
    return (stack, gu, gv, gt)

def _side_level(data, spacing, center, e1, e2, e3, u_range, v_range, half_t, step=0.25, gap_mm=0.5, side_mm=4.0):
    """What lies just outside a slab |t| <= half_t, at every (u, v): the median of a layer
    side_mm thick starting gap_mm beyond each face, and the lower of the two sides (so
    something resting against one face of the board does not count). Points outside the
    scan are ignored."""
    sides = []
    for sgn in (1, -1):
        t0, t1 = (sgn * (half_t + gap_mm), sgn * (half_t + gap_mm + side_mm))
        s, *_ = _reslice(data, spacing, center, e1, e2, e3, u_range, v_range, (min(t0, t1), max(t0, t1)), step, t_step=0.5, cval=np.nan)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            sides.append(np.nanmedian(s, axis=0))
    side = np.fmin(*sides)
    if np.isnan(side).any():
        side[np.isnan(side)] = np.nanmedian(side) if np.isfinite(side).any() else float(data.min())
    return side

def _contrast_cut(contrast, side, dense_thr):
    """Threshold for a layer-contrast map: 10% of the way from the air (median of `side`) to
    the dense-material threshold, raised above the map's own background noise if that is
    higher (the lowest 30% of a face-on map is always background: 10th-30th percentile
    spread = 0.76 sigma for normal noise; the cut is at least 6 sigma above it)."""
    q10, q30 = np.percentile(contrast, [10, 30])
    sigma = (q30 - q10) / 0.757
    return float(max(0.1 * (dense_thr - np.median(side)), q30 + 0.524 * sigma + 6 * sigma))

def _footprint(contrast, seed, step, cut, edge_mm=2.0):
    """Board footprint in a face-on layer-contrast map.
    1) Which pixels are board: above `cut`, gaps narrower than 2 mm closed, only the parts
       touching `seed` (pixels known to be on the board), holes filled.
    2) Where the edge is: the low cut lies far out on the blurred edge, so the outer
       `edge_mm` are trimmed back to the half-maximum between the board's rim (1-3 mm inside
       the edge) and the background. Only that outer band can be trimmed, so a dark streak
       reaching the edge leaves a notch at most, never a hollow board.
    3) Thin attachments (cables, wires) trimmed off, the biggest piece kept, holes filled."""
    fp = ndi.binary_closing(contrast > cut, structure=_disk(1.0 / step))
    lab, n = ndi.label(fp)
    keep = np.unique(lab[seed & (lab > 0)])
    fp = ndi.binary_fill_holes(np.isin(lab, keep[keep > 0]))
    rim = fp & ndi.binary_erosion(fp, structure=_disk(1.0 / step)) & ~ndi.binary_erosion(fp, structure=_disk(3.0 / step))
    out = ~ndi.binary_dilation(fp, structure=_disk(3.0 / step))
    if rim.any() and out.any():
        half = 0.5 * (np.median(contrast[rim]) + np.median(contrast[out]))
        band = fp & ~ndi.binary_erosion(fp, structure=_disk(edge_mm / step))
        fp = fp & ~(band & (contrast <= half))
    fp = ndi.binary_opening(fp, structure=_disk(2.5 / step))
    lab, n = ndi.label(fp)
    if n > 1:
        fp = lab == 1 + np.argmax(ndi.sum(fp, lab, np.arange(1, n + 1)))
    return ndi.binary_fill_holes(fp)

def _measure_board(data, spacing, affine, pts, grid, board_size, max_thickness, min_rect, slab_mm, pitch_mm, min_contrast, dense_thr):
    """Measure one flat dense object as a sensor board (helper for find_arrays).
    Returns (record, None) for a board with a sensor grid, or (None, reason)."""
    step = 0.25
    if len(pts) > 200000:
        pts = pts[np.random.default_rng(0).choice(len(pts), 200000, replace=False)]
    c, u, v, n = _fit_plane(pts)
    near = np.abs((pts - c) @ n) <= 3.0
    pu, pv = ((pts[near] - c) @ u, (pts[near] - c) @ v)
    if pu.max() - pu.min() > 1.5 * board_size[1]:
        return (None, f'too large ({pu.max() - pu.min():.0f} mm)')
    m = 0.5 * board_size[1] + 5
    u_rng, v_rng = ((pu.min() - m, pu.max() + m), (pv.min() - m, pv.max() + m))
    passes = (slab_mm + 3, slab_mm + 1.5, slab_mm)
    for k_pass, half_t in enumerate(passes):
        stack, gu, gv, gt = _reslice(data, spacing, c, u, v, n, u_rng, v_rng, (-half_t, half_t), step)
        side = _side_level(data, spacing, c, u, v, n, u_rng, v_rng, half_t, step)
        contrast = stack.max(0) - side
        cut = _contrast_cut(contrast, side, dense_thr)
        on = np.abs((pts - c) @ n) <= half_t
        seed = np.zeros(contrast.shape, bool)
        seed[np.clip(np.rint(((pts[on] - c) @ u - gu[0]) / step).astype(int), 0, len(gu) - 1), np.clip(np.rint(((pts[on] - c) @ v - gv[0]) / step).astype(int), 0, len(gv) - 1)] = True
        fp = _footprint(contrast, seed, step, cut)
        if fp.sum() < 50:
            return (None, 'no flat footprint')
        if k_pass == len(passes) - 1:
            break
        fu, fv = np.argwhere(fp).T
        tt, kk = np.nonzero(stack[:, fu, fv] > dense_thr)
        if len(tt) < 100:
            break
        if len(tt) > 200000:
            pick = np.random.default_rng(0).choice(len(tt), 200000, replace=False)
            tt, kk = (tt[pick], kk[pick])
        P = c + gu[fu[kk], None] * u + gv[fv[kk], None] * v + gt[tt, None] * n
        c, u, v, n = _fit_plane(P, keep_mm=2.0)
        pu, pv = ((P - c) @ u, (P - c) @ v)
        u_rng, v_rng = ((pu.min() - 15, pu.max() + 15), (pv.min() - 15, pv.max() + 15))
        pts = P
    body = ndi.binary_opening(fp, structure=_disk(4.0 / step))
    body = body if body.sum() > 0.5 * fp.sum() else fp
    fp_pts = np.argwhere(body) * step + np.array([gu[0], gv[0]])
    rc, sides, R = _min_area_rect(fp_pts)
    sides = sides + step
    rect = fp.sum() * step ** 2 / np.prod(sides)
    if not board_size[0] <= sides[1] <= sides[0] <= board_size[1]:
        return (None, f'not board-sized ({sides[0]:.0f} x {sides[1]:.0f} mm)')
    if rect < min_rect:
        return (None, f'not rectangular (fills {rect:.0%} of its rectangle)')
    e1 = R[0, 0] * u + R[0, 1] * v
    e2 = R[1, 0] * u + R[1, 1] * v
    e3 = np.cross(e1, e2)
    center = c + rc[0] * u + rc[1] * v
    hu, hv = (sides[0] / 2 + 6, sides[1] / 2 + 6)
    stack, gu, gv, gt = _reslice(data, spacing, center, e1, e2, e3, (-hu, hu), (-hv, hv), (-slab_mm, slab_mm), step)
    side = _side_level(data, spacing, center, e1, e2, e3, (-hu, hu), (-hv, hv), slab_mm, step)
    mean, mip = (stack.mean(0), stack.max(0))
    U, V = np.meshgrid(gu, gv, indexing='ij')
    in_rect = (np.abs(U) <= sides[0] / 2) & (np.abs(V) <= sides[1] / 2)
    contrast = mip - side
    fp = _footprint(contrast, in_rect & (contrast > cut), step, cut)
    if fp.sum() < 50:
        return (None, 'no flat footprint')
    plate = float(np.median(mean[fp]))
    outside = ~ndi.binary_dilation(fp, structure=_disk(1.0 / step))
    air = float(np.median(mean[outside])) if outside.any() else float(np.median(side))
    prof = stack[:, fp].mean(1)
    half = 0.5 * (prof.max() + prof.min())
    above = np.flatnonzero(prof >= half)
    thickness = (above[-1] - above[0] + 1) * step if len(above) else 0.0
    t_mid = float(np.sum(gt * np.clip(prof - half, 0, None)) / max(np.sum(np.clip(prof - half, 0, None)), 1e-06))
    if thickness > max_thickness:
        return (None, f'too thick ({thickness:.1f} mm)')
    inside = ndi.binary_erosion(fp, structure=_disk(1.0 / step))
    g = _find_grid(mean, inside, grid, step, pitch_mm)
    if g is None:
        return (None, 'no room for a sensor grid')
    score, g_c, g_p, (n_v, n_u) = g
    contrast_needed = min_contrast if min_contrast is not None else 0.1 * (plate - air)
    if score < contrast_needed:
        return (None, f'no sensor grid (contrast {score:.0f} < {contrast_needed:.0f})')
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
            t = float(np.sum(gt * dw) / max(dw.sum(), 1e-06))
            sensors_uvt.append([gu[0] + pc[0] * step, gv[0] + pc[1] * step, t])
    sensors_uvt = np.array(sensors_uvt)
    ab = np.array([(a, b) for b in np.arange(n_v) - (n_v - 1) / 2 for a in np.arange(n_u) - (n_u - 1) / 2])
    M = np.c_[np.ones(len(ab)), ab]
    fit = np.linalg.lstsq(M, sensors_uvt[:, :2], rcond=None)[0]
    grid_rms = float(np.sqrt(np.mean(np.sum((M @ fit - sensors_uvt[:, :2]) ** 2, axis=1))))
    if np.mean(sensors_uvt[:, 2]) < t_mid:
        e2, e3 = (-e2, -e3)
        sensors_uvt[:, 1:] *= -1
        t_mid = -t_mid
        mean, mip, fp, stack = (mean[:, ::-1], mip[:, ::-1], fp[:, ::-1], stack[::-1, :, ::-1])
        gv, gt = (-gv[::-1], -gt[::-1])
    to_mm = lambda q: center + q[..., :1] * e1 + q[..., 1:2] * e2 + q[..., 2:3] * e3
    sensors_mm = to_mm(sensors_uvt)
    board_center_mm = center + t_mid * e3
    pu_mm, pv_mm = np.linalg.norm(fit[1:], axis=1)
    blk = (sensors_uvt[:, 0].min() - pu_mm / 2, sensors_uvt[:, 0].max() + pu_mm / 2, sensors_uvt[:, 1].min() - pv_mm / 2, sensors_uvt[:, 1].max() + pv_mm / 2)
    rec = dict(kind='array', center_mm=sensors_mm.mean(0), board_center_mm=board_center_mm, e1=e1, e2=e2, e3=e3, dims=np.array([sides[0], sides[1], thickness]), rectangularity=rect, grid_contrast=score, grid_rms_mm=grid_rms, depth_sd_mm=float(np.std(sensors_uvt[:, 2])), grid_shape=(n_v, n_u), pitch_mm=np.array([pu_mm, pv_mm]), sensors_mm=sensors_mm, sensors_uvt=sensors_uvt, block_uv=blk, face=dict(mean=mean, mip=mip, fp=fp, gu=gu, gv=gv, step=step), t_half=thickness / 2, t_mid=t_mid, board_level=plate, spacing=spacing)
    to_lps = lambda p_mm: voxel_to_lps(affine, p_mm / spacing)
    rec['center_voxel'] = rec['center_mm'] / spacing
    rec['center_lps'] = to_lps(rec['center_mm'])
    rec['board_center_lps'] = to_lps(board_center_mm)
    rec['sensors_lps'] = np.array([to_lps(p) for p in sensors_mm])
    for key in ('e1', 'e2', 'e3'):
        rec[key + '_lps'] = dir_to_lps(affine, rec[key])
    rec['normal_lps'] = rec['e3_lps']
    rec['mask'], rec['window'], rec['block_mask'] = _board_masks(rec, spacing, data.shape)
    return (rec, None)

def install(core):
    """Install exact reviewed Hybrid board measurement into existing CT pipeline.

    Other magnet/BB detectors are unchanged. Ordinary face reslices reuse
    accelerated production interpolation; special side-layer reslices still
    use the alternate method. Match controlled offline research adapter.
    """
    for name, value in vars(core).items():
        if name not in globals() and not name.startswith("__"):
            globals()[name] = value
    # Preserve alternative reslice before routing ordinary slabs to core.
    local_reslice = _reslice
    accelerated_reslice = core._reslice

    def hybrid_reslice(*args, t_step=None, cval=None, **kwargs):
        if t_step is None and cval is None:
            return accelerated_reslice(*args, **kwargs)
        return local_reslice(*args, t_step=t_step, cval=cval, **kwargs)

    globals()["_reslice"] = hybrid_reslice

    def board_measure(data, spacing, affine, pts, grid, board_size,
                      max_thickness, min_rect, slab_mm, pitch_mm,
                      min_contrast, dense_thr, audit=None):
        # Accepted alternate _measure_board does not expose the optional
        # timing audit argument used by the maintained optimized detector.
        return _measure_board(
            data, spacing, affine, pts, grid, board_size, max_thickness,
            min_rect, slab_mm, pitch_mm, min_contrast, dense_thr
        )

    core._measure_board = board_measure
    return SOURCE_FUNCTIONS_SHA256
