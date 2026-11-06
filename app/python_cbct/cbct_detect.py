"""Standalone CBCT detector: DICOM series or ZIP -> detection JSON + QC.

The detector functions are copied unchanged from the collaborator's notebook.
Run image processing BEFORE streaming, and never activate its unreviewed output.
"""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zipfile
import sys
from time import perf_counter

def progress(message):
    print('CBCT_PROGRESS:' + message, flush=True)



def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fd:
        for chunk in iter(lambda: fd.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


@contextmanager
def scan_path(path):
    """Extract a bounded local ZIP safely; never let paths escape the temp root."""
    if path.suffix.lower() != '.zip':
        if not path.exists():
            raise FileNotFoundError(path)
        yield path
        return
    progress('Inspecting ZIP archive...')
    with zipfile.ZipFile(path) as archive:
        infos = archive.infolist()
        if len(infos) > 5000 or sum(x.file_size for x in infos) > 1_000_000_000:
            raise ValueError('Scan archive exceeds extraction limits')
        for info in infos:
            parts = Path(info.filename).parts
            if (Path(info.filename).is_absolute() or '..' in parts or '\\' in info.filename or
                    not parts or ':' in info.filename or info.filename.startswith('\\\\')):
                raise ValueError(f'Unsafe archive entry: {info.filename!r}')
            if info.is_dir():
                continue
        with tempfile.TemporaryDirectory(prefix='cbct_detector_') as tmp:
            progress(f'Extracting {len(infos)} archive entries...')
            archive.extractall(tmp)
            progress('ZIP extraction complete')
            yield Path(tmp)


def board_site_diagnostics(objects, audit):
    """Describe detected/missed boards from nearby named magnets, not unstable array names."""
    known = [
        ("magnet_left", "PATIENT LEFT"),
        ("magnet_right", "PATIENT RIGHT"),
        ("magnet_superior", "TOP OF HEAD / SUPERIOR"),
    ]
    magnets = {o["name"]: o for o in objects if o["kind"] == "magnet"}
    if any(name not in magnets for name, _ in known):
        return {"status": "unresolved",
                "reason": "Not all three named magnet anchors were detected; cannot identify missing physical board safely."}
    def distance(a, b):
        return sum((float(x)-float(y))**2 for x,y in zip(a,b)) ** 0.5
    anchors = [(name, side, magnets[name]["center_lps"]) for name,side in known]
    arrays = [o for o in objects if o["kind"]=="array"]
    rejected = audit.get("rejected_candidates", [])
    sites = []
    for name,side,center in anchors:
        def confident_near(p):
            d=distance(p,center)
            alternatives=[distance(p,xyz) for n,_,xyz in anchors if n!=name]
            return d <= 50 and min(alternatives)-d >= 25
        matched = [a for a in arrays if confident_near(a["center_lps"])]
        nearby_rejected = [
            {"distance_mm": round(distance(r["center_lps_mm"],center),2),
             "reason": r["reason"], "center_lps_mm": r["center_lps_mm"]}
            for r in rejected
            if r.get("center_lps_mm") and confident_near(r["center_lps_mm"])
        ]
        nearby_rejected.sort(key=lambda r:r["distance_mm"])
        sites.append({
            "physical_location":side,"magnet_anchor":name,
            "status": ("DETECTED" if len(matched)==1 else
                       "NOT DETECTED" if len(matched)==0 else "AMBIGUOUS"),
            "array_names":[a["name"] for a in matched],
            "nearby_rejected_candidates":nearby_rejected[:8],
            "nearby_rejected_count":len(nearby_rejected),
        })
    return {"status":"provisional_magnet_proximity","sites":sites,
            "caveat":"Sites are inferred from DICOM LPS proximity; review QC and physical setup."}


def detect(scan, output, *, qc=True, expected_magnets=3, expected_boards=3, expected_bbs=1,
           probe_complete_superior=False, allow_chip_first_superior=True):
    started = perf_counter()
    stage_times = {}
    array_audit = {}
    magnet_audit = {}
    image_holder = {}
    # Import after CLI parsing so --help works without heavy image dependencies.
    import numpy as np
    import detector_core as core
    output.mkdir(parents=True, exist_ok=True)
    progress('Preparing DICOM scan...')
    # The original detector functions are preserved. Emit stages only at their
    # boundaries, so progress never claims a false percentage.
    for name, label in (
        ('load_scan', 'Loading CT volume...'),
        ('magnet_candidates', 'Finding magnet candidates...'),
        ('find_arrays', 'Detecting sensor boards and chip grids...'),
        ('pick_magnets', 'Resolving the three magnets...'),
        ('find_glass_bbs', 'Identifying glass BB candidates...'),
        ('draw_overview', 'Generating QC overview...'),
        ('write_tables', 'Writing detection tables...'),
        ('save_label_map', 'Saving segmentation...'),
    ):
        original = getattr(core, name)
        def wrapped(*args, _original=original, _label=label, _name=name, **kwargs):
            progress(_label)
            stage_start=perf_counter()
            try:
                result=_original(*args, **kwargs)
                if _name=="load_scan":
                    # Retain the already loaded DICOM tensor for a small
                    # missing-board ROI test. Never reread the entire scan.
                    image_holder["image"]=result
                if _name=="magnet_candidates":
                    # Like find_arrays, this function sets last_audit on the
                    # wrapped module-global function name.
                    magnet_audit.update(
                        getattr(core.magnet_candidates, "last_audit", {}))
                if _name=="find_arrays":
                    # detector_core.find_arrays assigns last_audit to its
                    # module-global function name. The timing wrapper now
                    # occupies that name, so the audit lives on the WRAPPER
                    # (core.find_arrays), not on _original.
                    array_audit.update(getattr(core.find_arrays,"last_audit",{}))
                return result
            finally:
                elapsed=perf_counter()-stage_start
                stage_times[_name]=round(stage_times.get(_name,0.0)+elapsed,3)
                print(f"CBCT_TIMING: {_name}: {elapsed:.3f} s",flush=True)
        setattr(core, name, wrapped)
    with scan_path(scan) as scan_directory:
        if qc:
            objects = core.run(scan_directory, out_dir=output, n_magnets=expected_magnets,
                               n_arrays=expected_boards, n_bbs=expected_bbs)
        else:
            image = core.load_scan(scan_directory)
            candidates, rejected = core.magnet_candidates(image)
            arrays = core.find_arrays(image, exclude=candidates, expected=expected_boards,
                                      grid=(4, 4))
            magnets = core.pick_magnets(candidates, rejected, arrays, expected=expected_magnets,
                                        ref_lps=core.scan_center_lps(image))
            bbs = core.find_glass_bbs(image, metal=candidates, arrays=arrays,
                                       expected=expected_bbs)
            objects = magnets + arrays + bbs
            core.pair_magnets(objects)
            core.write_tables(objects, output)
        # No patient demographics, PatientID or PatientName are exported.
        frame_uid = None
        # Explicit DICOM frame UID prevents assuming different scans share axes.
        dicom_files = sorted(scan_directory.rglob('IMG*.dcm')) if scan_directory.is_dir() else []
        if dicom_files:
            header = core.sitk.ImageFileReader()
            header.SetFileName(str(dicom_files[0]))
            header.ReadImageInformation()
            if header.HasMetaDataKey('0020|0052'):
                frame_uid = header.GetMetaData('0020|0052').strip()
        records = []
        for obj in objects:
            rec = {'kind': obj['kind'], 'name': obj['name'],
                   'center_lps_mm': [float(x) for x in obj['center_lps']]}
            if obj['kind'] == 'magnet':
                rec['unsigned_axis_lps'] = [float(x) for x in obj['axis_lps']]
                rec['apparent_dimensions_mm'] = [float(x) for x in obj['dims']]
                rec['nearest_array'] = obj.get('nearest_array')
            elif obj['kind'] == 'array':
                rec['chip_grid_lps_mm'] = np.asarray(obj['sensors_lps']).astype(float).tolist()
                rec['grid_shape'] = list(map(int, obj['grid_shape']))
                rec['grid_pitch_mm'] = np.asarray(obj['pitch_mm']).astype(float).tolist()
                rec['grid_rms_mm'] = float(obj['grid_rms_mm'])
                rec['board_center_lps_mm'] = [float(x) for x in obj['board_center_lps']]
                rec['board_normal_lps'] = [float(x) for x in obj['normal_lps']]
            elif obj['kind'] == 'bb':
                rec['diameter_mm'] = float(obj['eq_diam'])
                rec['proposed_identity'] = 'unverified_glass_bb'
            records.append(rec)
        # Preserve rejected flat-object positions and exact failure reasons,
        # even when the scan cannot produce a full 3-board calibration.
        board_audit=array_audit
        site_diagnostics=board_site_diagnostics(objects,board_audit)
        (output / "board_detection_audit.json").write_text(
            json.dumps({"audit":board_audit,"physical_board_sites":site_diagnostics},indent=2)+"\n")
        for site in site_diagnostics.get("sites",[]):
            msg=f"CBCT_BOARD: {site['physical_location']}: {site['status']}"
            if site["nearby_rejected_candidates"]:
                reasons="; ".join(
                    f"{entry['reason']} ({entry['distance_mm']:.1f} mm from magnet)"
                    for entry in site["nearby_rejected_candidates"][:3])
                msg+=f"; nearby rejected: {reasons}"
            print(msg,flush=True)
        if site_diagnostics.get("status")=="unresolved":
            print(f"CBCT_BOARD: UNRESOLVED: {site_diagnostics['reason']}",flush=True)
        counts = {kind: sum(o['kind'] == kind for o in objects)
                  for kind in ('magnet', 'array', 'bb')}
        expected = {'magnet': expected_magnets, 'array': expected_boards, 'bb': expected_bbs}
        # Independent of the original PCB LENGTH / rectangularity test:
        # try a chip-first lattice around rejected component seeds for the
        # ONE missing receiver, whatever its patient-side identity. Successful
        # detection gives CT geometry for review, NOT magnetic calibration.
        chip_grid_diagnostics = None
        chip_grid_attempts = []
        chip_grid_rescue = {"status":"not_needed" if counts["array"]==expected_boards
                            else "not_attempted"}
        missing_sites=[
            site for site in site_diagnostics.get("sites", [])
            if site["status"]=="NOT DETECTED"
        ]
        board_for_anchor={
            "magnet_left":"array_left",
            "magnet_right":"array_right",
            "magnet_superior":"array_superior",
        }
        accepted_superior=next(
            (o for o in objects
             if o["kind"]=="array" and o["name"]=="array_superior"),None)
        if (allow_chip_first_superior and counts["array"]==2
                and counts["magnet"]==3 and counts["bb"] in (0,1)
                and len(missing_sites)==1
                and missing_sites[0].get("nearby_rejected_candidates")
                and image_holder.get("image") is not None):
            target=board_for_anchor.get(missing_sites[0].get("magnet_anchor"))
            if target is not None:
                from cbct_superior_roi import crop_lps_box
                from chip_grid_recovery import recover_grid
                from superior_board_rescue import recover_missing_array,update_detection_tables
                seeds=missing_sites[0]["nearby_rejected_candidates"][:5]
                strong=[]
                started_fallback=perf_counter()
                progress(f"Searching for 4x4 chip lattice at missing {target} site...")
                for seed in seeds:
                    item={"board":target,
                          "seed_distance_to_magnet_mm":seed["distance_mm"],
                          "original_rejection":seed["reason"]}
                    try:
                        crop, affine, _, _=crop_lps_box(
                            image_holder["image"],seed["center_lps_mm"],48)
                        diagnostic=recover_grid(
                            crop,affine,seed["center_lps_mm"],
                            board_audit["board_threshold"])
                        diagnostic["source"]=(
                            f"rejected_{target.removeprefix('array_')}_candidate_roi")
                        diagnostic["target_board"]=target
                        item["status"]=diagnostic.get("status")
                        if diagnostic.get("status")=="strong_chip_grid_candidate":
                            trial={
                                "format":"gk_cbct_detections_v1",
                                "coordinate_frame":"DICOM_LPS_mm",
                                "objects":records,
                                "chip_grid_diagnostic":diagnostic,
                                "board_detection_audit":board_audit,
                            }
                            board=recover_missing_array(trial,crop,affine,target)
                            item["status"]="passed_geometry_qc"
                            item["grid_rms_mm"]=board["grid_rms_mm"]
                            item["center_lps_mm"]=board["center_lps_mm"]
                            strong.append((board,diagnostic))
                    except Exception as error:
                        item["status"]="rejected"
                        item["reason"]=f"{type(error).__name__}: {error}"
                    chip_grid_attempts.append(item)
                stage_times["chip_first_board_fallback"]=round(
                    perf_counter()-started_fallback,3)
                if strong:
                    # Alternative rejected fragments may identify the SAME
                    # actual 4x4 chip array. Distinct valid arrays near the
                    # same anchor are ambiguous: never silently take the best.
                    centers=np.array([x[0]["center_lps_mm"] for x in strong])
                    ambiguity=max(
                        float(np.linalg.norm(a-b))
                        for a in centers for b in centers)
                    if ambiguity>3:
                        chip_grid_rescue={
                            "status":"rejected_ambiguous_distinct_grids",
                            "target_board":target,
                            "distinct_candidate_spread_mm":round(ambiguity,3),
                        }
                    else:
                        board,chip_grid_diagnostics=min(
                            strong,key=lambda x:x[0]["grid_rms_mm"])
                        try:
                            update_detection_tables(output,board)
                            records.append(board)
                            counts["array"]+=1
                            chip_grid_rescue={
                                "status":"accepted_for_manual_geometry_review",
                                "target_board":target,
                                "method":"chip_first_generic_board_fallback",
                                "grid_rms_mm":board["grid_rms_mm"],
                                "quality":board["quality"],
                                "warning":"16 measured CT chips, no independently recovered PCB outline or center; not a magnetic calibration.",
                            }
                            for site in site_diagnostics["sites"]:
                                if site["magnet_anchor"]=="magnet_"+target.removeprefix("array_"):
                                    site["status"]="RECOVERED_CHIP_FIRST"
                                    site["array_names"]=[target]
                            print(f"CBCT_BOARD: {target} recovered as 4x4 chip grid "
                                  "(manual geometry review required)",flush=True)
                        except Exception as error:
                            chip_grid_rescue={
                                "status":"rejected_export_error",
                                "target_board":target,
                                "reason":f"{type(error).__name__}: {error}",
                            }
                else:
                    chip_grid_rescue={
                        "status":"rejected_no_valid_grid",
                        "target_board":target,
                        "attempt_count":len(chip_grid_attempts),
                    }
                # Preserve all candidate outcomes, including weak/failed
                # seeds. Do not publish CT pixels or invent a board outline.
                if strong and chip_grid_diagnostics is None:
                    chip_grid_diagnostics=strong[0][1]
        elif not allow_chip_first_superior and counts["array"]!=expected_boards:
            chip_grid_rescue={"status":"disabled_by_cli"}
        elif counts["array"]!=expected_boards and len(missing_sites)!=1:
            chip_grid_rescue={"status":"not_attempted_ambiguous_or_multiple_missing_sites"}
        # Preserve the independent successful-scan reference comparison.
        if (probe_complete_superior and accepted_superior is not None
                and image_holder.get("image") is not None):
            started_probe=perf_counter()
            try:
                from cbct_superior_roi import crop_lps_box
                from chip_grid_recovery import recover_grid, compare_recovery_to_accepted_grid
                crop,affine,_,_=crop_lps_box(
                    image_holder["image"],accepted_superior["center_lps"],48)
                reference=recover_grid(
                    crop,affine,accepted_superior["center_lps"],
                    board_audit["board_threshold"])
                reference["source"]="accepted_superior_reference_probe"
                reference["comparison_to_original_array"]=(
                    compare_recovery_to_accepted_grid(
                        reference,accepted_superior["sensors_lps"],
                        accepted_superior.get("normal_lps")))
                chip_grid_diagnostics=reference
            except Exception as error:
                chip_grid_attempts.append({
                    "status":"reference_probe_exception",
                    "reason":f"{type(error).__name__}: {error}"})
            finally:
                stage_times["superior_chip_grid_diagnostic"]=round(
                    perf_counter()-started_probe,3)
        if chip_grid_diagnostics is not None:
            (output/"chip_grid_diagnostic.json").write_text(
                json.dumps(chip_grid_diagnostics,indent=2)+"\n")
            if chip_grid_diagnostics.get("target_board")=="array_superior" or (
                    chip_grid_diagnostics.get("source")=="accepted_superior_reference_probe"):
                (output/"superior_chip_grid_diagnostic.json").write_text(
                    json.dumps(chip_grid_diagnostics,indent=2)+"\n")
        (output/"chip_grid_fallback_attempts.json").write_text(
            json.dumps(chip_grid_attempts,indent=2)+"\n")
        superior_rescue=(chip_grid_rescue if chip_grid_rescue.get("target_board")=="array_superior"
                         else {"status":"not_needed" if accepted_superior else "not_attempted"})
        result = {
            'format': 'gk_cbct_detections_v1', 'coordinate_frame': 'DICOM_LPS_mm',
            'source_name': scan.name, 'frame_of_reference_uid': frame_uid,
            'source_archive_sha256': sha256(scan) if scan.is_file() else None,
            'collaborator_notebook': 'find_magnets_arrays.ipynb',
            'counts': counts, 'expected_counts': expected,
            'board_detection_audit': board_audit,
            'magnet_detection_audit': magnet_audit,
            'physical_board_sites': site_diagnostics,
            'superior_chip_grid_diagnostic': (
                chip_grid_diagnostics
                if chip_grid_diagnostics and (
                    chip_grid_diagnostics.get("target_board")=="array_superior"
                    or chip_grid_diagnostics.get("source")=="accepted_superior_reference_probe")
                else None),
            'chip_grid_diagnostic': chip_grid_diagnostics,
            'chip_grid_fallback_attempts': chip_grid_attempts,
            'chip_first_board_rescue': chip_grid_rescue,
            'superior_rescue': superior_rescue,
            'stage_timing_seconds': stage_times,
            'status': 'unreviewed' if counts == expected else 'incomplete_detection',
            'requires_review': True,
            'warning': 'Chip grid indices are NOT firmware indices. Magnet axes are unsigned. '
                       'Glass BB identity is unverified. No magnetic calibration is inferred.',
            'objects': records,
        }
        progress('Writing detection JSON...')
        result["total_timing_seconds"]=round(perf_counter()-started,3)
        (output / 'detections.json').write_text(json.dumps(result, indent=2) + '\n')
        print(f'Wrote {output / "detections.json"}: {counts}, status={result["status"]}')
        for name,secs in sorted(stage_times.items(),key=lambda x:-x[1]):
            print(f"CBCT_TIMING_SUMMARY: {name} {secs:.3f} s",flush=True)
        print(f"CBCT_TIMING_SUMMARY: total {result['total_timing_seconds']:.3f} s (includes extraction, IO, imports and hashing)",flush=True)
        progress('Detection complete')
        return result


# The existing benchmark scripts established a switch for each accepted speedup.
# OFF = legacy slow implementation; ON = maintained production implementation.
# The unvalidated batch-reslice experiment stays disabled for all three modes.
DETECTOR_SPEED_FLAGS = (
    "GK_CBCT_REUSE_INITIAL_MAGNET_EDT",
    "GK_CBCT_CACHE_RESLICE_MIN",
    "GK_CBCT_MAGNET_EXCLUSION_EDT",
    "GK_CBCT_SOLIDITY_HULL_VERTICES",
    "GK_CBCT_AFFINE_RESLICE",
    "GK_CBCT_CROP_MAGNET_OUTLINE_EDT",
    "GK_CBCT_CROP_MAGNET_CORE_EDT",
)


def configure_speed_mode(mode):
    """Explicitly pin speed switches; inherited shell values cannot silently
    turn Original into Optimized or vice versa. No threshold/QC relaxation."""
    if mode not in ("original", "optimized", "hybrid"):
        raise ValueError("Unknown CT detection method: " + str(mode))
    enabled = mode != "original"
    for key in DETECTOR_SPEED_FLAGS:
        os.environ[key] = "1" if enabled else "0"
    os.environ["GK_CBCT_BATCH_RESLICE"] = "0"
    return {"mode": mode, "accelerated_core": enabled}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scan', type=Path, help='DICOM series folder, .zip, or NIfTI file')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--no-qc', action='store_true', help='Skip PNG images and label NIfTI')
    parser.add_argument('--detector-variant', choices=('original','hybrid','optimized'),
                        default='optimized',
                        help='Original non-accelerated pipeline, Hybrid board detector, or Optimized maintained pipeline')
    parser.add_argument('--probe-complete-superior', action='store_true',
                        help='Also test chip-first recovery on an accepted superior array (research QC only)')
    parser.add_argument('--no-chip-first-board-rescue', '--no-superior-rescue',
                        dest='no_chip_first_board_rescue',action='store_true',
                        help='Disable generic 4x4 chip-first fallback for any missing GK board; keep original detector result')
    args = parser.parse_args()
    speed = configure_speed_mode(args.detector_variant)
    hybrid_source_sha256 = None
    if args.detector_variant == 'hybrid':
        # Pure maintained Python module, vendored ONCE at development time.
        # There is no notebook selection, notebook execution, or local
        # Jupyter dependency when a live DICOM scan is imported.
        try:
            import hybrid_board_detector
        except ImportError as exc:
            raise RuntimeError(
                "Built-in Hybrid detector source is not installed. "
                "The reviewed board methods must first be vendored into "
                "tools/cbct_bridge/hybrid_board_detector.py using "
                "vendor_hybrid_board_detector.py on a developer workstation. "
                "Original and Optimized remain available; no fallback was applied."
            ) from exc
        import detector_core as core
        hybrid_board_detector.install(core)
        hybrid_source_sha256 = hybrid_board_detector.SOURCE_FUNCTIONS_SHA256
        if not isinstance(hybrid_source_sha256, str) or len(hybrid_source_sha256) != 64:
            raise RuntimeError("Hybrid source provenance is missing or invalid")
    progress(f"Selected {args.detector_variant} CT detector "
             f"(speedups={'on' if speed['accelerated_core'] else 'off'}; "
             "geometry is review-only)")

    result = detect(args.scan,args.output,qc=not args.no_qc,
                    probe_complete_superior=args.probe_complete_superior,
                    # Hybrid follows the controlled candidate path: no rescue.
                    # Both baseline and optimized independent detectors retain
                    # the same optional chip-first fallback.
                    allow_chip_first_superior=(
                        not args.no_chip_first_board_rescue
                        if args.detector_variant != 'hybrid' else False))
    result['selected_detector_variant'] = args.detector_variant
    result['accelerated_detector_core'] = speed['accelerated_core']
    if hybrid_source_sha256 is not None:
        result['hybrid_source_sha256'] = hybrid_source_sha256
    (args.output / 'detections.json').write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    if result['status'] != 'unreviewed':
        raise SystemExit('Incomplete detection: inspect results, do not build a live profile.')


if __name__ == '__main__':
    main()
