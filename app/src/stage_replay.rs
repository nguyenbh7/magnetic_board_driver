//! Standalone OFFLINE six-DOF replay for controlled rotation-stage recordings.
//!
//! Isolation guarantees (see docs/STAGE_OFFLINE_REPLAY.md in the research repo):
//! * Reached ONLY through `app stage-replay --input X --out-dir Y`; the GUI/live
//!   application never calls this module and its default profile is unchanged.
//! * The input schema (`deny_unknown_fields`) has NO field for stage angles,
//!   plateau labels, optical HDMM, or POST/held-out CT endpoints. Such data
//!   cannot reach calibration or pose reconstruction.
//! * Magnet strengths and board offsets are fitted ONCE from the declared
//!   stationary PRE window, gated by the existing calibration acceptance rules,
//!   and then frozen. The existing finite-cylinder solver (`rigid_fit`) is
//!   reused unchanged; nothing here re-implements optimisation.
//! * Rejected fits and missing intervals are reported, never interpolated.

use std::path::{Path, PathBuf};

use data_transfer::rpc::{BoardFrame, BoardPresence};
use serde::Deserialize;

use crate::cbct_geometry::CbctGeometry;
use crate::rigid_fit::{
    fit_magnet_strengths_masked, fit_strengths_with_board_offsets_masked, RigidLiveFit,
    RigidProfile,
};

pub const INPUT_FORMAT: &str = "gk_stage_offline_replay_input_v1";
pub const REPORT_FORMAT: &str = "gk_stage_offline_replay_solver_report_v1";
pub const POSES_FORMAT: &str = "gk_controlled_signed_stage_frozen_poses_v1";
/// Operator-confirmed identical 5/16 x 1/8 inch cylinders (research profile).
pub const PROFILE_ALL_THIN: &str = "gk_all_thin_7.9375x3.175_research_v1";
/// Historical 6.35 mm superior magnet; selectable only as an explicit control.
pub const PROFILE_HISTORICAL: &str = "gk_historical_thick_superior_7.9375x6.35_v1";
const FIT_STRIDE: u32 = 10;
const MIN_ACCEPTED_FOR_SCORER: usize = 16;
const GAP_FACTOR: f64 = 3.0;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Frame {
    board_id: u16,
    frame_id: u32,
    base_time_us: u64,
    sensor_mask: u16,
    offsets_us: [u16; 16],
    fields_ut: [[f64; 3]; 16],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReplayInput {
    format: String,
    profile_id: String,
    cad_mapping_reviewed_by_operator: bool,
    magnet_dimensions_operator_confirmed: bool,
    ct_detector_json_path: String,
    ct_detector_sha256: String,
    clock_basis: String,
    calibration_window_s: [f64; 2],
    range_guard_fraction: f64,
    fit_frame_stride: u32,
    sensor_masks: [u16; 3],
    calibration_stationary_raw_mt: [[[f64; 3]; 16]; 3],
    calibration_background_raw_mt: Option<[[[f64; 3]; 16]; 3]>,
    frames: Vec<Frame>,
    /// Opaque hashes/attestations carried into the report. Never used in math.
    provenance: serde_json::Value,
}

#[derive(Debug)]
pub struct Outcome {
    pub calibration_passed: bool,
    pub accepted_poses: usize,
}

/// `app stage-replay --input FILE --out-dir NEW_DIR`; returns the process exit code.
pub fn run_cli(args: &[String]) -> i32 {
    let mut input = None;
    let mut out_dir = None;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--input" => input = it.next().map(PathBuf::from),
            "--out-dir" => out_dir = it.next().map(PathBuf::from),
            other => {
                eprintln!("stage-replay: unknown argument {other}");
                return 2;
            }
        }
    }
    let (Some(input), Some(out_dir)) = (input, out_dir) else {
        eprintln!("usage: app stage-replay --input replay_input.json --out-dir NEW_DIR");
        return 2;
    };
    match replay(&input, &out_dir) {
        Ok(o) if !o.calibration_passed => {
            eprintln!("stage-replay: PRE calibration QC FAILED; no poses exported (see solver_report.json)");
            3
        }
        Ok(o) => {
            println!("stage-replay: {} accepted poses written to {}", o.accepted_poses, out_dir.display());
            0
        }
        Err(e) => {
            eprintln!("stage-replay: {e}");
            2
        }
    }
}

fn select_profile(geometry: &CbctGeometry, id: &str) -> Result<RigidProfile, String> {
    // Explicit selection only; there is deliberately NO default arm.
    let (profile, thickness) = match id {
        PROFILE_ALL_THIN => (geometry.experimental_nov12_all_thin_profile_with_reviewed_cad()?, [3.175; 3]),
        PROFILE_HISTORICAL => (
            geometry.experimental_candidate_profile_with_reviewed_cad()?,
            [3.175, 3.175, 6.35],
        ),
        other => return Err(format!("Unknown or missing profile_id {other:?}; refusing to fall back")),
    };
    for (m, expected) in profile.magnets.iter().zip(thickness) {
        if m.diameter_mm != 7.9375 || m.thickness_mm != expected {
            return Err(format!("Profile {id} does not have the expected cylinder geometry"));
        }
    }
    Ok(profile)
}

fn create_new_file(path: &Path, text: &str) -> Result<(), String> {
    use std::io::Write;
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| format!("Cannot create {}: {e}", path.display()))?;
    file.write_all(text.as_bytes()).map_err(|e| format!("Cannot write {}: {e}", path.display()))
}

pub fn replay(input_path: &Path, out_dir: &Path) -> Result<Outcome, String> {
    if out_dir.exists() {
        return Err(format!("Refusing to overwrite existing result directory {}", out_dir.display()));
    }
    let input_bytes = std::fs::read(input_path).map_err(|e| format!("Read replay input: {e}"))?;
    let input_sha = sha256_hex(&input_bytes);
    let data: ReplayInput =
        serde_json::from_slice(&input_bytes).map_err(|e| format!("Replay input schema error: {e}"))?;
    if data.format != INPUT_FORMAT {
        return Err(format!("Unsupported replay input format {:?}", data.format));
    }
    if !data.cad_mapping_reviewed_by_operator || !data.magnet_dimensions_operator_confirmed {
        return Err("Operator attestation of CAD/firmware mapping review and magnet dimensions is required".into());
    }
    if data.clock_basis != "device_clock_relative_to_first_raw_base_us" {
        return Err("Unsupported clock basis".into());
    }
    if data.range_guard_fraction != 0.9 || data.fit_frame_stride != FIT_STRIDE {
        return Err("Only the existing prespecified 90% range guard and 10-frame fit stride are supported".into());
    }
    let w = data.calibration_window_s;
    if !(w[0].is_finite() && w[1].is_finite() && w[0] >= 0.0 && w[1] - w[0] >= 1.0) {
        return Err("Calibration window must be finite, nonnegative and at least 1 s long".into());
    }
    if data.frames.is_empty() || data.sensor_masks.iter().any(|m| m.count_ones() < 8) {
        return Err("Need frames and at least 8 valid sensors per board".into());
    }
    for f in &data.frames {
        if f.board_id > 2 || f.sensor_mask & !data.sensor_masks[f.board_id as usize] != 0 {
            return Err("A frame mask cannot reinstate a sensor excluded from calibration".into());
        }
        if f.fields_ut.iter().flatten().any(|v| !v.is_finite()) {
            return Err("Nonfinite Hall field in replay input".into());
        }
    }
    let ct_bytes = std::fs::read(&data.ct_detector_json_path)
        .map_err(|e| format!("Read CT detector JSON: {e}"))?;
    if sha256_hex(&ct_bytes) != data.ct_detector_sha256.to_ascii_lowercase() {
        return Err("CT detector JSON SHA-256 does not match the replay input".into());
    }
    let geometry = CbctGeometry::from_detector_json(
        &String::from_utf8(ct_bytes).map_err(|e| format!("CT detector JSON is not UTF-8: {e}"))?,
        PathBuf::from(&data.ct_detector_json_path),
        PathBuf::from("stage-replay-no-qc-directory"),
    )?;
    let mut profile = select_profile(&geometry, &data.profile_id)?;
    let masks = data.sensor_masks;

    // PRE-only calibration with the existing routines; frozen afterwards.
    let background_treatment = if data.calibration_background_raw_mt.is_some() {
        "measured_per_sensor_saved_background"
    } else {
        "no_saved_background_nine_PRE_fitted_constant_board_components"
    };
    let calibrated = match data.calibration_background_raw_mt.as_ref() {
        Some(bg) => fit_magnet_strengths_masked(&profile, bg, &data.calibration_stationary_raw_mt, masks)
            .map(|(s, rmse)| (s, [[0.0; 3]; 3], rmse)),
        None => fit_strengths_with_board_offsets_masked(&profile, &data.calibration_stationary_raw_mt, masks),
    };
    let magnet_geometry = |p: &RigidProfile, strengths: Option<[f64; 3]>| {
        p.magnets
            .iter()
            .enumerate()
            .map(|(i, m)| {
                serde_json::json!({
                    "center_lps_mm": m.center_mm, "unsigned_ct_axis_lps": m.axis,
                    "diameter_mm": m.diameter_mm, "thickness_mm": m.thickness_mm,
                    "fitted_effective_signed_remanence_mt": strengths.map(|s| s[i]),
                })
            })
            .collect::<Vec<_>>()
    };
    let mut report = serde_json::json!({
        "format": REPORT_FORMAT,
        "status": "RESEARCH_ONLY_NOT_PHYSICALLY_VALIDATED_STAGE_REPLAY",
        "solver_crate_version": env!("CARGO_PKG_VERSION"),
        "profile_id": data.profile_id,
        "profile_name": profile.name,
        "sources": {
            "replay_input_sha256": input_sha,
            "ct_detector_json_sha256": data.ct_detector_sha256,
            "ct_detector_json_path": data.ct_detector_json_path,
        },
        "input_provenance_passthrough": data.provenance,
        "stage_truth_fields_accepted_by_input_schema": false,
        "pose_convention": pose_convention(&profile),
        "magnets_before_calibration": magnet_geometry(&profile, None),
        "profile_limits": {"max_rmse_mt": profile.max_rmse_mt,
            "lower_bounds": profile.lower_bounds, "upper_bounds": profile.upper_bounds},
    });
    std::fs::create_dir_all(out_dir.parent().unwrap_or_else(|| Path::new(".")))
        .map_err(|e| format!("Create parent directory: {e}"))?;
    std::fs::create_dir(out_dir).map_err(|e| format!("Create result directory: {e}"))?;

    let (strengths, offsets, calibration_rmse) = match calibrated {
        Ok(v) => v,
        Err(reason) => {
            report["calibration"] = serde_json::json!({"qc_pass": false, "reason": reason});
            report["fit_status"] = serde_json::json!("stationary_calibration_failed");
            create_new_file(&out_dir.join("solver_report.json"), &(serde_json::to_string_pretty(&report).unwrap() + "\n"))?;
            return Ok(Outcome { calibration_passed: false, accepted_poses: 0 });
        }
    };
    let plausible = strengths.iter().all(|v| v.abs() >= 300.0 && v.abs() <= 2200.0);
    let gate = calibration_rmse <= profile.max_rmse_mt && plausible;
    report["calibration"] = serde_json::json!({
        "window_s": data.calibration_window_s,
        "background_treatment": background_treatment,
        "sensor_masks": masks,
        "fitted_effective_signed_remanence_mt": strengths,
        "pre_fitted_constant_board_offsets_lps_mt": offsets,
        "rmse_mt": calibration_rmse,
        "gate_max_rmse_mt": profile.max_rmse_mt,
        "gate_strength_abs_range_mt": [300.0, 2200.0],
        "strengths_plausible": plausible,
        "qc_pass": gate,
        "frozen_before_motion_reconstruction": true,
        "uses_post_ct_stage_angle_or_optical_data": false,
        "note": "Effective model amplitudes; magnet grade, remanence and signed magnetization axes are NOT independently verified.",
    });
    report["magnets_after_calibration"] = serde_json::json!(magnet_geometry(&profile, Some(strengths)));
    if !gate {
        report["fit_status"] = serde_json::json!("calibration_QC_FAILED_pose_fit_not_run");
        create_new_file(&out_dir.join("solver_report.json"), &(serde_json::to_string_pretty(&report).unwrap() + "\n"))?;
        return Ok(Outcome { calibration_passed: false, accepted_poses: 0 });
    }
    for (m, s) in profile.magnets.iter_mut().zip(strengths) {
        m.remanence_mt = s;
    }
    for b in &mut profile.boards {
        b.background_mt = offsets[b.raw_board_id as usize];
    }
    let origin = profile.origin_mm;
    let max_rmse = profile.max_rmse_mt;
    let mut live = RigidLiveFit::default();
    live.set_experimental_gk_profile(profile, data.calibration_background_raw_mt);
    live.set_presence(BoardPresence { board_mask: 0b111, sensor_masks: masks });

    let mut poses = Vec::<serde_json::Value>::new();
    let mut rejected = Vec::<serde_json::Value>::new();
    let mut accepted_t = Vec::<f64>::new();
    let (mut attempts, mut stride_without_job) = (0_usize, 0_usize);
    for src in &data.frames {
        let mut frame = BoardFrame::default();
        frame.board_id = src.board_id;
        frame.frame_id = src.frame_id;
        frame.base_time_us = src.base_time_us;
        frame.sensor_mask = src.sensor_mask;
        for i in 0..16 {
            frame.samples[i].bx_ut = src.fields_ut[i][0] as f32;
            frame.samples[i].by_ut = src.fields_ut[i][1] as f32;
            frame.samples[i].bz_ut = src.fields_ut[i][2] as f32;
            frame.samples[i].time_offset_us = src.offsets_us[i];
        }
        let eligible = src.frame_id % FIT_STRIDE == 0;
        let Some(job) = live.receive_frame(&frame, eligible) else {
            if eligible {
                stride_without_job += 1;
            }
            continue;
        };
        attempts += 1;
        let completed = job.run();
        let time_s = completed.time_us as f64 / 1e6;
        let result = completed.result.clone();
        let rejected_before = live.summary().rejected;
        live.apply_completion(completed);
        if live.summary().rejected != rejected_before {
            let (reason, rmse) = match &result {
                None => ("solver_failed", None),
                Some(r) => ("field_rmse_above_gate", Some(r.rmse_mt)),
            };
            rejected.push(serde_json::json!({"t_s": time_s, "reason": reason, "field_rmse_mt": rmse}));
            continue;
        }
        if let Some(r) = result {
            accepted_t.push(time_s);
            poses.push(serde_json::json!({
                "t_s": time_s, "pose_mm_rad": r.pose,
                "field_rmse_mt": r.rmse_mt, "sensor_count": r.sensor_count, "fitted": true,
            }));
        }
    }
    let gaps = accepted_gaps(&accepted_t);
    let n = poses.len();
    let qc = if n >= MIN_ACCEPTED_FOR_SCORER { "PASS_ENOUGH_ACCEPTED_POSES" } else { "INSUFFICIENT_ACCEPTED_POSES_FOR_SCORER" };
    report["fit_status"] = serde_json::json!("pose_replay_completed");
    report["replay"] = serde_json::json!({
        "frames": data.frames.len(), "fit_stride_frames": FIT_STRIDE, "fit_attempts": attempts,
        "stride_frames_without_fit_job": stride_without_job,
        "accepted_poses": n, "rejected_fits": rejected.len(), "rejected": rejected,
        "accepted_pose_gaps": gaps,
        "gap_policy": "A gap is an interval between accepted poses longer than 3x the median spacing. Missing poses are NEVER interpolated or filled.",
        "board_field_time_alignment": "existing live-fit synchronisation interpolates raw board FIELDS to a common time (<=120 ms bracket); poses are never interpolated",
        "qc": qc,
        "mean_field_rmse_mt": if n == 0 { None } else {
            Some(poses.iter().map(|p| p["field_rmse_mt"].as_f64().unwrap()).sum::<f64>() / n as f64) },
        "max_rmse_gate_mt": max_rmse,
    });
    let frozen = serde_json::json!({
        "format": POSES_FORMAT,
        "status": "RESEARCH_ONLY_CANDIDATE_TRAJECTORY_NOT_PHYSICALLY_VALIDATED",
        "profile_id": data.profile_id,
        "not_optical_hdmm_measurement": true,
        "timestamps_are_relative_to_original_device_clock": true,
        "mask_and_synthetic_gap_interpolation": false,
        "stage_truth_used": false,
        "replay_input_sha256": input_sha,
        "initial_magnet_centroid_pre_lps_mm": origin,
        "pose_convention": pose_convention_static(),
        "poses": poses,
    });
    create_new_file(&out_dir.join("frozen_poses.json"), &(serde_json::to_string(&frozen).unwrap() + "\n"))?;
    create_new_file(&out_dir.join("solver_report.json"), &(serde_json::to_string_pretty(&report).unwrap() + "\n"))?;
    Ok(Outcome { calibration_passed: true, accepted_poses: n })
}

fn pose_convention_static() -> serde_json::Value {
    serde_json::json!({
        "pose_mm_rad": "[tx,ty,tz in mm, rx,ry,rz rotation vector in rad]",
        "frame": "DICOM_LPS_mm (same CT frame as the source detector geometry)",
        "absolute_not_PRE_relative": true,
        "identity_pose": "magnets and receivers at the source-CT geometry",
        "point_mapping": "x_lps = origin + t + Rodrigues(rotvec) * (x_ct - origin)",
        "no_frame_or_sign_transform_applied": true,
    })
}

fn pose_convention(profile: &RigidProfile) -> serde_json::Value {
    let mut v = pose_convention_static();
    v["origin_lps_mm"] = serde_json::json!(profile.origin_mm);
    v
}

fn accepted_gaps(t: &[f64]) -> Vec<serde_json::Value> {
    if t.len() < 3 {
        return Vec::new();
    }
    let mut dt: Vec<f64> = t.windows(2).map(|w| w[1] - w[0]).collect();
    dt.sort_by(f64::total_cmp);
    let median = dt[dt.len() / 2];
    t.windows(2)
        .filter(|w| w[1] - w[0] > GAP_FACTOR * median)
        .map(|w| serde_json::json!({"after_accepted_pose_s": w[0], "before_next_accepted_pose_s": w[1]}))
        .collect()
}

/// Minimal FIPS 180-4 SHA-256 (no new dependency; verified against known vectors).
pub fn sha256_hex(data: &[u8]) -> String {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    ];
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
    ];
    let mut message = data.to_vec();
    message.push(0x80);
    while message.len() % 64 != 56 {
        message.push(0);
    }
    message.extend_from_slice(&((data.len() as u64) * 8).to_be_bytes());
    for chunk in message.chunks_exact(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes(chunk[4 * i..4 * i + 4].try_into().unwrap());
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16].wrapping_add(s0).wrapping_add(w[i - 7]).wrapping_add(s1);
        }
        let mut v = h;
        for i in 0..64 {
            let s1 = v[4].rotate_right(6) ^ v[4].rotate_right(11) ^ v[4].rotate_right(25);
            let ch = (v[4] & v[5]) ^ (!v[4] & v[6]);
            let t1 = v[7].wrapping_add(s1).wrapping_add(ch).wrapping_add(K[i]).wrapping_add(w[i]);
            let s0 = v[0].rotate_right(2) ^ v[0].rotate_right(13) ^ v[0].rotate_right(22);
            let maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
            let t2 = s0.wrapping_add(maj);
            v = [t1.wrapping_add(t2), v[0], v[1], v[2], v[3].wrapping_add(t1), v[4], v[5], v[6]];
        }
        for i in 0..8 {
            h[i] = h[i].wrapping_add(v[i]);
        }
    }
    h.iter().map(|x| format!("{x:08x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    const TRUTH_STRENGTHS: [f64; 3] = [-1600.0, 1400.0, -650.0];
    const RAW_OFFSET_UT: [[f64; 3]; 3] = [[30.0, -20.0, 10.0], [-15.0, 25.0, 5.0], [8.0, 12.0, -30.0]];
    const AXIS: [f64; 3] = [0.6, 0.0, 0.8];
    const PIVOT_SHIFT: [f64; 3] = [5.0, -10.0, 3.0];

    fn angle_deg(t: f64, scale: f64) -> f64 {
        let knots = [(0.0, 0.0), (8.0, 0.0), (10.0, 2.0), (14.0, 2.0), (16.0, 0.0), (18.0, 0.0),
                     (20.0, -3.0), (24.0, -3.0), (26.0, 0.0), (30.0, 0.0)];
        for w in knots.windows(2) {
            if t >= w[0].0 && t <= w[1].0 {
                return scale * (w[0].1 + (w[1].1 - w[0].1) * (t - w[0].0) / (w[1].0 - w[0].0));
            }
        }
        0.0
    }

    fn rot(v: [f64; 3]) -> [[f64; 3]; 3] {
        let a = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if a < 1e-15 { return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]; }
        let k = [v[0] / a, v[1] / a, v[2] / a];
        let (s, c) = (a.sin(), a.cos());
        let kx = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]];
        let mut r = [[0.0; 3]; 3];
        for i in 0..3 { for j in 0..3 {
            let k2: f64 = (0..3).map(|m| kx[i][m] * kx[m][j]).sum();
            r[i][j] = if i == j { 1.0 } else { 0.0 } + s * kx[i][j] + (1.0 - c) * k2;
        }}
        r
    }

    fn truth_pose(origin: [f64; 3], t: f64, scale: f64) -> [f64; 6] {
        let theta = angle_deg(t, scale).to_radians();
        let rv = [AXIS[0] * theta, AXIS[1] * theta, AXIS[2] * theta];
        let r = rot(rv);
        let pivot = [origin[0] + PIVOT_SHIFT[0], origin[1] + PIVOT_SHIFT[1], origin[2] + PIVOT_SHIFT[2]];
        let d = [origin[0] - pivot[0], origin[1] - pivot[1], origin[2] - pivot[2]];
        let moved: Vec<f64> = (0..3).map(|i| pivot[i] + (0..3).map(|j| r[i][j] * d[j]).sum::<f64>() - origin[i]).collect();
        [moved[0], moved[1], moved[2], rv[0], rv[1], rv[2]]
    }

    struct Fixture {
        dir: PathBuf,
        ct_path: PathBuf,
        profile: RigidProfile,
        input: Value,
    }

    #[derive(Clone, Copy)]
    struct Options { angle_scale: f64, corrupt_motion: bool, corrupt_calibration: bool }

    fn dir_for(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("gk_stage_replay_test_{name}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn build(name: &str, profile_id: &str, opt: Options) -> Fixture {
        let dir = dir_for(name);
        let ct_text = crate::cbct_geometry::tests::synthetic_reviewed_cad_report().to_string();
        let ct_path = dir.join("synthetic_ct_detector.json");
        std::fs::write(&ct_path, &ct_text).unwrap();
        let geometry = CbctGeometry::from_detector_json(&ct_text, ct_path.clone(), "qc".into()).unwrap();
        // Synthetic TRUTH always uses the physically corrected all-thin geometry.
        let mut truth = geometry.experimental_nov12_all_thin_profile_with_reviewed_cad().unwrap();
        for (m, s) in truth.magnets.iter_mut().zip(TRUTH_STRENGTHS) { m.remanence_mt = s; }
        let origin = truth.origin_mm;
        let mut frames = Vec::new();
        let n = 600_usize;
        for b in 0..3_u16 {
            let board = truth.boards.iter().find(|x| x.raw_board_id == b).unwrap();
            let m = board.raw_to_world;
            for k in 0..n {
                let t = k as f64 * 0.05 + b as f64 * 0.01;
                let world = crate::rigid_fit::forward_board_fields_mt(&truth, truth_pose(origin, t, opt.angle_scale))[b as usize];
                let mut fields = [[0.0; 3]; 16];
                for i in 0..16 {
                    for a in 0..3 {
                        // raw = M^T * world (verified orthonormal below), mT -> uT, plus a raw board offset
                        let r: f64 = (0..3).map(|c| m[c][a] * world[i][c]).sum();
                        fields[i][a] = r * 1000.0 + RAW_OFFSET_UT[b as usize][a];
                    }
                    let in_cal = t >= 1.0 && t <= 7.0;
                    let in_bad_motion = t >= 20.1 && t <= 20.6;
                    if (opt.corrupt_motion && in_bad_motion && i < 8)
                        || (opt.corrupt_calibration && in_cal && i < 6) {
                        fields[i][0] += 6000.0 * if i % 2 == 0 { 1.0 } else { -1.0 };
                        fields[i][1] -= 5000.0;
                    }
                }
                frames.push(json!({
                    "board_id": b, "frame_id": k, "base_time_us": (t * 1e6).round() as u64,
                    "sensor_mask": 0xffffu32, "offsets_us": (0..16).map(|i| i * 100).collect::<Vec<_>>(),
                    "fields_ut": fields,
                }));
            }
        }
        frames.sort_by_key(|f| (f["base_time_us"].as_u64().unwrap(), f["board_id"].as_u64().unwrap()));
        // PRE calibration medians in mT from the declared window only.
        let mut cal = [[[0.0_f64; 3]; 16]; 3];
        for b in 0..3 {
            let sel: Vec<&Value> = frames.iter().filter(|f| {
                let t = f["base_time_us"].as_f64().unwrap() / 1e6;
                f["board_id"].as_u64().unwrap() == b as u64 && t >= 1.0 && t <= 7.0
            }).collect();
            for i in 0..16 { for a in 0..3 {
                let mut v: Vec<f64> = sel.iter().map(|f| f["fields_ut"][i][a].as_f64().unwrap()).collect();
                v.sort_by(f64::total_cmp);
                cal[b][i][a] = v[v.len() / 2] / 1000.0;
            }}
        }
        let input = json!({
            "format": INPUT_FORMAT, "profile_id": profile_id,
            "cad_mapping_reviewed_by_operator": true, "magnet_dimensions_operator_confirmed": true,
            "ct_detector_json_path": ct_path.to_string_lossy(),
            "ct_detector_sha256": sha256_hex(ct_text.as_bytes()),
            "clock_basis": "device_clock_relative_to_first_raw_base_us",
            "calibration_window_s": [1.0, 7.0], "range_guard_fraction": 0.9, "fit_frame_stride": 10,
            "sensor_masks": [0xffffu32, 0xffffu32, 0xffffu32], "calibration_stationary_raw_mt": cal,
            "calibration_background_raw_mt": Value::Null, "frames": frames,
            "provenance": {"note": "synthetic"},
        });
        Fixture { dir, ct_path, profile: truth, input }
    }

    const CLEAN: Options = Options { angle_scale: 1.0, corrupt_motion: false, corrupt_calibration: false };

    fn run(fx: &Fixture, input: &Value, out: &str) -> Result<(Outcome, PathBuf), String> {
        let path = fx.dir.join(format!("{out}_input.json"));
        std::fs::write(&path, serde_json::to_string(input).unwrap()).unwrap();
        let out_dir = fx.dir.join(out);
        replay(&path, &out_dir).map(|o| (o, out_dir))
    }

    fn read(dir: &Path, file: &str) -> Value {
        serde_json::from_str(&std::fs::read_to_string(dir.join(file)).unwrap()).unwrap()
    }

    #[test]
    fn sha256_matches_known_vectors() {
        assert_eq!(sha256_hex(b""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        assert_eq!(sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        let long = vec![b'a'; 1000];
        assert_eq!(sha256_hex(&long).len(), 64);
    }

    #[test]
    fn all_thin_replay_recovers_synthetic_rotation_and_freezes_calibration() {
        let fx = build("recover", PROFILE_ALL_THIN, CLEAN);
        let (outcome, out) = run(&fx, &fx.input, "run").unwrap();
        assert!(outcome.calibration_passed && outcome.accepted_poses >= 40, "{}", outcome.accepted_poses);
        let report = read(&out, "solver_report.json");
        let poses = read(&out, "frozen_poses.json");
        // Explicit all-thin profile, reported geometry.
        assert_eq!(report["profile_id"], PROFILE_ALL_THIN);
        for m in report["magnets_after_calibration"].as_array().unwrap() {
            assert_eq!(m["thickness_mm"], 3.175);
            assert_eq!(m["diameter_mm"], 7.9375);
        }
        // PRE-only fitted strengths recover the synthetic truth and are frozen.
        let s: Vec<f64> = report["calibration"]["fitted_effective_signed_remanence_mt"]
            .as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
        for (fit, truth) in s.iter().zip(TRUTH_STRENGTHS) { assert!((fit / truth - 1.0).abs() < 0.01, "{fit} {truth}"); }
        assert_eq!(report["calibration"]["frozen_before_motion_reconstruction"], true);
        assert_eq!(report["calibration"]["uses_post_ct_stage_angle_or_optical_data"], false);
        // Scorer-required provenance keys and absolute-pose convention.
        assert_eq!(poses["format"], POSES_FORMAT);
        assert_eq!(poses["not_optical_hdmm_measurement"], true);
        assert_eq!(poses["timestamps_are_relative_to_original_device_clock"], true);
        assert_eq!(poses["stage_truth_used"], false);
        assert_eq!(poses["pose_convention"]["absolute_not_PRE_relative"], true);
        assert!(poses["initial_magnet_centroid_pre_lps_mm"].is_array());
        // Reconstructed rotation about the known axis matches the synthetic stage angle.
        let origin = fx.profile.origin_mm;
        let mut checked = 0;
        for p in poses["poses"].as_array().unwrap() {
            assert_eq!(p["fitted"], true);
            let t = p["t_s"].as_f64().unwrap();
            let truth = truth_pose(origin, t, 1.0);
            let fit: Vec<f64> = p["pose_mm_rad"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
            // Timing may differ by up to one board stagger; test only on the plateaus.
            if (11.0..13.5).contains(&t) || (21.0..23.5).contains(&t) {
                let proj: f64 = (0..3).map(|i| fit[3 + i] * AXIS[i]).sum();
                let want: f64 = (0..3).map(|i| truth[3 + i] * AXIS[i]).sum();
                assert!((proj - want).abs().to_degrees() < 0.05, "t={t} {proj} {want}");
                for i in 0..3 { assert!((fit[i] - truth[i]).abs() < 0.1, "t={t} axis {i}"); }
                checked += 1;
            }
        }
        assert!(checked >= 8, "{checked}");
    }

    #[test]
    fn historical_profile_is_only_an_explicit_control_never_a_silent_fallback() {
        let fx = build("hist", PROFILE_ALL_THIN, CLEAN);
        let mut hist = fx.input.clone();
        hist["profile_id"] = json!(PROFILE_HISTORICAL);
        let (_, out) = run(&fx, &hist, "hist").unwrap();
        let thick: Vec<f64> = read(&out, "solver_report.json")["magnets_after_calibration"]
            .as_array().unwrap().iter().map(|m| m["thickness_mm"].as_f64().unwrap()).collect();
        assert_eq!(thick, vec![3.175, 3.175, 6.35]);
        let (_, thin_out) = run(&fx, &fx.input, "thin").unwrap();
        let strengths = |d: &Path| -> Vec<f64> { read(d, "solver_report.json")["calibration"]["fitted_effective_signed_remanence_mt"]
            .as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect() };
        let (hs, ts) = (strengths(&out), strengths(&thin_out));
        // Data were generated with all-thin cylinders. PRE RMSE cannot tell the thicknesses apart (strengths
        // absorb it), but the thick model needs about half the superior strength for the same moment.
        assert!((ts[2] / TRUTH_STRENGTHS[2] - 1.0).abs() < 0.01, "{ts:?}");
        assert!(hs[2].abs() / ts[2].abs() < 0.6, "thick {hs:?} thin {ts:?}");
        for bad in [json!(""), json!("default"), json!("nov12"), Value::Null] {
            let mut x = fx.input.clone();
            x["profile_id"] = bad;
            assert!(run(&fx, &x, "bad_profile").is_err());
            let _ = std::fs::remove_dir_all(fx.dir.join("bad_profile"));
        }
        let mut missing = fx.input.clone();
        missing.as_object_mut().unwrap().remove("profile_id");
        assert!(run(&fx, &missing, "missing_profile").unwrap_err().contains("schema"));
    }

    #[test]
    fn stage_truth_optical_and_postct_fields_are_rejected_by_the_input_schema() {
        let fx = build("leak", PROFILE_ALL_THIN, CLEAN);
        for (i, key) in ["stage_angle_deg", "plateau_log", "optical_hdmm_xyz_mm", "post_ct_endpoint_pose",
                         "stage_score_target", "measured_deg"].iter().enumerate() {
            let mut x = fx.input.clone();
            x[*key] = json!(2.0);
            let e = run(&fx, &x, &format!("leak{i}")).unwrap_err();
            assert!(e.contains("schema") && e.contains(key), "{e}");
            assert!(!fx.dir.join(format!("leak{i}")).exists());
        }
        let mut frame_leak = fx.input.clone();
        frame_leak["frames"][0]["stage_angle_deg"] = json!(1.0);
        assert!(run(&fx, &frame_leak, "frame_leak").unwrap_err().contains("schema"));
    }

    #[test]
    fn provenance_text_cannot_change_reconstruction() {
        let fx = build("prov", PROFILE_ALL_THIN, CLEAN);
        let (_, a) = run(&fx, &fx.input, "a").unwrap();
        let mut other = fx.input.clone();
        other["provenance"] = json!({"stage_angle_deg": [1, 2, 3], "plateau": "heldout", "hdmm": 99});
        let (_, b) = run(&fx, &other, "b").unwrap();
        assert_eq!(read(&a, "frozen_poses.json")["poses"], read(&b, "frozen_poses.json")["poses"]);
        assert_eq!(read(&a, "solver_report.json")["calibration"], read(&b, "solver_report.json")["calibration"]);
    }

    #[test]
    fn calibration_is_fitted_from_pre_window_only_so_later_motion_cannot_change_it() {
        let a = build("motion_a", PROFILE_ALL_THIN, CLEAN);
        let b = build("motion_b", PROFILE_ALL_THIN, Options { angle_scale: 2.0, ..CLEAN });
        assert_ne!(a.input["frames"], b.input["frames"]);
        let (_, ra) = run(&a, &a.input, "run").unwrap();
        let (_, rb) = run(&b, &b.input, "run").unwrap();
        assert_eq!(read(&ra, "solver_report.json")["calibration"], read(&rb, "solver_report.json")["calibration"]);
    }

    #[test]
    fn rejected_fits_are_reported_and_missing_poses_are_not_interpolated() {
        let fx = build("reject", PROFILE_ALL_THIN, Options { corrupt_motion: true, ..CLEAN });
        let (_, out) = run(&fx, &fx.input, "run").unwrap();
        let report = read(&out, "solver_report.json");
        let rep = &report["replay"];
        let rejected = rep["rejected"].as_array().unwrap();
        assert!(!rejected.is_empty(), "corrupted fields must be rejected");
        let poses = read(&out, "frozen_poses.json");
        let times: Vec<f64> = poses["poses"].as_array().unwrap().iter().map(|p| p["t_s"].as_f64().unwrap()).collect();
        for r in rejected {
            let t = r["t_s"].as_f64().unwrap();
            assert!((19.9..21.0).contains(&t), "{t}");
            assert!(!times.iter().any(|x| (x - t).abs() < 1e-9), "rejected pose exported");
        }
        assert!(!times.iter().any(|t| (20.2..20.5).contains(t)), "pose invented/kept inside the bad interval");
        assert_eq!(rep["accepted_poses"].as_u64().unwrap() as usize, times.len());
        assert_eq!(rep["fit_attempts"].as_u64().unwrap() as usize, times.len() + rejected.len());
        assert_eq!(poses["mask_and_synthetic_gap_interpolation"], false);
        assert!(rep["gap_policy"].as_str().unwrap().contains("NEVER interpolated"));
    }

    #[test]
    fn calibration_failure_exports_no_poses() {
        let fx = build("calfail", PROFILE_ALL_THIN, Options { corrupt_calibration: true, ..CLEAN });
        let (outcome, out) = run(&fx, &fx.input, "run").unwrap();
        assert!(!outcome.calibration_passed);
        assert!(out.join("solver_report.json").is_file());
        assert!(!out.join("frozen_poses.json").exists());
        assert_eq!(read(&out, "solver_report.json")["calibration"]["qc_pass"], false);
    }

    #[test]
    fn provenance_and_overwrite_guards() {
        let fx = build("guards", PROFILE_ALL_THIN, CLEAN);
        let (_, _) = run(&fx, &fx.input, "run").unwrap();
        assert!(run(&fx, &fx.input, "run").unwrap_err().contains("Refusing to overwrite"));
        let mut tampered = fx.input.clone();
        tampered["ct_detector_sha256"] = json!("0".repeat(64));
        assert!(run(&fx, &tampered, "tampered").unwrap_err().contains("SHA-256"));
        let mut edited = fx.input.clone();
        std::fs::write(&fx.ct_path, b"{}").unwrap();
        edited["provenance"] = json!({});
        assert!(run(&fx, &edited, "edited").is_err());
        let mut unattested = fx.input.clone();
        unattested["cad_mapping_reviewed_by_operator"] = json!(false);
        assert!(run(&fx, &unattested, "unattested").unwrap_err().contains("attestation"));
        let mut loose = fx.input.clone();
        loose["range_guard_fraction"] = json!(1.0);
        assert!(run(&fx, &loose, "loose").unwrap_err().contains("prespecified"));
        let mut reinstated = fx.input.clone();
        reinstated["sensor_masks"] = json!([0x00ffu32, 0xffffu32, 0xffffu32]);
        assert!(run(&fx, &reinstated, "reinstated").unwrap_err().contains("reinstate"));
    }

    /// Independent re-statement of the existing November harness procedure
    /// (calibrate -> freeze -> RigidLiveFit stride loop). The stage replayer
    /// must reproduce its accepted poses exactly on the same fixture.
    #[test]
    fn replayer_matches_the_existing_finite_cylinder_harness_on_a_matched_fixture() {
        let fx = build("match", PROFILE_ALL_THIN, CLEAN);
        let (_, out) = run(&fx, &fx.input, "run").unwrap();
        let replayed = read(&out, "frozen_poses.json");
        let ct_text = std::fs::read_to_string(&fx.ct_path).unwrap();
        let geometry = CbctGeometry::from_detector_json(&ct_text, fx.ct_path.clone(), "qc".into()).unwrap();
        let mut profile = geometry.experimental_nov12_all_thin_profile_with_reviewed_cad().unwrap();
        // Parse the very same bytes the replayer read (serde_json float parsing is not guaranteed bit-exact across Value round trips).
        let file_input: Value = serde_json::from_slice(&std::fs::read(fx.dir.join("run_input.json")).unwrap()).unwrap();
        let cal: [[[f64; 3]; 16]; 3] = serde_json::from_value(file_input["calibration_stationary_raw_mt"].clone()).unwrap();
        let (strengths, offsets, _) = fit_strengths_with_board_offsets_masked(&profile, &cal, [0xffff; 3]).unwrap();
        for (m, s) in profile.magnets.iter_mut().zip(strengths) { m.remanence_mt = s; }
        for b in &mut profile.boards { b.background_mt = offsets[b.raw_board_id as usize]; }
        let mut live = RigidLiveFit::default();
        live.set_experimental_gk_profile(profile, None);
        live.set_presence(BoardPresence { board_mask: 0b111, sensor_masks: [0xffff; 3] });
        let mut expected = Vec::new();
        for src in file_input["frames"].as_array().unwrap() {
            let mut frame = BoardFrame::default();
            frame.board_id = src["board_id"].as_u64().unwrap() as u16;
            frame.frame_id = src["frame_id"].as_u64().unwrap() as u32;
            frame.base_time_us = src["base_time_us"].as_u64().unwrap();
            frame.sensor_mask = 0xffff;
            for i in 0..16 {
                frame.samples[i].bx_ut = src["fields_ut"][i][0].as_f64().unwrap() as f32;
                frame.samples[i].by_ut = src["fields_ut"][i][1].as_f64().unwrap() as f32;
                frame.samples[i].bz_ut = src["fields_ut"][i][2].as_f64().unwrap() as f32;
                frame.samples[i].time_offset_us = src["offsets_us"][i].as_u64().unwrap() as u16;
            }
            let Some(job) = live.receive_frame(&frame, frame.frame_id % 10 == 0) else { continue };
            let done = job.run();
            let t = done.time_us as f64 / 1e6;
            let pose = done.result.as_ref().map(|r| r.pose);
            let before = live.summary().rejected;
            live.apply_completion(done);
            if live.summary().rejected == before { expected.push((t, pose.unwrap())); }
        }
        let got = replayed["poses"].as_array().unwrap();
        assert_eq!(got.len(), expected.len());
        for (g, (t, pose)) in got.iter().zip(expected) {
            // Identical computation; tolerance only covers JSON float re-parsing (1 ULP).
            assert!((g["t_s"].as_f64().unwrap() - t).abs() < 1e-12);
            for i in 0..6 { assert!((g["pose_mm_rad"][i].as_f64().unwrap() - pose[i]).abs() < 1e-12); }
        }
    }

    #[test]
    fn truth_fixture_uses_orthonormal_boards_so_raw_conversion_is_exact() {
        let fx = build("ortho", PROFILE_ALL_THIN, CLEAN);
        for b in &fx.profile.boards {
            for i in 0..3 { for j in 0..3 {
                let d: f64 = (0..3).map(|k| b.raw_to_world[i][k] * b.raw_to_world[j][k]).sum();
                assert!((d - if i == j { 1.0 } else { 0.0 }).abs() < 1e-9);
            }}
        }
    }

    fn varuint(mut v: u64, out: &mut Vec<u8>) {
        while v > 127 { out.push(((v & 127) | 128) as u8); v >>= 7; }
        out.push(v as u8);
    }

    /// Writes a SYNTHETIC MLXBF002 recording plus its CT geometry and truth timeline for the
    /// research repository's end-to-end test (raw bin -> Python prep -> this solver -> scorer).
    /// No-op unless GK_STAGE_REPLAY_FIXTURE_DIR is set.
    #[test]
    fn write_stage_replay_e2e_fixture_when_requested() {
        let Ok(dir) = std::env::var("GK_STAGE_REPLAY_FIXTURE_DIR") else { return };
        let dir = PathBuf::from(dir);
        std::fs::create_dir_all(&dir).unwrap();
        let fx = build("e2e_fixture", PROFILE_ALL_THIN, CLEAN);
        let mut bin = b"MLXBF002".to_vec();
        for f in fx.input["frames"].as_array().unwrap() {
            let mut p = Vec::new();
            varuint(f["board_id"].as_u64().unwrap(), &mut p);
            varuint(f["frame_id"].as_u64().unwrap() + 1880, &mut p);
            varuint(0xffff, &mut p);
            varuint(f["base_time_us"].as_u64().unwrap() + 10_000_000, &mut p);
            for i in 0..16 {
                for a in 0..3 { p.extend_from_slice(&(f["fields_ut"][i][a].as_f64().unwrap() as f32).to_le_bytes()); }
                p.extend_from_slice(&22.0_f32.to_le_bytes());
                varuint(f["offsets_us"][i].as_u64().unwrap(), &mut p);
            }
            bin.extend_from_slice(&(p.len() as u32).to_le_bytes());
            bin.extend_from_slice(&p);
        }
        std::fs::write(dir.join("synthetic_movement.bin"), bin).unwrap();
        std::fs::copy(&fx.ct_path, dir.join("synthetic_ct_detector.json")).unwrap();
        let o = fx.profile.origin_mm;
        std::fs::write(dir.join("truth.json"), serde_json::to_string(&json!({
            "origin_lps_mm": o, "axis_lps": AXIS,
            "pivot_lps_mm": [o[0] + PIVOT_SHIFT[0], o[1] + PIVOT_SHIFT[1], o[2] + PIVOT_SHIFT[2]],
            "synthetic_angle_deg_at": [[11.0, 2.0], [13.5, 2.0], [21.0, -3.0], [23.5, -3.0]],
        })).unwrap()).unwrap();
    }}
