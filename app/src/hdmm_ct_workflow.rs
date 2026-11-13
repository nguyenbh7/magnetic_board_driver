//! The CT sticker review workflow is separate from magnetic calibration.
//! Bright CT components are *candidates*, never automatically an optical
//! HDMM reflector. No Python script is run from the UI/event-loop thread.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use serde_json::Value;
use crate::cbct_geometry::{self, CbctGeometry};

#[derive(Debug, Clone)]
pub struct StickerCandidate {
    pub id: String,
    pub detail: String,
    pub edge_limited: bool,
}

#[derive(Debug, Clone)]
pub struct HdmmProposal {
    pub source_scan: PathBuf,
    pub frame_uid: String,
    pub detections_path: PathBuf,
    pub proposal_path: PathBuf,
    pub preview_path: Option<PathBuf>,
    pub status: String,
    pub candidates: Vec<StickerCandidate>,
    pub distal_review_focus: Option<String>,
}

#[derive(Debug, Clone)]
pub struct HdmmApprovedFile {
    pub path: PathBuf,
    pub source_scan: PathBuf,
    pub frame_uid: String,
    pub proposal_path: PathBuf,
}

fn fresh_path(directory: &Path, name: &str) -> Result<PathBuf, String> {
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH)
        .map_err(|e| e.to_string())?.as_nanos();
    let path = directory.join(format!(
        "{name}_{}_{}.json", std::process::id(), stamp
    ));
    if path.exists() {
        return Err(format!("Refusing to overwrite HDMM review data: {}", path.display()));
    }
    Ok(path)
}

fn ensure_original_scan(geometry: &CbctGeometry) -> Result<PathBuf, String> {
    let path = &geometry.scan_path;
    if !path.is_dir()
        && !(path.is_file() && path.extension().and_then(|x| x.to_str())
            .is_some_and(|x| x.eq_ignore_ascii_case("zip")))
    {
        return Err(
            "HDMM candidate detection needs the original CBCT ZIP or DICOM folder. \
             Load JSON imports CT positions only; re-import the original CBCT to search for the sticker."
                .into()
        );
    }
    let detections = geometry.qc_directory.join("detections.json");
    if !detections.is_file() {
        return Err(format!(
            "Original matching CT detections.json missing: {}. Import CBCT ZIP/folder first.",
            detections.display()
        ));
    }
    Ok(detections)
}

fn run_python(script_name: &str, arguments: &[String]) -> Result<(), String> {
    let (script, python, prefix) = cbct_geometry::hdmm_review_backend(script_name)?;
    let output = Command::new(&python)
        .args(&prefix)
        .arg("-u").arg(&script)
        .args(arguments)
        .output()
        .map_err(|e| format!("Cannot launch app-bundled {script_name}: {e}"))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let stdout = String::from_utf8_lossy(&output.stdout);
        return Err(format!(
            "{script_name} failed ({}). {} {}",
            output.status,
            stderr.chars().take(1600).collect::<String>(),
            stdout.chars().take(700).collect::<String>()
        ));
    }
    Ok(())
}

pub fn find_candidates(geometry: CbctGeometry) -> Result<HdmmProposal, String> {
    let detections = ensure_original_scan(&geometry)?;
    let report_path = fresh_path(&geometry.qc_directory, "hdmm_anatomy_candidates_v2")?;
    run_python("hdmm_anatomy_first.py", &[
        "propose".into(),
        "--scan".into(), geometry.scan_path.to_string_lossy().into_owned(),
        "--detections".into(), detections.to_string_lossy().into_owned(),
        "--landmark-method".into(), "slice-profile-v2".into(),
        "--radius-mm".into(), "25".into(),
        "--out".into(), report_path.to_string_lossy().into_owned(),
    ])?;

    let raw = std::fs::read_to_string(&report_path)
        .map_err(|e| format!("Cannot read sticker candidates: {e}"))?;
    let value: Value = serde_json::from_str(&raw)
        .map_err(|e| format!("Invalid sticker candidate JSON: {e}"))?;
    if value["format"].as_str() != Some("gk_hdmm_anatomy_first_candidates_v1")
        || value["automatic_sticker_identity"].as_bool() != Some(false)
        || value["no_prior_sticker_coordinates_used"].as_bool() != Some(true)
        || value["landmark_method"].as_str() != Some("slice-profile-v2")
        || value["source_frame_of_reference_uid"].as_str() != Some(&geometry.frame_uid)
    {
        return Err("HDMM detector returned untrusted/mismatched CT proposal provenance".into());
    }
    let status = value["status"].as_str().ok_or("HDMM proposal lacks status")?;
    if !matches!(status,
        "ANATOMY_GUIDED_CANDIDATES_REQUIRE_CT_REVIEW"
        | "ABSTAIN_ANATOMY_NOT_RELIABLY_IDENTIFIED"
        | "NO_PLAUSIBLE_HIGH_DENSITY_COMPONENT"
    ) {
        return Err(format!("Unknown HDMM proposal status: {status}"));
    }
    let rows = value["candidates"].as_array()
        .ok_or("HDMM proposal lacks candidate list")?;
    if rows.len() > 1000 {
        return Err("Unbounded CT candidate list refused".into());
    }
    let mut candidates = Vec::with_capacity(rows.len());
    for candidate in rows {
        let id = candidate["candidate_id"].as_str()
            .ok_or("HDMM candidate has no ID")?;
        let p = candidate["centroid_lps_mm"].as_array()
            .filter(|p| p.len() == 3)
            .ok_or("HDMM candidate does not contain 3 LPS coordinates")?;
        let xyz = p.iter().map(|v| v.as_f64()
            .filter(|x| x.is_finite()).ok_or("Non-finite HDMM candidate coordinate"))
            .collect::<Result<Vec<_>,_>>()?;
        let threshold = candidate["threshold_raw_ct"].as_f64()
            .ok_or("Missing HDMM candidate threshold")?;
        let n = candidate["voxel_count"].as_u64()
            .ok_or("Missing HDMM candidate voxel count")?;
        let edge = candidate["touches_scan_boundary"].as_bool() == Some(true)
            || candidate["touches_search_roi_boundary"].as_bool() == Some(true)
            || candidate["full_support_inside_reconstructed_box"].as_bool() != Some(true);
        let detail = format!(
            "{id}: CT LPS [{:.2}, {:.2}, {:.2}] mm · threshold {:.0} · {} voxels{}",
            xyz[0], xyz[1], xyz[2], threshold, n,
            if edge { " · EDGE/ROI LIMITED — cannot approve" } else { "" }
        );
        candidates.push(StickerCandidate {
            id: id.to_string(), detail, edge_limited: edge
        });
    }
    if value["candidate_count"].as_u64() != Some(candidates.len() as u64) {
        return Err("HDMM candidate count does not match original CT proposal".into());
    }
    let png = report_path.with_extension("png");
    Ok(HdmmProposal {
        source_scan: geometry.scan_path,
        frame_uid: geometry.frame_uid,
        detections_path: detections,
        proposal_path: report_path,
        preview_path: png.is_file().then_some(png),
        status: status.into(),
        candidates,
        distal_review_focus: value["distal_review_focus_candidate_id"]
            .as_str().map(str::to_owned),
    })
}

/// Explicit operator action only. The Python bridge rechecks the exact scan,
/// detector SHA, field of view, candidate coordinates, and review provenance.
pub fn approve_candidate(
    proposal: HdmmProposal,
    selected_id: String,
    note: String,
) -> Result<HdmmApprovedFile, String> {
    let candidate = proposal.candidates.iter()
        .find(|c| c.id == selected_id)
        .ok_or("Select a candidate ID from the current CBCT report")?;
    if candidate.edge_limited {
        return Err("Cannot approve an edge/ROI-limited candidate. Inspect a complete CT.".into());
    }
    if note.trim().chars().count() < 20 {
        return Err("Explain how the physical HDMM sticker was verified in CT (20+ characters)".into());
    }
    let output = fresh_path(
        proposal.proposal_path.parent().ok_or("CT proposal has no output directory")?,
        "hdmm_operator_reviewed_reference",
    )?;
    run_python("approve_anatomy_hdmm_sticker.py", &[
        "--proposal".into(), proposal.proposal_path.to_string_lossy().into_owned(),
        "--candidate-id".into(), selected_id,
        "--reference-detections".into(), proposal.detections_path.to_string_lossy().into_owned(),
        "--reference-scan".into(), proposal.source_scan.to_string_lossy().into_owned(),
        "--operator-confirmed".into(),
        "--note".into(), note.trim().to_string(),
        "--out".into(), output.to_string_lossy().into_owned(),
    ])?;
    if !output.is_file() {
        return Err("Sticker review completed without creating the expected JSON".into());
    }
    Ok(HdmmApprovedFile {
        path: output,
        source_scan: proposal.source_scan,
        frame_uid: proposal.frame_uid,
        proposal_path: proposal.proposal_path,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn proposal_and_review_never_come_from_detector_json_alone() {
        let geom = CbctGeometry {
            selected_detector_variant: "optimized".into(),
            hybrid_source_sha256: None,
            scan_path: PathBuf::from("detections.json"),
            qc_directory: PathBuf::from("."),
            frame_uid: "review_only".into(),
            rigid_origin_lps_mm: [0.;3],
            rigid_axes_to_lps: [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
            magnets: Vec::new(), arrays: Vec::new(), markers: Vec::new(),
        };
        assert!(ensure_original_scan(&geom).is_err());
    }

    #[test]
    fn a_candidate_without_review_note_or_incomplete_support_cannot_be_approved() {
        let proposal = HdmmProposal {
            source_scan: PathBuf::from("inaccessible.zip"),
            frame_uid: "frame".into(),
            detections_path: PathBuf::from("detections.json"),
            proposal_path: PathBuf::from("review.json"),
            preview_path: None, status: "UNREVIEWED".into(),
            distal_review_focus: None,
            candidates: vec![StickerCandidate {
                id: "N01".into(), detail: String::new(), edge_limited: true
            }],
        };
        assert!(approve_candidate(proposal.clone(), "N01".into(), "A complete 3D CT inspection confirms reflector".into()).is_err());
        let mut safe = proposal;
        safe.candidates[0].edge_limited = false;
        assert!(approve_candidate(safe, "N01".into(), "short".into()).is_err());
    }
}
