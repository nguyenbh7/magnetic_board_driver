//! Reviewed CT HDMM point import (not optical marker detection).
//! A rigidly transferred point is never represented as a direct CT detection.
use serde_json::Value;

const MAGNETS: [&str; 3] = ["magnet_left", "magnet_right", "magnet_superior"];
const MAX_MAGNET_DISAGREEMENT_MM: f64 = 0.75;

#[derive(Debug, Clone, PartialEq)]
pub struct ReviewedHdmmPoint {
    pub marker_lps_mm: [f64; 3],
    pub provenance: String,
}

fn point(value: &Value, name: &str) -> Result<[f64; 3], String> {
    let items = value.as_array().ok_or_else(|| format!("{name} must be an LPS XYZ array"))?;
    if items.len() != 3 { return Err(format!("{name}: expected three components")); }
    let mut xyz = [0.0; 3];
    for (i, item) in items.iter().enumerate() {
        xyz[i] = item.as_f64().ok_or_else(|| format!("{name}: nonnumeric component"))?;
        if !xyz[i].is_finite() {
            return Err(format!("{name}: nonfinite CT coordinate"));
        }
    }
    Ok(xyz)
}

fn get_string<'a>(record: &'a Value, key: &str) -> Result<&'a str, String> {
    record.get(key).and_then(Value::as_str).filter(|s| !s.trim().is_empty())
        .ok_or_else(|| format!("Missing required HDMM field: {key}"))
}

/// Require the reviewed record to agree with the *currently imported* CT
/// magnet constellation; no unrelated reference/session may be imported.
pub fn parse_reviewed_hdmm_point(
    raw: &str,
    current_magnets: &[(&str, [f64; 3])],
    current_frame_uid: &str,
) -> Result<ReviewedHdmmPoint, String> {
    if raw.len() > 2_000_000 { return Err("HDMM JSON larger than 2 MB".into()); }
    let doc: Value = serde_json::from_str(raw)
        .map_err(|err| format!("Invalid HDMM review JSON: {err}"))?;
    if get_string(&doc, "coordinate_frame")? != "DICOM_LPS_mm" {
        return Err("Reviewed HDMM point must use DICOM LPS millimetres".into());
    }
    let (pt, reference_magnets, uid, provenance) = match get_string(&doc, "format")? {
        "gk_hdmm_sticker_reference_v1" => {
            if get_string(&doc, "status")? !=
                "OPERATOR_REVIEWED_REFERENCE_NOT_ALGORITHM_VERIFIED" ||
                doc["immutable_reference"].as_bool() != Some(true)
            {
                return Err("Unreviewed HDMM reference candidate cannot be loaded".into());
            }
            let evidence = get_string(&doc, "evidence_type")?;
            if !matches!(evidence, "CT_VISIBLE_OPERATOR_REVIEWED" |
                                  "INDEPENDENT_PHYSICAL_RIGID_OFFSET")
            {
                return Err("Sticker reference requires physical/CT review evidence".into());
            }
            let note = get_string(&doc, "evidence_note")?;
            if note.len() < 12 {
                return Err("Sticker identification evidence note is too short".into());
            }
            (
                point(&doc["hdmm_marker_lps_mm"], "reference sticker position")?,
                &doc["reference_magnets_lps_mm"],
                doc["reference_frame_of_reference_uid"].as_str(),
                format!("OPERATOR-REVIEWED CT REFERENCE ({evidence}): {note}"),
            )
        }
        "gk_hdmm_sticker_transfer_v1" => {
            if get_string(&doc, "status")? !=
                "RIGID_TRANSFER_ESTIMATE_NOT_DIRECT_CT_DETECTION" ||
                doc["transfer_must_be_approved_by_operator"].as_bool() != Some(true) ||
                doc["marker_was_directly_identified_in_target"].as_bool() != Some(false)
            {
                return Err("Transferred point must be a clearly labeled reviewed estimate".into());
            }
            let fov = get_string(&doc["target_fov"], "status")?;
            if !matches!(fov, "OUTSIDE_FOV" | "CROPPED_OR_EDGE_LIMITED" |
                              "CENTER_AND_SUPPORT_INSIDE_FOV")
            {
                return Err("Target CT FOV must have been checked before importing".into());
            }
            for field in ["stationary_receiver_registration_rms_mm",
                          "head_magnet_rigid_fit_rms_mm"] {
                let rms = doc[field].as_f64()
                    .ok_or_else(|| format!("Missing rigid CT QC: {field}"))?;
                if !rms.is_finite() || !(0.0..=0.75).contains(&rms) {
                    return Err(format!("{field} violates 0.75 mm QC threshold"));
                }
            }
            let evidence = get_string(&doc, "reference_evidence_type")?;
            if !matches!(evidence, "CT_VISIBLE_OPERATOR_REVIEWED" |
                                  "INDEPENDENT_PHYSICAL_RIGID_OFFSET")
            {
                return Err("Transferred point has no reviewed reference evidence".into());
            }
            let note = get_string(&doc, "reference_evidence_note")?;
            if note.len() < 12 {
                return Err("Transferred point lacks reference identification note".into());
            }
            (
                point(&doc["predicted_target_hdmm_marker_lps_mm"],
                      "rigidly transferred target position")?,
                &doc["target_magnets_lps_mm"],
                doc["target_frame_of_reference_uid"].as_str(),
                format!("RIGIDLY TRANSFERRED — NOT CT-DETECTED IN TARGET; FOV {fov}; reference {evidence}: {note}"),
            )
        }
        _ => return Err("Only approved reference/transfer JSON is accepted; an automatic candidate is not a confirmed sticker".into()),
    };
    if let Some(uid) = uid {
        if !uid.is_empty() && !current_frame_uid.is_empty() && uid != current_frame_uid {
            return Err("Sticker record belongs to a different CT FrameOfReferenceUID".into());
        }
    }
    for name in MAGNETS {
        let seen = current_magnets.iter().filter(|(n,_)| *n == name)
            .map(|(_,p)| *p).collect::<Vec<_>>();
        if seen.len() != 1 {
            return Err(format!("Missing/duplicated magnet in imported CT: {name}"));
        }
        let expected = point(&reference_magnets[name], name)?;
        let error = expected.iter().zip(seen[0])
            .map(|(a,b)| (*a - b).powi(2)).sum::<f64>().sqrt();
        if !error.is_finite() || error > MAX_MAGNET_DISAGREEMENT_MM {
            return Err(format!(
                "HDMM sticker file belongs to another CT scan: {name} differs by {error:.3} mm"
            ));
        }
    }
    Ok(ReviewedHdmmPoint {marker_lps_mm: pt, provenance})
}

#[cfg(test)]
mod tests {
    use super::*;

    fn magnets<'a>() -> Vec<(&'a str, [f64; 3])> {
        vec![("magnet_left",[10.,20.,30.]),("magnet_right",[50.,20.,31.]),
             ("magnet_superior",[30.,25.,75.])]
    }
    fn landmark_map() -> &'static str {
        r#"{"magnet_left":[10,20,30],
            "magnet_right":[50,20,31],"magnet_superior":[30,25,75]}"#
    }
    fn reviewed() -> String {
        format!(r#"{{
            "format":"gk_hdmm_sticker_reference_v1",
            "coordinate_frame":"DICOM_LPS_mm",
            "status":"OPERATOR_REVIEWED_REFERENCE_NOT_ALGORITHM_VERIFIED",
            "immutable_reference":true,
            "evidence_type":"CT_VISIBLE_OPERATOR_REVIEWED",
            "evidence_note":"Optical sticker reviewed by operator in actual CBCT slices",
            "reference_frame_of_reference_uid":"frame-1",
            "hdmm_marker_lps_mm":[30,70,60],
            "reference_magnets_lps_mm":{}
        }}"#, landmark_map())
    }
    fn transfer() -> String {
        format!(r#"{{
            "format":"gk_hdmm_sticker_transfer_v1",
            "coordinate_frame":"DICOM_LPS_mm",
            "status":"RIGID_TRANSFER_ESTIMATE_NOT_DIRECT_CT_DETECTION",
            "transfer_must_be_approved_by_operator":true,
            "marker_was_directly_identified_in_target":false,
            "target_fov":{{"status":"OUTSIDE_FOV"}},
            "stationary_receiver_registration_rms_mm":0.186,
            "head_magnet_rigid_fit_rms_mm":0.053,
            "reference_evidence_type":"CT_VISIBLE_OPERATOR_REVIEWED",
            "reference_evidence_note":"Optical sticker reviewed by operator in actual CBCT slices",
            "target_frame_of_reference_uid":"frame-1",
            "predicted_target_hdmm_marker_lps_mm":[33,68,62],
            "target_magnets_lps_mm":{}
        }}"#, landmark_map())
    }
    #[test]
    fn reviewed_reference_requires_same_cbct_magnets_and_frame() {
        let p=parse_reviewed_hdmm_point(&reviewed(),&magnets(),"frame-1").unwrap();
        assert_eq!(p.marker_lps_mm,[30.,70.,60.]);
        assert!(p.provenance.contains("OPERATOR-REVIEWED"));
        assert!(parse_reviewed_hdmm_point(&reviewed(),&magnets(),"other").is_err());
        let mut wrong=magnets(); wrong[2].1[2] += 5.;
        assert!(parse_reviewed_hdmm_point(&reviewed(),&wrong,"frame-1").is_err());
    }
    #[test]
    fn transfer_must_be_labeled_predicted_and_field_of_view_checked() {
        let p=parse_reviewed_hdmm_point(&transfer(),&magnets(),"frame-1").unwrap();
        assert_eq!(p.marker_lps_mm,[33.,68.,62.]);
        assert!(p.provenance.contains("NOT CT-DETECTED"));
        assert!(parse_reviewed_hdmm_point(
            &transfer().replace("OUTSIDE_FOV","TARGET_FOV_UNCHECKED"),
            &magnets(),"frame-1").is_err());
        assert!(parse_reviewed_hdmm_point(
            &transfer().replace(
                r#""marker_was_directly_identified_in_target":false"#,
                r#""marker_was_directly_identified_in_target":true"#),
            &magnets(),"frame-1").is_err());
    }
    #[test]
    fn unreviewed_brightness_candidate_cannot_become_hdmm_marker() {
        let raw=r#"{"format":"gk_hdmm_sticker_roi_candidates_v1",
                      "coordinate_frame":"DICOM_LPS_mm"}"#;
        assert!(parse_reviewed_hdmm_point(raw,&magnets(),"frame-1").is_err());
        assert!(parse_reviewed_hdmm_point(
            &reviewed().replace("CT_VISIBLE_OPERATOR_REVIEWED","MODEL_GUESS"),
            &magnets(),"frame-1").is_err());
        assert!(parse_reviewed_hdmm_point("{",&magnets(),"frame-1").is_err());
    }
}
