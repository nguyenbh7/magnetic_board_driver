//! DICOM/CBCT initial geometry import for the desktop live-fitting app.
//!
//! The Python detector runs off the UI thread. This module parses ONLY
//! gk_cbct_detections_v1; it does not construct or activate a magnetic
//! RigidProfile. The detected BB is NOT automatically an HDMM sticker.
use serde::Deserialize;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone)]
pub struct CbctGeometry {
    pub scan_path: PathBuf,
    pub qc_directory: PathBuf,
    pub frame_uid: String,
    pub rigid_origin_lps_mm: [f64; 3],
    pub magnets: Vec<CbctMagnet>,
    pub arrays: Vec<CbctArray>,
    pub markers: Vec<CbctMarker>,
}

#[derive(Debug, Clone)]
pub struct CbctMagnet {
    pub name: String,
    pub center_lps_mm: [f64; 3],
    pub offset_from_origin_mm: [f64; 3],
    pub unsigned_axis_lps: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct CbctArray {
    pub name: String,
    /// CT detector ordering: NOT raw Hall or firmware sensor index!
    pub detected_chip_positions_lps_mm: Vec<[f64; 3]>,
    pub center_lps_mm: [f64; 3],
    pub grid_rms_mm: f64,
}

#[derive(Debug, Clone)]
pub struct CbctMarker {
    pub name: String,
    pub center_lps_mm: [f64; 3],
    pub offset_from_origin_mm: [f64; 3],
    /// For this detector, 'bb' means candidate CT glass BB.
    /// It must not be labeled HDMM sticker without independent evidence.
    pub classification: String,
}

#[derive(Debug, Deserialize)]
struct DetectorReport {
    format: String,
    coordinate_frame: String,
    frame_of_reference_uid: Option<String>,
    status: String,
    #[serde(default)]
    requires_review: bool,
    objects: Vec<DetectedObject>,
}

#[derive(Debug, Deserialize)]
struct DetectedObject {
    name: String,
    kind: String,
    center_lps_mm: [f64; 3],
    chip_grid_lps_mm: Option<Vec<[f64; 3]>>,
    grid_rms_mm: Option<f64>,
    unsigned_axis_lps: Option<[f64; 3]>,
}

fn valid_point(point: &[f64; 3]) -> bool {
    point.iter().all(|value| value.is_finite())
}
fn offset(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn magnitude(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

impl CbctGeometry {
    /// Create an initial-geometry preview; no magnetic calibration fields.
    pub fn from_detector_json(source: &str, scan_path: PathBuf, qc_directory: PathBuf) -> Result<Self, String> {
        let report: DetectorReport = serde_json::from_str(source)
            .map_err(|error| format!("Invalid CBCT detection JSON: {error}"))?;
        if report.format != "gk_cbct_detections_v1" || report.coordinate_frame != "DICOM_LPS_mm" {
            return Err("Unsupported CT detector format or coordinate frame".into());
        }
        if report.status != "unreviewed" || !report.requires_review {
            return Err("The detector did not report complete, review-required geometry".into());
        }
        let uid = report.frame_of_reference_uid
            .filter(|text| !text.trim().is_empty())
            .ok_or("DICOM Frame of Reference UID missing")?;
        let mut magnets = Vec::new();
        let mut arrays = Vec::new();
        let mut markers = Vec::new();
        for object in report.objects {
            if !valid_point(&object.center_lps_mm) {
                return Err(format!("{} has nonfinite DICOM position", object.name));
            }
            match object.kind.as_str() {
                "magnet" => {
                    let axis = object.unsigned_axis_lps
                        .ok_or_else(|| format!("{} missing CT cylinder axis", object.name))?;
                    let len = magnitude(axis);
                    if !valid_point(&axis) || !(0.9..=1.1).contains(&len) {
                        return Err(format!("{} has invalid CT cylinder axis", object.name));
                    }
                    magnets.push(CbctMagnet {
                        name: object.name,
                        center_lps_mm: object.center_lps_mm,
                        offset_from_origin_mm: [0.0; 3],
                        unsigned_axis_lps: [axis[0] / len, axis[1] / len, axis[2] / len],
                    });
                }
                "array" => {
                    let pts = object.chip_grid_lps_mm
                        .ok_or_else(|| format!("{} lacks 16 CT chip detections", object.name))?;
                    if pts.len() != 16 || !pts.iter().all(valid_point) {
                        return Err(format!("{} must contain 16 finite chip positions", object.name));
                    }
                    let rms = object.grid_rms_mm
                        .filter(|value| value.is_finite() && *value >= 0.0 && *value <= 1.0)
                        .ok_or_else(|| format!("{} has invalid/large chip grid RMS", object.name))?;
                    arrays.push(CbctArray {
                        name: object.name,
                        detected_chip_positions_lps_mm: pts,
                        center_lps_mm: object.center_lps_mm,
                        grid_rms_mm: rms,
                    });
                }
                "bb" => markers.push(CbctMarker {
                    name: object.name,
                    center_lps_mm: object.center_lps_mm,
                    offset_from_origin_mm: [0.0; 3],
                    classification: "CT glass BB (HDMM sticker identity UNVERIFIED)".into(),
                }),
                other => return Err(format!("Unrecognized CT detected object kind: {other}")),
            }
        }
        if magnets.len() != 3 || arrays.len() != 3 || markers.len() != 1 {
            return Err(format!(
                "Expected exactly 3 magnets, 3 arrays (48 chips), 1 glass BB; got {}/{}/{}",
                magnets.len(), arrays.len(), markers.len()
            ));
        }
        let unique = |names: Vec<&str>| {
            let count = names.len();
            let set = names.into_iter().collect::<std::collections::HashSet<_>>();
            set.len() == count
        };
        if !unique(magnets.iter().map(|x| x.name.as_str()).collect())
            || !unique(arrays.iter().map(|x| x.name.as_str()).collect()) {
            return Err("Duplicate CT magnet or array names".into());
        }
        let mut origin = [0.0; 3];
        for magnet in &magnets {
            for (o, v) in origin.iter_mut().zip(magnet.center_lps_mm) {
                *o += v / 3.0;
            }
        }
        for m in &mut magnets {
            m.offset_from_origin_mm = offset(m.center_lps_mm, origin);
        }
        for m in &mut markers {
            m.offset_from_origin_mm = offset(m.center_lps_mm, origin);
        }
        Ok(Self {
            scan_path, qc_directory,
            frame_uid: uid,
            rigid_origin_lps_mm: origin,
            magnets, arrays, markers,
        })
    }

    pub fn summary(&self) -> String {
        let mut lines = vec![
            "CBCT INITIAL GEOMETRY — UNREVIEWED (not a magnetic fit profile)".to_string(),
            format!("Source: {}", self.scan_path.display()),
            format!("DICOM LPS frame: {}", self.frame_uid),
            format!("Three-magnet rigid origin L/P/S: {:.3}, {:.3}, {:.3} mm",
                self.rigid_origin_lps_mm[0], self.rigid_origin_lps_mm[1], self.rigid_origin_lps_mm[2]),
        ];
        for magnet in &self.magnets {
            let p = magnet.center_lps_mm;
            let d = magnet.offset_from_origin_mm;
            lines.push(format!("{}: magnet center L/P/S [{:.2}, {:.2}, {:.2}], rigid offset [{:+.2}, {:+.2}, {:+.2}] mm (axis unsigned)",
                magnet.name,p[0],p[1],p[2],d[0],d[1],d[2]));
        }
        for array in &self.arrays {
            let p = array.center_lps_mm;
            lines.push(format!("{}: {} CT-detected chips, center [{:.2}, {:.2}, {:.2}] LPS mm; grid RMS {:.3} mm (NOT firmware index order)",
                array.name,array.detected_chip_positions_lps_mm.len(),p[0],p[1],p[2],array.grid_rms_mm));
        }
        for marker in &self.markers {
            let p=marker.center_lps_mm;
            let d=marker.offset_from_origin_mm;
            lines.push(format!("{}: {} at [{:.2}, {:.2}, {:.2}] LPS mm; relative to magnet origin [{:+.2}, {:+.2}, {:+.2}] mm",
                marker.name,marker.classification,p[0],p[1],p[2],d[0],d[1],d[2]));
        }
        lines.push("HDMM reflective sticker: NOT independently detected/identified; BB-to-HDMM correspondence requires manual confirmation.".into());
        lines.push("Geometry import DOES NOT load or enable a RigidProfile. The magnetic field calibration is a separate step.".into());
        lines.push(format!("Detector review images/JSON: {}", self.qc_directory.display()));
        lines.join("\n")
    }
}

/// Runs the maintained Python detector in another process, never in the live fit.
/// Output goes to local application data (never inside a source checkout).
pub fn import_dicom(scan: &Path) -> Result<CbctGeometry, String> {
    if !scan.exists() {
        return Err(format!("DICOM series or ZIP not found: {}", scan.display()));
    }
    if scan.is_file() && scan.extension().and_then(|x| x.to_str()).is_none_or(|ext| !ext.eq_ignore_ascii_case("zip")) {
        return Err("Select a DICOM series FOLDER or a ZIP containing the complete series, not a single .dcm image".into());
    }
    let script = std::env::var_os("CBCT_DETECT_SCRIPT")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("..").join("..").join("hall-effect-motion-tracking")
                .join("tools").join("cbct_bridge").join("cbct_detect.py")
        });
    if !script.is_file() {
        return Err(format!(
            "CBCT detector not found: {}. Clone hall-effect-motion-tracking beside magnetic_board_driver or set CBCT_DETECT_SCRIPT.",
            script.display()
        ));
    }
    let root = std::env::var_os("LOCALAPPDATA").map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir)
        .join("magnetic_board_driver").join("cbct_imports");
    std::fs::create_dir_all(&root).map_err(|e|format!("Cannot create CBCT results folder: {e}"))?;
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH)
        .map_err(|e| e.to_string())?.as_nanos();
    let result_dir = root.join(format!("import_{}_{}",std::process::id(),stamp));
    std::fs::create_dir(&result_dir).map_err(|e|format!("Cannot create unique CBCT output folder: {e}"))?;
    let python = std::env::var_os("CBCT_PYTHON")
        .unwrap_or_else(|| std::ffi::OsString::from("python"));
    let result=Command::new(python)
        .arg(&script).arg(scan).arg("--output").arg(&result_dir)
        .output()
        .map_err(|e|format!("Unable to launch Python CBCT detector: {e}. Configure CBCT_PYTHON to an environment with SimpleITK, scipy, numpy and matplotlib."))?;
    if !result.status.success() {
        let stderr=String::from_utf8_lossy(&result.stderr);
        let stdout=String::from_utf8_lossy(&result.stdout);
        return Err(format!("CBCT detector failed (exit {}): {} {}. Outputs (if any): {}",
            result.status,stderr.chars().take(800).collect::<String>(),
            stdout.chars().take(600).collect::<String>(),result_dir.display()));
    }
    let raw=std::fs::read_to_string(result_dir.join("detections.json"))
        .map_err(|e|format!("CBCT detector did not write detections.json: {e}"))?;
    CbctGeometry::from_detector_json(&raw,scan.to_path_buf(),result_dir)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> String {
        let mut objects=Vec::new();
        for i in 0..3 {
            objects.push(serde_json::json!({
                "kind":"magnet","name":format!("magnet_{i}"),
                "center_lps_mm":[i as f64*10.0,0.0,20.0],
                "unsigned_axis_lps":[0.,0.,1.]
            }));
            objects.push(serde_json::json!({
                "kind":"array","name":format!("array_{i}"),
                "center_lps_mm":[i as f64*15.0,0.0,0.0],
                "grid_rms_mm":0.14,"chip_grid_lps_mm":vec![[0.,0.,0.];16]
            }));
        }
        objects.push(serde_json::json!({
            "kind":"bb","name":"glass_bb","center_lps_mm":[15.,0.,38.]
        }));
        serde_json::json!({
            "format":"gk_cbct_detections_v1",
            "coordinate_frame":"DICOM_LPS_mm",
            "frame_of_reference_uid":"1.2.3.test",
            "status":"unreviewed",
            "requires_review":true,
            "objects":objects
        }).to_string()
    }

    #[test]
    fn parse_three_magnets_arrays_and_bb_with_rigid_origin() {
        let c=CbctGeometry::from_detector_json(&sample(),"scan.zip".into(),"qc".into()).unwrap();
        assert_eq!(c.magnets.len(),3);
        assert_eq!(c.arrays.len(),3);
        assert_eq!(c.arrays.iter().map(|b|b.detected_chip_positions_lps_mm.len()).sum::<usize>(),48);
        assert_eq!(c.rigid_origin_lps_mm,[10.,0.,20.]);
        assert_eq!(c.markers[0].offset_from_origin_mm,[5.,0.,18.]);
        assert!(c.summary().contains("HDMM reflective sticker: NOT independently"));
    }
    #[test]
    fn refuse_incomplete_scan_without_mutating_fit() {
        let broken=sample().replace("\"status\":\"unreviewed\"","\"status\":\"incomplete_detection\"");
        assert!(CbctGeometry::from_detector_json(&broken,"x".into(),"y".into()).is_err());
    }
    #[test]
    fn fail_closed_when_frame_uid_missing() {
        let broken=sample().replace("1.2.3.test","");
        assert!(CbctGeometry::from_detector_json(&broken,"x".into(),"y".into()).is_err());
    }
    #[test]
    fn refuse_wrong_number_of_chip_detections() {
        let mut parsed:serde_json::Value=serde_json::from_str(&sample()).unwrap();
        parsed["objects"][1]["chip_grid_lps_mm"]=serde_json::json!([[1.,2.,3.]]);
        assert!(CbctGeometry::from_detector_json(&parsed.to_string(),"x".into(),"y".into()).is_err());
    }
    #[test]
    fn refuse_unknown_object_as_hdmm_sticker() {
        let mut parsed:serde_json::Value=serde_json::from_str(&sample()).unwrap();
        parsed["objects"].as_array_mut().unwrap().push(serde_json::json!({
            "kind":"hdmm","name":"sticker","center_lps_mm":[0.,0.,0.]
        }));
        assert!(CbctGeometry::from_detector_json(&parsed.to_string(),"x".into(),"y".into()).is_err());
    }
}
