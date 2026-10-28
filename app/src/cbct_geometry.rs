//! DICOM/CBCT initial geometry import for the desktop live-fitting app.
//!
//! The Python detector runs off the UI thread. This module parses ONLY
//! gk_cbct_detections_v1; it does not construct or activate a magnetic
//! RigidProfile. The detected BB is NOT automatically an HDMM sticker.
use serde::Deserialize;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::io::{BufRead, BufReader, Read};
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::time::Duration;

#[derive(Default, Debug)]
pub struct ImportProgress {
    pub latest: String,
    pub log: Vec<String>,
    pub cancel: AtomicBool,
}

fn record(progress: &Arc<Mutex<ImportProgress>>, line: String) {
    if let Ok(mut state) = progress.lock() {
        state.latest = line.clone();
        state.log.push(line);
        if state.log.len() > 300 { state.log.remove(0); }
    }
}

use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone)]
pub struct CbctGeometry {
    pub scan_path: PathBuf,
    pub qc_directory: PathBuf,
    pub frame_uid: String,
    pub rigid_origin_lps_mm: [f64; 3],
    /// Columns are body +X (Left->Right), +Y (toward Superior magnet),
    /// +Z = cross(+X,+Y). Proper right-handed basis in DICOM LPS.
    pub rigid_axes_to_lps: [[f64; 3]; 3],
    pub magnets: Vec<CbctMagnet>,
    pub arrays: Vec<CbctArray>,
    pub markers: Vec<CbctMarker>,
}

#[derive(Debug, Clone)]
pub struct CbctMagnet {
    pub name: String,
    pub center_lps_mm: [f64; 3],
    pub offset_from_origin_mm: [f64; 3],
    pub offset_in_rigid_frame_mm: [f64; 3],
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
    pub offset_in_rigid_frame_mm: [f64; 3],
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
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
}
fn unit(a: [f64; 3]) -> Result<[f64; 3], String> {
    let length=magnitude(a);
    if !length.is_finite() || length < 1e-3 {
        return Err("Three-magnet constellation does not define distinct, non-collinear rigid axes".into());
    }
    Ok([a[0]/length,a[1]/length,a[2]/length])
}
fn in_basis(vector_lps:[f64;3], e1:[f64;3],e2:[f64;3],e3:[f64;3]) -> [f64;3] {
    [dot(vector_lps,e1),dot(vector_lps,e2),dot(vector_lps,e3)]
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
                        offset_in_rigid_frame_mm: [0.0; 3],
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
                    offset_in_rigid_frame_mm: [0.0; 3],
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
        // Rigid coordinate convention: +X from left to right magnet;
        // +Y toward the superior magnet after orthogonalization;
        // +Z right-handed. This is a geometric INITIAL frame only.
        let named_center = |name: &str| magnets.iter()
            .find(|m|m.name==name).map(|m|m.center_lps_mm);
        let left=named_center("magnet_left")
            .ok_or("Missing named left magnet for geometric rigid-body frame")?;
        let right=named_center("magnet_right")
            .ok_or("Missing named right magnet for geometric rigid-body frame")?;
        let top=named_center("magnet_superior")
            .ok_or("Missing named superior magnet for geometric rigid-body frame")?;
        let e1=unit(offset(right,left))?;
        let mid=[(left[0]+right[0])*0.5,(left[1]+right[1])*0.5,(left[2]+right[2])*0.5];
        let superior_axis=offset(top,mid);
        let component=dot(superior_axis,e1);
        let orthogonal=[
            superior_axis[0]-component*e1[0],
            superior_axis[1]-component*e1[1],
            superior_axis[2]-component*e1[2],
        ];
        let e2=unit(orthogonal)?;
        let e3=unit(cross(e1,e2))?;
        let axes=[
            [e1[0],e2[0],e3[0]],
            [e1[1],e2[1],e3[1]],
            [e1[2],e2[2],e3[2]],
        ];
        for m in &mut magnets {
            m.offset_from_origin_mm = offset(m.center_lps_mm, origin);
            m.offset_in_rigid_frame_mm = in_basis(m.offset_from_origin_mm,e1,e2,e3);
        }
        for m in &mut markers {
            m.offset_from_origin_mm = offset(m.center_lps_mm, origin);
            m.offset_in_rigid_frame_mm = in_basis(m.offset_from_origin_mm,e1,e2,e3);
        }
        Ok(Self {
            scan_path, qc_directory,
            frame_uid: uid,
            rigid_origin_lps_mm: origin,
            rigid_axes_to_lps: axes,
            magnets, arrays, markers,
        })
    }

    /// Full positions for inspection/export; the 0..15 chip indices here are
    /// detector order, NOT the hardware firmware order. No signed Hall fields.
    pub fn coordinates_csv(&self) -> String {
        let mut lines = vec![
            "kind,name,detector_index,L_mm,P_mm,S_mm,offset_L_mm,offset_P_mm,offset_S_mm,verified".to_string(),
        ];
        let format_row = |kind: &str, name: &str, index: String, point: [f64; 3],
                          origin: [f64; 3]| {
            let offset = offset(point, origin);
            format!("{kind},{name},{index},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},false",
                point[0],point[1],point[2],offset[0],offset[1],offset[2])
        };
        for magnet in &self.magnets {
            lines.push(format_row("magnet",&magnet.name,String::new(),
                magnet.center_lps_mm,self.rigid_origin_lps_mm));
        }
        for array in &self.arrays {
            for (index,chip) in array.detected_chip_positions_lps_mm.iter().enumerate() {
                lines.push(format_row("sensor_ct_detector_index",&array.name,index.to_string(),
                    *chip,self.rigid_origin_lps_mm));
            }
        }
        for marker in &self.markers {
            lines.push(format_row("glass_BB_not_verified_HDMM",&marker.name,String::new(),
                marker.center_lps_mm,self.rigid_origin_lps_mm));
        }
        lines.join("\n")
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
            lines.push(format!(
                "    Candidate BB lever arm in initial rigid frame: [{:+.2}, {:+.2}, {:+.2}] mm (identity/attachment UNVERIFIED)",
                marker.offset_in_rigid_frame_mm[0],marker.offset_in_rigid_frame_mm[1],marker.offset_in_rigid_frame_mm[2]));
        }
        lines.push(format!(
            "Rigid initial +X (left->right), +Y (toward superior), +Z (right-hand) in LPS: X=[{:.3},{:.3},{:.3}], Y=[{:.3},{:.3},{:.3}], Z=[{:.3},{:.3},{:.3}]",
            self.rigid_axes_to_lps[0][0],self.rigid_axes_to_lps[1][0],self.rigid_axes_to_lps[2][0],
            self.rigid_axes_to_lps[0][1],self.rigid_axes_to_lps[1][1],self.rigid_axes_to_lps[2][1],
            self.rigid_axes_to_lps[0][2],self.rigid_axes_to_lps[1][2],self.rigid_axes_to_lps[2][2]));
        lines.push("HDMM reflective sticker: NOT independently detected/identified; BB-to-HDMM correspondence requires manual confirmation.".into());
        lines.push("Geometry import DOES NOT load or enable a RigidProfile. The magnetic field calibration is a separate step.".into());
        lines.push(format!("Detector review images/JSON: {}", self.qc_directory.display()));
        lines.join("\n")
    }
}

/// Runs the maintained Python detector in another process, never in the live fit.
/// Output goes to local application data (never inside a source checkout).
pub fn import_dicom(scan: &Path, progress: Arc<Mutex<ImportProgress>>) -> Result<CbctGeometry, String> {
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
    record(&progress, "Launching Python CBCT detector...".into());
    let mut child = Command::new(python)
        .arg("-u").arg(&script).arg(scan).arg("--output").arg(&result_dir)
        .stdout(Stdio::piped()).stderr(Stdio::piped())
        .spawn()
        .map_err(|e|format!("Unable to launch Python CBCT detector: {e}. Configure CBCT_PYTHON to an environment with SimpleITK, scipy, numpy and matplotlib."))?;
    let stdout = child.stdout.take().ok_or("Missing Python stdout pipe")?;
    let stderr = child.stderr.take().ok_or("Missing Python stderr pipe")?;
    let updates = Arc::clone(&progress);
    let output_reader = std::thread::spawn(move || {
        let mut collected = String::new();
        for item in BufReader::new(stdout).lines() {
            let Ok(line) = item else { break };
            if let Some(stage) = line.strip_prefix("CBCT_PROGRESS:") {
                record(&updates, stage.to_owned());
            } else if !line.trim().is_empty() {
                collected.push_str(&line);
                collected.push('\n');
                // Full terminal detail is retained in the optional UI log.
                if let Ok(mut state) = updates.lock() {
                    state.log.push(line);
                    if state.log.len() > 300 { state.log.remove(0); }
                }
            }
        }
        collected
    });
    let error_reader = std::thread::spawn(move || {
        let mut output = String::new();
        let _ = BufReader::new(stderr).read_to_string(&mut output);
        output
    });
    let (status, cancelled) = loop {
        if progress.lock().map(|p|p.cancel.load(Ordering::Relaxed)).unwrap_or(false) {
            let _ = child.kill();
            break (child.wait().map_err(|e|e.to_string())?, true);
        }
        if let Some(status) = child.try_wait().map_err(|e|e.to_string())? {
            break (status, false);
        }
        std::thread::sleep(Duration::from_millis(100));
    };
    let stdout_text = output_reader.join().unwrap_or_default();
    let stderr_text = error_reader.join().unwrap_or_default();
    if cancelled { return Err("Detection cancelled by user. No geometry was imported.".into()); }
    if !status.success() {
        return Err(format!("CBCT detector failed (exit {}): {} {}. Outputs (if any): {}",
            status,stderr_text.chars().take(800).collect::<String>(),
            stdout_text.chars().take(600).collect::<String>(),result_dir.display()));
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
            let (magnet_name, magnet_center) = match i {
                0 => ("magnet_left", [0.,-10.,20.]),
                1 => ("magnet_right", [20.,-10.,20.]),
                _ => ("magnet_superior", [10.,20.,20.]),
            };
            objects.push(serde_json::json!({
                "kind":"magnet","name":magnet_name,
                "center_lps_mm":magnet_center,
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
        assert_eq!(c.markers[0].offset_in_rigid_frame_mm,[5.,0.,18.]);
        assert_eq!(c.rigid_axes_to_lps,[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]);
        assert!(c.summary().contains("HDMM reflective sticker: NOT independently"));
    }
    #[test]
    fn exported_coordinates_contain_all_48_ct_detector_indices() {
        let c=CbctGeometry::from_detector_json(&sample(),"scan.zip".into(),"qc".into()).unwrap();
        let csv=c.coordinates_csv();
        assert_eq!(csv.lines().count(),1+3+48+1);
        assert!(csv.contains("sensor_ct_detector_index,array_0,0,"));
        assert!(csv.contains("glass_BB_not_verified_HDMM"));
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
    fn reject_collinear_magnets_without_initial_rotation() {
        let mut val: serde_json::Value=serde_json::from_str(&sample()).unwrap();
        // Place superior on the left-right axis.
        val["objects"][4]["center_lps_mm"]=serde_json::json!([10.,-10.,20.]);
        assert!(CbctGeometry::from_detector_json(&val.to_string(),"x".into(),"y".into()).is_err());
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
