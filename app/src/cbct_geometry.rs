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
    pub pcb_center_lps_mm: Option<[f64; 3]>,
    pub chip_face_normal_lps: Option<[f64; 3]>,
}
/// Provisional v2 CT/JST chip correspondence, in firmware sensor order.
/// The coordinate transformation uses the KiCad/A1 chip-axis hypothesis.
#[derive(Debug, Clone)]
pub struct CbctRegisteredBoard {
    pub detection_name: String,
    pub raw_board_id: u16,
    pub firmware_to_detector: [usize; 16],
    pub sensors_lps_mm: Vec<[f64; 3]>,
    pub raw_to_lps: [[f64; 3]; 3],
    pub geometric_rms_mm: f64,
    pub jst_proxy_offset_mm: f64,
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
    board_center_lps_mm: Option<[f64; 3]>,
    board_normal_lps: Option<[f64; 3]>,
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
                        pcb_center_lps_mm: object.board_center_lps_mm,
                        chip_face_normal_lps: object.board_normal_lps,
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


    /// Propose the *current* v2 firmware-index correspondence. Uses the
    /// PCB-center-to-chip-center displacement as a JST-side proxy; never
    /// silently falls back to a symmetric grid-only orientation.
    pub fn register_ct_boards_v2(&self) -> Result<Vec<CbctRegisteredBoard>, String> {
        let mut registered = Vec::with_capacity(3);
        for (name, raw_id) in [
            ("array_superior", 0_u16), ("array_left", 1_u16), ("array_right", 2_u16)
        ] {
            let array = self.arrays.iter().find(|b| b.name == name)
                .ok_or_else(|| format!("Missing {name} for v2 registration"))?;
            let pcb = array.pcb_center_lps_mm
                .ok_or_else(|| format!("{name}: missing CT PCB centroid/JST proxy"))?;
            let normal = array.chip_face_normal_lps
                .ok_or_else(|| format!("{name}: missing CT chip-face normal"))?;
            if !valid_point(&pcb) || !valid_point(&normal) {
                return Err(format!("{name}: nonfinite CT board geometry"));
            }
            let normal = unit(normal)?;
            let points = &array.detected_chip_positions_lps_mm;
            if points.len() != 16 {return Err(format!("{name}: expected 16 CT chip positions"));}
            let mut centroid=[0.0;3];
            for p in points {
                for k in 0..3 {centroid[k]+=p[k]/16.0;}
            }
            let to_connector = offset(pcb,centroid);
            let along_normal = dot(to_connector,normal);
            let projected=[
                to_connector[0]-along_normal*normal[0],
                to_connector[1]-along_normal*normal[1],
                to_connector[2]-along_normal*normal[2],
            ];
            let proxy_offset=magnitude(projected);
            if proxy_offset < 1.0 || !proxy_offset.is_finite() {
                return Err(format!("{name}: cannot determine JST end from PCB/chip centroids"));
            }
            // Local +Y is away from JST, +X is right as seen from chip face.
            let up=unit([-projected[0],-projected[1],-projected[2]])?;
            let right=unit(cross(up,normal))?;
            let mut projected_pts=(0..16).map(|i|{
                let d=offset(points[i],centroid);
                (i,dot(d,right),dot(d,up),dot(d,normal))
            }).collect::<Vec<_>>();
            projected_pts.sort_by(|a,b|a.1.total_cmp(&b.1));
            let mut permutation=[0_usize;16];
            let mut squared=0.0;
            let mut maxerr=0.0_f64;
            for col in 0..4 {
                let group=&mut projected_pts[col*4..(col+1)*4];
                group.sort_by(|a,b|a.2.total_cmp(&b.2));
                for row in 0..4 {
                    let (detector,x,y,z)=group[row];
                    permutation[col*4+row]=detector;
                    let expected_x=-6.75+4.5*col as f64;
                    let expected_y=-6.75+4.5*row as f64;
                    let e=((x-expected_x).powi(2)+(y-expected_y).powi(2)+z*z).sqrt();
                    squared+=e*e;
                    maxerr=maxerr.max(e);
                }
            }
            let rms=(squared/16.0).sqrt();
            if rms>0.70 || maxerr>1.50 {
                return Err(format!("{name}: CT-to-v2 firmware grid is inconsistent (RMS {rms:.3} mm, max {maxerr:.3} mm)"));
            }
            let mut sensors=Vec::with_capacity(16);
            for index in permutation {sensors.push(points[index]);}
            // A1/KiCad convention: [B_grid_x,B_grid_y,B_grid_z] =
            // [B_raw_y,-B_raw_x,B_raw_z]. Thus the raw-to-LPS columns
            // are [-up,+right,+normal] for the CT-oriented board frame.
            let raw_to_lps=[
                [-up[0],right[0],normal[0]],
                [-up[1],right[1],normal[1]],
                [-up[2],right[2],normal[2]],
            ];
            registered.push(CbctRegisteredBoard {
                detection_name:name.into(),raw_board_id:raw_id,
                firmware_to_detector:permutation,
                sensors_lps_mm:sensors,
                raw_to_lps, geometric_rms_mm:rms,
                jst_proxy_offset_mm:proxy_offset,
            });
        }
        Ok(registered)
    }

    /// Human-readable offline verification before Hall hardware is connected.
    pub fn registration_preview_v2(&self) -> String {
        match self.register_ct_boards_v2() {
            Ok(boards) => {
                let mut lines=vec!["CT->firmware v2 indexing and Hall-axis candidate: GEOMETRY PASS (physical JST/normal review still required)".to_string()];
                for b in boards {
                    lines.push(format!(
                        "  {} = raw board {}: chip-grid RMS {:.3} mm, JST proxy offset {:.2} mm",
                        b.detection_name,b.raw_board_id,b.geometric_rms_mm,b.jst_proxy_offset_mm));
                    lines.push(format!("    firmware sensor 0 -> CT detector chip {}; sensor 15 -> chip {}",
                        b.firmware_to_detector[0],b.firmware_to_detector[15]));
                    lines.push(format!("    firmware-index -> CT detector-index: {:?}",b.firmware_to_detector));
                    lines.push(format!("    candidate raw Bx/By/Bz -> DICOM LPS matrix: {:?}",b.raw_to_lps));
                }
                lines.push("Board ID assignment assumes raw0=Superior, raw1=Left, raw2=Right. Match actual wiring and inspect JST/chip-face orientation.".into());
                lines.join("\n")
            }
            Err(e) => format!("CT->firmware v2 registration NOT READY: {e}")
        }
    }

    /// Construct an experiment-specific *candidate* from the CT registration.
    /// Magnetic remanences here are placeholders; callers MUST fit them from
    /// current background-subtracted stationary measurements before activation.
    pub fn experimental_candidate_profile(&self) -> Result<crate::rigid_fit::RigidProfile, String> {
        use crate::rigid_fit::{RigidBoard, RigidMagnet, RigidProfile};
        let reg=self.register_ct_boards_v2()?;
        let boards=reg.into_iter().map(|b|RigidBoard {
            raw_board_id:b.raw_board_id,
            sensors_mm:b.sensors_lps_mm,
            raw_to_world:b.raw_to_lps,
            background_mt:[0.0;3],
            sensor_gains:None,
        }).collect::<Vec<_>>();
        // Current physical GK magnets: 5/16-inch diameter. The superior
        // magnet is 1/4-inch thick; side magnets are 1/8-inch. These are
        // hardware dimensions, not CT bounding-box guesses. Verify if the
        // magnets are changed before the next experiment.
        let mut magnets=Vec::with_capacity(3);
        for (name, thickness) in [
            ("magnet_left",3.175_f64),
            ("magnet_right",3.175_f64),
            ("magnet_superior",6.35_f64)
        ] {
            let m=self.magnets.iter().find(|m|m.name==name)
                .ok_or_else(||format!("Missing magnet {name}"))?;
            magnets.push(RigidMagnet {
                center_mm:m.center_lps_mm, axis:m.unsigned_axis_lps,
                diameter_mm:7.9375, thickness_mm:thickness,
                remanence_mt:1.0,
            });
        }
        Ok(RigidProfile {
            name:"GK CT experimental signed-strength fit (review required)".into(),
            coordinate_frame:Some("DICOM_LPS_mm".into()),
            origin_mm:self.rigid_origin_lps_mm,
            boards,magnets,
            initial_pose:[0.0;6],
            lower_bounds:[-20.0,-20.0,-20.0,-0.5235987756,-0.5235987756,-0.5235987756],
            upper_bounds:[20.0,20.0,20.0,0.5235987756,0.5235987756,0.5235987756],
            max_rmse_mt:0.50,
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

/// Resolve an interpreter that can run the CT detector on Windows and Unix.
/// Prefer the detector's venv or an active venv, then Windows py launchers.
/// A successful Python launch alone is insufficient: probe the image libraries
/// before selecting a candidate so the Windows Store python alias is rejected.
fn discover_cbct_python(script: &Path) -> Result<(std::ffi::OsString, Vec<std::ffi::OsString>, String), String> {
    use std::ffi::OsString;
    let mut candidates: Vec<(OsString, Vec<OsString>)> = Vec::new();
    let explicit = std::env::var_os("CBCT_PYTHON");
    if let Some(path) = explicit.as_ref() {
        candidates.push((path.clone(), vec![]));
    } else {
        let exe = if cfg!(windows) { "Scripts/python.exe" } else { "bin/python" };
        if let Some(dir) = script.parent() {
            for base in [Some(dir), dir.parent().and_then(Path::parent)] {
                if let Some(base) = base {
                    let venv = base.join(".venv").join(exe);
                    if venv.is_file() { candidates.push((venv.into_os_string(),vec![])); }
                }
            }
        }
        for key in ["VIRTUAL_ENV", "CONDA_PREFIX"] {
            if let Some(folder) = std::env::var_os(key) {
                let path = if cfg!(windows) {
                    PathBuf::from(folder).join("Scripts").join("python.exe")
                } else { PathBuf::from(folder).join("bin").join("python") };
                if path.is_file() { candidates.push((path.into_os_string(),vec![])); }
            }
        }
        if cfg!(windows) {
            for version in ["-3.11","-3.12","-3.13","-3"] {
                candidates.push((OsString::from("py"),vec![OsString::from(version)]));
            }
            candidates.push((OsString::from("py"),vec![]));
        }
        candidates.push((OsString::from("python"),vec![]));
        candidates.push((OsString::from("python3"),vec![]));
    }
    let mut diagnostics=Vec::new();
    for (program, prefix) in candidates {
        let label = format!("{} {}",program.to_string_lossy(),
            prefix.iter().map(|v|v.to_string_lossy().into_owned()).collect::<Vec<_>>().join(" "));
        let result=Command::new(&program).args(&prefix).arg("-c")
            .arg("import numpy, scipy, SimpleITK, matplotlib, pandas, torchio; import sys; print(sys.executable)")
            .output();
        match result {
            Ok(out) if out.status.success() => {
                let interpreter = String::from_utf8_lossy(&out.stdout).trim().to_string();
                return Ok((program,prefix,interpreter));
            }
            Ok(out) => {
                let error = String::from_utf8_lossy(&out.stderr);
                let reason = error.lines().rev().find(|line|!line.trim().is_empty())
                    .unwrap_or("Python interpreter failed");
                diagnostics.push(format!("{label}: {reason}"));
            }
            Err(err) => diagnostics.push(format!("{label}: {err}")),
        }
    }
    Err(format!(
        "No Python interpreter with the DICOM detector dependencies was found. \
         Install requirements using: py -3.11 -m pip install -r \"<hall-effect-motion-tracking>\\tools\\cbct_bridge\\requirements.txt\". \
         Or set CBCT_PYTHON to the complete path of that environment's python.exe. \
         Checked: {}",
        diagnostics.join(" | ")
    ))
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
    record(&progress, "Finding Python environment with DICOM dependencies...".into());
    let (python,prefix,python_location) = discover_cbct_python(&script)?;
    record(&progress, format!("DICOM Python: {python_location}"));
    record(&progress, "Launching Python CBCT detector...".into());
    let mut child = Command::new(&python)
        .args(&prefix).arg("-u").arg(&script).arg(scan).arg("--output").arg(&result_dir)
        .stdout(Stdio::piped()).stderr(Stdio::piped())
        .spawn()
        .map_err(|e|format!("Unable to launch selected Python DICOM environment {python_location}: {e}"))?;
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
    fn v2_ct_registration_recovers_firmware_grid_for_rotated_boards() {
        let mut json: serde_json::Value = serde_json::from_str(&sample()).unwrap();
        let right_axes=[
            [1.0,0.0,0.0], [0.0,0.0,1.0], [0.0,0.0,-1.0],
        ];
        let up=[0.0,1.0,0.0];
        let labels=["array_superior","array_left","array_right"];
        for (idx, name) in labels.iter().enumerate() {
            let center=[idx as f64 * 40.0,0.0,0.0];
            let right=right_axes[idx];
            let normal=cross(right,up);
            let mut chip_pts=Vec::new();
            for col in 0..4 {
                for row in 0..4 {
                    let x=-6.75+4.5*col as f64;
                    let y=-6.75+4.5*row as f64;
                    chip_pts.push([
                        center[0]+x*right[0]+y*up[0],
                        center[1]+x*right[1]+y*up[1],
                        center[2]+x*right[2]+y*up[2],
                    ]);
                }
            }
            chip_pts.reverse(); // CT detector order is deliberately different
            let obj=&mut json["objects"][idx*2+1];
            obj["name"]=(*name).into();
            obj["center_lps_mm"]=serde_json::json!(center);
            obj["chip_grid_lps_mm"]=serde_json::json!(chip_pts);
            obj["board_normal_lps"]=serde_json::json!(normal);
            obj["board_center_lps_mm"]=serde_json::json!([
                center[0],center[1]-8.0,center[2]
            ]);
        }
        let geometry=CbctGeometry::from_detector_json(
            &json.to_string(),"synthetic_scan".into(),"qc".into()).unwrap();
        let boards=geometry.register_ct_boards_v2().unwrap();
        assert_eq!(boards.len(),3);
        for (idx,b) in boards.iter().enumerate() {
            assert_eq!(b.detection_name,labels[idx]);
            assert_eq!(b.raw_board_id,idx as u16);
            assert_eq!(b.firmware_to_detector[0],15);
            assert_eq!(b.firmware_to_detector[15],0);
            assert!(b.geometric_rms_mm<1e-9);
            assert!((b.jst_proxy_offset_mm-8.0).abs()<1e-9);
            // raw Bx = -up, raw By = right, raw Bz = chip-face normal
            for k in 0..3 {
                assert!((b.raw_to_lps[k][0]+up[k]).abs()<1e-10);
                assert!((b.raw_to_lps[k][1]-right_axes[idx][k]).abs()<1e-10);
            }
        }
        let profile=geometry.experimental_candidate_profile().unwrap();
        assert_eq!(profile.boards.len(),3);
        assert_eq!(profile.magnets.len(),3);
        assert_eq!(profile.boards[0].sensors_mm[0],boards[0].sensors_lps_mm[0]);
        let mut missing=json.clone();
        missing["objects"][1].as_object_mut().unwrap().remove("board_normal_lps");
        let broken=CbctGeometry::from_detector_json(
            &missing.to_string(),"synthetic_scan".into(),"qc".into()).unwrap();
        assert!(broken.register_ct_boards_v2().is_err());
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
