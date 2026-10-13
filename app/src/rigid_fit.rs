//! Joint six-DOF finite-cylinder fit for three synchronized Hall boards.
//! Geometry and magnetic calibration are immutable during live tracking.
//! See docs/LIVE_RIGID_FITTING.md for profile provenance and acquisition conventions.
use std::collections::VecDeque;
use std::f64::consts::PI;
use std::time::Instant;

use data_transfer::rpc::{BoardFrame, BoardPresence};
use serde::Deserialize;
use crate::live_fit::DisplacementPoint;
use crate::stage_presets::MagnetLocationPreset;

const HISTORY_LEN: usize = 600;
const FRAME_BUFFER_LEN: usize = 8;
const MAX_INTERPOLATION_US: u64 = 120_000;
const RAD_TO_DEG: f64 = 180.0 / PI;
const LEGENDRE_NODES: [f64; 4] = [
    -0.8611363115940526, -0.3399810435848563,
     0.3399810435848563,  0.8611363115940526,
];
const LEGENDRE_WEIGHTS: [f64; 4] = [
    0.3478548451374538, 0.6521451548625461,
    0.6521451548625461, 0.3478548451374538,
];

#[derive(Debug, Clone, Deserialize)]
pub struct RigidProfile {
    pub name: String,
    /// A tagged coordinate frame is needed before selecting a measured stage preset.
    /// Older/untagged files remain loadable as custom profiles but are never
    /// silently identified with CT or rotation-stage reference locations.
    #[serde(default)]
    pub coordinate_frame: Option<String>,
    /// Point about which the baseline constellation rotates.
    pub origin_mm: [f64; 3],
    /// Each board names its actual firmware ID; this is not a physical board index.
    pub boards: Vec<RigidBoard>,
    pub magnets: Vec<RigidMagnet>,
    /// Translation (mm) and rotation vector (radians).
    pub initial_pose: [f64; 6],
    pub lower_bounds: [f64; 6],
    pub upper_bounds: [f64; 6],
    pub max_rmse_mt: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RigidBoard {
    pub raw_board_id: u16,
    /// Sensor positions in the common (world/CT) coordinate frame, firmware indices 0..15.
    pub sensors_mm: Vec<[f64; 3]>,
    /// Multiply raw [Bx,By,Bz] by this matrix to obtain field components in world coordinates.
    pub raw_to_world: [[f64; 3]; 3],
    /// Frozen background in world coordinates, subtracted from live measurements.
    pub background_mt: [f64; 3],
    /// Optional per-sensor scalar gains (observations are divided by them).
    #[serde(default)]
    pub sensor_gains: Option<Vec<f64>>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RigidMagnet {
    /// Baseline center and signed axis in the same coordinate frame as sensors.
    pub center_mm: [f64; 3],
    pub axis: [f64; 3],
    pub diameter_mm: f64,
    pub thickness_mm: f64,
    /// Signed effective remanence; never re-estimated by the pose optimizer.
    pub remanence_mt: f64,
}

impl RigidProfile {
    pub fn from_json(source: &str) -> Result<Self, String> {
        let profile: Self = serde_json::from_str(source).map_err(|e| e.to_string())?;
        profile.validate()?;
        Ok(profile)
    }

    pub fn a1_reference() -> Result<Self, String> {
        Self::from_json(include_str!("../config/a1_rigid_reference.json"))
    }

    fn validate(&self) -> Result<(), String> {
        if self.boards.len() != 3 || self.magnets.len() != 3 {
            return Err("Rigid fitting needs exactly three boards and three magnets".into());
        }
        if self.name.trim().is_empty() || self.max_rmse_mt <= 0.0 || !self.max_rmse_mt.is_finite() {
            return Err("Profile name and positive finite max_rmse_mt are required".into());
        }
        if !self.origin_mm.iter().all(|x| x.is_finite()) {
            return Err("Nonfinite origin".into());
        }
        for i in 0..6 {
            if !self.initial_pose[i].is_finite()
                || !self.lower_bounds[i].is_finite()
                || !self.upper_bounds[i].is_finite()
                || self.lower_bounds[i] >= self.upper_bounds[i]
                || self.initial_pose[i] < self.lower_bounds[i]
                || self.initial_pose[i] > self.upper_bounds[i]
            {
                return Err(format!("Invalid parameter bounds or seed at index {i}"));
            }
        }
        let mut seen = [false; 3];
        for b in &self.boards {
            if b.raw_board_id > 2 || seen[b.raw_board_id as usize] {
                return Err("Board firmware IDs must be a permutation of 0, 1, 2".into());
            }
            seen[b.raw_board_id as usize] = true;
            if b.sensors_mm.len() != 16
                || b.sensors_mm.iter().flatten().any(|x| !x.is_finite())
                || b.raw_to_world.iter().flatten().any(|x| !x.is_finite())
                || b.background_mt.iter().any(|x| !x.is_finite())
            {
                return Err(format!("Invalid sensor geometry/field transform for board {}", b.raw_board_id));
            }
            if let Some(gains) = &b.sensor_gains {
                if gains.len() != 16 || gains.iter().any(|x| !x.is_finite() || *x <= 0.0) {
                    return Err("Sensor gain array must have 16 positive finite values".into());
                }
            }
            // Reject accidental transposes/scale factors in the sensor-axis transform.
            for i in 0..3 {
                for j in 0..3 {
                    let dot = dot(b.raw_to_world[i], b.raw_to_world[j]);
                    if (dot - if i == j { 1.0 } else { 0.0 }).abs() > 0.02 {
                        return Err(format!("Board {} field rotation is not orthonormal", b.raw_board_id));
                    }
                }
            }
        }
        for m in &self.magnets {
            if m.center_mm.iter().chain(m.axis.iter()).any(|x| !x.is_finite())
                || !m.remanence_mt.is_finite()
                || m.diameter_mm <= 0.0 || m.thickness_mm <= 0.0
                || !m.diameter_mm.is_finite() || !m.thickness_mm.is_finite()
                || (dot(m.axis, m.axis) - 1.0).abs() > 0.02
            {
                return Err("Invalid magnet geometry, signed field scale, or non-unit axis".into());
            }
        }
        Ok(())
    }
}

#[derive(Clone)]
struct BoardSnapshot {
    time_us: u64,
    fields_mt: [Option<[f64; 3]>; 16],
}

#[derive(Debug, Clone)]
struct Observation {
    sensor_mm: [f64; 3],
    measured_mt: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct RigidFitResult {
    pub pose: [f64; 6],
    pub rmse_mt: f64,
    pub sensor_count: usize,
}

#[derive(Debug, Clone)]
pub struct RigidFitJob {
    pub generation: u64,
    pub time_us: u64,
    profile: RigidProfile,
    observations: Vec<Observation>,
    seed: [f64; 6],
    cold_start: bool,
}

#[derive(Debug, Clone)]
pub struct RigidFitCompletion {
    pub generation: u64,
    pub time_us: u64,
    pub result: Option<RigidFitResult>,
    pub elapsed_ms: f64,
}

impl RigidFitJob {
    pub fn run(self) -> RigidFitCompletion {
        let start = Instant::now();
        // Warm-start is fast during ordinary tracking, but the weak far-field
        // A1 signal can lead to a falsely accepted pose on an XYZ bound.
        // As the carrier approaches a board, reusing that trapped seed can
        // yield a ~1.6 mT residual even though the calibrated model fits the
        // same measured fields at <0.04 mT. Retry from calibrated seeds only
        // when the warm-start RMSE fails the existing acceptance threshold.
        let mut best = optimize_pose(&self.profile, &self.observations, self.seed);
        let a1 = self.profile.name.starts_with("A1 ");
        if self.cold_start || (a1 && best.as_ref()
            .is_none_or(|r| r.rmse_mt > self.profile.max_rmse_mt))
        {
            let mut seeds = vec![self.profile.initial_pose];
            // A1 Z is an absolute magnet-board gap. The CT/GK profile
            // uses relative displacement; do not introduce A1 gap seeds.
            if a1 {
                for z in [22.0, 32.0, 44.0, 55.0] {
                    let mut pose = self.profile.initial_pose;
                    pose[2] = z;
                    seeds.push(pose);
                }
            }
            for seed in seeds {
                if seed == self.seed { continue; }
                if let Some(result) = optimize_pose(&self.profile, &self.observations, seed) {
                    if best.as_ref().is_none_or(|b| result.rmse_mt < b.rmse_mt) {
                        best = Some(result);
                    }
                }
                // During ordinary A1 tracking, stop once recovery has
                // produced an acceptable fit; extra seeds waste CPU.
                if !self.cold_start && best.as_ref()
                    .is_some_and(|b| b.rmse_mt <= self.profile.max_rmse_mt) {
                    break;
                }
            }
        }
        RigidFitCompletion {
            generation: self.generation,
            time_us: self.time_us,
            result: best,
            elapsed_ms: start.elapsed().as_secs_f64() * 1000.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RigidXyzPoint {
    pub time_s: f64,
    pub dx_mm: f64,
    pub dy_mm: f64,
    pub dz_mm: f64,
}

#[derive(Debug, Clone)]
pub struct RigidSummary {
    pub enabled: bool,
    pub location_preset: Option<MagnetLocationPreset>,
    /// True only after background captured on all 3 A1 boards.
    pub a1_background_ready: bool,
    pub zero_ready: bool,
    pub profile_name: Option<String>,
    pub status: String,
    pub result: Option<RigidFitResult>,
    pub translation_from_zero: Option<[f64; 3]>,
    pub rotation_from_zero_deg: Option<f64>,
    pub displacement_history: Vec<DisplacementPoint>,
    pub xyz_history: Vec<RigidXyzPoint>,
    pub rotation_history: Vec<DisplacementPoint>,
    pub busy: bool,
    pub completed: u64,
    pub last_duration_ms: Option<f64>,
    pub rejected: u64,
}

#[derive(Default)]
pub struct RigidLiveFit {
    profile: Option<RigidProfile>,
    location_preset: Option<MagnetLocationPreset>,
    enabled: bool,
    frames: [VecDeque<BoardSnapshot>; 3],
    presence: BoardPresence,
    /// A1 reference background from live sensor frames; the printer's G-code
    /// is responsible for moving the magnets sufficiently far away.
    background_raw: [Option<[[f64;3];16]>;3],
    generation: u64,
    busy: bool,
    last_dispatched_us: Option<u64>,
    first_time_us: Option<u64>,
    last_elapsed_ms: Option<f64>,
    completed: u64,
    rejected: u64,
    result: Option<RigidFitResult>,
    zero_pose: Option<[f64; 6]>,
    displacement: VecDeque<DisplacementPoint>,
    xyz: VecDeque<RigidXyzPoint>,
    rotation: VecDeque<DisplacementPoint>,
    status: String,
}

impl RigidLiveFit {
    pub fn set_profile(&mut self, profile: RigidProfile) {
        // Switching to an actual calibrated profile also selects the matching
        // stage reference, if any; never allow stale location references to
        // override measured board/sensor geometry.
        self.location_preset = MagnetLocationPreset::identify_profile(&profile);
        self.profile = Some(profile);
        self.background_raw = [None;3];
        self.enabled = true;
        self.reset_stream();
    }

    /// The A1 selection loads the bundled complete calibration automatically.
    /// The CT/rotation-stage selections are reference centers only until the
    /// user imports matching calibrated sensor geometry and magnetic nuisance data.
    pub fn select_location_preset(&mut self, preset: MagnetLocationPreset) {
        if preset == MagnetLocationPreset::A1Demo {
            match RigidProfile::a1_reference() {
                Ok(profile) => self.set_profile(profile),
                Err(error) => {
                    self.profile = None;
                    self.location_preset = Some(preset);
                    self.enabled = false;
                    self.reset_stream();
                    self.status = format!("Cannot load bundled A1 V1 setup: {error}");
                }
            }
            return;
        }
        let matching = self.profile.as_ref().is_some_and(|p| preset.matches_profile(p));
        self.location_preset = Some(preset);
        if !matching {
            self.profile = None;
            self.background_raw = [None; 3];
        }
        self.enabled = matching;
        self.reset_stream();
        if !matching {
            self.status = format!(
                "{} reference centers selected. Load a matching complete sensor/board/magnet calibration JSON before fitting.",
                preset.title()
            );
        }
    }

    pub fn set_status(&mut self, status: impl Into<String>) {
        self.status = status.into();
    }

    pub fn enabled(&self) -> bool { self.enabled }

    pub fn set_presence(&mut self, presence: BoardPresence) {
        self.presence = presence;
        self.reset_stream();
    }

    pub fn reset_stream(&mut self) {
        self.status = if !self.enabled {
            "Rigid fitting disabled".into()
        } else if self.profile.as_ref().is_some_and(|p| p.name.starts_with("A1 "))
            && self.background_raw.iter().any(Option::is_none)
        {
            "Capture A1 background on all 3 boards before fitting".into()
        } else {
            "Waiting for synchronized board frames".into()
        };
        self.generation = self.generation.wrapping_add(1);
        self.frames.iter_mut().for_each(VecDeque::clear);
        self.busy = false;
        self.last_dispatched_us = None;
        self.first_time_us = None;
        self.zero_pose = None;
        self.result = None;
        self.displacement.clear();
        self.xyz.clear();
        self.rotation.clear();
        self.completed = 0;
        self.rejected = 0;
        self.last_elapsed_ms = None;
    }

    /// Capture the existing raw A1 background (one complete frame per board).
    /// Timing and the magnet separation are controlled by the printer G-code.
    pub fn capture_a1_background(&mut self) -> Result<[[[f64; 3]; 16]; 3], String> {
        if !self.profile.as_ref().is_some_and(|p| p.name.starts_with("A1 ")) {
            return Err("Background capture is only offered for the A1 reference; CT uses frozen fitted background".into());
        }
        let mut captures = [[[0.0; 3]; 16]; 3];
        for board_id in 0..3 {
            let frame = self.frames[board_id].back().ok_or("No frame available on every board")?;
            for (i, v) in frame.fields_mt.iter().enumerate() {
                captures[board_id][i] = v.ok_or("Background frame is missing sensors")?;
            }
        }
        // Publish only after every sensor on every board has been checked.
        self.background_raw = captures.map(Some);
        self.reset_stream();
        self.status = "A1 background captured on all 3 boards".into();
        Ok(captures)
    }

    pub fn clear_background_for_new_port(&mut self) {
        self.background_raw = [None;3];
        self.reset_stream();
        self.status = "Board connection changed: verify profile and recapture backgrounds".into();
    }

    pub fn reset_zero(&mut self) {
        if let Some(result) = &self.result {
            self.zero_pose = Some(result.pose);
            self.first_time_us = self.last_dispatched_us;
            self.displacement.clear();
            self.xyz.clear();
            self.rotation.clear();
            self.status = "Tracking; rigid zero set to current pose".into();
        } else {
            self.status = "Cannot set rigid zero yet: wait for an accepted fit".into();
        }
    }

    pub fn summary(&self) -> RigidSummary {
        let rel = self.result.as_ref().and_then(|r| {
            self.zero_pose.map(|z| {
                let d = [
                    r.pose[0] - z[0],
                    r.pose[1] - z[1],
                    r.pose[2] - z[2],
                ];
                let rz = rodrigues([z[3], z[4], z[5]]);
                let rr = rodrigues([r.pose[3], r.pose[4], r.pose[5]]);
                let relative = mul_matrix(transpose(rz), rr);
                let trace = relative[0][0] + relative[1][1] + relative[2][2];
                let angle = ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos() * RAD_TO_DEG;
                (d, angle)
            })
        });
        RigidSummary {
            enabled: self.enabled,
            location_preset: self.location_preset,
            a1_background_ready: self.profile.as_ref()
                .is_some_and(|p| p.name.starts_with("A1 "))
                && self.background_raw.iter().all(Option::is_some),
            zero_ready: self.zero_pose.is_some(),
            profile_name: self.profile.as_ref().map(|p| p.name.clone()),
            status: self.status.clone(),
            result: self.result.clone(),
            translation_from_zero: rel.map(|r| r.0),
            rotation_from_zero_deg: rel.map(|r| r.1),
            displacement_history: self.displacement.iter().cloned().collect(),
            xyz_history: self.xyz.iter().cloned().collect(),
            rotation_history: self.rotation.iter().cloned().collect(),
            busy: self.busy,
            completed: self.completed,
            last_duration_ms: self.last_elapsed_ms,
            rejected: self.rejected,
        }
    }

    /// Always receive the board's raw frames for A1 background capture.
    /// Only launch the expensive rigid solver when its tracking mode is selected.
    pub fn receive_frame(&mut self, frame: &BoardFrame, fit_active: bool) -> Option<RigidFitJob> {
        if !self.enabled { return None; }
        let profile = self.profile.as_ref()?;
        let b_id = frame.board_id as usize;
        if b_id >= 3 || !profile.boards.iter().any(|b| b.raw_board_id == frame.board_id)
            || self.presence.board_mask & (1u8 << b_id) == 0
        { return None; }
        let expected = self.presence.sensor_masks[b_id];
        if (frame.sensor_mask & expected).count_ones() < 8 { return None; }
        let mut fields_mt = [None; 16];
        let mut min_t = u64::MAX;
        let mut max_t = 0_u64;
        for (i, f) in frame.samples.iter().enumerate() {
            if (frame.sensor_mask & expected) & (1u16 << i) == 0 { continue; }
            if !(f.bx_ut.is_finite() && f.by_ut.is_finite() && f.bz_ut.is_finite()) {
                continue;
            }
            fields_mt[i] = Some([
                f.bx_ut as f64 / 1000.0,
                f.by_ut as f64 / 1000.0,
                f.bz_ut as f64 / 1000.0,
            ]);
            let time = frame.base_time_us + u64::from(f.time_offset_us);
            min_t = min_t.min(time);
            max_t = max_t.max(time);
        }
        if min_t == u64::MAX { return None; }
        let t = min_t + (max_t - min_t) / 2;
        let queue = &mut self.frames[b_id];
        if queue.back().is_some_and(|f| f.time_us >= t) { return None; }
        queue.push_back(BoardSnapshot { time_us: t, fields_mt });
        while queue.len() > FRAME_BUFFER_LEN { queue.pop_front(); }

        // In independent mode, continue buffering three-board fields, but
        // don't fit. That keeps shared A1 background capture functional
        // without forcing the rigid solver to compete for CPU.
        if !fit_active { return None; }

        if profile.name.starts_with("A1 ") && self.background_raw.iter().any(Option::is_none) {
            self.status = "Awaiting A1 background capture on all three boards".into();
            return None;
        }
        if self.busy { return None; }
        for b in &profile.boards {
            if self.frames[b.raw_board_id as usize].len() < 2 { return None; }
        }
        // Latest common timestamp, bracketed independently by each board.
        let common_t = profile.boards.iter()
            .filter_map(|b| self.frames[b.raw_board_id as usize].back().map(|f| f.time_us))
            .min()?;
        if self.last_dispatched_us.is_some_and(|last| common_t <= last) { return None; }

        let mut observations = Vec::with_capacity(48);
        for b in &profile.boards {
            let samples = interpolate(&self.frames[b.raw_board_id as usize], common_t)?;
            let mut used = 0;
            for (i, raw) in samples.iter().enumerate() {
                let Some(raw) = raw else { continue };
                let gain = b.sensor_gains.as_ref().map(|g| g[i]).unwrap_or(1.0);
                let background = self.background_raw[b.raw_board_id as usize]
                    .as_ref().map(|samples| samples[i]).unwrap_or([0.0;3]);
                let converted = mat_vec(b.raw_to_world, sub(*raw, background));
                observations.push(Observation {
                    sensor_mm: b.sensors_mm[i],
                    measured_mt: [
                        converted[0] / gain - b.background_mt[0],
                        converted[1] / gain - b.background_mt[1],
                        converted[2] / gain - b.background_mt[2],
                    ],
                });
                used += 1;
            }
            if used < 8 { return None; }
        }

        self.busy = true;
        self.last_dispatched_us = Some(common_t);
        self.status = "Fitting synchronized 6-DOF pose".into();
        Some(RigidFitJob {
            generation: self.generation,
            time_us: common_t,
            profile: profile.clone(),
            observations,
            seed: self.result.as_ref().map(|r| r.pose).unwrap_or(profile.initial_pose),
            cold_start: self.result.is_none(),
        })
    }

    pub fn apply_completion(&mut self, completion: RigidFitCompletion) {
        if completion.generation != self.generation { return; }
        self.busy = false;
        self.completed = self.completed.saturating_add(1);
        self.last_elapsed_ms = Some(completion.elapsed_ms);
        let Some(profile) = &self.profile else { return; };
        let Some(result) = completion.result else {
            self.rejected += 1;
            self.status = "Solver failed (previous pose retained)".into();
            return;
        };
        if result.rmse_mt > profile.max_rmse_mt {
            self.rejected += 1;
            self.status = format!(
                "Fit rejected: {:.3} mT exceeds {:.3} mT threshold",
                result.rmse_mt, profile.max_rmse_mt
            );
            return;
        }
        let zero = *self.zero_pose.get_or_insert(result.pose);
        let initial_time = *self.first_time_us.get_or_insert(completion.time_us);
        let translation = [
            result.pose[0] - zero[0],
            result.pose[1] - zero[1],
            result.pose[2] - zero[2],
        ];
        let translation_norm = dot(translation, translation).sqrt();
        let relative = mul_matrix(
            transpose(rodrigues([zero[3], zero[4], zero[5]])),
            rodrigues([result.pose[3], result.pose[4], result.pose[5]])
        );
        let trace = relative[0][0] + relative[1][1] + relative[2][2];
        let angle_deg = ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos() * RAD_TO_DEG;
        let time_s = completion.time_us.saturating_sub(initial_time) as f64 / 1_000_000.0;
        self.displacement.push_back(DisplacementPoint {time_s, displacement_mm: translation_norm});
        self.xyz.push_back(RigidXyzPoint {
            time_s,
            dx_mm: translation[0],
            dy_mm: translation[1],
            dz_mm: translation[2],
        });
        self.rotation.push_back(DisplacementPoint {time_s, displacement_mm: angle_deg});
        while self.displacement.len() > HISTORY_LEN { self.displacement.pop_front(); }
        while self.xyz.len() > HISTORY_LEN { self.xyz.pop_front(); }
        while self.rotation.len() > HISTORY_LEN { self.rotation.pop_front(); }
        self.result = Some(result);
        self.status = "Tracking".into();
    }
}

fn interpolate(queue: &VecDeque<BoardSnapshot>, time_us: u64) -> Option<[Option<[f64; 3]>; 16]> {
    let mut a = None;
    let mut b = None;
    for item in queue {
        if item.time_us <= time_us { a = Some(item); }
        if item.time_us >= time_us { b = Some(item); break; }
    }
    let a = a?;
    let b = b?;
    if time_us.saturating_sub(a.time_us) > MAX_INTERPOLATION_US
        || b.time_us.saturating_sub(time_us) > MAX_INTERPOLATION_US
    { return None; }
    let fraction = if a.time_us == b.time_us { 0.0 }
        else { (time_us - a.time_us) as f64 / (b.time_us - a.time_us) as f64 };
    let mut out = [None; 16];
    for (i, dst) in out.iter_mut().enumerate() {
        if let (Some(v0), Some(v1)) = (a.fields_mt[i], b.fields_mt[i]) {
            *dst = Some([
                v0[0] + fraction * (v1[0] - v0[0]),
                v0[1] + fraction * (v1[1] - v0[1]),
                v0[2] + fraction * (v1[2] - v0[2]),
            ]);
        }
    }
    Some(out)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 { a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn scale(a: [f64; 3], t: f64) -> [f64; 3] { [a[0]*t,a[1]*t,a[2]*t] }
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
}
fn mat_vec(m: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [dot(m[0],v),dot(m[1],v),dot(m[2],v)]
}
fn transpose(m: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [[m[0][0],m[1][0],m[2][0]],[m[0][1],m[1][1],m[2][1]],[m[0][2],m[1][2],m[2][2]]]
}
fn mul_matrix(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let bt = transpose(b);
    [
        [dot(a[0],bt[0]),dot(a[0],bt[1]),dot(a[0],bt[2])],
        [dot(a[1],bt[0]),dot(a[1],bt[1]),dot(a[1],bt[2])],
        [dot(a[2],bt[0]),dot(a[2],bt[1]),dot(a[2],bt[2])],
    ]
}
fn rodrigues(v: [f64; 3]) -> [[f64; 3]; 3] {
    let angle2 = dot(v,v);
    let (alpha,beta) = if angle2 < 1e-12 {
        (1.0-angle2/6.0,0.5-angle2/24.0)
    } else {
        let a = angle2.sqrt();
        (a.sin()/a,(1.0-a.cos())/angle2)
    };
    let k = [[0.0,-v[2],v[1]],[v[2],0.0,-v[0]],[-v[1],v[0],0.0]];
    let k2 = mul_matrix(k,k);
    let mut result = [[0.0;3];3];
    for i in 0..3 { for j in 0..3 {
        result[i][j] = (if i==j {1.0} else {0.0}) + alpha*k[i][j] + beta*k2[i][j];
    }}
    result
}

fn quadrature(magnet: &RigidMagnet, center: [f64; 3], axis: [f64; 3]) -> Vec<([f64; 3], [f64; 3], f64)> {
    let u = scale(axis, 1.0 / dot(axis,axis).sqrt());
    let ref_axis = if u[2].abs() > 0.9 { [0.0,1.0,0.0] } else {[0.0,0.0,1.0]};
    let e1 = cross(ref_axis,u);
    let e1 = scale(e1, 1.0 / dot(e1,e1).sqrt());
    let e2 = cross(u,e1);
    let mut out = Vec::with_capacity(48);
    let radius = magnet.diameter_mm * 0.5;
    for (&node,&weight) in LEGENDRE_NODES.iter().zip(LEGENDRE_WEIGHTS.iter()) {
        let r = 0.5*radius*(node+1.0);
        let rw = 0.5*radius*weight;
        for k in 0..12 {
            let theta = 2.0*PI*k as f64 / 12.0;
            let offset = add(scale(e1,r*theta.cos()), scale(e2,r*theta.sin()));
            let top = add(add(center,offset),scale(u,magnet.thickness_mm*0.5));
            let bottom = sub(add(center,offset),scale(u,magnet.thickness_mm*0.5));
            out.push((top,bottom,r*rw*(2.0*PI/12.0)*magnet.remanence_mt/(4.0*PI)));
        }
    }
    out
}

fn predict(sensor: [f64; 3], sources: &[Vec<([f64; 3],[f64; 3],f64)>]) -> [f64;3] {
    let mut result = [0.0;3];
    for magnet_sources in sources {
        for &(top,bottom,w) in magnet_sources {
            let rt = sub(sensor,top);
            let rb = sub(sensor,bottom);
            let r2t = dot(rt,rt).max(1e-12);
            let r2b = dot(rb,rb).max(1e-12);
            let contribution = sub(scale(rt,1.0/(r2t*r2t.sqrt())),scale(rb,1.0/(r2b*r2b.sqrt())));
            result = add(result,scale(contribution,w));
        }
    }
    result
}

fn residual(profile: &RigidProfile, observations: &[Observation], pose: [f64;6]) -> Option<Vec<f64>> {
    let rotation = rodrigues([pose[3],pose[4],pose[5]]);
    let mut sources = Vec::with_capacity(profile.magnets.len());
    for magnet in &profile.magnets {
        let center = add(
            add(profile.origin_mm,[pose[0],pose[1],pose[2]]),
            mat_vec(rotation,sub(magnet.center_mm,profile.origin_mm))
        );
        let axis = mat_vec(rotation,magnet.axis);
        sources.push(quadrature(magnet,center,axis));
    }
    let mut out = Vec::with_capacity(3*observations.len());
    for sample in observations {
        let modeled = predict(sample.sensor_mm,&sources);
        for i in 0..3 { out.push(modeled[i]-sample.measured_mt[i]); }
    }
    if out.iter().any(|x| !x.is_finite()) { return None; }
    Some(out)
}

fn solve_6(mut a: [[f64;6];6], mut b: [f64;6]) -> Option<[f64;6]> {
    for pivot in 0..6 {
        let max = (pivot..6).max_by(|&i,&j|
            a[i][pivot].abs().total_cmp(&a[j][pivot].abs())
        )?;
        if a[max][pivot].abs() < 1e-20 { return None; }
        a.swap(pivot,max); b.swap(pivot,max);
        let d = a[pivot][pivot];
        for j in pivot..6 { a[pivot][j] /= d; }
        b[pivot] /= d;
        for row in 0..6 {
            if row == pivot { continue; }
            let t = a[row][pivot];
            for col in pivot..6 { a[row][col] -= t*a[pivot][col]; }
            b[row] -= t*b[pivot];
        }
    }
    Some(b)
}

fn optimize_pose(profile: &RigidProfile, observations: &[Observation], initial: [f64;6]) -> Option<RigidFitResult> {
    let mut p = initial;
    for i in 0..6 { p[i] = p[i].clamp(profile.lower_bounds[i],profile.upper_bounds[i]); }
    let mut r = residual(profile,observations,p)?;
    let mut cost: f64 = r.iter().map(|x| x*x).sum();
    let mut damping = 1e-3;
    for _ in 0..45 {
        let mut columns = Vec::with_capacity(6);
        for j in 0..6 {
            let h = if j < 3 { 0.02 } else { 0.0002 };
            let mut test = p;
            test[j] = (p[j] + h).min(profile.upper_bounds[j]);
            if test[j] == p[j] {test[j] = (p[j]-h).max(profile.lower_bounds[j]);}
            let delta = test[j]-p[j];
            if delta.abs() < 1e-12 {return None;}
            let rh = residual(profile,observations,test)?;
            columns.push(rh.iter().zip(&r).map(|(a,b)| (a-b)/delta).collect::<Vec<f64>>());
        }
        let mut hessian = [[0.0;6];6];
        let mut gradient = [0.0;6];
        for i in 0..6 {
            gradient[i] = -columns[i].iter().zip(&r).map(|(j,r)| j*r).sum::<f64>();
            for j in 0..6 {
                hessian[i][j] = columns[i].iter().zip(&columns[j]).map(|(a,b)| a*b).sum();
            }
        }
        let diagonal = std::array::from_fn::<f64,6,_>(|i| hessian[i][i].max(1e-10));
        let mut accepted = false;
        let mut step_small = false;
        for _ in 0..9 {
            let mut regularized = hessian;
            for i in 0..6 { regularized[i][i] += damping*diagonal[i]; }
            let Some(step) = solve_6(regularized,gradient) else {return None};
            let mut trial = p;
            for i in 0..6 {
                trial[i] = (p[i]+step[i]).clamp(profile.lower_bounds[i],profile.upper_bounds[i]);
            }
            step_small = (0..6).map(|i| (trial[i]-p[i]).abs()).fold(0.0_f64,f64::max) < 1e-7;
            let candidate = residual(profile,observations,trial)?;
            let trial_cost = candidate.iter().map(|x| x*x).sum::<f64>();
            if trial_cost < cost {
                let improvement = cost - trial_cost;
                cost = trial_cost; p = trial; r = candidate;
                damping = (damping*0.35).max(1e-8);
                accepted = true;
                if improvement < 1e-12 { step_small = true; }
                break;
            }
            damping = (damping*6.0).min(1e9);
        }
        if step_small || !accepted { break; }
    }
    let rmse = (cost / (3*observations.len()) as f64).sqrt();
    if !rmse.is_finite() {return None;}
    Some(RigidFitResult {pose:p,rmse_mt:rmse,sensor_count:observations.len()})
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn profile_is_valid() { RigidProfile::a1_reference().unwrap(); }

    /// Small recorded-data fixture, NOT fields synthesized from the profile
    /// currently under test. Captured with the 3-board A1 V1 demo at gaps
    /// 25/30/40 mm while the carrier was moving in X or Y.
    ///
    /// This must fail if any bundled magnet polarity is inadvertently
    /// reversed again, or if coordinate/magnetic-model changes move fitted
    /// XYZ substantially or push field residuals above acceptance.
    #[test]
    fn measured_a1_three_gap_moving_fit_polarity_regression() {
        let profile = RigidProfile::a1_reference().expect("bundled A1 profile");
        let mut raw_ut = [[[[0.0_f64; 3]; 16]; 3]; 4];
        let mut rows_seen = [[false; 3]; 4];

        for row in include_str!("../testdata/a1_three_gap_fields_ut.csv").lines() {
            if row.is_empty() || row.starts_with('#') {
                continue;
            }
            let mut fields = row.split(',');
            let stage = match fields.next().expect("stage") {
                "background" => 0,
                "25" => 1,
                "30" => 2,
                "40" => 3,
                value => panic!("unknown recorded stage {value}"),
            };
            let board: usize = fields.next().expect("board").parse().expect("board ID");
            assert!(board < 3, "unexpected board ID");
            assert!(!rows_seen[stage][board], "duplicate recorded stage/board");
            rows_seen[stage][board] = true;
            let values: Vec<f64> = fields.map(|s| s.parse().expect("recorded µT")).collect();
            assert_eq!(values.len(), 16 * 3, "truncated measured board frame");
            for sensor in 0..16 {
                raw_ut[stage][board][sensor] = [
                    values[sensor*3],
                    values[sensor*3+1],
                    values[sensor*3+2],
                ];
            }
        }
        assert!(rows_seen.iter().flatten().all(|&seen| seen), "incomplete real-data fixture");

        // The measured BG is a stationary median of the first 8 seconds.
        // The expected poses come from independently reconstructing the
        // actual XY motion and commanded 25/30/40 mm gap sequence.
        let expected_translation = [
            [-4.926, -0.078, 26.650],
            [ 1.299, -0.111, 31.652],
            [-0.017, -2.052, 41.672],
        ];
        let gaps = [25, 30, 40];
        for case in 0..3 {
            let mut observations = Vec::with_capacity(48);
            for board in &profile.boards {
                let b = board.raw_board_id as usize;
                for sensor in 0..16 {
                    let observed_raw_mt = raw_ut[case+1][b][sensor].map(|v| v / 1000.0);
                    let background_mt = raw_ut[0][b][sensor].map(|v| v / 1000.0);
                    let corrected = mat_vec(
                        board.raw_to_world,
                        sub(observed_raw_mt, background_mt),
                    );
                    let gain = board.sensor_gains.as_ref()
                        .map(|g| g[sensor]).unwrap_or(1.0);
                    let measured_mt = std::array::from_fn(|i| {
                        corrected[i] / gain - board.background_mt[i]
                    });
                    observations.push(Observation {
                        sensor_mm: board.sensors_mm[sensor],
                        measured_mt,
                    });
                }
            }

            let xyz = expected_translation[case];
            let seed = [0.0, 0.0, xyz[2], 0.0, 0.0, 0.0];
            let fit = optimize_pose(&profile, &observations, seed)
                .unwrap_or_else(|| panic!("{} mm: optimization failed", gaps[case]));
            assert!(
                fit.rmse_mt < profile.max_rmse_mt,
                "{} mm: recorded moving pose RMSE {:.4} mT exceeds {:.4} mT",
                gaps[case], fit.rmse_mt, profile.max_rmse_mt,
            );
            assert_eq!(fit.sensor_count, 48);
            for (axis, expected) in xyz.iter().enumerate() {
                assert!(
                    (fit.pose[axis] - expected).abs() < 0.75,
                    "{} mm: XYZ axis {} recovered {:.3} mm, expected {:.3} mm",
                    gaps[case], axis, fit.pose[axis], expected,
                );
            }

            // Ensure each of the three signs is genuinely constrained
            // by measured fields, not merely checked by a literal
            // sign assertion in the configuration file.
            for magnet_index in 0..3 {
                let mut wrong_polarity = profile.clone();
                wrong_polarity.magnets[magnet_index].remanence_mt *= -1.0;
                let residuals = residual(&wrong_polarity, &observations, fit.pose)
                    .expect("finite wrong-polarity residuals");
                let wrong_rmse = (
                    residuals.iter().map(|r| r*r).sum::<f64>() / residuals.len() as f64
                ).sqrt();
                assert!(
                    wrong_rmse > profile.max_rmse_mt,
                    "{} mm: magnet {} sign flip unexpectedly fits measured fields",
                    gaps[case], magnet_index,
                );
            }
        }
    }


    /// Replays *measured* raw fields from the second A1 demo through the
    /// production frame -> synchronization -> job -> completion path.
    /// The approach to 25 mm previously got stuck using a far-field pose
    /// on the XYZ bounds: warm-only RMSE rose to roughly 1.6 mT while a
    /// properly initialized fit remained <0.04 mT.
    #[test]
    fn measured_a1_live_approach_recovers_from_far_field_seed() {
        let mut recorded = [[[[0.0_f32; 3]; 16]; 3]; 7];
        let mut present = [[false; 3]; 7];
        for line in include_str!("../testdata/a1_second_demo_live_approach_ut.csv").lines() {
            if line.is_empty() || line.starts_with('#') { continue; }
            let mut fields = line.split(',');
            let stage = match fields.next().expect("stage") {
                "background" => 0,
                "weak" => 1,
                "approach" => 2,
                "moving" => 3,
                "gap25" => 4,
                "gap30" => 5,
                "gap40" => 6,
                stage => panic!("unexpected measured stage {stage}"),
            };
            let board: usize = fields.next().expect("board ID").parse().unwrap();
            assert!(board < 3 && !present[stage][board]);
            present[stage][board] = true;
            let fields: Vec<f32> = fields.map(|v| v.parse().expect("raw microtesla")).collect();
            assert_eq!(fields.len(), 48);
            for sensor in 0..16 {
                recorded[stage][board][sensor] = [
                    fields[3*sensor], fields[3*sensor+1], fields[3*sensor+2],
                ];
            }
        }
        assert!(present.iter().flatten().all(|&ready| ready));

        let mut live = RigidLiveFit::default();
        live.set_profile(RigidProfile::a1_reference().unwrap());
        live.set_presence(BoardPresence { board_mask: 0b111, sensor_masks: [0xffff; 3] });

        for stage in 0..7 {
            // A weak far-field measurement can be accepted while XYZ lies on
            // optimizer bounds. Force that *observed* failure condition before
            // the recorded moving-approach frame to make the test deterministic.
            if stage == 3 {
                live.result = Some(RigidFitResult {
                    pose: [30.0, 30.0, 65.0, 0.0, 0.0, 0.0],
                    rmse_mt: 0.05,
                    sensor_count: 48,
                });
            }
            let mut job = None;
            // Provide two nearby real-field board cycles per sampled stage so
            // the app's normal three-board temporal interpolation is exercised.
            // Timestamps here are replay ticks; magnetic XYZ triples are from
            // the actual second demo's captured complete board frames.
            for cycle in 0..2 {
                for board in 0..3 {
                    let mut frame = BoardFrame::default();
                    frame.board_id = board as u16;
                    frame.frame_id = (2 * stage + cycle) as u32;
                    frame.sensor_mask = 0xffff;
                    frame.base_time_us = 1_000_000 * (stage as u64 + 1)
                        + 30_000 * cycle as u64 + 3_000 * board as u64;
                    for (sensor, sample) in frame.samples.iter_mut().enumerate() {
                        let value = recorded[stage][board][sensor];
                        sample.bx_ut = value[0];
                        sample.by_ut = value[1];
                        sample.bz_ut = value[2];
                        sample.time_offset_us = 175 * sensor as u16;
                    }
                    if let Some(next) = live.receive_frame(&frame, true) {
                        assert!(job.is_none(), "more than one in-flight fit");
                        job = Some(next);
                    }
                }
            }
            if stage == 0 {
                assert!(job.is_none(), "no fit should run without background");
                live.capture_a1_background().expect("recorded background");
                continue;
            }
            let job = job.unwrap_or_else(|| panic!("no interpolated fit at stage {stage}"));
            assert!(!job.cold_start || stage == 1);
            if stage == 3 {
                assert_eq!(job.seed[..3], [30.0, 30.0, 65.0]);
                let warm_only = optimize_pose(&job.profile, &job.observations, job.seed)
                    .expect("finite trapped warm-start fit");
                assert!(
                    warm_only.rmse_mt > job.profile.max_rmse_mt,
                    "regression fixture no longer triggers trapped-seed failure: {:.3} mT",
                    warm_only.rmse_mt,
                );
            }
            let completion = job.run();
            let fit = completion.result.as_ref().expect("optimizer returned pose");
            assert!(
                fit.rmse_mt < 0.15,
                "stage {stage} was rejected: {:.4} mT", fit.rmse_mt,
            );
            live.apply_completion(completion);
            assert_eq!(live.summary().rejected, 0, "live rejection at stage {stage}");
            if stage == 4 {
                live.reset_zero();
                assert!(live.summary().zero_ready);
            }
        }
        assert_eq!(live.summary().completed, 6);
        assert_eq!(live.summary().rejected, 0);
    }

    #[test]
    fn rodrigues_is_orthogonal_and_rotates_axes() {
        let r = rodrigues([0.0,0.0,PI/2.0]);
        let x = mat_vec(r,[1.0,0.0,0.0]);
        assert!((x[0]).abs()<1e-10 && (x[1]-1.0).abs()<1e-10);
        let ortho = mul_matrix(transpose(r),r);
        for i in 0..3 { for j in 0..3 {
            assert!((ortho[i][j] - if i==j {1.0} else {0.0}).abs()<1e-10);
        }}
    }

    #[test]
    fn synthetic_joint_pose_recovery() {
        let profile = RigidProfile::a1_reference().unwrap();
        let truth = [1.5,-1.3,32.0,0.03,-0.025,0.04];
        let rotation = rodrigues([truth[3],truth[4],truth[5]]);
        let sources = profile.magnets.iter().map(|m| {
            let center = add(add(profile.origin_mm,[truth[0],truth[1],truth[2]]),
                             mat_vec(rotation,sub(m.center_mm,profile.origin_mm)));
            quadrature(m,center,mat_vec(rotation,m.axis))
        }).collect::<Vec<_>>();
        let observations = profile.boards.iter().flat_map(|b|
            b.sensors_mm.iter().map(|&sensor_mm| Observation {
                sensor_mm, measured_mt: predict(sensor_mm,&sources)
            })
        ).collect::<Vec<_>>();
        let fit = optimize_pose(&profile,&observations,[0.0,0.0,30.0,0.0,0.0,0.0]).unwrap();
        for i in 0..3 {assert!((fit.pose[i]-truth[i]).abs()<0.05);}
        for i in 3..6 {assert!((fit.pose[i]-truth[i]).abs()<0.004);}
        assert!(fit.rmse_mt<1e-4);
    }

    #[test]
    fn choosing_a1_demo_loads_bundled_profile_and_requires_background() {
        let mut live = RigidLiveFit::default();
        live.select_location_preset(MagnetLocationPreset::A1Demo);
        let status = live.summary();
        assert_eq!(status.location_preset, Some(MagnetLocationPreset::A1Demo));
        assert!(status.enabled);
        assert!(status.profile_name.as_deref().is_some_and(|n| n.starts_with("A1 ")));
        assert!(!status.a1_background_ready);
        assert!(!status.zero_ready);
        assert!(status.status.contains("Capture A1 background"));
        // Reselecting the A1 preset keeps its full fitting profile ready.
        live.select_location_preset(MagnetLocationPreset::A1Demo);
        assert!(live.enabled());
        // An A1 fit must not be reused with Gamma Knife coordinates.
        live.select_location_preset(MagnetLocationPreset::RotationStage);
        assert!(live.profile.is_none());
        assert!(!live.enabled());
        live.select_location_preset(MagnetLocationPreset::A1Demo);
        assert!(live.enabled());
    }

    #[test]
    fn stage_selection_never_reuses_incompatible_calibration() {
        let mut live = RigidLiveFit::default();
        live.set_profile(RigidProfile::a1_reference().unwrap());
        assert!(live.enabled());
        live.select_location_preset(MagnetLocationPreset::LinearStageCt);
        assert!(!live.enabled());
        assert!(live.profile.is_none());
        // A location-only stage selection cannot enable fitting without
        // importing a matching full calibration profile.
        let mut ct_like = RigidProfile::a1_reference().unwrap();
        ct_like.coordinate_frame = Some("gk_may15_ct".to_owned());
        for (magnet, center) in ct_like.magnets.iter_mut()
            .zip(MagnetLocationPreset::LinearStageCt.centers_mm()) {
            magnet.center_mm = center;
        }
        live.set_profile(ct_like);
        assert_eq!(live.summary().location_preset, Some(MagnetLocationPreset::LinearStageCt));
        assert!(live.enabled());
        live.select_location_preset(MagnetLocationPreset::RotationStage);
        assert!(!live.enabled());
        assert!(live.profile.is_none());
    }

    #[test]
    fn cylinder_quadrature_matches_analytic_on_axis() {
        let magnet = RigidMagnet {
            center_mm: [0.0,0.0,0.0], axis: [0.0,0.0,1.0],
            diameter_mm: 7.9375, thickness_mm: 6.35, remanence_mt: 1480.0,
        };
        let z: f64 = 30.0;
        let r = magnet.diameter_mm * 0.5;
        let half = magnet.thickness_mm * 0.5;
        let expected = magnet.remanence_mt*0.5 * (
            (z+half) / ((z+half).powi(2)+r*r).sqrt()
            - (z-half) / ((z-half).powi(2)+r*r).sqrt()
        );
        let actual = predict([0.0,0.0,z], &[quadrature(&magnet,magnet.center_mm,magnet.axis)]);
        assert!(((actual[2]-expected)/expected).abs()<0.005);
        assert!(actual[0].abs()<1e-8 && actual[1].abs()<1e-8);
    }

    #[test]
    fn synchronized_frames_create_one_job_and_reset_invalidates_it() {
        let mut live = RigidLiveFit::default();
        live.set_profile(RigidProfile::a1_reference().unwrap());
        live.set_presence(BoardPresence {
            board_mask: 0b111, sensor_masks: [0xffff;3],
        });
        let mut job = None;
        for cycle in 0..4 {
            for board_id in 0..3 {
                let mut frame = BoardFrame::default();
                frame.board_id = board_id;
                frame.frame_id = cycle;
                frame.sensor_mask = 0xffff;
                frame.base_time_us = 100_000+100_000*cycle as u64+board_id as u64;
                for sample in &mut frame.samples {sample.bz_ut = 300.0;}
                job = live.receive_frame(&frame, true).or(job);
            }
            if cycle == 1 {
                assert!(job.is_none(), "A1 must not fit without a background capture");
                live.capture_a1_background().unwrap();
            }
        }
        let job = job.expect("Three bracketed board streams must provide a joint fit job");
        assert_eq!(job.observations.len(),48);
        assert!(live.summary().busy);
        live.reset_stream();
        live.apply_completion(RigidFitCompletion {
            generation: job.generation, time_us:job.time_us,
            result:Some(RigidFitResult {pose: [0.0;6],rmse_mt:0.0,sensor_count:48}),
            elapsed_ms: 1.0,
        });
        assert!(live.summary().result.is_none());
    }

    #[test]
    fn accepted_rigid_poses_produce_signed_xyz_and_zero_reset() {
        let mut live = RigidLiveFit::default();
        live.set_profile(RigidProfile::a1_reference().unwrap());
        let generation = live.generation;
        for (time_us, dx, dy) in [(1_000_000, 0.0, 0.0), (2_000_000, 5.0, -5.0)] {
            live.apply_completion(RigidFitCompletion {
                generation, time_us,
                result: Some(RigidFitResult {
                    pose: [dx, dy, 30.0, 0.0, 0.0, 0.0],
                    rmse_mt: 0.0, sensor_count: 48,
                }),
                elapsed_ms: 1.0,
            });
        }
        let history = live.summary().xyz_history;
        assert_eq!(history.len(), 2);
        assert_eq!(history[1].dx_mm, 5.0);
        assert_eq!(history[1].dy_mm, -5.0);
        assert_eq!(history[1].dz_mm, 0.0);
        assert_eq!(history[1].time_s, 1.0);
        live.reset_zero();
        assert!(live.summary().xyz_history.is_empty());
    }

    #[test]
    fn independent_mode_keeps_raw_background_frames_without_running_rigid_solver() {
        let mut live = RigidLiveFit::default();
        live.set_profile(RigidProfile::a1_reference().unwrap());
        live.set_presence(BoardPresence {
            board_mask: 0b111,
            sensor_masks: [0xffff; 3],
        });
        for cycle in 0..2 {
            for board_id in 0..3 {
                let mut frame = BoardFrame::default();
                frame.board_id = board_id;
                frame.frame_id = cycle;
                frame.sensor_mask = 0xffff;
                frame.base_time_us = 100_000 + 100_000 * cycle as u64 + board_id as u64;
                for (index, sample) in frame.samples.iter_mut().enumerate() {
                    sample.bz_ut = (board_id as f32 * 100.0) + index as f32;
                }
                assert!(live.receive_frame(&frame, false).is_none());
            }
        }
        assert_eq!(live.summary().completed, 0);
        let raw = live.capture_a1_background().expect("capture works without rigid fitting");
        assert_eq!(raw[1][7][2], 0.107); // raw 107 microtesla -> 0.107 mT
        assert!(live.summary().a1_background_ready);
    }

    #[test]
    fn interpolation_refuses_unbracketed_samples() {
        let mut q = VecDeque::new();
        q.push_back(BoardSnapshot {time_us:100,fields_mt:[Some([0.0;3]);16]});
        q.push_back(BoardSnapshot {time_us:200,fields_mt:[Some([2.0;3]);16]});
        assert_eq!(interpolate(&q,150).unwrap()[0],Some([1.0;3]));
        assert!(interpolate(&q,250).is_none());
    }
}
