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
    pub hdmm_history: Vec<DisplacementPoint>,
    pub hdmm_current_mm: Option<f64>,
    pub hdmm_point_ready: bool,
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
    /// Previous accepted pose is only a seed, not a new registration.
    reacquisition_seed: Option<[f64;6]>,
    /// Manually CT-confirmed HDMM reflective sticker position in LPS mm.
    hdmm_target_world: Option<[f64;3]>,
    hdmm_history: VecDeque<DisplacementPoint>,
    displacement: VecDeque<DisplacementPoint>,
    xyz: VecDeque<RigidXyzPoint>,
    rotation: VecDeque<DisplacementPoint>,
    status: String,
}

impl RigidLiveFit {
    /// Begin a fresh Gamma Knife CT session. Previous profiles and fit results
    /// must not remain active when the geometry or magnetic field changes.
    pub fn invalidate_for_gk_calibration(&mut self) {
        self.profile = None;
        self.hdmm_target_world=None;
        self.location_preset = None;
        self.enabled = false;
        self.background_raw = [None; 3];
        self.reset_stream();
        self.status = "GK calibration incomplete; tracking disabled".into();
    }

    pub fn set_profile(&mut self, profile: RigidProfile) {
        // Switching to an actual calibrated profile also selects the matching
        // stage reference, if any; never allow stale location references to
        // override measured board/sensor geometry.
        self.location_preset = MagnetLocationPreset::identify_profile(&profile);
        self.profile = Some(profile);
        self.hdmm_target_world = None;
        self.background_raw = [None;3];
        self.enabled = true;
        self.reset_stream();
    }

    /// Activate an explicitly reviewed experimental GK profile using the
    /// recorded per-sensor no-magnet background in raw Hall axes. No double
    /// subtraction: the assembled CT profile has zero background_mt.
    pub fn set_experimental_gk_profile(
        &mut self, profile: RigidProfile, background_raw: Option<[[[f64;3];16];3]>
    ) {
        self.set_profile(profile);
        self.background_raw = background_raw.map_or([None;3], |raw| raw.map(Some));
        self.status = if self.background_raw.iter().all(Option::is_some) {
            "EXPERIMENTAL GK six-DOF tracking with measured sensor background; validate against HDMM".into()
        } else {
            "EXPERIMENTAL GK six-DOF with fitted board-constant offsets only; per-sensor backgrounds UNMEASURED".into()
        };
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
        self.reacquisition_seed=None;
        self.result = None;
        self.hdmm_history.clear();
        self.displacement.clear();
        self.xyz.clear();
        self.rotation.clear();
        self.completed = 0;
        self.rejected = 0;
        self.last_elapsed_ms = None;
    }

    /// Restart with an unchanged fixed-board CT calibration. Keep only a
    /// previous absolute fit as a warm-start seed; the first accepted fit
    /// establishes the NEW relative zero and history.
    pub fn start_new_acquisition(&mut self) {
        let old=self.result.as_ref().map(|r|r.pose);
        self.reset_stream();
        if self.enabled {
            self.reacquisition_seed=old;
            self.status=if old.is_some() {
                "New stream: reusing previous absolute fit as optimizer seed; new zero follows first accepted fit (requires unchanged receiver mounting)".into()
            } else {
                "New stream: waiting for first accepted fit to establish zero".into()
            };
        }
    }

    /// Stop acquisition without discarding the completed measurement.
    ///
    /// Keeping the last accepted pose, zero reference, and displacement/XYZ/
    /// rotation histories lets the user inspect plots after Stop Field Stream,
    /// just as they can with the independent fits. Still invalidate in-flight
    /// optimization and drop buffered frames so no stale completion can
    /// modify the frozen graphs after acquisition stops.
    pub fn stop_stream(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        self.frames.iter_mut().for_each(VecDeque::clear);
        self.busy = false;
        self.status = "Field stream stopped; last rigid fit and plots retained".into();
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
            self.hdmm_history.clear();
            self.status = "Tracking; rigid zero set to current pose".into();
        } else {
            self.status = "Cannot set rigid zero yet: wait for an accepted fit".into();
        }
    }

    /// The user must independently identify the reflective sticker in CT.
    /// A detected glass BB cannot be silently equated with the HDMM marker.
    pub fn set_hdmm_target_lps(&mut self, xyz:[f64;3]) -> Result<(),String> {
        if xyz.iter().any(|v|!v.is_finite() || v.abs()>100_000.0) {
            return Err("HDMM CT LPS coordinates must be finite millimetres".into());
        }
        if !self.profile.as_ref().is_some_and(|p|
            p.coordinate_frame.as_deref()==Some("DICOM_LPS_mm")) {
            return Err("HDMM projection requires enabled CT-based GK profile".into());
        }
        self.hdmm_target_world=Some(xyz);
        self.hdmm_history.clear();
        self.status="HDMM CT point selected: projected motion is derived from rigid fit, NOT an actual HDMM measurement".into();
        Ok(())
    }

    pub fn clear_hdmm_target(&mut self) {
        self.hdmm_target_world=None;
        self.hdmm_history.clear();
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
            hdmm_history: self.hdmm_history.iter().cloned().collect(),
            hdmm_current_mm: self.hdmm_history.back().map(|p|p.displacement_mm),
            hdmm_point_ready: self.hdmm_target_world.is_some(),
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
            seed: self.result.as_ref().map(|r|r.pose)
                .or(self.reacquisition_seed).unwrap_or(profile.initial_pose),
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
        self.reacquisition_seed=None;
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
        if let Some(point)=self.hdmm_target_world {
            let now=rigid_point_at_pose(profile,result.pose,point);
            let initial=rigid_point_at_pose(profile,zero,point);
            let delta=sub(now,initial);
            self.hdmm_history.push_back(DisplacementPoint {
                time_s,displacement_mm:dot(delta,delta).sqrt(),
            });
            while self.hdmm_history.len()>HISTORY_LEN {self.hdmm_history.pop_front();}
        }
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

fn rigid_point_at_pose(profile:&RigidProfile,pose:[f64;6],point:[f64;3]) -> [f64;3] {
    add(add(profile.origin_mm,[pose[0],pose[1],pose[2]]),
        mat_vec(rodrigues([pose[3],pose[4],pose[5]]),
                sub(point,profile.origin_mm)))
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


/// Fit three signed finite-cylinder strengths when CT chip registration is verified.
/// Caller must supply firmware-ordered chip positions and known raw Hall axes.
pub fn fit_magnet_strengths(
    profile: &RigidProfile,
    background: &[[[f64; 3]; 16]; 3],
    stationary: &[[[f64; 3]; 16]; 3],
) -> Result<([f64; 3], f64), String> {
    fit_magnet_strengths_masked(profile,background,stationary,[0xffff;3])
}

/// Research endpoint screening: masked channels NEVER enter calibration.
/// Default production/legacy callers still use all sensors via the wrapper.
pub fn fit_magnet_strengths_masked(
    profile: &RigidProfile,
    background: &[[[f64; 3]; 16]; 3],
    stationary: &[[[f64; 3]; 16]; 3],
    masks: [u16;3],
) -> Result<([f64; 3], f64), String> {
    profile.validate()?;
    if masks.iter().any(|m|m.count_ones()<8) {
        return Err("Fewer than 8 calibration sensors per board".into());
    }
    let sources: Vec<_> = profile.magnets.iter().map(|magnet| {
        let mut m = magnet.clone();
        m.remanence_mt = 1.0;
        vec![quadrature(&m, m.center_mm, m.axis)]
    }).collect();
    let mut a = [[0.0_f64; 4]; 3];
    let mut samples = Vec::new();
    for board in &profile.boards {
        let id = board.raw_board_id as usize;
        for i in 0..16 {
            if masks[id] & (1u16<<i) == 0 { continue; }
            let gain = board.sensor_gains.as_ref().map(|g| g[i]).unwrap_or(1.0);
            let observed = sub(
                scale(mat_vec(board.raw_to_world,
                    sub(stationary[id][i], background[id][i])), 1.0/gain),
                board.background_mt);
            let fields = [
                predict(board.sensors_mm[i], &sources[0]),
                predict(board.sensors_mm[i], &sources[1]),
                predict(board.sensors_mm[i], &sources[2]),
            ];
            for axis in 0..3 {
                let row = [fields[0][axis], fields[1][axis], fields[2][axis]];
                if !observed[axis].is_finite() || row.iter().any(|x| !x.is_finite()) {
                    return Err("Nonfinite calibration data".into());
                }
                for j in 0..3 {
                    for k in 0..3 { a[j][k] += row[j]*row[k]; }
                    a[j][3] += row[j]*observed[axis];
                }
                samples.push((row, observed[axis]));
            }
        }
    }
    let max_diag = (0..3).map(|i|a[i][i]).fold(0.0_f64,f64::max);
    for k in 0..3 {
        let pivot = (k..3).max_by(|&i,&j| a[i][k].abs().total_cmp(&a[j][k].abs())).unwrap();
        a.swap(k,pivot);
        if a[k][k].abs() < max_diag*1e-8 {
            return Err("Unobservable or ill-conditioned magnet strengths".into());
        }
        let scale = a[k][k];
        for j in k..4 {a[k][j]/=scale;}
        for i in 0..3 { if i!=k {
            let factor=a[i][k];
            for j in k..4 {a[i][j]-=factor*a[k][j];}
        }}
    }
    let strengths=[a[0][3],a[1][3],a[2][3]];
    let mse=samples.iter().map(|(row,y)| {
        let e=row.iter().zip(strengths).map(|(x,m)|x*m).sum::<f64>()-y;
        e*e
    }).sum::<f64>()/samples.len() as f64;
    if !mse.is_finite() {return Err("Nonfinite calibration residual".into());}
    Ok((strengths,mse.sqrt()))
}

/// Background-free experimental alternative. Estimate the three signed
/// magnet strengths and one constant nuisance field vector per BOARD
/// (9 more coefficients) from the spatial variations across 16 chips.
/// This does NOT recover per-sensor backgrounds or calibrate Hall gains.
pub fn fit_strengths_with_board_offsets(
    profile: &RigidProfile,
    stationary: &[[[f64;3];16];3],
) -> Result<([f64;3], [[f64;3];3], f64), String> {
    fit_strengths_with_board_offsets_masked(profile,stationary,[0xffff;3])
}

/// Same board-constant offset model as baseline, but only valid selected
/// sensor channels contribute to EACH board mean and design row.
pub fn fit_strengths_with_board_offsets_masked(
    profile: &RigidProfile,
    stationary: &[[[f64;3];16];3],
    masks: [u16;3],
) -> Result<([f64;3], [[f64;3];3], f64), String> {
    profile.validate()?;
    if masks.iter().any(|m|m.count_ones()<8) {
        return Err("Fewer than 8 calibration sensors per board".into());
    }
    let sources: Vec<_>=profile.magnets.iter().map(|m|{
        let mut unit=m.clone();
        unit.remanence_mt=1.0;
        quadrature(&unit,unit.center_mm,unit.axis)
    }).collect();
    let mut design=Vec::<([f64;3],f64)>::with_capacity(144);
    let mut model_board_means=[[[0.0;3];3];3]; // board, magnet, xyz
    let mut observed_means=[[0.0;3];3]; // board, xyz
    let mut board_values=Vec::with_capacity(3);
    for board in &profile.boards {
        let id=board.raw_board_id as usize;
        let n= f64::from(masks[id].count_ones());
        let mut predictions=[[[0.0_f64;3];3];16];
        let mut measured=[[0.0_f64;3];16];
        for i in 0..16 {
            if masks[id] & (1u16<<i) == 0 { continue; }
            let gain=board.sensor_gains.as_ref().map(|g|g[i]).unwrap_or(1.0);
            measured[i]=scale(mat_vec(board.raw_to_world,stationary[id][i]),1.0/gain);
            if measured[i].iter().any(|v|!v.is_finite()) {
                return Err("Nonfinite stationary Hall fields".into());
            }
            for j in 0..3 {
                predictions[i][j]=predict(board.sensors_mm[i],std::slice::from_ref(&sources[j]));
            }
            for axis in 0..3 {
                observed_means[id][axis]+=measured[i][axis]/n;
                for j in 0..3 {
                    model_board_means[id][j][axis]+=predictions[i][j][axis]/n;
                }
            }
        }
        board_values.push((id,predictions,measured));
    }
    let mut matrix=[[0.0_f64;4];3];
    for (id,predictions,measured) in &board_values {
        for i in 0..16 {
            if masks[*id] & (1u16<<i) == 0 { continue; }
            for axis in 0..3 {
                let row=std::array::from_fn::<_,3,_>(|j|
                    predictions[i][j][axis]-model_board_means[*id][j][axis]);
                let y=measured[i][axis]-observed_means[*id][axis];
                for j in 0..3 {
                    for k in 0..3 {matrix[j][k]+=row[j]*row[k];}
                    matrix[j][3]+=row[j]*y;
                }
                design.push((row,y));
            }
        }
    }
    let max_diag=(0..3).map(|i|matrix[i][i]).fold(0.0_f64,f64::max);
    if !max_diag.is_finite() || max_diag <= 0.0 {
        return Err("No spatial sensitivity to magnet strengths".into());
    }
    for k in 0..3 {
        let pivot=(k..3).max_by(|&i,&j|
            matrix[i][k].abs().total_cmp(&matrix[j][k].abs())).unwrap();
        matrix.swap(k,pivot);
        if !matrix[k][k].is_finite() || matrix[k][k].abs()<max_diag*1e-8 {
            return Err("Cannot distinguish three magnet strengths from spatial Hall gradients without background".into());
        }
        let scale=matrix[k][k];
        for col in k..4 {matrix[k][col]/=scale;}
        for row in 0..3 {
            if row==k {continue;}
            let factor=matrix[row][k];
            for col in k..4 {matrix[row][col]-=factor*matrix[k][col];}
        }
    }
    let strengths=[matrix[0][3],matrix[1][3],matrix[2][3]];
    let mut biases=[[0.0_f64;3];3];
    for board in &profile.boards {
        let id=board.raw_board_id as usize;
        for axis in 0..3 {
            biases[id][axis]=observed_means[id][axis]-
                (0..3).map(|j|model_board_means[id][j][axis]*strengths[j]).sum::<f64>();
        }
    }
    let mse=design.iter().map(|(row,y)|{
        let diff=row.iter().zip(strengths).map(|(a,s)|a*s).sum::<f64>()-y;
        diff*diff
    }).sum::<f64>()/design.len() as f64;
    let rmse=mse.sqrt();
    if !rmse.is_finite() || biases.iter().flatten().any(|v|!v.is_finite()) {
        return Err("Background-free fit produced nonfinite offsets or residuals".into());
    }
    Ok((strengths,biases,rmse))
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
    fn signed_strength_recovery_from_synthetic_stationary_fields() {
        let mut profile = RigidProfile::a1_reference().unwrap();
        let truth = [1250.0, -1370.0, 1410.0];
        for (i, m) in profile.magnets.iter_mut().enumerate() {
            m.remanence_mt = truth[i];
        }
        let mut background = [[[0.0;3];16];3];
        let mut measured = background;
        // predict expects one quadrature-source Vec per magnet, not an
        // extra Vec wrapper (the production strength fitter wraps each
        // magnet separately because it calls predict once per magnet).
        let src: Vec<_> = profile.magnets.iter()
            .map(|m| quadrature(m, m.center_mm, m.axis))
            .collect();
        for board in &mut profile.boards {
            board.background_mt = [0.0;3];
            board.sensor_gains = None;
            for (i, point) in board.sensors_mm.iter().enumerate() {
                let world = predict(*point, &src);
                let rotation = board.raw_to_world;
                let raw = [
                    rotation[0][0]*world[0]+rotation[1][0]*world[1]+rotation[2][0]*world[2],
                    rotation[0][1]*world[0]+rotation[1][1]*world[1]+rotation[2][1]*world[2],
                    rotation[0][2]*world[0]+rotation[1][2]*world[1]+rotation[2][2]*world[2],
                ];
                measured[board.raw_board_id as usize][i] = raw;
            }
        }
        let (estimated, rmse) = fit_magnet_strengths(&profile,&background,&measured).unwrap();
        for j in 0..3 {
            assert!((estimated[j]-truth[j]).abs()<1e-4);
        }
        assert!(rmse<1e-8, "calibration RMSE {rmse}");
        background[0][0][0]=f64::NAN;
        assert!(fit_magnet_strengths(&profile,&background,&measured).is_err());
    }

    /// Reconstruct CT registration and fit strengths using ONLY the GK app's
    /// own exported calibration-input JSON. No manually authored profile.
    /// This remains an experimental screening measurement, not HDMM validation.
    #[test]
    fn experimental_gk_automatic_ct_replay_from_export() {
        let Ok(path)=std::env::var("GK_CALIBRATION_INPUTS") else { return; };
        let input:serde_json::Value=serde_json::from_str(
            &std::fs::read_to_string(&path).expect("Read exported GK calibration JSON"))
            .expect("Parse calibration JSON");
        assert_eq!(input["format"],"gk_experiment_calibration_inputs_v1");
        let detector=input["detector_replay"].to_string();
        let geometry=crate::cbct_geometry::CbctGeometry::from_detector_json(
            &detector, "exported_gk_scan".into(), "exported_gk_qc".into())
            .expect("Reconstruct CT geometry from exported candidate");
        let approved_cad=input["cad_mapping_approved_in_session"].as_bool()==Some(true)
            && input["cad_mapping_review_preview"].as_str()
                .is_some_and(|s|s.contains("HUMAN REVIEW REQUIRED"));
        let registration=if approved_cad {
            geometry.register_ct_boards_v2_with_reviewed_cad()
        } else {
            geometry.register_ct_boards_v2()
        }.expect("Reviewed CT-to-v2 firmware indices/JST geometry");
        for board in &registration {
            println!("{} raw{}: CT grid RMS {:.3} mm, JST proxy {:.3} mm",
                board.detection_name, board.raw_board_id,
                board.geometric_rms_mm, board.jst_proxy_offset_mm);
        }
        let profile=if approved_cad {
            geometry.experimental_candidate_profile_with_reviewed_cad()
        } else {
            geometry.experimental_candidate_profile()
        }.expect("Build geometry candidate from explicitly reviewed CT registration");
        let background:[[[f64;3];16];3]=serde_json::from_value(
            input["background_raw_mt"].clone()).expect("48 background vectors");
        let stationary:[[[f64;3];16];3]=serde_json::from_value(
            input["stationary_raw_mt"].clone()).expect("48 stationary vectors");
        let (strengths,rmse)=fit_magnet_strengths(&profile,&background,&stationary)
            .expect("Signed magnet-strength calibration from CT and Hall fields");
        println!("EXPERIMENTAL automatic GK CT calibration, not HDMM-validated:");
        println!("  CT mapping source: {}",
            if approved_cad {"EXPLICITLY REVIEWED KiCad/JST PRIOR (NOT CT-detected connectors)"}
            else {"CT-detected PCB centroid/JST proxy"});
        println!("  signed strengths [mT]: {strengths:?}; stationary RMSE [mT]: {rmse:.6}");
        println!("  preliminary QC: {}, expected |M| 300..2200 mT and RMSE <= 0.50 mT",
            if strengths.iter().all(|x|x.abs()>=300.0 && x.abs()<=2200.0) && rmse<=0.5
                {"PASS (experimental only)"} else {"REVIEW REQUIRED"});
        assert!(rmse.is_finite());
    }

    /// Optional real-data diagnostic; never runs unless both paths are supplied.
    /// The geometry profile must have independently registered CT chip indices
    /// and raw Hall axes. These measurements are for analysis, NOT activation.
    #[test]
    fn experimental_gk_stationary_calibration_from_export() {
        let Ok(inputs_path) = std::env::var("GK_CALIBRATION_INPUTS") else { return; };
        let profile_path = std::env::var("GK_REGISTERED_GEOMETRY")
            .expect("Set GK_REGISTERED_GEOMETRY to a reviewed, firmware-indexed RigidProfile JSON");
        let inputs: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(&inputs_path).expect("Read GK inputs"))
            .expect("Parse GK inputs");
        assert_eq!(inputs["format"], "gk_experiment_calibration_inputs_v1");
        assert_eq!(inputs["sensor_fields_order"], "raw_firmware_index_0_to_15");
        let background: [[[f64; 3]; 16]; 3] = serde_json::from_value(
            inputs["background_raw_mt"].clone()).expect("48 background vectors");
        let reference: [[[f64; 3]; 16]; 3] = serde_json::from_value(
            inputs["stationary_raw_mt"].clone()).expect("48 reference vectors");
        let profile = RigidProfile::from_json(
            &std::fs::read_to_string(&profile_path).expect("Read registered CT geometry"))
            .expect("Validated registered CT geometry");
        let (strengths, rmse) = fit_magnet_strengths(&profile, &background, &reference)
            .expect("Experimental signed-strength fit");
        println!("EXPERIMENTAL GK STATIONARY CALIBRATION (NOT A VALIDATED TRACKING PROFILE)");
        println!("signed strengths [mT]: {strengths:?}; field RMSE [mT]: {rmse:.6}");
        assert!(strengths.iter().all(|x|x.is_finite()) && rmse.is_finite());
    }

    /// The complete October30 GK recorded Hall NPZ->JSON replay, using exactly
    /// this app's RigidLiveFit receive_frame -> RigidFitJob::run ->
    /// apply_completion path. All private sensor frames and CT coordinates
    /// remain on the user's machine; only scalar results are saved.
    ///
    /// Opt-in env:
    /// GK_OCT30_REPLAY_IN, GK_OCT30_PRIMARY_PRE, GK_OCT30_SUMMARY_OUT.
    /// GK_OCT30_HYBRID_PRE is optional: registration failure is reported and
    /// does not block the original KiCad-based control.
    #[test]
    fn experimental_oct30_recorded_gk_motion_time_series_replay() {
        use serde::Deserialize;
        use std::path::Path;
        #[derive(Deserialize)]
        struct Frame {
            board_id:u16, frame_id:u32, base_time_us:u64, sensor_mask:u16,
            offsets_us:[u16;16], fields_ut:[[f64;3];16],
        }
        #[derive(Deserialize)]
        struct ChannelMask {board_id:u16, sensor_indices:Vec<usize>, valid_mask:u16}
        #[derive(Deserialize)]
        struct Truth {
            rotvec_rad:[f64;3], centroid_translation_lps_mm:[f64;3],
            angle_deg:f64, centroid_displacement_mm:f64,
        }
        #[derive(Deserialize)]
        struct Recorded {
            format:String,frame_count:usize,frame_counts_by_board:[usize;3],
            calibration_stationary_raw_mt:[[[f64;3];16];3],
            #[serde(default)]
            calibration_background_raw_mt:Option<[[[f64;3];16];3]>,
            excluded_channels:Vec<ChannelMask>,
            ct_reference:Truth, frames:Vec<Frame>,
        }
        let Ok(path)=std::env::var("GK_OCT30_REPLAY_IN") else {return};
        let pre=std::env::var("GK_OCT30_PRIMARY_PRE")
            .expect("GK_OCT30_PRIMARY_PRE must be original same-scan optimized+KiCad CT JSON");
        let out=std::env::var("GK_OCT30_SUMMARY_OUT")
            .expect("GK_OCT30_SUMMARY_OUT must be a new local path outside Git");
        assert!(!Path::new(&out).exists(),"Refusing overwrite of existing GK analysis report");
        let data:Recorded=serde_json::from_str(&std::fs::read_to_string(&path)
            .expect("Read PRIVATE recorded three-board Hall frames"))
            .expect("Parse private GK recorded frames");
        assert_eq!(data.format,"gk_oct30_private_hall_replay_frames_v1");
        assert_eq!(data.frame_counts_by_board,[1382;3]);
        assert_eq!(data.frame_count,4146);
        assert_eq!(data.frames.len(),4146);
        assert!(data.ct_reference.angle_deg>0.1 && data.ct_reference.angle_deg<15.0);
        assert!(data.ct_reference.centroid_displacement_mm>0.05
                && data.ct_reference.centroid_displacement_mm<20.0);
        let mut masks=[0_u16;3];
        assert_eq!(data.excluded_channels.len(),3);
        for (b,entry) in data.excluded_channels.iter().enumerate() {
            assert_eq!(entry.board_id as usize,b);
            assert!(entry.valid_mask.count_ones()>=8);
            for &i in &entry.sensor_indices {
                assert!(i<16 && entry.valid_mask&(1<<i)==0);
            }
            masks[b]=entry.valid_mask;
        }
        let ct_input=std::fs::read_to_string(&pre).expect("Read primary pre CT detector JSON");
        let geom=crate::cbct_geometry::CbctGeometry::from_detector_json(
            &ct_input,"oct30_pre".into(),"oct30_qc".into())
            .expect("Primary CT import");
        let baseline_mapping=geom.register_ct_boards_v2_with_reviewed_cad()
            .expect("Original KiCad CT-to-firmware registration");
        let baseline=geom.experimental_candidate_profile_with_reviewed_cad()
            .expect("Primary KiCad-reviewed experimental profile");

        let stride=std::env::var("GK_OCT30_FIT_STRIDE").ok()
            .map(|v|v.parse::<u32>().expect("GK_OCT30_FIT_STRIDE integer"))
            .unwrap_or(10);
        assert!((1..=100).contains(&stride),"Fit stride must be 1..100");

        // Report comparison of physical transforms; a matching angle magnitude
        // alone is NOT an accurate 6-DOF motion fit.
        fn pose_delta(first:[f64;6],last:[f64;6],truth:&Truth)
            -> serde_json::Value
        {
            let trans=[
                last[0]-first[0],last[1]-first[1],last[2]-first[2],
            ];
            let initial=rodrigues([first[3],first[4],first[5]]);
            let final_r=rodrigues([last[3],last[4],last[5]]);
            let relative=mul_matrix(final_r,transpose(initial));
            let tr=relative[0][0]+relative[1][1]+relative[2][2];
            let angle=(((tr-1.0)/2.0).clamp(-1.0,1.0)).acos()*RAD_TO_DEG;
            let ct_r=rodrigues(truth.rotvec_rad);
            let mismatch=mul_matrix(relative,transpose(ct_r));
            let mismatch_angle=(((mismatch[0][0]+mismatch[1][1]+mismatch[2][2]-1.0)/2.0)
                .clamp(-1.0,1.0)).acos()*RAD_TO_DEG;
            let shift_norm=dot(trans,trans).sqrt();
            let vector_err=sub(trans,truth.centroid_translation_lps_mm);
            serde_json::json!({
                "translation_lps_mm":trans,
                "translation_magnitude_mm":shift_norm,
                "translation_vector_error_vs_CT_mm":dot(vector_err,vector_err).sqrt(),
                "rotation_angle_deg":angle,
                "rotation_geodesic_error_vs_CT_deg":mismatch_angle,
                "rotation_angle_magnitude_error_vs_CT_deg":
                    (angle-truth.angle_deg).abs(),
                "translation_magnitude_error_vs_CT_mm":
                    (shift_norm-truth.centroid_displacement_mm).abs(),
                "rotation_reference_origin":"PRE magnet centroid; initial fitted pose is subtracted",
            })
        }
        fn med_pose(p:&[[f64;6]]) -> [f64;6] {
            std::array::from_fn(|i|{
                let mut v=p.iter().map(|x|x[i]).collect::<Vec<_>>();
                v.sort_by(f64::total_cmp);
                if v.len()%2==0 {(v[v.len()/2-1]+v[v.len()/2])*0.5}
                else {v[v.len()/2]}
            })
        }
        fn one_arm(
            name:&str,mut profile:RigidProfile,data:&Recorded,masks:[u16;3],
            stride:u32,trajectory:&mut Vec<serde_json::Value>
        )->serde_json::Value {
            // Reuse actual original no-magnet background if provided.
            // Otherwise match the app's explicit lower-confidence
            // per-board-constant nuisance-field calibration.
            let with_background=data.calibration_background_raw_mt.is_some();
            let fitted=if let Some(bg)=data.calibration_background_raw_mt.as_ref() {
                fit_magnet_strengths(
                    &profile,bg,&data.calibration_stationary_raw_mt)
                    .map(|(s,r)|(s,[[0.0;3];3],r))
            } else {
                fit_strengths_with_board_offsets(
                    &profile,&data.calibration_stationary_raw_mt)
            };
            let (strengths,offsets,calibration_rmse)=match fitted {
                Ok(v)=>v,
                Err(e)=>return serde_json::json!({
                    "arm":name,"status":"stationary_calibration_failed",
                    "background_measured":with_background,
                    "reason":e}),
            };
            for (m,s) in profile.magnets.iter_mut().zip(strengths) {
                m.remanence_mt=s;
            }
            for b in &mut profile.boards {
                b.background_mt=offsets[b.raw_board_id as usize];
            }
            let plausible=strengths.iter().all(|s|s.abs()>=300.0 && s.abs()<=2200.0);
            let acceptable=plausible && calibration_rmse<=profile.max_rmse_mt;
            let max_rmse=profile.max_rmse_mt;
            let mut fit=RigidLiveFit::default();
            fit.set_experimental_gk_profile(
                profile,data.calibration_background_raw_mt);
            fit.set_presence(BoardPresence {board_mask:0b111,sensor_masks:masks});
            let mut accepted=Vec::<(f64,[f64;6],f64,usize)>::new();
            let mut attempts=0;
            let mut fail_solver=0;
            for source in &data.frames {
                let mut frame=BoardFrame::default();
                frame.board_id=source.board_id;
                frame.frame_id=source.frame_id;
                frame.base_time_us=source.base_time_us;
                frame.sensor_mask=source.sensor_mask;
                for i in 0..16 {
                    frame.samples[i].bx_ut=source.fields_ut[i][0] as f32;
                    frame.samples[i].by_ut=source.fields_ut[i][1] as f32;
                    frame.samples[i].bz_ut=source.fields_ut[i][2] as f32;
                    frame.samples[i].time_offset_us=source.offsets_us[i];
                }
                // Ingest EVERY frame, but dispatch finite-cylinder fits at
                // a fixed, documented stride. This is offline subsampling,
                // NOT an assertion about achieved live frame rate.
                let Some(job)=fit.receive_frame(
                    &frame,source.frame_id%stride==0) else {continue};
                attempts+=1;
                let completed=job.run();
                let when=completed.time_us as f64/1e6;
                let elapsed=completed.elapsed_ms;
                let solved=completed.result.clone();
                let old_rejected=fit.summary().rejected;
                if solved.is_none(){fail_solver+=1;}
                fit.apply_completion(completed);
                let summary=fit.summary();
                let ok=summary.rejected==old_rejected;
                let (pose,rmse,count)=solved.map_or((None,None,None),
                    |v|(Some(v.pose),Some(v.rmse_mt),Some(v.sensor_count)));
                if ok {
                    if let (Some(p),Some(r),Some(n))=(pose,rmse,count) {
                        accepted.push((when,p,r,n));
                    }
                }
                trajectory.push(serde_json::json!({
                    "arm":name,"t_s":when,"accepted":ok,
                    "pose_mm_rad":pose,"rmse_mt":rmse,"sensor_count":count,
                    "solver_ms":elapsed,
                }));
            }
            let duration=data.frames.iter().map(|f|f.base_time_us)
                .max().unwrap_or(0) as f64/1e6;
            let pre=accepted.iter().filter(|(t,_,_,_)|
                *t>=1.0 && *t<=4.5).map(|(_,p,_,_)|*p).collect::<Vec<_>>();
            let post=accepted.iter().filter(|(t,_,_,_)|
                *t>=duration-8.0).map(|(_,p,_,_)|*p).collect::<Vec<_>>();
            let comparison=if pre.len()>=3 && post.len()>=3 {
                pose_delta(med_pose(&pre),med_pose(&post),&data.ct_reference)
            } else {
                serde_json::json!({"status":"insufficient_accepted_pre_or_post_plateau_fits",
                    "pre_fit_count":pre.len(),"post_fit_count":post.len()})
            };
            let rms=accepted.iter().map(|(_,_,r,_)|*r).collect::<Vec<_>>();
            let mean_rmse=if rms.is_empty(){None}else {
                Some(rms.iter().sum::<f64>()/rms.len() as f64)};
            serde_json::json!({
                "arm":name,
                "status":if acceptable {"research_replay_calibration_QC_pass"}
                    else {"research_replay_calibration_QC_FAILED_not_activated"},
                "calibration_source":if with_background {
                    "app fit_magnet_strengths with original measured raw sensor background"
                } else {
                    "app fit_strengths_with_board_offsets; no measured raw sensor background"
                },
                "measured_background_used":with_background,
                "signed_remanences_mt":strengths,
                "stationary_rmse_mt":calibration_rmse,
                "calibration_gate_max_rmse_mt":max_rmse,
                "calibration_strengths_plausible":plausible,
                "calibration_QC_pass":acceptable,
                "live_fitting_activated":false,
                "fit_stride_per_board_frame":stride,
                "all_raw_frames_ingested":true,
                "fit_attempts":attempts,
                "solver_failed_count":fail_solver,
                "accepted_fits":accepted.len(),
                "rejected_fits":fit.summary().rejected,
                "mean_accepted_pose_rmse_mt":mean_rmse,
                "pre_accepted_count":pre.len(),"post_accepted_count":post.len(),
                "rigid_motion_vs_CT":comparison
            })
        }

        let mut trajectory=Vec::new();
        let mut results=vec![one_arm(
            "A_original_CT_KiCad_prior",baseline,&data,masks,stride,&mut trajectory)];
        if let Ok(optional)=std::env::var("GK_OCT30_HYBRID_PRE") {
            let candidate=std::fs::read_to_string(&optional)
                .expect("Read hybrid PCB-only pre geometry");
            let check=crate::cbct_geometry::CbctGeometry::from_detector_json(
                &candidate,"oct30_pre".into(),"oct30_qc".into())
                .expect("Parse hybrid PCB-only geometry");
            // Registered CT measurements and the Hall chips are held fixed.
            assert_eq!(geom.magnets.len(),check.magnets.len());
            for (a,b) in geom.magnets.iter().zip(&check.magnets) {
                assert_eq!(a.center_lps_mm,b.center_lps_mm);
                assert_eq!(a.unsigned_axis_lps,b.unsigned_axis_lps);
            }
            for a in &geom.arrays {
                let b=check.arrays.iter().find(|b|b.name==a.name)
                    .expect("Hybrid missing board");
                assert_eq!(a.detected_chip_positions_lps_mm,b.detected_chip_positions_lps_mm);
                assert_eq!(a.chip_face_normal_lps,b.chip_face_normal_lps);
            }
            match check.register_ct_boards_v2() {
                Ok(registration) if registration.iter().all(|b| {
                    baseline_mapping.iter().any(|a|
                        a.raw_board_id==b.raw_board_id &&
                        a.firmware_to_detector==b.firmware_to_detector)
                })=>{
                    match check.experimental_candidate_profile() {
                        Ok(profile)=>results.push(one_arm(
                            "B_hybrid_superior_PCB_only",profile,
                            &data,masks,stride,&mut trajectory)),
                        Err(error)=>results.push(serde_json::json!({
                            "arm":"B_hybrid_superior_PCB_only",
                            "status":"hybrid_profile_construction_failed",
                            "error":error
                        })),
                    }
                }
                Ok(_)=>results.push(serde_json::json!({
                    "arm":"B_hybrid_superior_PCB_only",
                    "status":"firmware_sensor_correspondence_changed_REJECTED",
                    "reason":"Do not compare changed channel indices against baseline A",
                })),
                Err(error)=>results.push(serde_json::json!({
                    "arm":"B_hybrid_superior_PCB_only",
                    "status":"CT_to_firmware_registration_failed_do_NOT_relax_gate",
                    "error":error,
                })),
            }
        }
        let summary=serde_json::json!({
            "format":"gk_oct30_rust_recorded_motion_replay_summary_v1",
            "status":"RESEARCH_ONLY_NOT_VALIDATED_LIVE_GK_TRACKING",
            "source":"original 1382/board Oct30 Hall recording + independently registered CT pre/post",
            "ct_reference":{
                "rotation_vector_rad":data.ct_reference.rotvec_rad,
                "angle_deg":data.ct_reference.angle_deg,
                "magnet_centroid_translation_lps_mm":data.ct_reference.centroid_translation_lps_mm,
                "magnet_centroid_displacement_mm":data.ct_reference.centroid_displacement_mm,
            },
            "frame_count":data.frame_count,
            "sensor_exclusions":data.excluded_channels.iter()
                .map(|e|serde_json::json!({
                    "board_id":e.board_id,"sensor_indices":e.sensor_indices,
                    "valid_mask":e.valid_mask})).collect::<Vec<_>>(),
            "arms":results,
            "warnings":[
                "The movement NPZ has no no-magnet background; a separate matched original MLXBF002 background may be supplied, otherwise board-offset calibration is lower confidence.",
                "Magnet polarity and signed Hall-axis mapping remain experimental.",
                "Sensor gain/wrap state unavailable; predeclared channels are excluded, never unwrapped.",
                "Pre/post CT reference is one movement only. No intermediate CT ground truth.",
                "Finite-cylinder Rust model differs from earlier Python point-dipole differential fit.",
                "All recorded frames are ingested, but finite-cylinder solve is attempted only at fixed stride.",
                "Calibration or tracking acceptance must not be inferred from research replay alone."
            ]
        });
        let output=std::path::PathBuf::from(&out);
        if let Some(dir)=output.parent(){std::fs::create_dir_all(dir)
            .expect("Create local GK summary directory");}
        std::fs::write(&output,serde_json::to_string_pretty(&summary).unwrap()+"\n")
            .expect("Write sanitized scalar GK motion summary");
        if let Ok(traj_path)=std::env::var("GK_OCT30_TRAJECTORY_OUT") {
            assert!(!Path::new(&traj_path).exists(),"Refuse overwrite of trajectory file");
            let file=std::path::PathBuf::from(traj_path);
            if let Some(dir)=file.parent(){std::fs::create_dir_all(dir).unwrap();}
            std::fs::write(file,serde_json::to_string_pretty(&trajectory).unwrap()+"\n")
                .expect("Write private time-resolved pose trajectory");
        }
        println!("GK Oct30 recorded motion -> {}\n{}",
            out,serde_json::to_string_pretty(&summary).unwrap());
    }

    /// Nov06 strictly offline stationary-pre / stationary-post experiment.
    ///
    /// Reuses exactly the magnetic finite-cylinder solver and fixed strength
    /// calibration used in live fitting; intentionally receives NO middle
    /// movement frames, because Nov06 board0 raw components wrap ~65–73s.
    /// No live-tracking settings are changed; no result is clinical accuracy.
    #[test]
    fn experimental_gk_nov06_endpoint_only_finite_cylinder_replay() {
        use serde::Deserialize;
        use std::path::Path;
        #[derive(Deserialize)]
        struct Frame {
            board_id:u16, frame_id:u32, base_time_us:u64, sensor_mask:u16,
            offsets_us:[u16;16], fields_ut:[[f64;3];16],
        }
        #[derive(Deserialize)]
        struct Mask {board_id:u16, valid_mask:u16, sensor_indices:Vec<usize>}
        #[derive(Deserialize)]
        struct Truth {
            rotvec_rad:[f64;3], centroid_translation_lps_mm:[f64;3],
            angle_deg:f64, centroid_displacement_mm:f64,
        }
        #[derive(Deserialize)]
        struct BbReference {
            bb_pre_lps_mm:[f64;3],bb_delta_pre_lps_mm:[f64;3],
            bb_displacement_mm:f64, manual_review_completed:bool,
        }
        #[derive(Deserialize)]
        struct Data {
            format:String,status:String,frame_count:usize,
            frame_counts_by_board:[usize;3],
            source_frame_counts_by_board:[usize;3],
            time_duration_s:f64,
            stationary_window_s:[f64;2],
            final_window_last_s:f64,
            calibration_stationary_raw_mt:[[[f64;3];16];3],
            calibration_background_raw_mt:Option<[[[f64;3];16];3]>,
            endpoint_masks:Vec<Mask>,
            ct_reference:Truth,ct_bb_reference:BbReference,
            frames:Vec<Frame>,
        }
        let Ok(source)=std::env::var("GK_NOV06_ENDPOINT_REPLAY_IN") else {return};
        let pre_path=std::env::var("GK_NOV06_PRE_CT")
            .expect("GK_NOV06_PRE_CT must name the SAME Nov06 A detector JSON");
        let summary_path=std::env::var("GK_NOV06_ENDPOINT_SUMMARY_OUT")
            .expect("GK_NOV06_ENDPOINT_SUMMARY_OUT must be a NEW local private analysis report");
        assert!(!Path::new(&summary_path).exists(),"Refusing to overwrite Nov06 endpoint report");
        let data:Data=serde_json::from_str(
            &std::fs::read_to_string(&source).expect("Read PRIVATE Nov06 plateau replay")
        ).expect("Parse Nov06 plateau replay");
        assert_eq!(data.format,"gk_nov06_private_endpoint_replay_v1");
        assert_eq!(data.status,"RESEARCH_ONLY_PRELIMINARY_ENDPOINT_FIT");
        assert_eq!(data.frames.len(),data.frame_count);
        assert_eq!(data.frame_counts_by_board.iter().sum::<usize>(),data.frame_count);
        assert_eq!(data.endpoint_masks.len(),3);
        assert_eq!(data.stationary_window_s,[1.0,4.5],
            "Do not tune pre plateau to the measured CT endpoint");
        assert_eq!(data.final_window_last_s,8.0,
            "Do not change final plateau to optimize agreement");
        assert!(data.time_duration_s>20.0 && data.time_duration_s<3600.0);
        assert!(data.source_frame_counts_by_board.iter().all(|n|*n>100));
        assert!(!data.ct_bb_reference.manual_review_completed,
            "Source CT is detector-derived: never claim automatic visual approval");
        assert!((0.05..20.0).contains(&data.ct_reference.centroid_displacement_mm));
        assert!((0.05..15.0).contains(&data.ct_reference.angle_deg));
        let mut masks=[0_u16;3];
        for (idx,m) in data.endpoint_masks.iter().enumerate() {
            assert_eq!(m.board_id as usize,idx);
            assert!(m.valid_mask.count_ones()>=8);
            for &sensor in &m.sensor_indices {
                assert!(sensor<16 && m.valid_mask&(1<<sensor)==0);
            }
            masks[idx]=m.valid_mask;
        }
        for frame in &data.frames {
            let t=frame.base_time_us as f64/1e6;
            assert!(t<=data.stationary_window_s[1]+0.1 ||
                    t>=data.time_duration_s-data.final_window_last_s-0.1,
                "Mid-motion (possibly aliased) frames must NEVER enter the endpoint solver");
            assert_eq!(frame.sensor_mask,0xffff);
        }
        let geom=crate::cbct_geometry::CbctGeometry::from_detector_json(
            &std::fs::read_to_string(&pre_path).expect("Read Nov06 A detector JSON"),
            "GK Nov06 A".into(),"pre-CT review".into()
        ).expect("Parse Nov06 source CT geometry (not approved)");
        // Prefer the same physically constrained board orientation/JST prior
        // used for the successful Oct30 experimental A arm. If its provenance
        // or CT-to-firmware checks fail, report that blockage, not a fallback
        // to a differently mapped geometry.
        let profile=geom.experimental_candidate_profile_with_reviewed_cad();
        let mut summary=serde_json::json!({
            "format":"gk_nov06_finite_cylinder_endpoint_preliminary_v1",
            "status":"RESEARCH_ONLY_NOT_VALIDATED_LIVE_GK_TRACKING",
            "experiment":"GK_20261106",
            "window_source":"PRE 1.0..4.5 s and POST final 8 s only; middle frames omitted",
            "source_frame_count":data.source_frame_counts_by_board,
            "endpoint_frame_count":data.frame_counts_by_board,
            "source_detector_approved":false,
            "physical_post_ct_matches_final_plateau":false,
            "hdmm_clock_and_zero_verified":false,
            "background_source":if data.calibration_background_raw_mt.is_some() {
                "EXPLICITLY_OPERATOR_CONFIRMED_same_session_no_magnet"
            } else {"UNMEASURED_board_constant_offsets_only"},
            "global_wrap_exclusions_deliberately_not_reused":true,
            "endpoint_sensor_masks":masks,
            "ct_reference":{
                "head_rotation_deg":data.ct_reference.angle_deg,
                "magnet_centroid_displacement_mm":data.ct_reference.centroid_displacement_mm,
                "bb_displacement_mm":data.ct_bb_reference.bb_displacement_mm
            },
            "warnings":[
                "Not a full motion trajectory. Board0 sensor wrapping was observed in excluded middle frames.",
                "A stable post-wrap alias may evade local plateau jump screening.",
                "Detector-derived CT and superior KiCad/JST geometry are NOT independently reviewed.",
                "Final Hall plateau is not yet independently matched to CBCT1106B acquisition.",
                "Not a validated HDMM comparison or authorized live tracker.",
                "CT POST and BB coordinates are used only for evaluation, NEVER magnetic optimization."
            ]
        });
        match profile {
            Err(reason)=>{
                summary["fit_status"]=serde_json::json!("CT_to_firmware_KiCad_registration_blocked");
                summary["reason"]=serde_json::json!(reason);
            }
            Ok(mut profile)=>{
                let calibrate=if let Some(bg)=data.calibration_background_raw_mt.as_ref(){
                    fit_magnet_strengths(
                        &profile,bg,&data.calibration_stationary_raw_mt)
                        .map(|(rem,rmse)|(rem,[[0.0;3];3],rmse))
                } else {
                    fit_strengths_with_board_offsets(
                        &profile,&data.calibration_stationary_raw_mt)
                };
                match calibrate {
                    Err(reason)=>{
                        summary["fit_status"]=serde_json::json!("stationary_calibration_failed");
                        summary["reason"]=serde_json::json!(reason);
                    }
                    Ok((strengths,offsets,calibration_rmse))=>{
                        let plausible=strengths.iter()
                            .all(|v|v.abs()>=300.0 && v.abs()<=2200.0);
                        let gate=calibration_rmse<=profile.max_rmse_mt && plausible;
                        summary["signed_remanences_mt"]=serde_json::json!(strengths);
                        summary["stationary_calibration_rmse_mt"]=serde_json::json!(calibration_rmse);
                        summary["stationary_calibration_QC_pass"]=serde_json::json!(gate);
                        if !gate {
                            summary["fit_status"]=serde_json::json!(
                                "calibration_QC_FAILED_pose_fit_not_run");
                        } else {
                            for (m,value) in profile.magnets.iter_mut().zip(strengths) {
                                m.remanence_mt=value;
                            }
                            for b in &mut profile.boards {
                                b.background_mt=offsets[b.raw_board_id as usize];
                            }
                            let origin=profile.origin_mm;
                            let mut live=RigidLiveFit::default();
                            live.set_experimental_gk_profile(
                                profile,data.calibration_background_raw_mt);
                            live.set_presence(BoardPresence {
                                board_mask:0b111,sensor_masks:masks
                            });
                            let mut pre=Vec::<[f64;6]>::new();
                            let mut post=Vec::<[f64;6]>::new();
                            let mut errors=Vec::<f64>::new();
                            let mut failed=0_usize;
                            let mut attempts=0_usize;
                            for src in &data.frames {
                                let mut frame=BoardFrame::default();
                                frame.board_id=src.board_id;
                                frame.frame_id=src.frame_id;
                                frame.base_time_us=src.base_time_us;
                                frame.sensor_mask=src.sensor_mask;
                                for i in 0..16 {
                                    frame.samples[i].bx_ut=src.fields_ut[i][0] as f32;
                                    frame.samples[i].by_ut=src.fields_ut[i][1] as f32;
                                    frame.samples[i].bz_ut=src.fields_ut[i][2] as f32;
                                    frame.samples[i].time_offset_us=src.offsets_us[i];
                                }
                                // Fixed 10-frame stride, based on original
                                // frame IDs, cannot be tuned against CT.
                                let Some(job)=live.receive_frame(
                                    &frame,src.frame_id%10==0) else {continue};
                                attempts+=1;
                                let completed=job.run();
                                let pose=completed.result.as_ref().map(|x|x.pose);
                                let rmse=completed.result.as_ref().map(|x|x.rmse_mt);
                                let was_rejected=live.summary().rejected;
                                let time=completed.time_us as f64/1e6;
                                live.apply_completion(completed);
                                if live.summary().rejected!=was_rejected {
                                    failed+=1;continue;
                                }
                                if let (Some(p),Some(e))=(pose,rmse){
                                    errors.push(e);
                                    if time<=data.stationary_window_s[1]+0.1 {
                                        pre.push(p)
                                    } else if time>=data.time_duration_s-
                                        data.final_window_last_s-0.1 {
                                        post.push(p)
                                    }
                                }
                            }
                            summary["fit_attempts"]=serde_json::json!(attempts);
                            summary["fit_rejected"]=serde_json::json!(failed);
                            summary["pre_accepted"]=serde_json::json!(pre.len());
                            summary["post_accepted"]=serde_json::json!(post.len());
                            summary["mean_pose_field_rmse_mt"]=serde_json::json!(
                                if errors.is_empty(){None}else{
                                    Some(errors.iter().sum::<f64>()/errors.len() as f64)
                                });
                            if pre.len()<3 || post.len()<3 {
                                summary["fit_status"]=serde_json::json!(
                                    "insufficient_accepted_stationary_endpoint_poses");
                            } else {
                                fn median(v:&[[f64;6]])->[f64;6]{
                                    std::array::from_fn(|i|{
                                        let mut x=v.iter().map(|p|p[i]).collect::<Vec<_>>();
                                        x.sort_by(f64::total_cmp);
                                        let n=x.len();
                                        if n%2==0 {(x[n/2-1]+x[n/2])*0.5}else{x[n/2]}
                                    })
                                }
                                let a=median(&pre);
                                let b=median(&post);
                                let shift=sub([b[0],b[1],b[2]],[a[0],a[1],a[2]]);
                                let rot_a=rodrigues([a[3],a[4],a[5]]);
                                let rot_b=rodrigues([b[3],b[4],b[5]]);
                                let rel=mul_matrix(rot_b,transpose(rot_a));
                                let ct_rot=rodrigues(data.ct_reference.rotvec_rad);
                                let d_rot=mul_matrix(rel,transpose(ct_rot));
                                let angle=(((rel[0][0]+rel[1][1]+rel[2][2]-1.0)/2.0)
                                    .clamp(-1.0,1.0)).acos()*RAD_TO_DEG;
                                let rot_err=(((d_rot[0][0]+d_rot[1][1]+d_rot[2][2]-1.0)/2.0)
                                    .clamp(-1.0,1.0)).acos()*RAD_TO_DEG;
                                let trans_err=sub(shift,data.ct_reference.centroid_translation_lps_mm);
                                // Same physical BB in the PRE CT receiver frame.
                                let marker=data.ct_bb_reference.bb_pre_lps_mm;
                                let bb_a=add(add(origin,[a[0],a[1],a[2]]),
                                    mat_vec(rot_a,sub(marker,origin)));
                                let bb_b=add(add(origin,[b[0],b[1],b[2]]),
                                    mat_vec(rot_b,sub(marker,origin)));
                                let bb_delta=sub(bb_b,bb_a);
                                let bb_err=sub(bb_delta,data.ct_bb_reference.bb_delta_pre_lps_mm);
                                summary["fit_status"]=serde_json::json!(
                                    "computed_preliminary_unreviewed_CT_and_stationary_windows");
                                summary["finite_cylinder_pose"] = serde_json::json!({
                                    "head_rotation_deg":angle,
                                    "rotation_geodesic_error_vs_CT_deg":rot_err,
                                    "magnet_centroid_translation_lps_mm":shift,
                                    "magnet_centroid_translation_vector_error_vs_CT_mm":
                                        dot(trans_err,trans_err).sqrt(),
                                    "bb_delta_lps_mm":bb_delta,
                                    "bb_displacement_mm":dot(bb_delta,bb_delta).sqrt(),
                                    "bb_vector_error_vs_CT_mm":dot(bb_err,bb_err).sqrt(),
                                    "bb_displacement_magnitude_error_vs_CT_mm":
                                        (dot(bb_delta,bb_delta).sqrt()-
                                        data.ct_bb_reference.bb_displacement_mm).abs()
                                });
                            }
                        }
                    }
                }
            }
        }
        let target=Path::new(&summary_path);
        if let Some(parent)=target.parent(){
            std::fs::create_dir_all(parent).expect("Create private summary folder");
        }
        std::fs::write(target,serde_json::to_string_pretty(&summary).unwrap()+"\n")
            .expect("Write Nov06 scalar research summary");
        println!("NOV06 ENDPOINT RESEARCH REPLAY: {}",summary_path);
        println!("{}",serde_json::to_string_pretty(&summary).unwrap());
    }

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
                assert_eq!(&job.seed[..3], &[30.0, 30.0, 65.0]);
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
    fn stopping_stream_preserves_rigid_plots_but_invalidates_pending_fit() {
        let mut live = RigidLiveFit::default();
        live.set_profile(RigidProfile::a1_reference().unwrap());
        let generation = live.generation;
        live.apply_completion(RigidFitCompletion {
            generation, time_us: 1_000_000,
            result: Some(RigidFitResult {
                pose: [0.0, 0.0, 25.0, 0.0, 0.0, 0.0],
                rmse_mt: 0.03, sensor_count: 48,
            }),
            elapsed_ms: 5.0,
        });
        live.reset_zero();
        live.apply_completion(RigidFitCompletion {
            generation, time_us: 2_000_000,
            result: Some(RigidFitResult {
                pose: [5.0, -3.0, 25.0, 0.0, 0.0, 0.0],
                rmse_mt: 0.03, sensor_count: 48,
            }),
            elapsed_ms: 5.0,
        });
        let before = live.summary();
        assert!(before.zero_ready);
        assert_eq!(before.xyz_history.len(), 1);
        assert_eq!(before.rotation_history.len(), 1);
        assert_eq!(before.displacement_history.len(), 1);

        live.stop_stream();
        let stopped = live.summary();
        assert!(stopped.status.contains("stopped"));
        assert!(stopped.zero_ready);
        assert_eq!(stopped.completed, before.completed);
        assert_eq!(stopped.rejected, before.rejected);
        assert_eq!(stopped.xyz_history.len(), before.xyz_history.len());
        assert_eq!(stopped.rotation_history.len(), before.rotation_history.len());
        assert_eq!(stopped.displacement_history.len(), before.displacement_history.len());
        assert_eq!(stopped.xyz_history[0].dx_mm, 5.0);
        assert_eq!(stopped.xyz_history[0].dy_mm, -3.0);
        assert_eq!(stopped.result.unwrap().pose, [5.0, -3.0, 25.0, 0.0, 0.0, 0.0]);
        assert!(live.frames.iter().all(VecDeque::is_empty));
        assert!(!live.busy);

        // Any solver task launched before Stop must not change the frozen plot.
        live.apply_completion(RigidFitCompletion {
            generation, time_us: 3_000_000,
            result: Some(RigidFitResult {
                pose: [15.0, 0.0, 25.0, 0.0, 0.0, 0.0],
                rmse_mt: 0.03, sensor_count: 48,
            }),
            elapsed_ms: 5.0,
        });
        assert_eq!(live.summary().xyz_history.len(), 1);
        assert_eq!(live.summary().completed, before.completed);
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
    fn stream_restarts_from_last_pose_but_establishes_fresh_zero() {
        let mut live=RigidLiveFit::default();
        live.set_profile(RigidProfile::a1_reference().unwrap());
        let old_generation=live.generation;
        let previous=[4.0,1.0,30.0,0.01,0.0,0.0];
        live.apply_completion(RigidFitCompletion {
            generation:old_generation, time_us:1_000_000,
            result:Some(RigidFitResult {pose:previous,rmse_mt:0.02,sensor_count:48}),
            elapsed_ms:1.0,
        });
        assert!(live.summary().zero_ready);
        live.stop_stream();
        live.start_new_acquisition();
        assert_eq!(live.reacquisition_seed,Some(previous));
        assert!(live.summary().result.is_none());
        assert!(!live.summary().zero_ready);
        assert!(live.summary().displacement_history.is_empty());
        assert!(live.summary().xyz_history.is_empty());
        assert!(live.summary().rotation_history.is_empty());
        // Results from before restart are never accepted.
        live.apply_completion(RigidFitCompletion {
            generation:old_generation,time_us:2_000_000,
            result:Some(RigidFitResult {pose:previous,rmse_mt:0.0,sensor_count:48}),
            elapsed_ms:1.0,
        });
        assert!(live.summary().result.is_none());
        let new_pose=[8.0,2.0,31.0,0.0,0.0,0.0];
        live.apply_completion(RigidFitCompletion {
            generation:live.generation,time_us:3_000_000,
            result:Some(RigidFitResult {pose:new_pose,rmse_mt:0.0,sensor_count:48}),
            elapsed_ms:1.0,
        });
        assert_eq!(live.summary().translation_from_zero,Some([0.0;3]));
        assert!(live.reacquisition_seed.is_none());
        live.invalidate_for_gk_calibration();
        assert!(live.reacquisition_seed.is_none());
        assert!(!live.summary().zero_ready);
    }

    #[test]
    fn predicted_hdmm_marker_includes_rigid_rotation_and_translation() {
        let mut profile=RigidProfile::a1_reference().unwrap();
        profile.coordinate_frame=Some("DICOM_LPS_mm".into());
        profile.origin_mm=[0.0;3];
        let marker=[100.0,0.0,0.0];
        let a=rigid_point_at_pose(&profile,[0.0;6],marker);
        let b=rigid_point_at_pose(
            &profile,[0.0,0.0,0.0,0.0,0.0,PI/2.0],marker);
        assert!((dot(sub(b,a),sub(b,a)).sqrt()-100.0*2.0_f64.sqrt()).abs()<1e-8);
        let mut fit=RigidLiveFit::default();
        fit.set_profile(profile);
        fit.set_hdmm_target_lps(marker).unwrap();
        let generation=fit.generation;
        for (time_us,rotation) in [(1_000_000,0.0),(2_000_000,PI/2.0)] {
            fit.apply_completion(RigidFitCompletion {
                generation,time_us,
                result:Some(RigidFitResult {
                    pose:[0.0,0.0,0.0,0.0,0.0,rotation],rmse_mt:0.0,sensor_count:48
                }),elapsed_ms:1.0
            });
        }
        assert!((fit.summary().hdmm_current_mm.unwrap()-100.0*2.0_f64.sqrt()).abs()<1e-8);
        fit.reset_zero();
        assert!(fit.summary().hdmm_history.is_empty());
        fit.invalidate_for_gk_calibration();
        assert!(!fit.summary().hdmm_point_ready);
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
