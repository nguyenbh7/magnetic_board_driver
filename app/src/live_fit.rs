use std::collections::{BTreeMap, VecDeque};
use std::fmt;

use data_transfer::rpc::{BoardPresence, SensorField};

const MAX_HISTORY_POINTS: usize = 600;
const BACKGROUND_CAPTURE_FRAMES: usize = 20;
const UT_PER_MT: f64 = 1000.0;
const MOMENT_NORM_PRIOR_WEIGHT_MT: f64 = 0.25;

#[derive(Debug, Clone, Default)]
pub struct BoardLiveFits {
    boards: BTreeMap<u16, BoardLiveFitState>,
    presence: BoardPresence,
}

#[derive(Debug, Clone)]
pub struct BoardLiveFitState {
    current_frame: BoardFrameAccumulator,
    result: Option<FitResult>,
    calibrated_moment_norm: Option<f64>,

    background: Option<BTreeMap<u8, [f64; 3]>>,
    background_capture: Option<BackgroundCapture>,

    origin: Option<(f64, f64, f64)>,
    start_time_us: Option<u64>,
    last_fit_time_us: Option<u64>,
    displacement_history: VecDeque<DisplacementPoint>,

    magnet_preset: MagnetPreset,
    magnet_effective_scale: f64,
    use_known_magnet_prior: bool,
}

impl Default for BoardLiveFitState {
    fn default() -> Self {
        Self {
            current_frame: BoardFrameAccumulator::default(),
            result: None,
            calibrated_moment_norm: None,

            background: None,
            background_capture: None,

            origin: None,
            start_time_us: None,
            last_fit_time_us: None,
            displacement_history: VecDeque::new(),

            magnet_preset: MagnetPreset::ThickD54N52,
            magnet_effective_scale: 1.0,
            use_known_magnet_prior: true,
        }
    }
}

#[derive(Debug, Clone, Default)]
struct BoardFrameAccumulator {
    fields: BTreeMap<u8, SensorField>,
    frame_id: Option<u32>,
    frame_start_time_us: Option<u64>,
    frame_end_time_us: Option<u64>,
}

#[derive(Debug, Clone, Default)]
struct BackgroundCapture {
    frames_collected: usize,
    sums: BTreeMap<u8, [f64; 3]>,
}

impl BackgroundCapture {
    fn add_complete_frame(&mut self, samples: &[Sample]) -> Option<BTreeMap<u8, [f64; 3]>> {
        if samples.is_empty() {
            return None;
        }

        if self.frames_collected > 0 {
            let same_sensor_set = samples.len() == self.sums.len()
                && samples
                    .iter()
                    .all(|sample| self.sums.contains_key(&sample.sensor_index));

            if !same_sensor_set {
                return None;
            }
        }

        for sample in samples {
            let sum = self.sums.entry(sample.sensor_index).or_insert([0.0; 3]);
            sum[0] += sample.field[0];
            sum[1] += sample.field[1];
            sum[2] += sample.field[2];
        }

        self.frames_collected += 1;

        if self.frames_collected < BACKGROUND_CAPTURE_FRAMES {
            return None;
        }

        let denominator = self.frames_collected as f64;

        Some(
            std::mem::take(&mut self.sums)
                .into_iter()
                .map(|(sensor_index, sum)| {
                    (
                        sensor_index,
                        [
                            sum[0] / denominator,
                            sum[1] / denominator,
                            sum[2] / denominator,
                        ],
                    )
                })
                .collect(),
        )
    }
}

#[derive(Debug, Clone)]
pub struct FitResult {
    pub position: (f64, f64, f64),
    pub residual_rms: f64,
    pub objective_score: f64,
    pub n_sensors: usize,
    pub moment: (f64, f64, f64),
    pub moment_norm: f64,
}

#[derive(Debug, Clone)]
pub struct DisplacementPoint {
    pub time_s: f64,
    pub displacement_mm: f64,
}

#[derive(Debug, Clone)]
struct CompletedBoardFrame {
    fields: Vec<SensorField>,
    frame_id: u32,
    frame_start_time_us: u64,
    frame_end_time_us: u64,
}

#[derive(Debug, Clone)]
struct Sample {
    sensor_index: u8,
    position: [f64; 3],
    field: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagnetPreset {
    ThinD52N52,
    ThickD54N52,
}

impl MagnetPreset {
    pub fn label(self) -> &'static str {
        match self {
            Self::ThinD52N52 => "Thin D52-N52",
            Self::ThickD54N52 => "Thick D54-N52",
        }
    }

    pub fn diameter_mm(self) -> f64 {
        7.94
    }

    pub fn thickness_mm(self) -> f64 {
        match self {
            Self::ThinD52N52 => 3.17,
            Self::ThickD54N52 => 6.35,
        }
    }

    pub fn br_t(self) -> f64 {
        1.48
    }

    pub fn moment_norm_app(self) -> f64 {
        let radius_m = (self.diameter_mm() * 1.0e-3) / 2.0;
        let thickness_m = self.thickness_mm() * 1.0e-3;
        let volume_m3 = std::f64::consts::PI * radius_m * radius_m * thickness_m;
        let mu0 = 4.0 * std::f64::consts::PI * 1.0e-7;

        let moment_si = self.br_t() * volume_m3 / mu0;

        // The app's dipole matrix uses mm^-3 and field units of mT.
        // B_mT = A_mm * moment_app, so moment_app = 1e5 * moment_si.
        1.0e5 * moment_si
    }
}

impl MagnetPreset {
    pub const ALL: [MagnetPreset; 2] = [
        MagnetPreset::ThinD52N52,
        MagnetPreset::ThickD54N52,
    ];
}

impl fmt::Display for MagnetPreset {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

impl Default for MagnetPreset {
    fn default() -> Self {
        Self::ThickD54N52
    }
}

impl BoardLiveFits {
    pub fn set_presence(&mut self, presence: BoardPresence) {
        self.presence = presence;

        self.boards.retain(|board_id, _| {
            let index = *board_id as usize;
            index < self.presence.sensor_masks.len()
                && self.presence.board_mask & (1u8 << index) != 0
        });

        for board_index in 0..self.presence.sensor_masks.len() {
            if self.presence.board_mask & (1u8 << board_index) != 0 {
                self.boards.entry(board_index as u16).or_default();
            }
        }
    }

    pub fn reset_displacement(&mut self, board_id: u16) {
        if let Some(board) = self.boards.get_mut(&board_id) {
            board.reset_displacement();
        }
    }

    pub fn calibrate_board_magnet(&mut self, board_id: u16) {
        if let Some(board) = self.boards.get_mut(&board_id) {
            board.calibrate_current_magnet();
        }
    }

    pub fn capture_board_background(&mut self, board_id: u16) {
        if let Some(board) = self.boards.get_mut(&board_id) {
            board.start_background_capture();
        }
    }

    pub fn update(&mut self, field: SensorField) {
        // frame_id == 0 is reserved for ad-hoc single reads. Those should
        // never enter the streaming frame accumulator used by the live fitter.
        if field.frame_id == 0 || !self.is_expected_field(&field) {
            return;
        }

        let board_id = field.board_id;
        let expected_sensor_count = self.expected_sensor_count(board_id).unwrap_or(16);

        let board = self.boards.entry(board_id).or_default();

        if let Some(frame) = board.current_frame.update(field, expected_sensor_count) {
            board.update_from_completed_frame(frame);
        }
    }

    pub fn board_summaries(&self) -> Vec<BoardFitSummary> {
        self.boards
            .iter()
            .map(|(board_id, board)| BoardFitSummary {
                board_id: *board_id,
                seen_sensors: board.current_frame.fields.len(),
                expected_sensors: self.expected_sensor_count(*board_id).unwrap_or(16),
                result: board.result.clone(),
                displacement_history: board.displacement_history.iter().cloned().collect(),
                is_calibrated: board.calibrated_moment_norm.is_some(),
                has_background: board.background.is_some(),
                magnet_preset: board.magnet_preset,
                magnet_effective_scale: board.magnet_effective_scale,
                use_known_magnet_prior: board.use_known_magnet_prior,
                target_moment_norm: if board.use_known_magnet_prior {
                    Some(board.magnet_preset.moment_norm_app() * board.magnet_effective_scale)
                } else {
                    board.calibrated_moment_norm
                },
            })
            .collect()
    }

    fn is_expected_field(&self, field: &SensorField) -> bool {
        let board_index = field.board_id as usize;

        if board_index >= self.presence.sensor_masks.len() {
            return false;
        }

        if self.presence.board_mask & (1u8 << board_index) == 0 {
            return false;
        }

        let Some(sensor_index) = address_to_sensor_index(field.address) else {
            return false;
        };

        self.presence.sensor_masks[board_index] & (1u16 << sensor_index) != 0
    }

    fn expected_sensor_count(&self, board_id: u16) -> Option<usize> {
        let board_index = board_id as usize;
        let sensor_mask = *self.presence.sensor_masks.get(board_index)?;

        let count = sensor_mask.count_ones() as usize;

        if count == 0 {
            None
        } else {
            Some(count)
        }
    }

    pub fn set_board_magnet_preset(&mut self, board_id: u16, preset: MagnetPreset) {
        if let Some(board) = self.boards.get_mut(&board_id) {
            board.magnet_preset = preset;
            board.magnet_effective_scale = 1.0;
            board.calibrated_moment_norm = None;
            board.reset_displacement();
        }
    }
}

#[derive(Debug, Clone)]
pub struct BoardFitSummary {
    pub board_id: u16,
    pub seen_sensors: usize,
    pub expected_sensors: usize,
    pub result: Option<FitResult>,
    pub displacement_history: Vec<DisplacementPoint>,
    pub is_calibrated: bool,
    pub has_background: bool,
    pub magnet_preset: MagnetPreset,
    pub magnet_effective_scale: f64,
    pub use_known_magnet_prior: bool,
    pub target_moment_norm: Option<f64>,
}

impl BoardLiveFitState {
    fn update_from_completed_frame(&mut self, frame: CompletedBoardFrame) {
        let frame_sensor_count = frame.fields.len();
        let raw_samples: Vec<_> = frame
            .fields
            .iter()
            .filter_map(sensor_field_to_sample)
            .collect();

        if raw_samples.len() < 6 {
            return;
        }

        let was_capturing_background = self.background_capture.is_some();

        if raw_samples.len() == frame_sensor_count {
            let completed_background = self
                .background_capture
                .as_mut()
                .and_then(|capture| capture.add_complete_frame(&raw_samples));

            if let Some(background) = completed_background {
                self.background = Some(background);
                self.background_capture = None;
                self.clear_fit_after_background_change();
            }
        }

        // Do not fit magnet position while a no-magnet background capture is in progress.
        if was_capturing_background {
            return;
        }

        let samples = if let Some(background) = &self.background {
            subtract_background_samples(&raw_samples, background)
        } else {
            raw_samples
        };

        if samples.len() < 6 {
            return;
        }

        let fit_result = if self.use_known_magnet_prior {
            let target_moment_norm =
                self.magnet_preset.moment_norm_app() * self.magnet_effective_scale;

            fit_dipole_grid_moment_norm_prior(&samples, target_moment_norm)
        } else if let Some(calibrated_moment_norm) = self.calibrated_moment_norm {
            fit_dipole_grid_moment_norm_prior(&samples, calibrated_moment_norm)
        } else {
            fit_dipole_grid(&samples)
        };

        if let Some(result) = fit_result {
            let frame_mid_time_us =
                frame.frame_start_time_us
                    + (frame.frame_end_time_us.saturating_sub(frame.frame_start_time_us) / 2);

            self.last_fit_time_us = Some(frame_mid_time_us);
            self.record_displacement(frame_mid_time_us, &result);
            self.result = Some(result);
        }
    }

    fn record_displacement(&mut self, time_us: u64, result: &FitResult) {
        let position = result.position;

        let origin = *self.origin.get_or_insert(position);
        let start_time_us = *self.start_time_us.get_or_insert(time_us);

        let dx = position.0 - origin.0;
        let dy = position.1 - origin.1;
        let dz = position.2 - origin.2;

        let displacement_mm = (dx * dx + dy * dy + dz * dz).sqrt();
        let time_s = time_us.saturating_sub(start_time_us) as f64 / 1_000_000.0;

        self.displacement_history.push_back(DisplacementPoint {
            time_s,
            displacement_mm,
        });

        while self.displacement_history.len() > MAX_HISTORY_POINTS {
            self.displacement_history.pop_front();
        }
    }

    fn reset_displacement(&mut self) {
        self.displacement_history.clear();

        if let Some(result) = &self.result {
            self.origin = Some(result.position);
            self.start_time_us = self.last_fit_time_us;

            if self.last_fit_time_us.is_some() {
                self.displacement_history.push_back(DisplacementPoint {
                    time_s: 0.0,
                    displacement_mm: 0.0,
                });
            }
        } else {
            self.origin = None;
            self.start_time_us = None;
            self.last_fit_time_us = None;
        }
    }

    fn calibrate_current_magnet(&mut self) {
        let Some(result) = &self.result else {
            return;
        };

        if !result.moment_norm.is_finite() || result.moment_norm <= 1.0e-12 {
            return;
        }

        if self.use_known_magnet_prior {
            let model_moment_norm = self.magnet_preset.moment_norm_app();

            if model_moment_norm.is_finite() && model_moment_norm > 1.0e-12 {
                self.magnet_effective_scale = result.moment_norm / model_moment_norm;
            }

            self.calibrated_moment_norm = None;
        } else {
            self.calibrated_moment_norm = Some(result.moment_norm);
        }

        self.reset_displacement();
    }

    fn start_background_capture(&mut self) {
        self.background = None;
        self.background_capture = Some(BackgroundCapture::default());
        self.clear_fit_after_background_change();
    }

    fn clear_fit_after_background_change(&mut self) {
        // Existing calibration and displacement zero are no longer valid
        // because the field model changed between raw/background-subtracted data.
        self.calibrated_moment_norm = None;
        self.result = None;
        self.origin = None;
        self.start_time_us = None;
        self.last_fit_time_us = None;
        self.displacement_history.clear();
    }
}

impl BoardFrameAccumulator {
    fn update(
        &mut self,
        field: SensorField,
        expected_sensor_count: usize,
    ) -> Option<CompletedBoardFrame> {
        let incoming_frame_id = field.frame_id;

        match self.frame_id {
            Some(current_frame_id) if current_frame_id != incoming_frame_id => {
                // The previous sweep was incomplete. Drop it rather than
                // mixing its samples with the next explicit firmware sweep.
                self.fields.clear();
                self.frame_id = Some(incoming_frame_id);
                self.frame_start_time_us = Some(field.time);
                self.frame_end_time_us = None;
            }
            None => {
                self.frame_id = Some(incoming_frame_id);
                self.frame_start_time_us = Some(field.time);
            }
            _ => {}
        }

        self.frame_end_time_us = Some(field.time);
        self.fields.insert(field.address, field);

        if self.fields.len() < expected_sensor_count {
            return None;
        }

        let fields = std::mem::take(&mut self.fields)
            .into_values()
            .collect::<Vec<_>>();

        let frame_id = self.frame_id.take().unwrap_or(incoming_frame_id);
        let frame_start_time_us = self.frame_start_time_us.take().unwrap_or(0);
        let frame_end_time_us = self.frame_end_time_us.take().unwrap_or(frame_start_time_us);

        Some(CompletedBoardFrame {
            fields,
            frame_id,
            frame_start_time_us,
            frame_end_time_us,
        })
    }
}

fn address_to_sensor_index(address: u8) -> Option<u8> {
    let normalized = address & !0b0100_0000;

    if (0x0C..=0x1B).contains(&normalized) {
        Some(normalized - 0x0C)
    } else {
        None
    }
}

fn sensor_field_to_sample(field: &SensorField) -> Option<Sample> {
    let sensor_index = address_to_sensor_index(field.address)?;

    let bx = field.field.x?.value() / UT_PER_MT;
    let by = field.field.y?.value() / UT_PER_MT;
    let bz = field.field.z?.value() / UT_PER_MT;

    Some(Sample {
        sensor_index,
        position: [
            field.position.0 as f64,
            field.position.1 as f64,
            field.position.2 as f64,
        ],
        field: [bx, by, bz],
    })
}

fn subtract_background_samples(
    samples: &[Sample],
    background: &BTreeMap<u8, [f64; 3]>,
) -> Vec<Sample> {
    samples
        .iter()
        .filter_map(|sample| {
            let background_field = background.get(&sample.sensor_index)?;

            Some(Sample {
                sensor_index: sample.sensor_index,
                position: sample.position,
                field: [
                    sample.field[0] - background_field[0],
                    sample.field[1] - background_field[1],
                    sample.field[2] - background_field[2],
                ],
            })
        })
        .collect()
}

fn sparkline(values: &[f64]) -> String {
    const BARS: [&str; 8] = ["▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"];

    if values.is_empty() {
        return String::new();
    }

    let min = values.iter().copied().fold(f64::INFINITY, f64::min);
    let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    if (max - min).abs() < 1.0e-12 {
        return "▁".repeat(values.len().min(80));
    }

    let stride = (values.len() / 80).max(1);

    values
        .iter()
        .step_by(stride)
        .map(|value| {
            let normalized = ((*value - min) / (max - min)).clamp(0.0, 1.0);
            let index = (normalized * (BARS.len() as f64 - 1.0)).round() as usize;
            BARS[index]
        })
        .collect()
}

fn fit_dipole_grid(samples: &[Sample]) -> Option<FitResult> {
    fit_dipole_grid_with(samples, evaluate_position)
}

fn fit_dipole_grid_moment_norm_prior(
    samples: &[Sample],
    target_moment_norm: f64,
) -> Option<FitResult> {
    fit_dipole_grid_with(samples, |samples, magnet_pos| {
        evaluate_position_moment_norm_prior(samples, magnet_pos, target_moment_norm)
    })
}

fn fit_dipole_grid_with(
    samples: &[Sample],
    mut evaluate: impl FnMut(&[Sample], [f64; 3]) -> Option<FitResult>,
) -> Option<FitResult> {
    let (min_x, max_x, min_y, max_y) = sensor_bounds(samples)?;

    let mut best: Option<(FitResult, [f64; 3])> = None;

    let mut center = [
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        10.0,
    ];

    let passes = [
        // Coarse/global passes.
        (8.0, 2.0, 36.0, false),
        (4.0, 2.0, 28.0, false),
        (2.0, 2.0, 20.0, false),
        (1.0, 2.0, 14.0, false),

        // Fine/local passes to reduce threshold/jump behavior.
        (0.5, 2.0, 14.0, true),
        (0.25, 2.0, 14.0, true),
        (0.10, 2.0, 14.0, true),
    ];

    for (step, z_min, z_max, local_z) in passes {
        let x_min = center[0] - 3.0 * step;
        let x_max = center[0] + 3.0 * step;
        let y_min = center[1] - 3.0 * step;
        let y_max = center[1] + 3.0 * step;

        let mut x = x_min.max(min_x - 10.0);

        while x <= x_max.min(max_x + 10.0) {
            let mut y = y_min.max(min_y - 10.0);

            while y <= y_max.min(max_y + 10.0) {
                let z_start = if local_z {
                    (center[2] - 3.0 * step).max(z_min)
                } else {
                    z_min
                };

                let z_stop = if local_z {
                    (center[2] + 3.0 * step).min(z_max)
                } else {
                    z_max
                };

                let mut z = z_start;

                while z <= z_stop {
                    let candidate_pos = [x, y, z];

                    if let Some(candidate) = evaluate(samples, candidate_pos) {
                        let is_better = best
                            .as_ref()
                            .map(|(best_result, _)| {
                                candidate.objective_score < best_result.objective_score
                            })
                            .unwrap_or(true);

                        if is_better {
                            best = Some((candidate, candidate_pos));
                        }
                    }

                    z += step;
                }

                y += step;
            }

            x += step;
        }

        if let Some((_, best_pos)) = &best {
            center = *best_pos;
        }
    }

    best.map(|(result, _)| result)
}

fn sensor_bounds(samples: &[Sample]) -> Option<(f64, f64, f64, f64)> {
    let first = samples.first()?;

    let mut min_x = first.position[0];
    let mut max_x = first.position[0];
    let mut min_y = first.position[1];
    let mut max_y = first.position[1];

    for sample in samples {
        min_x = min_x.min(sample.position[0]);
        max_x = max_x.max(sample.position[0]);
        min_y = min_y.min(sample.position[1]);
        max_y = max_y.max(sample.position[1]);
    }

    Some((min_x, max_x, min_y, max_y))
}

fn evaluate_position(samples: &[Sample], magnet_pos: [f64; 3]) -> Option<FitResult> {
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];

    for sample in samples {
        let a = dipole_matrix(sample.position, magnet_pos)?;

        for row in 0..3 {
            let b = sample.field[row];

            for col in 0..3 {
                atb[col] += a[row][col] * b;

                for col2 in 0..3 {
                    ata[col][col2] += a[row][col] * a[row][col2];
                }
            }
        }
    }

    let moment = solve_3x3(ata, atb)?;

    let mut sum_sq = 0.0;
    let mut n_components = 0usize;

    for sample in samples {
        let a = dipole_matrix(sample.position, magnet_pos)?;
        let pred = mat_vec_mul(a, moment);

        for i in 0..3 {
            let r = pred[i] - sample.field[i];
            sum_sq += r * r;
            n_components += 1;
        }
    }

    let residual_rms = (sum_sq / n_components as f64).sqrt();

    let moment_norm = (
        moment[0] * moment[0]
            + moment[1] * moment[1]
            + moment[2] * moment[2]
    ).sqrt();

    Some(FitResult {
        position: (magnet_pos[0], magnet_pos[1], magnet_pos[2]),
        residual_rms,
        objective_score: residual_rms,
        n_sensors: samples.len(),
        moment: (moment[0], moment[1], moment[2]),
        moment_norm,
    })
}

fn evaluate_position_moment_norm_prior(
    samples: &[Sample],
    magnet_pos: [f64; 3],
    target_moment_norm: f64,
) -> Option<FitResult> {
    if !target_moment_norm.is_finite() || target_moment_norm <= 1.0e-12 {
        return None;
    }

    let mut result = evaluate_position(samples, magnet_pos)?;

    if !result.moment_norm.is_finite() || result.moment_norm <= 1.0e-12 {
        return None;
    }

    let log_ratio = (result.moment_norm / target_moment_norm).ln();
    let moment_penalty = MOMENT_NORM_PRIOR_WEIGHT_MT * log_ratio * log_ratio;

    result.objective_score = result.residual_rms + moment_penalty;

    Some(result)
}

fn dipole_matrix(sensor_pos: [f64; 3], magnet_pos: [f64; 3]) -> Option<[[f64; 3]; 3]> {
    let rx = sensor_pos[0] - magnet_pos[0];
    let ry = sensor_pos[1] - magnet_pos[1];
    let rz = sensor_pos[2] - magnet_pos[2];

    let r2 = rx * rx + ry * ry + rz * rz;

    if r2 < 1.0e-9 {
        return None;
    }

    let r = r2.sqrt();
    let r3 = r2 * r;
    let r5 = r3 * r2;

    let rr = [
        [rx * rx, rx * ry, rx * rz],
        [ry * rx, ry * ry, ry * rz],
        [rz * rx, rz * ry, rz * rz],
    ];

    let mut a = [[0.0_f64; 3]; 3];

    for i in 0..3 {
        for j in 0..3 {
            let identity = if i == j { 1.0 } else { 0.0 };
            a[i][j] = 3.0 * rr[i][j] / r5 - identity / r3;
        }
    }

    Some(a)
}

fn mat_vec_mul(a: [[f64; 3]; 3], x: [f64; 3]) -> [f64; 3] {
    [
        a[0][0] * x[0] + a[0][1] * x[1] + a[0][2] * x[2],
        a[1][0] * x[0] + a[1][1] * x[1] + a[1][2] * x[2],
        a[2][0] * x[0] + a[2][1] * x[1] + a[2][2] * x[2],
    ]
}

fn solve_3x3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = determinant_3x3(a);

    if det.abs() < 1.0e-18 {
        return None;
    }

    let mut ax = a;
    ax[0][0] = b[0];
    ax[1][0] = b[1];
    ax[2][0] = b[2];

    let mut ay = a;
    ay[0][1] = b[0];
    ay[1][1] = b[1];
    ay[2][1] = b[2];

    let mut az = a;
    az[0][2] = b[0];
    az[1][2] = b[1];
    az[2][2] = b[2];

    Some([
        determinant_3x3(ax) / det,
        determinant_3x3(ay) / det,
        determinant_3x3(az) / det,
    ])
}

fn determinant_3x3(a: [[f64; 3]; 3]) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(sensor_index: u8, field: [f64; 3]) -> Sample {
        Sample {
            sensor_index,
            position: [0.0, 0.0, 0.0],
            field,
        }
    }

    fn streamed_field(frame_id: u32, address: u8, time: u64) -> SensorField {
        let mut field = SensorField::default();
        field.frame_id = frame_id;
        field.address = address;
        field.time = time;
        field
    }

    #[test]
    fn frame_accumulator_drops_incomplete_previous_sweep() {
        let mut accumulator = BoardFrameAccumulator::default();

        assert!(accumulator
            .update(streamed_field(1, 0x0C, 10), 2)
            .is_none());

        // A new explicit firmware frame arrives before frame 1 completed.
        // Sensor 0 from frame 1 must be discarded.
        assert!(accumulator
            .update(streamed_field(2, 0x0D, 20), 2)
            .is_none());

        let completed = accumulator
            .update(streamed_field(2, 0x0C, 21), 2)
            .expect("two sensors from frame 2 should complete one board frame");

        assert_eq!(completed.frame_id, 2);
        assert_eq!(completed.fields.len(), 2);
        assert!(completed.fields.iter().all(|field| field.frame_id == 2));
    }

    #[test]
    fn background_capture_averages_twenty_complete_frames_per_sensor_and_axis() {
        let mut capture = BackgroundCapture::default();
        let mut result = None;

        for frame_index in 0..BACKGROUND_CAPTURE_FRAMES {
            let f = frame_index as f64;
            let samples = vec![
                sample(0, [f, 2.0 * f, -f]),
                sample(1, [10.0 + f, 20.0 + f, 30.0 + f]),
            ];

            result = capture.add_complete_frame(&samples);

            if frame_index + 1 < BACKGROUND_CAPTURE_FRAMES {
                assert!(result.is_none());
            }
        }

        let background = result.expect("20th frame should finish background capture");
        let expected_mean = (BACKGROUND_CAPTURE_FRAMES as f64 - 1.0) / 2.0;

        assert_eq!(background.len(), 2);
        assert!((background[&0][0] - expected_mean).abs() < 1.0e-12);
        assert!((background[&0][1] - 2.0 * expected_mean).abs() < 1.0e-12);
        assert!((background[&0][2] + expected_mean).abs() < 1.0e-12);
        assert!((background[&1][0] - (10.0 + expected_mean)).abs() < 1.0e-12);
        assert!((background[&1][1] - (20.0 + expected_mean)).abs() < 1.0e-12);
        assert!((background[&1][2] - (30.0 + expected_mean)).abs() < 1.0e-12);
    }

    #[test]
    fn background_capture_rejects_a_changed_sensor_set_without_advancing() {
        let mut capture = BackgroundCapture::default();

        assert!(capture
            .add_complete_frame(&[
                sample(0, [1.0, 2.0, 3.0]),
                sample(1, [4.0, 5.0, 6.0]),
            ])
            .is_none());
        assert_eq!(capture.frames_collected, 1);

        assert!(capture
            .add_complete_frame(&[sample(0, [7.0, 8.0, 9.0])])
            .is_none());
        assert_eq!(capture.frames_collected, 1);
    }

    #[test]
    fn background_subtraction_is_per_sensor_and_per_axis() {
        let samples = vec![
            sample(0, [10.0, 20.0, 30.0]),
            sample(1, [-5.0, 8.0, 12.0]),
        ];
        let background = BTreeMap::from([
            (0, [1.0, 2.0, 3.0]),
            (1, [-2.0, 3.0, 4.0]),
        ]);

        let corrected = subtract_background_samples(&samples, &background);

        assert_eq!(corrected.len(), 2);
        assert_eq!(corrected[0].field, [9.0, 18.0, 27.0]);
        assert_eq!(corrected[1].field, [-3.0, 5.0, 8.0]);
    }
}
