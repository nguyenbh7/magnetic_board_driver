use std::collections::{BTreeMap, VecDeque};

use data_transfer::rpc::{BoardPresence, SensorField};

const EXPECTED_SENSORS_PER_BOARD: usize = 16;
const MAX_HISTORY_POINTS: usize = 600;

#[derive(Debug, Clone, Default)]
pub struct BoardLiveFits {
    boards: BTreeMap<u16, BoardLiveFitState>,
    presence: BoardPresence,
}

#[derive(Debug, Clone, Default)]
pub struct BoardLiveFitState {
    current_frame: BoardFrameAccumulator,
    result: Option<FitResult>,

    origin: Option<(f64, f64, f64)>,
    start_time_us: Option<u64>,
    last_fit_time_us: Option<u64>,
    displacement_history: VecDeque<DisplacementPoint>,
}

#[derive(Debug, Clone, Default)]
struct BoardFrameAccumulator {
    fields: BTreeMap<u8, SensorField>,
    frame_start_time_us: Option<u64>,
    frame_end_time_us: Option<u64>,
}

#[derive(Debug, Clone)]
pub struct FitResult {
    pub position: (f64, f64, f64),
    pub residual_rms: f64,
    pub n_sensors: usize,
}

#[derive(Debug, Clone)]
pub struct DisplacementPoint {
    pub time_s: f64,
    pub displacement_mm: f64,
}

#[derive(Debug, Clone)]
struct CompletedBoardFrame {
    fields: Vec<SensorField>,
    frame_start_time_us: u64,
    frame_end_time_us: u64,
}

#[derive(Debug, Clone)]
struct Sample {
    position: [f64; 3],
    field: [f64; 3],
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

    pub fn update(&mut self, field: SensorField) {
        if !self.is_expected_field(&field) {
            return;
        }

        let board = self.boards.entry(field.board_id).or_default();

        if let Some(frame) = board.current_frame.update(field) {
            board.update_from_completed_frame(frame);
        }
    }

    pub fn board_summaries(&self) -> Vec<BoardFitSummary> {
        self.boards
            .iter()
            .map(|(board_id, board)| BoardFitSummary {
                board_id: *board_id,
                seen_sensors: board.current_frame.fields.len(),
                result: board.result.clone(),
                displacement_history: board.displacement_history.iter().cloned().collect(),
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

        let sensor_index = address_to_sensor_index(field.address);

        self.presence.sensor_masks[board_index] & (1u16 << sensor_index) != 0
    }
}

#[derive(Debug, Clone)]
pub struct BoardFitSummary {
    pub board_id: u16,
    pub seen_sensors: usize,
    pub result: Option<FitResult>,
    pub displacement_history: Vec<DisplacementPoint>,
}

impl BoardLiveFitState {
    fn update_from_completed_frame(&mut self, frame: CompletedBoardFrame) {
        let samples: Vec<_> = frame
            .fields
            .iter()
            .filter_map(sensor_field_to_sample)
            .collect();

        if samples.len() < 6 {
            return;
        }

        if let Some(result) = fit_dipole_grid(&samples) {
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

    fn displacement_plot_text(&self) -> String {
        if self.displacement_history.len() < 2 {
            return "Displacement plot: waiting for fitted motion history".to_string();
        }

        let points: Vec<_> = self.displacement_history.iter().collect();
        let values: Vec<f64> = points.iter().map(|p| p.displacement_mm).collect();

        let t0 = points.first().unwrap().time_s;
        let t1 = points.last().unwrap().time_s;
        let y_last = values.last().copied().unwrap_or(0.0);
        let y_max = values.iter().copied().fold(0.0_f64, f64::max);

        format!(
            "Displacement vs time: {}\n{:.1}s → {:.1}s, current={:.2} mm, max={:.2} mm",
            sparkline(&values),
            t0,
            t1,
            y_last,
            y_max,
        )
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
}

impl BoardFrameAccumulator {
    fn update(&mut self, field: SensorField) -> Option<CompletedBoardFrame> {
        if self.fields.is_empty() {
            self.frame_start_time_us = Some(field.time);
        }

        self.frame_end_time_us = Some(field.time);
        self.fields.insert(field.address, field);

        if self.fields.len() < EXPECTED_SENSORS_PER_BOARD {
            return None;
        }

        let fields = std::mem::take(&mut self.fields)
            .into_values()
            .collect::<Vec<_>>();

        let frame_start_time_us = self.frame_start_time_us.take().unwrap_or(0);
        let frame_end_time_us = self.frame_end_time_us.take().unwrap_or(frame_start_time_us);

        Some(CompletedBoardFrame {
            fields,
            frame_start_time_us,
            frame_end_time_us,
        })
    }
}

fn address_to_sensor_index(address: u8) -> u8 {
    address & 0x0F
}

fn sensor_field_to_sample(field: &SensorField) -> Option<Sample> {
    let bx = field.field.x?.value();
    let by = field.field.y?.value();
    let bz = field.field.z?.value();

    Some(Sample {
        position: [
            field.position.0 as f64,
            field.position.1 as f64,
            field.position.2 as f64,
        ],
        field: [bx, by, bz],
    })
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
    let (min_x, max_x, min_y, max_y) = sensor_bounds(samples)?;

    let mut best: Option<(FitResult, [f64; 3])> = None;

    let mut center = [
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        10.0,
    ];

    let passes = [
        (8.0, 2.0, 36.0),
        (4.0, 2.0, 28.0),
        (2.0, 2.0, 20.0),
        (1.0, 2.0, 14.0),
    ];

    for (step, z_min, z_max) in passes {
        let x_min = center[0] - 3.0 * step;
        let x_max = center[0] + 3.0 * step;
        let y_min = center[1] - 3.0 * step;
        let y_max = center[1] + 3.0 * step;

        let mut x = x_min.max(min_x - 10.0);
        while x <= x_max.min(max_x + 10.0) {
            let mut y = y_min.max(min_y - 10.0);
            while y <= y_max.min(max_y + 10.0) {
                let mut z = z_min;
                while z <= z_max {
                    let candidate_pos = [x, y, z];

                    if let Some(candidate) = evaluate_position(samples, candidate_pos) {
                        let is_better = best
                            .as_ref()
                            .map(|(best_result, _)| {
                                candidate.residual_rms < best_result.residual_rms
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

    Some(FitResult {
        position: (magnet_pos[0], magnet_pos[1], magnet_pos[2]),
        residual_rms,
        n_sensors: samples.len(),
    })
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