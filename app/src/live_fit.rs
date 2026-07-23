use std::collections::BTreeMap;

use data_transfer::rpc::SensorField;

#[derive(Debug, Clone, Default)]
pub struct LiveFitState {
    latest: BTreeMap<(u16, u8), SensorField>,
    result: Option<FitResult>,
}

#[derive(Debug, Clone)]
pub struct FitResult {
    pub position: (f64, f64, f64),
    pub residual_rms: f64,
    pub n_sensors: usize,
}

#[derive(Debug, Clone)]
struct Sample {
    position: [f64; 3],
    field: [f64; 3],
}

impl LiveFitState {
    pub fn update(&mut self, field: SensorField) {
        self.latest.insert((field.board_id, field.address), field);

        let samples = self.samples();

        if samples.len() >= 6 {
            self.result = fit_dipole_grid(&samples);
        }
    }

    pub fn result(&self) -> Option<&FitResult> {
        self.result.as_ref()
    }

    pub fn n_sensors(&self) -> usize {
        self.latest.len()
    }

    fn samples(&self) -> Vec<Sample> {
        self.latest
            .values()
            .filter_map(sensor_field_to_sample)
            .collect()
    }
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

fn fit_dipole_grid(samples: &[Sample]) -> Option<FitResult> {
    let (min_x, max_x, min_y, max_y) = sensor_bounds(samples)?;

    let mut best: Option<(FitResult, [f64; 3])> = None;

    // Coarse-to-fine grid search.
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