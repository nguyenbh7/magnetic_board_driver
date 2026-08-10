use std::f64::consts::PI;

const POSITION_SIGMA_MM: f64 = 6.0;
const ANGLE_SIGMA_RAD: f64 = 35.0 * PI / 180.0;
const LOWER: [f64; 5] = [-100.0, -100.0, 0.1, 0.0, -PI];
const UPPER: [f64; 5] = [100.0, 100.0, 100.0, PI, PI];
const FD_STEP: [f64; 5] = [1.0e-3, 1.0e-3, 1.0e-3, 1.0e-5, 1.0e-5];
const MAX_ITERATIONS: usize = 80;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseSample {
    pub position: [f64; 3],
    pub field: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseState {
    pub values: [f64; 5],
}

impl PoseState {
    pub fn position(self) -> [f64; 3] {
        [self.values[0], self.values[1], self.values[2]]
    }

    pub fn axis(self) -> [f64; 3] {
        axis_from_angles(self.values[3], self.values[4])
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PoseFit {
    pub state: PoseState,
    pub residual_rms: f64,
    pub measurement_sse: f64,
    pub objective_score: f64,
    pub moment: [f64; 3],
    pub moment_norm: f64,
}

pub fn fit_initial_pose(samples: &[PoseSample], moment_norm: f64) -> Option<PoseFit> {
    let starts = [
        [0.0, 0.0, 8.0, 1.0e-6, 0.0],
        [0.0, 0.0, 10.0, 1.0e-6, 0.0],
        [0.0, 0.0, 15.0, 1.0e-6, 0.0],
        [0.0, 0.0, 10.0, 10.0_f64.to_radians(), 0.0],
        [0.0, 0.0, 10.0, 10.0_f64.to_radians(), PI / 2.0],
        [0.0, 0.0, 10.0, 10.0_f64.to_radians(), -PI / 2.0],
    ];

    starts
        .into_iter()
        .filter_map(|start| fit_local(samples, moment_norm, start, None))
        .min_by(|a, b| a.measurement_sse.total_cmp(&b.measurement_sse))
}

pub fn fit_temporal_pose(
    samples: &[PoseSample],
    moment_norm: f64,
    previous: PoseState,
) -> Option<PoseFit> {
    fit_local(samples, moment_norm, previous.values, Some(previous))
}

pub fn fit_reacquire_pose(
    samples: &[PoseSample],
    moment_norm: f64,
    previous: PoseState,
) -> Option<PoseFit> {
    let p = previous.values;
    let offsets = [
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [-2.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, -2.0, 0.0],
        [3.0, 0.0, 0.0],
        [-3.0, 0.0, 0.0],
        [0.0, 3.0, 0.0],
        [0.0, -3.0, 0.0],
    ];

    let mut best = fit_initial_pose(samples, moment_norm);

    for offset in offsets {
        let start = [
            p[0] + offset[0],
            p[1] + offset[1],
            p[2] + offset[2],
            p[3],
            p[4],
        ];
        if let Some(candidate) = fit_local(samples, moment_norm, start, None) {
            let replace = best
                .as_ref()
                .map(|current| candidate.measurement_sse < current.measurement_sse)
                .unwrap_or(true);
            if replace {
                best = Some(candidate);
            }
        }
    }

    best
}

pub fn field_jump_rms_m_t(current: &[PoseSample], previous: &[PoseSample]) -> Option<f64> {
    if current.len() != previous.len() || current.is_empty() {
        return None;
    }

    let mut sum_sq = 0.0;
    let mut n = 0usize;
    for (a, b) in current.iter().zip(previous.iter()) {
        for axis in 0..3 {
            let d = a.field[axis] - b.field[axis];
            sum_sq += d * d;
            n += 1;
        }
    }
    Some((sum_sq / n as f64).sqrt())
}

fn fit_local(
    samples: &[PoseSample],
    moment_norm: f64,
    initial: [f64; 5],
    previous: Option<PoseState>,
) -> Option<PoseFit> {
    if samples.len() < 4 || !moment_norm.is_finite() || moment_norm <= 0.0 {
        return None;
    }

    let mut p = clip_state(initial);
    let mut residual = residual_vector(samples, moment_norm, p, previous)?;
    let mut cost = dot(&residual, &residual);
    let mut lambda = 1.0e-3;

    for _ in 0..MAX_ITERATIONS {
        let jacobian = numerical_jacobian(samples, moment_norm, p, previous, &residual)?;
        let mut h = [[0.0_f64; 5]; 5];
        let mut g = [0.0_f64; 5];

        for row in 0..residual.len() {
            for col in 0..5 {
                g[col] += jacobian[row][col] * residual[row];
                for col2 in 0..5 {
                    h[col][col2] += jacobian[row][col] * jacobian[row][col2];
                }
            }
        }

        let mut accepted = false;
        let mut accepted_delta_norm = 0.0;
        for _ in 0..10 {
            let mut damped = h;
            for i in 0..5 {
                damped[i][i] += lambda * (1.0 + h[i][i].abs());
            }
            let rhs = [-g[0], -g[1], -g[2], -g[3], -g[4]];
            let delta = solve_5x5(damped, rhs)?;
            let trial = clip_state([
                p[0] + delta[0],
                p[1] + delta[1],
                p[2] + delta[2],
                p[3] + delta[3],
                p[4] + delta[4],
            ]);
            let trial_residual = residual_vector(samples, moment_norm, trial, previous)?;
            let trial_cost = dot(&trial_residual, &trial_residual);

            if trial_cost < cost {
                let improvement = cost - trial_cost;
                p = trial;
                residual = trial_residual;
                cost = trial_cost;
                lambda = (lambda / 3.0).max(1.0e-12);
                accepted_delta_norm = delta.iter().map(|v| v * v).sum::<f64>().sqrt();
                accepted = true;
                if improvement < 1.0e-14 {
                    break;
                }
                break;
            }

            lambda = (lambda * 10.0).min(1.0e12);
        }

        if !accepted || accepted_delta_norm < 1.0e-6 {
            break;
        }
    }

    pose_fit(samples, moment_norm, p, cost)
}

fn pose_fit(
    samples: &[PoseSample],
    moment_norm: f64,
    params: [f64; 5],
    objective_score: f64,
) -> Option<PoseFit> {
    let measurement = measurement_residual(samples, moment_norm, params)?;
    let measurement_sse = dot(&measurement, &measurement);
    let residual_rms = (measurement_sse / measurement.len() as f64).sqrt();
    let state = PoseState { values: params };
    let axis = state.axis();
    let moment = [
        moment_norm * axis[0],
        moment_norm * axis[1],
        moment_norm * axis[2],
    ];

    Some(PoseFit {
        state,
        residual_rms,
        measurement_sse,
        objective_score,
        moment,
        moment_norm,
    })
}

fn residual_vector(
    samples: &[PoseSample],
    moment_norm: f64,
    params: [f64; 5],
    previous: Option<PoseState>,
) -> Option<Vec<f64>> {
    let mut residual = measurement_residual(samples, moment_norm, params)?;

    if let Some(previous) = previous {
        residual.push((params[0] - previous.values[0]) / POSITION_SIGMA_MM);
        residual.push((params[1] - previous.values[1]) / POSITION_SIGMA_MM);
        residual.push((params[2] - previous.values[2]) / POSITION_SIGMA_MM);
        residual.push(axis_angle(params, previous.values) / ANGLE_SIGMA_RAD);
    }

    Some(residual)
}

fn measurement_residual(
    samples: &[PoseSample],
    moment_norm: f64,
    params: [f64; 5],
) -> Option<Vec<f64>> {
    let axis = axis_from_angles(params[3], params[4]);
    let moment = [
        moment_norm * axis[0],
        moment_norm * axis[1],
        moment_norm * axis[2],
    ];
    let magnet_position = [params[0], params[1], params[2]];
    let mut residual = Vec::with_capacity(samples.len() * 3);

    for sample in samples {
        let predicted = predict_field(sample.position, magnet_position, moment)?;
        residual.push(predicted[0] - sample.field[0]);
        residual.push(predicted[1] - sample.field[1]);
        residual.push(predicted[2] - sample.field[2]);
    }

    Some(residual)
}

fn predict_field(
    sensor_position: [f64; 3],
    magnet_position: [f64; 3],
    moment: [f64; 3],
) -> Option<[f64; 3]> {
    let r = [
        sensor_position[0] - magnet_position[0],
        sensor_position[1] - magnet_position[1],
        sensor_position[2] - magnet_position[2],
    ];
    let r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    if !r2.is_finite() || r2 < 1.0e-12 {
        return None;
    }
    let radius = r2.sqrt();
    let r3 = r2 * radius;
    let r5 = r3 * r2;
    let dot_rm = r[0] * moment[0] + r[1] * moment[1] + r[2] * moment[2];

    Some([
        3.0 * r[0] * dot_rm / r5 - moment[0] / r3,
        3.0 * r[1] * dot_rm / r5 - moment[1] / r3,
        3.0 * r[2] * dot_rm / r5 - moment[2] / r3,
    ])
}

fn numerical_jacobian(
    samples: &[PoseSample],
    moment_norm: f64,
    params: [f64; 5],
    previous: Option<PoseState>,
    base_residual: &[f64],
) -> Option<Vec<[f64; 5]>> {
    let mut jacobian = vec![[0.0_f64; 5]; base_residual.len()];

    for col in 0..5 {
        let h = FD_STEP[col];
        let mut plus = params;
        plus[col] += h;
        plus = clip_state(plus);
        let plus_r = residual_vector(samples, moment_norm, plus, previous)?;

        let mut minus = params;
        minus[col] -= h;
        minus = clip_state(minus);
        let minus_r = residual_vector(samples, moment_norm, minus, previous)?;

        let centered = if col == 4 {
            true
        } else {
            (plus[col] - minus[col]).abs() > 0.5 * h
        };

        if centered {
            let denominator = 2.0 * h;
            for row in 0..base_residual.len() {
                jacobian[row][col] = (plus_r[row] - minus_r[row]) / denominator;
            }
        } else {
            for row in 0..base_residual.len() {
                jacobian[row][col] = (plus_r[row] - base_residual[row]) / h;
            }
        }
    }

    Some(jacobian)
}

fn solve_5x5(mut a: [[f64; 5]; 5], mut b: [f64; 5]) -> Option<[f64; 5]> {
    for pivot_col in 0..5 {
        let mut pivot_row = pivot_col;
        let mut pivot_abs = a[pivot_col][pivot_col].abs();
        for row in (pivot_col + 1)..5 {
            let candidate = a[row][pivot_col].abs();
            if candidate > pivot_abs {
                pivot_abs = candidate;
                pivot_row = row;
            }
        }
        if !pivot_abs.is_finite() || pivot_abs < 1.0e-18 {
            return None;
        }
        if pivot_row != pivot_col {
            a.swap(pivot_row, pivot_col);
            b.swap(pivot_row, pivot_col);
        }

        let pivot = a[pivot_col][pivot_col];
        for col in pivot_col..5 {
            a[pivot_col][col] /= pivot;
        }
        b[pivot_col] /= pivot;

        for row in 0..5 {
            if row == pivot_col {
                continue;
            }
            let factor = a[row][pivot_col];
            for col in pivot_col..5 {
                a[row][col] -= factor * a[pivot_col][col];
            }
            b[row] -= factor * b[pivot_col];
        }
    }

    if b.iter().all(|v| v.is_finite()) {
        Some(b)
    } else {
        None
    }
}

fn clip_state(mut p: [f64; 5]) -> [f64; 5] {
    p[4] = wrap_phi(p[4]);
    for i in 0..5 {
        p[i] = p[i].clamp(LOWER[i], UPPER[i]);
    }
    p
}

fn wrap_phi(phi: f64) -> f64 {
    (phi + PI).rem_euclid(2.0 * PI) - PI
}

fn axis_from_angles(theta: f64, phi: f64) -> [f64; 3] {
    let sin_theta = theta.sin();
    [sin_theta * phi.cos(), sin_theta * phi.sin(), theta.cos()]
}

fn axis_angle(a: [f64; 5], b: [f64; 5]) -> f64 {
    let aa = axis_from_angles(a[3], a[4]);
    let bb = axis_from_angles(b[3], b[4]);
    let cross = [
        aa[1] * bb[2] - aa[2] * bb[1],
        aa[2] * bb[0] - aa[0] * bb[2],
        aa[0] * bb[1] - aa[1] * bb[0],
    ];
    let cross_norm = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
    let dot = (aa[0] * bb[0] + aa[1] * bb[1] + aa[2] * bb[2]).clamp(-1.0, 1.0);
    cross_norm.atan2(dot)
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid_samples(state: PoseState, moment_norm: f64) -> Vec<PoseSample> {
        let xs = [6.75, 2.25, -2.25, -6.75];
        let ys = [-6.75, -2.25, 2.25, 6.75];
        let axis = state.axis();
        let moment = [
            moment_norm * axis[0],
            moment_norm * axis[1],
            moment_norm * axis[2],
        ];
        let mut samples = Vec::new();

        for y in ys {
            for x in xs {
                let position = [x, y, 0.0];
                let field = predict_field(position, state.position(), moment).unwrap();
                samples.push(PoseSample { position, field });
            }
        }
        samples
    }

    fn axis_error_deg(a: PoseState, b: PoseState) -> f64 {
        axis_angle(a.values, b.values).to_degrees()
    }

    #[test]
    fn initial_multistart_recovers_synthetic_fixed_magnitude_pose() {
        let moment_norm = 37_500.0;
        let truth = PoseState {
            values: [2.0, -3.0, 12.0, 0.35, -0.70],
        };
        let samples = grid_samples(truth, moment_norm);
        let fit = fit_initial_pose(&samples, moment_norm).expect("synthetic pose should fit");
        let p = fit.state.position();

        assert!((p[0] - 2.0).abs() < 0.02);
        assert!((p[1] + 3.0).abs() < 0.02);
        assert!((p[2] - 12.0).abs() < 0.02);
        assert!(axis_error_deg(fit.state, truth) < 0.1);
        assert!(fit.residual_rms < 1.0e-8);
        assert!((fit.moment_norm - moment_norm).abs() < 1.0e-12);
    }

    #[test]
    fn temporal_fit_warm_starts_from_previous_pose() {
        let moment_norm = 37_500.0;
        let previous = PoseState {
            values: [1.0, -2.0, 11.5, 0.30, -0.60],
        };
        let truth = PoseState {
            values: [1.8, -2.4, 12.2, 0.34, -0.66],
        };
        let samples = grid_samples(truth, moment_norm);
        let fit = fit_temporal_pose(&samples, moment_norm, previous)
            .expect("temporal synthetic pose should fit");
        let p = fit.state.position();

        assert!((p[0] - truth.values[0]).abs() < 0.15);
        assert!((p[1] - truth.values[1]).abs() < 0.15);
        assert!((p[2] - truth.values[2]).abs() < 0.15);
        assert!(axis_error_deg(fit.state, truth) < 1.0);
    }

    #[test]
    fn field_jump_is_rms_across_all_sensor_components() {
        let a = vec![
            PoseSample {
                position: [0.0; 3],
                field: [1.0, 2.0, 3.0],
            },
            PoseSample {
                position: [0.0; 3],
                field: [4.0, 5.0, 6.0],
            },
        ];
        let b = vec![
            PoseSample {
                position: [0.0; 3],
                field: [0.0, 1.0, 2.0],
            },
            PoseSample {
                position: [0.0; 3],
                field: [3.0, 4.0, 5.0],
            },
        ];
        let jump = field_jump_rms_m_t(&a, &b).unwrap();
        assert!((jump - 1.0).abs() < 1.0e-12);
    }
}
