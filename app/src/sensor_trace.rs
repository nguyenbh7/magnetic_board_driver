use std::collections::{BTreeMap, VecDeque};

use data_transfer::rpc::{BoardPresence, SensorField};

const MAX_TRACE_POINTS: usize = 600;
const UT_PER_MT: f64 = 1000.0;

#[derive(Debug, Clone, Default)]
pub struct SensorTraceState {
    boards: BTreeMap<u16, BoardSensorTraceState>,
    presence: BoardPresence,
}

#[derive(Debug, Clone, Default)]
struct BoardSensorTraceState {
    selected_sensor_index: Option<u8>,
    traces: BTreeMap<u8, SensorTraceHistory>,
    invalid_samples: u64,
}

#[derive(Debug, Clone, Default)]
struct SensorTraceHistory {
    start_time_us: Option<u64>,
    points: VecDeque<SensorTracePoint>,
}

#[derive(Debug, Clone)]
pub struct SensorTracePoint {
    pub time_s: f64,
    pub bx_mt: f64,
    pub by_mt: f64,
    pub bz_mt: f64,
}

#[derive(Debug, Clone)]
pub struct BoardSensorTraceSummary {
    pub board_id: u16,
    pub available_sensors: Vec<u8>,
    pub selected_sensor_index: Option<u8>,
    pub selected_trace: Vec<SensorTracePoint>,
    /// Peak absolute component across EVERY available recorded sensor,
    /// not only the one currently plotted. No saturation claim is implied.
    pub peak_component_mt: Option<f64>,
    pub peak_sensor_index: Option<u8>,
    /// Latest available Bx/By/Bz magnitudes across ALL firmware sensors.
    /// Each entry is (sensor_index, absolute mT); old window peaks do not
    /// keep the move-away warning latched indefinitely.
    pub latest_axis_peaks_mt: [Option<(u8, f64)>; 3],
    pub invalid_samples: u64,
}

impl SensorTraceState {
    /// A fresh stream must not splice raw traces from a previous recording
    /// into new rigid-body time-zero plots. Preserve sensor choices.
    pub fn reset_histories(&mut self) {
        for board in self.boards.values_mut() {
            board.traces.clear();
            board.invalid_samples=0;
        }
    }

    pub fn set_presence(&mut self, presence: BoardPresence) {
        self.presence = presence;

        self.boards.retain(|board_id, _| {
            let index = *board_id as usize;
            index < self.presence.sensor_masks.len()
                && self.presence.board_mask & (1u8 << index) != 0
        });

        for board_index in 0..self.presence.sensor_masks.len() {
            if self.presence.board_mask & (1u8 << board_index) == 0 {
                continue;
            }

            let available = self.available_sensors_for_board(board_index as u16);
            let board = self.boards.entry(board_index as u16).or_default();

            if board
                .selected_sensor_index
                .is_none_or(|selected| !available.contains(&selected))
            {
                board.selected_sensor_index = available.first().copied();
            }
        }
    }

    pub fn update(&mut self, field: &SensorField) {
        if !self.is_expected_field(field) {
            return;
        }

        let Some(sensor_index) = address_to_sensor_index(field.address) else {
            return;
        };

        let Some(point) = sensor_field_to_trace_point(field) else {
            self.boards.entry(field.board_id).or_default().invalid_samples += 1;
            return;
        };

        let board = self.boards.entry(field.board_id).or_default();

        board
            .selected_sensor_index
            .get_or_insert(sensor_index);

        let history = board.traces.entry(sensor_index).or_default();
        let start_time_us = *history.start_time_us.get_or_insert(field.time);

        let mut point = point;
        point.time_s = field.time.saturating_sub(start_time_us) as f64 / 1_000_000.0;

        history.points.push_back(point);

        while history.points.len() > MAX_TRACE_POINTS {
            history.points.pop_front();
        }
    }

    pub fn select_sensor(&mut self, board_id: u16, sensor_index: u8) {
        let available = self.available_sensors_for_board(board_id);

        if !available.contains(&sensor_index) {
            return;
        }

        self.boards
            .entry(board_id)
            .or_default()
            .selected_sensor_index = Some(sensor_index);
    }

    pub fn board_summaries(&self) -> Vec<BoardSensorTraceSummary> {
        self.boards
            .iter()
            .map(|(board_id, board)| {
                let available_sensors = self.available_sensors_for_board(*board_id);
                let selected_sensor_index = board
                    .selected_sensor_index
                    .filter(|selected| available_sensors.contains(selected))
                    .or_else(|| available_sensors.first().copied());

                let selected_trace = selected_sensor_index
                    .and_then(|sensor_index| board.traces.get(&sensor_index))
                    .map(|history| history.points.iter().cloned().collect())
                    .unwrap_or_default();

                let peak=board.traces.iter().filter_map(|(&sensor,history)|{
                    history.points.iter().map(|p|
                        p.bx_mt.abs().max(p.by_mt.abs()).max(p.bz_mt.abs())
                    ).reduce(f64::max).map(|value|(sensor,value))
                }).max_by(|a,b|a.1.total_cmp(&b.1));
                let mut latest_axis_peaks_mt: [Option<(u8, f64)>; 3] = [None, None, None];
                for (&sensor, history) in &board.traces {
                    if let Some(last) = history.points.back() {
                        for (axis, value) in [last.bx_mt, last.by_mt, last.bz_mt]
                            .into_iter().enumerate()
                        {
                            let magnitude = value.abs();
                            if latest_axis_peaks_mt[axis]
                                .is_none_or(|(_, current)| magnitude > current)
                            {
                                latest_axis_peaks_mt[axis] = Some((sensor, magnitude));
                            }
                        }
                    }
                }
                BoardSensorTraceSummary {
                    board_id: *board_id,
                    available_sensors,
                    selected_sensor_index,
                    selected_trace,
                    peak_component_mt:peak.map(|x|x.1),
                    peak_sensor_index:peak.map(|x|x.0),
                    latest_axis_peaks_mt,
                    invalid_samples:board.invalid_samples,
                }
            })
            .collect()
    }

    fn available_sensors_for_board(&self, board_id: u16) -> Vec<u8> {
        let board_index = board_id as usize;

        let Some(sensor_mask) = self.presence.sensor_masks.get(board_index) else {
            return Vec::new();
        };

        (0..16)
            .filter(|sensor_index| sensor_mask & (1u16 << sensor_index) != 0)
            .map(|sensor_index| sensor_index as u8)
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
}

/// Approximate margin against an OPERATOR-VERIFIED component full-scale;
/// without a valid sensor range, a no-warning state is not evidence of safety.
pub fn above_warning_margin(peak_mt: Option<f64>, limit_mt: Option<f64>) -> bool {
    match (peak_mt,limit_mt) {
        (Some(peak),Some(limit)) if peak.is_finite() && limit.is_finite() && limit>0.0 =>
            peak>=0.9*limit,
        _=>false,
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

fn sensor_field_to_trace_point(field: &SensorField) -> Option<SensorTracePoint> {
    let components=[
        field.field.x?.value() / UT_PER_MT,
        field.field.y?.value() / UT_PER_MT,
        field.field.z?.value() / UT_PER_MT,
    ];
    if components.iter().any(|v|!v.is_finite()) {return None;}
    Some(SensorTracePoint {
        time_s:0.0,
        bx_mt:components[0],by_mt:components[1],bz_mt:components[2],
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn warning_requires_verified_full_scale_and_uses_ninety_percent() {
        assert!(!above_warning_margin(Some(25.0),None));
        assert!(!above_warning_margin(Some(8.9),Some(10.0)));
        assert!(above_warning_margin(Some(9.1),Some(10.0)));
        assert!(!above_warning_margin(Some(999.0),Some(-10.0)));
    }

    #[test]
    fn all_sensor_peak_not_just_selected_sensor_and_new_epoch_clears_history() {
        let mut trace=SensorTraceState::default();
        trace.set_presence(BoardPresence{
            board_mask:0b001,sensor_masks:[0xffff,0,0],
        });
        let board=trace.boards.get_mut(&0).unwrap();
        board.selected_sensor_index=Some(0);
        board.traces.entry(0).or_default().points.push_back(
            SensorTracePoint{time_s:0.0,bx_mt:0.1,by_mt:0.0,bz_mt:0.0});
        board.traces.entry(15).or_default().points.push_back(
            SensorTracePoint{time_s:0.0,bx_mt:-22.0,by_mt:0.0,bz_mt:0.0});
        let summary=trace.board_summaries();
        assert_eq!(summary[0].selected_sensor_index,Some(0));
        assert_eq!(summary[0].peak_component_mt,Some(22.0));
        assert_eq!(summary[0].peak_sensor_index,Some(15));
        assert_eq!(summary[0].latest_axis_peaks_mt[0],Some((15,22.0)));
        assert_eq!(summary[0].latest_axis_peaks_mt[1],Some((0,0.0)));
        assert!(above_warning_margin(summary[0].peak_component_mt,Some(24.0)));
        trace.reset_histories();
        let reset=trace.board_summaries();
        assert_eq!(reset[0].selected_sensor_index,Some(0));
        assert!(reset[0].peak_component_mt.is_none());
        assert_eq!(reset[0].latest_axis_peaks_mt,[None,None,None]);
    }
}
