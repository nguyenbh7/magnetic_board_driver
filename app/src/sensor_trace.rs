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
}

impl SensorTraceState {
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

                BoardSensorTraceSummary {
                    board_id: *board_id,
                    available_sensors,
                    selected_sensor_index,
                    selected_trace,
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

fn address_to_sensor_index(address: u8) -> Option<u8> {
    let normalized = address & !0b0100_0000;

    if (0x0C..=0x1B).contains(&normalized) {
        Some(normalized - 0x0C)
    } else {
        None
    }
}

fn sensor_field_to_trace_point(field: &SensorField) -> Option<SensorTracePoint> {
    Some(SensorTracePoint {
        time_s: 0.0,
        bx_mt: field.field.x?.value() / UT_PER_MT,
        by_mt: field.field.y?.value() / UT_PER_MT,
        bz_mt: field.field.z?.value() / UT_PER_MT,
    })
}