use crate::SerialPortInfo;
use std::collections::{BTreeMap, VecDeque};
use std::time::Duration;

use data_transfer::{
    self,
    messaging::MessageReader,
    rpc::{MagneticTopic, PingEndpoint, SensorField, SingleFieldValue},
};

use postcard_rpc::{
    header::VarSeqKind,
    host_client::{HostClient, HostErr, Subscription},
    standard_icd::{ERROR_PATH, WireError},
};
use sipper::{FutureExt, Stream};
use std::convert::Infallible;

const FRAME_TIMING_WINDOW: usize = 60;
const FRAME_TIMING_REPORT_EVERY: usize = 30;
const FRAME_TIMING_MIN_SAMPLES: usize = 10;

pub struct SensorSubscription {
    subscription: Subscription<SensorField>,
    frame_timing: BTreeMap<u16, FrameTimingTracker>,
}

#[derive(Debug, Default)]
struct FrameTimingTracker {
    current_frame_id: Option<u32>,
    current_start_us: Option<u64>,
    current_end_us: Option<u64>,
    current_sample_count: usize,
    last_frame_mid_us: Option<u64>,
    sweep_history_us: VecDeque<u64>,
    period_history_us: VecDeque<u64>,
    samples_per_frame: VecDeque<usize>,
    completed_frames: usize,
}

#[derive(Debug, Clone, Copy)]
struct FrameTimingReport {
    frames_in_window: usize,
    average_sweep_ms: f64,
    average_period_ms: Option<f64>,
    effective_hz: Option<f64>,
    average_samples_per_frame: f64,
}

impl FrameTimingTracker {
    fn observe(&mut self, field: &SensorField) -> Option<FrameTimingReport> {
        // frame_id == 0 is reserved for ad-hoc single reads and is not part of
        // the firmware field stream used by the live fitter.
        if field.frame_id == 0 {
            return None;
        }

        let mut report = None;

        match self.current_frame_id {
            Some(current_frame_id) if current_frame_id != field.frame_id => {
                report = self.finish_current_frame();
                self.start_frame(field);
            }
            None => self.start_frame(field),
            _ => {
                self.current_end_us = Some(field.time);
                self.current_sample_count += 1;
            }
        }

        report
    }

    fn start_frame(&mut self, field: &SensorField) {
        self.current_frame_id = Some(field.frame_id);
        self.current_start_us = Some(field.time);
        self.current_end_us = Some(field.time);
        self.current_sample_count = 1;
    }

    fn finish_current_frame(&mut self) -> Option<FrameTimingReport> {
        let start_us = self.current_start_us?;
        let end_us = self.current_end_us.unwrap_or(start_us);
        let sweep_us = end_us.saturating_sub(start_us);
        let frame_mid_us = start_us + sweep_us / 2;

        push_capped(
            &mut self.sweep_history_us,
            sweep_us,
            FRAME_TIMING_WINDOW,
        );
        push_capped(
            &mut self.samples_per_frame,
            self.current_sample_count,
            FRAME_TIMING_WINDOW,
        );

        if let Some(previous_mid_us) = self.last_frame_mid_us {
            if frame_mid_us > previous_mid_us {
                push_capped(
                    &mut self.period_history_us,
                    frame_mid_us - previous_mid_us,
                    FRAME_TIMING_WINDOW,
                );
            }
        }

        self.last_frame_mid_us = Some(frame_mid_us);
        self.completed_frames += 1;

        self.current_frame_id = None;
        self.current_start_us = None;
        self.current_end_us = None;
        self.current_sample_count = 0;

        if self.completed_frames % FRAME_TIMING_REPORT_EVERY != 0
            || self.sweep_history_us.len() < FRAME_TIMING_MIN_SAMPLES
        {
            return None;
        }

        let average_sweep_us = mean_u64(&self.sweep_history_us)?;
        let average_period_us = mean_u64(&self.period_history_us);
        let average_samples_per_frame = mean_usize(&self.samples_per_frame)?;

        Some(FrameTimingReport {
            frames_in_window: self.sweep_history_us.len(),
            average_sweep_ms: average_sweep_us / 1000.0,
            average_period_ms: average_period_us.map(|period_us| period_us / 1000.0),
            effective_hz: average_period_us
                .filter(|period_us| *period_us > 0.0)
                .map(|period_us| 1_000_000.0 / period_us),
            average_samples_per_frame,
        })
    }
}

fn push_capped<T>(history: &mut VecDeque<T>, value: T, capacity: usize) {
    history.push_back(value);
    while history.len() > capacity {
        history.pop_front();
    }
}

fn mean_u64(values: &VecDeque<u64>) -> Option<f64> {
    if values.is_empty() {
        return None;
    }

    Some(values.iter().map(|value| *value as f64).sum::<f64>() / values.len() as f64)
}

fn mean_usize(values: &VecDeque<usize>) -> Option<f64> {
    if values.is_empty() {
        return None;
    }

    Some(values.iter().map(|value| *value as f64).sum::<f64>() / values.len() as f64)
}

pub struct SensorWatcher {
    client: HostClient<WireError>,
    port_info: SerialPortInfo,
}

#[derive(Debug)]
pub enum SensorError<E> {
    Comms(HostErr<WireError>),
    Endpoint(E),
}

impl<E> From<HostErr<WireError>> for SensorError<E> {
    fn from(value: HostErr<WireError>) -> Self {
        Self::Comms(value)
    }
}

trait FlattenErr {
    type Good;
    type Bad;
    fn flatten(self) -> Result<Self::Good, SensorError<Self::Bad>>;
}

impl<T, E> FlattenErr for Result<T, E> {
    type Good = T;
    type Bad = E;
    fn flatten(self) -> Result<Self::Good, SensorError<Self::Bad>> {
        self.map_err(SensorError::Endpoint)
    }
}

#[derive(Debug, Clone)]
pub struct MagneticData {
    pub field: data_transfer::conversions::MagneticField,
    pub position: (f32, f32, f32),
    pub time: u64,
}

impl MagneticData {
    pub fn from_message(message: &data_transfer::messaging::Message) -> Self {
        Self {
            field: message.field,
            position: message.position,
            time: message.time,
        }
    }
}

impl SensorWatcher {
    pub fn new(serial_port_info: &SerialPortInfo) -> SensorWatcher {
        let p = &serial_port_info.0.port_name;
        let baud_rate = 115200;

        let client =
            HostClient::<WireError>::new_serial_cobs(p, ERROR_PATH, 64, 115_200, VarSeqKind::Seq2);

        SensorWatcher {
            client,
            port_info: serial_port_info.clone(),
        }
    }

    pub fn port_info(&self) -> &SerialPortInfo {
        &self.port_info
    }

    pub fn get_client(&self) -> HostClient<WireError> {
        self.client.clone()
    }

    pub async fn get_single_sensor_field(&self, board: u32, sensor: u32) -> SensorField {
        self.client
            .send_resp::<SingleFieldValue>(&(board, sensor))
            .await
            .unwrap()
    }

    pub async fn subscribe(&mut self) -> Option<Subscription<SensorField>> {
        self.client
            .subscribe_exclusive::<MagneticTopic>(64)
            .await
            .ok()
    }
}

impl SensorSubscription {
    pub fn new(subscription: Subscription<SensorField>) -> Self {
        Self {
            subscription,
            frame_timing: BTreeMap::new(),
        }
    }

    pub async fn recv(&mut self) -> Option<SensorField> {
        let field = self.subscription.recv().await?;

        if let Some(report) = self
            .frame_timing
            .entry(field.board_id)
            .or_default()
            .observe(&field)
        {
            match (report.average_period_ms, report.effective_hz) {
                (Some(period_ms), Some(hz)) => eprintln!(
                    "Board {} frame timing (rolling {} frames): sweep={:.3} ms, period={:.3} ms ({:.2} Hz), samples/frame={:.2}",
                    field.board_id,
                    report.frames_in_window,
                    report.average_sweep_ms,
                    period_ms,
                    hz,
                    report.average_samples_per_frame,
                ),
                _ => eprintln!(
                    "Board {} frame timing (rolling {} frames): sweep={:.3} ms, period=collecting, samples/frame={:.2}",
                    field.board_id,
                    report.frames_in_window,
                    report.average_sweep_ms,
                    report.average_samples_per_frame,
                ),
            }
        }

        Some(field)
    }
}

impl Stream for SensorSubscription {
    type Item = SensorField;

    fn poll_next(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        let fut = self.get_mut().recv();
        let value = std::pin::pin!(fut).poll(cx);
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn streamed_field(frame_id: u32, time: u64) -> SensorField {
        let mut field = SensorField::default();
        field.frame_id = frame_id;
        field.time = time;
        field
    }

    #[test]
    fn frame_timing_tracks_sweep_period_and_samples() {
        let mut tracker = FrameTimingTracker::default();

        assert!(tracker.observe(&streamed_field(1, 100)).is_none());
        assert!(tracker.observe(&streamed_field(1, 120)).is_none());
        assert!(tracker.observe(&streamed_field(1, 160)).is_none());

        // Starting frame 2 finalizes frame 1.
        assert!(tracker.observe(&streamed_field(2, 200)).is_none());
        assert_eq!(tracker.sweep_history_us.back(), Some(&60));
        assert_eq!(tracker.samples_per_frame.back(), Some(&3));

        assert!(tracker.observe(&streamed_field(2, 260)).is_none());

        // Starting frame 3 finalizes frame 2. Midpoints are 130 and 230 us.
        assert!(tracker.observe(&streamed_field(3, 300)).is_none());
        assert_eq!(tracker.sweep_history_us.back(), Some(&60));
        assert_eq!(tracker.period_history_us.back(), Some(&100));
        assert_eq!(tracker.samples_per_frame.back(), Some(&2));
    }

    #[test]
    fn frame_timing_ignores_ad_hoc_frame_zero() {
        let mut tracker = FrameTimingTracker::default();

        assert!(tracker.observe(&streamed_field(0, 100)).is_none());
        assert!(tracker.current_frame_id.is_none());
        assert!(tracker.sweep_history_us.is_empty());
    }
}
