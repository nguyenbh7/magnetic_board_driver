

use std::ops::Deref;
use std::path::PathBuf;
use std::{collections::{BTreeMap, VecDeque}, time::Duration};

use crossterm::event::{self, Event, KeyCode};
use data_transfer::rpc::{BoardFrameTopic, StopFieldStream};
use postcard::experimental::max_size::MaxSize;
use postcard_rpc::host_client::{HostClient, Subscription};
use postcard_rpc::standard_icd::WireError;
use ratatui::{text::Text, widgets::Row, Frame};
mod sensor_monitor;
mod displacement_plot;
use displacement_plot::{displacement_plot, rotation_plot};
mod rigid_xyz_plot;
use rigid_xyz_plot::rigid_xyz_plot;
mod sensor_trace;
mod sensor_trace_plot;
use sensor_trace::SensorTraceState;
use sensor_trace_plot::sensor_trace_plot;
mod live_fit;
mod rigid_fit;
mod stage_presets;
use stage_presets::MagnetLocationPreset;
use rigid_fit::{RigidFitCompletion, RigidFitJob, RigidLiveFit, RigidProfile, RigidSummary};
use live_fit::{BoardFitSummary, BoardLiveFits, FitCompletion, FitJob, MagnetPreset};
use rfd::FileHandle;
use sensor_monitor::MagneticData;
use sipper::Sender;
use tokio::sync::mpsc::UnboundedSender;
use tokio::sync::watch::{self, Receiver};
use std::io::{BufWriter, Write};
use std::fs::File;

use crate::sensor_monitor::{SensorSubscription, SensorWatcher};
use iced::widget::{
    button, column, combo_box, container, pick_list, row, scrollable, text, text_input,
};
use iced::{Element, Length, Task};
use std::fmt::{Display, format};
use std::sync::Arc;
use tokio::sync::Mutex;
use data_transfer::{
    self,
    conversions::{MagneticField, MagneticValue, TempValue},
    messaging::MessageReader,
    rpc::{
        SensorField,
        BoardFrame,
        SingleFieldValue,
        StartFieldStream,
        GetBoardPresence,
        BoardPresence,
        MAX_SENSOR_BOARDS,
        SENSOR_POSITIONS_MM,
        GetMlxSensitivity,
        SetMlxSensitivity,
        MlxSensitivityConfig,
        MlxSensitivityStatus,
        GetMlxTiming,
        MlxTimingStatus,
    },
};

#[derive(Debug, Clone)]
struct SerialPortInfo(serialport::SerialPortInfo);

impl Display for SerialPortInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0.port_name)
    }
}



#[derive(Debug, Clone)]
enum Message {
    PortSelected(SerialPortInfo),
    UpdatePorts,
    UpdatePingBoard(String),
    UpdatePingSensor(String),
    GetField,
    RecievedField(data_transfer::rpc::SensorField),
    ReceivedStreamFrame(data_transfer::rpc::BoardFrame),
    FitCompleted(FitCompletion),
    RigidFitCompleted(RigidFitCompletion),
    SelectMagnetLocations(MagnetLocationPreset),
    LoadRigidProfile,
    RigidProfilePicked(Option<PathBuf>),
    ToggleRigidFit,
    CaptureRigidBackground,
    SelectTrackingMode(TrackingMode),
    ResetRigidZero,
    CopyRigidStatus,
    ReceivedBoardPresence(BoardPresence),
    ResetBoardDisplacement(u16),
    CalibrateBoardMagnet(u16),
    CaptureBoardBackground(u16),
    SelectBoardMagnetPreset {
        board_id: u16,
        preset: MagnetPreset,
    },

    SelectDashboardTab(DashboardTab),
    SelectSensorTrace {
        board_id: u16,
        sensor_index: u8,
    },

    StartFieldStream,
    FieldStreamStarted(Result<(), String>),
    FieldStreamStopped,
    StopFieldStream,
    SelectFile,
    FileOpened(Result<FileHandle, Error>),
    StopLogging,
    WroteFile(usize),
    UpdateMlxGain(String),
    UpdateMlxResolution(String),
    UpdateMlxHallConf(String),
    GetMlxSensitivity,
    SetMlxSensitivity,
    ReceivedMlxSensitivity(MlxSensitivityStatus),
    GetMlxTiming,
    ReceivedMlxTiming(MlxTimingStatus),
    UiScaleIncrease,
    UiScaleDecrease,
    UiScaleReset,
    CopyMlxStatus,
    CopyMlxTiming,
    CopyBoardCadence(u16),
    CopyFitStatus(u16),
    CopyLogStatus,
    CopyAllDiagnostics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum DashboardTab {
    #[default]
    LiveTracking,
    SensorTraces,
    Both,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum TrackingMode {
    #[default]
    Rigid6Dof,
    Independent,
}

impl Display for TrackingMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Rigid6Dof => "Rigid 6-DOF",
            Self::Independent => "Independent fits",
        })
    }
}

impl TrackingMode {
    const ALL: [Self; 2] = [Self::Rigid6Dof, Self::Independent];
}

#[derive(Debug, Clone, Default)]
struct PingArgs {
    board: String,
    sensor: String,
}


#[derive(Default)]
struct SensorGrid {
    sensor_cols: [[SensorField; 4]; 4]
}

#[derive(Default)]
struct SensorGridCollection(BTreeMap<u16, SensorGrid>);

impl SensorGridCollection {
    fn update_with_field(&mut self, field: &SensorField) {
        let grid = if let Some(v) = self.0.get_mut(&field.board_id) {
            v
        } else {
            self.0.insert(field.board_id, SensorGrid::default());
            self.0.get_mut(&field.board_id).unwrap()
        };
    }
}


#[derive(Default)]
struct Context {
    sensor_watcher: Option<SensorWatcher>,
    file_writer: Option<Arc<Mutex<BufWriter<File>>>>,
    sensors: combo_box::State<SerialPortInfo>,
    ping_args: PingArgs,
    ping_field: Option<SensorField>,
    sensor_grids: BTreeMap<u8, SensorGrid>,
    sensor_traces: SensorTraceState,
    dashboard_tab: DashboardTab,
    tracking_mode: TrackingMode,
    board_presence: BoardPresence,
    live_fits: BoardLiveFits,
    rigid_fit: RigidLiveFit,
    mlx_gain: String,
    mlx_resolution: String,
    mlx_hall_conf: String,
    mlx_status: Option<MlxSensitivityStatus>,
    mlx_timing_status: Option<MlxTimingStatus>,
    ui_scale_steps: i8,
    logged_bytes: u64,
    log_started_at: Option<std::time::Instant>,
    log_queue: VecDeque<BoardFrame>,
    log_write_in_progress: bool,
    log_stop_requested: bool,
    stream_status: String,
    received_stream_frames: u64,
}

#[derive(Debug, Clone)]
pub enum Error {
    DialogClosed,
    IoError(tokio::io::ErrorKind),
}

const MLX_HALL_CONF_DEFAULT_LABEL: &str = "0xC = Four-phase / default";
const MLX_HALL_CONF_FAST_LABEL: &str = "0x0 = Two-phase";

fn hall_conf_label_to_value(label: &str) -> u8 {
    match label {
        MLX_HALL_CONF_FAST_LABEL => 0x0,
        MLX_HALL_CONF_DEFAULT_LABEL => 0xC,
        "" => 0xC,
        _ => 0xC,
    }
}

fn hall_conf_value_to_label(value: u8) -> &'static str {
    match value {
        0x0 => MLX_HALL_CONF_FAST_LABEL,
        0xC => MLX_HALL_CONF_DEFAULT_LABEL,
        _ => MLX_HALL_CONF_DEFAULT_LABEL,
    }
}

fn board_presence_text(presence: &BoardPresence) -> String {
    let mut lines = Vec::new();

    for board_index in 0..MAX_SENSOR_BOARDS {
        if presence.board_mask & (1u8 << board_index) == 0 {
            continue;
        }

        let sensor_count = presence.sensor_masks[board_index].count_ones();

        lines.push(format!(
            "Board {}: {}/16 sensors",
            board_index,
            sensor_count,
        ));
    }

    if lines.is_empty() {
        "Connected boards: none detected".to_string()
    } else {
        format!("Connected boards:\n{}", lines.join("\n"))
    }
}

fn ui_scale_factor(context: &Context) -> f32 {
    (1.0 + 0.10 * f32::from(context.ui_scale_steps)).clamp(0.60, 2.00)
}

fn mlx_status_text(context: &Context) -> String {
    match &context.mlx_status {
        Some(status) => format!(
            "Current: ok={} · gain={} · resolution={} · hall_conf=0x{:X}",
            status.ok,
            status.gain,
            status.resolution,
            status.hall_conf,
        ),
        None => "Current: not read yet".to_string(),
    }
}

fn mlx_timing_text(context: &Context) -> String {
    match &context.mlx_timing_status {
        Some(status) if status.ok => format!(
            "Timing: board {} sensor {} · reg02=0x{:02X}{:02X} · OSR={} · DIG_FILT={} · OSR2={} · axis={} us · temp={} us · XYZ+T predicted={} us",
            status.board_id,
            status.sensor_index,
            status.register_02_msb,
            status.register_02_lsb,
            status.osr,
            status.dig_filt,
            status.osr2,
            status.magnetic_axis_conversion_time_us,
            status.temperature_conversion_time_us,
            status.xyz_t_single_measurement_time_us,
        ),
        Some(_) => "Timing: no detected sensor".to_string(),
        None => "Timing: not read yet".to_string(),
    }
}

fn board_cadence_text(summary: &BoardFitSummary) -> String {
    match (
        summary.frame_rate_hz,
        summary.frame_period_ms,
        summary.frame_span_ms,
    ) {
        (Some(rate_hz), Some(period_ms), Some(span_ms)) => format!(
            "Board {} · Cadence: {:.3} Hz · cycle {:.2} ms · acquisition span {:.2} ms · frame {} · incomplete {}",
            summary.board_id,
            rate_hz,
            period_ms,
            span_ms,
            summary
                .current_frame_id
                .map(|id| id.to_string())
                .unwrap_or_else(|| "-".to_string()),
            summary.incomplete_frames,
        ),
        _ => format!(
            "Board {} · Cadence: waiting for completed frames · frame {} · incomplete {}",
            summary.board_id,
            summary
                .current_frame_id
                .map(|id| id.to_string())
                .unwrap_or_else(|| "-".to_string()),
            summary.incomplete_frames,
        ),
    }
}

fn board_fit_status_text(summary: &BoardFitSummary) -> String {
    let duration_text = summary
        .last_fit_duration_ms
        .map(|ms| format!("{:.1} ms", ms))
        .unwrap_or_else(|| "-".to_string());

    format!(
        "Board {} · Fit worker: {} · completed {} · last {}",
        summary.board_id,
        if summary.fit_in_progress { "busy" } else { "idle" },
        summary.completed_fit_count,
        duration_text,
    )
}

fn rigid_fit_status_text(summary: &RigidSummary) -> String {
    let name = summary.profile_name.as_deref().unwrap_or("No calibrated profile loaded");
    let mut text = format!(
        "Rigid 6-DOF · {} · {} · {} · worker {} · completed {} · rejected {} · last {:.1} ms",
        name, if summary.enabled { "enabled" } else { "disabled" }, summary.status,
        if summary.busy { "busy" } else { "idle" },
        summary.completed, summary.rejected, summary.last_duration_ms.unwrap_or(0.0)
    );
    if let Some(result) = &summary.result {
        let p = result.pose;
        text.push_str(&format!(
            "\nAbsolute pose: translation [{:.3}, {:.3}, {:.3}] mm, rotation vector [{:.3}, {:.3}, {:.3}] deg, field RMSE {:.5} mT, sensors {}",
            p[0], p[1], p[2], p[3].to_degrees(), p[4].to_degrees(), p[5].to_degrees(),
            result.rmse_mt, result.sensor_count
        ));
    }
    if let (Some(t), Some(angle)) = (summary.translation_from_zero, summary.rotation_from_zero_deg) {
        text.push_str(&format!(
            "\nRelative to rigid zero: dx={:.3}, dy={:.3}, dz={:.3} mm; rotation={:.3} deg",
            t[0], t[1], t[2], angle
        ));
    }
    text
}

fn sensor_position_from_index(sensor_index: usize) -> (f32, f32, f32) {
    SENSOR_POSITIONS_MM[sensor_index]
}

fn sensor_address_from_index(board_id: u16, sensor_index: usize) -> u8 {
    let address = 0x0C + sensor_index as u8;

    if board_id == 2 {
        address ^ 0b0100_0000
    } else {
        address
    }
}

fn compact_sample_to_sensor_field(
    frame: &BoardFrame,
    sensor_index: usize,
) -> Option<SensorField> {
    if frame.sensor_mask & (1u16 << sensor_index) == 0 {
        return None;
    }

    let sample = frame.samples[sensor_index];

    let magnetic_value = |value: f32| {
        if value.is_finite() {
            Some(MagneticValue::uT(value as f64))
        } else {
            None
        }
    };

    let temperature = if sample.temperature_c.is_finite() {
        Some(TempValue::Celsius(sample.temperature_c as f64))
    } else {
        None
    };

    Some(SensorField {
        field: MagneticField {
            x: magnetic_value(sample.bx_ut),
            y: magnetic_value(sample.by_ut),
            z: magnetic_value(sample.bz_ut),
            t: temperature,
        },
        board_id: frame.board_id,
        frame_id: frame.frame_id,
        position: sensor_position_from_index(sensor_index),
        address: sensor_address_from_index(frame.board_id, sensor_index),
        time: frame.base_time_us + u64::from(sample.time_offset_us),
    })
}

fn log_status_text(context: &Context) -> String {
    let megabytes = context.logged_bytes as f64 / 1_000_000.0;

    if context.file_writer.is_some() {
        let state = if context.log_stop_requested {
            "stopping"
        } else {
            "recording"
        };
        let pending = context.log_queue.len()
            + if context.log_write_in_progress { 1 } else { 0 };

        if let Some(started_at) = context.log_started_at {
            let elapsed_minutes = started_at.elapsed().as_secs_f64() / 60.0;
            if elapsed_minutes > 0.0 {
                return format!(
                    "Log: {} · {:.2} MB written · {:.2} MB/min average · {} pending · compact board-frame v2 format",
                    state,
                    megabytes,
                    megabytes / elapsed_minutes,
                    pending,
                );
            }
        }

        return format!(
            "Log: {} · {:.2} MB written · {} pending · compact board-frame v2 format",
            state,
            megabytes,
            pending,
        );
    }

    if context.logged_bytes > 0 {
        format!(
            "Log: stopped · {:.2} MB in last file · compact board-frame v2 format",
            megabytes,
        )
    } else {
        "Log: not recording · compact board-frame v2 format".to_string()
    }
}

fn all_diagnostics_text(context: &Context) -> String {
    let mut lines = Vec::new();
    lines.push(format!(
        "Stream: {} · host frames received {}",
        if context.stream_status.is_empty() {
            "idle"
        } else {
            context.stream_status.as_str()
        },
        context.received_stream_frames,
    ));
    lines.push(log_status_text(context));
    lines.push(rigid_fit_status_text(&context.rigid_fit.summary()));

    for summary in context.live_fits.board_summaries() {
        lines.push(board_cadence_text(&summary));
        lines.push(board_fit_status_text(&summary));
    }

    lines.join("\n")
}

fn open_rigid_profile(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Select rigid-body geometry/calibration JSON")
        .add_filter("JSON", &["json"])
        .set_parent(&window);
    async move { dialog.pick_file().await.map(|handle| handle.path().to_path_buf()) }
}

fn open_file(
    window: &dyn iced::Window,
) -> impl Future<Output = Result<rfd::FileHandle, Error>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Open a file...")
        .set_directory("/")
        .add_filter("bin", &["bin"])
        .set_parent(&window);

    async move {
        let picked_file = dialog.save_file().await.ok_or(Error::DialogClosed);
        
        picked_file
        
    }
}

async fn get_single_value(board: u32, sensor: u32, client: HostClient<WireError>) -> SensorField {
    let args = (board, sensor);
    client.send_resp::<SingleFieldValue>(&args).await.unwrap()
}

async fn restart_field_stream(client: HostClient<WireError>) -> Result<(), String> {
    // App restarts can leave the firmware's spawned stream task alive.
    // Always request termination first, allow the current acquisition/publish
    // cycle to drain, then start one fresh task.
    let _ = client.send_resp::<StopFieldStream>(&()).await;
    async_std::task::sleep(Duration::from_millis(100)).await;

    client
        .send_resp::<StartFieldStream>(&())
        .await
        .map_err(|err| format!("{err:?}"))
}

async fn get_board_presence(client: HostClient<WireError>) -> BoardPresence {
    client
        .send_resp::<GetBoardPresence>(&())
        .await
        .unwrap_or_default()
}

async fn get_mlx_timing(client: HostClient<WireError>) -> MlxTimingStatus {
    client
        .send_resp::<GetMlxTiming>(&())
        .await
        .unwrap_or_default()
}

async fn field_subscribe(client: HostClient<WireError>) -> Option<SensorSubscription> {
    let subs = client.subscribe_exclusive::<BoardFrameTopic>(64).await.ok()?;
    Some(SensorSubscription::new(subs))
}

async fn write_frame_data(writer: &Mutex<BufWriter<File>>, frame: &BoardFrame) -> usize {
    let mut writer = writer.lock().await;
    let mut data = [0; BoardFrame::POSTCARD_MAX_SIZE];

    if let Ok(encoded) = postcard::to_slice(frame, &mut data) {
        let length = (encoded.len() as u32).to_le_bytes();
        if writer.write_all(&length).is_ok() && writer.write_all(encoded).is_ok() {
            return length.len() + encoded.len();
        }
    }

    0
}

fn next_log_write_task(context: &mut Context) -> Option<Task<Message>> {
    if context.log_write_in_progress {
        return None;
    }

    let Some(frame) = context.log_queue.pop_front() else {
        if context.log_stop_requested {
            context.file_writer = None;
            context.log_started_at = None;
            context.log_stop_requested = false;
        }
        return None;
    };

    let Some(writer) = context.file_writer.as_ref().cloned() else {
        context.log_queue.clear();
        return None;
    };

    context.log_write_in_progress = true;

    Some(Task::perform(
        async move {
            write_frame_data(&*writer, &frame).await
        },
        Message::WroteFile,
    ))
}

async fn stop_field_stream(client: HostClient<WireError>) -> () {
    client.send_resp::<StopFieldStream>(&()).await.unwrap()
}

async fn run_fit_job(job: FitJob) -> FitCompletion {
    async_std::task::spawn_blocking(move || job.run()).await
}

async fn run_rigid_fit_job(job: RigidFitJob) -> RigidFitCompletion {
    async_std::task::spawn_blocking(move || job.run()).await
}

fn update(context: &mut Context, message: Message) -> Task<Message> {

    //println!("{:#?}", message);
    match message {
        Message::PortSelected(serial_port_info) => {
            context.sensor_watcher = Some(SensorWatcher::new(&serial_port_info));
            context.board_presence = BoardPresence::default();
            context.live_fits = BoardLiveFits::default();
            context.rigid_fit.clear_background_for_new_port();
            context.sensor_traces = SensorTraceState::default();
            context.dashboard_tab = DashboardTab::LiveTracking;
            context.tracking_mode = TrackingMode::Rigid6Dof;
            context.stream_status = "idle".to_string();
            context.received_stream_frames = 0;
            Task::none()
        }
        Message::UpdatePorts => {
            let ports = serialport::available_ports()
                .unwrap_or(Vec::new())
                .into_iter()
                .map(SerialPortInfo)
                .collect();
            let port_info = context.sensor_watcher.as_ref().map(|s| s.port_info());
            context.sensors = combo_box::State::<_>::with_selection(ports, port_info);
            Task::none()
        }
        Message::UpdatePingBoard(s) => {
            context.ping_args.board = s;
            Task::none()
        }
        Message::UpdatePingSensor(s) => {
            context.ping_args.sensor = s;
            Task::none()
        }
        Message::GetField => {
            if let Some(sw) = &context.sensor_watcher {
                let board = &context.ping_args.board;
                let sensor = &context.ping_args.sensor;
                let board = board.parse().unwrap_or(0);
                let sensor = sensor.parse().unwrap_or(0);

                let client = sw.get_client();
                Task::perform(
                    get_single_value(board, sensor, client), 
                    Message::RecievedField,
                )
            } else {
                Task::none()
            }

        }
        Message::CalibrateBoardMagnet(board_id) => {
            context.live_fits.calibrate_board_magnet(board_id);
            Task::none()
        }
        Message::CaptureBoardBackground(board_id) => {
            context.live_fits.capture_board_background(board_id);
            Task::none()
        }
        Message::SelectBoardMagnetPreset { board_id, preset } => {
            context.live_fits.set_board_magnet_preset(board_id, preset);
            Task::none()
        }
        Message::SelectMagnetLocations(preset) => {
            context.rigid_fit.select_location_preset(preset);
            Task::none()
        }
        Message::LoadRigidProfile => {
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, open_rigid_profile))
                .then(Task::future)
                .map(Message::RigidProfilePicked)
        }
        Message::RigidProfilePicked(path) => {
            if let Some(path) = path {
                match std::fs::read_to_string(&path)
                    .map_err(|e| e.to_string())
                    .and_then(|src| RigidProfile::from_json(&src)) {
                    Ok(profile) => context.rigid_fit.set_profile(profile),
                    Err(e) => context.rigid_fit.set_status(format!("Rigid profile load failed: {e}")),
                }
            }
            Task::none()
        }
        Message::ToggleRigidFit => {
            let enabled = !context.rigid_fit.enabled();
            context.rigid_fit.set_enabled(enabled);
            Task::none()
        }
        Message::ResetRigidZero => {
            context.rigid_fit.reset_zero();
            Task::none()
        }
        Message::CaptureRigidBackground => {
            match context.rigid_fit.capture_a1_background() {
                Ok(raw) => context.live_fits.apply_background_snapshot(&raw),
                Err(e) => context.rigid_fit.set_status(format!("Background: {e}")),
            }
            Task::none()
        }
        Message::SelectTrackingMode(mode) => {
            context.tracking_mode = mode;
            // Independent field fitting is deliberately opt-in via this
            // visible selector; it does not require any background capture.
            context.live_fits.set_fit_enabled(mode == TrackingMode::Independent);
            Task::none()
        }
        Message::RigidFitCompleted(completion) => {
            context.rigid_fit.apply_completion(completion);
            Task::none()
        }
        Message::CopyRigidStatus => {
            iced::clipboard::write(rigid_fit_status_text(&context.rigid_fit.summary()))
        }
        Message::RecievedField(sensor_field) => {
            context.sensor_traces.update(&sensor_field);
            context.ping_field = Some(sensor_field);
            Task::none()
        },
        Message::ReceivedStreamFrame(frame) => {
            context.received_stream_frames =
                context.received_stream_frames.saturating_add(1);
            context.stream_status = "receiving".to_string();

            let mut fit_job = None;

            for sensor_index in 0..frame.samples.len() {
                let Some(sensor_field) =
                    compact_sample_to_sensor_field(&frame, sensor_index)
                else {
                    continue;
                };

                if let Some(job) = context.live_fits.update(sensor_field.clone()) {
                    fit_job = Some(job);
                }

                context.sensor_traces.update(&sensor_field);
                context.ping_field = Some(sensor_field);
            }

            let mut tasks = Vec::new();

            if let Some(job) = context.rigid_fit.receive_frame(
                &frame,
                context.tracking_mode == TrackingMode::Rigid6Dof,
            ) {
                tasks.push(Task::perform(
                    run_rigid_fit_job(job),
                    Message::RigidFitCompleted,
                ));
            }

            if let Some(job) = fit_job {
                tasks.push(Task::perform(
                    run_fit_job(job),
                    Message::FitCompleted,
                ));
            }

            if context.file_writer.is_some() && !context.log_stop_requested {
                context.log_queue.push_back(frame.clone());

                if let Some(task) = next_log_write_task(context) {
                    tasks.push(task);
                }
            }

            Task::batch(tasks)
        },
        Message::FitCompleted(completion) => {
            context.live_fits.apply_fit_completion(completion);
            Task::none()
        },
        Message::StartFieldStream => {
            context.rigid_fit.reset_stream();
            context.stream_status = "detecting boards".to_string();
            context.received_stream_frames = 0;

            if let Some(sw) = &context.sensor_watcher {
                let client = sw.get_client();

                Task::perform(
                    get_board_presence(client),
                    Message::ReceivedBoardPresence,
                )
            } else {
                Task::none()
            }
        }
        Message::ReceivedBoardPresence(presence) => {
            println!("Detected board presence: {:#?}", presence);

            context.board_presence = presence;
            context.live_fits.set_presence(context.board_presence.clone());
            context.rigid_fit.set_presence(context.board_presence.clone());
            context.sensor_traces.set_presence(context.board_presence.clone());

            context.stream_status = "restarting firmware stream".to_string();

            if let Some(sw) = &context.sensor_watcher {
                let client = sw.get_client();

                Task::perform(
                    restart_field_stream(client),
                    Message::FieldStreamStarted,
                )
            } else {
                Task::none()
            }
        }
        Message::ResetBoardDisplacement(board_id) => {
            context.live_fits.reset_displacement(board_id);
            Task::none()
        }
        Message::FieldStreamStarted(result) => {
            if let Err(err) = result {
                context.stream_status = format!("start failed: {err}");
                eprintln!("Failed to start field stream: {err}");
                return Task::none();
            }

            context.stream_status = "subscribing".to_string();

            if let Some(sw) = &mut context.sensor_watcher {
                let client = sw.get_client();

                Task::future(field_subscribe(client)).and_then(|sub| {
                    Task::run(sub, Message::ReceivedStreamFrame)
                })
            } else {
                Task::none()
            }
        }

        Message::StopFieldStream => {
                context.stream_status = "stopping".to_string();
                if let Some(sw) = &context.sensor_watcher {
                    let client = sw.get_client();
                    Task::perform(
                        stop_field_stream(client), 
                        |_| Message::FieldStreamStopped,
                    )
                } else {
                    Task::none()
                }                
            
        }
        Message::FieldStreamStopped => {
            context.rigid_fit.reset_stream();
            context.stream_status = "idle".to_string();
            Task::none()
        },

        Message::SelectFile => {
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, open_file))
                .then(Task::future)
                .map(Message::FileOpened)
        },
        Message::FileOpened(fh) => {
            if let Ok(fh) = fh {
                let path = fh.path();
                let file = File::create(path).unwrap();
                let mut writer = BufWriter::new(file);
                let _ = writer.write_all(b"MLXBF002");
                let writer = Arc::new(Mutex::new(writer));
                context.file_writer = Some(writer);
                context.logged_bytes = 8;
                context.log_started_at = Some(std::time::Instant::now());
                context.log_queue.clear();
                context.log_write_in_progress = false;
                context.log_stop_requested = false;
            }
            Task::none()
        },
        Message::StopLogging => {
            context.log_stop_requested = true;

            next_log_write_task(context).unwrap_or_else(|| Task::none())
        },
        Message::WroteFile(bytes_written) => {
            context.logged_bytes = context.logged_bytes.saturating_add(bytes_written as u64);
            context.log_write_in_progress = false;

            next_log_write_task(context).unwrap_or_else(|| Task::none())
        },
        Message::UpdateMlxGain(s) => {
            context.mlx_gain = s;
            Task::none()
        }

        Message::UpdateMlxResolution(s) => {
            context.mlx_resolution = s;
            Task::none()
        }

        Message::UpdateMlxHallConf(s) => {
            context.mlx_hall_conf = s;
            Task::none()
        }

        Message::GetMlxSensitivity => {
            if let Some(sw) = &context.sensor_watcher {
                let client = sw.get_client();

                Task::perform(
                    async move {
                        client
                            .send_resp::<GetMlxSensitivity>(&())
                            .await
                            .unwrap()
                    },
                    Message::ReceivedMlxSensitivity,
                )
            } else {
                Task::none()
            }
        }

        Message::SetMlxSensitivity => {
            if let Some(sw) = &context.sensor_watcher {
                let gain = context.mlx_gain.parse::<u8>().unwrap_or(255);
                let resolution = context.mlx_resolution.parse::<u8>().unwrap_or(255);
                let hall_conf = hall_conf_label_to_value(&context.mlx_hall_conf);

                let config = MlxSensitivityConfig {
                    gain,
                    resolution,
                    hall_conf,
                };

                let client = sw.get_client();

                Task::perform(
                    async move {
                        client
                            .send_resp::<SetMlxSensitivity>(&config)
                            .await
                            .unwrap()
                    },
                    Message::ReceivedMlxSensitivity,
                )
            } else {
                Task::none()
            }
        }

        Message::ReceivedMlxSensitivity(status) => {
            context.mlx_gain = status.gain.to_string();
            context.mlx_resolution = status.resolution.to_string();
            context.mlx_hall_conf = hall_conf_value_to_label(status.hall_conf).to_string();
            context.mlx_status = Some(status);

            Task::none()
        }

        Message::GetMlxTiming => {
            if let Some(sw) = &context.sensor_watcher {
                let client = sw.get_client();
                Task::perform(
                    get_mlx_timing(client),
                    Message::ReceivedMlxTiming,
                )
            } else {
                Task::none()
            }
        }

        Message::ReceivedMlxTiming(status) => {
            context.mlx_timing_status = Some(status);
            Task::none()
        }


        Message::UiScaleIncrease => {
            context.ui_scale_steps = (context.ui_scale_steps + 1).min(10);
            Task::none()
        }

        Message::UiScaleDecrease => {
            context.ui_scale_steps = (context.ui_scale_steps - 1).max(-4);
            Task::none()
        }

        Message::UiScaleReset => {
            context.ui_scale_steps = 0;
            Task::none()
        }

        Message::CopyMlxStatus => {
            iced::clipboard::write(mlx_status_text(context))
        }

        Message::CopyMlxTiming => {
            iced::clipboard::write(mlx_timing_text(context))
        }

        Message::CopyBoardCadence(board_id) => {
            let cadence = context
                .live_fits
                .board_summaries()
                .into_iter()
                .find(|summary| summary.board_id == board_id)
                .map(|summary| board_cadence_text(&summary))
                .unwrap_or_else(|| format!("Board {} · Cadence: unavailable", board_id));

            iced::clipboard::write(cadence)
        }

        Message::CopyFitStatus(board_id) => {
            let status = context
                .live_fits
                .board_summaries()
                .into_iter()
                .find(|summary| summary.board_id == board_id)
                .map(|summary| board_fit_status_text(&summary))
                .unwrap_or_else(|| format!("Board {} · Fit worker: unavailable", board_id));

            iced::clipboard::write(status)
        }

        Message::CopyLogStatus => {
            iced::clipboard::write(log_status_text(context))
        }

        Message::CopyAllDiagnostics => {
            iced::clipboard::write(all_diagnostics_text(context))
        }

        Message::SelectDashboardTab(tab) => {
            // Switching between plots does not change the selected fitter:
            // Both always shows the same tracking mode plus sensor traces.
            context.dashboard_tab = tab;
            Task::none()
        }

        Message::SelectSensorTrace {
            board_id,
            sensor_index,
        } => {
            context.sensor_traces.select_sensor(board_id, sensor_index);
            Task::none()
        }
    }
}

fn view(context: &Context) -> Element<'_, Message> {
    let serial_selector = combo_box(
        &context.sensors,
        "Select Sensor",
        context.sensor_watcher.as_ref().map(|s| s.port_info()),
        Message::PortSelected,
    )
    .on_open(Message::UpdatePorts);

    let ping_widget = container(row![
        text("Board Number: "),
        text_input("Board Number", &context.ping_args.board).on_input(Message::UpdatePingBoard),
        text("Sensor Number: "),
        text_input("Sensor Number", &context.ping_args.sensor).on_input(Message::UpdatePingSensor),
        button("Get Field Value").on_press(Message::GetField),
        text(format!("x: {:.2}", context.ping_field.as_ref().map(|f| f.field.x.map(|x| x.value()).unwrap_or(0.0)).unwrap_or(0.0))),
        text(format!("y: {:.2}", context.ping_field.as_ref().map(|f| f.field.y.map(|y| y.value()).unwrap_or(0.0)).unwrap_or(0.0))),
        text(format!("z: {:.2}", context.ping_field.as_ref().map(|f| f.field.z.map(|z| z.value()).unwrap_or(0.0)).unwrap_or(0.0))),
    ]);

    let stream_widget = container(
        column![
            row![
                button("Start Field Stream").on_press(Message::StartFieldStream),
                button("Stop Field Stream").on_press(Message::StopFieldStream),
                button("Copy all diagnostics").on_press(Message::CopyAllDiagnostics),
            ]
            .spacing(8),
            text(format!(
                "Stream: {} · host frames received {}",
                if context.stream_status.is_empty() {
                    "idle"
                } else {
                    context.stream_status.as_str()
                },
                context.received_stream_frames,
            )),
            text(board_presence_text(&context.board_presence)),
            row![
                text(log_status_text(context)),
                button("Copy log stats").on_press(Message::CopyLogStatus),
            ]
            .spacing(8),
            row![
                text("File output: "),
                text_input("File", &context.ping_args.sensor),
                button("Select File").on_press(Message::SelectFile),
                button("Stop logging").on_press(Message::StopLogging)
            ]
        ]
    );

    let live_tracking_widget = {
        let summaries = context.live_fits.board_summaries();

        let rigid = context.rigid_fit.summary();
        let enabled_label = if rigid.enabled { "Disable rigid fit" } else { "Enable rigid fit" };
        let pose_label = rigid_fit_status_text(&rigid);
        let is_a1_demo = rigid.location_preset == Some(MagnetLocationPreset::A1Demo);
        let selected_setup = rigid.location_preset.map(|preset| preset.summary())
            .unwrap_or_else(|| "Custom calibrated geometry (or no setup selected)".into());
        let setup_help = if is_a1_demo {
            if rigid.a1_background_ready {
                "A1 background captured. Resume the motion program, then set rigid zero after an accepted fit."
            } else {
                "Rigid fitting waits for the A1 baseline at the G-code background pause. You can view Sensor Traces or switch to Independent fits immediately."
            }
        } else if rigid.location_preset.is_some() {
            "Gamma Knife stage references specify magnet centers only. Load matching complete CT/stage board geometry and magnetic calibration JSON to track."
        } else {
            "Select A1 V1 for an immediate calibrated demo, or load your own complete rigid geometry JSON."
        };
        let fit_quality = rigid.result.as_ref()
            .map(|r| format!("Field RMSE {:.3} mT · {} sensors · accepted {} / rejected {} · fit {:.0} ms",
                r.rmse_mt, r.sensor_count, rigid.completed.saturating_sub(rigid.rejected),
                rigid.rejected, rigid.last_duration_ms.unwrap_or(0.0)))
            .unwrap_or_else(|| format!("No accepted fit yet · {} completed / {} rejected",
                rigid.completed, rigid.rejected));
        let position_line = rigid.translation_from_zero
            .map(|d| {
                let magnitude = (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]).sqrt();
                format!("X {:+.2} mm     Y {:+.2} mm     Z {:+.2} mm     |Δr| {:.2} mm",
                    d[0], d[1], d[2], magnitude)
            }).unwrap_or_else(|| "XYZ: waiting for accepted fit and zero".into());
        let rotation_line = rigid.rotation_from_zero_deg
            .map(|angle| format!("Relative rotation: {:.2}°", angle))
            .unwrap_or_else(|| "Relative rotation: awaiting fit".into());
        let mut rigid_content = column![
            text("Live Tracking — joint 6-DOF rigid body"),
            row![
                text("Setup:"),
                pick_list(MagnetLocationPreset::ALL.to_vec(),
                    rigid.location_preset, Message::SelectMagnetLocations),
                button("Load geometry (.json)").on_press(Message::LoadRigidProfile),
                button(enabled_label).on_press(Message::ToggleRigidFit),
            ].spacing(8),
            text(selected_setup),
            text(setup_help),
            row![
                button("Set rigid zero").on_press(Message::ResetRigidZero),
                text(if rigid.zero_ready { "Zero: set" } else { "Zero: awaiting fit" }),
            ].spacing(8),
            text(position_line),
            text(rotation_line),
            text(format!("Status: {}", rigid.status)),
            text(fit_quality),
        ].spacing(7);
        // Always show the charts, including before the first background
        // capture or before any fitted pose exists. The canvases display
        // "Waiting..." rather than making the whole tracking view disappear.
        rigid_content = rigid_content
            .push(rigid_xyz_plot(rigid.xyz_history))
            .push(rotation_plot(u16::MAX, rigid.rotation_history));
        let mut live_fit_content = column![
            text("Independent magnet tracking — one fitted magnet per board"),
            text("Works immediately from raw Hall fields; background subtraction is optional. These are not a joint rigid 6-DOF estimate."),
        ].spacing(12);

        if summaries.is_empty() {
            live_fit_content = live_fit_content.push(text("No detected boards"));
        } else {
            for summary in summaries {
                let fit_text = match &summary.result {
                    Some(result) => format!(
                        "Position: x={:.2} mm, y={:.2} mm, z={:.2} mm    Residual RMS: {:.4} mT    Sensors: {}    Fit scale: {:.3e}",
                        result.position.0,
                        result.position.1,
                        result.position.2,
                        result.residual_rms,
                        result.n_sensors,
                        result.moment_norm,
                    ),
                    None => format!(
                        "Waiting for completed frame; seen {}/{} sensors",
                        summary.seen_sensors,
                        summary.expected_sensors,
                    ),
                };

                let board_id = summary.board_id;
                let magnet_options = MagnetPreset::ALL.to_vec();

                let mode_text = if summary.use_known_magnet_prior {
                    "Mode: known magnet prior, free orientation"
                } else if summary.is_calibrated {
                    "Mode: calibrated strength prior, free orientation"
                } else {
                    "Mode: free moment"
                };

                let target_text = summary
                    .target_moment_norm
                    .map(|target| format!("Target moment: {:.3e} mT·mm³", target))
                    .unwrap_or_else(|| "Target moment: none".to_string());

                let cadence_text = board_cadence_text(&summary);
                let fit_status_text = board_fit_status_text(&summary);

                let mut board_controls = row![
                    text(format!("Board {}", summary.board_id)),
                    button("Reset displacement zero")
                        .on_press(Message::ResetBoardDisplacement(summary.board_id)),
                    button("Calibrate magnet / set zero")
                        .on_press(Message::CalibrateBoardMagnet(summary.board_id)),
                ].spacing(12);
                // An A1 snapshot can feed all three fitting boards at once.
                // Keep individual capture for custom/other board setups.
                if !is_a1_demo {
                    board_controls = board_controls.push(
                        button("Capture board background")
                            .on_press(Message::CaptureBoardBackground(summary.board_id))
                    );
                }
                let board_card = container(
                    column![
                        board_controls,

                        row![
                            text("Magnet:"),
                            pick_list(
                                magnet_options,
                                Some(summary.magnet_preset),
                                move |preset| Message::SelectBoardMagnetPreset {
                                    board_id,
                                    preset,
                                },
                            ),
                            text(format!(
                                "Effective scale: {:.3}×",
                                summary.magnet_effective_scale,
                            )),
                            text(target_text),
                        ]
                        .spacing(12),

                        text(mode_text),
                        row![
                            text(cadence_text),
                            button("Copy cadence")
                                .on_press(Message::CopyBoardCadence(summary.board_id)),
                        ]
                        .spacing(8),
                        row![
                            text(fit_status_text),
                            button("Copy fit status")
                                .on_press(Message::CopyFitStatus(summary.board_id)),
                        ]
                        .spacing(8),
                        text(if summary.has_background {
                            "Background: captured"
                        } else {
                            "Background: not captured"
                        }),
                        text(fit_text),
                        displacement_plot(
                            summary.board_id,
                            summary.displacement_history,
                        ),
                    ]
                    .spacing(6)
                )
                .padding(10);

                live_fit_content = live_fit_content.push(board_card);
            }
        }

        let selected_content = match context.tracking_mode {
            TrackingMode::Rigid6Dof => container(rigid_content).padding(10),
            TrackingMode::Independent => container(live_fit_content).padding(10),
        };

        let mut mode_header = column![
            row![
                text("Tracking model:"),
                pick_list(
                    TrackingMode::ALL.to_vec(),
                    Some(context.tracking_mode),
                    Message::SelectTrackingMode,
                ),
            ].spacing(10),
        ].spacing(7);

        if is_a1_demo {
            mode_header = mode_header.push(row![
                button("Capture background (all 3 boards)")
                    .on_press(Message::CaptureRigidBackground),
                text(if rigid.a1_background_ready {
                    "A1 background: captured"
                } else {
                    "A1 background: not captured (rigid fit needs it)"
                }),
            ].spacing(8));
        }

        container(mode_header.push(selected_content)).padding(10)
    };

    let sensor_trace_widget = {
        let summaries = context.sensor_traces.board_summaries();

        let mut trace_content = column![
            text("Sensor field traces"),
            text("Select one sensor per board to plot Bx, By, and Bz over time.")
        ]
        .spacing(12);

        if summaries.is_empty() {
            trace_content = trace_content.push(text("No detected boards"));
        } else {
            for summary in summaries {
                let board_id = summary.board_id;
                let sensor_options = summary.available_sensors.clone();
                let selected_sensor = summary.selected_sensor_index;

                let selected_text = selected_sensor
                    .map(|sensor| format!("Sensor {}", sensor))
                    .unwrap_or_else(|| "No sensor selected".to_string());

                let board_card = container(
                    column![
                        row![
                            text(format!("Board {}", board_id)),
                            text("Sensor index"),
                            pick_list(
                                sensor_options,
                                selected_sensor,
                                move |sensor_index| Message::SelectSensorTrace {
                                    board_id,
                                    sensor_index,
                                },
                            ),
                            text(selected_text),
                        ]
                        .spacing(12),
                        sensor_trace_plot(
                            board_id,
                            selected_sensor,
                            summary.selected_trace,
                        ),
                    ]
                    .spacing(8)
                )
                .padding(10);

                trace_content = trace_content.push(board_card);
            }
        }

        container(trace_content)
    };

    let mlx_status_text = mlx_status_text(context);

    let selected_hall_conf = Some(if context.mlx_hall_conf.is_empty() {
        MLX_HALL_CONF_DEFAULT_LABEL.to_string()
    } else {
        context.mlx_hall_conf.clone()
    });

    let mlx_timing_text = mlx_timing_text(context);

    let mlx_widget = container(
        column![
            row![
                text("MLX90393 sensitivity"),
                text(mlx_status_text),
                button("Copy status").on_press(Message::CopyMlxStatus),
            ]
            .spacing(16),

            text("Changes are applied to the detected sensor boards."),

            row![
                column![
                    text("Gain"),
                    text_input("0..7", &context.mlx_gain)
                        .on_input(Message::UpdateMlxGain),
                    text("0 = max range, 7 = highest sensitivity")
                ]
                .spacing(4),

                column![
                    text("Resolution"),
                    text_input("0..3", &context.mlx_resolution)
                        .on_input(Message::UpdateMlxResolution),
                    text("0 = finest/smallest range, 3 = coarser")
                ]
                .spacing(4),
            ]
            .spacing(18),

            column![
                text("Hall configuration"),
                pick_list(
                    vec![
                        MLX_HALL_CONF_DEFAULT_LABEL.to_string(),
                        MLX_HALL_CONF_FAST_LABEL.to_string(),
                    ],
                    selected_hall_conf,
                    Message::UpdateMlxHallConf,
                ),
                text("Hall configuration changes the Hall sampling topology; timing is diagnosed separately from OSR/DIG_FILT.")
            ]
            .spacing(4),

            row![
                text(mlx_timing_text),
                button("Copy timing").on_press(Message::CopyMlxTiming),
            ]
            .spacing(8),

            row![
                button("Read from board").on_press(Message::GetMlxSensitivity),
                button("Read timing").on_press(Message::GetMlxTiming),
                button("Apply to detected boards").on_press(Message::SetMlxSensitivity),
            ]
            .spacing(12),
        ]
        .spacing(10)
    )
    .padding(10);

    let tab_selector = row![
        button("Live Tracking").on_press(Message::SelectDashboardTab(DashboardTab::LiveTracking)),
        button("Sensor Traces").on_press(Message::SelectDashboardTab(DashboardTab::SensorTraces)),
        button("Both").on_press(Message::SelectDashboardTab(DashboardTab::Both)),
    ]
    .spacing(8);

    let active_dashboard_widget = match context.dashboard_tab {
        DashboardTab::LiveTracking => live_tracking_widget,
        DashboardTab::SensorTraces => sensor_trace_widget,
        DashboardTab::Both => {
            container(
                row![
                    live_tracking_widget.width(Length::FillPortion(1)),
                    sensor_trace_widget.width(Length::FillPortion(1)),
                ]
                .spacing(12)
            )
        }
    };
    
    let mut dashboard_sections = column![
        stream_widget,
        tab_selector,
        active_dashboard_widget,
    ].spacing(12);
    // Keep advanced board/timing settings accessible when the independent
    // tracking mode is selected, without adding a fourth dashboard tab.
    if context.dashboard_tab == DashboardTab::LiveTracking
        && context.tracking_mode == TrackingMode::Independent
    {
        dashboard_sections = dashboard_sections.push(mlx_widget);
    }
    let dashboard_content = container(dashboard_sections).width(Length::Fill);

    let dashboard = scrollable(dashboard_content)
        .width(Length::Fill)
        .height(Length::Fill);

    let scale_percent = (ui_scale_factor(context) * 100.0).round() as u16;
    let scale_controls = row![
        text(format!("UI scale: {}%", scale_percent)),
        button("−").on_press(Message::UiScaleDecrease),
        button("+").on_press(Message::UiScaleIncrease),
        button("Reset").on_press(Message::UiScaleReset),
    ]
    .spacing(8);

    let mut content = column![
        scale_controls,
        serial_selector,
    ].spacing(10);
    if context.dashboard_tab == DashboardTab::LiveTracking
        && context.tracking_mode == TrackingMode::Independent
    {
        content = content.push(ping_widget);
    }
    content.push(dashboard).padding(10).into()
}

#[tokio::main]
pub async fn main() -> iced::Result {
    iced::application(Context::default, update, view)
        .scale_factor(ui_scale_factor)
        .run()
}
