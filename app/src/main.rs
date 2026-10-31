

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
mod cbct_geometry;
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
    ImportCbctZip,
    ImportCbctFolder,
    LoadCbctDetectionsJson,
    CbctImportPicked(Option<PathBuf>),
    CbctJsonPicked(Option<PathBuf>),
    CbctImportCompleted(Result<cbct_geometry::CbctGeometry, String>),
    CbctProgressTick,
    CancelCbctImport,
    ToggleCbctLog,
    ToggleCbctDetails,
    CopyCbctCoordinates,
    CopyCbctStatus,
    CopyCbctDetails,
    CopyCbctLog,
    CopyGkCalibrationStatus,
    CaptureRigidBackground,
    BeginGkBackground,
    BeginGkReference,
    ReviewCadGkRegistration,
    ApproveCadGkRegistration,
    RevokeCadGkRegistration,
    CalibrateGkTracking,
    EnableExperimentalGkTracking,
    ExportGkCalibration,
    GkCalibrationExportPicked(Option<PathBuf>),
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
    StartLogging,
    StopLogging,
    ResumeLogging,
    WroteFile(Result<usize, String>),
    LogFlushed(Result<(), String>),
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


// Per-experiment acquisition. The raw firmware order and axes are never guessed
// from CT detector ordering. A new CT or port invalidates these measurements.
#[derive(Default)]
struct GkCalibration {
    background: Option<[[[f64; 3]; 16]; 3]>,
    reference: Option<[[[f64; 3]; 16]; 3]>,
    capturing: Option<GkCapture>,
    calibrated_candidate: Option<(RigidProfile, f64)>,
    /// A human must inspect candidate mapping and then separately approve
    /// the specific currently loaded CT. No persistence across CT/port reload.
    cad_review_preview: Option<String>,
    cad_mapping_approved: bool,
    status: String,
}
struct GkCapture {
    reference: bool,
    started: std::time::Instant,
    sum: [[[f64; 3]; 16]; 3],
    counts: [[u32; 16]; 3],
}
impl GkCalibration {
    fn begin(&mut self, reference: bool) {
        self.calibrated_candidate = None;
        self.capturing = Some(GkCapture {
            reference, started: std::time::Instant::now(),
            sum: [[[0.0; 3]; 16]; 3], counts: [[0; 16]; 3],
        });
        if reference { self.reference = None; }
        else { self.background = None; self.reference = None; }
        self.status = if reference { "Capturing stationary magnet fields (3 seconds)..." }
            else { "Capturing no-magnet background (3 seconds)..." }.into();
    }
    fn accept_frame(&mut self, frame: &BoardFrame) {
        let Some(c) = &mut self.capturing else { return; };
        let id = frame.board_id as usize;
        if id >= 3 { return; }
        for (i, sample) in frame.samples.iter().enumerate().take(16) {
            if frame.sensor_mask & (1u16 << i) == 0 { continue; }
            let v = [sample.bx_ut as f64 / 1000.0,
                     sample.by_ut as f64 / 1000.0,
                     sample.bz_ut as f64 / 1000.0];
            if v.iter().any(|x| !x.is_finite()) { continue; }
            for (k, component) in v.iter().enumerate() { c.sum[id][i][k] += component; }
            c.counts[id][i] += 1;
        }
        if c.started.elapsed().as_secs_f64() < 3.0 { return; }
        // Do not accept missing sensors or a snapshot from a single frame.
        let min_count = c.counts.iter().flatten().copied().min().unwrap_or(0);
        if min_count < 2 {
            self.status = format!("Collecting all 48 sensors; minimum samples per sensor: {min_count}");
            return;
        }
        let mut average = c.sum;
        for (board, sensors) in average.iter_mut().enumerate() {
            for (i, vector) in sensors.iter_mut().enumerate() {
                for component in vector { *component /= c.counts[board][i] as f64; }
            }
        }
        let is_reference = c.reference;
        self.capturing = None;
        if is_reference { self.reference = Some(average); }
        else { self.background = Some(average); }
        self.status = format!("{} captured: 48/48 sensors, at least {min_count} frames each",
            if is_reference { "Stationary reference" } else { "No-magnet background" });
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
    cbct_geometry: Option<cbct_geometry::CbctGeometry>,
    gk_calibration: GkCalibration,
    cbct_importing: bool,
    cbct_status: String,
    cbct_progress: Option<Arc<std::sync::Mutex<cbct_geometry::ImportProgress>>>,
    cbct_started: Option<std::time::Instant>,
    cbct_show_log: bool,
    cbct_show_details: bool,
    cbct_last_log: String,
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
    log_finalize_in_progress: bool,
    log_write_failed: bool,
    log_file_dialog_open: bool,
    log_file_path: Option<PathBuf>,
    // True only after a successfully flushed/closed log. A newly selected
    // filename is NOT a log until Start Logging atomically creates it.
    log_resume_available: bool,
    log_notice: String,
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
            if context.stream_status == "receiving" { "recording" } else { "open / waiting for stream" }
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

    if context.log_resume_available {
        format!("Log: closed / append available · {:.2} MB · compact board-frame v2 format",megabytes)
    } else if context.log_file_path.is_some() {
        "Log: filename SELECTED — NOT recording. Press Start Logging · compact board-frame v2 format".into()
    } else {
        "Log: no filename selected — NOT recording · compact board-frame v2 format".into()
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
    if !context.log_notice.is_empty() { lines.push(format!("Log notice: {}", context.log_notice)); }
    if let Some(path) = &context.log_file_path { lines.push(format!("Log filename (current or last): {}", path.display())); }
    lines.push(rigid_fit_status_text(&context.rigid_fit.summary()));

    for summary in context.live_fits.board_summaries() {
        lines.push(board_cadence_text(&summary));
        lines.push(board_fit_status_text(&summary));
    }

    lines.join("\n")
}

// Keep the archived RigidProfile Deserialize-only schema unchanged while
// exporting enough candidate information for offline review and replay.
fn p_to_json(p: &RigidProfile) -> serde_json::Value {
    serde_json::json!({
        "name":p.name, "coordinate_frame":p.coordinate_frame, "origin_mm":p.origin_mm,
        "boards":p.boards.iter().map(|b| serde_json::json!({
            "raw_board_id":b.raw_board_id,"sensors_mm":b.sensors_mm,
            "raw_to_world":b.raw_to_world,"background_mt":b.background_mt,
            "sensor_gains":b.sensor_gains
        })).collect::<Vec<_>>(),
        "magnets":p.magnets.iter().map(|m| serde_json::json!({
            "center_mm":m.center_mm,"axis":m.axis,
            "diameter_mm":m.diameter_mm,"thickness_mm":m.thickness_mm,
            "remanence_mt":m.remanence_mt
        })).collect::<Vec<_>>(),
        "initial_pose":p.initial_pose,"lower_bounds":p.lower_bounds,
        "upper_bounds":p.upper_bounds,"max_rmse_mt":p.max_rmse_mt
    })
}

fn save_gk_calibration_file(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Save NEW GK calibration inputs JSON (existing files are protected)")
        .set_file_name("gk_calibration_inputs.json")
        .add_filter("JSON", &["json"])
        .set_parent(&window);
    async move { dialog.save_file().await.map(|handle| handle.path().to_path_buf()) }
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

fn select_cbct_zip(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Import CBCT/DICOM ZIP series (read-only geometry preview)")
        .add_filter("ZIP DICOM series", &["zip"])
        .set_parent(&window);
    async move { dialog.pick_file().await.map(|handle| handle.path().to_path_buf()) }
}

fn select_cbct_directory(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Select directory containing a complete DICOM series")
        .set_parent(&window);
    async move { dialog.pick_folder().await.map(|handle| handle.path().to_path_buf()) }
}

fn select_cbct_detections_json(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Load existing reviewed CBCT detector output (detections.json)")
        .add_filter("CBCT detector JSON", &["json"])
        .set_parent(&window);
    async move { dialog.pick_file().await.map(|handle| handle.path().to_path_buf()) }
}

async fn run_cbct_import_off_thread(path: PathBuf, progress: Arc<std::sync::Mutex<cbct_geometry::ImportProgress>>) -> Result<cbct_geometry::CbctGeometry, String> {
    // Run SimpleITK/Python out of the UI event loop, independently of
    // live Hall streaming and six-DOF fit jobs.
    async_std::task::spawn_blocking(move || cbct_geometry::import_dicom(&path, progress)).await
}

fn open_file(
    window: &dyn iced::Window,
) -> impl Future<Output = Result<rfd::FileHandle, Error>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Select a NEW log filename (existing files are protected)")
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


 // Never truncate a previous recording. The existence check and create are atomic.
fn create_new_log(path: &std::path::Path) -> std::io::Result<BufWriter<File>> {
    let file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)?;
    let mut writer = BufWriter::new(file);
    writer.write_all(b"MLXBF002")?;
    writer.flush()?;
    Ok(writer)
}

// Append only to a completed, well-framed log. Never truncate/rewrite
// the header or silently repair a file with an incomplete final frame.
fn reopen_log_for_append(path: &std::path::Path) -> std::io::Result<(BufWriter<File>, u64)> {
    use std::io::{Read, Seek, SeekFrom};
    let mut file = std::fs::OpenOptions::new().read(true).append(true).open(path)?;
    let size = file.metadata()?.len();
    if size < 8 {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "Log too short for MLXBF002 header"));
    }
    let mut magic=[0_u8;8];
    file.read_exact(&mut magic)?;
    if &magic != b"MLXBF002" {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "Not an MLXBF002 board-frame log"));
    }
    let mut offset=8_u64;
    while offset<size {
        if size-offset<4 {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "Truncated frame length at end of log"));
        }
        let mut bytes=[0_u8;4];
        file.read_exact(&mut bytes)?;
        let frame_len=u32::from_le_bytes(bytes) as u64;
        if frame_len==0 || frame_len > BoardFrame::POSTCARD_MAX_SIZE as u64
            || frame_len > size-offset-4 {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidData,
                format!("Invalid or incomplete MLXBF002 record at byte {offset}")));
        }
        offset += 4+frame_len;
        file.seek(SeekFrom::Start(offset))?;
    }
    Ok((BufWriter::new(file),size))
}

async fn write_frame_data(
    writer: &Mutex<BufWriter<File>>,
    frame: &BoardFrame,
) -> Result<usize, String> {
    let mut writer = writer.lock().await;
    let mut data = [0; BoardFrame::POSTCARD_MAX_SIZE];
    let encoded = postcard::to_slice(frame, &mut data)
        .map_err(|err| format!("Failed to encode BoardFrame: {err}"))?;
    let length = (encoded.len() as u32).to_le_bytes();
    writer.write_all(&length).map_err(|err| err.to_string())?;
    writer.write_all(encoded).map_err(|err| err.to_string())?;
    Ok(length.len() + encoded.len())
}

fn next_log_write_task(context: &mut Context) -> Option<Task<Message>> {
    if context.log_write_in_progress || context.log_finalize_in_progress {
        return None;
    }

    if let Some(frame) = context.log_queue.pop_front() {
        let Some(writer) = context.file_writer.as_ref().cloned() else {
            context.log_queue.clear();
            return None;
        };
        context.log_write_in_progress = true;
        return Some(Task::perform(
            async move { write_frame_data(&*writer, &frame).await },
            Message::WroteFile,
        ));
    }

    if context.log_stop_requested {
        if let Some(writer) = context.file_writer.as_ref().cloned() {
            context.log_finalize_in_progress = true;
            return Some(Task::perform(
                async move { writer.lock().await.flush().map_err(|err| err.to_string()) },
                Message::LogFlushed,
            ));
        }
    }
    None
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
            context.rigid_fit.invalidate_for_gk_calibration();
            context.gk_calibration = GkCalibration::default();
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
        Message::ImportCbctZip => {
            if context.cbct_importing { return Task::none(); }
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, select_cbct_zip))
                .then(Task::future)
                .map(Message::CbctImportPicked)
        }
        Message::ImportCbctFolder => {
            if context.cbct_importing { return Task::none(); }
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, select_cbct_directory))
                .then(Task::future)
                .map(Message::CbctImportPicked)
        }
        Message::LoadCbctDetectionsJson => {
            if context.cbct_importing { return Task::none(); }
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, select_cbct_detections_json))
                .then(Task::future)
                .map(Message::CbctJsonPicked)
        }
        Message::CbctImportPicked(path) => {
            if let Some(path) = path {
                if context.cbct_importing { return Task::none(); }
                context.cbct_importing = true;
                context.cbct_status = format!(
                    "Detecting initial magnets, 48 chips, and glass BB from {} (background task)...",
                    path.display()
                );
                context.cbct_geometry = None;
                context.gk_calibration.calibrated_candidate = None;
                context.gk_calibration.cad_review_preview = None;
                context.gk_calibration.cad_mapping_approved = false;
                context.rigid_fit.invalidate_for_gk_calibration();
                context.cbct_last_log.clear();
                context.cbct_show_details = false;
                let progress = Arc::new(std::sync::Mutex::new(cbct_geometry::ImportProgress::default()));
                context.cbct_progress = Some(Arc::clone(&progress));
                context.cbct_started = Some(std::time::Instant::now());
                Task::perform(run_cbct_import_off_thread(path, progress), Message::CbctImportCompleted)
            } else {
                Task::none()
            }
        }
        Message::CbctJsonPicked(path) => {
            if let Some(path) = path {
                if context.cbct_importing { return Task::none(); }
                let loaded = std::fs::read_to_string(&path)
                    .map_err(|e| format!("Cannot open CBCT detector JSON: {e}"))
                    .and_then(|source| {
                        cbct_geometry::CbctGeometry::from_detector_json(
                            &source,
                            path.clone(),
                            path.parent().unwrap_or_else(|| std::path::Path::new(".")).to_path_buf(),
                        )
                    });
                match loaded {
                    Ok(geometry) => {
                        context.cbct_status = "CT initial geometry imported for review; live magnetic profile unchanged.".into();
                        context.cbct_geometry = Some(geometry);
                    // Preserve previously captured raw reference/background
                    // when DICOM is processed after acquisition. Review scan
                    // correspondence before enabling experimental tracking.
                    context.gk_calibration.calibrated_candidate = None;
                    context.gk_calibration.cad_review_preview = None;
                    context.gk_calibration.cad_mapping_approved = false;
                    context.rigid_fit.invalidate_for_gk_calibration();
                    }
                    Err(error) => {
                        context.cbct_status = format!("CBCT import failed: {error}");
                    }
                }
            }
            Task::none()
        }
        Message::CbctProgressTick => {
            if let Some(progress) = &context.cbct_progress {
                if let Ok(state) = progress.lock() {
                    if !state.latest.is_empty() { context.cbct_status = state.latest.clone(); }
                }
            }
            Task::none()
        }
        Message::CancelCbctImport => {
            if let Some(progress) = &context.cbct_progress {
                if let Ok(state) = progress.lock() {
                    state.cancel.store(true, std::sync::atomic::Ordering::Relaxed);
                }
                context.cbct_status = "Cancelling detection...".into();
            }
            Task::none()
        }
        Message::ToggleCbctDetails => {
            context.cbct_show_details = !context.cbct_show_details;
            Task::none()
        }
        Message::ToggleCbctLog => {
            context.cbct_show_log = !context.cbct_show_log;
            Task::none()
        }
        Message::CbctImportCompleted(result) => {
            if let Some(progress) = &context.cbct_progress {
                if let Ok(state) = progress.lock() { context.cbct_last_log = state.log.join("\n"); }
            }
            context.cbct_progress = None;
            context.cbct_started = None;
            context.cbct_importing = false;
            match result {
                Ok(geometry) => {
                    context.cbct_status = if geometry.markers.is_empty() {
                        "DICOM import: 3 magnets and 48 chips detected; NO glass BB. Geometry ready for review; HDMM marker unavailable.".into()
                    } else {
                        "DICOM import complete; initial geometry ready for review (no magnetic calibration activated).".into()
                    };
                    context.cbct_geometry = Some(geometry);
                context.gk_calibration.calibrated_candidate = None;
                context.gk_calibration.cad_review_preview = None;
                context.gk_calibration.cad_mapping_approved = false;
                context.rigid_fit.invalidate_for_gk_calibration();
                }
                Err(error) => {
                    context.cbct_status = format!("DICOM import failed: {error}");
                }
            }
            Task::none()
        }
        Message::CopyCbctStatus => {
            iced::clipboard::write(format!("DICOM import status: {}\nCT registration: {}",
                context.cbct_status,
                context.cbct_geometry.as_ref().map(|g|g.registration_preview_v2())
                    .unwrap_or_else(|| "No geometry imported.".into())))
        }
        Message::CopyCbctDetails => {
            match &context.cbct_geometry {
                Some(g) => iced::clipboard::write(format!("{}\n\n{}",
                    g.summary(),g.registration_preview_v2())),
                None => iced::clipboard::write(format!("No CT geometry. Status: {}",context.cbct_status)),
            }
        }
        Message::CopyCbctLog => {
            let log=context.cbct_progress.as_ref()
                .and_then(|p|p.lock().ok().map(|state|state.log.join("\n")))
                .unwrap_or_else(||context.cbct_last_log.clone());
            iced::clipboard::write(format!("DICOM status: {}\n\n{}",context.cbct_status,log))
        }
        Message::CopyGkCalibrationStatus => {
            let mut result=format!("GK calibration status: {}\n",context.gk_calibration.status);
            if let Some(g)=&context.cbct_geometry {
                result.push_str(&format!("CT: {}\n{}\n",
                    g.scan_path.display(),g.registration_preview_v2()));
            }
            result.push_str(&format!(
                "CAD registration preview:\n{}\nExplicit approval for this CT: {}\n",
                context.gk_calibration.cad_review_preview.as_deref()
                    .unwrap_or("not reviewed"),
                context.gk_calibration.cad_mapping_approved));
            if let Some((profile,rmse))=&context.gk_calibration.calibrated_candidate {
                result.push_str(&format!("Experimental candidate strengths [mT]: {:?}; RMSE {:.5} mT\n",
                    profile.magnets.iter().map(|m|m.remanence_mt).collect::<Vec<_>>(),rmse));
            }
            iced::clipboard::write(result)
        }
        Message::CopyCbctCoordinates => {
            match &context.cbct_geometry {
                Some(geometry) => iced::clipboard::write(geometry.coordinates_csv()),
                None => Task::none(),
            }
        }
        Message::BeginGkBackground => {
            context.rigid_fit.invalidate_for_gk_calibration();
            context.gk_calibration.begin(false);
            Task::none()
        }
        Message::BeginGkReference => {
            // Capture all 48 raw Hall fields even if CT cannot be imported
            // at the GK console. They can be paired with DICOM offline.
            context.rigid_fit.invalidate_for_gk_calibration();
            context.gk_calibration.begin(true);
            Task::none()
        }
        Message::ReviewCadGkRegistration => {
            context.gk_calibration.cad_mapping_approved = false;
            context.gk_calibration.cad_review_preview = None;
            context.gk_calibration.calibrated_candidate = None;
            context.rigid_fit.invalidate_for_gk_calibration();
            match context.cbct_geometry.as_ref() {
                Some(geometry) => match geometry.registration_review_cad_preview() {
                    Ok(details) => {
                        context.gk_calibration.cad_review_preview = Some(details);
                        context.gk_calibration.status =
                            "CAD/JST mounting-based registration geometry PASSED. Inspect the full mapping below and confirm raw board wiring; THEN explicitly approve for experimental magnetic calibration.".into();
                    }
                    Err(error) => {
                        context.gk_calibration.status =
                            format!("CAD mapping preview REJECTED: {error}");
                    }
                },
                None => {
                    context.gk_calibration.status =
                        "Import CT geometry before reviewing the CAD mapping.".into();
                }
            }
            Task::none()
        }
        Message::ApproveCadGkRegistration => {
            // Preview AND the current candidate must independently validate.
            let validated=context.cbct_geometry.as_ref()
                .ok_or_else(|| "CT geometry missing".to_string())
                .and_then(|g|g.registration_review_cad_preview());
            match validated {
                Ok(current) if context.gk_calibration.cad_review_preview.as_ref()==Some(&current) => {
                    context.gk_calibration.cad_mapping_approved=true;
                    context.gk_calibration.calibrated_candidate=None;
                    context.gk_calibration.status =
                        "CAD/JST sensor mapping explicitly APPROVED FOR EXPERIMENTAL CALIBRATION ONLY (not CT-measured connector geometry, not live motion accuracy). Capture stationary fields and calibrate; live tracking remains disabled.".into();
                }
                Ok(_) => {
                    context.gk_calibration.cad_mapping_approved=false;
                    context.gk_calibration.status =
                        "CAD mapping was not previewed for this CT; review again.".into();
                }
                Err(error) => {
                    context.gk_calibration.cad_mapping_approved=false;
                    context.gk_calibration.status=format!(
                        "CAD mapping approval rejected: {error}");
                }
            }
            Task::none()
        }
        Message::RevokeCadGkRegistration => {
            context.gk_calibration.cad_mapping_approved=false;
            context.gk_calibration.cad_review_preview=None;
            context.gk_calibration.calibrated_candidate=None;
            context.rigid_fit.invalidate_for_gk_calibration();
            context.gk_calibration.status =
                "CAD mapping approval revoked; experimental tracking disabled.".into();
            Task::none()
        }
        Message::CalibrateGkTracking => {
            context.gk_calibration.calibrated_candidate = None;
            let outcome = (|| -> Result<(RigidProfile, [f64;3], f64, bool), String> {
                let geometry = context.cbct_geometry.as_ref().ok_or("Import the current CT first")?;
                let reference = context.gk_calibration.reference.as_ref()
                    .ok_or("Capture the stationary magnet fields first")?;
                let mut profile=if context.gk_calibration.cad_mapping_approved {
                    geometry.experimental_candidate_profile_with_reviewed_cad()?
                } else {
                    geometry.experimental_candidate_profile()?
                };
                let background_used=context.gk_calibration.background.is_some();
                let (strengths,rmse)=if let Some(background)=context.gk_calibration.background.as_ref() {
                    rigid_fit::fit_magnet_strengths(&profile,background,reference)?
                } else {
                    // No background: estimate 3 signed strengths AND 3
                    // board-constant nuisance field vectors from spatial
                    // sensor-to-sensor field differences. Lower-confidence fit.
                    let (strengths, offsets, rmse)=
                        rigid_fit::fit_strengths_with_board_offsets(&profile,reference)?;
                    for board in &mut profile.boards {
                        board.background_mt=offsets[board.raw_board_id as usize];
                    }
                    (strengths,rmse)
                };
                for (magnet, strength) in profile.magnets.iter_mut().zip(strengths) {
                    magnet.remanence_mt=strength;
                }
                Ok((profile,strengths,rmse,background_used))
            })();
            match outcome {
                Ok((profile,strengths,rmse,background_used)) => {
                    // Screening limits, not proof of clinical/HDMM accuracy.
                    let plausible = strengths.iter().all(|s|s.abs()>=300.0 && s.abs()<=2200.0);
                    if rmse <= profile.max_rmse_mt && plausible {
                        context.gk_calibration.calibrated_candidate=Some((profile,rmse));
                        context.gk_calibration.status=format!(
                            "EXPERIMENTAL {} calibration candidate: signed remanences [{:.1}, {:.1}, {:.1}] mT; field RMSE {:.4} mT. Review CT/JST and magnet dimensions before enabling.",
                            if background_used { "measured-background" } else { "NO-BACKGROUND (fitted board offsets; lower confidence)" },
                            strengths[0],strengths[1],strengths[2],rmse);
                    } else {
                        context.gk_calibration.status=format!(
                            "Calibration QC FAILED (not enabled): strengths [{:.1}, {:.1}, {:.1}] mT; field RMSE {:.4} mT (limit {:.2}); 300–2200 mT absolute strength required. Save data for offline analysis.",
                            strengths[0],strengths[1],strengths[2],rmse,profile.max_rmse_mt);
                    }
                }
                Err(error) => {
                    context.gk_calibration.status=format!(
                        "Calibration cannot proceed: {error}. Previous rigid profile remains disabled.");
                }
            }
            Task::none()
        }
        Message::EnableExperimentalGkTracking => {
            if let Some((profile,rmse)) = context.gk_calibration.calibrated_candidate.clone() {
                context.rigid_fit.set_experimental_gk_profile(profile,context.gk_calibration.background);
                context.tracking_mode=TrackingMode::Rigid6Dof;
                context.live_fits.set_fit_enabled(false);
                context.gk_calibration.status=format!(
                    "EXPERIMENTAL 6-DOF model enabled (stationary field RMSE {rmse:.4} mT). Compare with HDMM and raw/independent tracking; not yet validated.");
            } else {
                context.gk_calibration.status="No accepted experimental candidate; recalibrate.".into();
            }
            Task::none()
        }
        Message::ExportGkCalibration => {
            if context.gk_calibration.reference.is_none() {
                context.gk_calibration.status = "Capture stationary Hall reference before export; CT and background optional.".into();
                return Task::none();
            }
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, save_gk_calibration_file))
                .then(Task::future)
                .map(Message::GkCalibrationExportPicked)
        }
        Message::GkCalibrationExportPicked(path) => {
            if let Some(path) = path {
                let data = context.cbct_geometry.as_ref()
                    .zip(context.gk_calibration.reference.as_ref())
                    .map(|(geometry, reference)| {
                        serde_json::json!({
                            "format": "gk_experiment_calibration_inputs_v1",
                            "status": "experimental_geometry_candidate_not_independently_validated",
                            "units": {"positions": "mm", "fields": "mT"},
                            "sensor_fields_order": "raw_firmware_index_0_to_15",
                            "geometry_sensor_order": "CT_detector_order_with_v2_JST_candidate",
                            "frame_of_reference_uid": geometry.frame_uid,
                            "detector_replay": {
                                "format": "gk_cbct_detections_v1",
                                "coordinate_frame": "DICOM_LPS_mm",
                                "frame_of_reference_uid": geometry.frame_uid,
                                "status": "unreviewed",
                                "requires_review": true,
                                "objects": geometry.magnets.iter().map(|m|serde_json::json!({
                                    "kind":"magnet","name":m.name,
                                    "center_lps_mm":m.center_lps_mm,
                                    "unsigned_axis_lps":m.unsigned_axis_lps
                                })).chain(geometry.arrays.iter().map(|a|serde_json::json!({
                                    "kind":"array","name":a.name,
                                    "center_lps_mm":a.center_lps_mm,
                                    "chip_grid_lps_mm":a.detected_chip_positions_lps_mm,
                                    "grid_rms_mm":a.grid_rms_mm,
                                    "board_center_lps_mm":a.pcb_center_lps_mm,
                                    "board_normal_lps":a.chip_face_normal_lps,
                                    "detection_method":a.detection_method,
                                    "pcb_center_cad_estimate_lps_mm":a.pcb_center_cad_estimate_lps_mm,
                                    "pcb_cad_geometry": if a.cad_nominal_prior_valid {
                                        Some(serde_json::json!({
                                            "status":"CAD_PREDICTION_NOT_CT_MEASUREMENT",
                                            "source":"MLX90393_board4/MLX90393_board3.kicad_pcb Edge.Cuts + J1-J3; known GK JST/chip-face mounting",
                                            "nominal_pcb_dimensions_mm":[43.4,60.45],
                                            "board_center_offset_toward_jst_mm":14.875,
                                            "jst_toward_from_documented_mounting_lps":match a.name.as_str() {
                                                "array_superior"=>[-1.,0.,0.],
                                                _=>[0.,0.,-1.]
                                            },
                                            "jst_connector_footprint_centers_lps_mm":a.jst_connector_cad_estimates_lps_mm,
                                            "connectors_found_in_ct":false,
                                            "pcb_outline_found_in_ct":false
                                        }))
                                    } else {None}
                                }))).chain(geometry.markers.iter().map(|m|serde_json::json!({
                                    "kind":"bb","name":m.name,
                                    "center_lps_mm":m.center_lps_mm
                                }))).collect::<Vec<_>>()
                            },
                            "scan_source": geometry.scan_path.to_string_lossy(),
                            "rigid_origin_lps_mm": geometry.rigid_origin_lps_mm,
                            "rigid_axes_to_lps": geometry.rigid_axes_to_lps,
                            "magnets": geometry.magnets.iter().map(|m| serde_json::json!({
                                "name": m.name, "center_lps_mm": m.center_lps_mm,
                                "unsigned_axis_lps": m.unsigned_axis_lps
                            })).collect::<Vec<_>>(),
                            "sensor_arrays": geometry.arrays.iter().map(|a| serde_json::json!({
                                "name": a.name, "center_lps_mm": a.center_lps_mm,
                                "detected_chip_positions_lps_mm": a.detected_chip_positions_lps_mm,
                                "grid_rms_mm": a.grid_rms_mm,
                                "pcb_center_lps_mm":a.pcb_center_lps_mm,
                                "chip_face_normal_lps":a.chip_face_normal_lps,
                                "pcb_center_cad_estimate_lps_mm":a.pcb_center_cad_estimate_lps_mm,
                                "jst_cad_landmarks_lps_mm":a.jst_connector_cad_estimates_lps_mm,
                                "cad_prior_validated_by_geometry":a.cad_nominal_prior_valid
                            })).collect::<Vec<_>>(),
                            "cad_mapping_review_preview":context.gk_calibration.cad_review_preview,
                            "cad_mapping_approved_in_session":context.gk_calibration.cad_mapping_approved,
                            "cad_mapping_approved_scope":"experimental_calibration_only_not_CT_JST_measurement",
                            "registration_v2_candidates": (
                                if context.gk_calibration.cad_mapping_approved {
                                    geometry.register_ct_boards_v2_with_reviewed_cad()
                                } else {geometry.register_ct_boards_v2()}
                            ).ok().map(|boards| {
                                boards.iter().map(|b|serde_json::json!({
                                    "detected_board":b.detection_name,"raw_board_id":b.raw_board_id,
                                    "firmware_to_detected_index":b.firmware_to_detector,
                                    "raw_to_lps":b.raw_to_lps,
                                    "geometric_rms_mm":b.geometric_rms_mm,
                                    "jst_proxy_offset_mm":b.jst_proxy_offset_mm
                                })).collect::<Vec<_>>()
                            }),
                            "background_raw_mt": context.gk_calibration.background.as_ref(),
                            "background_method": if context.gk_calibration.background.is_some() {
                                "measured_per_sensor"
                            } else {
                                "not_measured_board_constant_nuisance_fit_if_calibrated"
                            },
                            "stationary_raw_mt": reference,
                            "calibration_candidate": context.gk_calibration.calibrated_candidate.as_ref()
                                .map(|(p,rmse)|serde_json::json!({
                                    "signed_strengths_mt": p.magnets.iter().map(|m|m.remanence_mt).collect::<Vec<_>>(),
                                    "stationary_rmse_mt": rmse,
                                    "candidate_profile": p_to_json(p)
                                })),
                            "calibration_model_activated": context.rigid_fit.enabled()
                        })
                    });
                let data = data.or_else(|| context.gk_calibration.reference.as_ref().map(|reference| {
                    // A complete raw-field record survives even if today's
                    // DICOM is missing or the detector finds too few objects.
                    serde_json::json!({
                        "format":"gk_experiment_calibration_inputs_v1",
                        "status":"pending_ct_geometry_no_model_activated",
                        "units":{"positions":"mm","fields":"mT"},
                        "sensor_fields_order":"raw_firmware_index_0_to_15",
                        "frame_of_reference_uid":null,
                        "detector_replay":null,
                        "registration_v2_candidates":null,
                        "background_raw_mt":context.gk_calibration.background.as_ref(),
                        "background_method":if context.gk_calibration.background.is_some() {
                            "measured_per_sensor"
                        } else {
                            "not_measured"
                        },
                        "stationary_raw_mt":reference,
                        "calibration_candidate":null,
                        "calibration_model_activated":false,
                        "warning":"No verified CT geometry in this export; pair with matching DICOM scan offline. Do not treat absent background as zero."
                    })
                }));
                let result = data.ok_or_else(|| "Stationary Hall reference no longer available".to_string())
                    .and_then(|data| serde_json::to_vec_pretty(&data).map_err(|e|e.to_string()))
                    .and_then(|bytes| {
                        use std::io::Write as _;
                        let mut file = std::fs::OpenOptions::new()
                            .write(true).create_new(true).open(&path)
                            .map_err(|e| format!("Cannot create NEW file: {e}"))?;
                        file.write_all(&bytes).map_err(|e|e.to_string())
                    });
                context.gk_calibration.status = match result {
                    Ok(()) => format!("GK calibration inputs saved: {}",path.display()),
                    Err(e) => format!("GK calibration export failed: {e}"),
                };
            }
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

            context.gk_calibration.accept_frame(&frame);
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
            context.gk_calibration.capturing = None;
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
                context.gk_calibration.capturing = None;
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
            // Freeze the completed rigid history for review (same behavior as
            // independent plots). The next StartFieldStream still resets
            // fitting state for a fresh acquisition.
            context.rigid_fit.stop_stream();
            context.stream_status = "idle".to_string();
            Task::none()
        },

        Message::SelectFile => {
            if context.file_writer.is_some() || context.log_stop_requested
                || context.log_write_in_progress || context.log_finalize_in_progress {
                context.log_notice = "Finish stopping/closing the current log before selecting a new file.".into();
                return Task::none();
            }
            if context.log_file_dialog_open {return Task::none();}
            context.log_file_dialog_open=true;
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, open_file))
                .then(Task::future)
                .map(Message::FileOpened)
        },
        Message::FileOpened(picked) => {
            context.log_file_dialog_open=false;
            if context.file_writer.is_some() || context.log_stop_requested
                || context.log_write_in_progress || context.log_finalize_in_progress {
                context.log_notice="File selection ignored: existing log is still open or closing.".into();
                return Task::none();
            }
            if let Ok(handle)=picked {
                let path=handle.path().to_path_buf();
                if path.exists() {
                    // Do not change a previously resumable log into a
                    // pending-new file, and NEVER overwrite an old recording.
                    context.log_notice=format!(
                        "EXISTING FILE PROTECTED: {}. To append to your last log, use Resume previous log; otherwise choose a NEW unique filename.",
                        path.display());
                } else {
                    // Selection ONLY stores the path; no file is opened or
                    // created until the user explicitly clicks Start Logging.
                    context.log_file_path=Some(path.clone());
                    context.log_resume_available=false;
                    context.logged_bytes=0;
                    context.log_started_at=None;
                    context.log_queue.clear();
                    context.log_write_in_progress=false;
                    context.log_stop_requested=false;
                    context.log_finalize_in_progress=false;
                    context.log_write_failed=false;
                    context.log_notice=format!(
                        "Filename selected (NOT recording): {}. Click Start Logging to create and record into it.",path.display());
                }
            }
            Task::none()
        },
        Message::StartLogging => {
            if context.file_writer.is_some() || context.log_stop_requested
                || context.log_write_in_progress || context.log_finalize_in_progress
                || context.log_file_dialog_open {
                context.log_notice="Logging already active, selecting a file, or still closing.".into();
                return Task::none();
            }
            if context.log_resume_available {
                context.log_notice="This log already exists. Use Resume previous log (append) to preserve it, or Select File for a NEW recording.".into();
                return Task::none();
            }
            let Some(path)=context.log_file_path.clone() else {
                context.log_notice="Select a NEW filename before starting logging.".into();
                return Task::none();
            };
            // Atomic create_new protects against another application creating
            // the selected file between the file dialog and this click.
            match create_new_log(&path) {
                Ok(writer) => {
                    context.file_writer=Some(Arc::new(Mutex::new(writer)));
                    context.logged_bytes=8;
                    context.log_started_at=Some(std::time::Instant::now());
                    context.log_queue.clear();
                    context.log_write_in_progress=false;
                    context.log_stop_requested=false;
                    context.log_finalize_in_progress=false;
                    context.log_write_failed=false;
                    context.log_notice=format!(
                        "LOGGING STARTED: {}. Saving incoming board frames; start the field stream if needed.",path.display());
                }
                Err(error) if error.kind()==std::io::ErrorKind::AlreadyExists => {
                    context.log_notice=format!(
                        "EXISTING FILE PROTECTED: {}. No data overwritten. Select a NEW filename.",path.display());
                }
                Err(error) => {
                    context.log_notice=format!("Cannot start logging to {}: {error}",path.display());
                }
            }
            Task::none()
        },
        Message::ResumeLogging => {
            if context.file_writer.is_some() || context.log_stop_requested
                || context.log_write_in_progress || context.log_finalize_in_progress
                || context.log_file_dialog_open {
                context.log_notice = "Cannot resume until the prior log is fully flushed and closed.".into();
                return Task::none();
            }
            if context.log_write_failed {
                context.log_notice = "Previous write failed: resume refused. Preserve this file and select a NEW filename.".into();
                return Task::none();
            }
            if !context.log_resume_available {
                context.log_notice = "No successfully closed log to resume; press Start Logging for a new selected filename.".into();
                return Task::none();
            }
            if let Some(path) = context.log_file_path.clone() {
                match reopen_log_for_append(&path) {
                    Ok((writer,bytes)) => {
                        context.file_writer=Some(Arc::new(Mutex::new(writer)));
                        context.log_resume_available=false;
                        context.logged_bytes=bytes;
                        context.log_started_at=Some(std::time::Instant::now());
                        context.log_queue.clear();
                        context.log_stop_requested=false;
                        context.log_finalize_in_progress=false;
                        context.log_write_in_progress=false;
                        context.log_notice=format!(
                            "APPENDING to existing log: {}. Previous bytes preserved; Start Field Stream if stopped.",path.display());
                    }
                    Err(error) => {
                        context.log_notice=format!(
                            "Cannot safely resume {}: {error}. Existing file unchanged; select a NEW filename.",path.display());
                    }
                }
            } else {
                context.log_notice="No previous log to resume; select a NEW filename.".into();
            }
            Task::none()
        }
        Message::StopLogging => {
            if context.file_writer.is_none() || context.log_stop_requested {
                return Task::none();
            }
            context.log_stop_requested = true;
            context.log_notice = "Stopping log: draining frames and flushing to disk...".into();
            next_log_write_task(context).unwrap_or_else(Task::none)
        },
        Message::WroteFile(result) => {
            context.log_write_in_progress = false;
            match result {
                Ok(bytes_written) => {
                    context.logged_bytes = context.logged_bytes.saturating_add(bytes_written as u64);
                }
                Err(err) => {
                    context.log_write_failed = true;
                    context.log_queue.clear();
                    context.log_stop_requested = true;
                    context.log_notice = format!(
                        "LOG WRITE ERROR: {err}. Stopped recording; inspect file integrity."
                    );
                }
            }
            next_log_write_task(context).unwrap_or_else(Task::none)
        },
        Message::LogFlushed(result) => {
            context.file_writer = None;
            context.log_started_at = None;
            context.log_stop_requested = false;
            context.log_finalize_in_progress = false;
            context.log_resume_available = result.is_ok() && !context.log_write_failed;
            context.log_notice = match result {
                Err(err) => format!(
                    "LOG FLUSH ERROR: {err}. Inspect file integrity before use."
                ),
                Ok(()) if context.log_write_failed =>
                    "Log closed following a write error; inspect file integrity.".into(),
                Ok(()) => "Log flushed and closed. Use Resume previous log (append), or Select File for a new one.".into(),
            };
            Task::none()
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

    let can_manage_log=context.file_writer.is_none()
        && !context.log_stop_requested && !context.log_finalize_in_progress
        && !context.log_write_in_progress && !context.log_file_dialog_open;
    let mut select_log_button=button("Select File");
    if can_manage_log {
        select_log_button=select_log_button.on_press(Message::SelectFile);
    }
    let mut start_log_button=button("Start Logging");
    if can_manage_log && context.log_file_path.is_some() && !context.log_resume_available {
        start_log_button=start_log_button.on_press(Message::StartLogging);
    }
    let mut resume_log_button=button("Resume Previous Log (append)");
    if can_manage_log && context.log_resume_available && !context.log_write_failed {
        resume_log_button=resume_log_button.on_press(Message::ResumeLogging);
    }
    let mut stop_log_button=button("Stop Logging");
    if context.file_writer.is_some() && !context.log_stop_requested {
        stop_log_button=stop_log_button.on_press(Message::StopLogging);
    }
    let log_path_label = context.log_file_path.as_ref()
        .map(|path| path.display().to_string())
        .unwrap_or_else(|| "None selected".into());

    // Keep the complete acquisition workflow in one visually distinct panel.
    // The instructions deliberately precede Select File and the stream controls.
    let logging_panel = container(
        column![
            text("Measurement logging").size(18),
            text("1. Select File: choose a NEW .bin filename (nothing is created or recorded yet)."),
            text("2. Start Field Stream to view sensor data. Start Logging separately to CREATE the selected file and save incoming frames."),
            text("3. Stop Logging drains and closes the file. Resume Previous Log (append) continues that file; Start Logging never overwrites."),
            text("4. Stop Field Stream pauses acquisition only; Restart Field Stream continues recording if logging is active."),
            row![
                select_log_button,
                start_log_button,
                stop_log_button,
            ].spacing(8),
            row![
                resume_log_button,
            ].spacing(8),
            row![
                button("Start Field Stream").on_press(Message::StartFieldStream),
                button("Stop Field Stream").on_press(Message::StopFieldStream),
            ].spacing(8),
            text(format!("Current / last log: {log_path_label}")),
            row![
                text(log_status_text(context)),
                button("Copy log stats").on_press(Message::CopyLogStatus),
            ].spacing(8),
            text(if context.log_notice.is_empty() {
                "Select a new .bin filename before starting the stream.".to_string()
            } else {
                context.log_notice.clone()
            }),
            text("LOGGING and STREAMING are independent. File selected alone is NOT recording. Resume checks MLXBF002 frame boundaries and appends only; Stop Logging does not stop streaming."),
        ]
        .spacing(6),
    )
    .padding(12)
    .width(Length::Fill)
    .style(iced::widget::container::bordered_box);

    let stream_widget = container(
        column![
            logging_panel,
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
            button("Copy all diagnostics").on_press(Message::CopyAllDiagnostics),
        ]
        .spacing(10),
    );

    let live_tracking_widget = {
        let summaries = context.live_fits.board_summaries();

        let rigid = context.rigid_fit.summary();
        // Retained only for archived A1-specific background handling.
        let is_a1_demo = rigid.location_preset == Some(MagnetLocationPreset::A1Demo);
        let pose_label = rigid_fit_status_text(&rigid);
        let profile_status = if let Some(name) = &rigid.profile_name {
            format!("Active profile: {name}")
        } else if rigid.location_preset.is_some() {
            "Profile: calibration required (setup reference only)".to_string()
        } else {
            "Active profile: none — load a calibrated Gamma Knife profile".to_string()
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
        let dicom_panel = container(column![
            text("DICOM Geometry — CT preview"),
            row![
                button("Import ZIP").on_press(Message::ImportCbctZip),
                button("Import folder").on_press(Message::ImportCbctFolder),
                button("Open detections.json").on_press(Message::LoadCbctDetectionsJson),
            ].spacing(8),
            text(if context.cbct_status.is_empty() {
                "No DICOM geometry imported".to_string()
            } else { context.cbct_status.clone() }),
            // Animated activity indicator: NOT a measured percentage. Detection
            // stages currently provide no reliable work denominator.
            text(if let Some(start) = context.cbct_started {
                let slot = (start.elapsed().as_millis() / 250 % 20) as usize;
                let mut cells = vec!['░'; 20];
                for offset in 0..5 { cells[(slot + offset) % 20] = '█'; }
                format!("{}  Working · {:.0}s elapsed", cells.into_iter().collect::<String>(),
                    start.elapsed().as_secs_f64())
            } else { String::new() }),
            row![
                if context.cbct_importing {
                    button("Cancel detection").on_press(Message::CancelCbctImport)
                } else {
                    button("Cancel detection")
                },
                button(if context.cbct_show_log { "Hide log" } else { "Show log" })
                    .on_press(Message::ToggleCbctLog),
                button(if context.cbct_show_details { "Hide details" } else { "Show details" })
                    .on_press(Message::ToggleCbctDetails),
            ].spacing(8),
            row![
                if context.cbct_geometry.is_some() {
                    button("Copy CT positions").on_press(Message::CopyCbctCoordinates)
                } else { button("Copy CT positions") },
                button("Copy status").on_press(Message::CopyCbctStatus),
                if context.cbct_geometry.is_some() {
                    button("Copy details").on_press(Message::CopyCbctDetails)
                } else { button("Copy details") },
                button("Copy log").on_press(Message::CopyCbctLog),
            ].spacing(8),
            text(context.cbct_geometry.as_ref().map(|geometry| {
                format!("{} · {} magnets · {} sensor arrays ({} chips) · {} glass BB candidate\nUnreviewed CT geometry only — magnetic calibration and HDMM identity not verified.",
                    geometry.scan_path.file_name().map(|v| v.to_string_lossy().into_owned()).unwrap_or_default(),
                    geometry.magnets.len(), geometry.arrays.len(),
                    geometry.arrays.iter().map(|a|a.detected_chip_positions_lps_mm.len()).sum::<usize>(),
                    geometry.markers.len())
            }).unwrap_or_default()),
            text(context.cbct_geometry.as_ref().map(|geometry| {
                match geometry.register_ct_boards_v2() {
                    Ok(boards) => format!("CT sensor mapping v2: {} boards registered geometrically (provisional; review JST and chip-face orientation).",boards.len()),
                    Err(error) => format!("CT sensor mapping v2: review needed — {error}")
                }
            }).unwrap_or_default()),
            text(if context.cbct_show_details {
                context.cbct_geometry.as_ref()
                    .map(|geometry| format!("{}\n\n{}",geometry.summary(),geometry.registration_preview_v2()))
                    .unwrap_or_else(|| "No geometry results to display.".into())
            } else { String::new() }),
            text(if context.cbct_show_log {
                context.cbct_progress.as_ref()
                    .and_then(|p| p.lock().ok().map(|state| state.log.join("\n")))
                    .unwrap_or_else(|| if context.cbct_last_log.is_empty() {
                        "No detection log available.".into()
                    } else { context.cbct_last_log.clone() })
            } else { String::new() }),
        ].spacing(6))
        .padding(12)
        .width(Length::Fill)
        .style(iced::widget::container::bordered_box);

        // DICOM imports are unreviewed geometry, never a tracking profile.
        // Historical A1/stage presets remain available in the implementation,
        // but no preset selection is required for the GK import workflow.
        let tracking_panel = container(column![
            text("Tracking Configuration & Controls"),
            text("Model: Joint 6-DOF rigid body"),
            text("1. OPTIONAL: with magnets removed and boards fixed, capture 3 s of background. No background = lower-confidence fitted board offsets."),
            if context.gk_calibration.capturing.is_none() {
                button("Capture no-magnet background").on_press(Message::BeginGkBackground)
            } else { button("Capture no-magnet background") },
            text(if context.gk_calibration.background.is_some() { "Background: 48 sensors captured" }
                else { "Background: skipped / not captured (allowed)" }),
            text("2. Install magnets; import/review CT. A chip-first receiver without visible PCB outline may require a reviewed KiCad/JST mounting prior."),
            text(if context.cbct_geometry.is_some() { "CT: imported for review" } else { "CT: not imported" }),
            if context.cbct_geometry.is_some() {
                button("1. Preview CAD-assisted CT-to-firmware mapping")
                    .on_press(Message::ReviewCadGkRegistration)
            } else { button("1. Preview CAD-assisted CT-to-firmware mapping") },
            text(context.gk_calibration.cad_review_preview.as_deref()
                .unwrap_or("CAD mapping not reviewed. The default measured-PCB registration remains unchanged.")),
            text("CHECK before approval: raw0=TOP, raw1=PATIENT LEFT, raw2=PATIENT RIGHT; 16 chips per board; current PCB revision; chip-facing sides; top JST points PATIENT RIGHT, side JSTs point INFERIOR. CAD connector locations have NOT been measured in CT."),
            if context.gk_calibration.cad_review_preview.is_some()
                    && !context.gk_calibration.cad_mapping_approved {
                button("2. Approve reviewed CAD/JST mapping for experimental calibration")
                    .on_press(Message::ApproveCadGkRegistration)
            } else { button("2. Approve reviewed CAD/JST mapping for experimental calibration") },
            if context.gk_calibration.cad_mapping_approved {
                button("Revoke CAD/JST mapping approval").on_press(Message::RevokeCadGkRegistration)
            } else { button("Revoke CAD/JST mapping approval") },
            text(if context.gk_calibration.cad_mapping_approved {
                "CAD/JST MAPPING APPROVED for experimental calibration only — NOT validated magnetic tracking."
            } else { "CAD/JST MAPPING NOT APPROVED. No CAD-only geometry may enter calibration." }),
            text("3. Capture 3 s of stationary Hall fields with or WITHOUT CT import; archive with corresponding raw logs."),
            if context.gk_calibration.capturing.is_none() {
                button("Capture stationary reference").on_press(Message::BeginGkReference)
            } else { button("Capture stationary reference") },
            text(if context.gk_calibration.reference.is_some() { "Reference fields: 48 sensors captured" }
                else { "Reference fields: not captured" }),
            text("4. Calibrate signed magnet strengths and validate the field residuals before tracking."),
            if context.gk_calibration.reference.is_some() {
                button("Calibrate tracking model").on_press(Message::CalibrateGkTracking)
            } else { button("Calibrate tracking model") },
            text(if context.gk_calibration.status.is_empty() {
                "Calibration: not started".to_string()
            } else { context.gk_calibration.status.clone() }),
            button("Copy calibration diagnostics").on_press(Message::CopyGkCalibrationStatus),
            text("CT registration: v2 JST/PCB-centroid geometry candidate, using the established A1 Hall axis convention. Inspect the board orientation and magnet dimensions before enabling."),
            if context.gk_calibration.calibrated_candidate.is_some() {
                button("Enable EXPERIMENTAL 6-DOF tracking (review first)").on_press(Message::EnableExperimentalGkTracking)
            } else { button("Enable EXPERIMENTAL 6-DOF tracking (review first)") },
            if context.gk_calibration.reference.is_some() {
                button("Save calibration inputs (.json)").on_press(Message::ExportGkCalibration)
            } else { button("Save calibration inputs (.json)") },
            text("CT and background are optional for raw logging/reference export. Calibration requires CT; without measured background it fits board-constant offsets only. Retain .bin and DICOM logs."),
            // Retain the legacy profile importer under an explicit archival label.
            button("Legacy: load manually calibrated JSON").on_press(Message::LoadRigidProfile),
            text(profile_status),
            row![
                if rigid.result.is_some() {
                    button("Set rigid zero").on_press(Message::ResetRigidZero)
                } else {
                    button("Set rigid zero")
                },
                text(if rigid.zero_ready { "Reference: set" } else { "Reference: not set" }),
            ].spacing(8),
            text(position_line),
            text(rotation_line),
            text(format!("Status: {}", rigid.status)),
            text(fit_quality),
        ].spacing(6))
        .padding(12)
        .width(Length::Fill)
        .style(iced::widget::container::bordered_box);

        let mut rigid_content = column![dicom_panel, tracking_panel].spacing(12);
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

#[cfg(test)]
mod tracking_mode_ui_tests {
    use super::*;

    #[test]
    fn algorithm_selection_is_independent_of_the_three_plot_tabs() {
        let mut context = Context::default();
        assert_eq!(context.dashboard_tab, DashboardTab::LiveTracking);
        assert_eq!(context.tracking_mode, TrackingMode::Rigid6Dof);
        assert!(!context.live_fits.fit_enabled());

        let _ = update(&mut context, Message::SelectTrackingMode(TrackingMode::Independent));
        assert_eq!(context.tracking_mode, TrackingMode::Independent);
        assert!(context.live_fits.fit_enabled());
        // No background capture is required to activate independent fitting.
        assert!(!context.live_fits.board_summaries().iter().any(|b| b.has_background));

        let _ = update(&mut context, Message::SelectDashboardTab(DashboardTab::SensorTraces));
        assert!(context.live_fits.fit_enabled());
        let _ = update(&mut context, Message::SelectDashboardTab(DashboardTab::Both));
        assert_eq!(context.tracking_mode, TrackingMode::Independent);
        assert!(context.live_fits.fit_enabled());

        let _ = update(&mut context, Message::SelectTrackingMode(TrackingMode::Rigid6Dof));
        assert!(!context.live_fits.fit_enabled());
        assert_eq!(context.dashboard_tab, DashboardTab::Both);
    }
}

pub fn main() -> iced::Result {
    iced::application(Context::default, update, view)
        .subscription(|context: &Context| {
            if context.cbct_importing {
                iced::time::every(std::time::Duration::from_millis(250)).map(|_| Message::CbctProgressTick)
            } else { iced::Subscription::none() }
        })
        .scale_factor(ui_scale_factor)
        .run()
}

#[cfg(test)]
mod logging_safety_tests {
    use super::*;
    use std::fs;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn fresh_test_path() -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH)
            .expect("System clock before Unix epoch").as_nanos();
        std::env::temp_dir().join(format!(
            "mlx_logging_protection_{}_{}.bin", std::process::id(), nanos
        ))
    }

    #[test]
    fn selection_alone_does_not_create_a_file_and_start_is_explicit() {
        let path=fresh_test_path();
        let mut context=Context::default();
        // The Select File dialog only chooses this path; it must not create
        // an open writer or write a header until Start Logging is requested.
        context.log_file_path=Some(path.clone());
        assert!(!path.exists());
        assert!(context.file_writer.is_none());
        assert!(log_status_text(&context).contains("NOT recording"));

        let _=update(&mut context,Message::StartLogging);
        assert!(context.file_writer.is_some());
        assert_eq!(context.logged_bytes,8);
        assert!(!context.log_resume_available);

        // Clicking Start again must not replace or reset an open recording.
        let _=update(&mut context,Message::StartLogging);
        assert_eq!(context.logged_bytes,8);
        context.file_writer=None;
        assert_eq!(fs::read(&path).unwrap(),b"MLXBF002");
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn append_only_resume_requires_a_completed_prior_log() {
        let path=fresh_test_path();
        let mut context=Context::default();
        context.log_file_path=Some(path.clone());
        // A selected-but-not-yet-created log cannot be resumed.
        let _=update(&mut context,Message::ResumeLogging);
        assert!(context.file_writer.is_none());
        assert!(!path.exists());

        let mut file=create_new_log(&path).unwrap();
        use std::io::Write;
        // Valid header followed by a complete length-prefixed frame.
        file.write_all(&(4_u32).to_le_bytes()).unwrap();
        file.write_all(b"data").unwrap();
        file.flush().unwrap();
        drop(file);
        let before=fs::read(&path).unwrap();

        context.log_resume_available=true;
        let _=update(&mut context,Message::ResumeLogging);
        assert!(context.file_writer.is_some());
        assert_eq!(context.logged_bytes,before.len() as u64);
        context.file_writer=None;
        assert_eq!(fs::read(&path).unwrap(),before);
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn existing_log_is_preserved_byte_for_byte() {
        let path = fresh_test_path();
        let original = b"MLXBF002 original experiment data";
        fs::write(&path, original).expect("Create pre-existing log");
        let err = create_new_log(&path).expect_err("Refuse to overwrite existing log");
        assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists);
        assert_eq!(fs::read(&path).unwrap(), original);
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn new_log_has_magic_and_cannot_be_reopened() {
        let path = fresh_test_path();
        let writer = create_new_log(&path).expect("Create new log");
        drop(writer);
        assert_eq!(fs::read(&path).unwrap(), b"MLXBF002");
        let err = create_new_log(&path).expect_err("Refuse repeated file selection");
        assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists);
        assert_eq!(fs::read(&path).unwrap(), b"MLXBF002");
        fs::remove_file(path).unwrap();
    }
}
