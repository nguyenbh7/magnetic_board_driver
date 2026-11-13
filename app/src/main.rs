

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
use displacement_plot::{displacement_plot, rotation_plot, hdmm_prediction_plot};
mod rigid_xyz_plot;
use rigid_xyz_plot::rigid_xyz_plot;
mod sensor_trace;
mod sensor_trace_plot;
mod offline_replay;
use offline_replay::{OfflineRecording, OfflineReplay, ReplaySpeed};
use sensor_trace::SensorTraceState;
use sensor_trace_plot::{sensor_trace_plot, TraceRangeGuides};
mod live_fit;
mod rigid_fit;
mod cbct_geometry;
mod hdmm_sticker;
mod hdmm_live_readout;
mod hdmm_ct_workflow;
use hdmm_ct_workflow::{HdmmProposal, HdmmApprovedFile};
mod gk_background;
use gk_background::GkBackgroundFile;
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
    conversions::{MagneticField, MagneticValue, TempValue, nominal_symmetric_mlx_half_ranges_mt},
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
    LoadOfflineReplay,
    ReplayFilePicked(Option<PathBuf>),
    ReplayLoaded(Result<Arc<OfflineRecording>, String>),
    ReplayPlay,
    ReplayPause,
    ReplayRestart,
    ReplayUnload,
    ReplaySpeedSelected(ReplaySpeed),
    ReplayTick,
    FitCompleted(FitCompletion),
    RigidFitCompleted(RigidFitCompletion),
    SelectMagnetLocations(MagnetLocationPreset),
    LoadRigidProfile,
    RigidProfilePicked(Option<PathBuf>),
    SelectCbctDetectorVariant(cbct_geometry::CbctDetectorVariant),
    CheckCbctSetup,
    CbctSetupChecked(Result<String,String>),
    CopyCbctSetupInstructions,
    CopyCbctSetupResults,
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
    ToggleCadMappingDetails,
    CopyCadMappingDetails,
    CaptureRigidBackground,
    BeginGkBackground,
    ExportGkBackground,
    GkBackgroundExportPicked(Option<PathBuf>),
    ImportGkBackground,
    GkBackgroundImportPicked(Option<PathBuf>),
    ConfirmImportedGkBackground,
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
    FindHdmmCtCandidates,
    HdmmCtCandidatesFound(Result<HdmmProposal, String>),
    SelectHdmmCtCandidate(String),
    UpdateHdmmReviewNote(String),
    ApproveHdmmCtCandidate,
    HdmmCtCandidateApproved(Result<HdmmApprovedFile, String>),
    OpenHdmmCtPreview,
    CopyHdmmCtReportPath,
    UpdateHdmmMarkerCoordinates(String),
    ImportReviewedHdmmSticker,
    ReviewedHdmmStickerFilePicked(Option<PathBuf>),
    ApplyHdmmMarkerCoordinates,
    ClearHdmmMarkerCoordinates,
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
    SetSensorComponentFullScale(String),

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
    SelectWideMlxRangePreset,
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
    /// Loaded files are NOT trusted for this setup until explicit confirmation.
    pending_background: Option<[[[f64; 3]; 16]; 3]>,
    reference: Option<[[[f64; 3]; 16]; 3]>,
    capturing: Option<GkCapture>,
    calibrated_candidate: Option<(RigidProfile, f64)>,
    /// A human must inspect candidate mapping and then separately approve
    /// the specific currently loaded CT. No persistence across CT/port reload.
    cad_review_preview: Option<String>,
    cad_review_summary: Option<String>,
    cad_review_details_expanded: bool,
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
        else { self.background = None; self.pending_background=None; self.reference = None; }
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
    offline_replay: Option<OfflineReplay>,
    replay_loading: bool,
    replay_notice: String,
    /// Requires an operator-verified range for the current MLX gain/res config.
    sensor_component_full_scale_mt: String,
    dashboard_tab: DashboardTab,
    tracking_mode: TrackingMode,
    board_presence: BoardPresence,
    live_fits: BoardLiveFits,
    rigid_fit: RigidLiveFit,
    cbct_geometry: Option<cbct_geometry::CbctGeometry>,
    cbct_detector_variant: cbct_geometry::CbctDetectorVariant,
    cbct_setup_status: String,
    cbct_setup_checking: bool,
    hdmm_marker_coordinate_input: String,
    hdmm_sticker_source_status: String,
    hdmm_ct_proposal: Option<HdmmProposal>,
    hdmm_ct_searching: bool,
    hdmm_ct_approving: bool,
    hdmm_ct_candidate_choice: Option<String>,
    hdmm_ct_review_note: String,
    hdmm_ct_status: String,
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

impl Context {
    /// A candidate/approval from an earlier CT must not be reused after a new
    /// scan or a different CBCT detector is selected.
    fn clear_hdmm_ct_review(&mut self) {
        self.hdmm_ct_proposal = None;
        self.hdmm_ct_searching = false;
        self.hdmm_ct_approving = false;
        self.hdmm_ct_candidate_choice = None;
        self.hdmm_ct_review_note.clear();
        self.hdmm_ct_status.clear();
    }
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

/// One shared source of axis-specific limits for the live warning AND the
/// dotted Hall trace guides. Editing an un-applied gain/resolution control
/// MUST NOT enlarge the limits used during an existing stream.
#[derive(Debug, Clone, Copy)]
struct LiveHallRangeModel {
    digital_mt: [f64; 3],
    advisory_mt: [f64; 3],
    provenance: &'static str,
}

fn live_hall_range_model(context: &Context) -> LiveHallRangeModel {
    let configured = context.mlx_status.as_ref().filter(|s| s.ok)
        .and_then(|s| nominal_symmetric_mlx_half_ranges_mt(
            s.gain, s.resolution, s.hall_conf));
    // Explicitly conservative fallback, never an assertion that any chip
    // is running the hardware-reset gain7/res0/four-phase configuration.
    let digital_mt = configured.unwrap_or([4.9152, 4.9152, 7.929856]);
    // The specific IMC option code is not verified. Use a provisional 25mT
    // analog guard against showing the entire digital range as safe.
    let advisory_mt = digital_mt.map(|digital| digital.min(25.0));
    LiveHallRangeModel {
        digital_mt,
        advisory_mt,
        provenance: if configured.is_some() {
            "CACHED MLX setting (NOT verified 48-chip registers)"
        } else {
            "UNVERIFIED conservative gain7/res0 reference; NOT actual chip settings"
        },
    }
}

fn live_hall_trace_guides(context: &Context) -> TraceRangeGuides {
    TraceRangeGuides {
        advisory_mt: live_hall_range_model(context).advisory_mt,
    }
}

fn live_hall_trace_limit_caption(context: &Context) -> String {
    let range = live_hall_range_model(context);
    format!(
        "DOTTED +/- advisory guides: Bx {:.2}, By {:.2}, Bz {:.2} mT; nominal digital +/- {:.2}/{:.2}/{:.2} mT. {}. Equal Bx/By guides overlap. Guides change only after a successful firmware configuration response, not by editing unapplied settings. Neither line is a verified analog saturation limit.",
        range.advisory_mt[0], range.advisory_mt[1], range.advisory_mt[2],
        range.digital_mt[0], range.digital_mt[1], range.digital_mt[2],
        range.provenance,
    )
}

/// Safety-oriented readout warning on the CURRENT frame of EVERY sensor,
/// not just the manually selected trace. No claim about actual saturation:
/// the GetMlxSensitivity RPC exposes a cached configuration, not 48-chip
/// acquisition-time register readbacks.
fn live_hall_range_status(context: &Context) -> String {
    let range = live_hall_range_model(context);
    let limits = range.digital_mt;
    let advisory_limits = range.advisory_mt;
    let provenance = range.provenance;
    let mut worst: Option<(f64, u16, u8, usize, f64)> = None;
    let mut invalid = 0u64;
    let mut jump_count = 0u64;
    let mut last_jump: Option<(u16, u8, usize, f64)> = None;
    for board in context.sensor_traces.board_summaries() {
        invalid += board.invalid_samples;
        jump_count = jump_count.saturating_add(board.sudden_jump_count);
        if let Some((sensor, axis, magnitude)) = board.last_sudden_jump {
            last_jump = Some((board.board_id, sensor, axis, magnitude));
        }
        for (axis, peak) in board.latest_axis_peaks_mt.iter().enumerate() {
            if let Some((sensor, magnitude)) = peak {
                let fraction = magnitude / advisory_limits[axis];
                if worst.as_ref().is_none_or(|old| fraction > old.0) {
                    worst = Some((fraction, board.board_id, *sensor, axis, *magnitude));
                }
            }
        }
    }
    let source = format!(
        "Digital X/Y/Z +/-{:.2}/{:.2}/{:.2} mT; advisory X/Y/Z +/-{:.2}/{:.2}/{:.2} mT (25mT analog guard until option code verified; {provenance})",
        limits[0], limits[1], limits[2],
        advisory_limits[0], advisory_limits[1], advisory_limits[2]);
    let jump_warning = if let Some((board, sensor, axis, magnitude)) = last_jump {
        format!(
            "RAW STEP WARNING: {jump_count} >8mT sudden step(s) this stream; "
        ) + &format!(
            "most recent board{board} sensor{sensor} {} {:.2}mT. ",
            ["Bx","By","Bz"][axis], magnitude
        )
    } else {
        String::new()
    };
    match worst {
        Some((ratio, board, sensor, axis, measured)) => {
            let axis_name = ["Bx", "By", "Bz"][axis];
            let action = if ratio >= 0.80 || jump_count > 0 {
                "BACK OFF MAGNET / REVIEW DATA - range or raw step warning"
            } else if ratio >= 0.65 {
                "CAUTION - approaching readout range"
            } else {
                "Below early warning margin (NOT proof against wrapping)"
            };
            format!(
                "{action} | {jump_warning}raw board{board} sensor{sensor} {axis_name} latest |B|={measured:.3} mT / advisory {:.3} mT ({:.0}%) | {source}{}",
                advisory_limits[axis], ratio * 100.,
                if invalid > 0 { " | INVALID OR MISSING TRACE SAMPLES" } else { "" }
            )
        }
        None => format!("Hall range: awaiting all-sensor trace samples. {jump_warning}{source}"),
    }
}

/// Conservative fail-closed screen applied after ALL raw sensors in each
/// board frame have been updated, before any new 6-DOF job is dispatched.
/// The 80% limit is *advisory*, not proof of hardware saturation. Jump events
/// are independent of magnitude and remain latched for the recording.
fn critical_rigid_input_reason(context: &Context) -> Option<String> {
    let limits = live_hall_range_model(context);
    let issue = context.sensor_traces.critical_input(limits.advisory_mt, 0.80)?;
    Some(match issue {
        sensor_trace::CriticalRawInput::SuddenStep {
            board_id, sensor, axis, delta_mt,
        } => format!(
            "raw Board {board_id} sensor {sensor} {} sudden {delta_mt:.2} mT step (>8 mT, 0.25 s); digital wrapping POSSIBLE, NOT PROVEN",
            ["Bx", "By", "Bz"][axis],
        ),
        sensor_trace::CriticalRawInput::AdvisoryRange {
            board_id, sensor, axis, magnitude_mt, advisory_limit_mt,
        } => format!(
            "raw Board {board_id} sensor {sensor} {} |B|={magnitude_mt:.3} mT reaches {:.0}% of {advisory_limit_mt:.3} mT advisory limit ({}); overrange POSSIBLE, NOT PROVEN",
            ["Bx", "By", "Bz"][axis],
            magnitude_mt / advisory_limit_mt * 100.0,
            limits.provenance,
        ),
    })
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

fn save_gk_background_file(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Export NEW no-magnet GK background (48 raw sensor vectors)")
        .set_file_name("gk_raw_hall_background.json")
        .add_filter("JSON", &["json"])
        .set_parent(&window);
    async move { dialog.save_file().await.map(|handle|handle.path().to_path_buf()) }
}

fn import_gk_background_file(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Import previously saved no-magnet GK background for the SAME setup")
        .add_filter("JSON", &["json"])
        .set_parent(&window);
    async move { dialog.pick_file().await.map(|handle|handle.path().to_path_buf()) }
}

fn pick_reviewed_hdmm_sticker_file(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Import reviewed HDMM CT marker location (NOT raw bright-object candidates)")
        .add_filter("HDMM sticker geometry JSON", &["json"])
        .set_parent(&window);
    async move {
        dialog.pick_file().await.map(|handle| handle.path().to_path_buf())
    }
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

fn pick_offline_replay_file(
    window: &dyn iced::Window,
) -> impl Future<Output = Option<PathBuf>> + use<> {
    let dialog = rfd::AsyncFileDialog::new()
        .set_title("Open READ-ONLY MLXBF002 board-frame recording")
        .add_filter("MLXBF002 recording", &["bin"])
        .set_parent(&window);
    async move { dialog.pick_file().await.map(|handle| handle.path().to_path_buf()) }
}

async fn load_offline_replay_file(
    path: PathBuf,
) -> Result<Arc<OfflineRecording>, String> {
    async_std::task::spawn_blocking(move || OfflineRecording::load(&path)).await
}

async fn run_cbct_import_off_thread(
    path: PathBuf,
    variant: cbct_geometry::CbctDetectorVariant,
    progress: Arc<std::sync::Mutex<cbct_geometry::ImportProgress>>,
) -> Result<cbct_geometry::CbctGeometry, String> {
    // Snapshot method BEFORE launch. Hybrid algorithm is maintained Python code,
    // not an external notebook selected at run time.
    async_std::task::spawn_blocking(move || {
        cbct_geometry::import_dicom_with_variant(&path,variant,progress)
    }).await
}

async fn find_hdmm_ct_candidates_off_thread(
    geometry: cbct_geometry::CbctGeometry,
) -> Result<HdmmProposal, String> {
    async_std::task::spawn_blocking(move || hdmm_ct_workflow::find_candidates(geometry)).await
}

async fn approve_hdmm_ct_candidate_off_thread(
    proposal: HdmmProposal,
    candidate_id: String,
    note: String,
) -> Result<HdmmApprovedFile, String> {
    async_std::task::spawn_blocking(move ||
        hdmm_ct_workflow::approve_candidate(proposal, candidate_id, note)
    ).await
}

async fn run_cbct_dependency_check_off_thread() -> Result<String,String> {
    async_std::task::spawn_blocking(cbct_geometry::inspect_dicom_environment).await
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

/// One shared path for frames from hardware and read-only offline replay.
/// Replay never drives the logging writer or pretends to be a live stream.
fn ingest_board_frame(
    context: &mut Context, frame: BoardFrame, from_replay: bool,
) -> Task<Message> {
    context.received_stream_frames =
        context.received_stream_frames.saturating_add(1);
    if !from_replay {
        context.stream_status = "receiving".to_string();
    }
    let mut fit_job = None;
    for sensor_index in 0..frame.samples.len() {
        let Some(field) = compact_sample_to_sensor_field(&frame, sensor_index)
        else { continue };
        if let Some(job) = context.live_fits.update(field.clone()) {
            fit_job = Some(job);
        }
        context.sensor_traces.update(&field);
        context.ping_field = Some(field);
    }
    // Protect the *pose*, not merely the warning banner. A faulty raw
    // channel anywhere on any available receiver invalidates the current
    // synchronized input set. The raw trace and optional original-source
    // logging continue for forensic review. No unwrapping is attempted.
    if context.rigid_fit.input_unreliable().is_none() {
        if let Some(reason) = critical_rigid_input_reason(context) {
            context.rigid_fit.block_unreliable_input(reason.clone());
            context.live_fits.set_fit_enabled(false);
            // Any in-progress GK calibration capture is also compromised.
            // Do not let range-crossed data become a new calibration reference.
            if context.gk_calibration.capturing.take().is_some() {
                context.gk_calibration.status = format!(
                    "CALIBRATION CAPTURE ABORTED: unreliable Hall input ({reason})"
                );
            }
        }
    }
    // A recording alone is not a new measurement/background reference.
    // Keep experimental calibration commands exclusively on live hardware.
    if !from_replay { context.gk_calibration.accept_frame(&frame); }
    let mut tasks = Vec::new();
    if let Some(job) = context.rigid_fit.receive_frame(
        &frame, context.tracking_mode == TrackingMode::Rigid6Dof,
    ) {
        tasks.push(Task::perform(run_rigid_fit_job(job), Message::RigidFitCompleted));
    }
    if context.rigid_fit.input_unreliable().is_none() {
        if let Some(job) = fit_job {
            tasks.push(Task::perform(run_fit_job(job), Message::FitCompleted));
        }
    }
    if !from_replay && context.file_writer.is_some() && !context.log_stop_requested {
        context.log_queue.push_back(frame.clone());
        if let Some(task) = next_log_write_task(context) { tasks.push(task); }
    }
    Task::batch(tasks)
}

fn update(context: &mut Context, message: Message) -> Task<Message> {

    //println!("{:#?}", message);
    match message {
        Message::LoadOfflineReplay => {
            if context.offline_replay.is_some() || context.replay_loading
                || (context.stream_status != "idle" && !context.stream_status.is_empty())
                || context.file_writer.is_some() || context.log_stop_requested
                || context.log_write_in_progress || context.log_finalize_in_progress
                || context.log_file_dialog_open
            {
                context.replay_notice = "Stop the hardware stream, finish/close logging, and unload any existing replay before opening a recording.".into();
                return Task::none();
            }
            context.replay_loading = true;
            context.replay_notice = "Selecting an existing MLXBF002 recording for READ-ONLY replay".into();
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, pick_offline_replay_file))
                .then(Task::future)
                .map(Message::ReplayFilePicked)
        }
        Message::ReplayFilePicked(picked) => {
            if let Some(path) = picked {
                context.replay_notice = format!("Reading and validating: {}", path.display());
                Task::perform(load_offline_replay_file(path), Message::ReplayLoaded)
            } else {
                context.replay_loading = false;
                context.replay_notice = "Replay selection cancelled; no data modified.".into();
                Task::none()
            }
        }
        Message::ReplayLoaded(result) => {
            context.replay_loading = false;
            match result {
                Ok(recording) => {
                    let presence = recording.presence();
                    context.replay_notice = format!(
                        "OFFLINE REPLAY (READ ONLY): {} frames across {:.2}s; {} out-of-order board timestamps. The raw log does NOT contain per-chip gain/resolution or CT calibration. Any prior live calibration is cleared: explicitly load a verified matching rigid profile for optional motion fitting.",
                        recording.total_frames(), recording.duration_s(),
                        recording.reordered_timestamps(),
                    );
                    context.offline_replay = Some(OfflineReplay::new(recording));
                    context.sensor_traces = SensorTraceState::default();
                    context.board_presence = presence.clone();
                    context.sensor_traces.set_presence(presence.clone());
                    context.live_fits = BoardLiveFits::default();
                    context.live_fits.set_fit_enabled(context.tracking_mode == TrackingMode::Independent);
                    context.live_fits.set_presence(presence.clone());
                    // A live-session profile/HDMM target must never silently
                    // carry over to a historical log with unrelated geometry.
                    // A matching rigid profile may be loaded explicitly AFTER
                    // replay opens; raw field traces work without a profile.
                    context.rigid_fit.invalidate_for_gk_calibration();
                    context.rigid_fit.reset_input_guard_for_new_acquisition();
                    context.rigid_fit.set_presence(presence);
                    context.gk_calibration = GkCalibration::default();
                    context.cbct_geometry = None;
                    context.hdmm_marker_coordinate_input.clear();
                    context.hdmm_sticker_source_status.clear();
                    context.cbct_status = "Old live CBCT setup cleared when offline recording was loaded; explicitly import matching provenance for further analysis.".into();
                    context.mlx_status = None; // Historical log lacks true register settings.
                    context.sensor_component_full_scale_mt.clear();
                    context.received_stream_frames = 0;
                    context.dashboard_tab = DashboardTab::LiveTracking;
                    context.stream_status = "idle".into();
                }
                Err(err) => {
                    context.replay_notice = format!("REPLAY LOAD FAILED: {err}; original file unchanged.");
                }
            }
            Task::none()
        }
        Message::ReplayPlay => {
            if let Some(replay) = &mut context.offline_replay {
                replay.play();
            }
            Task::none()
        }
        Message::ReplayPause => {
            if let Some(replay) = &mut context.offline_replay {
                replay.pause();
                // Retain plotted history and invalidate in-flight fits so a
                // delayed completion cannot arrive while replay is paused.
                context.rigid_fit.stop_stream();
            }
            Task::none()
        }
        Message::ReplayRestart => {
            if let Some(replay) = &mut context.offline_replay {
                replay.restart();
                context.sensor_traces.reset_histories();
                context.live_fits = BoardLiveFits::default();
                context.live_fits.set_fit_enabled(context.tracking_mode == TrackingMode::Independent);
                context.live_fits.set_presence(context.board_presence.clone());
                context.rigid_fit.reset_input_guard_for_new_acquisition();
                context.received_stream_frames = 0;
                context.replay_notice = "Replay reset to first frame. Press Play; no original file was changed.".into();
            }
            Task::none()
        }
        Message::ReplayUnload => {
            if context.offline_replay.take().is_some() {
                let none = BoardPresence::default();
                context.rigid_fit.set_presence(none.clone());
                context.sensor_traces = SensorTraceState::default();
                context.live_fits = BoardLiveFits::default();
                context.board_presence = none;
                context.mlx_status = None;
                context.ping_field = None;
                context.stream_status = "idle".into();
                context.received_stream_frames = 0;
                context.replay_notice = "Offline recording unloaded. Live hardware may be selected; a new stream must re-detect its boards.".into();
            }
            Task::none()
        }
        Message::ReplaySpeedSelected(speed) => {
            if let Some(replay) = &mut context.offline_replay {
                replay.speed = speed;
            }
            Task::none()
        }
        Message::ReplayTick => {
            let frames = context.offline_replay.as_mut()
                .map(|replay| replay.tick()).unwrap_or_default();
            let mut tasks = Vec::new();
            for frame in frames {
                tasks.push(ingest_board_frame(context, frame, true));
            }
            if context.offline_replay.as_ref().is_some_and(|replay| replay.finished()) {
                // Do not discard a solver job still completing from the last
                // replay frames; results are visibly research-only.
                context.replay_notice = "OFFLINE REPLAY COMPLETE. Source unchanged; last accepted rigid fit (if a matching profile was explicitly loaded) may still be finishing.".into();
            }
            Task::batch(tasks)
        }
        Message::PortSelected(serial_port_info) => {
            if context.offline_replay.is_some() || context.replay_loading {
                context.replay_notice = "Unload offline replay before switching to a live controller.".into();
                return Task::none();
            }
            context.sensor_watcher = Some(SensorWatcher::new(&serial_port_info));
            context.board_presence = BoardPresence::default();
            context.live_fits = BoardLiveFits::default();
            context.rigid_fit.clear_background_for_new_port();
            context.rigid_fit.invalidate_for_gk_calibration();
            context.rigid_fit.reset_input_guard_for_new_acquisition();
            context.gk_calibration = GkCalibration::default();
            context.sensor_traces = SensorTraceState::default();
            context.sensor_component_full_scale_mt.clear();
            context.mlx_status = None; // old port's cached settings are not this device's
            context.hdmm_marker_coordinate_input.clear();
            context.hdmm_sticker_source_status.clear();
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
            if context.offline_replay.is_some() { return Task::none(); }
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
        Message::SelectCbctDetectorVariant(variant) => {
            if context.cbct_importing { return Task::none(); }
            if context.cbct_detector_variant != variant {
                context.cbct_detector_variant = variant;
                // A different detector is a different unreviewed geometry source.
                // Never carry forward a prior CT-to-firmware approval or fit.
                context.cbct_geometry = None;
                context.gk_calibration.calibrated_candidate = None;
                context.gk_calibration.cad_review_preview = None;
                context.gk_calibration.cad_review_summary = None;
                context.gk_calibration.cad_review_details_expanded = false;
                context.gk_calibration.cad_mapping_approved = false;
                context.rigid_fit.invalidate_for_gk_calibration();
                context.hdmm_marker_coordinate_input.clear();
                context.hdmm_sticker_source_status.clear();
                context.clear_hdmm_ct_review();
                context.cbct_status = format!(
                    "Selected {}. Re-import CT; previous geometry and mapping approval cleared.",
                    variant
                );
                context.cbct_last_log.clear();
            }
            Task::none()
        }
        Message::CheckCbctSetup => {
            if context.cbct_setup_checking || context.cbct_importing {return Task::none();}
            context.cbct_setup_checking=true;
            context.cbct_setup_status="Checking the bundled DICOM script, Python environment and packages (no installation)...".into();
            Task::perform(run_cbct_dependency_check_off_thread(),Message::CbctSetupChecked)
        }
        Message::CbctSetupChecked(result) => {
            context.cbct_setup_checking=false;
            context.cbct_setup_status=match result {
                Ok(summary)=>format!("DICOM environment ready for Original/Optimized.\n{summary}"),
                Err(error)=>format!("DICOM setup needs attention:\n{error}"),
            };
            Task::none()
        }
        Message::CopyCbctSetupInstructions => {
            iced::clipboard::write(cbct_geometry::dicom_setup_instructions())
        }
        Message::CopyCbctSetupResults => {
            iced::clipboard::write(context.cbct_setup_status.clone())
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
                let variant=context.cbct_detector_variant;
                context.cbct_status = format!(
                    "{}: detecting magnets, 48 chips, and glass BB from {} (background task)...",
                    variant,path.display()
                );
                context.cbct_geometry = None;
                context.gk_calibration.calibrated_candidate = None;
                context.gk_calibration.cad_review_preview = None;
                context.gk_calibration.cad_review_summary = None;
                context.gk_calibration.cad_review_details_expanded = false;
                context.gk_calibration.cad_mapping_approved = false;
                context.rigid_fit.invalidate_for_gk_calibration();
                context.hdmm_marker_coordinate_input.clear();
                context.hdmm_sticker_source_status.clear();
                context.clear_hdmm_ct_review();
                context.cbct_last_log.clear();
                context.cbct_show_details = false;
                let progress = Arc::new(std::sync::Mutex::new(cbct_geometry::ImportProgress::default()));
                context.cbct_progress = Some(Arc::clone(&progress));
                context.cbct_started = Some(std::time::Instant::now());
                Task::perform(
                    run_cbct_import_off_thread(path,variant,progress),
                    Message::CbctImportCompleted,
                )
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
                        context.cbct_status = "CT initial geometry imported for review. HDMM sticker NOT SEARCHED; reimport the source ZIP/folder to find sticker candidates. Live magnetic profile unchanged.".into();
                        context.cbct_geometry = Some(geometry);
                    // Preserve previously captured raw reference/background
                    // when DICOM is processed after acquisition. Review scan
                    // correspondence before enabling experimental tracking.
                    context.gk_calibration.calibrated_candidate = None;
                    context.gk_calibration.cad_review_preview = None;
                    context.gk_calibration.cad_review_summary = None;
                    context.gk_calibration.cad_review_details_expanded = false;
                    context.gk_calibration.cad_mapping_approved = false;
                    context.rigid_fit.invalidate_for_gk_calibration();
                    context.hdmm_marker_coordinate_input.clear();
                    context.hdmm_sticker_source_status.clear();
                    context.clear_hdmm_ct_review();
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
                    context.clear_hdmm_ct_review();
                    context.cbct_status = if geometry.markers.is_empty() {
                        format!("{}: 3 magnets and 48 chips detected; NO glass BB. Geometry ready for review. Optical HDMM sticker NOT SEARCHED; use Find HDMM candidates.",context.cbct_detector_variant)
                    } else {
                        format!("{}: initial geometry ready for review. Optical HDMM sticker NOT SEARCHED; use Find HDMM candidates. No magnetic calibration activated.",context.cbct_detector_variant)
                    };
                    context.cbct_geometry = Some(geometry);
                context.gk_calibration.calibrated_candidate = None;
                context.gk_calibration.cad_review_preview = None;
                context.gk_calibration.cad_review_summary = None;
                context.gk_calibration.cad_review_details_expanded = false;
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
            iced::clipboard::write(format!("DICOM import status: {}\nSelected import method: {}\nCT registration: {}",
                context.cbct_status,
                context.cbct_detector_variant,
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
                result.push_str(&format!("CT source: {}\nCT-only registration (no CAD):\n{}\n",
                    g.scan_path.display(),g.registration_preview_v2()));
            }
            result.push_str(&format!(
                "CAD-assisted mapping summary:\n{}\nApproved for experimental calibration only: {}\nFull sensor mapping and matrices: use Copy mapping.\n",
                context.gk_calibration.cad_review_summary.as_deref()
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
            if context.offline_replay.is_some() {
                context.replay_notice = "No-magnet background capture requires live hardware, not an old recording.".into();
                return Task::none();
            }
            context.rigid_fit.invalidate_for_gk_calibration();
            context.hdmm_marker_coordinate_input.clear();
            context.hdmm_sticker_source_status.clear();
            context.gk_calibration.begin(false);
            Task::none()
        }
        Message::ExportGkBackground => {
            if context.gk_calibration.background.is_none() {
                context.gk_calibration.status = "Capture a no-magnet background before exporting it.".into();
                return Task::none();
            }
            iced::window::oldest()
                .and_then(|id| iced::window::run(id,save_gk_background_file))
                .then(Task::future)
                .map(Message::GkBackgroundExportPicked)
        }
        Message::GkBackgroundExportPicked(path) => {
            if let Some(path)=path {
                let saved = context.gk_calibration.background
                    .ok_or_else(||"No GK no-magnet background was captured".to_string())
                    .and_then(GkBackgroundFile::from_raw)
                    .and_then(|file|file.to_bytes())
                    .and_then(|bytes|{
                        use std::io::Write as _;
                        let mut writer=std::fs::OpenOptions::new()
                            .write(true).create_new(true).open(&path)
                            .map_err(|e|format!("Cannot create NEW file: {e}"))?;
                        writer.write_all(&bytes).map_err(|e|e.to_string())
                    });
                context.gk_calibration.status=match saved {
                    Ok(())=>format!("Standalone 48-sensor no-magnet background exported: {}",path.display()),
                    Err(e)=>format!("Background export failed: {e}"),
                };
            }
            Task::none()
        }
        Message::ImportGkBackground => {
            if context.gk_calibration.capturing.is_some() {
                context.gk_calibration.status="Finish the current 3-second Hall capture before importing a background".into();
                return Task::none();
            }
            iced::window::oldest()
                .and_then(|id| iced::window::run(id,import_gk_background_file))
                .then(Task::future)
                .map(Message::GkBackgroundImportPicked)
        }
        Message::GkBackgroundImportPicked(path) => {
            if let Some(path)=path {
                let loaded=std::fs::read(&path).map_err(|e|e.to_string())
                    .and_then(|raw|GkBackgroundFile::from_bytes(&raw));
                match loaded {
                    Ok(value)=>{
                        context.gk_calibration.pending_background=Some(value.raw());
                        context.gk_calibration.status=format!(
                            "Background file validated: {}. NOT ACTIVE. Explicitly confirm that all boards, sensor settings, mounting and magnetic environment match before using.",
                            path.display());
                    }
                    Err(e)=>{
                        context.gk_calibration.pending_background=None;
                        context.gk_calibration.status=format!("Background import rejected: {e}");
                    }
                }
            }
            Task::none()
        }
        Message::ConfirmImportedGkBackground => {
            if context.gk_calibration.capturing.is_some() {
                context.gk_calibration.status="Cannot replace background during an active Hall capture".into();
                return Task::none();
            }
            if let Some(raw)=context.gk_calibration.pending_background.take() {
                context.rigid_fit.invalidate_for_gk_calibration();
                context.gk_calibration.background=Some(raw);
                context.gk_calibration.reference=None;
                context.gk_calibration.calibrated_candidate=None;
                context.gk_calibration.status="Imported 48-sensor background ACTIVATED for this explicitly confirmed unchanged physical setup; capture a new stationary reference before calibration.".into();
            }
            Task::none()
        }
        Message::BeginGkReference => {
            if context.offline_replay.is_some() {
                context.replay_notice = "Stationary reference capture is disabled during offline replay.".into();
                return Task::none();
            }
            // Capture all 48 raw Hall fields even if CT cannot be imported
            // at the GK console. They can be paired with DICOM offline.
            context.rigid_fit.invalidate_for_gk_calibration();
            context.gk_calibration.begin(true);
            Task::none()
        }
        Message::ReviewCadGkRegistration => {
            context.gk_calibration.cad_mapping_approved = false;
            context.gk_calibration.cad_review_preview = None;
            context.gk_calibration.cad_review_summary = None;
            context.gk_calibration.cad_review_details_expanded = false;
            context.gk_calibration.calibrated_candidate = None;
            context.rigid_fit.invalidate_for_gk_calibration();
            match context.cbct_geometry.as_ref() {
                Some(geometry) => match geometry.registration_review_cad_preview()
                    .and_then(|details|geometry.registration_review_cad_summary()
                        .map(|summary|(details,summary))) {
                    Ok((details,summary)) => {
                        context.gk_calibration.cad_review_preview = Some(details);
                        context.gk_calibration.cad_review_summary = Some(summary);
                        context.gk_calibration.status =
                            "Mapping checks passed. Review the three boards, then approve for experimental calibration.".into();
                    }
                    Err(error) => {
                        context.gk_calibration.status =
                            format!("Mapping checks failed: {error}");
                    }
                },
                None => {
                    context.gk_calibration.status =
                        "Import CT geometry before reviewing the mapping.".into();
                }
            }
            Task::none()
        }
        Message::ToggleCadMappingDetails => {
            if context.gk_calibration.cad_review_preview.is_some() {
                context.gk_calibration.cad_review_details_expanded =
                    !context.gk_calibration.cad_review_details_expanded;
            }
            Task::none()
        }
        Message::CopyCadMappingDetails => {
            if let Some(report)=&context.gk_calibration.cad_review_preview {
                iced::clipboard::write(report.clone())
            } else {
                Task::none()
            }
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
                        "Mapping approved for experimental calibration. Tracking remains off until Hall calibration succeeds.".into();
                }
                Ok(_) => {
                    context.gk_calibration.cad_mapping_approved=false;
                    context.gk_calibration.status =
                        "Mapping changed or was not previewed. Review again.".into();
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
            context.gk_calibration.cad_review_summary=None;
            context.gk_calibration.cad_review_details_expanded=false;
            context.gk_calibration.calibrated_candidate=None;
            context.rigid_fit.invalidate_for_gk_calibration();
            context.gk_calibration.status =
                "Mapping approval revoked. Experimental tracking disabled.".into();
            Task::none()
        }
        Message::CalibrateGkTracking => {
            if context.offline_replay.is_some() {
                context.replay_notice = "Do not treat old replay frames as a new CT/Hall calibration. Load a verified matching rigid profile if needed.".into();
                return Task::none();
            }
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
            if context.offline_replay.is_some() {
                context.replay_notice = "Enabling a newly calibrated GK experiment is unavailable during offline replay.".into();
                return Task::none();
            }
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
                            "selected_detector_variant":geometry.selected_detector_variant,
                            "hybrid_source_sha256":geometry.hybrid_source_sha256,
                            "detector_replay": {
                                "format": "gk_cbct_detections_v1",
                                "coordinate_frame": "DICOM_LPS_mm",
                                "frame_of_reference_uid": geometry.frame_uid,
                                "selected_detector_variant": geometry.selected_detector_variant,
                                "hybrid_source_sha256": geometry.hybrid_source_sha256,
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
                                    "board_center_source":a.board_center_source,
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
                                "board_center_source":a.board_center_source,
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
        Message::FindHdmmCtCandidates => {
            if context.cbct_importing || context.hdmm_ct_searching ||
                context.hdmm_ct_approving { return Task::none(); }
            let Some(geometry) = context.cbct_geometry.clone() else {
                context.hdmm_ct_status =
                    "First import the ORIGINAL CBCT ZIP or DICOM folder.".into();
                return Task::none();
            };
            context.hdmm_ct_searching = true;
            context.hdmm_ct_proposal = None;
            context.hdmm_ct_candidate_choice = None;
            context.hdmm_ct_review_note.clear();
            context.hdmm_ct_status =
                "Searching CT for unverified nose-area HDMM sticker candidates (not identifying the optical reflector)...".into();
            Task::perform(
                find_hdmm_ct_candidates_off_thread(geometry),
                Message::HdmmCtCandidatesFound,
            )
        }
        Message::HdmmCtCandidatesFound(result) => {
            // A previously started worker must never attach its results to a
            // newly imported scan, or resurrect them after source invalidation.
            if !context.hdmm_ct_searching { return Task::none(); }
            context.hdmm_ct_searching = false;
            match result {
                Ok(proposal) => {
                    let same_scan = context.cbct_geometry.as_ref().is_some_and(|geometry|
                        geometry.frame_uid == proposal.frame_uid &&
                        geometry.scan_path == proposal.source_scan
                    );
                    if !same_scan {
                        context.hdmm_ct_status =
                            "Outdated HDMM search result discarded after CT change.".into();
                        return Task::none();
                    }
                    context.hdmm_ct_status = format!(
                        "CT search: {} — {} UNVERIFIED components. Review the original CT and PNG; no sticker automatically identified. {}",
                        proposal.status,
                        proposal.candidates.len(),
                        if proposal.preview_path.is_some() {
                            "Use Open CT review image before selecting a candidate."
                        } else { "No review preview produced (anatomy search abstained)." },
                    );
                    context.hdmm_ct_proposal = Some(proposal);
                }
                Err(e) => {
                    context.hdmm_ct_status = format!("HDMM CT candidate search FAILED: {e}");
                }
            }
            Task::none()
        }
        Message::SelectHdmmCtCandidate(id) => {
            // Explicitly selecting a candidate ID is NOT a sticker approval.
            if context.hdmm_ct_proposal.as_ref()
                .is_some_and(|p|p.candidates.iter().any(|c|c.id==id)) {
                context.hdmm_ct_candidate_choice = Some(id);
            }
            Task::none()
        }
        Message::UpdateHdmmReviewNote(value) => {
            context.hdmm_ct_review_note = value;
            Task::none()
        }
        Message::OpenHdmmCtPreview => {
            let Some(path) = context.hdmm_ct_proposal.as_ref()
                .and_then(|p|p.preview_path.clone()) else {
                context.hdmm_ct_status = "No HDMM CT preview image to open.".into();
                return Task::none();
            };
            if !path.is_file() {
                context.hdmm_ct_status =
                    format!("HDMM preview image no longer exists: {}",path.display());
                return Task::none();
            }
            let mut command = if cfg!(windows) {
                std::process::Command::new("explorer.exe")
            } else if cfg!(target_os="macos") {
                std::process::Command::new("open")
            } else {
                std::process::Command::new("xdg-open")
            };
            context.hdmm_ct_status = match command.arg(&path).spawn() {
                Ok(_) => format!("Opened CT review image: {}. Verify original CT slices as well; the review focus is NOT an optical sticker identification.",path.display()),
                Err(e) => format!("Cannot open CT preview automatically ({e}). Open this image manually: {}",path.display()),
            };
            Task::none()
        }
        Message::CopyHdmmCtReportPath => {
            if let Some(proposal) = context.hdmm_ct_proposal.as_ref() {
                iced::clipboard::write(format!(
                    "Candidate JSON: {}\nCT review PNG: {}",
                    proposal.proposal_path.display(),
                    proposal.preview_path.as_ref()
                        .map(|p|p.display().to_string())
                        .unwrap_or_else(||"none — anatomy search abstained".into())
                ))
            } else { Task::none() }
        }
        Message::ApproveHdmmCtCandidate => {
            if context.hdmm_ct_searching || context.hdmm_ct_approving {
                return Task::none();
            }
            let (Some(proposal), Some(selected_id)) = (
                context.hdmm_ct_proposal.clone(),
                context.hdmm_ct_candidate_choice.clone()
            ) else {
                context.hdmm_ct_status =
                    "Find and independently review a candidate in the original CT first.".into();
                return Task::none();
            };
            let note = context.hdmm_ct_review_note.trim().to_string();
            if note.chars().count() < 20 {
                context.hdmm_ct_status =
                    "Describe the original CT evidence verifying the physical optical sticker (20+ characters).".into();
                return Task::none();
            }
            if proposal.candidates.iter().any(|c|c.id==selected_id && c.edge_limited) {
                context.hdmm_ct_status =
                    "Selected component touches the CT/search boundary. Do NOT approve this partial candidate.".into();
                return Task::none();
            }
            context.hdmm_ct_approving = true;
            context.hdmm_ct_status =
                "Validating selected CT candidate against exact scan and detections (operator review; not optical calibration)...".into();
            Task::perform(
                approve_hdmm_ct_candidate_off_thread(proposal, selected_id, note),
                Message::HdmmCtCandidateApproved,
            )
        }
        Message::HdmmCtCandidateApproved(result) => {
            if !context.hdmm_ct_approving { return Task::none(); }
            context.hdmm_ct_approving = false;
            match result {
                Ok(approved) => {
                    let same_scan = context.cbct_geometry.as_ref().is_some_and(|geometry|
                        geometry.frame_uid == approved.frame_uid &&
                        geometry.scan_path == approved.source_scan
                    );
                    let same_proposal = context.hdmm_ct_proposal.as_ref()
                        .is_some_and(|p|p.proposal_path==approved.proposal_path);
                    if !same_scan || !same_proposal {
                        context.hdmm_ct_status =
                            "Outdated HDMM review result discarded after CT change.".into();
                        return Task::none();
                    }
                    // Reuse the existing strict JSON importer; success only
                    // stages the coordinates. A separate Set remains required.
                    context.hdmm_ct_status = format!(
                        "Operator-reviewed CT centroid saved to {}. Check staged coordinates, then click Set HDMM point. This is NOT a calibrated optical center.",
                        approved.path.display()
                    );
                    update(context, Message::ReviewedHdmmStickerFilePicked(
                        Some(approved.path)
                    ))
                }
                Err(e) => {
                    context.hdmm_ct_status =
                        format!("CT sticker review REJECTED: {e}");
                    Task::none()
                }
            }
        }
        Message::UpdateHdmmMarkerCoordinates(value) => {
            // Editing a reviewed value breaks its file provenance. Invalidate
            // the active marker prediction until Set is explicitly pressed.
            context.rigid_fit.clear_hdmm_target();
            context.hdmm_marker_coordinate_input=value;
            context.hdmm_sticker_source_status =
                "MANUAL ENTRY — previous reviewed-file provenance no longer applies. Independently verify actual sticker identity and CURRENT CT LPS coordinates; press Set to reactivate.".into();
            Task::none()
        }
        Message::ImportReviewedHdmmSticker => {
            if context.cbct_geometry.is_none() {
                context.hdmm_sticker_source_status =
                    "Import matching CT geometry BEFORE loading a reviewed HDMM marker.".into();
                return Task::none();
            }
            iced::window::oldest()
                .and_then(|id| iced::window::run(id, pick_reviewed_hdmm_sticker_file))
                .then(Task::future)
                .map(Message::ReviewedHdmmStickerFilePicked)
        }
        Message::ReviewedHdmmStickerFilePicked(picked) => {
            let Some(path) = picked else { return Task::none(); };
            // An old marker must not survive a newly attempted, incompatible
            // import. Operator has to click Set HDMM point after successful
            // review; the JSON picker NEVER directly activates fitting.
            context.rigid_fit.clear_hdmm_target();
            context.hdmm_marker_coordinate_input.clear();
            context.hdmm_sticker_source_status.clear();
            let result = (|| {
                let geometry = context.cbct_geometry.as_ref()
                    .ok_or("No current CT geometry loaded")?;
                let size = std::fs::metadata(&path)
                    .map_err(|e| format!("Cannot read marker JSON metadata: {e}"))?.len();
                if size > 2_000_000 {
                    return Err("HDMM marker JSON exceeds 2 MB".to_owned());
                }
                let raw = std::fs::read_to_string(&path)
                    .map_err(|e| format!("Cannot read marker JSON: {e}"))?;
                let magnets = geometry.magnets.iter()
                    .map(|m| (m.name.as_str(), m.center_lps_mm))
                    .collect::<Vec<_>>();
                hdmm_sticker::parse_reviewed_hdmm_point(
                    &raw, &magnets, &geometry.frame_uid
                )
            })();
            match result {
                Ok(reviewed) => {
                    let p = reviewed.marker_lps_mm;
                    context.hdmm_marker_coordinate_input =
                        format!("{:.6},{:.6},{:.6}", p[0], p[1], p[2]);
                    context.hdmm_sticker_source_status = format!(
                        "REVIEW REQUIRED BEFORE ACTIVATING: {}. Source file: {}. Inspect the predicted/reference point in its CT scan, then click Set HDMM point.",
                        reviewed.provenance, path.display()
                    );
                }
                Err(error) => {
                    context.hdmm_sticker_source_status =
                        format!("HDMM sticker geometry NOT accepted: {error}");
                }
            }
            Task::none()
        }
        Message::ApplyHdmmMarkerCoordinates => {
            // Only a measured reflective sticker point, not a generic CT
            // glass BB. This point is not supplied by the automatic detector.
            let values=context.hdmm_marker_coordinate_input
                .split(',').map(str::trim).map(str::parse::<f64>)
                .collect::<Result<Vec<_>,_>>();
            let result=match values {
                Ok(ref v) if v.len()==3 =>
                    context.rigid_fit.set_hdmm_target_lps([v[0],v[1],v[2]]),
                _=>Err("Enter exactly three CT LPS coordinates in millimetres, e.g. 100.2,-75.0,28.4".into())
            };
            match result {
                Ok(()) => {
                    if context.hdmm_sticker_source_status.starts_with(
                        "REVIEW REQUIRED BEFORE ACTIVATING:"
                    ) {
                        // Importing a reviewed file is not the same as setting
                        // its point; preserve provenance after explicit Set.
                        context.hdmm_sticker_source_status =
                            context.hdmm_sticker_source_status.replacen(
                                "REVIEW REQUIRED BEFORE ACTIVATING:",
                                "REVIEWED CT POINT SET:",
                                1,
                            );
                    }
                }
                Err(e) => context.rigid_fit.set_status(format!("HDMM point rejected: {e}")),
            }
            Task::none()
        }
        Message::ClearHdmmMarkerCoordinates => {
            context.rigid_fit.clear_hdmm_target();
            context.hdmm_marker_coordinate_input.clear();
            context.hdmm_sticker_source_status.clear();
            context.hdmm_sticker_source_status = "HDMM point cleared".into();
            Task::none()
        }
        Message::CaptureRigidBackground => {
            if context.offline_replay.is_some() {
                context.replay_notice = "Background capture is live-hardware-only; replayed fields are not a new independent background.".into();
                return Task::none();
            }
            match context.rigid_fit.capture_a1_background() {
                Ok(raw) => context.live_fits.apply_background_snapshot(&raw),
                Err(e) => context.rigid_fit.set_status(format!("Background: {e}")),
            }
            Task::none()
        }
        Message::SelectTrackingMode(mode) => {
            context.tracking_mode = mode;
            // No tracking algorithm may resume silently by switching modes
            // after the acquisition's raw input was invalidated.
            context.live_fits.set_fit_enabled(
                mode == TrackingMode::Independent &&
                context.rigid_fit.input_unreliable().is_none()
            );
            Task::none()
        }
        Message::RigidFitCompleted(completion) => {
            // The fitter also independently rejects stale generations.
            if context.rigid_fit.input_unreliable().is_some() {
                return Task::none();
            }
            context.rigid_fit.apply_completion(completion);
            Task::none()
        }
        Message::CopyRigidStatus => {
            iced::clipboard::write(rigid_fit_status_text(&context.rigid_fit.summary()))
        }
        Message::RecievedField(sensor_field) => {
            if context.offline_replay.is_some() { return Task::none(); }
            context.sensor_traces.update(&sensor_field);
            context.ping_field = Some(sensor_field);
            Task::none()
        },
        Message::ReceivedStreamFrame(frame) => {
            // Firmware subscriptions must never contaminate an offline run.
            if context.offline_replay.is_some() || context.replay_loading {
                return Task::none();
            }
            ingest_board_frame(context, frame, false)
        },
        Message::FitCompleted(completion) => {
            if context.rigid_fit.input_unreliable().is_some() {
                return Task::none();
            }
            context.live_fits.apply_fit_completion(completion);
            Task::none()
        },
        Message::StartFieldStream => {
            if context.offline_replay.is_some() || context.replay_loading {
                context.replay_notice = "Unload offline replay before starting the hardware field stream.".into();
                return Task::none();
            }
            context.rigid_fit.start_new_acquisition();
            context.sensor_traces.reset_histories();
            context.live_fits.set_fit_enabled(
                context.tracking_mode == TrackingMode::Independent
            );
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
                if context.offline_replay.is_some() || context.replay_loading {
                    context.replay_notice = "Use Pause or Unload for offline replay.".into();
                    return Task::none();
                }
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
            if context.offline_replay.is_some() || context.replay_loading {
                context.log_notice = "Logging is disabled during offline replay; recorded source files are read-only.".into();
                return Task::none();
            }
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
            if context.offline_replay.is_some() || context.replay_loading {
                context.log_notice = "Recording replayed frames is disabled; unload replay to log a new hardware acquisition.".into();
                return Task::none();
            }
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
            if context.offline_replay.is_some() || context.replay_loading {
                context.log_notice = "Cannot append replay frames into a hardware log.".into();
                return Task::none();
            }
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

        Message::SelectWideMlxRangePreset => {
            // Preset only; never write hardware behind the operator's back.
            // RES=1 is the widest symmetric digital 16-bit mode supported
            // whether or not the existing chip has TCMP enabled.
            // RES=2/3 require independently checking TCMP and include an
            // asymmetric output encoding at RES=3.
            context.mlx_gain = "0".to_string();
            context.mlx_resolution = "1".to_string();
            context.mlx_hall_conf = MLX_HALL_CONF_DEFAULT_LABEL.to_string();
            Task::none()
        }

        Message::GetMlxSensitivity => {
            if context.offline_replay.is_some() { return Task::none(); }
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
            if context.offline_replay.is_some() || context.replay_loading {
                context.replay_notice = "Historical MLX settings cannot be changed via replay; hardware writes disabled.".into();
                return Task::none();
            }
            // Changing sensitivity mid-stream changes the physical field
            // conversion and invalidates any stationary calibration.
            if context.stream_status != "idle" || context.file_writer.is_some() {
                return Task::none();
            }
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
            if context.offline_replay.is_some() { return Task::none(); }
            context.mlx_gain = status.gain.to_string();
            context.mlx_resolution = status.resolution.to_string();
            let changed=context.mlx_status.as_ref().is_none_or(|old|
                old.gain!=status.gain || old.resolution!=status.resolution ||
                old.hall_conf!=status.hall_conf);
            context.mlx_hall_conf = hall_conf_value_to_label(status.hall_conf).to_string();
            context.mlx_status = Some(status);
            // Changing gain/resolution can change full-scale, but merely
            // rereading unchanged settings should preserve the user range.
            if changed {
                context.sensor_component_full_scale_mt.clear();
                // No old GK stationary model may be silently reused after
                // an observed conversion-configuration change.
                context.rigid_fit.invalidate_for_gk_calibration();
                context.gk_calibration.calibrated_candidate = None;
                context.gk_calibration.reference = None;
                context.gk_calibration.background = None;
                context.gk_calibration.pending_background = None;
                context.gk_calibration.status =
                    "MLX sensitivity changed or first read: prior Hall background/reference invalid. "
                    .to_string()
                    + "Retake no-magnet background and stationary calibration.";
            }
            Task::none()
        }

        Message::GetMlxTiming => {
            if context.offline_replay.is_some() { return Task::none(); }
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
        Message::SetSensorComponentFullScale(value) => {
            context.sensor_component_full_scale_mt=value;
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
        && !context.log_write_in_progress && !context.log_file_dialog_open
        && context.offline_replay.is_none() && !context.replay_loading;
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
                if context.offline_replay.is_some() || context.replay_loading {
                    button("Start Field Stream (disabled during replay)")
                } else {
                    button("Start Field Stream").on_press(Message::StartFieldStream)
                },
                if context.offline_replay.is_some() || context.replay_loading {
                    button("Stop Field Stream (use replay controls)")
                } else {
                    button("Stop Field Stream").on_press(Message::StopFieldStream)
                },
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

    // Offline replay is a separate, read-only acquisition mode. Neither
    // its source file nor any existing hardware log can be written by it.
    let can_load_replay = context.offline_replay.is_none() && !context.replay_loading
        && (context.stream_status.is_empty() || context.stream_status == "idle")
        && context.file_writer.is_none() && !context.log_stop_requested
        && !context.log_write_in_progress && !context.log_finalize_in_progress
        && !context.log_file_dialog_open;
    let mut replay_load_button = button("Open MLXBF002 recording");
    if can_load_replay {
        replay_load_button = replay_load_button.on_press(Message::LoadOfflineReplay);
    }
    let mut replay_play_button = button("Play");
    let mut replay_pause_button = button("Pause");
    let mut replay_restart_button = button("Restart");
    let mut replay_unload_button = button("Unload");
    if let Some(replay) = &context.offline_replay {
        if !replay.running && !replay.finished() {
            replay_play_button = replay_play_button.on_press(Message::ReplayPlay);
        }
        if replay.running {
            replay_pause_button = replay_pause_button.on_press(Message::ReplayPause);
        }
        replay_restart_button = replay_restart_button.on_press(Message::ReplayRestart);
        replay_unload_button = replay_unload_button.on_press(Message::ReplayUnload);
    }
    let replay_status_text = if let Some(replay) = &context.offline_replay {
        format!("{} | {}", replay.recording.path().display(), replay.progress())
    } else if context.replay_loading {
        "Loading recording; please do not start a hardware stream.".to_string()
    } else {
        "No recording loaded".to_string()
    };
    let replay_panel = container(column![
        text("OFFLINE REPLAY — read-only, no sensor boards needed").size(18),
        text("Load an existing MLXBF002 log to drive raw Hall traces, range warnings and (only with a separately verified matching calibration profile) the experimental six-DOF fitter. This does NOT import measured optical HDMM."),
        row![
            replay_load_button,
            replay_play_button,
            replay_pause_button,
            replay_restart_button,
            replay_unload_button,
            text("Speed:"),
            pick_list(
                ReplaySpeed::ALL.to_vec(),
                context.offline_replay.as_ref().map(|replay| replay.speed),
                Message::ReplaySpeedSelected,
            ),
        ].spacing(8),
        text(replay_status_text),
        text(context.replay_notice.as_str())
            .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
        text("REPLAY SAFETY: live acquisition and logging are disabled while a file is loaded. Historical firmware gain/resolution is UNKNOWN from this binary; dotted limits are provisional. Raw fields may already be wrapped; QC and independent CT/HDMM truth are still required."),
    ].spacing(8))
        .padding(12)
        .width(Length::Fill)
        .style(iced::widget::container::bordered_box);

    let stream_widget = container(
        column![
            replay_panel,
            logging_panel,
            text(format!(
                "Stream: {} · host frames received {}",
                if context.offline_replay.is_some() {
                    "OFFLINE REPLAY (not hardware)"
                } else if context.stream_status.is_empty() {
                    "idle"
                } else {
                    context.stream_status.as_str()
                },
                context.received_stream_frames,
            )),
            text(board_presence_text(&context.board_presence)),
            text(context.rigid_fit.input_unreliable()
                .map(|reason| format!("RIGID POSE INVALID — {reason}; historical plots frozen; restart acquisition after correction."))
                .unwrap_or_else(|| "Rigid input QC: no latched critical range/step fault; not validated absolute accuracy.".into()))
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            text(live_hall_range_status(context))
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            text("Range warning scans latest Bx/By/Bz of ALL sensors; BACK OFF at 80%, caution at 65% of the lower of digital range and conservative 25mT analog guard. A sudden >8mT step also latches a review warning for this stream. Values may already be aliased. Stop streaming before changing settings."),
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
        let fit_quality = if rigid.input_unreliable.is_some() {
            format!("NO VALID CURRENT POSE — input QC latched; {} earlier completed fits retained for historical review ONLY.", rigid.completed)
        } else {
            rigid.result.as_ref()
                .map(|r| format!("Field RMSE {:.3} mT · {} sensors · accepted {} / rejected {} · fit {:.0} ms",
                    r.rmse_mt, r.sensor_count, rigid.completed.saturating_sub(rigid.rejected),
                    rigid.rejected, rigid.last_duration_ms.unwrap_or(0.0)))
                .unwrap_or_else(|| format!("No accepted fit yet · {} completed / {} rejected",
                    rigid.completed, rigid.rejected))
        };
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
                text("Detector:"),
                pick_list(
                    cbct_geometry::CbctDetectorVariant::ALL.to_vec(),
                    Some(context.cbct_detector_variant),
                    Message::SelectCbctDetectorVariant,
                ),
            ].spacing(8),
            text("Original: baseline detector, speedups off. Hybrid: alternate board detector. Optimized: accelerated maintained detector."),
            text("Original and Optimized use scripts shipped inside this app repo (Python runtime still needed). Hybrid is experimental and unavailable until its reviewed detector source is installed."),
            text("Changing detector revokes mapping approval. A successful CT import never enables magnetic tracking."),
            row![
                if context.cbct_setup_checking || context.cbct_importing {
                    button("Check DICOM packages")
                } else {button("Check DICOM packages").on_press(Message::CheckCbctSetup)},
                button("Copy setup commands").on_press(Message::CopyCbctSetupInstructions),
                button("Copy package report").on_press(Message::CopyCbctSetupResults),
            ].spacing(8),
            text("Install once on each lab computer: app/python_cbct/requirements.txt. Run scripts/setup_dicom_windows.ps1 or use Copy setup commands. No packages are installed by these buttons."),
            text(if context.cbct_setup_status.is_empty() {
                "DICOM Python environment not checked on this computer.".to_string()
            } else {context.cbct_setup_status.clone()}),
            row![
                button("Import ZIP").on_press(Message::ImportCbctZip),
                button("Import folder").on_press(Message::ImportCbctFolder),
                button("Load JSON").on_press(Message::LoadCbctDetectionsJson),
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
                    Ok(boards) => format!("CT-only mapping: {} boards registered geometrically (provisional; confirm JST and chip faces).",boards.len()),
                    Err(error) => format!("CT-only mapping unavailable: {error}. The separate CAD-assisted review may still be valid.")
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

        // Keep the full scientific registration report available on demand
        // without flooding the main tracking controls with 48 indices and
        // three raw Hall-to-LPS matrices.
        let cad_mapping_panel = container(column![
            text("CT sensor mapping").size(20),
            text("CT measures the sensor chips. The top board's connector direction comes from a KiCad mounting estimate, not a CT measurement.")
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            row![
                if context.cbct_geometry.is_some() {
                    button("Preview mapping").on_press(Message::ReviewCadGkRegistration)
                } else { button("Preview mapping") },
                if context.gk_calibration.cad_review_preview.is_some() {
                    button(if context.gk_calibration.cad_review_details_expanded {
                        "Hide details"
                    } else { "Show details" }).on_press(Message::ToggleCadMappingDetails)
                } else { button("Show details") },
                if context.gk_calibration.cad_review_preview.is_some() {
                    button("Copy mapping").on_press(Message::CopyCadMappingDetails)
                } else { button("Copy mapping") },
            ].spacing(8),
            text(context.gk_calibration.cad_review_summary.as_deref()
                .unwrap_or("No mapping preview yet. Import CT results and select Preview mapping."))
                .size(16)
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            if context.gk_calibration.cad_review_details_expanded {
                // The scrollable and its content must BOTH fill the available
                // horizontal space. Otherwise Iced sizes the scroll area to
                // the unwrapped text, leaving a blank strip at the right.
                container(column![
                    text("Detailed sensor mapping (scroll for all three boards)")
                        .size(17).width(Length::Fill),
                    scrollable(
                        text(context.gk_calibration.cad_review_preview.as_deref()
                            .unwrap_or("No mapping details available."))
                            .size(16)
                            .width(Length::Fill)
                            .wrapping(iced::widget::text::Wrapping::WordOrGlyph)
                    )
                    .width(Length::Fill)
                    .height(Length::Fixed(420.0)),
                ]
                .spacing(8)
                .width(Length::Fill))
                .padding(12)
                .width(Length::Fill)
                .style(iced::widget::container::bordered_box)
            } else {
                container(text(""))
            },
            text("Before approval: confirm board 0 = top, 1 = left, 2 = right; correct PCB revision and chip-facing sides; top JST toward patient right and side JSTs inferior.")
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            row![
                if context.gk_calibration.cad_review_preview.is_some()
                    && !context.gk_calibration.cad_mapping_approved {
                    button("Approve for calibration")
                        .on_press(Message::ApproveCadGkRegistration)
                } else { button("Approve for calibration") },
                if context.gk_calibration.cad_mapping_approved {
                    button("Revoke approval").on_press(Message::RevokeCadGkRegistration)
                } else { button("Revoke approval") },
            ].spacing(8),
            text(if context.gk_calibration.cad_mapping_approved {
                "APPROVED for experimental calibration only. This does not validate tracking."
            } else if context.gk_calibration.cad_review_preview.is_some() {
                "PREVIEW PASSED. Awaiting explicit approval."
            } else {
                "NOT APPROVED. A valid CT mapping preview is required."
            }),
        ].spacing(9).width(Length::Fill))
        .padding(12)
        .width(Length::Fill)
        .style(iced::widget::container::bordered_box);

        // Magnetic calibration stays separate from CT geometry and approval.
        let tracking_panel = container(column![
            text("Magnetic calibration and tracking").size(20),
            text("Experiment workflow: fixed boards, no magnets, then stationary magnets."),
            if context.gk_calibration.capturing.is_none() {
                button("Capture background").on_press(Message::BeginGkBackground)
            } else { button("Capture background") },
            text(if context.gk_calibration.background.is_some() {
                "No-magnet background ready (48 sensors); reusable only for unchanged hardware."
            } else {
                "No background captured. Optional; fitted offsets are less reliable."
            }),
            row![
                if context.gk_calibration.background.is_some() {
                    button("Save background only").on_press(Message::ExportGkBackground)
                } else { button("Save background only") },
                button("Load background").on_press(Message::ImportGkBackground),
            ].spacing(8),
            if context.gk_calibration.pending_background.is_some()
                && context.gk_calibration.capturing.is_none() {
                button("Confirm same setup and use background")
                    .on_press(Message::ConfirmImportedGkBackground)
            } else { button("Confirm same setup and use background") },
            text("Background needs NO CT or stationary reference. Export before changing the setup. Import requires confirmation; never reuse after board/magnet mounting changes."),
            if context.gk_calibration.capturing.is_none() {
                button("Capture stationary fields").on_press(Message::BeginGkReference)
            } else { button("Capture stationary fields") },
            text(if context.gk_calibration.reference.is_some() {
                "Stationary reference captured (48 sensors)."
            } else {
                "Stationary reference not captured."
            }),
            if context.gk_calibration.reference.is_some() {
                button("Calibrate tracking").on_press(Message::CalibrateGkTracking)
            } else { button("Calibrate tracking") },
            text(if context.gk_calibration.status.is_empty() {
                "Calibration not started.".to_string()
            } else { context.gk_calibration.status.clone() }),
            button("Copy calibration report").on_press(Message::CopyGkCalibrationStatus),
            text("Calibration requires matching CT and Hall measurements. Passing a stationary fit does not establish motion accuracy."),
            if context.gk_calibration.calibrated_candidate.is_some() {
                button("Enable experimental tracking").on_press(Message::EnableExperimentalGkTracking)
            } else { button("Enable experimental tracking") },
            text(if context.gk_calibration.calibrated_candidate.is_some() {
                "Experimental fit available after stationary calibration. Verify against HDMM."
            } else { "Experimental tracking disabled until calibration succeeds." }),
            if context.gk_calibration.reference.is_some() {
                button("Save calibration data").on_press(Message::ExportGkCalibration)
            } else { button("Save calibration data") },
            text("Keep DICOM and raw Hall logs for offline analysis. Save calibration data before leaving the GK setup."),
            button("Load legacy calibration").on_press(Message::LoadRigidProfile),
            text(profile_status),
            row![
                if rigid.result.is_some() {
                    button("Set zero").on_press(Message::ResetRigidZero)
                } else { button("Set zero") },
                text(if rigid.zero_ready { "Reference set" } else { "Reference not set" }),
            ].spacing(8),
            text(position_line.clone()),
            text(rotation_line.clone()),
            text("HDMM comparison (predicted from rigid pose — NOT measured HDMM):"),
            text("Use a physically reviewed HDMM sticker reference or a magnet-rigid transferred point matching the CURRENT CT. A target-scan prediction is not a direct CT detection. Original glass BB is not the optical reflector."),
            if context.cbct_geometry.is_some() {
                button("Load reviewed sticker JSON")
                    .on_press(Message::ImportReviewedHdmmSticker)
            } else {
                button("Load reviewed sticker JSON")
            },
            text(if context.hdmm_sticker_source_status.is_empty() {
                "No reviewed sticker file loaded. Manual CT LPS entry requires independent verification."
            } else {
                context.hdmm_sticker_source_status.as_str()
            }).wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            row![
                text_input("CT LPS X,Y,Z (mm)",&context.hdmm_marker_coordinate_input)
                    .on_input(Message::UpdateHdmmMarkerCoordinates),
                button("Set HDMM point").on_press(Message::ApplyHdmmMarkerCoordinates),
                button("Clear").on_press(Message::ClearHdmmMarkerCoordinates),
            ].spacing(8),
            text(if rigid.hdmm_point_ready {
                rigid.hdmm_current_mm.map(|mm|
                    format!("Predicted HDMM point displacement: {mm:.3} mm; compare with the external HDMM display at the same zero/time.")
                ).unwrap_or_else(||"HDMM point selected; awaiting accepted fit after zero.".into())
            } else {"No measured HDMM marker coordinates entered; prediction intentionally disabled.".into()}),
            text(format!("Tracking status: {}", rigid.status)),
            text(fit_quality.clone()),
        ].spacing(9))
        .padding(12)
        .width(Length::Fill)
        .style(iced::widget::container::bordered_box);

        // Put LIVE tracking and raw sensor traces together ABOVE long CT
        // detection/calibration details. A positioning operator should never
        // need to scroll through the DICOM setup just to see a range warning.
        let live_range_message = live_hall_range_status(context);
        let live_trace_limits = live_hall_trace_guides(context);
        let live_range_color = if live_range_message.contains("BACK OFF MAGNET") {
            iced::Color::from_rgb8(190, 25, 25)
        } else if live_range_message.contains("CAUTION") {
            iced::Color::from_rgb8(180, 110, 0)
        } else {
            iced::Color::from_rgb8(70, 90, 110)
        };
        let input_qc_banner = rigid.input_unreliable.as_ref()
            .map(|reason| format!(
                "INVALID / UNRELIABLE 6-DOF TRACKING — {reason}. All pose/HDMM updates stopped. Prior curves are FROZEN pre-warning history, not valid current motion. Correct conditions and start a NEW acquisition or restart offline replay; do not just switch fits."
            ))
            .unwrap_or_else(|| "INPUT QC: no critical all-sensor range/step fault latched this acquisition (NOT proof of sensor accuracy).".to_string());
        let input_qc_color = if rigid.input_unreliable.is_some() {
            iced::Color::from_rgb8(190, 25, 25)
        } else {
            iced::Color::from_rgb8(70, 90, 110)
        };
        let mut rigid_content = column![
            text(live_range_message)
                .size(18)
                .color(live_range_color)
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            text(position_line.clone()).size(18),
            text(rotation_line.clone()).size(17),
            text(fit_quality.clone()),
        ].spacing(12);
        // In the live 6-DOF view, the HDMM-point prediction is the primary
        // motion monitor beside the raw Hall traces. Translation/rotation
        // plots remain available underneath it for additional diagnostics.
        // This is NOT a live measurement from the vendor HDMM hardware.
        let hdmm_display = hdmm_live_readout::hdmm_live_readout(
            rigid.hdmm_point_ready,
            rigid.zero_ready,
            rigid.hdmm_current_mm,
            rigid.input_unreliable.is_some(),
            context.stream_status == "receiving",
            context.offline_replay.is_some(),
            &rigid.status,
        );
        let hdmm_value_color = if hdmm_display.invalid {
            iced::Color::from_rgb8(190, 25, 25)
        } else if hdmm_display.held {
            iced::Color::from_rgb8(105, 110, 120)
        } else {
            iced::Color::from_rgb8(20, 75, 110)
        };
        let hdmm_review_hint = if rigid.hdmm_point_ready {
            "HDMM sticker point selected; see its CT provenance in the setup section."
        } else if context.hdmm_sticker_source_status.starts_with(
            "REVIEW REQUIRED BEFORE ACTIVATING:"
        ) {
            "Reviewed CT sticker loaded. Verify the location in this CT; then press Set HDMM point."
        } else if context.hdmm_sticker_source_status.starts_with(
            "HDMM sticker geometry NOT accepted:"
        ) {
            "Sticker JSON rejected; check CT identity, magnet positions, and details in setup."
        } else if !context.hdmm_marker_coordinate_input.trim().is_empty() {
            "Manual CT LPS coordinates require independent sticker verification before Set."
        } else {
            "Import geometry from the current CBCT, then load an operator-reviewed sticker JSON."
        };
        let rigid_plots = column![
            text(if rigid.input_unreliable.is_some() {
                "HDMM PREDICTION INVALID NOW — plot contains FROZEN earlier accepted estimates (NOT live HDMM)"
            } else {
                "Hall-predicted HDMM marker displacement (NOT measured HDMM)"
            }),
            // Keep a large Hall-derived numeric displacement immediately
            // above its graph for side-by-side comparison with the external
            // optical HDMM display. HOLD/INVALID must never look live.
            text(hdmm_display.value).size(34).color(hdmm_value_color),
            text(hdmm_display.caption)
                .size(13)
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            row![
                if context.cbct_geometry.is_some() {
                    button("Load reviewed sticker")
                        .on_press(Message::ImportReviewedHdmmSticker)
                } else { button("Load reviewed sticker") },
                if !context.hdmm_marker_coordinate_input.trim().is_empty() {
                    button("Set HDMM point")
                        .on_press(Message::ApplyHdmmMarkerCoordinates)
                } else { button("Set HDMM point") },
            ].spacing(8),
            text(hdmm_review_hint)
                .size(12)
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            hdmm_prediction_plot(rigid.hdmm_history),
            text(if rigid.input_unreliable.is_some() {
                "FROZEN PRE-WARNING RIGID MOTION — NOT CURRENT TRACKING"
            } else {
                "Additional rigid-motion diagnostics (experimental)"
            }),
            text("Joint rigid-body XYZ displacement"),
            rigid_xyz_plot(rigid.xyz_history),
            text("Rigid-body rotation"),
            rotation_plot(u16::MAX, rigid.rotation_history),
            // Keep the validity result WITH its rigid-motion charts:
            // on a tall lab monitor a banner above all plots is too far
            // from the rotation/HDMM monitor to be noticed while tracking.
            container(
                column![
                    text(if rigid.input_unreliable.is_some() {
                        "TRACKING INVALID — RIGID POSE AND HDMM PREDICTION SUSPENDED"
                    } else {
                        "Rigid-body input validity"
                    })
                    .size(if rigid.input_unreliable.is_some() { 19 } else { 14 })
                    .color(input_qc_color),
                    text(input_qc_banner)
                        .size(if rigid.input_unreliable.is_some() { 16 } else { 12 })
                        .color(input_qc_color)
                        .width(Length::Fill)
                        .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
                ]
                .spacing(6)
            )
            .padding(10)
            .width(Length::Fill)
            .style(iced::widget::container::bordered_box),
        ].spacing(9).width(Length::Fill);

        if context.dashboard_tab == DashboardTab::LiveTracking {
            // Put guide instructions ABOVE both chart sections, never in a
            // narrow side column. The key Hall traces and HDMM prediction
            // share the window in a 3:2 row, with XYZ/rotation secondary.
            rigid_content = rigid_content
                .push(
                    text(live_hall_trace_limit_caption(context))
                        .size(12)
                        .width(Length::Fill)
                        .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
                );

            let mut live_traces = column![
                text("LIVE RAW HALL TRACES — keep visible while moving magnets")
                    .size(17),
                text("Selected Bx/By/Bz history; range warning checks latest X/Y/Z on EVERY sensor, including those not plotted."),
            ].spacing(8).width(Length::Fill);
            // Always reserve the three board trace panels, even without
            // connected hardware. Real data replaces each placeholder as
            // BoardPresence/SensorTraceState reports a board.
            let observed_boards = context.sensor_traces.board_summaries();
            for board_id in 0u16..3 {
                if let Some(sensor_board) =
                    observed_boards.iter().find(|board| board.board_id == board_id)
                {
                    let latest = sensor_board.latest_axis_peaks_mt;
                    let peak_labels = ["Bx", "By", "Bz"].into_iter()
                        .enumerate()
                        .map(|(axis, label)| match latest[axis] {
                            Some((sensor, value)) =>
                                format!("{label}: {value:.2} mT (sensor {sensor})"),
                            None => format!("{label}: waiting"),
                        }).collect::<Vec<_>>().join("  |  ");
                    live_traces = live_traces
                        .push(row![
                            text(format!("Board {board_id} — raw sensor")),
                            pick_list(
                                sensor_board.available_sensors.clone(),
                                sensor_board.selected_sensor_index,
                                move |sensor_index| Message::SelectSensorTrace {
                                    board_id, sensor_index
                                },
                            ),
                        ].spacing(8))
                        .push(text(format!("LATEST all-sensor peaks: {peak_labels}")))
                        .push(text(if sensor_board.invalid_samples > 0 {
                            format!("MISSING/INVALID DATA: {} fields", sensor_board.invalid_samples)
                        } else {
                            "Digital readout may already be wrapped even with no active warning.".to_string()
                        }))
                        .push(sensor_trace_plot(
                            board_id,
                            sensor_board.selected_sensor_index,
                            sensor_board.selected_trace.clone(),
                            live_trace_limits,
                        ));
                } else {
                    live_traces = live_traces
                        .push(text(format!(
                            "Board {board_id} — NOT CONNECTED (waiting for stream)"
                        )))
                        .push(sensor_trace_plot(
                            board_id,
                            None,
                            Vec::new(),
                            live_trace_limits,
                        ));
                }
            }
            // During the experiment the raw Bx/By/Bz traces AND the
            // Hall-predicted HDMM-point displacement must stay visible
            // together, near the top and immediately below the guide.
            // Allocate more width to the three Hall-board charts. Keep
            // secondary XYZ/rotation plots beneath HDMM in the right column.
            // Board placeholders remain visible when hardware is absent.
            rigid_content = rigid_content.push(
                row![
                    container(live_traces).width(Length::FillPortion(3)),
                    container(rigid_plots).width(Length::FillPortion(2)),
                ]
                .spacing(16)
                .width(Length::Fill)
            );
        } else {
            // The Both tab places the separate Sensor Traces panel below
            // full-width rigid plots rather than duplicating the raw charts.
            rigid_content = rigid_content.push(
                container(rigid_plots).width(Length::Fill)
            );
        }
        // CT import/mapping remains available when needed, but does not
        // obstruct monitoring during an active 6DOF positioning experiment.
        rigid_content = rigid_content
            .push(tracking_panel)
            .push(dicom_panel)
            .push(cad_mapping_panel);

        let mut live_fit_content = column![
            text("Independent magnet tracking — one fitted magnet per board"),
            text(if rigid.input_unreliable.is_some() {
                "INVALID RAW INPUT — independent fitting also suspended; any older independent poses are historical ONLY. Start a fresh acquisition after correction."
            } else {
                "Works immediately from raw Hall fields; background subtraction is optional. These are not a joint rigid 6-DOF estimate."
            }).color(if rigid.input_unreliable.is_some() {
                iced::Color::from_rgb8(190, 25, 25)
            } else {
                iced::Color::from_rgb8(70, 90, 110)
            }),
        ].spacing(12);

        if summaries.is_empty() {
            live_fit_content = live_fit_content.push(text("No detected boards"));
        } else {
            for summary in summaries {
                let fit_text = match (rigid.input_unreliable.as_ref(), &summary.result) {
                    (Some(_), _) => "INVALID CURRENT INPUT — prior independent pose is not current; fitting suspended.".into(),
                    (None, Some(result)) => format!(
                        "Position: x={:.2} mm, y={:.2} mm, z={:.2} mm    Residual RMS: {:.4} mT    Sensors: {}    Fit scale: {:.3e}",
                        result.position.0,
                        result.position.1,
                        result.position.2,
                        result.residual_rms,
                        result.n_sensors,
                        result.moment_norm,
                    ),
                    (None, None) => format!(
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
            TrackingMode::Rigid6Dof => container(rigid_content)
                .padding(10).width(Length::Fill),
            TrackingMode::Independent => container(live_fit_content)
                .padding(10).width(Length::Fill),
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

        container(mode_header.push(selected_content))
            .padding(10).width(Length::Fill)
    };

    let sensor_trace_widget = {
        let summaries = context.sensor_traces.board_summaries();
        let trace_limits = live_hall_trace_guides(context);

        let mut trace_content = column![
            text("Sensor field traces"),
            text("Select one sensor per board to plot Bx, By, and Bz over time."),
            text(live_hall_range_status(context))
                .width(Length::Fill)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            text(live_hall_trace_limit_caption(context))
                .size(12)
                .wrapping(iced::widget::text::Wrapping::WordOrGlyph),
            text("Live back-off warning uses axis-specific nominal limits and latest samples from ALL sensors. This separate legacy panel's optional manual scalar applies only to its historical peak warning; no hardware saturation flag is available."),
            row![
                text("Optional manual common full-scale for legacy peak history (mT):"),
                text_input("Range for current gain/res", &context.sensor_component_full_scale_mt)
                    .on_input(Message::SetSensorComponentFullScale),
            ].spacing(8),
        ]
        .spacing(12);

        if summaries.is_empty() {
            trace_content = trace_content
                .push(text("NO SENSOR BOARDS DETECTED — waiting for streaming data. Dotted limits are provisional until the actual chip configuration is verified."));
            for board_id in 0u16..3 {
                trace_content = trace_content
                    .push(text(format!("Board {board_id} — NOT CONNECTED")))
                    .push(sensor_trace_plot(
                        board_id,
                        None,
                        Vec::new(),
                        trace_limits,
                    ));
            }
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
                        text(if summary.invalid_samples>0 {
                            format!("RAW DATA WARNING: {} invalid/missing components",summary.invalid_samples)
                        } else { "Received field values are finite; clipping remains possible".into() }),
                        text(match (summary.peak_component_mt,summary.peak_sensor_index) {
                            (Some(peak),Some(sensor)) => {
                                let limit=context.sensor_component_full_scale_mt.trim()
                                    .parse::<f64>().ok().filter(|v|v.is_finite() && *v>0.0);
                                format!("Peak across all sensors: {:.3} mT (sensor {}). {}",
                                    peak,sensor,
                                    if sensor_trace::above_warning_margin(Some(peak),limit) {
                                        "WARNING >=90% of verified full-scale; possible clipping"
                                    } else if limit.is_some() {
                                        "Below set alert margin; not proof against saturation"
                                    } else {"Full-scale unknown; cannot assess saturation"})
                            }
                            _=>"No Hall fields yet".into(),
                        }),
                        sensor_trace_plot(
                            board_id,
                            selected_sensor,
                            summary.selected_trace,
                            trace_limits,
                        ),
                    ]
                    .spacing(8)
                )
                .padding(10);

                trace_content = trace_content.push(board_card);
            }
        }

        container(trace_content).width(Length::Fill)
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

            text("The displayed range is digital readout range, NOT analog saturation or verified 48-chip settings. Stop streaming and logging before changing sensitivity; recalibrate after any change."),
            row![
                button("Select wider symmetric range (gain 0, res 1)")
                    .on_press(Message::SelectWideMlxRangePreset),
                text("Preset only: gain 0 / RES 1 / four-phase. Click Apply manually while stopped."),
            ].spacing(8),
            text("RES 2/3 can provide larger digital counts with TCMP disabled, but TCMP state must be read back and RES 3 is asymmetric; this preset avoids claiming unverified maximum physical range."),

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
                button("Read firmware cached setting").on_press(Message::GetMlxSensitivity),
                button("Read timing").on_press(Message::GetMlxTiming),
                if (context.stream_status.is_empty() || context.stream_status == "idle")
                    && context.file_writer.is_none()
                {
                    button("Apply to detected boards").on_press(Message::SetMlxSensitivity)
                } else {
                    button("Stop streaming/logging before Apply")
                },
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
            // Stack entire panels rather than squeezing every chart into
            // two half-width columns. The sensor guide caption stays ABOVE
            // the separate raw trace plots in sensor_trace_widget.
            container(
                column![
                    live_tracking_widget.width(Length::Fill),
                    sensor_trace_widget.width(Length::Fill),
                ]
                .spacing(12)
                .width(Length::Fill)
            )
            .width(Length::Fill)
        }
    };
    
    let mut dashboard_sections = column![
        stream_widget,
        tab_selector,
        active_dashboard_widget,
    ].spacing(12);
    // Keep advanced board/timing settings accessible when the independent
    // tracking mode is selected, without adding a fourth dashboard tab.
    // Sensitivity and range controls must remain accessible with rigid 6DOF.
    // Changing settings is disabled by the handler while streaming/logging.
    if context.dashboard_tab == DashboardTab::LiveTracking {
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
    fn mapping_details_are_optional_and_cannot_approve_without_valid_ct() {
        let mut context=Context::default();
        let _=update(&mut context,Message::ToggleCadMappingDetails);
        assert!(!context.gk_calibration.cad_review_details_expanded);
        assert!(!context.gk_calibration.cad_mapping_approved);

        context.gk_calibration.cad_review_preview=Some(
            "CANDIDATE — not an independently measured JST connector".into());
        context.gk_calibration.cad_review_summary=Some(
            "PASS: 3 boards, 48 CT sensor positions".into());
        let _=update(&mut context,Message::ToggleCadMappingDetails);
        assert!(context.gk_calibration.cad_review_details_expanded);
        let _=update(&mut context,Message::ToggleCadMappingDetails);
        assert!(!context.gk_calibration.cad_review_details_expanded);

        // Expanding the report and retaining a plausible-looking summary
        // can NEVER authorize CAD-based magnetic calibration by themselves.
        let _=update(&mut context,Message::ApproveCadGkRegistration);
        assert!(!context.gk_calibration.cad_mapping_approved);
        assert!(context.gk_calibration.calibrated_candidate.is_none());
        let _=update(&mut context,Message::RevokeCadGkRegistration);
        assert!(context.gk_calibration.cad_review_preview.is_none());
        assert!(context.gk_calibration.cad_review_summary.is_none());
        assert!(!context.gk_calibration.cad_review_details_expanded);
    }

    #[test]
    fn background_capture_and_portable_roundtrip_need_no_ct_or_stationary_reference() {
        let mut capture=GkCalibration::default();
        capture.begin(false);
        capture.capturing.as_mut().unwrap().started=
            std::time::Instant::now()-std::time::Duration::from_secs(4);
        for cycle in 0..2 {
            for raw_board in 0..3 {
                let mut frame=BoardFrame::default();
                frame.board_id=raw_board;
                frame.sensor_mask=0xffff;
                frame.frame_id=cycle;
                for (i,value) in frame.samples.iter_mut().enumerate() {
                    value.bx_ut=1000.0*raw_board as f32 + i as f32;
                    value.by_ut=0.0;
                    value.bz_ut=0.0;
                }
                capture.accept_frame(&frame);
            }
        }
        let raw=capture.background.expect("48 vectors must complete after two full board cycles");
        assert!(capture.reference.is_none());
        let json=GkBackgroundFile::from_raw(raw).unwrap().to_bytes().unwrap();
        assert_eq!(GkBackgroundFile::from_bytes(&json).unwrap().raw(),raw);
    }

    #[test]
    fn loaded_background_is_inactive_until_operator_confirms_matching_hardware() {
        let mut context=Context::default();
        let mut raw=[[[0.0;3];16];3];
        raw[0][0]=[0.012,-0.024,0.033];
        context.gk_calibration.pending_background=Some(raw);
        assert!(context.gk_calibration.background.is_none());
        let _=update(&mut context,Message::ConfirmImportedGkBackground);
        assert_eq!(context.gk_calibration.background,Some(raw));
        assert!(context.gk_calibration.pending_background.is_none());
        assert!(context.gk_calibration.reference.is_none());
        assert!(!context.rigid_fit.enabled());
    }

    #[test]
    fn rigid_live_range_warning_catches_all_sensors_and_uses_axis_limits() {
        let mut context = Context::default();
        context.board_presence = BoardPresence {
            board_mask: 0b001, sensor_masks: [0xffff, 0, 0],
        };
        context.sensor_traces.set_presence(context.board_presence.clone());
        let mut frame = BoardFrame::default();
        frame.board_id = 0;
        frame.sensor_mask = 0xffff;
        for (idx, sample) in frame.samples.iter_mut().enumerate() {
            sample.bx_ut = if idx == 15 { 4350.0 } else { 300.0 };
            sample.by_ut = 0.0;
            sample.bz_ut = if idx == 9 { 7300.0 } else { 0.0 };
        }
        let _=update(&mut context,Message::ReceivedStreamFrame(frame));
        let status = live_hall_range_status(&context);
        assert!(status.contains("BACK OFF MAGNET"));
        assert!(status.contains("UNVERIFIED"));
        // 7.3mT Z is a 92% warning; X 4.35mT is 89%.
        assert!(status.contains("sensor9 Bz"));
        assert!(status.contains("7.300 mT"));
    }

    #[test]
    fn sudden_raw_step_latches_back_off_even_after_latest_value_returns() {
        let mut context = Context::default();
        context.sensor_traces.set_presence(BoardPresence {
            board_mask: 1, sensor_masks: [0xffff,0,0],
        });
        let mut before = BoardFrame::default();
        before.board_id=0;
        before.sensor_mask=0xffff;
        before.base_time_us=1_000_000;
        before.samples[4].bx_ut=4850.0;
        let mut after = before.clone();
        after.base_time_us=1_030_000;
        after.frame_id=1;
        after.samples[4].bx_ut=-4850.0;
        let _=update(&mut context, Message::ReceivedStreamFrame(before));
        let _=update(&mut context, Message::ReceivedStreamFrame(after));
        let summaries=context.sensor_traces.board_summaries();
        assert!(summaries[0].sudden_jump_count>0);
        assert!(live_hall_range_status(&context).contains("RAW STEP WARNING"));
        assert!(live_hall_range_status(&context).contains("BACK OFF MAGNET"));
        context.sensor_traces.reset_histories();
        assert!(!live_hall_range_status(&context).contains("RAW STEP WARNING"));
    }

    #[test]
    fn dotted_trace_limits_follow_acknowledged_mlx_settings_not_unapplied_input() {
        let mut context = Context::default();
        // Without a credible firmware status, both trace guides and the
        // all-48-sensor warning use the same conservative reference.
        let old = live_hall_range_model(&context);
        assert_eq!(old.advisory_mt, [4.9152, 4.9152, 7.929856]);
        assert_eq!(live_hall_trace_guides(&context).advisory_mt, old.advisory_mt);
        assert!(live_hall_trace_limit_caption(&context).contains("UNVERIFIED"));

        // Selecting a wider preset does NOT program chips. The plot must
        // not claim increased headroom before the hardware RPC responds.
        let _ = update(&mut context, Message::SelectWideMlxRangePreset);
        assert_eq!(context.mlx_gain, "0");
        assert_eq!(live_hall_trace_guides(&context).advisory_mt, old.advisory_mt);

        // The firmware replies successfully to an actual configuration
        // operation (or returns its cached setting). This is still NOT a
        // per-chip hardware register readback.
        let _ = update(&mut context, Message::ReceivedMlxSensitivity(
            MlxSensitivityStatus {ok: true, gain: 0, resolution: 1, hall_conf: 0xC}
        ));
        let wide = live_hall_range_model(&context);
        assert!((wide.digital_mt[0] - 49.217536).abs() < 1e-8);
        assert!((wide.digital_mt[2] - 79.29856).abs() < 1e-8);
        assert_eq!(wide.advisory_mt, [25.0, 25.0, 25.0]);
        assert_eq!(live_hall_trace_guides(&context).advisory_mt, wide.advisory_mt);
        assert!(live_hall_trace_limit_caption(&context).contains("CACHED"));

        // New text-entry settings cannot silently change active guides.
        let _ = update(&mut context, Message::UpdateMlxGain("7".into()));
        assert_eq!(live_hall_range_model(&context).advisory_mt, wide.advisory_mt);
        let _ = update(&mut context, Message::ReceivedMlxSensitivity(
            MlxSensitivityStatus {ok: true, gain: 7, resolution: 1, hall_conf: 0xC}
        ));
        let changed = live_hall_range_model(&context);
        assert!((changed.advisory_mt[0] - 9.8304).abs() < 1e-8);
        assert!((changed.advisory_mt[1] - 9.8304).abs() < 1e-8);
        assert!((changed.advisory_mt[2] - 15.859712).abs() < 1e-8);
        // Unsupported RES2 is never displayed as a verified symmetric range.
        let _ = update(&mut context, Message::ReceivedMlxSensitivity(
            MlxSensitivityStatus {ok: true, gain: 7, resolution: 2, hall_conf: 0xC}
        ));
        assert_eq!(live_hall_range_model(&context).advisory_mt, old.advisory_mt);
        assert!(live_hall_trace_limit_caption(&context).contains("UNVERIFIED"));
    }

    #[test]
    fn live_dashboard_pairs_hall_traces_with_hdmm_prediction() {
        // Structural regression: the primary monitoring columns must be
        // adjacent below the dotted-guide caption, not separated by
        // scroll-length rigid displacement/rotation charts.
        // Cargo check verifies Iced types; visual layout needs a GUI review.
        let source = include_str!("main.rs");
        let rigid_heading = source.find(
            "Hall-predicted HDMM marker displacement (NOT measured HDMM)"
        ).expect("HDMM prediction explicitly distinguished from measurement");
        let hdmm = source[rigid_heading..].find(
            "hdmm_prediction_plot(rigid.hdmm_history)"
        ).expect("HDMM prediction chart") + rigid_heading;
        let xyz = source[rigid_heading..].find(
            "rigid_xyz_plot(rigid.xyz_history)"
        ).expect("secondary displacement chart") + rigid_heading;
        let rotation = source[rigid_heading..].find(
            "rotation_plot(u16::MAX, rigid.rotation_history)"
        ).expect("secondary rotation chart") + rigid_heading;
        assert!(hdmm < xyz && xyz < rotation);
        let readout = source[rigid_heading..].find(
            "text(hdmm_display.value).size(34)"
        ).expect("prominent numeric Hall sticker prediction") + rigid_heading;
        let source_hint = source[rigid_heading..].find(
            "text(hdmm_display.caption)"
        ).expect("readout provenance and HOLD/INVALID indication") + rigid_heading;
        let import = source[rigid_heading..].find(
            "button(\"Load reviewed sticker\")"
        ).expect("reviewed sticker import near prediction") + rigid_heading;
        let activate = source[rigid_heading..].find(
            "button(\"Set HDMM point\")"
        ).expect("explicit HDMM marker activation near graph") + rigid_heading;
        assert!(rigid_heading < readout && readout < source_hint
                && source_hint < import && import < activate && activate < hdmm);
        // The invalid/current status must live in the RIGHT-HAND
        // rigid-motion column, directly after its rotation chart—not
        // above the full-width plot row where it was hard to see.
        let rigid_section_end = source[rigid_heading..].find(
            "].spacing(9).width(Length::Fill);"
        ).expect("rigid column end") + rigid_heading;
        let rigid_column = &source[rigid_heading..rigid_section_end];
        let invalid_banner = rigid_column.find(
            "text(input_qc_banner)"
        ).expect("tracking validity status below rigid rotation");
        assert!(invalid_banner + rigid_heading > rotation);
        let top_section = &source[source.find("let mut rigid_content = column![")
            .expect("main tracking header")..rigid_heading];
        assert!(!top_section.contains("text(input_qc_banner)"),
            "tracking validity must not be separated from rigid charts");

        let start = source.find("// Put guide instructions ABOVE both chart sections")
            .expect("rigid monitoring section");
        let end = source[start..].find("// CT import/mapping")
            .expect("end of rigid monitoring section") + start;
        let live = &source[start..end];
        let guide = live.find("text(live_hall_trace_limit_caption(context))")
            .expect("dotted guide explanation");
        let trace_title = live.find("LIVE RAW HALL TRACES")
            .expect("raw sensor trace section");
        let paired = live.find(
            "container(live_traces).width(Length::FillPortion(3)),"
        ).expect("Hall traces in primary left column");
        let predicted = live.find(
            "container(rigid_plots).width(Length::FillPortion(2)),"
        ).expect("HDMM chart in adjacent right column");
        assert!(guide < trace_title && trace_title < paired && paired < predicted);
        assert!(live.contains("for board_id in 0u16..3"));
        assert!(live.contains("NOT CONNECTED (waiting for stream)"));
        assert!(source.contains("NO SENSOR BOARDS DETECTED — waiting for streaming data"));
        let start = source.find("let active_dashboard_widget = match context.dashboard_tab")
            .expect("dashboard tab selection");
        let end = source[start..].find("let mut dashboard_sections")
            .expect("end of dashboard tab selection") + start;
        let both = &source[start..end];
        // Structural invariant: Both stacks Live Tracking above Sensor Traces.
        // Ignore platform line endings and indentation (Windows CRLF vs LF).
        let normalized = both.split_whitespace().collect::<Vec<_>>().join(" ");
        assert!(
            normalized.contains(
                "column![ live_tracking_widget.width(Length::Fill), sensor_trace_widget.width(Length::Fill),"
            ),
            "Both tab must stack Live Tracking above Sensor Traces at full width"
        );
    }

    #[test]
    fn offline_replay_is_read_only_updates_traces_and_blocks_live_acquisition() {
        use std::fs;
        use std::time::{SystemTime, UNIX_EPOCH};
        let suffix = SystemTime::now().duration_since(UNIX_EPOCH)
            .unwrap().as_nanos();
        let path = std::env::temp_dir().join(
            format!("mlx_offline_readonly_{}_{}.bin", std::process::id(), suffix)
        );
        let mut bytes = b"MLXBF002".to_vec();
        let mut original_frames = Vec::new();
        for (board_id, microsecond) in [(0u16, 1_000_000u64),
            (1, 1_010_000), (2, 1_020_000), (0, 1_050_000)]
        {
            let mut frame = BoardFrame::default();
            frame.board_id = board_id;
            frame.base_time_us = microsecond;
            frame.sensor_mask = 0xffff;
            for sample in frame.samples.iter_mut() {
                sample.bx_ut = 4_800.0;
                sample.by_ut = 500.0;
                sample.bz_ut = 7_300.0;
            }
            let packed = postcard::to_stdvec(&frame).unwrap();
            bytes.extend_from_slice(&(packed.len() as u32).to_le_bytes());
            bytes.extend_from_slice(&packed);
            original_frames.push(frame);
        }
        fs::write(&path, &bytes).unwrap();
        let rec = OfflineRecording::load(&path).expect("decode actual logger framing");
        let mut context = Context::default();
        let _ = update(&mut context, Message::ReplayLoaded(Ok(rec)));
        assert!(context.offline_replay.is_some());
        assert_eq!(context.board_presence.board_mask, 0b111);
        assert!(!context.rigid_fit.enabled());
        assert!(live_hall_range_status(&context).contains("UNVERIFIED"));

        // While a recorded file is loaded, firmware and logging cannot run.
        let _ = update(&mut context, Message::StartFieldStream);
        assert_eq!(context.stream_status, "idle");
        let _ = update(&mut context, Message::StartLogging);
        assert!(context.file_writer.is_none());
        let _ = update(&mut context, Message::SetMlxSensitivity);
        assert!(context.mlx_status.is_none());

        let _ = update(&mut context, Message::ReplayPlay);
        let _ = update(&mut context, Message::ReplayTick);
        assert_eq!(context.received_stream_frames, 1);
        let boards = context.sensor_traces.board_summaries();
        assert_eq!(boards.len(), 3);
        assert_eq!(boards[0].selected_trace.len(), 1);
        assert!((boards[0].selected_trace[0].bx_mt - 4.8).abs() < 1e-6);
        assert!(live_hall_range_status(&context).contains("BACK OFF MAGNET"));
        assert!(context.rigid_fit.input_unreliable().is_some());
        assert!(context.rigid_fit.summary().hdmm_current_mm.is_none());

        // A stray firmware message must NOT be blended with recorded fields.
        let before = context.received_stream_frames;
        let _ = update(&mut context,
            Message::ReceivedStreamFrame(original_frames[0].clone()));
        assert_eq!(context.received_stream_frames, before);
        let _ = update(&mut context, Message::ReplayPause);
        let _ = update(&mut context, Message::ReplayRestart);
        assert_eq!(context.received_stream_frames, 0);
        assert!(context.rigid_fit.input_unreliable().is_none());
        assert!(context.sensor_traces.board_summaries()[0].selected_trace.is_empty());
        let _ = update(&mut context, Message::ReplayUnload);
        assert!(context.offline_replay.is_none());
        assert_eq!(context.board_presence.board_mask, 0);
        assert_eq!(fs::read(&path).unwrap(), bytes);
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn replay_interface_exposes_transport_and_explicit_source_label() {
        let source = include_str!("main.rs");
        assert!(source.contains("Open MLXBF002 recording"));
        assert!(source.contains("Message::ReplayPlay"));
        assert!(source.contains("Message::ReplayPause"));
        assert!(source.contains("Message::ReplayRestart"));
        assert!(source.contains("Message::ReplayUnload"));
        assert!(source.contains("OFFLINE REPLAY (not hardware)"));
        assert!(source.contains("Historical firmware gain/resolution is UNKNOWN"));
        assert!(source.contains("if !from_replay { context.gk_calibration.accept_frame(&frame); }"));
        assert!(source.contains("if !from_replay && context.file_writer.is_some()"));
    }

    #[test]
    fn critical_raw_range_latches_invalid_tracking_but_raw_traces_continue() {
        let mut context = Context::default();
        let presence = BoardPresence {
            board_mask: 1, sensor_masks: [0xffff, 0, 0],
        };
        context.board_presence = presence.clone();
        context.sensor_traces.set_presence(presence);
        let mut unsafe_frame = BoardFrame::default();
        unsafe_frame.board_id = 0;
        unsafe_frame.sensor_mask = 0xffff;
        unsafe_frame.base_time_us = 1_000_000;
        for sample in unsafe_frame.samples.iter_mut() {
            sample.bx_ut = 300.0;
            sample.by_ut = 0.0;
            sample.bz_ut = 0.0;
        }
        // The selected chart is sensor0 but sensor12 is unsafe:
        // 7.1mT Z exceeds 80% of provisional +/-7.929856mT.
        unsafe_frame.samples[12].bz_ut = 7100.0;
        let _ = update(&mut context, Message::ReceivedStreamFrame(unsafe_frame.clone()));
        let reason = context.rigid_fit.input_unreliable().expect("must latch");
        assert!(reason.contains("sensor 12 Bz"));
        assert!(reason.contains("UNVERIFIED"));
        assert!(!context.live_fits.fit_enabled());
        assert!(context.rigid_fit.summary().result.is_none());
        assert!(context.rigid_fit.summary().hdmm_current_mm.is_none());
        assert!(context.sensor_traces.board_summaries()[0].selected_trace.len() > 0);

        // Subsequent quiet frames keep raw evidence, but do not silently
        // re-enable a potentially aliased channel or any fitting model.
        let mut quiet = unsafe_frame.clone();
        quiet.base_time_us += 100_000;
        quiet.frame_id += 1;
        quiet.samples[12].bz_ut = 50.0;
        let _ = update(&mut context, Message::ReceivedStreamFrame(quiet));
        assert!(context.rigid_fit.input_unreliable().is_some());
        assert_eq!(context.received_stream_frames, 2);
        let _ = update(&mut context, Message::SelectTrackingMode(TrackingMode::Independent));
        assert!(!context.live_fits.fit_enabled());
        let _ = update(&mut context, Message::StartFieldStream);
        assert!(context.rigid_fit.input_unreliable().is_none());
        assert!(context.sensor_traces.board_summaries()[0].selected_trace.is_empty());
        assert!(context.live_fits.fit_enabled());
    }

    #[test]
    fn caution_does_not_latch_but_jump_below_wide_range_limit_does() {
        let mut context = Context::default();
        let presence = BoardPresence {
            board_mask: 1, sensor_masks: [0xffff, 0, 0],
        };
        context.board_presence = presence.clone();
        context.sensor_traces.set_presence(presence);
        let mut first = BoardFrame::default();
        first.board_id = 0;
        first.sensor_mask = 0xffff;
        first.base_time_us = 1_000_000;
        for sample in first.samples.iter_mut() {
            sample.bx_ut = 3440.0; // 70% provisional X advisory, CAUTION only
        }
        let _ = update(&mut context, Message::ReceivedStreamFrame(first.clone()));
        assert!(live_hall_range_status(&context).contains("CAUTION"));
        assert!(context.rigid_fit.input_unreliable().is_none());

        // New separate wide-range acquisition: +9mT is BELOW its 20mT
        // 80% advisory threshold, but a fast >8mT jump must still halt.
        context.mlx_status = Some(MlxSensitivityStatus {
            ok: true, gain: 0, resolution: 1, hall_conf: 0xC,
        });
        context.sensor_traces.reset_histories();
        let mut baseline = first.clone();
        for sample in baseline.samples.iter_mut() { sample.bx_ut = 0.0; }
        let _ = update(&mut context, Message::ReceivedStreamFrame(baseline.clone()));
        assert!(context.rigid_fit.input_unreliable().is_none());
        let mut jumped = baseline;
        jumped.base_time_us += 40_000;
        jumped.frame_id = 1;
        jumped.samples[10].bx_ut = 9000.0;
        let _ = update(&mut context, Message::ReceivedStreamFrame(jumped));
        let fault = context.rigid_fit.input_unreliable().expect("jump must latch");
        assert!(fault.contains("sensor 10 Bx"));
        assert!(fault.contains("sudden"));
        assert!(live_hall_range_status(&context).contains("RAW STEP WARNING"));
        assert!(context.rigid_fit.summary().result.is_none());
    }

    #[test]
    fn wide_range_preset_requires_explicit_apply_and_does_not_change_tracking_mode() {
        let mut context = Context::default();
        let initial_mode = context.tracking_mode;
        let _=update(&mut context,Message::SelectWideMlxRangePreset);
        assert_eq!(context.mlx_gain,"0");
        assert_eq!(context.mlx_resolution,"1");
        assert_eq!(context.mlx_hall_conf,MLX_HALL_CONF_DEFAULT_LABEL);
        assert_eq!(context.tracking_mode, initial_mode);
        context.stream_status="receiving".into();
        let _=update(&mut context,Message::SetMlxSensitivity);
        assert_eq!(context.stream_status,"receiving");
    }

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
            let mut subscriptions = Vec::new();
            if context.cbct_importing {
                subscriptions.push(
                    iced::time::every(std::time::Duration::from_millis(250))
                        .map(|_| Message::CbctProgressTick)
                );
            }
            if context.offline_replay.as_ref().is_some_and(|replay| replay.running) {
                subscriptions.push(
                    iced::time::every(std::time::Duration::from_millis(50))
                        .map(|_| Message::ReplayTick)
                );
            }
            iced::Subscription::batch(subscriptions)
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
