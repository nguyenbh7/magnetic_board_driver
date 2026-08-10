

use std::ops::Deref;
use std::path::PathBuf;
use std::{collections::BTreeMap, time::Duration};

use crossterm::event::{self, Event, KeyCode};
use data_transfer::rpc::{MagneticTopic, StopFieldStream};
use postcard::experimental::max_size::MaxSize;
use postcard_rpc::host_client::{HostClient, Subscription};
use postcard_rpc::standard_icd::WireError;
use ratatui::{text::Text, widgets::Row, Frame};
mod sensor_monitor;
mod displacement_plot;
use displacement_plot::displacement_plot;
mod sensor_trace;
mod sensor_trace_plot;
use sensor_trace::SensorTraceState;
use sensor_trace_plot::sensor_trace_plot;
mod live_fit;
use live_fit::{BoardLiveFits, MagnetPreset};
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
    messaging::MessageReader,
    rpc::{
        SensorField,
        SingleFieldValue,
        StartFieldStream,
        GetBoardPresence,
        BoardPresence,
        MAX_SENSOR_BOARDS,
        GetMlxSensitivity,
        SetMlxSensitivity,
        MlxSensitivityConfig,
        MlxSensitivityStatus,
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
    RecievedStreamField(data_transfer::rpc::SensorField),
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
    WroteFile,
    UpdateMlxGain(String),
    UpdateMlxResolution(String),
    UpdateMlxHallConf(String),
    GetMlxSensitivity,
    SetMlxSensitivity,
    ReceivedMlxSensitivity(MlxSensitivityStatus),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum DashboardTab {
    #[default]
    LiveFits,
    SensorTraces,
    Both,
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
    board_presence: BoardPresence,
    live_fits: BoardLiveFits,
    mlx_gain: String,
    mlx_resolution: String,
    mlx_hall_conf: String,
    mlx_status: Option<MlxSensitivityStatus>,
}

#[derive(Debug, Clone)]
pub enum Error {
    DialogClosed,
    IoError(tokio::io::ErrorKind),
}

const MLX_HALL_CONF_DEFAULT_LABEL: &str = "0xC = Default Sampling";
const MLX_HALL_CONF_FAST_LABEL: &str = "0x0 = Faster Sampling";

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

fn mlx_resolution_bit_depth(raw: u8) -> Option<u8> {
    match raw {
        0..=3 => Some(16 + raw),
        _ => None,
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

async fn start_field_stream(client: HostClient<WireError>) -> Result<(), String> {
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

async fn field_subscribe(client: HostClient<WireError>) -> Option<SensorSubscription> {
    let subs = client.subscribe_exclusive::<MagneticTopic>(64).await.ok()?;
    Some(SensorSubscription::new(subs))
}

async fn write_data(writer: &Mutex<BufWriter<File>>, field: &SensorField) {
    let mut writer = writer.lock().await;
    let mut data = [0; SensorField::POSTCARD_MAX_SIZE];
    let val = postcard::to_slice(field, &mut data);
    //println!("{:#?}", val);

    let val = writer.write_all(&mut data);
    //println!("{:#?}", val);
}

async fn stop_field_stream(client: HostClient<WireError>) -> () {
    client.send_resp::<StopFieldStream>(&()).await.unwrap()
}

fn update(context: &mut Context, message: Message) -> Task<Message> {

    //println!("{:#?}", message);
    match message {
        Message::PortSelected(serial_port_info) => {
            context.sensor_watcher = Some(SensorWatcher::new(&serial_port_info));
            context.board_presence = BoardPresence::default();
            context.live_fits = BoardLiveFits::default();
            context.sensor_traces = SensorTraceState::default();
            context.dashboard_tab = DashboardTab::LiveFits;
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
        Message::RecievedField(sensor_field) => {
            context.live_fits.update(sensor_field.clone());
            context.sensor_traces.update(&sensor_field);
            context.ping_field = Some(sensor_field);
            Task::none()
        },
        Message::RecievedStreamField(sensor_field) => {
            context.live_fits.update(sensor_field.clone());
            context.sensor_traces.update(&sensor_field);
            context.ping_field = Some(sensor_field.clone());

            match &context.file_writer {
                Some(w) => {
                    let wr = w.clone();
                    Task::perform(
                        (move || {
                            async move {
                                //println!("Test");
                                let sensor_field = sensor_field.clone();
                                let _val = write_data(&*wr, &sensor_field).await;
                            }
                        })(),
                        |_| Message::WroteFile
                    )
                },
                None => {
                    Task::none()
                },
            }
        },
        Message::StartFieldStream => {
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
            context.sensor_traces.set_presence(context.board_presence.clone());

            if let Some(sw) = &context.sensor_watcher {
                let client = sw.get_client();

                Task::perform(
                    start_field_stream(client),
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
                eprintln!("Failed to start field stream: {err}");

                // Most common cause: the firmware stream task is already running
                // or has not fully exited after Stop.
                return Task::none();
            }

            if let Some(sw) = &mut context.sensor_watcher {
                let client = sw.get_client();

                Task::future(field_subscribe(client)).and_then(|sub| {
                    Task::run(sub, Message::RecievedStreamField)
                })
            } else {
                Task::none()
            }
        }

        Message::StopFieldStream => {
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
                let writer = BufWriter::new(file);
                let writer = Arc::new(Mutex::new(writer));
                context.file_writer = Some(writer);
            }
            Task::none()
        },
        Message::WroteFile => {
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

        Message::SelectDashboardTab(tab) => {
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
            row![button("Start Field Stream").on_press(Message::StartFieldStream)],
            row![button("Stop Field Stream").on_press(Message::StopFieldStream)],
            text(board_presence_text(&context.board_presence)),
            row![
                text("File output: "),
                text_input("File", &context.ping_args.sensor),
                button("Select File").on_press(Message::SelectFile)
            ]
        ]
    );

    let live_fit_widget = {
        let summaries = context.live_fits.board_summaries();

        let mut live_fit_content = column![
            text("Live magnet fits by board")
        ]
        .spacing(12);

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

                let board_card = container(
                    column![
                        row![
                            text(format!("Board {}", summary.board_id)),
                            button("Capture no-magnet background")
                                .on_press(Message::CaptureBoardBackground(summary.board_id)),
                            button("Reset displacement zero")
                                .on_press(Message::ResetBoardDisplacement(summary.board_id)),
                            button("Calibrate magnet / set zero")
                                .on_press(Message::CalibrateBoardMagnet(summary.board_id)),
                        ]
                        .spacing(12),

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

        container(live_fit_content)
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

    let mlx_status_text = match &context.mlx_status {
        Some(status) => {
            let resolution_text = mlx_resolution_bit_depth(status.resolution)
                .map(|bits| format!("{} ({}-bit)", status.resolution, bits))
                .unwrap_or_else(|| format!("{} (invalid)", status.resolution));

            format!(
                "Hardware readback: ok={} · gain={} · resolution={} · hall_conf=0x{:X}",
                status.ok,
                status.gain,
                resolution_text,
                status.hall_conf,
            )
        }
        None => "Hardware readback: not read yet".to_string(),
    };

    let selected_hall_conf = Some(if context.mlx_hall_conf.is_empty() {
        MLX_HALL_CONF_DEFAULT_LABEL.to_string()
    } else {
        context.mlx_hall_conf.clone()
    });

    let mlx_widget = container(
        column![
            row![
                text("MLX90393 sensitivity"),
                text(mlx_status_text),
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
                    text("Raw value: 0 = 16-bit, 1 = 17-bit, 2 = 18-bit, 3 = 19-bit")
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
                text("Default is stable; faster sampling may reduce per-frame delay.")
            ]
            .spacing(4),

            row![
                button("Read from board").on_press(Message::GetMlxSensitivity),
                button("Apply to detected boards").on_press(Message::SetMlxSensitivity),
            ]
            .spacing(12),
        ]
        .spacing(10)
    )
    .padding(10);

    let tab_selector = row![
        button("Live fits").on_press(Message::SelectDashboardTab(DashboardTab::LiveFits)),
        button("Sensor traces").on_press(Message::SelectDashboardTab(DashboardTab::SensorTraces)),
        button("Both").on_press(Message::SelectDashboardTab(DashboardTab::Both)),
    ]
    .spacing(8);

    let active_dashboard_widget = match context.dashboard_tab {
        DashboardTab::LiveFits => live_fit_widget,
        DashboardTab::SensorTraces => sensor_trace_widget,
        DashboardTab::Both => {
            container(
                row![
                    live_fit_widget.width(Length::FillPortion(1)),
                    sensor_trace_widget.width(Length::FillPortion(1)),
                ]
                .spacing(12)
            )
        }
    };
    
    let dashboard_content = container(
        column![
            stream_widget,
            tab_selector,
            active_dashboard_widget,
            mlx_widget,
        ]
        .spacing(12)
    )
    .width(Length::Fill);

    let dashboard = scrollable(dashboard_content)
        .width(Length::Fill)
        .height(Length::Fill);

    column![
        serial_selector,
        ping_widget,
        dashboard,
    ]
    .spacing(10)
    .padding(10)
    .into()
}

#[tokio::main]
pub async fn main() -> iced::Result {
    iced::run(update, view)
}
