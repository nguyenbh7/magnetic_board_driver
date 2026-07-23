

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
mod live_fit;
use live_fit::BoardLiveFits;
use rfd::FileHandle;
use sensor_monitor::MagneticData;
use sipper::Sender;
use tokio::sync::mpsc::UnboundedSender;
use tokio::sync::watch::{self, Receiver};
use std::io::{BufWriter, Write};
use std::fs::File;

use crate::sensor_monitor::{SensorSubscription, SensorWatcher};
use iced::widget::{button, column, combo_box, container, row, text, text_input, pick_list};
use iced::{Element, Task};
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
    StartFieldStream,
    FieldStreamStarted,
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

async fn start_field_stream(client: HostClient<WireError>) -> () {
    client.send_resp::<StartFieldStream>(&()).await.unwrap()
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
    println!("{:#?}", val);

    let val = writer.write_all(&mut data);
    println!("{:#?}", val);
}

async fn stop_field_stream(client: HostClient<WireError>) -> () {
    client.send_resp::<StopFieldStream>(&()).await.unwrap()
}

fn update(context: &mut Context, message: Message) -> Task<Message> {

    println!("{:#?}", message);
    match message {
        Message::PortSelected(serial_port_info) => {
            context.sensor_watcher = Some(SensorWatcher::new(&serial_port_info));
            context.board_presence = BoardPresence::default();
            context.live_fits = BoardLiveFits::default();
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
        Message::RecievedField(sensor_field) => {
            context.live_fits.update(sensor_field.clone());
            context.ping_field = Some(sensor_field);
            Task::none()
        },
        Message::RecievedStreamField(sensor_field) => {
            context.live_fits.update(sensor_field.clone());
            context.ping_field = Some(sensor_field.clone());

            match &context.file_writer {
                Some(w) => {
                    let wr = w.clone();
                    Task::perform(
                        (move || {
                            async move {
                                println!("Test");
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

            if let Some(sw) = &context.sensor_watcher {
                let client = sw.get_client();

                Task::perform(
                    start_field_stream(client),
                    |()| Message::FieldStreamStarted,
                )
            } else {
                Task::none()
            }
        }
        Message::ResetBoardDisplacement(board_id) => {
            context.live_fits.reset_displacement(board_id);
            Task::none()
        }
        Message::FieldStreamStarted => {
            if let Some(sw) = &mut context.sensor_watcher {
                let mut client = sw.get_client();
                Task::future(field_subscribe(client)).and_then(|sub| {
                    Task::run(sub, |f| {
                        println!("Revieved field: {}", f.field.x.unwrap().value());
                        Message::RecievedStreamField(f)
                    })
                })
            }  else {
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
    let mlx_status_text = match &context.mlx_status {
        Some(status) => format!(
            "Status: ok={}, gain={}, resolution={}, hall_conf={}",
            status.ok, status.gain, status.resolution, status.hall_conf
        ),
        None => "Status: not read yet".to_string(),
    };

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
                        "Position: x={:.2} mm, y={:.2} mm, z={:.2} mm    Residual RMS: {:.2} uT    Sensors: {}",
                        result.position.0,
                        result.position.1,
                        result.position.2,
                        result.residual_rms,
                        result.n_sensors,
                    ),
                    None => format!(
                        "Waiting for completed frame; seen {}/16 sensors",
                        summary.seen_sensors,
                    ),
                };

                let board_card = container(
                    column![
                        row![
                            text(format!("Board {}", summary.board_id)),
                            button("Reset displacement zero")
                                .on_press(Message::ResetBoardDisplacement(summary.board_id)),
                        ]
                        .spacing(12),
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

    let mlx_widget = container(
        column![
            text("MLX90393 sensitivity"),
            row![
                text("Gain 0–7 (0 = max range; 7 = highest sensitivity, saturates sooner)"),
                text_input("0..7", &context.mlx_gain)
                    .on_input(Message::UpdateMlxGain),
            ],
            row![
                text("Resolution 0–3 (0 = finest/smallest range; 2–3 = largest range, 3 is coarser)"),
                text_input("0=16bit, 3=19bit", &context.mlx_resolution)
                    .on_input(Message::UpdateMlxResolution),
            ],
            row![
                text("Hall Conf"),
                pick_list(
                    vec![
                        MLX_HALL_CONF_DEFAULT_LABEL.to_string(),
                        MLX_HALL_CONF_FAST_LABEL.to_string(),
                    ],
                    Some(if context.mlx_hall_conf.is_empty() {
                        MLX_HALL_CONF_DEFAULT_LABEL.to_string()
                    } else {
                        context.mlx_hall_conf.clone()
                    }),
                    Message::UpdateMlxHallConf,
                )
            ],
            row![
                button("Get MLX sensitivity").on_press(Message::GetMlxSensitivity),
                button("Apply MLX sensitivity").on_press(Message::SetMlxSensitivity),
            ],
            text(mlx_status_text),
        ]
    );
    
    column![
        serial_selector,
        ping_widget,
        stream_widget,
        live_fit_widget,
        mlx_widget,
    ]
    .padding(10)
    .into()
}

#[tokio::main]
pub async fn main() -> iced::Result {
    iced::run(update, view)
}
