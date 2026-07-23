use crate::app::{Context, SpawnCtx, AppTx};
use defmt::info;
use postcard_rpc::header::VarHeader;
use postcard_rpc::server::Sender;
use data_transfer::rpc::{
    SensorField,
    MagneticTopic,
    StartFieldStream,
    MlxSensitivityConfig,
    MlxSensitivityStatus,
    BoardPresence,
};
use data_transfer::rpc::GetBoardPresence;
use embassy_executor;
use portable_atomic::{AtomicBool, Ordering};

//use embassy_futures::join::join_array;
use embassy_time::{Duration, Ticker};
use crate::N; 
const BOARD_PRESENT_MIN_SENSORS: u32 = 1;
pub fn ping_handler(_context: &mut Context, _header: VarHeader, rqst: u32) -> u32 {
    info!("ping");
    rqst
}

pub fn get_mlx_sensitivity_handler(
    context: &mut Context,
    _header: VarHeader,
    _rqst: (),
) -> MlxSensitivityStatus {
    info!("get mlx sensitivity");

    MlxSensitivityStatus {
        ok: true,
        gain: context.mlx_sensitivity.gain,
        resolution: context.mlx_sensitivity.resolution,
        hall_conf: context.mlx_sensitivity.hall_conf,
    }
}

pub async fn get_board_presence_handler(
    context: &mut Context,
    _header: VarHeader,
    _rqst: (),
) -> BoardPresence {
    let presence = detect_board_presence_from_context(context).await;
    context.board_presence = presence.clone();
    presence
}

pub async fn set_mlx_sensitivity_handler(
    context: &mut Context,
    _header: VarHeader,
    rqst: MlxSensitivityConfig,
) -> MlxSensitivityStatus {
    info!("set mlx sensitivity");

    let valid =
        rqst.gain <= 7
        && rqst.resolution <= 3
        && matches!(rqst.hall_conf, 0x0 | 0xC);

    if !valid {
        return MlxSensitivityStatus {
            ok: false,
            gain: rqst.gain,
            resolution: rqst.resolution,
            hall_conf: rqst.hall_conf,
        };
    }

    // CURRENT CONNECTED-BOARD MODE:
    // Your app output is board_id=1, so only apply to board index 1.
    // This updates all 16 sensors on that one connected board.
    let connected_board_index = 1usize;

    let mut all_ok = true;
    let mut representative_readback = (255, 255, 255);

    {
        let mut group = context.sensor_groups[connected_board_index].lock().await;

        for sensor_index in 0..group.num_sensors() {
            let sensor = &mut group.sensors[sensor_index];

            let write_ok = sensor
                .set_sensitivity(rqst.gain, rqst.resolution, rqst.hall_conf)
                .await;

            let readback = sensor
                .read_sensitivity()
                .await
                .unwrap_or((255, 255, 255));

            let readback_ok =
                readback.0 == rqst.gain
                && readback.1 == rqst.resolution
                && readback.2 == rqst.hall_conf;

            if sensor_index == 0 {
                representative_readback = readback;
            }

            all_ok = all_ok && write_ok && readback_ok;
        }
    }

    if all_ok {
        context.mlx_sensitivity = rqst.clone();
    }

    // Returned values are readback from board 1 sensor 0.
    MlxSensitivityStatus {
        ok: all_ok,
        gain: representative_readback.0,
        resolution: representative_readback.1,
        hall_conf: representative_readback.2,
    }
}

pub fn stop_stream(context: &mut Context, _header: VarHeader, _rqst: ()) {
    let was_busy = core::array::from_fn::<_, N, _>(|i| context.sensor_groups[i].try_lock().is_err()).contains(&true);
    if was_busy {
        STOP.store(true, Ordering::Release);
    } ;
//    was_busy
}

pub async fn single_request_handler(context: &mut Context, _header: VarHeader, rqst: (u32, u32)) -> SensorField {
    let board_index = usize::try_from(rqst.0).or(Err(())).unwrap();
    let sensor_index = usize::try_from(rqst.1).or(Err(())).unwrap();
    let mut sensor = context.sensor_groups.get(board_index).unwrap().lock().await;

    let message = sensor.get_message(sensor_index).await;
    info!("{}", message);
    message.unwrap()
}

pub static STOP: AtomicBool = AtomicBool::new(false);



#[embassy_executor::task]
pub async fn stream_field(
    context: SpawnCtx,
    header: VarHeader,
    _rqst: (),
    sender: Sender<AppTx>,
) {
    let mut seq = 0u8;
    let mut ticker = Ticker::every(Duration::from_millis(0));

    if sender
        .reply::<StartFieldStream>(header.seq_no, &())
        .await
        .is_err()
    {
        defmt::error!("Failed to reply, stopping accel");
        return;
    }

    let presence = detect_board_presence_from_spawn(&context).await;

    info!(
        "Detected boards: mask={} sensor_masks={:?}",
        presence.board_mask,
        presence.sensor_masks
    );

    while !STOP.load(Ordering::Acquire) {
        for board_index in 0..N {
            if presence.board_mask & (1u8 << board_index) == 0 {
                continue;
            }

            let sensor_mask = presence.sensor_masks[board_index];
            let mut sg = context.sensor_groups[board_index].lock().await;

            for sensor_index in 0..sg.num_sensors() {
                if sensor_mask & (1u16 << sensor_index) == 0 {
                    continue;
                }

                ticker.next().await;

                let Ok(message) = sg.get_message(sensor_index).await else {
                    continue;
                };

                info!("{}", message);

                if sender
                    .publish::<MagneticTopic>(seq.into(), &message)
                    .await
                    .is_err()
                {
                    defmt::error!("Send error!");
                    break;
                }

                seq = seq.wrapping_add(1);
            }
        }
    }

    STOP.store(false, Ordering::Release);
}

async fn detect_board_presence_from_context(context: &mut Context) -> BoardPresence {
    let mut presence = BoardPresence::default();

    for board_index in 0..N {
        let mut group = context.sensor_groups[board_index].lock().await;
        let sensor_mask = group.detect_sensor_mask().await;
        let present_count = sensor_mask.count_ones();

        presence.sensor_masks[board_index] = sensor_mask;

        if present_count >= BOARD_PRESENT_MIN_SENSORS {
            presence.board_mask |= 1u8 << board_index;
        }
    }

    presence
}

async fn detect_board_presence_from_spawn(context: &SpawnCtx) -> BoardPresence {
    let mut presence = BoardPresence::default();

    for board_index in 0..N {
        let mut group = context.sensor_groups[board_index].lock().await;
        let sensor_mask = group.detect_sensor_mask().await;
        let present_count = sensor_mask.count_ones();

        presence.sensor_masks[board_index] = sensor_mask;

        if present_count >= BOARD_PRESENT_MIN_SENSORS {
            presence.board_mask |= 1u8 << board_index;
        }
    }

    presence
}
