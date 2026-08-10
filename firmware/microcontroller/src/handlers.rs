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

pub async fn get_mlx_sensitivity_handler(
    context: &mut Context,
    _header: VarHeader,
    _rqst: (),
) -> MlxSensitivityStatus {
    info!("get mlx sensitivity");

    // Re-detect first so the read reflects the hardware that is connected now,
    // not a cached startup assumption.
    let presence = detect_board_presence_from_context(context).await;
    context.board_presence = presence.clone();

    let mut any_sensor = false;
    let mut all_consistent = true;
    let mut representative_readback: Option<(u8, u8, u8)> = None;

    for board_index in 0..N {
        if presence.board_mask & (1u8 << board_index) == 0 {
            continue;
        }

        let sensor_mask = presence.sensor_masks[board_index];
        let mut group = context.sensor_groups[board_index].lock().await;

        for sensor_index in 0..group.num_sensors() {
            if sensor_mask & (1u16 << sensor_index) == 0 {
                continue;
            }

            any_sensor = true;
            let readback = group.sensors[sensor_index].read_sensitivity().await;

            match (representative_readback, readback) {
                (None, Some(values)) => representative_readback = Some(values),
                (Some(expected), Some(values)) if expected == values => {},
                (_, _) => all_consistent = false,
            }
        }
    }

    let ok = any_sensor && all_consistent && representative_readback.is_some();

    if let Some((gain, resolution, hall_conf)) = representative_readback {
        if ok {
            context.mlx_sensitivity = MlxSensitivityConfig {
                gain,
                resolution,
                hall_conf,
            };
        }

        MlxSensitivityStatus {
            ok,
            gain,
            resolution,
            hall_conf,
        }
    } else {
        MlxSensitivityStatus {
            ok: false,
            gain: context.mlx_sensitivity.gain,
            resolution: context.mlx_sensitivity.resolution,
            hall_conf: context.mlx_sensitivity.hall_conf,
        }
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

    // Re-detect immediately before applying sensitivity so this works
    // for whatever boards are actually connected now.
    let presence = detect_board_presence_from_context(context).await;
    context.board_presence = presence.clone();

    let mut any_sensor = false;
    let mut all_ok = true;
    let mut representative_readback: Option<(u8, u8, u8)> = None;

    for board_index in 0..N {
        if presence.board_mask & (1u8 << board_index) == 0 {
            continue;
        }

        let sensor_mask = presence.sensor_masks[board_index];
        let mut group = context.sensor_groups[board_index].lock().await;

        for sensor_index in 0..group.num_sensors() {
            if sensor_mask & (1u16 << sensor_index) == 0 {
                continue;
            }

            any_sensor = true;

            let sensor = &mut group.sensors[sensor_index];

            let write_ok = sensor
                .set_sensitivity(rqst.gain, rqst.resolution, rqst.hall_conf)
                .await;

            let readback = sensor.read_sensitivity().await;

            if representative_readback.is_none() {
                representative_readback = readback;
            }

            let readback_ok = matches!(
                readback,
                Some((read_gain, read_resolution, read_hall_conf))
                    if read_gain == rqst.gain
                        && read_resolution == rqst.resolution
                        && read_hall_conf == rqst.hall_conf
            );

            all_ok = all_ok && write_ok && readback_ok;
        }
    }

    let ok = any_sensor && all_ok;

    if ok {
        context.mlx_sensitivity = rqst.clone();
    }

    let readback = representative_readback.unwrap_or((
        rqst.gain,
        rqst.resolution,
        rqst.hall_conf,
    ));

    MlxSensitivityStatus {
        ok,
        gain: readback.0,
        resolution: readback.1,
        hall_conf: readback.2,
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
