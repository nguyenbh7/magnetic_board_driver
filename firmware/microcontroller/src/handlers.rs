use crate::app::{Context, SpawnCtx, AppTx, SensorGroupDefault};
use defmt::info;
use postcard_rpc::header::VarHeader;
use postcard_rpc::server::Sender;
use data_transfer::rpc::{
    SensorField,
    BoardFrame,
    BoardFrameSample,
    BoardFrameTopic,
    StartFieldStream,
    MlxSensitivityConfig,
    MlxSensitivityStatus,
    MlxTimingStatus,
    BoardPresence,
};
use embassy_executor;
use portable_atomic::{AtomicBool, Ordering};

use embassy_futures::join::join_array;
use embassy_time::{Instant, Timer};
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

pub async fn get_mlx_timing_handler(
    context: &mut Context,
    _header: VarHeader,
    _rqst: (),
) -> MlxTimingStatus {
    info!("get mlx timing");

    let presence = detect_board_presence_from_context(context).await;
    context.board_presence = presence.clone();

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

            let timing = group.sensors[sensor_index].read_timing().await;
            return MlxTimingStatus {
                ok: true,
                board_id: group.board_id,
                sensor_index: sensor_index as u8,
                register_02_msb: timing.register_02_msb,
                register_02_lsb: timing.register_02_lsb,
                osr: timing.osr,
                dig_filt: timing.dig_filt,
                osr2: timing.osr2,
                magnetic_axis_conversion_time_us:
                    timing.magnetic_axis_conversion_time_us,
                temperature_conversion_time_us:
                    timing.temperature_conversion_time_us,
                xyz_t_single_measurement_time_us:
                    timing.xyz_t_single_measurement_time_us,
            };
        }
    }

    MlxTimingStatus::default()
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



async fn acquire_board_frame(
    group: &SensorGroupDefault,
    sensor_mask: u16,
    frame_id: u32,
) -> BoardFrame {
    if sensor_mask == 0 {
        return BoardFrame::default();
    }

    let mut sg = group.lock().await;
    let mut measurement_times_us = [0u64; 16];
    let mut max_conversion_time_us = 0u64;

    // Trigger all present sensors on this board first so their conversions
    // overlap. When multiple boards run this helper concurrently, independent
    // I2C peripherals proceed in parallel; I2cDevice serializes only accesses
    // that truly share one physical bus.
    for sensor_index in 0..sg.num_sensors() {
        if sensor_mask & (1u16 << sensor_index) == 0 {
            continue;
        }

        let conversion_time_us = sg
            .predicted_measurement_time_us(sensor_index)
            .unwrap_or(2_000);
        max_conversion_time_us = max_conversion_time_us.max(conversion_time_us);

        if sg.trigger_measurement(sensor_index).await.is_err() {
            continue;
        }

        measurement_times_us[sensor_index] =
            Instant::now().as_micros() + conversion_time_us / 2;
    }

    if max_conversion_time_us > 0 {
        Timer::after_micros(max_conversion_time_us).await;
    }

    let base_time_us = measurement_times_us
        .iter()
        .copied()
        .filter(|time| *time != 0)
        .min()
        .unwrap_or(0);

    let mut frame = BoardFrame::default();
    frame.board_id = sg.board_id;
    frame.frame_id = frame_id;
    frame.base_time_us = base_time_us;

    for sensor_index in 0..sg.num_sensors() {
        if sensor_mask & (1u16 << sensor_index) == 0 {
            continue;
        }

        let measurement_time_us = measurement_times_us[sensor_index];
        if measurement_time_us == 0 {
            continue;
        }

        let Ok(message) = sg
            .read_message_at(sensor_index, measurement_time_us, frame_id)
            .await
        else {
            continue;
        };

        let time_offset_us = message
            .time
            .saturating_sub(base_time_us)
            .min(u64::from(u16::MAX)) as u16;

        frame.samples[sensor_index] = BoardFrameSample {
            bx_ut: message
                .field
                .x
                .map(|value| value.value() as f32)
                .unwrap_or(f32::NAN),
            by_ut: message
                .field
                .y
                .map(|value| value.value() as f32)
                .unwrap_or(f32::NAN),
            bz_ut: message
                .field
                .z
                .map(|value| value.value() as f32)
                .unwrap_or(f32::NAN),
            temperature_c: message
                .field
                .t
                .map(|value| value.value() as f32)
                .unwrap_or(f32::NAN),
            time_offset_us,
        };
        frame.sensor_mask |= 1u16 << sensor_index;
    }

    frame
}

#[embassy_executor::task]
pub async fn stream_field(
    context: SpawnCtx,
    header: VarHeader,
    _rqst: (),
    sender: Sender<AppTx>,
) {
    let mut seq = 0u8;
    let mut frame_ids = [0u32; N];

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
        // Acquire all detected boards concurrently. Board 0 uses I2C1 while
        // Boards 1 and 2 share I2C3, so this overlaps the independent bus and
        // also lets B1/B2 conversion waits overlap while their actual bus
        // transactions remain serialized by I2cDevice.
        let frames: [BoardFrame; N] = join_array(core::array::from_fn(|board_index| {
            let sensor_mask =
                if presence.board_mask & (1u8 << board_index) != 0 {
                    presence.sensor_masks[board_index]
                } else {
                    0
                };

            acquire_board_frame(
                &context.sensor_groups[board_index],
                sensor_mask,
                frame_ids[board_index],
            )
        }))
        .await;

        // Keep postcard-RPC publishing serialized. This isolates the timing
        // experiment to sensor-side concurrency and avoids concurrent writes
        // through one transport sender.
        for board_index in 0..N {
            if presence.board_mask & (1u8 << board_index) == 0 {
                continue;
            }

            let frame = &frames[board_index];

            if frame.sensor_mask != 0
                && sender
                    .publish::<BoardFrameTopic>(seq.into(), frame)
                    .await
                    .is_err()
            {
                defmt::error!("Send error!");
            }

            seq = seq.wrapping_add(1);
            frame_ids[board_index] = frame_ids[board_index].wrapping_add(1);
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
