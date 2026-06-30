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
};
use embassy_executor;
use portable_atomic::{AtomicBool, Ordering};

use embassy_futures::join::join_array;
use embassy_time::{Duration, Ticker};
use crate::N; 
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

pub fn set_mlx_sensitivity_handler(
    context: &mut Context,
    _header: VarHeader,
    rqst: MlxSensitivityConfig,
) -> MlxSensitivityStatus {
    info!("set mlx sensitivity");

    let valid =
        rqst.gain <= 7
        && rqst.resolution <= 3
        && matches!(rqst.hall_conf, 0x0 | 0xC);

    if valid {
        context.mlx_sensitivity = rqst.clone();
    }

    MlxSensitivityStatus {
        ok: valid,
        gain: rqst.gain,
        resolution: rqst.resolution,
        hall_conf: rqst.hall_conf,
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
    let seq = 0u8;
    let mut ticker = Ticker::every(Duration::from_millis(0));
    if sender
        .reply::<StartFieldStream>(header.seq_no, &())
        .await
        .is_err()
    {
        defmt::error!("Failed to reply, stopping accel");
        return;
    }
    while !STOP.load(Ordering::Acquire) {
        {

            let sensors = core::array::from_fn( |i| context.sensor_groups[i].lock());
            let mut sensors: [_; N] = join_array(sensors).await;

            for sg in &mut sensors {
                for i in 0..sg.num_sensors() {
                    ticker.next().await;
                    let message = sg.get_message(i).await.unwrap();
                    info!("{}", message);
                    if sender
                        .publish::<MagneticTopic>(seq.into(), &message)
                        .await
                        .is_err()
        {
            defmt::error!("Send error!");
            break;
        }
                    seq.wrapping_add(1);
                }

            }
        }
    }

    STOP.store(false, Ordering::Release);
}

 
