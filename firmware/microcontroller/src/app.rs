#![no_std]
#![no_main]

use crate::handlers::{
    ping_handler,
    single_request_handler,
    stream_field,
    stop_stream,
    get_mlx_sensitivity_handler,
    set_mlx_sensitivity_handler,
    get_board_presence_handler,
};
use crate::mlx90393::sensorgroup::SensorGroup;
use data_transfer::rpc::{
    PingEndpoint,
    SingleFieldValue,
    StartFieldStream,
    StopFieldStream,
    GetMlxSensitivity,
    SetMlxSensitivity,
    GetBoardPresence,
    MlxSensitivityConfig,
    BoardPresence,
    ENDPOINT_LIST,
    TOPICS_IN_LIST,
    TOPICS_OUT_LIST,
};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::exti::ExtiInput;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use postcard_rpc::{
    define_dispatch,
    server::{
        impls::embedded_io_async_v0_7::{
            dispatch_impl::{WireRxBuf, spawn_fn, WireRxImpl, WireSpawnImpl},
            EioWireTx, WireStorage,
        },
        Server, SpawnContext,
    },
};
use embassy_sync::mutex::Mutex;
use crate::N;
use {defmt_rtt as _, panic_probe as _};

type SensorGroupDefault = Mutex<CriticalSectionRawMutex, SensorGroup<
        I2cDevice<
            'static,
            CriticalSectionRawMutex,
            embassy_stm32::i2c::I2c<
                'static,
                embassy_stm32::mode::Async,
                embassy_stm32::i2c::Master,
            >,
        >,
        Option<ExtiInput<'static>>,
    >>;

pub struct Context {
    pub sensor_groups: &'static [SensorGroupDefault; N],
    pub mlx_sensitivity: MlxSensitivityConfig,
    pub board_presence: BoardPresence,
}

pub struct SpawnCtx {
    pub sensor_groups: &'static [SensorGroupDefault; N]
}

impl SpawnContext for Context {
    type SpawnCtxt = SpawnCtx;
    fn spawn_ctxt(&mut self) -> Self::SpawnCtxt {
        SpawnCtx {
            sensor_groups: self.sensor_groups
        }
    }
}

pub type Rx = embassy_stm32::usart::RingBufferedUartRx<'static>;
pub type Tx = embassy_stm32::usart::UartTx<'static, embassy_stm32::mode::Async>;
pub type Storage = WireStorage<Rx, Tx, CriticalSectionRawMutex, 512, 512>;
pub type AppTx = EioWireTx<CriticalSectionRawMutex, Tx>;

/// AppRx is the type of our receiver, which is how we receive information from the client
pub type AppRx = WireRxImpl<Rx>;
/// AppServer is the type of our postcard-rpc server
pub type AppServer = Server<AppTx, AppRx, WireRxBuf, MyApp>;

pub static STORAGE: Storage = Storage::new();
define_dispatch! {
    app: MyApp;
    spawn_fn: spawn_fn;
    tx_impl: AppTx;
    spawn_impl: WireSpawnImpl;
    context: Context;

    endpoints: {
        list: ENDPOINT_LIST;

        | EndpointTy                | kind      | handler                       |
        | ----------                | ----      | -------                       |
        | PingEndpoint              | blocking  | ping_handler                  |
        | SingleFieldValue          | async     | single_request_handler        |
        | StartFieldStream          | spawn     | stream_field                  |
        | StopFieldStream           | blocking  | stop_stream                   |
        | GetMlxSensitivity         | async     | get_mlx_sensitivity_handler   |
        | SetMlxSensitivity         | async     | set_mlx_sensitivity_handler   |
        | GetBoardPresence          | async     | get_board_presence_handler    |
    };

    topics_in: {
        list: TOPICS_IN_LIST;

        | TopicTy                   | kind      | handler                       |
        | ----------                | ----      | -------                       |
    };

    topics_out: {
        list: TOPICS_OUT_LIST;
    };
}
