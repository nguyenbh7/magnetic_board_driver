#![no_std]
#![no_main]

use crate::handlers::{
    ping_handler,
    single_request_handler,
    stream_field,
    stop_stream,
    get_mlx_sensitivity_handler,
    set_mlx_sensitivity_handler,
};
use crate::mlx90393::sensorgroup::SensorGroup;
use data_transfer::rpc::{
    PingEndpoint,
    SingleFieldValue,
    StartFieldStream,
    StopFieldStream,
    GetMlxSensitivity,
    SetMlxSensitivity,
    MlxSensitivityConfig,
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
/// AppServer is the type of the postcard-rpc server we are using
pub type AppServer = Server<AppTx, AppRx, WireRxBuf, MyApp>;

pub static STORAGE: Storage = Storage::new();
define_dispatch! {
    // You can set the name of your app to any valid Rust type name. We use
    // "MyApp" here. You'll use this in `main` to create an instance of the
    // app.
    app: MyApp;
    // This chooses how we spawn functions. Here, we use the implementation
    // from the `embassy_usb_v0_4` implementation
    spawn_fn: spawn_fn;
    // This is our TX impl, which we aliased above
    tx_impl: AppTx;
    // This is our spawn impl, which also comes from `embassy_usb_v0_4`.
    spawn_impl: WireSpawnImpl;
    // This is the context type we defined above
    context: Context;

    // Endpoints are how we handle request/response pairs from the client.
    //
    // The "EndpointTy" are the names of the endpoints we defined in our ICD
    // crate. The "kind" is the kind of handler, which can be "blocking",
    // "async", or "spawn". Blocking endpoints will be called directly.
    // Async endpoints will also be called directly, but will be await-ed on,
    // allowing you to call async functions. Spawn endpoints will spawn an
    // embassy task, which allows for handling messages that may take some
    // amount of time to complete.
    //
    // The "handler"s are the names of the functions (or tasks) that will be
    // called when messages from this endpoint are received.
    endpoints: {
        // This list comes from our ICD crate. All of the endpoint handlers we
        // define below MUST be contained in this list.
        list: ENDPOINT_LIST;

        | EndpointTy                | kind      | handler                       |
        | ----------                | ----      | -------                       |
        | PingEndpoint              | blocking  | ping_handler                  |
        | SingleFieldValue          | async     | single_request_handler        |
        | StartFieldStream          | spawn     | stream_field                  |
        | StopFieldStream           | blocking  | stop_stream                   |
        | GetMlxSensitivity         | blocking  | get_mlx_sensitivity_handler   |
        | SetMlxSensitivity         | blocking  | set_mlx_sensitivity_handler   |
    };

    // Topics IN are messages we receive from the client, but that we do not reply
    // directly to. These have the same "kinds" and "handlers" as endpoints, however
    // these handlers never return a value
    topics_in: {
        // This list comes from our ICD crate. All of the topic handlers we
        // define below MUST be contained in this list.
        list: TOPICS_IN_LIST;

        | TopicTy                   | kind      | handler                       |
        | ----------                | ----      | -------                       |
    };

    // Topics OUT are the messages we send to the client whenever we'd like. Since
    // these are outgoing, we do not need to define handlers for them.
    topics_out: {
        // This list comes from our ICD crate.
        list: TOPICS_OUT_LIST;
        
    };
}
