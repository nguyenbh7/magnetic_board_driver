use crate::SerialPortInfo;
use std::time::Duration;

use data_transfer::{self, messaging::MessageReader, rpc::{BoardFrame, BoardFrameTopic, PingEndpoint, SensorField, SingleFieldValue}};

use postcard_rpc::{
    header::VarSeqKind,
    host_client::{HostClient, HostErr, Subscription},
    standard_icd::{WireError, ERROR_PATH},
};
use sipper::{FutureExt, Stream};
use std::convert::Infallible;

pub struct SensorSubscription(Subscription<BoardFrame>);

pub struct SensorWatcher {
    client: HostClient<WireError>,
    port_info: SerialPortInfo,
}

#[derive(Debug)]
pub enum SensorError<E> {
    Comms(HostErr<WireError>),
    Endpoint(E),
}

impl<E> From<HostErr<WireError>> for SensorError<E> {
    fn from(value: HostErr<WireError>) -> Self {
        Self::Comms(value)
    }
}

trait FlattenErr {
    type Good;
    type Bad;
    fn flatten(self) -> Result<Self::Good, SensorError<Self::Bad>>;
}

impl<T, E> FlattenErr for Result<T, E> {
    type Good = T;
    type Bad = E;
    fn flatten(self) -> Result<Self::Good, SensorError<Self::Bad>> {
        self.map_err(SensorError::Endpoint)
    }
}

#[derive(Debug, Clone)]
pub struct MagneticData {
    pub field: data_transfer::conversions::MagneticField,
    pub position: (f32, f32, f32),
    pub time: u64,
}

impl MagneticData {
    pub fn from_message(message: &data_transfer::messaging::Message) -> Self {
        Self {
            field: message.field,
            position: message.position,
            time: message.time,
        }
    }
}

impl SensorWatcher {
    pub fn new(serial_port_info: &SerialPortInfo) -> SensorWatcher {
        let p = &serial_port_info.0.port_name;
        const SERIAL_BAUD_RATE: u32 = 921_600;

        let client =
            HostClient::<WireError>::new_serial_cobs(
                p,
                ERROR_PATH,
                64,
                SERIAL_BAUD_RATE,
                VarSeqKind::Seq2,
            );

        SensorWatcher {
            client,
            port_info: serial_port_info.clone(),
        }
    }

    pub fn port_info(&self) -> &SerialPortInfo {
        &self.port_info
    }

    pub fn get_client(&self) -> HostClient<WireError> {
        self.client.clone()
    }

    pub async fn get_single_sensor_field(&self, board: u32, sensor: u32) -> SensorField {
        self.client.send_resp::<SingleFieldValue>(&(board, sensor)).await.unwrap()
    }

    pub async fn subscribe(&mut self) -> Option<Subscription<BoardFrame>>{
        self.client.subscribe_exclusive::<BoardFrameTopic>(64).await.ok()
    }
    
}

impl SensorSubscription {

    pub fn new(subscription: Subscription<BoardFrame>) -> Self {
        Self(subscription)
    }
    
    pub async fn recv(&mut self) -> Option<BoardFrame>{
        self.0.recv().await
    }
}

impl Stream for SensorSubscription {
    type Item = BoardFrame;

    fn poll_next(self: std::pin::Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> std::task::Poll<Option<Self::Item>> {
        let fut = self.get_mut().recv();
        let value = std::pin::pin!(fut).poll(cx);
        value
    }
    
}

