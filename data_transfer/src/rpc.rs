

use postcard_rpc::{endpoints, topics, TopicDirection};
use postcard_schema::Schema;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use crate::conversions::MagneticField;

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Default, Serialize, Deserialize, Schema, PartialEq, MaxSize)]
pub struct SensorField {
    pub field: MagneticField,
    pub board_id: u16,
    pub position: (f32, f32, f32),
    pub address: u8,
    pub time: u64,
}

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Default, Serialize, Deserialize, Schema, PartialEq, MaxSize)]
pub struct MlxSensitivityConfig {
    pub gain: u8,       // 0..7
    pub resolution: u8, // 0..3, later map to 16/17/18/19-bit
    pub hall_conf: u8,  // 0 = 2-phase, 1 = 4-phase
}

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Default, Serialize, Deserialize, Schema, PartialEq, MaxSize)]
pub struct MlxSensitivityStatus {
    pub ok: bool,
    pub gain: u8,
    pub resolution: u8,
    pub hall_conf: u8,
}

endpoints! {
    list = ENDPOINT_LIST;
    | EndpointTy                | RequestTy     | ResponseTy            | Path              |
    | ----------                | ---------     | ----------            | ----              |
    | PingEndpoint              | u32           | u32                   | "ping"            |
    | SingleFieldValue          | (u32, u32)    | SensorField           | "bfield/single"   |
    | StartFieldStream          | ()            | ()                    | "bfield/start"    |
    | StopFieldStream           | ()            | ()                    | "bfield/stop"     |
    | GetMlxSensitivity         | ()            | MlxSensitivityStatus  | "mlx/sensitivity/get" |
    | SetMlxSensitivity  | MlxSensitivityConfig | MlxSensitivityStatus  | "mlx/sensitivity/set" |
}


topics! {
    list = TOPICS_IN_LIST;
    direction = TopicDirection::ToServer;
    | TopicTy                   | MessageTy     | Path              |
    | -------                   | ---------     | ----              |
}

topics! {
    list = TOPICS_OUT_LIST;
    direction = TopicDirection::ToClient;
    | TopicTy                   | MessageTy     | Path              | Cfg                           |
    | -------                   | ---------     | ----              | ---                           |
    | MagneticTopic             | SensorField   | "bfield/data"     |                               |
}
