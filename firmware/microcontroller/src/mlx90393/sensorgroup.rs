use data_transfer::{
    conversions::MagneticField,
    messaging::{self, Writable},
    rpc,
};
use defmt::Format;
use embassy_futures::join::join_array;
use embassy_stm32::exti::ExtiInput;
use embassy_time::{Instant, Timer};
use embedded_hal_async::{digital::Wait, i2c::I2c};
use embedded_io::Write;

use super::sensor::{Status, MLX90393};

pub struct Sensor<I, P> {
    pub position: (f32, f32, f32),
    mlx: MLX90393<I, P>,
}

#[derive(Debug, Format)]
pub enum Error {
    DataError(messaging::Error),
    StatusError(Status),
}

impl From<messaging::Error> for Error {
    fn from(value: messaging::Error) -> Self {
        Self::DataError(value)
    }
}

impl<I: I2c, P: Wait> Sensor<I, Option<P>> {
    pub async fn is_present(&mut self) -> bool {
        self.mlx.probe().await
    }
    pub async fn set_sensitivity(
        &mut self,
        gain: u8,
        resolution: u8,
        hall_conf: u8,
    ) -> bool {
        self.mlx
            .set_sensitivity_registers(gain, resolution, hall_conf)
            .await
    }
    pub async fn set_sensitivity_fast(
        &mut self,
        gain: u8,
        resolution: u8,
        hall_conf: u8,
    ) -> bool {
        self.mlx
            .set_sensitivity_registers_fast(gain, resolution, hall_conf)
            .await
    }
    pub async fn read_sensitivity(&mut self) -> Option<(u8, u8, u8)> {
        self.mlx.read_sensitivity_values().await
    }
    pub async fn new(address: u8, i2c: I, position: (f32, f32, f32)) -> Self
        where {
        let mlx = MLX90393::new(address, None, i2c);
        Self::from_mlx(mlx, position).await
    }

    pub async fn from_mlx(mlx: MLX90393<I, Option<P>>, position: (f32, f32, f32)) -> Self {
        let mut sensor = Self { mlx, position };
        sensor.mlx.reset().await;
        Timer::after_micros(500).await;
        sensor.mlx.set_measurement_configuration().await;
        //sensor.mlx.set_burst::<true, true, true, true>().await;
        sensor
    }

    pub async fn send_message<W: embedded_io_async::Write>(
        &mut self,
        writer: &mut W,
    ) -> Result<messaging::Message, Error> {
        self.mlx
            .set_single_measurmenet::<true, true, true, true>()
            .await;
        //Timer::after_millis(50).await;
        let (status, field) = self.mlx.get_field::<true, true, true, true>().await;
        //if status.is_some_and(|val| !val.burst_mode) {
        //self.mlx.set_burst::<true, true, true, true>().await;
        //}
        if status.error {
            return Err(Error::StatusError(status));
        }
        let time = Instant::now().as_micros();
        let message =
            field.map(|f| messaging::Message::new(f, self.position, self.mlx.address, time));
        let message = message.ok_or(data_transfer::messaging::Error::FailedRead)?;
        message.write_to(writer).await?;
        Ok(message)
    }

    pub async fn get_message(&mut self) -> Result<messaging::Message, ()> {
        self.mlx
            .set_single_measurmenet::<true, true, true, true>()
            .await;
        Timer::after_millis(50).await;
        let (_status, field) = self.mlx.get_field::<true, true, true, true>().await;
        //debug!("{:#?}", status);
        //if status.is_some_and(|val| !val.burst_mode) {
        //self.mlx.set_burst::<true, true, true, true>().await;
        //}
        let time = Instant::now().as_micros();
        let message =
            field.map(|f| messaging::Message::new(f, self.position, self.mlx.address, time));
        message.ok_or(())
    }

    pub async fn set_burst_mode(&mut self) {
        self.mlx.set_burst::<true, true, true, true>().await
    }
}

impl<'a, I: I2c> Sensor<I, Option<ExtiInput<'a>>>
where
    <I as embedded_hal_async::i2c::ErrorType>::Error: Format,
{
    pub async fn new_stm(address: u8, position: (f32, f32, f32), i2c: I) -> Self {
        Self::new(address, i2c, position).await
    }
}

trait SendValues {
    async fn send_message<W: Write>(
        &mut self,
        writer: &mut W,
    ) -> Result<MagneticField, data_transfer::messaging::Error>;
}

pub struct SensorBuilder {
    pub address: u8,
    pub position: (f32, f32, f32),
}

impl<'a> SensorBuilder {
    pub fn new_stm(address: u8, position: (f32, f32, f32)) -> Self {
        Self { address, position }
    }

    pub async fn with_i2c<I: I2c>(&mut self, i2c: I) -> Sensor<I, Option<ExtiInput<'a>>>
    where
        <I as embedded_hal_async::i2c::ErrorType>::Error: Format,
    {
        Sensor::new_stm(self.address, self.position, i2c).await
    }
}

pub struct SensorGroup<I, P, const N: usize = 16> {
    pub board_id: u16,
    pub sensors: [Sensor<I, P>; N],
}

impl<I: I2c, P: Wait, const N: usize> SensorGroup<I, Option<P>, N> {
    pub async fn detect_sensor_mask(&mut self) -> u16 {
        let mut mask = 0u16;

        for sensor_index in 0..self.num_sensors() {
            if self.sensors[sensor_index].is_present().await {
                mask |= 1u16 << sensor_index;
            }
        }

        mask
    }
    pub async fn set_sensitivity_all(
        &mut self,
        gain: u8,
        resolution: u8,
        hall_conf: u8,
    ) -> bool {
        let mut all_ok = true;

        for sensor in self.sensors.iter_mut() {
            let ok = sensor
                .set_sensitivity(gain, resolution, hall_conf)
                .await;
            all_ok = all_ok && ok;
        }

        all_ok
    }

    pub async fn set_sensitivity_all_fast(
        &mut self,
        gain: u8,
        resolution: u8,
        hall_conf: u8,
    ) -> bool {
        let mut all_ok = true;

        for sensor in self.sensors.iter_mut() {
            let ok = sensor
                .set_sensitivity_fast(gain, resolution, hall_conf)
                .await;

            all_ok = all_ok && ok;
        }

        all_ok
    }

    pub async fn get_message(&mut self, index: usize) -> Result<rpc::SensorField, ()> {
        let sensor = self.sensors.get_mut(index).ok_or(())?;
        let message = sensor.get_message().await?;
        Ok(rpc::SensorField {
            address: message.address,
            field: message.field,
            position: message.position,
            time: message.time,
            board_id: self.board_id,
        })
    }

    pub fn num_sensors(&self) -> usize {
        N
    }
}

pub struct SensorGroupBuilder<const N: usize> {
    board_id: u16,
    sensor_builders: [SensorBuilder; N],
}

impl<'a, const N: usize> SensorGroupBuilder<N> {
    pub fn new_stm(board_id: u16, sensor_builders: [SensorBuilder; N]) -> Self {
        Self {
            board_id,
            sensor_builders,
        }
    }
    pub async fn with_i2c<I: I2c>(
        &mut self,
        i2cs: [I; N],
    ) -> SensorGroup<I, Option<ExtiInput<'a>>, N>
    where
        <I as embedded_hal_async::i2c::ErrorType>::Error: Format,
    {
        let sensors_builders = self.sensor_builders.each_mut();

        let sensors_future = core::array::from_fn(async |i| {
            //SAFETY: Zipping arrays together together
            let sensor_builder = unsafe { core::ptr::read(&sensors_builders[i]) };
            let i2c = unsafe { core::ptr::read(&i2cs[i]) };
            sensor_builder.with_i2c(i2c).await
        });

        let sensors = join_array(sensors_future).await;

        SensorGroup {
            board_id: self.board_id,
            sensors,
        }
    }
}
