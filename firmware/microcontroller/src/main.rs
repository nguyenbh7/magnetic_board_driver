#![no_std]
#![no_main]

pub mod app;
pub mod handlers;
pub mod mlx90393;


use static_cell::StaticCell;
use app::{Context, MyApp, STORAGE, AppServer};
use postcard_rpc::server::{Server, Dispatch};


use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
    watch::Watch,
};

use defmt::{debug, info};
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, i2c,
    peripherals::{self},
};
use embassy_stm32::{
    rcc::{AHB5Prescaler, AHBPrescaler, APBPrescaler, Sysclk, VoltageScale},
    time::khz,
};

use embassy_stm32::rcc::{PllDiv, PllMul, PllPreDiv, PllSource};
use embassy_time::Duration;

use embassy_stm32::usart;
use mlx90393::sensorgroup::{SensorGroupBuilder, SensorBuilder};

use static_cell::{ConstStaticCell};

use embassy_stm32::exti::ExtiInput;
use {defmt_rtt as _, panic_probe as _};
use crate::mlx90393::sensorgroup::SensorGroup;
bind_interrupts!(
    struct Irqs {
        I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
        I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
        I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
        USART1 => usart::InterruptHandler<peripherals::USART1>;

    }
);

pub const N: usize = 3;

const SENSOR_GRID_SIDE_LENGTH_MM: f32 = 13.5;
const SENSOR_GRID_POINTS_PER_SIDE: usize = 4;
const SENSOR_GRID_PITCH_MM: f32 =
    SENSOR_GRID_SIDE_LENGTH_MM / (SENSOR_GRID_POINTS_PER_SIDE as f32 - 1.0);

/// Return the verified physical position for one acquisition-order sensor index.
///
/// Sensor indices 0..15 correspond to addresses 0x0C..0x1B on boards A/B.
/// The physical ordering matches the original Pi setup and the validated
/// `gk_analysis` geometry:
///
/// row y=-6.75: x=+6.75, +2.25, -2.25, -6.75
/// row y=-2.25: x=+6.75, +2.25, -2.25, -6.75
/// row y=+2.25: x=+6.75, +2.25, -2.25, -6.75
/// row y=+6.75: x=+6.75, +2.25, -2.25, -6.75
fn sensor_grid_position_mm(index: usize) -> (f32, f32, f32) {
    let row = index / SENSOR_GRID_POINTS_PER_SIDE;
    let column = index % SENSOR_GRID_POINTS_PER_SIDE;
    let half_side = SENSOR_GRID_SIDE_LENGTH_MM / 2.0;

    let x = half_side - SENSOR_GRID_PITCH_MM * column as f32;
    let y = -half_side + SENSOR_GRID_PITCH_MM * row as f32;

    (x, y, 0.0)
}

type SensorGroupDefault =Mutex<CriticalSectionRawMutex, SensorGroup<
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




#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("BOOT 00: main entered");
    static I2C_BUS1: StaticCell<
        Mutex<
            CriticalSectionRawMutex,
            i2c::I2c<'_, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>,
        >,
        > = StaticCell::new();

    static I2C_BUS3: StaticCell<
        Mutex<
            CriticalSectionRawMutex,
            i2c::I2c<'_, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>,
        >,
    > = StaticCell::new();

    static UART_TX_BUS: StaticCell<
        Mutex<CriticalSectionRawMutex, usart::Uart<embassy_stm32::mode::Async>>,
    > = StaticCell::new();
    static UART_RX_BUS: StaticCell<
        Mutex<CriticalSectionRawMutex, usart::RingBufferedUartRx>,
    > = StaticCell::new();
    static UART_RX_BUFFER: StaticCell<Mutex<CriticalSectionRawMutex, [u8; 256]>> = StaticCell::new();
    static UART_INTERFACE: StaticCell<usart::Uart<'_, embassy_stm32::mode::Async>> =
        StaticCell::new();

    static SENSOR_GROUPS: StaticCell<[SensorGroupDefault; N]> = StaticCell::new();
    static CONTEXT: StaticCell<Context> = StaticCell::new();

    
    static READY: Watch<CriticalSectionRawMutex, (), 16> = Watch::new();




    
    let mut config = embassy_stm32::Config::default();

    // Fine-tune PLL1 dividers/multipliers

    config.rcc.pll1 = Some(embassy_stm32::rcc::Pll {
        source: PllSource::HSI,

        prediv: PllPreDiv::DIV1, // PLLM = 1 → HSI / 1 = 16 MHz

        mul: PllMul::MUL30, // PLLN = 30 → 16 MHz * 30 = 480 MHz VCO

        divr: Some(PllDiv::DIV5), // PLLR = 5 → 96 MHz (Sysclk)

        // divq: Some(PllDiv::DIV10), // PLLQ = 10 → 48 MHz (NOT USED)
        divq: None,

        divp: Some(PllDiv::DIV30), // PLLP = 30 → 16 MHz (USBOTG)

        frac: Some(0), // Fractional part (enabled)
    });

    config.rcc.ahb_pre = AHBPrescaler::DIV1;

    config.rcc.apb1_pre = APBPrescaler::DIV1;

    config.rcc.apb2_pre = APBPrescaler::DIV1;

    config.rcc.apb7_pre = APBPrescaler::DIV1;

    config.rcc.ahb5_pre = AHB5Prescaler::DIV4;

    // voltage scale for max performance

    config.rcc.voltage_scale = VoltageScale::RANGE1;

    // route PLL1_P into the USB‐OTG‐HS block

    config.rcc.sys = Sysclk::PLL1_R;
    info!("BOOT 01: before embassy_stm32::init");
    let p = embassy_stm32::init(config);
    info!("BOOT 02: STM32 init complete");

    let UART_RX = p.PA8;
    let UART_TX = p.PB12;

    info!("BOOT 03: before USART1 init");
    let uart_interface = usart::Uart::new(
        p.USART1,
        UART_RX,
        UART_TX,
        Irqs,
        p.GPDMA1_CH2,
        p.GPDMA1_CH3,
        usart::Config::default(),
    )
        .unwrap();
    info!("BOOT 04: USART1 init complete");

    let (uart_tx, uart_rx) = uart_interface.split();
    info!("BOOT 05: USART1 split complete");

    let rx_buffer = [0u8; 256];
    let rx_buffer = UART_RX_BUFFER.init(Mutex::new(rx_buffer));
    info!("BOOT 06: UART RX buffer initialized");

    let uart_rx = uart_rx.into_ring_buffered(rx_buffer.get_mut());
    info!("BOOT 07: UART RX ring buffer initialized");
    
    //let uart_rx_bus_mutex = Mutex::new(uart_rx);
    //let uart_rx_bus = UART_RX_BUS.init(uart_rx_bus_mutex);
    let mut i2c_config = i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(500);
    i2c_config.frequency = khz(100);
    i2c_config.sda_pullup = true;
    i2c_config.scl_pullup = true;
    info!("BOOT 08: I2C config prepared");


    info!("BOOT 09: before I2C1 init");
    let i2c_bus1 = {
        let i2c_peri = p.I2C1;
        
        let sda = p.PB1;
        let scl = p.PB2;
        let rx_gpdma = p.GPDMA1_CH4;
        let tx_gpdma = p.GPDMA1_CH5;


        let i2cport = i2c::I2c::new(
            i2c_peri,
            scl,
            sda,
            Irqs,
            rx_gpdma,
            tx_gpdma,
            i2c_config,
        );
        let i2c_bus = Mutex::new(i2cport);
        I2C_BUS1.init(i2c_bus)
    };
    info!("BOOT 10: I2C1 init complete");

    info!("BOOT 11: before I2C3 init");
    let i2c_bus3 = {

        let i2c_peri = p.I2C3;
        
        let sda = p.PA7;
        let scl = p.PA6;
        let rx_gpdma = p.GPDMA1_CH0;
        let tx_gpdma = p.GPDMA1_CH1;
        
        let i2cport = i2c::I2c::new(
            i2c_peri,
            scl,
            sda,
            Irqs,
            rx_gpdma,
            tx_gpdma,
            i2c_config,
        );
        let i2c_bus = Mutex::new(i2cport);
        I2C_BUS3.init(i2c_bus)
    };
    info!("BOOT 12: I2C3 init complete");
    
    
    let i2c_devices_a = {
        core::array::from_fn(|_| I2cDevice::new(i2c_bus1))
    };

    let i2c_devices_b = {
        core::array::from_fn(|_| I2cDevice::new(i2c_bus3))
    };

    let i2c_devices_c = {
        core::array::from_fn(|_| I2cDevice::new(i2c_bus3))
    };
    info!("BOOT 13: shared I2C devices created");

    let positions: [_; 16] = core::array::from_fn(sensor_grid_position_mm);
    info!("BOOT 14: sensor positions created");
    
    let sensor_groups = {
        let sensor_builders_a: [_; 16] = core::array::from_fn(|i| SensorBuilder::new_stm(0x0C+(i as u8), positions[i]));
        let sensor_builders_b: [_; 16] = core::array::from_fn(|i| SensorBuilder::new_stm(0x0C+(i as u8), positions[i]));
        let sensor_builders_c: [_; 16] = core::array::from_fn(|i| SensorBuilder::new_stm((0x0C+(i as u8)) ^ 0b01000000, positions[i]));
        let mut sensor_group_builder_a = SensorGroupBuilder::new_stm(0, sensor_builders_a);
        let mut sensor_group_builder_b = SensorGroupBuilder::new_stm(1, sensor_builders_b);
        let mut sensor_group_builder_c = SensorGroupBuilder::new_stm(2, sensor_builders_c);

        info!("BOOT 15: before sensor group A construction");
        let sensor_group_a = sensor_group_builder_a.with_i2c(i2c_devices_a).await;
        info!("BOOT 16: sensor group A construction complete");

        info!("BOOT 17: before sensor group B construction");
        let sensor_group_b = sensor_group_builder_b.with_i2c(i2c_devices_b).await;
        info!("BOOT 18: sensor group B construction complete");

        info!("BOOT 19: before sensor group C construction");
        let sensor_group_c = sensor_group_builder_c.with_i2c(i2c_devices_c).await;
        info!("BOOT 20: sensor group C construction complete");

        let sensor_groups = [sensor_group_a, sensor_group_b, sensor_group_c];
        let sensor_groups = sensor_groups.map(Mutex::new);
        
        SENSOR_GROUPS.init(sensor_groups)
    };
    info!("BOOT 21: sensor groups stored");

    
    
    let impls = STORAGE.init(uart_rx, uart_tx);
    match impls {
        Some(_) => {},
        None => {
            debug!("Storage Failed");
            return
        }
    }
    info!("BOOT 22: RPC storage initialized");
    let (rx_impl, tx_impl) = impls.unwrap();

    static PACKET_RX_BUF: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0u8; 256]);
    
    
    let context = Context {
        sensor_groups,
        mlx_sensitivity: data_transfer::rpc::MlxSensitivityConfig {
            gain: 0,
            resolution: 0,
            hall_conf: 0xC,
        },
        board_presence: data_transfer::rpc::BoardPresence::default(),
    };

    let dispatcher = MyApp::new(context, spawner.into());
    let vkk = dispatcher.min_key_len();
    let mut server: AppServer =
        Server::new(tx_impl, rx_impl, PACKET_RX_BUF.take(), dispatcher, vkk);
    info!("BOOT 23: RPC server ready");
    loop {
        let _ = server.run().await;

    }
    
}
