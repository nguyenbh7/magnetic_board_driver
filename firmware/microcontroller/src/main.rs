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
    info!("Hello World!");
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
    let p = embassy_stm32::init(config);

    info!("Hello World!");
    let UART_RX = p.PA8;
    let UART_TX = p.PB12;

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
    let (uart_tx, uart_rx) = uart_interface.split();
    let rx_buffer = [0u8; 256];
    let rx_buffer = UART_RX_BUFFER.init(Mutex::new(rx_buffer));
    let uart_rx = uart_rx.into_ring_buffered(rx_buffer.get_mut());
    
    //let uart_rx_bus_mutex = Mutex::new(uart_rx);
    //let uart_rx_bus = UART_RX_BUS.init(uart_rx_bus_mutex);
    let mut i2c_config = i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(500);
    i2c_config.frequency = khz(100);
    i2c_config.sda_pullup = true;
    i2c_config.scl_pullup = true;


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
    
    
    let i2c_devices_a = {
        core::array::from_fn(|_| I2cDevice::new(i2c_bus1))
    };

    let i2c_devices_b = {
        core::array::from_fn(|_| I2cDevice::new(i2c_bus3))
    };

    let i2c_devices_c = {
        core::array::from_fn(|_| I2cDevice::new(i2c_bus3))
    };

    let sensor_grid_side_length = 13.5;
    let positions: [_; 16]  = core::array::from_fn(
        |i|{
            let step = sensor_grid_side_length/4.0;
            (-sensor_grid_side_length/2.0+step*((i / 4) as f32),-sensor_grid_side_length/2.0+step*((i % 4) as f32), 0.0)
        });
    
    let sensor_groups = {
        let sensor_builders_a: [_; 16] = core::array::from_fn(|i| SensorBuilder::new_stm(0x0C+(i as u8), positions[i]));
        let sensor_builders_b: [_; 16] = core::array::from_fn(|i| SensorBuilder::new_stm(0x0C+(i as u8), positions[i]));
        let sensor_builders_c: [_; 16] = core::array::from_fn(|i| SensorBuilder::new_stm((0x0C+(i as u8)) ^ 0b01000000, positions[i]));
        let mut sensor_group_builder_a = SensorGroupBuilder::new_stm(0, sensor_builders_a);
        let mut sensor_group_builder_b = SensorGroupBuilder::new_stm(1, sensor_builders_b);
        let mut sensor_group_builder_c = SensorGroupBuilder::new_stm(2, sensor_builders_c);
        let sensor_groups = [sensor_group_builder_a.with_i2c(i2c_devices_a).await, sensor_group_builder_b.with_i2c(i2c_devices_b).await, sensor_group_builder_c.with_i2c(i2c_devices_c).await];
        let sensor_groups = sensor_groups.map(Mutex::new);
        
        SENSOR_GROUPS.init(sensor_groups)
    };

    
    
    let impls = STORAGE.init(uart_rx, uart_tx);
    match impls {
        Some(_) => {},
        None => {
            debug!("Storage Failed");
            return
        }
    }
    let (rx_impl, tx_impl) = impls.unwrap();

    static PACKET_RX_BUF: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0u8; 256]);
    
    
    let context = Context {
        sensor_groups,
        mlx_sensitivity: data_transfer::rpc::MlxSensitivityConfig {
            gain: 0,
            resolution: 0,
            hall_conf: 0xC,
},
    };

    let dispatcher = MyApp::new(context, spawner.into());
    let vkk = dispatcher.min_key_len();
    let mut server: AppServer =
        Server::new(tx_impl, rx_impl, PACKET_RX_BUF.take(), dispatcher, vkk);
    loop {
        let _ = server.run().await;

    }
    
}
