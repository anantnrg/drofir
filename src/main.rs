#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use mpu6050::Mpu6050;
use panic_rtt_target as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    i2c::{BlockingI2c, Mode},
    pac,
    prelude::*,
};
use systick_monotonic::{fugit::Duration, Systick};

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        mpu: Mpu6050<BlockingI2c<pac::I2C1>>,
    }

    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        let mono = Systick::new(cx.core.SYST, 36_000_000);

        rtt_init_print!();
        rprintln!("Initializing MPU6050...");

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze(&mut flash.acr);

        // Setup I2C (PB6 = SCL, PB7 = SDA)
        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();

        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400.kHz(),
                duty_cycle: stm32f1xx_hal::i2c::DutyCycle::Ratio2to1,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        let mut mpu = Mpu6050::new(i2c);
        mpu.init().unwrap();

        // Schedule the sensor read task
        read_mpu::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        (Shared { mpu }, Local {}, init::Monotonics(mono))
    }

    #[task(shared = [mpu])]
    fn read_mpu(mut cx: read_mpu::Context) {
        let accel = cx.shared.mpu.lock(|mpu| mpu.get_acc_angles().unwrap());
        let gyro = cx.shared.mpu.lock(|mpu| mpu.get_gyro().unwrap());

        rprintln!(
            "Accel: x={} y={} z={}, Gyro: x={} y={} z={}",
            accel.x,
            accel.y,
            accel.z,
            gyro.x,
            gyro.y,
            gyro.z
        );

        read_mpu::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }
}
