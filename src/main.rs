#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use mpu6050::*;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    i2c::{BlockingI2c, Mode},
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().to_Hz());

    let mut afio = dp.AFIO.constrain();
    let mut gpiob = dp.GPIOB.split();
    afio.mapr.modify_mapr(|_, w| w.i2c1_remap().set_bit());

    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let mut i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Standard {
            frequency: 400.kHz(),
        },
        clocks,
        1000,
        10,
        5000,
        5000,
    );

    let mut mpu = Mpu6050::new(i2c);
    mpu.init(&mut delay).unwrap();

    loop {
        match mpu.get_acc_angles() {
            Ok(acc) => {
                let gyro = mpu.get_gyro().unwrap_or_default();
                let temp = mpu.get_temp().unwrap_or(35.36); // Default temp if read fails

                rprintln!(
                    "Temp: {}°C | Pitch: {:.2}° | Roll: {:.2}° | Gyro X Y Z: {:.2}, {:.2}, {:.2}",
                    temp,
                    acc.x,
                    acc.y,
                    gyro.x * 65.5,
                    gyro.y * 65.5,
                    gyro.z * 65.5
                );
            }
            Err(_) => {
                rprintln!("MPU6050 ERROR! Reinitializing...");
                delay.delay_ms(100);
                mpu.init(&mut delay).unwrap(); // Reset sensor
            }
        }

        delay.delay_ms(500);
    }
}
