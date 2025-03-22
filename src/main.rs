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
    timer::Tim3NoRemap,
};

const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;
const SENS_DPS: f32 = 16.4;
const ALPHA: f32 = 0.98;
const DT: f32 = 0.01;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().to_Hz());

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    afio.mapr.modify_mapr(|_, w| w.i2c1_remap().set_bit());

    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    // let servo_pin = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);

    // let pwm = dp
    //     .TIM3
    //     .pwm_hz::<Tim3NoRemap, _, _>(servo_pin, &mut afio.mapr, 50.Hz(), &clocks);

    // let mut channel = pwm.split();
    // channel.enable();

    let mut i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Standard {
            frequency: 100.kHz(),
        },
        clocks,
        1000,
        10,
        5000,
        5000,
    );
    let mut mpu = Mpu6050::new_with_sens(i2c, device::AccelRange::G4, device::GyroRange::D2000);

    mpu.init(&mut delay).unwrap();

    // let max_duty = channel.get_max_duty();
    // let min_pulse = max_duty / 40;
    // let max_pulse = max_duty / 8;

    let mut roll = 0.0;
    let mut pitch = 0.0;

    loop {
        let gyro = mpu.get_gyro().unwrap();
        let accel = mpu.get_acc().unwrap();
        let temp = mpu.get_temp().unwrap();

        let x_raw = (gyro.x * RAD_TO_DEG * SENS_DPS) as i16;
        let y_raw = (gyro.y * RAD_TO_DEG * SENS_DPS) as i16;
        let z_raw = (gyro.z * RAD_TO_DEG * SENS_DPS) as i16;

        let acc_roll = accel.y.atan2(accel.z) * RAD_TO_DEG;
        let acc_pitch = -accel
            .x
            .atan2((accel.y * accel.y + accel.z * accel.z).sqrt())
            * RAD_TO_DEG;

        rprintln!(
            "Xraw = {} Yraw = {} Zraw = {} Temperature = {:.2}",
            x_raw,
            y_raw,
            z_raw,
            temp
        );
        // rprintln!("Moving to 0 degrees with pulse: {}", min_pulse);
        // channel.set_duty(min_pulse);
        // delay.delay_ms(2000);

        // rprintln!("Moving to 180 degrees with pulse: {}", max_pulse);
        // channel.set_duty(max_pulse);
        // delay.delay_ms(2000);
    }
}
