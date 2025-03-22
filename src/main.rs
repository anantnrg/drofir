#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use libm::{atan2f, sqrtf};
use mpu6050::*;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    adc::Adc,
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
    let mut adc = Adc::adc1(dp.ADC1, clocks);
    let mut pot_pin = gpioa.pa0.into_analog(&mut gpioa.crl);
    let mut mpu = Mpu6050::new_with_sens(i2c, device::AccelRange::G4, device::GyroRange::D2000);

    mpu.init(&mut delay).unwrap();

    // let max_duty = channel.get_max_duty();
    // let min_pulse = max_duty / 40;
    // let max_pulse = max_duty / 8;

    let mut roll = 0.0;
    let mut pitch = 0.0;

    loop {
        let raw_adc: u16 = adc.read(&mut pot_pin).unwrap();
        let throttle = (raw_adc as f32 / 4095.0) * 100.0;
        let gyro = mpu.get_gyro().unwrap();
        let accel = mpu.get_acc().unwrap();

        let x_raw = gyro.x * RAD_TO_DEG * SENS_DPS;
        let y_raw = gyro.y * RAD_TO_DEG * SENS_DPS;

        let acc_roll = atan2f(accel.y, accel.z) * RAD_TO_DEG;
        let acc_pitch = -atan2f(accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)) * RAD_TO_DEG;
        roll = ALPHA * (roll + x_raw * DT) + (1.0 - ALPHA) * acc_roll;
        pitch = ALPHA * (pitch + y_raw * DT) + (1.0 - ALPHA) * acc_pitch;
        rprintln!(
            "Throttle: {:.2} Roll: {:.2} Pitch: {:.2}",
            throttle,
            roll,
            pitch
        );

        delay.delay_ms((DT * 1000.0) as u32);
        // rprintln!("Moving to 0 degrees with pulse: {}", min_pulse);
        // channel.set_duty(min_pulse);
        // delay.delay_ms(2000);

        // rprintln!("Moving to 180 degrees with pulse: {}", max_pulse);
        // channel.set_duty(max_pulse);
        // delay.delay_ms(2000);
    }
}
