#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use embedded_hal::i2c::I2c;
use mpu6050::Mpu6050;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    i2c::{BlockingI2c, Mode},
    pac,
    prelude::*,
    rcc::Clocks,
    timer::{Channel, Tim2NoRemap},
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("RTT initialized!");

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().to_Hz());

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    afio.mapr.modify_mapr(|_, w| w.i2c1_remap().set_bit());

    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let servo_1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
    let pins = (servo_1);

    let mut pwm = dp
        .TIM2
        .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 50.Hz(), &clocks);

    let max_duty = pwm.get_max_duty(); // Timer resolution
    let min_pulse = max_duty / 30; // 500us
    let max_pulse = max_duty / 7; // 2500us
    pwm.set_duty(Channel::C1, max_pulse);
    pwm.enable(Channel::C1);

    let min_duty = pulse_width_to_ticks(min_pulse, max_duty);
    let max_duty = pulse_width_to_ticks(max_pulse, max_duty);

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

    let mut mpu = Mpu6050::new(i2c);
    mpu.init(&mut delay).unwrap();
    loop {
        pwm.set_duty(Channel::C1, max_pulse);
        delay.delay_ms(1000);

        pwm.set_duty(Channel::C1, min_pulse);
        delay.delay_ms(1000);
    }
}

fn angle_to_duty(angle: u16, min_duty: u16, max_duty: u16) -> u16 {
    min_duty + ((max_duty - min_duty) * angle / 180)
}

fn pulse_width_to_ticks(pulse_width: u16, max_duty: u16) -> u16 {
    (pulse_width as u32 * max_duty as u32 / 20000) as u16
}

// loop {
//     let accel = mpu.get_acc().unwrap();
//     let gyro = mpu.get_gyro().unwrap();

//     rprintln!(
//         "AX: {:.2}, AY: {:.2}, AZ: {:.2} | GX: {:.2}, GY: {:.2}, GZ: {:.2}",
//         accel.x,
//         accel.y,
//         accel.z,
//         gyro.x,
//         gyro.y,
//         gyro.z
//     );
