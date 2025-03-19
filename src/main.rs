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
        .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz(), &clocks);
    pwm.enable(Channel::C1);
    let max_duty = pwm.get_max_duty();

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
        let accel = mpu.get_acc().unwrap();
        let gyro = mpu.get_gyro().unwrap();

        let ax = accel.x;
        let ay = accel.y;
        let az = accel.z;
        let gx = gyro.x;
        let gy = gyro.y;
        let gz = gyro.z;

        rprintln!("AX: {}, AY: {}, AZ: {}", ax, ay, az);
        rprintln!("GX: {}, GY: {}, GZ: {}", gx, gy, gz);

        delay.delay_ms(100u32);

        for angle in 0..=180 {
            let pulse = max_duty / 20 + (angle * max_duty / 180) / 20;
            pwm.set_duty(Channel::C1, pulse);
            cortex_m::asm::delay(8_000_000);
        }

        for angle in (0..=180).rev() {
            let pulse = max_duty / 20 + (angle * max_duty / 180) / 20;
            pwm.set_duty(Channel::C1, pulse);
            cortex_m::asm::delay(8_000_000);
        }
    }
}
