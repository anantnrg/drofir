#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;

use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    timer::{pwm, Tim3NoRemap},
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
    let mut gpioa = dp.GPIOA.split();
    let servo_pin = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);

    let pwm = dp
        .TIM3
        .pwm_hz::<Tim3NoRemap, _, _>(servo_pin, &mut afio.mapr, 50.Hz(), &clocks);

    let mut channel = pwm.split();
    channel.enable();

    let max_duty = channel.get_max_duty();
    let min_pulse = max_duty / 40;
    let max_pulse = max_duty / 8;

    loop {
        rprintln!("Moving to 0 degrees with pulse: {}", min_pulse);
        channel.set_duty(min_pulse);
        delay.delay_ms(1000);

        rprintln!("Moving to 180 degrees with pulse: {}", max_pulse);
        channel.set_duty(max_pulse);
        delay.delay_ms(1000);
    }
}
