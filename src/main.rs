#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use embedded_hal::i2c::I2c;
use mpu6050::Mpu6050;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    gpio::{
        gpiob::{PB6, PB7},
        Alternate, OpenDrain,
    },
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
    rcc::Clocks,
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("RTT initialized!");
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    // dp.I2C1.cr1.modify(|_, w| w.pe().clear_bit()); // Disable I2C
    // dp.I2C1.cr1.modify(|_, w| w.pe().set_bit()); // Re-enable I2C
    // rprintln!("I2C reset!");

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks: Clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().to_Hz());

    let mut afio = dp.AFIO.constrain();
    let mut gpiob = dp.GPIOB.split();
    afio.mapr.modify_mapr(|_, w| w.i2c1_remap().set_bit());

    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let mut i2c = BlockingI2c::i2c1(
        dp.I2C1,        // I2C1 peripheral
        (scl, sda),     // SDA & SCL pins
        &mut afio.mapr, // Alternate function remap
        Mode::Standard {
            frequency: 100.kHz(),
        },
        clocks, // Clocks
        1000,   // Start timeout (us)
        10,     // Start retries
        5000,   // Address timeout (us)
        5000,   // Data timeout (us)
    );
    // rprintln!("Scanning I2C bus...");
    // for addr in 0x08..=0x77 {
    //     if i2c.write(addr, &[0]).is_ok() {
    //         rprintln!("Device found at 0x{:X}", addr);
    //     }
    // }
    rprintln!("Scan complete.");

    let mut mpu = Mpu6050::new(i2c);
    mpu.init(&mut delay).unwrap(); // Pass delay reference

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
    }
}
