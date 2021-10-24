#![allow(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use stm32g0xx_hal::cortex_m;
extern crate cortex_m_rt as rt;
extern crate cortex_m_semihosting as sh;
extern crate nb;
extern crate panic_halt;
extern crate stm32g0xx_hal as hal;

use hal::prelude::*;
use hal::rcc::{self, PllConfig};
use hal::spi::{NoMiso,NoSck};
use hal::stm32;
use rt::entry;
use smart_leds::{SmartLedsWrite, RGB};
use ws2812_spi as ws2812;
use hal::gpio::gpioa::{PA5};
use ws2812::halfduplex::Ws2812;


#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    // Configure APB bus clock to 16MHz, cause ws2812 requires 4Mbps SPI
    //let pll_cfg = PllConfig::with_hsi(4, 24, 2);
    //let rcc_cfg = rcc::Config::pll().pll_cfg(pll_cfg);    
    let rcc_cfg = rcc::Config::hsi(rcc::Prescaler::NotDivided);
    let mut rcc = dp.RCC.freeze(rcc_cfg);


    let mut delay = cp.SYST.delay(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut spi = dp.SPI1.spi(
        (NoSck, NoMiso, gpiob.pb5),
        ws2812::MODE,
        4.mhz(),
        &mut rcc,
    );
    spi.half_duplex_enable(true);
    spi.half_duplex_oe(true);
    spi.data_size(5);

    let mut ws = Ws2812::new(spi);

    let mut blink = gpioa.pa5.into_push_pull_output();

    let mut cnt: usize = 0;
    let mut data: [RGB<u8>; 15] = [RGB::default(); 15];
    loop {
        for (idx, color) in data.iter_mut().enumerate() {
            *color = match (cnt + idx) % 3 {
                0 => RGB { r: 0x30, g: 0, b: 0 },
                1 => RGB { r: 0, g: 0x40, b: 0 },
                _ => RGB { r: 0, g: 0, b: 0x60 },
            };
        }
        ws.write(data.iter().cloned()).unwrap();
        cnt += 1;
        delay.delay(500.ms());
        blink.toggle().unwrap();
    }
}
