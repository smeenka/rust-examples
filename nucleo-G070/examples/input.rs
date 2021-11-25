#![no_std]
#![no_main]
#![allow(warnings)]

extern crate rtic;
use rtic::app;
#[app(device = hal::stm32, peripherals = true)]
mod app {

    extern crate panic_semihosting;
    extern crate stm32g0xx_hal as hal;
    use core::fmt::Write;

    //use hal::exti::Event;
    use hal::gpio::gpioa::PA5;
    use hal::gpio::{Output, PushPull};
    use hal::prelude::*;
    use hal::rcc;
    use hal::serial::{FifoThreshold, FullConfig, Rx, Serial, Tx};
    use hal::stm32::USART2;
    use hal::stm32::{TIM14, TIM16, TIM17};
    use hal::timer::pins::TimerPin;
    use hal::timer::Timer;
    use stm32g0xx_hal::cortex_m::asm;

    use nb;

    #[shared]
    struct Shared {
        //exti: stm32::EXTI,
        txd: Tx<USART2, FullConfig>,
    }
    #[local]
    struct Local {
        counter: usize,
        blink: PA5<Output<PushPull>>,
        tim17: Timer<TIM17>,
        rxd: Rx<USART2, FullConfig>,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // HSI prescaler: 8, sys_clk: 2MHz

        let cfg = rcc::Config::hsi(rcc::Prescaler::Div8);
        let mut rcc = ctx.device.RCC.freeze(cfg);
        //rcc.enable_low_power_mode();

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);

        let mut serial = ctx
            .device
            .USART2
            .usart(
                gpioa.pa2,
                gpioa.pa3,
                FullConfig::default()
                    .baudrate(115200.bps())
                    .fifo_enable()
                    .rx_fifo_enable_interrupt()
                    .rx_fifo_threshold(FifoThreshold::FIFO_4_BYTES),
                &mut rcc,
            )
            .unwrap();

        let (mut txd, mut rxd) = serial.split();
        rxd.listen();

        writeln!(txd, "Input example\r").unwrap();

        let mut blink = gpioa.pa5.into_push_pull_output();

        let mut tim17 = ctx.device.TIM17.timer(&mut rcc);
        tim17.start(5000.ms());
        tim17.listen();

        let counter: usize = 1;

        (
            Shared {
                //exti: ctx.device.EXTI,
                txd,
            },
            Local {
                counter,
                blink,
                tim17,
                rxd,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM17,  priority = 1, local = [tim17,  blink,counter], shared = [txd ])]
    fn timer17(ctx: timer17::Context) {
        ctx.local.blink.toggle().unwrap();
        let mut txd = ctx.shared.txd;
        let counter = ctx.local.counter;
        *counter = *counter + 1;
        txd.lock(|txd| writeln!(txd, "Counter:{}\r", counter).unwrap());
        ctx.local.tim17.clear_irq();
    } // tim 17

    #[task(binds = USART2,  priority = 1, local = [rxd], shared = [ txd])]
    fn serial_read(ctx: serial_read::Context) {
        let mut txd = ctx.shared.txd;
        loop {
            match ctx.local.rxd.read() {
                Err(nb::Error::WouldBlock) => {
                    // no more data available in fifo
                    break;
                }
                Err(nb::Error::Other(_err)) => {
                    // Handle other error Overrun, Framing, Noise or Parity
                    break;
                }
                Ok(byte) => {
                    txd.lock(|txd| write!(txd, "{}", byte as char).unwrap());
                }
            }
        }
    } // serial 2 RX

}
