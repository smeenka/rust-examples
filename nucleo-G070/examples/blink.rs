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
    use hal::serial::{FifoThreshold, FullConfig, Serial};
    use hal::stm32::USART2;
    use hal::stm32::{TIM14, TIM16, TIM17};
    use hal::timer::pins::TimerPin;
    use hal::timer::Timer;
    use stm32g0xx_hal::cortex_m::asm;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        //exti: stm32::EXTI,
        blink: PA5<Output<PushPull>>,
        tim17: Timer<TIM17>,
        serial: Serial<USART2, FullConfig>,
        counter: usize,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // HSI prescaler: 8, sys_clk: 2MHz

        let cfg = rcc::Config::hsi(rcc::Prescaler::Div8);
        let mut rcc = ctx.device.RCC.freeze(cfg);
        //rcc.enable_low_power_mode();

        let gpioa = ctx.device.GPIOA.split(&mut rcc);

        let mut serial = ctx
            .device
            .USART2
            .usart(gpioa.pa2, gpioa.pa3, FullConfig::default(), &mut rcc)
            .unwrap();

        writeln!(serial, "Blink example\r").unwrap();

        let mut blink = gpioa.pa5.into_push_pull_output();

        let mut tim17 = ctx.device.TIM17.timer(&mut rcc);
        tim17.start(100.ms());
        tim17.listen();

        let counter: usize = 1;

        (
            Shared {
            //exti: ctx.device.EXTI,
        },
            Local {
                blink,
                tim17,
                serial,
                counter,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM17,  priority = 1, local = [tim17,  blink, serial,counter])]
    fn timer17(ctx: timer17::Context) {
        ctx.local.blink.toggle().unwrap();
        let counter = ctx.local.counter;
        *counter = *counter + 1;
        writeln!(ctx.local.serial, "Counter:{}\r", counter).unwrap();
        ctx.local.tim17.clear_irq();
    } // tim 17
}
