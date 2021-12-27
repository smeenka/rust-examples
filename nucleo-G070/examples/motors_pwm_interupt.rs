#![no_std]
#![no_main]
#![allow(warnings)]

/*
    T1   channel 1:  pin pa6 : m1a
    T1   channel 2:  pin pa7 : m1b
    T1   channel 3:  pin pb0 : m2a
    T1   channel 4:  pin pb1 : m2b

    T3   channel 1:  pin pa6 : m3a
    T3   channel 2:  pin pa7 : m3b
    T3   channel 3:  pin pb0 : m4a
    T3   channel 4:  pin pb1 : m4b
*/

/*
This file test the PMW timer with interrupts enabled.
The interrupt to let the status let blink is from timer 3, also used as a pwm timer

Until merged with master, it can only work with next settings in Cargo.toml :

[dependencies.stm32g0xx-hal]
default-features = false
features = ["rt", "stm32g070"]
version = "0.1.2"
git = "https://github.com/smeenka/stm32g0xx-hal"
branch = "feature/pwm_with_interrupts"
*/

use rtic::app;

#[app(device = st_hal::stm32, peripherals = true)]
mod app {

    extern crate rtic;
    extern crate stm32g0xx_hal as st_hal;
    use core::fmt::{Debug, Display, Write};
    use core::marker::PhantomData;
    use nb;
    //use st_hal::exti::Event;
    extern crate panic_semihosting;
    use nb::block;
    use st_hal::gpio::gpioa::PA5;
    use st_hal::gpio::{Output, PushPull};
    use st_hal::prelude::*;
    use st_hal::rcc;
    use st_hal::serial::{FifoThreshold, FullConfig, Rx, Serial, Tx};
    use st_hal::stm32::USART2;
    use st_hal::stm32::{TIM1, TIM3};
    use st_hal::timer::pins::TimerPin;
    use st_hal::timer::pwm::{Pwm, PwmPin};
    use st_hal::timer::{Channel1, Channel2, Channel3, Channel4};
    use stm32g0xx_hal::cortex_m::asm;

    #[derive(Clone, Debug, Copy)]
    pub enum Direction {
        Forward,
        Backward,
    }
    #[derive(Clone, Debug, Copy)]
    pub enum Command {
        Parse,
        SetMotor,
        SetSpeed,
        StopAll,
    }
    #[derive(Clone, Debug, Copy)]
    pub enum Motor {
        MotorAll,
        Motor1,
        Motor2,
        Motor3,
        Motor4,
    }
    #[derive()]
    pub struct AppState {
        motor: Motor,
        command: Command,
        pwm: usize,
        dir: Direction,
        m1a: PwmPin<TIM1, Channel1>,
        m1b: PwmPin<TIM1, Channel2>,
        m2a: PwmPin<TIM1, Channel3>,
        m2b: PwmPin<TIM1, Channel4>,
        m3a: PwmPin<TIM3, Channel1>,
        m3b: PwmPin<TIM3, Channel2>,
        m4a: PwmPin<TIM3, Channel3>,
        m4b: PwmPin<TIM3, Channel4>,
        txd: Tx<USART2, FullConfig>,
        rxd: Rx<USART2, FullConfig>,
    }
    #[shared]
    #[derive(Debug, Display)]
    struct Shared {
    }
    #[local]
    struct Local {
        state: AppState,
        pwm3: Pwm<TIM3>,
        counter: usize,
        blink: PA5<Output<PushPull>>,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // HSI prescaler: 8, sys_clk: 2MHz

        //let cfg = rcc::Config::hsi(rcc::Prescaler::Div8);
        //let cfg = rcc::Config::default();
        //let cfg = rcc::Config::pll();
        let cfg = rcc::Config::hsi(rcc::Prescaler::NotDivided);

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
                    .rx_fifo_threshold(FifoThreshold::FIFO_4_BYTES),
                &mut rcc,
            )
            .unwrap();

        let (mut txd, mut rxd) = serial.split();

        writeln!(txd, "Test 4 motors interactive over Serial, with timer 3 also generating interrupts.\r").unwrap();


        let mut counter: usize = 1;

        let pwm1 = ctx.device.TIM1.pwm(100.hz(), &mut rcc);
        let mut pwm3 = ctx.device.TIM3.pwm(100.hz(), &mut rcc);
        pwm3.listen();

        let mut blink = gpioa.pa5.into_push_pull_output();

        let mut state = AppState {
            pwm: 0,
            motor: Motor::Motor1,
            command: Command::Parse,
            dir: Direction::Forward,
            m1a: pwm1.bind_pin(gpioa.pa8),
            m1b: pwm1.bind_pin(gpioa.pa9),
            m2a: pwm1.bind_pin(gpioa.pa10),
            m2b: pwm1.bind_pin(gpioa.pa11),
            m3a: pwm3.bind_pin(gpioa.pa6),
            m3b: pwm3.bind_pin(gpioa.pa7),
            m4a: pwm3.bind_pin(gpiob.pb0),
            m4b: pwm3.bind_pin(gpiob.pb1),
            txd,
            rxd,
        };
        state.m1a.enable();
        state.m1b.enable();
        state.m2a.enable();
        state.m2b.enable();
        state.m3a.enable();
        state.m3b.enable();
        state.m4a.enable();
        state.m4b.enable();

        state.m1a.set_duty(0);
        state.m1b.set_duty(0);
        state.m2a.set_duty(0);
        state.m2b.set_duty(0);
        state.m3a.set_duty(0);
        state.m3b.set_duty(0);
        state.m4a.set_duty(0);
        state.m4b.set_duty(0);

        // return tuple
        (
            Shared {},
            Local {
                counter, blink,  state, pwm3,
            },
            init::Monotonics(),
        )
    }

    #[idle(local = [state],  shared = [])]
    fn idle(ctx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        //        let idx: &'static mut u32 = ctx.local.idx;
        let mut state = ctx.local.state;

        loop {
            if parseFirstLetter(&mut state) {
                if parseSecondLetter(&mut state) {
                    execute(&mut state);
                };
            };
        }
    }

    // Parse the first letter of the command return true if succesfull
    fn parseFirstLetter(state: &mut AppState) -> bool {
        // flush rx
        loop {
            match state.rxd.read() {
                Err(nb::Error::WouldBlock) => break,
                _ => (),
            }
        }
        let mut result = true;
        let c = block!(state.rxd.read()).unwrap();

        state.command = match c as char {
            // skip spaces
            'm' | 'M' => Command::SetMotor,
            'h' | 'H' => {
                giveHelp(&mut state.txd);
                result = false;
                Command::Parse
            }
            'f' | 'F' => {
                state.dir = Direction::Forward;
                Command::SetSpeed
            }
            'b' | 'B' => {
                state.dir = Direction::Backward;
                Command::SetSpeed
            }
            's' | 'S' => Command::SetSpeed,
            _ => {
                writeln!(state.txd, "Unknown command\r").unwrap();
                giveHelp(&mut state.txd);
                result = false;
                Command::Parse
            }
        };
        result
    }

    // Parse the second letter of the command return true if succesfull
    fn parseSecondLetter(state: &mut AppState) -> bool {
        // read character until not a space
        let mut c = 32;
        while c == 32 {
            c = block!(state.rxd.read()).unwrap();
        }
        //writeln!(state.txd, "Reached second state: Second state command {:?} \r", state.command);
        match state.command {
            Command::SetMotor => {
                state.motor = match c as char {
                    '0' => Motor::MotorAll,
                    '1' => Motor::Motor1,
                    '2' => Motor::Motor2,
                    '3' => Motor::Motor3,
                    '4' => Motor::Motor4,
                    _ => {
                        writeln!(state.txd, "Allowed motor 0..4 \r").unwrap();
                        return false;
                    }
                };
            }
            Command::SetSpeed => {
                let inp = match c as char {
                    '0' | '1' | '2' | '3' | '4' | '5' | '6' | '7' | '8' | '9' => c - 0x30,
                    _ => {
                        writeln!(state.txd, "Allowed speed 0..9\r").unwrap();
                        return false;
                    }
                };
                let max1 = state.m1a.get_max_duty() as usize;
                state.pwm = (max1 * inp as usize) / 10;
            }
            _ => (),
        };
        true
    }

    fn execute(state: &mut AppState) {
        writeln!(
            state.txd,
            "Execute command {:?} dir {:?} pwm {} \r",
            state.command, state.dir, state.pwm
        )
        .unwrap();

        let mut mpos = 0;
        let mut mneg = 0;
        match state.dir {
            Direction::Forward => mpos = state.pwm,
            Direction::Backward => mneg = state.pwm,
        }
        match state.motor {
            Motor::Motor1 => {
                state.m1a.set_duty(mpos as u16);
                state.m1b.set_duty(mneg as u16);
            }
            Motor::Motor2 => {
                state.m2a.set_duty(mpos as u16);
                state.m2b.set_duty(mneg as u16);
            }
            Motor::Motor3 => {
                state.m3a.set_duty(mpos as u32);
                state.m3b.set_duty(mneg as u32);
            }
            Motor::Motor4 => {
                state.m4a.set_duty(mpos as u32);
                state.m4b.set_duty(mneg as u32);
            }
            Motor::MotorAll => {
                state.m1a.set_duty(mpos as u16);
                state.m1b.set_duty(mneg as u16);
                state.m2a.set_duty(mpos as u16);
                state.m2b.set_duty(mneg as u16);
                state.m3a.set_duty(mpos as u32);
                state.m3b.set_duty(mneg as u32);
                state.m4a.set_duty(mpos as u32);
                state.m4b.set_duty(mneg as u32);
            }
        };
    }

    fn giveHelp(tx: &mut Tx<USART2, FullConfig>) {
        writeln!(tx, "\rHow to use  ... \r").unwrap();
        writeln!(tx, "  h  -- help this message\r").unwrap();
        writeln!(tx, "  f<0..9> -- direction positive\r").unwrap();
        writeln!(tx, "  b<0..9> -- direction negative\r").unwrap();
        writeln!(tx, "  m<0..4>- -- select motor. 0 is all motors\r").unwrap();
        writeln!(tx, "  n        -- full stop all\r").unwrap();
        writeln!(tx, "  full command example: m3 f 3\r").unwrap();
    }

    #[task(binds = TIM3,  priority = 1, local = [blink, counter, pwm3], shared = [] )]
    fn pwm3(ctx: pwm3::Context) {
        let mut blink = ctx.local.blink;
        let mut counter = ctx.local.counter;

        *counter += 1;
        if *counter % 80 == 0 {
            blink.toggle().unwrap();
        }
        ctx.local.pwm3.clear_irq();
    } // tim 17
}
