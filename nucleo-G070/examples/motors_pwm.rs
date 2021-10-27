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

use rtic::app;

#[app(device = st_hal::stm32, peripherals = true)]
mod app {

    extern crate rtic;
    extern crate stm32g0xx_hal as st_hal;
    use core::fmt::{Debug, Display, Write};

    //use st_hal::exti::Event;
    extern crate panic_semihosting;
    use nb::block;
    use st_hal::gpio::gpioa::PA5;
    use st_hal::gpio::{Output, PushPull};
    use st_hal::prelude::*;
    use st_hal::rcc;
    use st_hal::serial::{FifoThreshold, FullConfig, Rx, Serial, Tx};
    use st_hal::stm32::USART2;
    use st_hal::stm32::{TIM1, TIM3, TIM17};
    use st_hal::timer::pins::TimerPin;
    use st_hal::timer::pwm::{Pwm, PwmPin};
    use st_hal::timer::{Channel1, Channel2, Channel3, Channel4, Timer};
    use stm32g0xx_hal::cortex_m::asm;

    #[derive(Clone, Debug, Copy)]
    pub enum Direction {
        Forward,
        Backward,
    }
    #[derive(Clone, Debug, Copy)]
    pub enum Command {
        Init,
        SetMotor,
        SetSpeed
    }
    #[derive(Clone, Debug, Copy)]
    pub enum Motor {
        Motor1,
        Motor2,
        Motor3,
        Motor4,
    }

    #[shared]
    #[derive(Debug, Display)]
    struct Shared {
        //exti: stm32::EXTI,
        blink: PA5<Output<PushPull>>,
        counter: usize,
    }

    #[local]
    struct Local {
        idx: u32,
        dir: Direction,
        command: u32,
        txd: Tx<USART2, FullConfig>,
        rxd: Rx<USART2, FullConfig>,
        m1a: PwmPin<TIM1, Channel1>,
        m1b: PwmPin<TIM1, Channel2>,
        m2a: PwmPin<TIM1, Channel3>,
        m2b: PwmPin<TIM1, Channel4>,
        m3a: PwmPin<TIM3, Channel1>,
        m3b: PwmPin<TIM3, Channel2>,
        m4a: PwmPin<TIM3, Channel3>,
        m4b: PwmPin<TIM3, Channel4>,
        tim17: Timer<TIM17>,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // HSI prescaler: 8, sys_clk: 2MHz

        //let cfg = rcc::Config::hsi(rcc::Prescaler::Div8);
        let cfg = rcc::Config::default();
        //let cfg = rcc::Config::pll();
        let mut rcc = ctx.device.RCC.freeze(cfg);
        //rcc.enable_low_power_mode();

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let gpioc = ctx.device.GPIOC.split(&mut rcc);

        //let mut serial = ctx.device.USART2
        //    .usart(gpioa.pa2, gpioa.pa3,
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

        writeln!(txd, "pwm example abcd ABCD\r").unwrap();

        let mut blink = gpioa.pa5.into_push_pull_output();

        let mut tim17 = ctx.device.TIM17.timer(&mut rcc);
        tim17.start(500.ms());
        tim17.listen();

        let mut counter: usize = 1;

        let pwm1 = ctx.device.TIM1.pwm(100.hz(), &mut rcc);
        let pwm3 = ctx.device.TIM3.pwm(100.hz(), &mut rcc);

        let mut m1a = pwm1.bind_pin(gpioa.pa8);
        let mut m1b = pwm1.bind_pin(gpioa.pa9);

        let mut m2a = pwm1.bind_pin(gpioa.pa10);
        let mut m2b = pwm1.bind_pin(gpioa.pa11);

        let mut m3a = pwm3.bind_pin(gpioa.pa6);
        let mut m3b = pwm3.bind_pin(gpioa.pa7);

        let mut m4a = pwm3.bind_pin(gpiob.pb0);
        let mut m4b = pwm3.bind_pin(gpiob.pb1);

        m1a.enable();
        m1b.enable();
        m2a.enable();
        m2b.enable();
        m3a.enable();
        m3b.enable();
        m4a.enable();
        m4b.enable();

        m1a.set_duty(10000);
        m1b.set_duty(10000);
        m2a.set_duty(10000);
        m2b.set_duty(10000);
        m3a.set_duty(10000);
        m3b.set_duty(10000);
        m4a.set_duty(10000);
        m4b.set_duty(10000);

        (
            Shared {
                blink,
                counter,
            },
            Local {
                idx: 0,
                dir: Direction::Forward,
                command: 0,
                txd,
                rxd,
                m1a,
                m1b,
                m2a,
                m2b,
                m3a,
                m3b,
                m4a,
                m4b,
                tim17: tim17
            },
            init::Monotonics(),
        )
    }

    #[idle(local = [rxd, txd, m1a, m1b, m2a, m2b, m3a,m3b,m4a,m4b ],  shared = [])]
    fn idle(ctx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        //        let idx: &'static mut u32 = ctx.local.idx;
        let tx = ctx.local.txd;
        let mut direction: Direction = Direction::Forward;
        let mut command: Command = Command::Init;
        let mut motor:Motor = Motor::Motor1;
        let mut pwm:u8 = 0;
        let mut m1a = ctx.local.m1a;
        let mut m1b = ctx.local.m1b;
        let mut m2a = ctx.local.m2a;
        let mut m2b = ctx.local.m2b;
        let mut m3a = ctx.local.m3a;
        let mut m3b = ctx.local.m3b;
        let mut m4a = ctx.local.m4a;
        let mut m4b = ctx.local.m4b;

        loop {
            let byte = block!(ctx.local.rxd.read()).unwrap();
            command = match byte as char  {
                // skip spaces
                ' '       => continue,
                'm' | 'M' => Command::SetMotor,
                'h' | 'H' => { giveHelp(tx); continue },
                'f' | 'F' => { 
                    direction = Direction::Forward; 
                    Command::SetSpeed 
                },
                'b' | 'B' => { 
                    direction = Direction::Backward; 
                    Command::SetSpeed 
                },
                's' | 'S' => Command::SetSpeed,
                _ => {
                    writeln!(tx, "Unknown command\r").unwrap();
                    giveHelp(tx);
                    continue
                }
            };

            let byte = block!(ctx.local.rxd.read()).unwrap();

            // skip spaces
            if byte == 32 {continue}

            writeln!(tx,"Command ...  {:?}  \r", command ).unwrap();

            match command {
                Command::SetMotor => {
                motor = match byte as char {
                        '1'=> Motor::Motor1, 
                        '2'=> Motor::Motor2, 
                        '3'=> Motor::Motor3, 
                        '4'=> Motor::Motor4,
                        _  => {
                            writeln!(tx, "Allowed motor 1..4 \r").unwrap();
                            continue
                        } 
                    }
                },
                Command::SetSpeed => {
                    pwm = match byte as char {
                       '0' | '1' |'2' |'3' |'4' |'5' |'6' |'7' |'8' |'9' => byte - 0x30,
                        _  => {
                            writeln!(tx, "Allowed speed 0..9\r").unwrap();
                            continue;
                        }   
                    }
                }, 
                _ => ()
            }
            
            writeln!(tx,
                "Executing ... motor {:?}  pwm :{} direction:{:?}  \r", motor, pwm, direction
            ).unwrap();

            let max1: u32 = m1a.get_max_duty() as u32;

            let mut mpos = 0;
            let mut mneg = 0;
            let duty = (max1 * pwm as u32) / 10;
            match direction {
                Forward => mpos = duty,
                Backward => mneg = duty,
            }
            match motor {
                Motor1 => {
                    m1a.set_duty(mpos as u16);
                    m1b.set_duty(mneg as u16);
                }
                Motor2 => {
                    m2a.set_duty(mpos as u16);
                    m2b.set_duty(mneg as u16);
                }
                Motor3 => {
                    m3a.set_duty(mpos as u32);
                    m3b.set_duty(mneg as u32);
                }
                Motor4 => {
                    m4a.set_duty(mpos as u32);
                    m4b.set_duty(mneg as u32);
                }
                _ => (),
            }
        }
    }

    fn giveHelp(tx: &mut Tx<USART2, FullConfig>) {
        writeln!(tx, "\rHow to use  ... \r").unwrap();
        writeln!(tx, "  h  -- help this message\r").unwrap();
        writeln!(tx, "  f<0..9> -- direction positive\r").unwrap();
        writeln!(tx, "  b<0..9> -- direction negative\r").unwrap();
        writeln!(tx, "  m<1..4>- -- select motor\r").unwrap();
        writeln!(tx, "  n        -- full stop all\r").unwrap();
        writeln!(tx, "  full command example: m3 f 3\r").unwrap();

    }

    
    #[task(binds = TIM17,  priority = 1, local = [tim17], shared = [  blink])]
    fn timer17(ctx: timer17::Context) {
        let mut blink = ctx.shared.blink;
        blink.lock( |b| b.toggle().unwrap());
        ctx.local.tim17.clear_irq();
    } // tim 17
    

    //extern "C" {
    //    fn I2C1();
    //}
}
