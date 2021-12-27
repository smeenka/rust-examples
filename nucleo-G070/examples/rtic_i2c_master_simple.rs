#![no_std]
#![no_main]
#![allow(warnings)]

use rtic::app;

#[app(device = hal::stm32, peripherals = true)]
mod app {
    extern crate rtic;
    extern crate stm32g0xx_hal as hal;

    use core::fmt::{Write, Debug, Display};
    
    //use hal::exti::Event;
    extern crate panic_semihosting;
    use stm32g0xx_hal::cortex_m::asm;
    use hal::gpio::{OpenDrain};
    use hal::gpio::gpioa::{PA5};
    use hal::gpio::gpiob::{PB5, PB8, PB9};
    use hal::gpio::{Output, PushPull, Analog};
    use hal::prelude::*;
    use hal::stm32::USART2;
    use hal::stm32::I2C1;
    use hal::rcc::{self, PllConfig};    
    use hal::timer::{Timer,Channel1,Channel2,Channel3,Channel4};
    use hal::timer::pwm::{PwmPin,Pwm };
    use hal::timer::pins::TimerPin;
    use hal::stm32::{TIM17};
    use hal::serial::{Serial, Tx, Rx,  FullConfig, FifoThreshold}; 
    use hal::i2c::{Config, I2c, I2cControl, I2cMaster};

    use nb::block;

    #[shared]
    #[derive(Debug, Display)]
    struct Shared {
        //exti: stm32::EXTI,
        counter:usize,
        txd:   Tx<USART2,FullConfig>,
        i2c:   I2c<I2C1, PB9<Output<OpenDrain>>, PB8<Output<OpenDrain>>>,
    }

    #[local]
    struct Local {
        tim17: Timer<TIM17>,
        blink: PA5<Output<PushPull>>,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // HSI prescaler: 1, sys_clk: 16MHz
        let cfg = rcc::Config::hsi(rcc::Prescaler::NotDivided);          

        let mut rcc = ctx.device.RCC.freeze(cfg);
        //rcc.enable_low_power_mode();
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let gpioc = ctx.device.GPIOC.split(&mut rcc);


        let mut tim17 = ctx.device.TIM17.timer(&mut rcc);
        tim17.start(100.ms() );
        tim17.listen();

        let sda = gpiob.pb9.into_open_drain_output();
        let scl = gpiob.pb8.into_open_drain_output();
        let configi2c = hal::i2c::Config::with_timing(0x2020_151b);
        let mut i2c = ctx.device.I2C1.i2c(sda, scl, configi2c, &mut rcc);
        i2c.listen();
    

        let mut serial = ctx.device.USART2
            .usart(gpioa.pa2, gpioa.pa3,  
        //let mut serial = ctx.device.USART2
        //    .usart(gpiob.pb6, gpiob.pb7,  
            FullConfig::default()
                .baudrate(115200.bps())
                .fifo_enable()
                .rx_fifo_threshold(FifoThreshold::FIFO_4_BYTES),
                &mut rcc)
            .unwrap();

        let (mut txd, mut rxd) = serial.split(); 

        writeln!(txd, "I2c with RTIC simple example.\r").unwrap();        
        writeln!(txd, "Connect a IO expander on address i2c address 0x40 for output.\r").unwrap();        
        writeln!(txd, "Connect a M5 Joystick on address 0x52 (x2) for input.\r").unwrap();        

        let mut counter:usize = 1;
        let mut blink = gpioa.pa5.into_push_pull_output();

        (Shared {
            counter,
            txd,
            i2c,
        }, 
        Local {
            tim17,
            blink
        }, 
        init::Monotonics())
    }

    #[idle(local = [],  shared = [counter])]
    fn idle(ctx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        let counter = ctx.shared.counter;
        loop {

        }
    }


    #[task(binds = TIM17,  priority = 3, local = [tim17, blink], shared = [counter, txd, i2c ])]
    fn timer17(ctx: timer17::Context) {
        let mut counter = ctx.shared.counter;
        let mut txd = ctx.shared.txd;
        let mut i2c = ctx.shared.i2c;

        ctx.local.blink.toggle().unwrap();
        let mut buf_short = [0_u8;1];
        (txd, i2c, counter ).lock(|txd,i2c, counter| {
            *counter += 1;
            writeln!(txd, "Counter:{}\r", counter ).unwrap();        
            
            buf_short[0] = *counter as u8;
            match i2c.master_write(0x20, &buf_short) {
                Ok(_) => (),
                Err(err) => writeln!(txd, "0x20 Write Error:{:?}\r", err).unwrap(),
            };
        });
        ctx.local.tim17.clear_irq();

    } // tim 17

    #[task(binds = I2C1,  priority = 4, local = [], shared = [i2c, txd ])]
    fn i2_irq(ctx: i2_irq::Context) {
        let mut txd = ctx.shared.txd;
        let mut i2c = ctx.shared.i2c;

        (i2c, txd).lock(|i2c, txd| {
            match i2c.check_isr_flags(){
                Ok( b) => writeln!(txd, "Ok {:?}\r", b ).unwrap(),
                Err(nb::Error::WouldBlock) => writeln!(txd, "0x20 irq Expected WouldBlock\r").unwrap(),
                Err(err) => writeln!(txd, "0x20 irq Error: {:?}\r", err).unwrap(),
            }
        });
    } // I2C


    //extern "C" {
    //    fn I2C1();
    //}
}

