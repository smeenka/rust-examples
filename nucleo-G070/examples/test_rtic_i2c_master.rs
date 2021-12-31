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
    use hal::stm32::{USART2,USART1};
    use hal::stm32::I2C1;
    use hal::rcc::{self, PllConfig};    
    use hal::timer::{Timer,Channel1,Channel2,Channel3,Channel4};
    use hal::timer::pwm::{PwmPin,Pwm };
    use hal::timer::delay::{Delay };
    use hal::timer::pins::TimerPin;
    use hal::stm32::{TIM17,TIM15};
    use hal::serial::{Serial, Tx, Rx,  FullConfig, FifoThreshold}; 
    use hal::i2c::{Error, Config, I2c, I2cControl, I2cMaster, I2cResult};

    use nb::block;

    #[shared]
    #[derive(Debug, Display)]
    struct Shared {
        //exti: stm32::EXTI,
        counter:usize,
        txd:   Tx<USART1,FullConfig>,
        i2c:   I2c<I2C1, PB9<Output<OpenDrain>>, PB8<Output<OpenDrain>>>,
    }

    #[local]
    struct Local {
        tim17: Timer<TIM17>,
        blink: PA5<Output<PushPull>>,
        delay: Delay<TIM15>,
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

        let mut delay = ctx.device.TIM15.delay(&mut rcc);


        //let mut serial = ctx.device.USART2
        //    .usart(gpioa.pa2, gpioa.pa3,  
        let mut serial = ctx.device.USART1
            .usart(gpiob.pb6, gpiob.pb7,  
            FullConfig::default()
                .baudrate(115200.bps())
                .fifo_enable()
                .rx_fifo_threshold(FifoThreshold::FIFO_4_BYTES),
                &mut rcc)
            .unwrap();

        let (mut txd, mut rxd) = serial.split(); 

        writeln!(txd,
            "RTIC driven non Blocking i2c master test. Should be used together with one of test_*_slave tests\r"
            ).unwrap();        

        let mut counter:usize = 1;
        let mut blink = gpioa.pa5.into_push_pull_output();

        (Shared {
            counter,
            txd,
            i2c,
        }, 
        Local {
            tim17,
            blink,
            delay,
        }, 
        init::Monotonics())
    }

    #[idle(local = [delay],  shared = [counter, i2c, txd ])]
    fn idle(ctx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        let counter = ctx.shared.counter;
        let mut txd = ctx.shared.txd;
        let mut i2c = ctx.shared.i2c;
        let mut delay = ctx.local.delay;

        let mut buf_long: [u8; 50] = [0; 50]; //
        let mut buf_short: [u8; 20] = [0; 20]; //
        let mut buf_rcv: [u8; 20] = [0; 20]; //
        let mut buf_10: [u8; 10] = [0; 10]; //
        let mut buf_io = [0_u8;1];
    

        loop {
            let mut ready = false;
            txd.lock(|txd | { writeln!(txd, "\r\rStart of test\r").unwrap(); });

            // test 0x21: slave address 0x21 should be addressable half the cases
            for i in 0..buf_short.len() {
                buf_short[i] = 0x21 + (i as u8)
            }
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_write(0x21, &buf_short), Err(nb::Error::WouldBlock))  
                });
            }
            ready = false; delay.delay(10.ms());

            // test 0x61: slave address 0x61 should be addressable half the cases
            for i in 0..buf_short.len() {
                buf_short[i] = 0x61 + (i as u8)
            }

            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_write(0x61, &buf_short), Err(nb::Error::WouldBlock))  
                });
            }
            // 0x41 good case master write slave read: master does send 20 bytes slave receives 20 bytes
            ready = false; delay.delay(10.ms());
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_write(0x41, &buf_short), Err(nb::Error::WouldBlock))  
                });
            }
            // 0x42 bad case master write slave read: master does send less than 20 bytes
            for i in 0..buf_10.len() {
                buf_10[i] = 0x20 + (i as u8)
            }
            ready = false; delay.delay(10.ms());

            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_write(0x42, &buf_10), Err(nb::Error::WouldBlock) );
                });
            }
            // 0x43 bad case master write slave read: master does send more than 20 bytes, slave does NACK
            for i in 0..buf_long.len() {
                buf_long[i] = 0x61 + (i as u8)
            }
            ready = false; delay.delay(10.ms());
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_write(0x43, &buf_long) , Err(nb::Error::WouldBlock) );  
                });
            }
            // 0x44 master write_read good case: master sends and expects 20 bytes
            for i in 0..buf_short.len() {
                buf_short[i] = 0x41 + (i as u8)
            }
            ready = false; delay.delay(10.ms());
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_write_read(0x44, &buf_short, 20), Err(nb::Error::WouldBlock) );  
                });
            }
            // 0x48 master read slave write good case: exact 20 characters
            for i in 0..buf_short.len() {
                buf_short[i] = 0x61 + (i as u8)
            }
            ready = false; delay.delay(10.ms());
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_read(0x48, 20) , Err(nb::Error::WouldBlock) );  
                });
            }
            // 0x49 master read slave write bad  case: master expects 50 slave does send 20 characters
            for i in 0..buf_long.len() {
                buf_long[i] = 0x61 + (i as u8)
            }
            ready = false; delay.delay(10.ms());
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_read(0x49, 50), Err(nb::Error::WouldBlock) );  
                });
            }
            // 0x4A master read slave write bad  case: master expects 20 does slave does send 50 characters
            for i in 0..buf_short.len() {
                buf_short[i] = 0x41 + (i as u8)
            }
            ready = false; delay.delay(10.ms());
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_read(0x4A, 20), Err(nb::Error::WouldBlock))  
                });
            }
            // 0x4F test end and slave will present results
            let mut result: [u8; 2] = [0, 0];
            ready = false; delay.delay(10.ms());
            while !ready {
                i2c.lock(|i2c| {
                    ready =  ! matches!( i2c.master_read(0x4f, 2) , Err(nb::Error::WouldBlock))
                });
            }
            delay.delay(10_000.ms());
        }
    }


    #[task(binds = TIM17,  priority = 3, local = [tim17, blink], shared = [counter,i2c ])]
    fn timer17(ctx: timer17::Context) {
        let mut counter = ctx.shared.counter;
        let mut i2c = ctx.shared.i2c;

        ctx.local.blink.toggle().unwrap();
        let mut buf_short = [0_u8;1];
        (i2c, counter ).lock(|i2c, counter| {
            *counter += 1;
            
            // concurrent acces does not work yet, internal state gets confused
            // buf_short[0] = *counter as u8;
            //let _ = i2c.master_write(0x21, &buf_short);  
            i2c.execute_watchdog();
        });
        ctx.local.tim17.clear_irq();

    } // tim 17

    #[task(binds = I2C1,  priority = 4, local = [], shared = [i2c, txd ])]
    fn i2_irq(ctx: i2_irq::Context) {
        let mut txd = ctx.shared.txd;
        let mut i2c = ctx.shared.i2c;

        (i2c, txd).lock(|i2c, txd| {
            match i2c.check_isr_flags(){
                Ok( b) => {
                    match b {
                    I2cResult::Addressed(ad, dir) => {
                            writeln!(txd, "Addressed {:?} dir {:?}\r", ad, dir ).unwrap();
                        },     
                    I2cResult::Data(address,_, d) => 
                        // prevent too many output from the i2c extender output
                        if address == 0x4f {
                            write!(
                                txd,
                                "Result of the whole test as reported by the slave count/errors: {}/{}\r\r",
                                d[0],d[1]
                            ).unwrap();
                        }else {
                            for i in 0..d.len() {
                                write!(txd, "{}", d[i] as char).unwrap();
                            }
                            writeln!(txd, "!\r").unwrap();
                            irq_ok_case(txd, i2c as &mut I2cMaster, address);
                        }
                    }
                },
                Err(nb::Error::WouldBlock) => (), // ignore the WouldBlock error
                Err(err) => irq_bad_case(txd, i2c as &mut I2cMaster, err),
            }
        });
    } // I2C

    fn irq_ok_case(txd: &mut Tx<USART1,FullConfig>, i2c:  &dyn I2cMaster, address:u16) {
        match address {
            // 0x20 i2c IO extender with 8 leds
            0x20 => (),
    
            // test that the slave can change its address dynamically
            0x21 =>writeln!(txd, "Test 0x21 Found slave!\r").unwrap(),
            0x61 =>writeln!(txd, "Test 0x61 Found slave!\r").unwrap(),
    
            // 0x41 good case master write slave read: master does send 20 bytes slave receives 20 bytes
            0x41 => writeln!(txd, "Test 0x41 Ok\r").unwrap(),
            // 0x42 bad case master write slave read: master does send less than 20 bytes
            0x42 => writeln!(
                        txd,
                        "Test 0x42 Ok. (Master cannot detect that frame is too short)  \r"
                    )
                    .unwrap(),
            // 0x43 bad case master write slave read: master does send more than 20 bytes, slave does NACK
            0x43 => writeln!(
                        txd,
                        "Test 0x43 not ok expected error IncorrectFramesize: \r"
                    )
                    .unwrap(),
            // 0x44 master write_read good case: master sends and expects 20 bytes
            0x44 => {
                    write!(txd, "Test 0x44 Ok ").unwrap();
            
                    writeln!(
                        txd,
                        "Uppercase input should be transformed to lowercase, A -> b \r"
                    )
                    .unwrap();
                },
            0x48 => {
                    write!(txd, "Test 0x48 Ok ").unwrap();
                },
            // 0x49 master read slave write bad  case: master expects 50 slave does send 20 characters
            0x49 => {
                    write!(txd, "Test 0x49 Ok ").unwrap();
                },
            // 0x4A master read slave write bad  case: master expects 20 does slave does send 50 characters
            0x4A => {
                    write!(txd, "Test 0x4A Ok ").unwrap();
                    writeln!(txd, "\r").unwrap();
                },
            _ => (),    
        }
    }
    
    fn irq_bad_case(txd: &mut Tx<USART1,FullConfig>, i2c:  &dyn I2cMaster, err:nb::Error<Error>) {
        let address = i2c.get_address();
        match address {
   
        0x21 => writeln!(txd, "Test 0x21 error: {:?}\r", err).unwrap(),
        0x61 => writeln!(txd, "Test 0x61 error: {:?}\r", err).unwrap(),
    
        // 0x41 good case master write slave read: master does send 20 bytes slave receives 20 bytes
        0x41 => writeln!(txd, "Test 0x41 Error: {:?}\r", err).unwrap(),
    
        // 0x42 bad case master write slave read: master does send less than 20 bytes
        0x42 => writeln!(txd, "Test 0x42 error IncorrectFramesize: {:?}\r", err).unwrap(),
    
        // 0x43 bad case master write slave read: master does send more than 20 bytes, slave does NACK
        0x43 => writeln!(
                txd,
                "Test 0x43 Ok Expected IncorrectFrameSize: {:?}\r",
                err
            )
            .unwrap(),
        
        // 0x44 master write_read good case: master sends and expects 20 bytes
        0x44 => writeln!(txd, "Test 0x44 error: {:?}\r", err).unwrap(),
        
        // 0x48 master read slave write good case: exact 20 characters
        0x48 => writeln!(txd, "Test 0x48 unexpected error: {:?}\r", err).unwrap(),
        
        // 0x49 master read slave write bad  case: master expects 50 slave does send 20 characters
        0x49 => writeln!(txd, "Test 0x49 error: {:?}\r", err).unwrap(),
        
        // 0x4A master read slave write bad  case: master expects 20 does slave does send 50 characters
        0x4A => writeln!(txd, "Test 0x4A error: {:?}\r", err).unwrap(),
        
        // 0x4F test end and slave will present results
        0x4F => writeln!(txd, "Test 0x4F unexpected error: {:?}\r", err).unwrap(),
        _ => ()
        }
    }
    
    
} // App



