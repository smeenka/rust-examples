#![no_std]
#![no_main]
#![allow(warnings)]

use rtic::app;

macro_rules! checkIsWrite {
    ($txd:ident, $direction:ident) => {
        match $direction {
            MasterWriteSlaveRead => (),
            _ => {
                write!($txd, "Error incorrect direction {:?}\r", $direction).unwrap();
                return;
            }
        }
    };
}
macro_rules! checkIsRead {
    ($txd:ident, $direction:ident) => {
        match $direction {
            MasterReadSlaveWrite => (),
            _ => {
                write!($txd, "Error incorrect direction {:?}\r", $direction).unwrap();
                return;
            }
        }
    };
}

#[app(device = hal::stm32, peripherals = true)]
mod app {
    extern crate rtic;
    extern crate stm32g0xx_hal as hal;

    use core::fmt::{Write, Debug, Display};
    use core::fmt;
    
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
    use hal::timer::delay::{Delay };
    use hal::timer::pins::TimerPin;
    use hal::stm32::{TIM17,TIM15};
    use hal::serial::{Serial, Tx, Rx,  FullConfig, FifoThreshold}; 
    use hal::i2c::{Error, Config, I2c, I2cControl, I2cSlave, I2cResult, SlaveAddressMask,I2cDirection};

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
        delay: Delay<TIM15>,
        count:usize,
        errors:usize,
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
        let mut configi2c = hal::i2c::Config::with_timing(0x2020_151b);
        configi2c.slave_address(0x61);
        configi2c.slave_address_2(0x40, SlaveAddressMask::MaskFourBits);
    
        let mut i2c = ctx.device.I2C1.i2c(sda, scl, configi2c, &mut rcc);
        i2c.listen();

        let mut delay = ctx.device.TIM15.delay(&mut rcc);


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

        writeln!(txd,
            "RTIC driven non Blocking i2c slave test. Should be used together with one of test_*_master tests\r"
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
            errors:0,
            count:0,
        }, 
        init::Monotonics())
    }

    #[idle(local = [delay],  shared = [counter, txd ])]
    fn idle(ctx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        let counter = ctx.shared.counter;
        let mut txd = ctx.shared.txd;
        let mut delay = ctx.local.delay;
    

        let mut ready = false;
        loop {
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
        });
        ctx.local.tim17.clear_irq();

    } // tim 17

    #[task(binds = I2C1,  priority = 4, local = [errors,count], shared = [i2c, txd ])]
    fn i2_irq(ctx: i2_irq::Context) {
        let mut txd = ctx.shared.txd;
        let mut i2c = ctx.shared.i2c;
        let mut errors = ctx.local.errors;
        let mut count = ctx.local.count;

        (i2c, txd).lock(|i2c, txd| {
            match i2c.check_isr_flags(){
                Ok( b) => {
                    match b {
                        I2cResult::Addressed(ad, dir) => {
                            write!(txd, "Address {:x}, ", ad).unwrap();
                            addressed_case(txd, i2c as &mut I2cSlave, ad, dir, count,errors);
                        },     
                        I2cResult::Data(address, _, d) => {
                                write!(txd, " len {}, ", d.len()).unwrap();
                                for i in 0..d.len() {
                                    write!(txd, "{}", d[i] as char).unwrap();
                                }
                                writeln!(txd, "\r").unwrap();
                                let len = d.len();
                                irq_ok_case(txd, i2c as &mut I2cSlave, address, len,  &mut errors);
                            }
                        }
                    
                },
                Err(nb::Error::WouldBlock) => (), // ignore the WouldBlock error
                Err(err) => {
                    writeln!(txd, "\r").unwrap();
                    irq_bad_case(txd, i2c as &mut I2cSlave, err, &mut errors)
                },
            }
        });
    } // I2C

    fn addressed_case(txd: &mut fmt::Write, i2c:  &mut dyn I2cSlave, address:u16, dir:I2cDirection, count:&mut usize, errors:&mut usize){
        let mut buf_long: [u8; 50] = [0; 50]; //
        let mut buf_short: [u8; 20] = [0; 20]; //

        *count += 1;    
        match address {
            0x44 => {
                // 0x44 master read write good case
                match dir {
                    I2cDirection::MasterWriteSlaveRead => (),
                    I2cDirection::MasterReadSlaveWrite => {
                        let buf = [0x31_u8;20];
                        let _ = i2c.slave_write(&buf); 
                    }
                };
            },
            0x48 => {
                // 0x48 master read slave write good case: exact 20 characters
                for i in 0..buf_short.len() {
                    buf_short[i] = 0x61 + (i as u8)
                }
                checkIsRead!(txd, dir);
                let _ = i2c.slave_write(&buf_short);
            },
            0x49 => {
                // 0x49 master read slave write bad  case: master expects 50 does slave does send 20 characters
                checkIsRead!(txd, dir);
                let _ = i2c.slave_write(&buf_short); 
            },
            0x4A => {
                // 0x4A master read slave write bad  case: master expects 20 does slave does send 50 characters
                checkIsRead!(txd, dir);
                let _ =  i2c.slave_write(&buf_long);
                
            },
            0x4B => {
                // 0x4B master write_read good case each 20 chars
                checkIsRead!(txd, dir);
                let _ = i2c.slave_write(&buf_long); 
                
            },
            0x4F => {
                checkIsRead!(txd, dir);
                let result: [u8; 2] = [*count as u8, *errors as u8 ];
                let _ =  i2c.slave_write(&result); 
                writeln!(
                    txd,
                    "Test finished. nr tests/nr errors: {}/{}!\r",
                    *count , *errors
                )
                .unwrap();
                writeln!(txd, "-----\r").unwrap();
                *count = 0;
                *errors = 0;
            }
            _ => (),
        }

    }

    fn irq_ok_case(txd: &mut fmt::Write, i2c:  &mut dyn I2cSlave, address:u16, len:usize, errors:&mut usize) {
        let expected = match address {
            0x21 | 0x61 | 0x41 | 0x44 | 0x48 => 20,
            0x42 => 10,
            0x43 => 50,
            // 0x49 master read slave write bad  case: master expects 50 slave does send 20 characters
            // slave cannot know that master wants more data
            0x49 => 20,
            // 0x4A master read slave write bad  case: master expects 20 does slave does send 50 characters
            // 21 bytes are send, as the slave does send a nack on byte 21
            0x4A => 21,
            0x4F => 2,
            _ => 1,
        };
        match address {
            0x21 => i2c.set_address(0x61),
            0x61 => i2c.set_address(0x21),
            _ => (),
        }
        if expected != len {
            *errors +=1;
            writeln!(txd, "Test {:x} frame size error expected {} actual {}  \r", address, expected, len).unwrap();
        }else {
            writeln!(txd, "Test {:x} Ok frame size:{}  \r", address, len).unwrap();
        }
    }
    
    fn irq_bad_case(txd: &mut fmt::Write, i2c:  &dyn I2cSlave, err:nb::Error<Error>, errors:&mut usize) {
        let address = i2c.get_address();
        *errors += 1; writeln!(txd, "Error Test: {:x} Error: {:?}\r", address, err).unwrap();
    }
} // App



