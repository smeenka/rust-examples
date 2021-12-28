#![allow(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use cortex_m_rt as rt;
use panic_halt;
use stm32g0xx_hal as hal;
use stm32g0xx_hal::cortex_m;

use core::fmt::Write;
use hal::i2c::{Config};
//use hal::i2c::I2c;
use hal::i2c::nonblocking::{I2cMaster, I2cControl};
use hal::prelude::*;
use hal::rcc::{self, PllConfig};
use hal::serial::FullConfig;
use hal::stm32;
use rt::entry;
use nb::{block};

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    // Configure APB bus clock to 16MHz
    let rcc_cfg = rcc::Config::hsi(rcc::Prescaler::NotDivided);
    let mut rcc = dp.RCC.freeze(rcc_cfg);

    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut delay = dp.TIM15.delay(&mut rcc);
    let config_uart = FullConfig::default();

    
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut usart = dp
        .USART2
        .usart(gpioa.pa2, gpioa.pa3, config_uart, &mut rcc)
        .unwrap();
    /*
    let mut usart = dp
        .USART1
        .usart(gpiob.pb6, gpiob.pb7, config_uart, &mut rcc)
        .unwrap();
    */
    writeln!(usart,"i2c master test nonblocking, without interrupts\r").unwrap();
    writeln!(usart, "Connect a IO expander on address 0x20 for output.\r").unwrap();        
    writeln!(usart, "Connect a M5 Joystick on address 0x52 for input.\r").unwrap();        

    let sda = gpiob.pb9.into_open_drain_output();
    let scl = gpiob.pb8.into_open_drain_output();

    let configi2c = hal::i2c::Config::with_timing(0x2020_151b);

    let mut i2c = dp.I2C1.i2c(sda, scl, configi2c, &mut rcc);
    let mut buf_ioext: [u8; 2] = [0;2]; //
    let mut counter:u8 = 0;
    delay.delay(100.ms());
    loop {
        counter +=1;
        counter = counter % 0xFF;
        // test 1: slave address 0x61 should not be addressable
        buf_ioext[1] += 1;
        writeln!(usart, "Counter {}\r", buf_ioext[1]).unwrap();
        // test a master read, but with incorrect data size (2) as the slave wil try to send 3 bytes
        block!(i2c.master_read(0x52, 2));
        match block!(i2c.check_isr_flags()){
            Ok( b) => writeln!(usart, "Joystick: x:{:x} y:{:x} no third byte read \r", b[0],b[1]).unwrap(),
            Err(err) => writeln!(usart, "0x52 Error: {:?}\r", err).unwrap(),
        }
        // test non existing address, expect a NACK
        block!(i2c.master_write(0x30, &buf_ioext) );
        match block!(i2c.check_isr_flags()){
            Ok(_) => (),
            Err(err) => writeln!(usart, "0x21 Error: {:?}\r", err).unwrap(),
        }
        // test existing address, expect Ok
        block!(i2c.master_write(0x21, &buf_ioext) );
        match block!(i2c.check_isr_flags()){
            Ok(_) => (),
            Err(err) => writeln!(usart, "0x21 Error: {:?}\r", err).unwrap(),
        }
        // test write with subaddressingexisting address, expect Ok
        block!(i2c.master_write_restart(0x21, &buf_ioext)); 
        match block!(i2c.check_isr_flags()){
            Ok(_) => (),
            Err(err) => writeln!(usart, "0x21 restart Error: {:?}\r", err).unwrap(),
        }
        // do a read to finish the master write with sub addressing
        block!(i2c.master_read(0x52, 3)); 
        match block!(i2c.check_isr_flags()){
            Ok( b) => writeln!(usart, "Joystick: x:{:x} y:{:x} button {} \r", b[0],b[1], b[2]).unwrap(),
            Err(err) => writeln!(usart, "0x52 Error: {:?}\r", err).unwrap(),
        }
        delay.delay(100.ms());
    }
}
