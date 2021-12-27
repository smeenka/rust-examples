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
use hal::i2c::Config;
//use hal::i2c::I2c;
use hal::i2c::blocking;
use hal::prelude::*;
use hal::rcc::{self, PllConfig};
use hal::serial::FullConfig;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    // Configure APB bus clock to 16MHz
    let rcc_cfg = rcc::Config::hsi(rcc::Prescaler::NotDivided);
    let mut rcc = dp.RCC.freeze(rcc_cfg);

    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut delay = dp.TIM15.delay(&mut rcc);
    let config_uart = FullConfig::default();

    /*
        let gpioa = dp.GPIOA.split(&mut rcc);
        let mut usart = dp
            .USART2
            .usart(gpioa.pa2, gpioa.pa3, config_uart, &mut rcc)
            .unwrap();
    */
    let mut usart = dp
        .USART1
        .usart(gpiob.pb6, gpiob.pb7, config_uart, &mut rcc)
        .unwrap();

    writeln!(
        usart,
        "Blocking i2c master test. Should be used together with one of test_*_slave tests\r"
    )
    .unwrap();

    let sda = gpiob.pb9.into_open_drain_output();
    let scl = gpiob.pb8.into_open_drain_output();

    let configi2c = hal::i2c::Config::with_timing(0x2020_151b);

    let mut i2c = dp.I2C1.i2c(sda, scl, configi2c, &mut rcc);

    let mut buf_long: [u8; 50] = [0; 50]; //
    let mut buf_short: [u8; 20] = [0; 20]; //
    let mut buf_rcv: [u8; 20] = [0; 20]; //
    let mut buf_10: [u8; 10] = [0; 10]; //
    let mut buf_io = [0_u8;1];

    delay.delay(100.ms());
    loop {
        writeln!(usart, "\r\rStart of test\r").unwrap();

        // test 1: slave address 0x61 should not be addressable
        for i in 0..buf_short.len() {
            buf_short[i] = 0x41 + (i as u8)
        }

        match i2c.write(0x61, &buf_short) {
            Ok(_) => writeln!(usart, "Test 1 Error: would expect nack\r").unwrap(),
            Err(err) => writeln!(usart, "Test 1 OK: expected NACK error: {:?}\r", err).unwrap(),
        }
        // 0x41 good case master write slave read: master does send 20 bytes slave receives 20 bytes
        match i2c.write(0x41, &buf_short) {
            Ok(_) => writeln!(usart, "Test 0x41 Ok\r").unwrap(),
            Err(err) => writeln!(usart, "Test 0x41 Error: {:?}\r", err).unwrap(),
        }
        // 0x42 bad case master write slave read: master does send less than 20 bytes
        for i in 0..buf_10.len() {
            buf_10[i] = 0x20 + (i as u8)
        }
        match i2c.write(0x42, &buf_10) {
            Ok(_) => writeln!(
                usart,
                "Test 0x42 Ok. (Master cannot detect that frame is too short)  \r"
            )
            .unwrap(),
            Err(err) => writeln!(usart, "Test 0x42 error IncorrectFramesize: {:?}\r", err).unwrap(),
        }
        // 0x43 bad case master write slave read: master does send more than 20 bytes, slave does NACK
        for i in 0..buf_long.len() {
            buf_long[i] = 0x61 + (i as u8)
        }
        match i2c.write(0x43, &buf_long) {
            Ok(_) => writeln!(
                usart,
                "Test 0x43 not ok expected error IncorrectFramesize: \r"
            )
            .unwrap(),
            Err(err) => writeln!(
                usart,
                "Test 0x43 Ok Expected IncorrectFrameSize: {:?}\r",
                err
            )
            .unwrap(),
        }
        // 0x44 master write_read good case: master sends and expects 20 bytes
        for i in 0..buf_short.len() {
            buf_short[i] = 0x41 + (i as u8)
        }
        for i in 0..buf_rcv.len() {
            buf_rcv[i] = 0x30 + (i as u8)
        }
        match i2c.write_read(0x44, &buf_short, &mut buf_rcv) {
            Ok(_) => {
                write!(usart, "Test 0x44 Ok ").unwrap();

                writeln!(
                    usart,
                    "Uppercase input should be transformed to lowercase, A -> b \r"
                )
                .unwrap();

                for i in 0..buf_rcv.len() {
                    write!(usart, "{}", buf_rcv[i] as char).unwrap();
                }
                writeln!(usart, "\r").unwrap();
            }
            Err(err) => writeln!(usart, "Test 0x44 error: {:?}\r", err).unwrap(),
        }
        // 0x48 master read slave write good case: exact 20 characters
        for i in 0..buf_short.len() {
            buf_short[i] = 0x61 + (i as u8)
        }
        match i2c.read(0x48, &mut buf_short) {
            Ok(_) => {
                write!(usart, "Test 0x48 Ok ").unwrap();
                for i in 0..buf_short.len() {
                    write!(usart, "{}", buf_short[i] as char).unwrap();
                }
                writeln!(usart, "\r").unwrap();
            }
            Err(err) => writeln!(usart, "Test 0x48 unexpected error: {:?}\r", err).unwrap(),
        }
        // 0x49 master read slave write bad  case: master expects 50 slave does send 20 characters
        for i in 0..buf_long.len() {
            buf_long[i] = 0x61 + (i as u8)
        }
        match i2c.read(0x49, &mut buf_long) {
            Ok(_) => {
                write!(usart, "Test 0x49 Ok ").unwrap();
                for i in 0..buf_long.len() {
                    write!(usart, "{}", buf_long[i] as char).unwrap();
                }
                writeln!(usart, "\r").unwrap();
            }
            Err(err) => writeln!(usart, "Test 0x49 error: {:?}\r", err).unwrap(),
        }
        // 0x4A master read slave write bad  case: master expects 20 does slave does send 50 characters
        for i in 0..buf_short.len() {
            buf_short[i] = 0x41 + (i as u8)
        }
        match i2c.read(0x4A, &mut buf_short) {
            Ok(_) => {
                write!(usart, "Test 0x4A Ok ").unwrap();

                for i in 0..buf_short.len() {
                    write!(usart, "{}", buf_short[i] as char).unwrap();
                }
                writeln!(usart, "\r").unwrap();
            }
            Err(err) => writeln!(usart, "Test 0x4A error: {:?}\r", err).unwrap(),
        }
        // 0x4F test end and slave will present results
        let mut result: [u8; 2] = [0, 0];
        match i2c.read(0x4F, &mut result) {
            Ok(_) => {
                write!(
                    usart,
                    "Result of the whole test as reported by the slave count/errors: {}/{}\r\r",
                    result[0], result[1]
                )
                .unwrap();
            }
            Err(err) => writeln!(usart, "Test 0x4F unexpected error: {:?}\r", err).unwrap(),
        }
        writeln!(usart, "\r").unwrap();
        let mut joyb = [0_u8;3];
        match i2c.read(0x52, &mut joyb) {
            Ok(_) => {
                write!(usart, "Test 0x52 Ok ").unwrap();
                for i in 0..joyb.len() {
                    write!(usart, " {} ", joyb[i] as char).unwrap();
                }
                writeln!(usart, "\r").unwrap();
            }
            Err(err) => writeln!(usart, "Test 0x52 unexpected error: {:?}\r", err).unwrap(),
        }
        match i2c.write(0x21, &mut buf_io) {
            Ok(_) => (),
            Err(err) => writeln!(usart, "Test 0x21 unexpected error: {:?}\r", err).unwrap(),
        }
        buf_io[0] += 1;
        delay.delay(10_000.ms());
    }
}
