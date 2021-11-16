#![allow(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use stm32g0xx_hal::cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32g0xx_hal as hal;

use core::fmt::Write;
use hal::i2c::{Config, Error, I2cDirection, I2cSlave, SlaveAddressMask};
use hal::prelude::*;
use hal::rcc::{self, PllConfig};
use hal::serial::FullConfig;
use hal::stm32;
use rt::entry;

macro_rules! checkIsWrite {
    ($usart:ident, $direction:ident) => {
        match $direction {
            MasterWriteSlaveRead => (),
            _ => {
                write!($usart, "Error incorrect direction {:?}\r", $direction).unwrap();
                continue;
            }
        }
    };
}
macro_rules! checkIsRead {
    ($usart:ident, $direction:ident) => {
        match $direction {
            MasterReadSlaveWrite => (),
            _ => {
                write!($usart, "Error incorrect direction {:?}\r", $direction).unwrap();
                continue;
            }
        }
    };
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    // Configure APB bus clock to 16MHz
    let rcc_cfg = rcc::Config::hsi(rcc::Prescaler::NotDivided);
    let mut rcc = dp.RCC.freeze(rcc_cfg);

    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let config_uart = FullConfig::default();
    let mut usart = dp
        .USART2
        .usart(gpioa.pa2, gpioa.pa3, config_uart, &mut rcc)
        .unwrap();
    writeln!(
        usart,
        "i2c slave test. Should be used together with i2c_master\r"
    )
    .unwrap();

    let sda = gpiob.pb9.into_open_drain_output();
    let scl = gpiob.pb8.into_open_drain_output();

    let mut configi2c = hal::i2c::Config::with_timing(0x2020_151b);
    configi2c.slave_address(0x20);
    configi2c.slave_address_2(0x40, SlaveAddressMask::MaskFourBits);

    let mut i2c = dp.I2C1.i2c(sda, scl, configi2c, &mut rcc);

    let mut buf_long: [u8; 50] = [0; 50]; // buffer is longer than master will send: wait for STOP condition
    let mut buf_short: [u8; 20] = [0; 20]; // buffer is longer than master will send: wait for STOP condition
    let mut errors = 0;
    let mut tcount = 0;
    loop {
        let mut address = 0;
        let mut dir = I2cDirection::MasterReadSlaveWrite;
        for i in 0..buf_short.len() {
            buf_short[i] = 0x61 + (i as u8)
        }
        for i in 0..buf_long.len() {
            buf_long[i] = 0x41 + (i as u8)
        }

        match i2c.slave_wait_addressed() {
            Ok(tup) => {
                address = tup.0;
                dir = tup.1;
                write!(usart, "{:x}: ", address).unwrap()
            }
            Err(err) => writeln!(usart, "addressed error: {:?}\r", err).unwrap(),
        }
        tcount += 1;
        match address {
            0x41 => {
                // 0x41 good case master write slave read: master does send 20 bytes slave receives 20 bytes
                checkIsWrite!(usart, dir);
                match i2c.slave_read(&mut buf_short) {
                    Ok(_) => {
                        for i in 0..buf_short.len() {
                            write!(usart, "{}", buf_short[i] as char).unwrap();
                        }
                        writeln!(usart, "!\r").unwrap();
                    }
                    Err(err) => {
                        errors += 1;
                        writeln!(usart, "Test 0x41 failed. Error: {:?}\r", err).unwrap()
                    }
                };
            }
            0x42 => {
                // 0x42 bad case master write slave read: master does send less than 20 bytes
                checkIsWrite!(usart, dir);
                match i2c.slave_read(&mut buf_short) {
                    Ok(_) => {
                        {
                            errors += 1;
                            writeln!(
                                usart,
                                "Test 0x42 failed. should return error IncorrectFrameSize"
                            )
                            .unwrap()
                        };
                    }
                    Err(err) => match err {
                        Error::IncorrectFrameSize(n) => writeln!(
                            usart,
                            "Test 0x42 Ok Received expected IncorrectFrameSize bytes: {}\r",
                            n
                        )
                        .unwrap(),
                        _ => {
                            errors += 1;
                            writeln!(usart, "Test 0x42 failed. Error: {:?}\r", err).unwrap()
                        }
                    },
                };
            }
            0x43 => {
                // 0x43 bad case master write slave read: master does send more than 20 bytes, slave does NACK
                checkIsWrite!(usart, dir);
                match i2c.slave_read(&mut buf_short) {
                    Ok(_) => {
                        {
                            writeln!(usart, "Test 0x43 Ok. Slave cannot detect that the master wants to send more\r").unwrap()
                        };
                    }
                    Err(err) => {
                        errors += 1;
                        writeln!(usart, "Test 0x41 failed. Error: {:?}\r", err).unwrap()
                    }
                };
            }
            0x44 => {
                // 0x44 master read write good case
                match dir {
                    I2cDirection::MasterWriteSlaveRead => {
                        i2c.slave_sbc(false);
                        match i2c.slave_read(&mut buf_short) {
                            Ok(_) => {
                                writeln!(usart, "Test 0x44 receive Ok. \r").unwrap();
                            }
                            Err(err) => {
                                errors += 1;
                                writeln!(usart, "Test 0x44 recv failed. Error: {:?}\r", err)
                                    .unwrap()
                            }
                        };
                        // make transformation from 'A' to 'b'
                        for idx in 0..buf_short.len() {
                            buf_short[idx] += 0x21;
                        }
                    }
                    I2cDirection::MasterReadSlaveWrite => {
                        i2c.slave_sbc(true);
                        match i2c.slave_write(&buf_short) {
                            Ok(_) => {
                                {
                                    writeln!(usart, "Test 0x44 send Ok. \r").unwrap()
                                };
                            }
                            Err(err) => {
                                errors += 1;
                                writeln!(usart, "Test 0x44 send failed. Error: {:?}\r", err)
                                    .unwrap()
                            }
                        };
                    }
                    _ => {
                        writeln!(usart, "UU\r").unwrap();
                    }
                };
            }
            0x48 => {
                // 0x48 master read slave write good case: exact 20 characters
                for i in 0..buf_short.len() {
                    buf_short[i] = 0x61 + (i as u8)
                }
                checkIsRead!(usart, dir);
                match i2c.slave_write(&buf_short) {
                    Ok(_) => {
                        writeln!(usart, "Test 0x48 ok: \r").unwrap();
                    }
                    Err(err) => {
                        errors += 1;
                        writeln!(usart, "Test 0x48 failed. Error: {:?}\r", err).unwrap()
                    }
                };
            }
            0x49 => {
                // 0x49 master read slave write bad  case: master expects 50 does slave does send 20 characters
                checkIsRead!(usart, dir);
                match i2c.slave_write(&buf_short) {
                    Ok(_) => {
                        writeln!(usart, "Test 0x49 Ok. Slave cannot detect this error case that the master wants to read more\r").unwrap();
                    }
                    Err(err) => {
                        errors += 1;
                        writeln!(usart, "Test 0x48 failed. Error: {:?}\r", err).unwrap()
                    }
                }
            }
            0x4A => {
                // 0x4A master read slave write bad  case: master expects 20 does slave does send 50 characters
                checkIsRead!(usart, dir);
                match i2c.slave_write(&buf_long) {
                    Ok(_) => {
                        errors += 1;
                        writeln!(
                            usart,
                            "Test 0x4A should fail with IncorrectFrameSize error\r"
                        )
                        .unwrap();
                    }
                    Err(err) => match err {
                        Error::IncorrectFrameSize(n) => writeln!(
                            usart,
                            "Test  0x4A Ok Received expected IncorrectFrameSize bytes: {}\r",
                            n
                        )
                        .unwrap(),
                        _ => {
                            errors += 1;
                            writeln!(usart, "Test 0x4A failed. Error: {:?}\r", err).unwrap()
                        }
                    },
                }
            }
            0x4B => {
                // 0x4B master write_read good case each 20 chars
                checkIsRead!(usart, dir);
                match i2c.slave_write(&buf_long) {
                    Ok(_) => {
                        errors += 1;
                        writeln!(
                            usart,
                            "Test 0x4A should fail with IncorrectFrameSize error\r"
                        )
                        .unwrap();
                    }
                    Err(err) => match err {
                        Error::IncorrectFrameSize(n) => writeln!(
                            usart,
                            "Test  0x4A Ok Received expected IncorrectFrameSize bytes: {}\r",
                            n
                        )
                        .unwrap(),
                        _ => {
                            errors += 1;
                            writeln!(usart, "Test 0x4A failed. Error: {:?}\r", err).unwrap()
                        }
                    },
                }
            }
            0x4F => {
                checkIsRead!(usart, dir);
                let result: [u8; 2] = [tcount, errors];
                match i2c.slave_write(&result) {
                    Ok(_) => {
                        writeln!(usart, "Test result send to master\r").unwrap();
                    }
                    Err(err) => writeln!(usart, "Unexpected error. Error: {:?}\r", err).unwrap(),
                }
                writeln!(
                    usart,
                    "Test finished. nr tests/nr errors: {}/{}!\r",
                    tcount, errors
                )
                .unwrap();
                writeln!(usart, "-----\r").unwrap();
                tcount = 0;
                errors = 0;
            }
            _ => (),
        }
    }
}
