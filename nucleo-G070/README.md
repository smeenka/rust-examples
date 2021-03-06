# NUCLE0-g070rb examples


## Blink 
---
Blink the led on the NUCLE0-g070rb  board with an STM32G070 chip

Blinking is done with the help of the RTFM (RTIC) os library

cargo embed --example blink 

## Neopixel

* cargo embed --example neopixel   --release


## Neopixel5bits

This example uses a special variant of the neopixel library 
git = "https://github.com/smeenka/ws2812-spi-rs"
branch = "feature/halfduplex"

which runs on 4 mhz, and which has a precise timing for the neopixels.

The ability to set the datasize to 5 and to switch to half duplex is not yet in the stm32g0xx-hal crate with version 0.1.1. It can be found on the master branch in github. 

Please use next lines in cargo.toml:

    [dependencies.stm32g0xx-hal]
    default-features = false
    features = ["rt", "stm32g070"]
    version = "0.1.1"
    git="https://github.com/stm32-rs/stm32g0xx-hal"

To build and run this example:
*  cargo embed --example neopixel5bits   --release

## motors_pwd

This version shows the ability to control 4 motors with pwm at the same moment. All motors are controlled with a timer generated PWM. 
At least the stm32g070 (nucleo-G070 demo board) is needed for this, because we need timer 1 and timer 3 each with 4 channels.

Motors are controlled by connecting a serial terminal and use the serial emulation over USB.

A cheap motor controller like L9110S DC Stepper / Motor Driver Board HBridge can be used to give power to the motors.

There is a patch in the master branch of the stm32g0xx-hal not yet in the crate with version 0.1.1

Please use next lines in cargo.toml:

    [dependencies.stm32g0xx-hal]
    default-features = false
    features = ["rt", "stm32g070"]
    version = "0.1.1"
    git="https://github.com/stm32-rs/stm32g0xx-hal"

To build and run this example:
*  cargo embed --example motors_pwm 

## test_block_i2c_master and test_block_i2c_slave

This test should be used with 2 (nucleo) boards with i2c connected.
You have to check if uart is correct, as I use one nucleo board, and a board  with different uart settings.
On one board load the master test:
*  cargo embed --example test_block_i2c_master
On the other board load the slave test
*  cargo embed --example test_block_i2c_slave

The blocking variant of i2c is tested. 

The test will check:
* 2 good case for sending and reading from master to slave
* bad cases where the frame size is not correct between master and slave
* one good case for the master write_read with subaddressing

Cargo.toml:

    [dependencies.stm32g0xx-hal]
    features = ["rt", "stm32g070"]


## test_rtic_i2c_master and test_rtic_i2c_slave

This test should be used with 2 (nucleo) boards with i2c connected.
You have to check if uart is correct, as I use one nucleo board, and a board  with different uart settings.
On one board load the master test:
*  cargo embed --example test_rtic_i2c_master
On the other board load the slave test
*  cargo embed --example test_rtic_i2c_slave

The non-blocking variant of i2c is tested with RTIC.

The test will check:
* Address change capacibility of the slave
* 2 good case for sending and reading from master to slave
* bad cases where the frame size is not correct between master and slave
* one good case for the master write_read with subaddressing
* The watchdog does prevent the i2c bus from hanging!
* One can test the watchdog capability by disconnecting and connecting the slave at any time, and check that the bus never hangs

Note that the blocking and nonblocking variants in the test can be interchanged, 
* master in blocking variant connected to a slave in nonblocking variant
* master in non-blocking variant connected to a slave in blocking variant

Cargo.toml:

    [dependencies.stm32g0xx-hal]
    # Needed to disable the default feature "i2c-blocking"
    default-features = false        
    features = ["rt", "stm32g070", "i2c-nonblocking"]


## nb_i2c_master_simple

Test the non-blocking variant of the master in blocking mode with the block! macro.
And IO expander (with leds connected) at address 0x20 should be connected and/or an M5 joystick on address 
0x52.
Note that both on the bus leads to buserrors, possible due to voltage incompatibility (3.3 and 5 V).

Cargo.toml:

    [dependencies.stm32g0xx-hal]
    # Needed to disable the default feature "i2c-blocking"
    default-features = false
    features = ["rt", "stm32g070", "i2c-nonblocking"]
