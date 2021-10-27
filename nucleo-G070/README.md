# NUCLE0-g070rb examples

## Blink 
---
Blink the led on the NUCLE0-g070rb  board with an STM32G070 chip

Blinking is done with the help of the RTFM (RTIC) os library

cargo embed --example blink 

## Neopixel

cargo embed --example neopixel   --release


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
