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

which runs on 4 mhz, and which has a precise timing for the neopixels
It is depending on a feature branch of the hal which is not yet merged with master
This feature branch does support setting the data size and switch spi to half duplex

cargo embed --example neopixel5bits   --release