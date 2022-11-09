# esp_stripper

Put on a show with this LED strip driver!

Designed for use with ESP32 applications built on the ESP-IDF framework.

------

The reference implementation for driving LED strips using the RMT peripheral
of the ESP32 is broken (official ESP-IDF example [here](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/led_strip)).
Or at least it appears to be when used alongside WiFi, Bluetooth, or other
processing-intensive applications (frequent flash reads/writes, etc.). In
these scenarios the RMT driver, is prone to introduce artifacts and glitches in
the data transmission stream, resulting in flashing and random colors
shown on the LED strip.

`esp_stripper` is a lean, performant, C-based port of the excellent _FastLED_ driver (Github: [https://github.com/FastLED/FastLED/tree/master/src/platforms/esp/32](https://github.com/FastLED/FastLED/tree/master/src/platforms/esp/32)).
This implementation is not vulnerable to the flickers and flashes of the basic
RMT driver implementation.

## Installation

`esp_stripper` is meant to be used as a _component_ as described in the ESP-IDF
Build System [documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html). Simply clone this repo into the `components` directory of your application:

```
cd <your-esp32-app>/components
git clone git@github.com:Vivero/esp_stripper.git
```

Alternatively, copy the `esp_stripper.h` and `esp_stripper.c` files into your project and include/link them as you see fit. Or add this repo as a submodule in your own app repo.

## Usage

API usage and implementation details are described in the header file `esp_stripper.h`.

## Example

