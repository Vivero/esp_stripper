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

Here's a minimal example to get an LED strip lit:

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_stripper.h"
#include "driver/gpio.h"
#include <stdio.h>

static void StripperTask(void* pvParameters);

void app_main(void)
{
    printf("Hello esp_stripper!\n");

    // Initialize esp_stripper on Core 1 to minimize interference with WiFi
    xTaskCreatePinnedToCore(
        StripperTask,
        "Stripper",
        2048 /* usStackDepth */,
        NULL, /* pvParameters */
        10 /* uxPriority */,
        NULL, /* flags */
        1 /* xCoreID */);
}

static void StripperTask(void* pvParameters)
{
    esp_stripper_init();

    const uint8_t stripper_index = 0;
    esp_stripper_ctrl_config_t cfg =
    {
        .ChipType = ESP_STRIPPER_CHIP_WS2812B,
        .uGpioNum = GPIO_NUM_27,
        .uNumPixels = 50,
    };
    esp_stripper_config_controller(stripper_index, &cfg);
    esp_stripper_clear_pixels(stripper_index);

    // Light up the first 5 pixels on the strip: red, blue, green, yellow, white
    uint32_t my_colors[6] = {0x000000FF, 0x0000FF00, 0x00FF0000, 0x0000FFFF, 0x00FFFFFF, 0x00000000};
    esp_stripper_load_pixels(stripper_index, my_colors, 5, 0);

    uint8_t red = 0;
    for (;;)
    {
        // Cycle the 6th pixel with shades of red
        red++;
        my_colors[5] = (uint32_t)red;
        esp_stripper_load_pixels(stripper_index, &my_colors[5], 1, 5);

        // Refresh the display every 1 second
        esp_stripper_display(); // This is a blocking call
        vTaskDelay((1000 /* msec */) / portTICK_PERIOD_MS);
    }

    esp_stripper_deinit();
    vTaskDelete(NULL);
}
```