/* MIT License
 *
 * Copyright (C) 2022 Charlie De Vivero
 *
 * Integration into FastLED ClocklessController
 * Copyright (c) 2018,2019,2020 Samuel Z. Guyer
 * Copyright (c) 2017 Thomas Basler
 * Copyright (c) 2017 Martin F. Falatic
 *
 * Copyright (c) 2013 FastLED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * -----------------------------------------------------------------------------
 *
 * This ESP-IDF component implements an LED strip driver, heavily based on the
 * code and architecture of FastLED (https://github.com/FastLED/FastLED).
 * The original code was ported to C and is designed to integrate into ESP-IDF
 * applications as a component.
 *
 * The architecture of this driver is directly borrowed from FastLED:
 * "Controllers" are configured to drive a specified number of LEDs, on a
 * specified GPIO pin, for a specified LED IC type (WS2812, SK6812RGBW, etc.).
 * The ESP32's RMT peripheral channels are dynamically assigned to drive
 * each Controller, until all color pixel data from all Controllers has been
 * transmitted.
 * The way that the original FastLED's interrupt routines were written allows
 * for precise transmission timings without letting other components of the
 * ESP32 interfere (such as WiFi, Bluetooth, etc.). `esp_stripper` maintains
 * these advantages.
 *
 * Nominally, the application of this driver is as follows:
 * - User shall initialize the `esp_stripper` component once
 * - Configure individual controllers as needed
 * - Load color pixel data onto each controller
 * - Call the display function
 *
 * The display function is a blocking call. Controller configurations can be
 * changed at runtime at any time while the display function is not running.
 * None of these functions are thread-safe, and undefined behavior may occur if
 * any of these functions are called from different Tasks at the same time.
 *
 * See API comments for each function in this header file for more details.
 *
 * -----------------------------------------------------------------------------
 *
 * To re-iterate -- MOST of this code has been copied from the FastLED
 * implementation here: https://github.com/FastLED/FastLED/tree/master/src/platforms/esp/32
 * - clockless_rmt_esp32.h
 * - clockless_rmt_esp32.cpp
 *
 * Credit is hereby attributed to the original author(s).
 *
 */

#ifndef ESP_STRIPPER_H
#define ESP_STRIPPER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


/* -------------------------------------------------------------------------- */
/*                                  Constants                                 */
/* -------------------------------------------------------------------------- */

/// @brief Defines how many LED strip controller abstractions are supported at
///        compile-time. One controller per GPIO output. Controller output
///        will be parallelized as much as possible (nominally 4 simultaneous signals).
#define ESP_STRIPPER_MAX_CONTROLLERS 8


/* -------------------------------------------------------------------------- */
/*                                    Types                                   */
/* -------------------------------------------------------------------------- */

typedef enum
{
    ESP_STRIPPER_ERR_OK = 0,
    ESP_STRIPPER_ERR_NOT_INIT = -1,
    ESP_STRIPPER_ERR_TX_OVERRUN = -2,
} esp_stripper_err_t;

/// @brief The LED strip types that are supported. Each type dictates the bit
///        timings and color data order to be transmitted on the output pins.
typedef enum
{
    ESP_STRIPPER_CHIP_WS2812B = 0,
    ESP_STRIPPER_CHIP_WS2811,
    ESP_STRIPPER_CHIP_SK6812RGBW,
} esp_stripper_chip_type_t;

/// @brief Configuration for each LED strip controller.
typedef struct
{
    esp_stripper_chip_type_t    ChipType;   /*!< LED chip type on the strip */
    uint8_t                     uGpioNum;   /*!< Data signal output pin */
    uint16_t                    uNumPixels; /*!< Number of color pixels to transmit at a time */
} esp_stripper_ctrl_config_t;


/* -------------------------------------------------------------------------- */
/*                          Public Interface Methods                          */
/* -------------------------------------------------------------------------- */

/// @brief  Initialize this controller subsystem. Shall only be called once.
///
/// @note   Recommend calling this from a task pinned to Core 1 of the ESP32
///         processor, as this avoids interference with WiFi.
void esp_stripper_init();


/// @brief  Apply the provided configuration to the LED strip controller given by index.
///
/// @note   This function will allocate memory on the heap to contain all the
///         color data for this LED strip, i.e. `uNumPixels` * 3- or 4-bytes,
///         depending on the LED chip type of the strip.
///
/// @param[in] uControllerIndex     Index of LED strip controller. Shall be less
///                                 than `ESP_STRIPPER_MAX_CONTROLLERS`.
/// @param[in] config               Configuration parameters
/// @return                         True if configuration was succesfully applied.
///                                 False when an unexpected failure occurs, and
///                                 in this case, no changes shall have been made to
///                                 any existing configuration on this controller.
bool esp_stripper_config_controller(uint8_t uControllerIndex, esp_stripper_ctrl_config_t* config);


/// @brief  Reset configuration parameters on the LED strip controller given by index.
///
/// @param[in] uControllerIndex     Index of LED strip controller. Shall be less
///                                 than `ESP_STRIPPER_MAX_CONTROLLERS`.
void esp_stripper_deinit_controller(uint8_t uControllerIndex);


/// @brief  Destroy any memory used by the `esp_stripper` component.
void esp_stripper_deinit();


/// @brief  Clears all color data from the pixel buffer of the given LED strip controller.
///
/// @param[in] uControllerIndex     Index of LED strip controller. Shall be less
///                                 than `ESP_STRIPPER_MAX_CONTROLLERS`.
void esp_stripper_clear_pixels(uint8_t uControllerIndex);


/// @brief  Load color data into the pixel buffer of the given LED strip controller.
///
/// @note   The alpha channel of the provided color data is ignored. Assumes
///         that the provided RGB colors have been pre-multiplied by the alpha.
///
/// @param[in] uControllerIndex     Index of LED strip controller. Shall be less
///                                 than `ESP_STRIPPER_MAX_CONTROLLERS`.
/// @param[in] pColorBufferRGBA     An array of 32-bit RGBA color data, where
///                                 each element's lowest byte is red channel,
///                                 followed by green, then blue, and alpha
///                                 channel as the highest byte. Assumes the
///                                 provided buffer is at least `uNumPixels` large.
/// @param[in] uNumPixels           The number of elements to copy into this LED
///                                 strip controller's pixel buffer.
/// @param[in] uPixelOffset         The starting pixel index to copy color data into.
void esp_stripper_load_pixels(uint8_t uControllerIndex, uint32_t* pColorBufferRGBA, uint16_t uNumPixels, uint16_t uPixelOffset);


/// @brief  Begin transmission of all LED strip controllers' data on their
///         configured GPIO output pin. Nominally, up to 4 output signals
///         are transmitted simultaneously; this function will block untill all
///         configured controllers have finished transmitting their data.
/// @return ESP_STRIPPER_ERR_OK         - data transmission completed successfully
///         ESP_STRIPPER_ERR_NOT_INIT   - esp_stripper_init has not been called yet
///         ESP_STRIPPER_ERR_TX_OVERRUN - one or more controllers took longer
///                                       than expected to transmit the data
esp_stripper_err_t esp_stripper_display();

void _esp_stripper_debug_print();

#endif // ESP_STRIPPER_H
