/* MIT License
 *
 * Copyright (C) 2022 Charlie De Vivero
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
 */

#include "esp_stripper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/rmt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "soc/rtc.h"

#include <string.h>

extern void spi_flash_op_lock(void);
extern void spi_flash_op_unlock(void);


/* -------------------------------------------------------------------------- */
/*                                  Constants                                 */
/* -------------------------------------------------------------------------- */

static const char* TAG = "stpr";

// white channel calibration values for 4500K (neutral white)
static const uint8_t k_uWhiteChannelR = 255;
static const uint8_t k_uWhiteChannelG = 219;
static const uint8_t k_uWhiteChannelB = 186;

// -- Configuration constants
#define DIVIDER                                 4
// #define ENABLE_DEBUG_LOGGING

// Note that the pulse timings are tied to the divider.
// If you change DIVIDER, then change xxx_CYCLES below.
// TODO: do this programmatically

// WS2812B
// - defined as cycles at 20MHz clock (20MHz * 0.4us = 8 cycles)
#define WS2812B_T0H_CYCLES                      8       // 400 ns
#define WS2812B_T0L_CYCLES                      17      // 850 ns
#define WS2812B_T1H_CYCLES                      16      // 800 ns
#define WS2812B_T1L_CYCLES                      9       // 450 ns
// Number of bytes per pixel: 3, channel transmission order: GRB
#define WS2812B_BYTES_PER_PIXEL                 3

// SK6812RGBW
// - defined as cycles at 20MHz clock (20MHz * 0.3us = 6 cycles)
#define SK6812RGBW_T0H_CYCLES                   6       // 300 ns
#define SK6812RGBW_T0L_CYCLES                   18      // 900 ns
#define SK6812RGBW_T1H_CYCLES                   12      // 600 ns
#define SK6812RGBW_T1L_CYCLES                   12      // 600 ns
// Number of bytes per pixel: 4, channel transmission order: GRBW
#define SK6812RGBW_BYTES_PER_PIXEL              4

// RMT clock cycles per RMT pulse
// TODO: calculate num cycles for each type of LED. For now this is
// an estimation used for all LED types.
#define RMT_CYCLES_PER_BIT                      (WS2812B_T0H_CYCLES + WS2812B_T0L_CYCLES)

// -- RMT memory configuration
//    By default we use two memory blocks for each RMT channel instead of 1. The
//    reason is that one memory block is only 64 signal bits, which causes the refill
//    interrupt to fire too often. When combined with WiFi, this leads to conflicts
//    between interrupts and weird flashy effects on the LEDs. Special thanks to
//    Brian Bulkowski for finding this problem and developing a fix.
#define FASTLED_RMT_MEM_BLOCKS                  2

// 64 for ESP32, ESP32S2
// 48 for ESP32S3, ESP32C3, ESP32H2
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#define RMT_MEM_WORDS_PER_CHANNEL               SOC_RMT_MEM_WORDS_PER_CHANNEL
#else
// ESP32 value (only chip variant supported on older IDF)
#define RMT_MEM_WORDS_PER_CHANNEL               64
#endif

#define MAX_PULSES                              (RMT_MEM_WORDS_PER_CHANNEL * FASTLED_RMT_MEM_BLOCKS)
#define PULSES_PER_FILL                         (MAX_PULSES >> 1) // Fill half of the channel buffer at a time
#define BYTES_PER_FILL                          (PULSES_PER_FILL >> 3) // PULSES_PER_FILL in terms of bytes (divide by 8)


/* -------------------------------------------------------------------------- */
/*                                    Types                                   */
/* -------------------------------------------------------------------------- */

// Define the internal state of individual LED strip controllers.
// We'll store a collection of these structs in DRAM, including pixel buffers
// so they can be accessed from the ISR.
typedef struct
{
    // user-configurable fields
    uint8_t                     uChipType;
    uint8_t                     uGpioNum;
    uint16_t                    uNumPixels;
    uint8_t*                    pPixelBuffer;
    uint16_t                    uPixelBufferSize;

    // internal controller state
    uint16_t                    uPixelBufferIdx;
    uint8_t                     RmtChannelIndex;
    uint8_t                     uWhichHalf;
    uint32_t                    uExceededTime;
    uint32_t                    uLastFillTimeClk;
    volatile uint32_t*          pRmtMemoryStart; // volatile because points to register
    volatile uint32_t*          pRmtMemoryPointer; // volatile because points to register
} esp_stripper_controller_t;


// Store the general internal state in DRAM
typedef struct
{
    esp_stripper_controller_t   Controllers[ESP_STRIPPER_MAX_CONTROLLERS];
    esp_stripper_controller_t*  ControllersByRmtChannel[(size_t)RMT_CHANNEL_MAX];
    uint8_t                     uNumConfiguredControllers;
    uint8_t                     uNumControllersDone;
    uint8_t                     uNextControllerIndexToStart;
    uint32_t                    uMaxCyclesPerFill;
} esp_stripper_state_t;


/* -------------------------------------------------------------------------- */
/*                              Static Variables                              */
/* -------------------------------------------------------------------------- */

static bool                         s_bIsInitialized = false;
static intr_handle_t                s_pRmtInterruptHandle = NULL;
static esp_stripper_state_t*        s_InternalState = NULL;

// TODO: Consider a design where no semaphore is used, instead it's more of
//       a fire-and-forget design. But how to protect user from calling
//       the display function too early?
//
// -- Global semaphore for the whole show process
//    Semaphore is not given until all data has been sent
static xSemaphoreHandle             s_SemaphoreRmtTx = NULL;


/* -------------------------------------------------------------------------- */
/*                            Forward Declarations                            */
/* -------------------------------------------------------------------------- */

static size_t calculate_pixel_buffer_size(uint16_t uNumPixels, esp_stripper_chip_type_t chipType);
static void set_pixel_buffer_color(esp_stripper_controller_t* pController, uint16_t uPixelIdx, uint32_t colorRGBA);

static void IRAM_ATTR rmt_interrupt_handler(void* arg);
static void IRAM_ATTR rmt_channel_done(rmt_channel_t channel, void * arg); // FastLED: doneOnChannel

static void IRAM_ATTR start_next_controller_on_rmt_channel(uint8_t uRmtChannelIndex); // FastLED: startNext
static void IRAM_ATTR start_controller_on_rmt_channel(esp_stripper_controller_t* pController, uint8_t uRmtChannelIndex); // FastLED: startOnChannel
static void IRAM_ATTR start_controller_tx(esp_stripper_controller_t* pController); // FastLED: tx_start

static void IRAM_ATTR fill_next(esp_stripper_controller_t* pController, bool bCheckTime); // FastLED: fillNext
static void IRAM_ATTR fill_next_WS2812B(esp_stripper_controller_t* pController);
static void IRAM_ATTR fill_next_SK6812RGBW(esp_stripper_controller_t* pController);


/* -------------------------------------------------------------------------- */
/*                              Inline Functions                              */
/* -------------------------------------------------------------------------- */

/// @brief                  Scale one byte by a second one, which is treated as
///                         the numerator of a fraction whose denominator is 256.
/// @param[in]  n           The byte to scale, [0-255]
/// @param[in]  frac        The numerator of scaling fraction [0-255], assuming denominator is 256.
/// @return                 Computes: n * ((frac + 1) / 256);
static inline uint8_t scale8(uint8_t n, uint8_t frac)
{
    return (uint8_t)(((uint16_t)n * (1 + (uint16_t)frac)) >> 8);
}

/// @brief                  Convert RGBA color packed into 32-bits, into 4-channel LED color (red,
///                         green, blue, white) for LED strips that have an extra white LED color.
///                         The input alpha channel is not used; assume that the RGB input has been
///                         pre-multiplied. Assumes white LED color temperature is 4500K.
///                         Reference: https://stackoverflow.com/questions/40312216/converting-rgb-to-rgbw
/// @param[in]  colorRGBA   Input RGBA color packed into 32-bits (8-bits per channel)
/// @param[out] r           The red LED output color
/// @param[out] g           The green LED output color
/// @param[out] b           The blue LED output color
/// @param[out] w           The white (4500K) LED output color
static inline void rgb32_to_rgbw(uint32_t colorRGBA, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* w)
{
    *r = (uint8_t)(colorRGBA & 0xFF);
    *g = (uint8_t)((colorRGBA >> 8) & 0xFF);
    *b = (uint8_t)((colorRGBA >> 16) & 0xFF);

    // white channel compensation
    uint8_t wR = scale8(*r, k_uWhiteChannelR);
    uint8_t wG = scale8(*g, k_uWhiteChannelG);
    uint8_t wB = scale8(*b, k_uWhiteChannelB);

    // white channel equals the lowest of the scaled colors
    *w = (wG < wB) ? wG : wB;
    *w = (*w < wR) ? *w : wR;

    // subtract white channel contribution from each color
    *r = (uint8_t)(*r - scale8(*w, k_uWhiteChannelR));
    *g = (uint8_t)(*g - scale8(*w, k_uWhiteChannelG));
    *b = (uint8_t)(*b - scale8(*w, k_uWhiteChannelB));
}


/* -------------------------------------------------------------------------- */
/*                          Public Interface Methods                          */
/* -------------------------------------------------------------------------- */

void esp_stripper_init()
{
    // TODO: improve error checking, return success value

    if (s_bIsInitialized)
    {
        ESP_LOGW(TAG, "Already initialized");
        return;
    }

    // Initialize the stored internal state
    s_InternalState = (esp_stripper_state_t *)calloc(1, sizeof(esp_stripper_state_t));

    // Initialize the RMT subsystem
    rmt_config_t rmt_tx;
    uint8_t uRmtChIdx = (uint8_t)RMT_CHANNEL_0;
    while (uRmtChIdx < (uint8_t)RMT_CHANNEL_MAX)
    {
        // -- RMT configuration for transmission
        memset(&rmt_tx, 0, sizeof(rmt_config_t));
        rmt_tx.channel = (rmt_channel_t)uRmtChIdx;
        rmt_tx.rmt_mode = RMT_MODE_TX;
        rmt_tx.gpio_num = GPIO_NUM_0; // doesn't matter here, will be changed dynamically
        rmt_tx.mem_block_num = FASTLED_RMT_MEM_BLOCKS;
        rmt_tx.clk_div = DIVIDER;
        rmt_tx.tx_config.loop_en = false;
        rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
        rmt_tx.tx_config.carrier_en = false;
        rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        rmt_tx.tx_config.idle_output_en = true;

        // -- Apply the configuration
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_config(&rmt_tx));

        // -- Set up the RMT to send 64 bits of the pulse buffer and then
        //    generate an interrupt. When we get this interrupt we
        //    fill the other part in preparation (like double-buffering)
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_set_tx_thr_intr_en((rmt_channel_t)uRmtChIdx, true, PULSES_PER_FILL));

        uRmtChIdx += FASTLED_RMT_MEM_BLOCKS;
    }

    // -- Create a semaphore to block execution until all the controllers are done
    if (s_SemaphoreRmtTx == NULL) {
        s_SemaphoreRmtTx = xSemaphoreCreateBinary();
        xSemaphoreGive(s_SemaphoreRmtTx); // semaphore is init as "empty", so it must be given first, before it can be taken
    }

    // -- Allocate the interrupt if we have not done so yet. This
    //    interrupt handler must work for all different kinds of
    //    strips, so it must delegate to a specific refill function
    //    for each type of LED strip.
    if (s_pRmtInterruptHandle == NULL)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_intr_alloc(
            ETS_RMT_INTR_SOURCE,
            ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
            rmt_interrupt_handler, 0, &s_pRmtInterruptHandle));
    }

    // -- Expected number of CPU cycles between buffer fills
    // -- If there is ever an interval greater than 1.5 times
    //    the expected time, then bail out.
    rtc_cpu_freq_config_t rtcCfg;
    rtc_clk_cpu_freq_get_config(&rtcCfg);
    uint32_t uCyclesPerFill = rtcCfg.freq_mhz * 1000000 / APB_CLK_FREQ * DIVIDER * RMT_CYCLES_PER_BIT * PULSES_PER_FILL;
    s_InternalState->uMaxCyclesPerFill = uCyclesPerFill + (uCyclesPerFill >> 1);

    // Done with all setup
    s_bIsInitialized = true;
}

bool esp_stripper_config_controller(uint8_t uControllerIndex, esp_stripper_ctrl_config_t* config)
{
    bool bSuccess = false;
    int ret = 0; (void)ret;
    ESP_GOTO_ON_FALSE(s_bIsInitialized, -1, Exit, TAG,
        "esp_stripper is not initialized");
    ESP_GOTO_ON_FALSE(uControllerIndex < ESP_STRIPPER_MAX_CONTROLLERS, -2, Exit, TAG,
        "uControllerIndex must be less than %u", ESP_STRIPPER_MAX_CONTROLLERS);
    ESP_GOTO_ON_FALSE(config != NULL, -3, Exit, TAG,
        "config cannot be NULL");

    // check the buffer size that is needed for this strip
    size_t uPixelBufferSizeNeeded = calculate_pixel_buffer_size(config->uNumPixels, config->ChipType);
    if (uPixelBufferSizeNeeded != s_InternalState->Controllers[uControllerIndex].uPixelBufferSize)
    {
        // new size needed is different than existing buffer
        if (uPixelBufferSizeNeeded > 0)
        {
            // allocate new buffer
            uint8_t* pNewBuf = (uint8_t *)malloc(uPixelBufferSizeNeeded);
            ESP_GOTO_ON_FALSE(pNewBuf != NULL, -4, Exit, TAG,
                "Failed to allocate %u bytes", uPixelBufferSizeNeeded);

            free(s_InternalState->Controllers[uControllerIndex].pPixelBuffer);
            s_InternalState->Controllers[uControllerIndex].pPixelBuffer = pNewBuf;
        }
        else
        {
            // new size is zero, just free the old buffer
            free(s_InternalState->Controllers[uControllerIndex].pPixelBuffer);
            s_InternalState->Controllers[uControllerIndex].pPixelBuffer = NULL;
        }
    }

    // update rest of fields
    s_InternalState->Controllers[uControllerIndex].uGpioNum = config->uGpioNum;
    s_InternalState->Controllers[uControllerIndex].uChipType = (uint8_t)config->ChipType;
    s_InternalState->Controllers[uControllerIndex].uNumPixels = config->uNumPixels;
    s_InternalState->Controllers[uControllerIndex].uPixelBufferSize = uPixelBufferSizeNeeded;

    bSuccess = true;

Exit:
    return bSuccess;
}

void esp_stripper_deinit_controller(uint8_t uControllerIndex)
{
    free(s_InternalState->Controllers[uControllerIndex].pPixelBuffer);
    memset(&s_InternalState->Controllers[uControllerIndex], 0, sizeof(esp_stripper_controller_t));
}

void esp_stripper_deinit()
{
    for (uint8_t i = 0; i < ESP_STRIPPER_MAX_CONTROLLERS; ++i)
    {
        esp_stripper_deinit_controller(i);
    }
    free(s_InternalState);
    s_InternalState = NULL;
    vSemaphoreDelete(s_SemaphoreRmtTx);
}

void esp_stripper_clear_pixels(uint8_t uControllerIndex)
{
    int ret = 0; (void)ret;
    ESP_GOTO_ON_FALSE(s_bIsInitialized, -1, Exit, TAG,
        "esp_stripper is not initialized");
    ESP_GOTO_ON_FALSE(uControllerIndex < ESP_STRIPPER_MAX_CONTROLLERS, -2, Exit, TAG,
        "uControllerIndex must be less than %u", ESP_STRIPPER_MAX_CONTROLLERS);

    if (s_InternalState->Controllers[uControllerIndex].uPixelBufferSize > 0)
    {
        memset(s_InternalState->Controllers[uControllerIndex].pPixelBuffer, 0, s_InternalState->Controllers[uControllerIndex].uPixelBufferSize);
    }

Exit:
    return;
}

void esp_stripper_load_pixels(uint8_t uControllerIndex, uint32_t* pColorBufferRGBA, uint16_t uNumPixels, uint16_t uPixelOffset)
{
    // check the arguments first
    int ret = 0; (void)ret;
    ESP_GOTO_ON_FALSE(s_bIsInitialized, -1, Exit, TAG,
        "esp_stripper is not initialized");
    ESP_GOTO_ON_FALSE(uControllerIndex < ESP_STRIPPER_MAX_CONTROLLERS, -2, Exit, TAG,
        "uControllerIndex must be less than %u", ESP_STRIPPER_MAX_CONTROLLERS);

    if (uNumPixels > 0)
    {
        // error if num pixels is greater than zero, but the given buffer is null
        ESP_GOTO_ON_FALSE(pColorBufferRGBA != NULL, -3, Exit, TAG,
            "pColorBufferRGBA cannot be null if uNumPixels > 0, uNumPixels=%u", uNumPixels);
    }

    // fail silently in these cases
    if (uNumPixels <= 0 || uPixelOffset >= s_InternalState->Controllers[uControllerIndex].uNumPixels)
    {
        return;
    }

    // calculate how many pixels to fill
    uint16_t uEndPixel = uPixelOffset + uNumPixels;
    if (uEndPixel > s_InternalState->Controllers[uControllerIndex].uNumPixels)
    {
        uEndPixel = s_InternalState->Controllers[uControllerIndex].uNumPixels;
    }

    // copy pixel data into the controller's buffer in the appropriate color order
    uint16_t uColorIdx = 0;
    for (uint16_t i = uPixelOffset; i < uEndPixel; ++i)
    {
        set_pixel_buffer_color(&s_InternalState->Controllers[uControllerIndex], i, pColorBufferRGBA[uColorIdx++]);
    }

Exit:
    return;
}

void esp_stripper_display()
{
    int ret = 0; (void)ret;
    ESP_GOTO_ON_FALSE(s_bIsInitialized, -1, Exit, TAG,
        "esp_stripper is not initialized");

    // How many controllers need to display?
    for (uint8_t i = 0; i < ESP_STRIPPER_MAX_CONTROLLERS; ++i)
    {
        if (s_InternalState->Controllers[i].uNumPixels > 0)
        {
            s_InternalState->uNumConfiguredControllers++;
        }
    }

    if (s_InternalState->uNumConfiguredControllers <= 0)
    {
        // Nothing ready to display, return here
        return;
    }

    // -- Make sure no flash operations happen right now
    spi_flash_op_lock();

    xSemaphoreTake(s_SemaphoreRmtTx, portMAX_DELAY);

    // -- First, fill all the available channels
    uint8_t uRmtChIdx = (uint8_t)RMT_CHANNEL_0;
    while (uRmtChIdx < (uint8_t)RMT_CHANNEL_MAX) {
        start_next_controller_on_rmt_channel(uRmtChIdx);
        // -- Important: when we use more than one memory block, we need to
        //    skip the channels that would otherwise overlap in memory.
        uRmtChIdx += FASTLED_RMT_MEM_BLOCKS;
    }

    // -- Wait here while the data is sent. The interrupt handler
    //    will keep refilling the RMT buffers until it is all
    //    done; then it gives the semaphore back.
    xSemaphoreTake(s_SemaphoreRmtTx, portMAX_DELAY);
    xSemaphoreGive(s_SemaphoreRmtTx);

    // -- Reset the counters
    s_InternalState->uNumControllersDone = 0;
    s_InternalState->uNextControllerIndexToStart = 0;
    s_InternalState->uNumConfiguredControllers = 0;

    spi_flash_op_unlock();

    // check if there were any transmit time overruns
    for (size_t i = 0; i < ESP_STRIPPER_MAX_CONTROLLERS; ++i)
    {
        if (s_InternalState->Controllers[i].uExceededTime > 0)
        {
            ESP_LOGW(TAG, "Controller %02u had tx time overrun: %u clk (max: %u)",
                i, s_InternalState->Controllers[i].uExceededTime, s_InternalState->uMaxCyclesPerFill);
        }
    }

 Exit:
     return;
}

#ifdef ENABLE_DEBUG_LOGGING
void _esp_stripper_debug_print()
{
    if (!s_bIsInitialized)
    {
        ESP_LOGI(TAG, "esp_stripper not initialized");
        return;
    }

    for (size_t i = 0; i < ESP_STRIPPER_MAX_CONTROLLERS; ++i)
    {
        ESP_LOGI(TAG, "* Ctrl%02u: GPIO=%u Chip=%u NumPx=%u PxBuf=%p PxBufSz=%u",
            i, s_InternalState->Controllers[i].uGpioNum, s_InternalState->Controllers[i].uChipType,
            s_InternalState->Controllers[i].uNumPixels, s_InternalState->Controllers[i].pPixelBuffer,
            s_InternalState->Controllers[i].uPixelBufferSize);
    }
}
#endif


/* -------------------------------------------------------------------------- */
/*                               Private Methods                              */
/* -------------------------------------------------------------------------- */

static size_t calculate_pixel_buffer_size(uint16_t uNumPixels, esp_stripper_chip_type_t chipType)
{
    size_t uSizeNeeded = 0;
    if (chipType == ESP_STRIPPER_CHIP_WS2812B)
    {
        uSizeNeeded = uNumPixels * WS2812B_BYTES_PER_PIXEL;
    }
    else if (chipType == ESP_STRIPPER_CHIP_SK6812RGBW)
    {
        uSizeNeeded = uNumPixels * SK6812RGBW_BYTES_PER_PIXEL;
    }
    else
    {
        ESP_LOGW(TAG, "Invalid chip type '%u', must be one of esp_stripper_chip_type_t enum values", (uint32_t)chipType);
    }

    return uSizeNeeded;
}

static void set_pixel_buffer_color(esp_stripper_controller_t* pController, uint16_t uPixelIdx, uint32_t colorRGBA)
{
    size_t uBufIdx = uPixelIdx;
    if (pController->uChipType == ESP_STRIPPER_CHIP_WS2812B)
    {
        uBufIdx *= WS2812B_BYTES_PER_PIXEL;
        pController->pPixelBuffer[uBufIdx] = (uint8_t)((colorRGBA >> 8) & 0xFF);
        pController->pPixelBuffer[uBufIdx + 1] = (uint8_t)(colorRGBA & 0xFF);
        pController->pPixelBuffer[uBufIdx + 2] = (uint8_t)((colorRGBA >> 16) & 0xFF);
    }
    else if (pController->uChipType == ESP_STRIPPER_CHIP_SK6812RGBW)
    {
        uBufIdx *= SK6812RGBW_BYTES_PER_PIXEL;

        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        uint8_t w = 0;
        rgb32_to_rgbw(colorRGBA, &r, &g, &b, &w);

        pController->pPixelBuffer[uBufIdx] = g;
        pController->pPixelBuffer[uBufIdx + 1] = r;
        pController->pPixelBuffer[uBufIdx + 2] = b;
        pController->pPixelBuffer[uBufIdx + 3] = w;
    }
}

// -- Custom interrupt handler
//    This interrupt handler handles two cases: a controller is
//    done writing its data, or a controller needs to fill the
//    next half of the RMT buffer with data.
static void IRAM_ATTR rmt_interrupt_handler(void* arg)
{
    // -- The basic structure of this code is borrowed from the
    //    interrupt handler in esp-idf/components/driver/rmt.c
    volatile uint32_t intr_st = RMT.int_st.val;
    uint8_t uRmtChannelIdx;

    for (uRmtChannelIdx = 0; uRmtChannelIdx < (uint8_t)RMT_CHANNEL_MAX; ++uRmtChannelIdx) {
        #if CONFIG_IDF_TARGET_ESP32S2
        int tx_done_bit = uRmtChannelIdx * 3;
        int tx_next_bit = uRmtChannelIdx + 12;
        #elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
        int tx_done_bit = uRmtChannelIdx;
        int tx_next_bit = uRmtChannelIdx + 8;
        #elif CONFIG_IDF_TARGET_ESP32
        int tx_done_bit = uRmtChannelIdx * 3; // ch#_tx_end
        int tx_next_bit = uRmtChannelIdx + 24; // ch#_tx_thr_event
        #else
        #error Not yet implemented for unknown ESP32 target
        #endif

        esp_stripper_controller_t* pController = s_InternalState->ControllersByRmtChannel[uRmtChannelIdx];
        if (intr_st & BIT(tx_next_bit))
        {
            // -- More to send on this channel
            fill_next(pController, true);
            RMT.int_clr.val |= BIT(tx_next_bit);
        }
        else
        {
            // -- Transmission is complete on this channel
            if (intr_st & BIT(tx_done_bit))
            {
                RMT.int_clr.val |= BIT(tx_done_bit);
                rmt_channel_done((rmt_channel_t)uRmtChannelIdx, NULL);
            }
        }
    }
}

// -- Start up the next controller
//    This method is static so that it can dispatch to the
//    appropriate start_controller_on_rmt_channel method of the given controller.
static void IRAM_ATTR start_next_controller_on_rmt_channel(uint8_t uRmtChannelIndex)
{
    // Find the next controller to display. It just needs to have uNumPixels greater than zero.
    while (s_InternalState->uNextControllerIndexToStart < ESP_STRIPPER_MAX_CONTROLLERS)
    {
        if (s_InternalState->Controllers[s_InternalState->uNextControllerIndexToStart].uNumPixels > 0)
        {
            // Found a controller to display, start it on this RMT channel and break out
            start_controller_on_rmt_channel(
                &s_InternalState->Controllers[s_InternalState->uNextControllerIndexToStart],
                uRmtChannelIndex);
            s_InternalState->uNextControllerIndexToStart++;
            break;
        }
        s_InternalState->uNextControllerIndexToStart++;
    }
}

// -- Start this controller on the given channel
//    This function just initiates the RMT write; it does not wait
//    for it to finish.
static void IRAM_ATTR start_controller_on_rmt_channel(esp_stripper_controller_t* pController, uint8_t uRmtChannelIndex)
{
    // -- Assign this channel and configure the RMT
    pController->RmtChannelIndex = (rmt_channel_t)uRmtChannelIndex;

    // -- Store a reference to this controller, so we can get it
    //    inside the interrupt handler
    s_InternalState->ControllersByRmtChannel[uRmtChannelIndex] = pController;

    // -- Assign the pin to this channel
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_set_gpio(pController->RmtChannelIndex, RMT_MODE_TX, pController->uGpioNum, false));
#else
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_set_pin(pController->RmtChannelIndex, RMT_MODE_TX, pController->uGpioNum));
#endif

    // -- Use our custom driver to send the data incrementally

    // -- Initialize the counters that keep track of where we are in
    //    the pixel data and the RMT buffer
    pController->pRmtMemoryStart = &(RMTMEM.chan[pController->RmtChannelIndex].data32[0].val);
    pController->pRmtMemoryPointer = pController->pRmtMemoryStart;
    pController->uPixelBufferIdx = 0;
    pController->uWhichHalf = 0;
    pController->uLastFillTimeClk = 0;
    pController->uExceededTime = 0;

    // -- Fill both halves of the RMT buffer (a total of 128 bits of pixel data)
    fill_next(pController, false);
    fill_next(pController, false);

    // -- Turn on the interrupts
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_set_tx_intr_en(pController->RmtChannelIndex, true));

    // -- Kick off the transmission
    start_controller_tx(pController);
}

// -- Fill RMT buffer
//    Puts 64 bits of pixel data into the next 64 slots in the RMT memory.
//    Each data bit is represented by a 32-bit RMT item that specifies how
//    long to hold the signal high, followed by how long to hold it low.
static void IRAM_ATTR fill_next(esp_stripper_controller_t* pController, bool bCheckTime)
{
    // Keep track of time elapsed
    uint32_t uCurrentTimeClk = xthal_get_ccount();
    if (bCheckTime)
    {
        uint32_t uTimeSinceLastFillClk = 0;
        if (pController->uLastFillTimeClk > uCurrentTimeClk)
        {
            // account for the CPU clock count overflow
            uTimeSinceLastFillClk = 0xFFFFFFFF - pController->uLastFillTimeClk + uCurrentTimeClk;
        }
        else
        {
            uTimeSinceLastFillClk = uCurrentTimeClk - pController->uLastFillTimeClk;
        }

        // check if we exceeded time limits
        if (uTimeSinceLastFillClk > s_InternalState->uMaxCyclesPerFill)
        {
            // record the timing
            pController->uExceededTime = uTimeSinceLastFillClk;

            // stop filling up new data
            pController->uPixelBufferIdx = pController->uPixelBufferSize;
        }
    }
    pController->uLastFillTimeClk = uCurrentTimeClk;

    // Pick an appropriate fill function for the LED type
    if (pController->uChipType == ESP_STRIPPER_CHIP_SK6812RGBW)
    {
        fill_next_SK6812RGBW(pController);
    }
    else
    {
        fill_next_WS2812B(pController);
    }
}

static void IRAM_ATTR fill_next_WS2812B(esp_stripper_controller_t* pController)
{
    // -- Get the zero and one values into local variables
    const static DRAM_ATTR rmt_item32_t bit0 = {{{ WS2812B_T0H_CYCLES, 1, WS2812B_T0L_CYCLES, 0 }}};
    const static DRAM_ATTR rmt_item32_t bit1 = {{{ WS2812B_T1H_CYCLES, 1, WS2812B_T1L_CYCLES, 0 }}};

    // -- Use locals for speed
    volatile uint32_t* pItem = pController->pRmtMemoryPointer;

    for (int i = 0; i < BYTES_PER_FILL; ++i) {
        if (pController->uPixelBufferIdx < pController->uPixelBufferSize)
        {
            // -- Get the next byte of pixel data
            uint32_t pixeldata = pController->pPixelBuffer[pController->uPixelBufferIdx] << 24;
            pController->uPixelBufferIdx++;

            // Shift bits out, MSB first, setting RMTMEM.chan[n].data32[x] to the
            // rmt_item32_t value corresponding to the buffered bit value
            for (uint32_t j = 0; j < 8; j++) {
                *pItem++ = (pixeldata & 0x80000000L) ? bit1.val : bit0.val;
                // Replaces: RMTMEM.chan[mRMT_channel].data32[mCurPulse].val = val;

                pixeldata <<= 1;
            }
        } else {
            // -- No more data; signal to the RMT we are done by filling the
            //    rest of the buffer with zeros
            *pItem++ = 0;
        }
    }

    // -- Flip to the other half, resetting the pointer if necessary
    pController->uWhichHalf++;
    if (pController->uWhichHalf >= 2) {
        pItem = pController->pRmtMemoryStart;
        pController->uWhichHalf = 0;
    }

    // -- Store the new pointer back into the object
    pController->pRmtMemoryPointer = pItem;
}

static void IRAM_ATTR fill_next_SK6812RGBW(esp_stripper_controller_t* pController)
{
    // -- Get the zero and one values into local variables
    const static DRAM_ATTR rmt_item32_t bit0 = {{{ SK6812RGBW_T0H_CYCLES, 1, SK6812RGBW_T0L_CYCLES, 0 }}};
    const static DRAM_ATTR rmt_item32_t bit1 = {{{ SK6812RGBW_T1H_CYCLES, 1, SK6812RGBW_T1L_CYCLES, 0 }}};

    // -- Use locals for speed
    volatile uint32_t* pItem = pController->pRmtMemoryPointer;

    for (int i = 0; i < BYTES_PER_FILL; ++i) {
        if (pController->uPixelBufferIdx < pController->uPixelBufferSize) {

            // -- Get the next byte of pixel data
            uint32_t pixeldata = pController->pPixelBuffer[pController->uPixelBufferIdx] << 24;
            pController->uPixelBufferIdx++;
            
            // Shift bits out, MSB first, setting RMTMEM.chan[n].data32[x] to the
            // rmt_item32_t value corresponding to the buffered bit value
            for (uint32_t j = 0; j < 8; j++) {
                *pItem++ = (pixeldata & 0x80000000L) ? bit1.val : bit0.val;
                // Replaces: RMTMEM.chan[mRMT_channel].data32[mCurPulse].val = val;

                pixeldata <<= 1;
            }
        } else {
            // -- No more data; signal to the RMT we are done by filling the
            //    rest of the buffer with zeros
            *pItem++ = 0;
        }
    }

    // -- Flip to the other half, resetting the pointer if necessary
    pController->uWhichHalf++;
    if (pController->uWhichHalf >= 2) {
        pItem = pController->pRmtMemoryStart;
        pController->uWhichHalf = 0;
    }

    // -- Store the new pointer back into the object
    pController->pRmtMemoryPointer = pItem;
}

// -- A controller is done
//    This function is called when a controller finishes writing
//    its data. It is called by the custom interrupt handler.
//    It is static because we don't know which
//    controller is done until we look it up.
static void IRAM_ATTR rmt_channel_done(rmt_channel_t channel, void * arg)
{
    // Look up the controller that was placed on this RMT channel
    esp_stripper_controller_t* pController = s_InternalState->ControllersByRmtChannel[(uint8_t)channel];

    // -- Turn off output on the pin
    // SZG: Do I really need to do this?
    gpio_matrix_out(pController->uGpioNum, 0x100, 0, 0);

    // -- Turn off the interrupts
    // rmt_set_tx_intr_en(channel, false);

    // Inline the code for rmt_set_tx_intr_en(channel, false) and rmt_tx_stop, so it can be placed in IRAM
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
    // rmt_ll_enable_tx_end_interrupt(&RMT, channel)
    RMT.int_ena.val &= ~(1 << channel);
    // rmt_ll_tx_stop(&RMT, channel)
    RMT.tx_conf[channel].tx_stop = 1;
    RMT.tx_conf[channel].conf_update = 1;
    // rmt_ll_tx_reset_pointer(&RMT, channel)
    RMT.tx_conf[channel].mem_rd_rst = 1;
    RMT.tx_conf[channel].mem_rd_rst = 0;
    RMT.tx_conf[channel].mem_rst = 1;
    RMT.tx_conf[channel].mem_rst = 0;
#elif CONFIG_IDF_TARGET_ESP32S3
    // rmt_ll_enable_tx_end_interrupt(&RMT, channel)
    RMT.int_ena.val &= ~(1 << channel);
    // rmt_ll_tx_stop(&RMT, channel)
    RMT.chnconf0[channel].tx_stop_n = 1;
    RMT.chnconf0[channel].conf_update_n = 1;
    // rmt_ll_tx_reset_pointer(&RMT, channel)
    RMT.chnconf0[channel].mem_rd_rst_n = 1;
    RMT.chnconf0[channel].mem_rd_rst_n = 0;
    RMT.chnconf0[channel].apb_mem_rst_n = 1;
    RMT.chnconf0[channel].apb_mem_rst_n = 0;
#elif CONFIG_IDF_TARGET_ESP32S2
    // rmt_ll_enable_tx_end_interrupt(&RMT, channel)
    RMT.int_ena.val &= ~(1 << (channel * 3));
    // rmt_ll_tx_stop(&RMT, channel)
    RMT.conf_ch[channel].conf1.tx_stop = 1;
    // rmt_ll_tx_reset_pointer(&RMT, channel)
    RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[channel].conf1.mem_rd_rst = 0;
#elif CONFIG_IDF_TARGET_ESP32
    // rmt_ll_enable_tx_end_interrupt(&RMT, channel)
    RMT.int_ena.val &= ~(1 << (channel * 3));
    // rmt_ll_tx_stop(&RMT, channel)
    RMT.conf_ch[channel].conf1.tx_start = 0;
    // rmt_ll_tx_reset_pointer(&RMT, channel)
    RMT.conf_ch[channel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[channel].conf1.mem_rd_rst = 0;
#else
    #error Not yet implemented for unknown ESP32 target
#endif

    // done with this controller on this specific channel, remove the reference
    s_InternalState->ControllersByRmtChannel[(uint8_t)channel] = NULL;
    s_InternalState->uNumControllersDone++;

    if (s_InternalState->uNumControllersDone == s_InternalState->uNumConfiguredControllers)
    {
        // -- If this is the last controller, signal that we are all done
        portBASE_TYPE bIsHiPriTaskAwoken = 0;
        xSemaphoreGiveFromISR(s_SemaphoreRmtTx, &bIsHiPriTaskAwoken);
        if (bIsHiPriTaskAwoken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        // -- Otherwise, if there are still controllers waiting, then
        //    start the next one on this channel
        start_next_controller_on_rmt_channel(channel);
    }
}

// -- Start RMT transmission
//    Setting this RMT flag is what actually kicks off the peripheral
static void IRAM_ATTR start_controller_tx(esp_stripper_controller_t* pController)
{
    // Inline the code for rmt_tx_start, so it can be placed in IRAM:
    // rmt_tx_start(pController->RmtChannelIndex, true);
    //
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
    // rmt_ll_tx_reset_pointer(&RMT, pController->RmtChannelIndex)
    RMT.tx_conf[pController->RmtChannelIndex].mem_rd_rst = 1;
    RMT.tx_conf[pController->RmtChannelIndex].mem_rd_rst = 0;
    RMT.tx_conf[pController->RmtChannelIndex].mem_rst = 1;
    RMT.tx_conf[pController->RmtChannelIndex].mem_rst = 0;
    // rmt_ll_clear_tx_end_interrupt(&RMT, pController->RmtChannelIndex)
    RMT.int_clr.val = (1 << (pController->RmtChannelIndex));
    // rmt_ll_enable_tx_end_interrupt(&RMT, pController->RmtChannelIndex, true)
    RMT.int_ena.val |= (1 << pController->RmtChannelIndex);
    // rmt_ll_tx_start(&RMT, pController->RmtChannelIndex)
    RMT.tx_conf[pController->RmtChannelIndex].conf_update = 1;
    RMT.tx_conf[pController->RmtChannelIndex].tx_start = 1;
#elif CONFIG_IDF_TARGET_ESP32S3
    // rmt_ll_tx_reset_pointer(&RMT, pController->RmtChannelIndex)
    RMT.chnconf0[pController->RmtChannelIndex].mem_rd_rst_n = 1;
    RMT.chnconf0[pController->RmtChannelIndex].mem_rd_rst_n = 0;
    RMT.chnconf0[pController->RmtChannelIndex].apb_mem_rst_n = 1;
    RMT.chnconf0[pController->RmtChannelIndex].apb_mem_rst_n = 0;
    // rmt_ll_clear_tx_end_interrupt(&RMT, pController->RmtChannelIndex)
    RMT.int_clr.val = (1 << (pController->RmtChannelIndex));
    // rmt_ll_enable_tx_end_interrupt(&RMT, pController->RmtChannelIndex, true)
    RMT.int_ena.val |= (1 << pController->RmtChannelIndex);
    // rmt_ll_tx_start(&RMT, pController->RmtChannelIndex)
    RMT.chnconf0[pController->RmtChannelIndex].conf_update_n = 1;
    RMT.chnconf0[pController->RmtChannelIndex].tx_start_n = 1;
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32
    // rmt_ll_tx_reset_pointer(&RMT, 0)
    RMT.conf_ch[pController->RmtChannelIndex].conf1.mem_rd_rst = 1;
    RMT.conf_ch[pController->RmtChannelIndex].conf1.mem_rd_rst = 0;
    // rmt_ll_clear_tx_end_interrupt(&RMT, 0)
    RMT.int_clr.val = (1 << (pController->RmtChannelIndex * 3));
    // rmt_ll_enable_tx_end_interrupt(&RMT, 0, true)
    RMT.int_ena.val &= ~(1 << (pController->RmtChannelIndex * 3));
    RMT.int_ena.val |= (1 << (pController->RmtChannelIndex * 3));
    // rmt_ll_tx_start(&RMT, 0)
    RMT.conf_ch[pController->RmtChannelIndex].conf1.tx_start = 1;
#else
    #error Not yet implemented for unknown ESP32 target
#endif
}
