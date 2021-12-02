/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

#define IS_RGBW false
#define NUM_PIXELS 450

#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
// default to pin 2 if the board doesn't have a default WS2812 pin defined
#define WS2812_PIN 13
#endif

/*******************************************************************************
 * Structs and enums
 */

// Pixel format
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} pixel_t;

/*******************************************************************************
 * Globals
 */

// Buffer for holding pixel values
static uint32_t pix_buf[NUM_PIXELS];

/*******************************************************************************
 * WS2812b control functions
 */

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// Get pixel value
static inline pixel_t get_pixel(int32_t pos) {
    
    pixel_t pix = {.r = 0, .g = 0, .b = 0};

    // Check to make sure our index is not out of bounds
    if ((pos >= NUM_PIXELS) || (pos < 0)) {
        return pix;
    }

    // Get pixel values
    pix.r = (uint8_t)(pix_buf[pos] >> 8);
    pix.g = (uint8_t)(pix_buf[pos] >> 16);
    pix.b = (uint8_t)(pix_buf[pos]);

    return pix;
}

// Set pixel value
static inline void set_pixel(int32_t pos, pixel_t pix) {
    
    // Check to make sure our index is not out of bounds
    if ((pos >= NUM_PIXELS) || (pos < 0)) {
        return;
    }

    // Set values
    pix_buf[pos] =  ((uint32_t)(pix.r) << 8) |
                    ((uint32_t)(pix.g) << 16) |
                    ((uint32_t)(pix.b));
}

// Send out pixel buffer
static inline void show() {
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(pix_buf[i]);
    }
}

/*******************************************************************************
 * Main
 */

int main() {

    pixel_t pix;

    //set_sys_clock_48();
    stdio_init_all();
    printf("WS2812 Smoke Test, using pin %d\r\n", WS2812_PIN);

    // todo get free sm
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    int t = 0;
    while (1) {
        for (int i = 0; i < NUM_PIXELS; ++i) {
            pix.r = 50;
            pix.g = 10;
            pix.b = 0;
            set_pixel(i, pix);
        }
        show();
        sleep_ms(1);
    }
}