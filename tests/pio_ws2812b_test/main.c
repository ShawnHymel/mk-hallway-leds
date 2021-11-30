/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Original: https://github.com/raspberrypi/pico-examples/tree/master/pio/ws2812
 * 
 * Modified by: Shawn Hymel
 * Date: November 23, 2021
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/sem.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "ws2812_parallel.pio.h"

/*******************************************************************************
 * Defines
 */

// Settings 
#define NUM_STRIPS      6   // Number of strips connected to PIO pins
#define NUM_PIXELS      64 // Number of pixels in each strip
#define WS2812_PIN_BASE 2   // Which pin to start PIO outputs at
#define DMA_CHANNEL     0   // Bit plane DMA channel

// Constants
#define BITS_PER_BYTE   8   // Well, yeah.
#define DMA_CB_CHANNEL  1   // DMA chain channel
#define NUM_COLORS      3   // Red, green, blue

// Calculated values
#define DMA_CHANNEL_MASK (1u << DMA_CHANNEL)
#define DMA_CB_CHANNEL_MASK (1u << DMA_CB_CHANNEL)
#define DMA_CHANNELS_MASK (DMA_CHANNEL_MASK | DMA_CB_CHANNEL_MASK)

/*******************************************************************************
 * Structs
 */

// Frame buffer pixel format
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} pixel_t;

// Store pixel values (8 bits per color channel) for multiple strips in bit
// planes. Bit plane N has the Nth bit of each strip.
typedef struct {
    // stored MSB first
    uint32_t planes[BITS_PER_BYTE];
} value_bits_t;

/*******************************************************************************
 * Globals
 */

// Set of pointers to double buffer for sending values to DMA
static uintptr_t fragment_start[(NUM_PIXELS * NUM_COLORS) + 1];

// Posted when it is safe to output a new set of values
static struct semaphore reset_delay_complete_sem;

// Alarm handle for handling delay
alarm_id_t reset_delay_alarm_id;

// Double buffer containing "bit planes"
static value_bits_t bit_planes_buf[2][NUM_PIXELS * NUM_COLORS];

/*******************************************************************************
 * DMA functions
 */

// Reset delay
int64_t reset_delay_complete(alarm_id_t id, void *user_data) {
    reset_delay_alarm_id = 0;
    sem_release(&reset_delay_complete_sem);
    // no repeat
    return 0;
}

// ISR: Cancel alarm on DMA transfer complete
void __isr dma_complete_handler() {
    if (dma_hw->ints0 & DMA_CHANNEL_MASK) {
        // clear IRQ
        dma_hw->ints0 = DMA_CHANNEL_MASK;
        // when the dma is complete we start the reset delay timer
        if (reset_delay_alarm_id) cancel_alarm(reset_delay_alarm_id);
        reset_delay_alarm_id = add_alarm_in_us( 400, 
                                                reset_delay_complete, 
                                                NULL, 
                                                true);
    }
}

// Initialize DMA
void dma_init(PIO pio, uint sm) {
    dma_claim_mask(DMA_CHANNELS_MASK);

    // Main DMA channel outputs 8 word fragmens, then chains back to chain ch.
    dma_channel_config channel_config = 
                        dma_channel_get_default_config(DMA_CHANNEL);
    channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&channel_config, DMA_CB_CHANNEL);
    channel_config_set_irq_quiet(&channel_config, true);
    dma_channel_configure(DMA_CHANNEL,
                          &channel_config,
                          &pio->txf[sm],
                          NULL, // set by chain
                          8, // 8 words for 8 bit planes
                          false);

    // Chain channel sends single word pointer to start of fragment each time
    dma_channel_config chain_config = 
                        dma_channel_get_default_config(DMA_CB_CHANNEL);
    dma_channel_configure(DMA_CB_CHANNEL,
                          &chain_config,
                          &dma_channel_hw_addr(
                                // ch DMA config (target "ring" buffer size 4)
                                DMA_CHANNEL)->al3_read_addr_trig,  
                          NULL, // set later
                          1,
                          false);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
    dma_channel_set_irq0_enabled(DMA_CHANNEL, true);
    irq_set_enabled(DMA_IRQ_0, true);
}

// Send data to DMA
void output_strings_dma(value_bits_t *bits, uint value_length) {
    for (uint i = 0; i < value_length; i++) {
        fragment_start[i] = (uintptr_t) bits[i].planes; // MSB first
    }
    fragment_start[value_length] = 0;
    dma_channel_hw_addr(DMA_CB_CHANNEL)->al3_read_addr_trig = 
                                                (uintptr_t) fragment_start;
}

/*******************************************************************************
 * Custom functions
 */

// Transform led buffer to "bit planes" (e.g. transpose bits)
void fill_bit_planes(   value_bits_t *bit_planes, 
                        pixel_t pix_buf[NUM_STRIPS][NUM_PIXELS], 
                        uint8_t current) {
    
    char buf[20];
    uint32_t plane;

    // Copy bits in GRB format to bit planes
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {

        // Transpose green channel
        for (int bit = 0; bit < 8; bit++) {
            plane = 0;
            for (int strip = 0; strip < NUM_STRIPS; strip++) {
                plane += ((pix_buf[strip][pixel].g >> (7 - bit)) & 0x1) << 
                                                                        strip;
            }
            bit_planes[(3 * pixel) + 0].planes[bit] = plane;
        }
        sprintf(buf, "0x%08X", plane);

        // Transpose red channel
        for (int bit = 0; bit < 8; bit++) {
            plane = 0;
            for (int strip = 0; strip < NUM_STRIPS; strip++) {
                plane += ((pix_buf[strip][pixel].r >> (7 - bit)) & 0x1) << 
                                                                        strip;
            }
            bit_planes[(3 * pixel) + 1].planes[bit] = plane;
        }

        // Transpose blue channel
        for (int bit = 0; bit < 8; bit++) {
            plane = 0;
            for (int strip = 0; strip < NUM_STRIPS; strip++) {
                plane += ((pix_buf[strip][pixel].b >> (7 - bit)) & 0x1) << 
                                                                        strip;
            }
            bit_planes[(3 * pixel) + 2].planes[bit] = plane;
        }

    }
}

/*******************************************************************************
 * Main
 */

int main() {

    char buf[20];
    uint8_t current = 0;
    PIO pio;
    int sm;
    uint offset;
    pixel_t pix_buf[NUM_STRIPS][NUM_PIXELS];

    // Initialize framebuffer
    for (int i = 0; i < NUM_STRIPS; i++) {
        for (int j = 0; j < NUM_PIXELS; j++) {
            pix_buf[i][j].r = 0;
            pix_buf[i][j].g = 0;
            pix_buf[i][j].b = 0;
        }
    }

    // Initialize serial port(s)
    stdio_init_all();
    puts("WS2812 parallel");

    // Start PIO program
    pio = pio0;
    sm = pio_claim_unused_sm(pio, true);
    if (sm == -1) {
        puts("ERROR: Could not claim PIO state machine");
        while(1);
    }
    offset = pio_add_program(pio, &ws2812_parallel_program);
    ws2812_parallel_program_init(   pio, 
                                    sm, 
                                    offset, 
                                    WS2812_PIN_BASE, 
                                    NUM_STRIPS, 
                                    800000);

    // Set binary semaphore to 1 so program does not block first time
    sem_init(&reset_delay_complete_sem, 1, 1);

    // Initialize DMA
    dma_init(pio, sm);

    // Superloop
    while (1) {
        
        // Animation!
        uint8_t strip;
        uint32_t pos;
        uint32_t head;
        uint32_t tail;
        uint32_t max_tail = 8;
        for (int i = 0; i < NUM_PIXELS * pow(2, 5); i++) {

            // 1st strip: really slow red
            strip = 0;
            head = (uint32_t)(i / pow(2, 5)) % NUM_PIXELS;
            for (tail = 0; tail < max_tail; tail++) {
                pos = ((head + NUM_PIXELS) - tail) % NUM_PIXELS;
                pix_buf[strip][pos].r = 0x80 >> tail;
            }
            pos = ((head + NUM_PIXELS) - max_tail) % NUM_PIXELS;
            pix_buf[strip][pos].r = 0;

            // 2nd strip: slow orange
            strip = 1;
            head = (uint32_t)(i / pow(2, 4)) % NUM_PIXELS;
            for (tail = 0; tail < max_tail; tail++) {
                pos = ((head + NUM_PIXELS) - tail) % NUM_PIXELS;
                pix_buf[strip][pos].r = 0x80 >> tail;
                pix_buf[strip][pos].g = 0x08 >> tail;
            }
            pos = ((head + NUM_PIXELS) - max_tail) % NUM_PIXELS;
            pix_buf[strip][pos].r = 0;
            pix_buf[strip][pos].g = 0;

            // 3rd strip: leisurely yellow
            strip = 2;
            head = (uint32_t)(i / pow(2, 3)) % NUM_PIXELS;
            for (tail = 0; tail < max_tail; tail++) {
                pos = ((head + NUM_PIXELS) - tail) % NUM_PIXELS;
                pix_buf[strip][pos].r = 0x80 >> tail;
                pix_buf[strip][pos].g = 0x80 >> tail;
            }
            pos = ((head + NUM_PIXELS) - max_tail) % NUM_PIXELS;
            pix_buf[strip][pos].r = 0;
            pix_buf[strip][pos].g = 0;

            // 4th strip: peppy green
            strip = 3;
            head = (uint32_t)(i / pow(2, 2)) % NUM_PIXELS;
            for (tail = 0; tail < max_tail; tail++) {
                pos = ((head + NUM_PIXELS) - tail) % NUM_PIXELS;
                pix_buf[strip][pos].g = 0x80 >> tail;
            }
            pos = ((head + NUM_PIXELS) - max_tail) % NUM_PIXELS;
            pix_buf[strip][pos].g = 0;

            // 5th strip: hyperspeed blue
            strip = 4;
            head = (uint32_t)(i / pow(2, 1)) % NUM_PIXELS;
            for (tail = 0; tail < max_tail; tail++) {
                pos = ((head + NUM_PIXELS) - tail) % NUM_PIXELS;
                pix_buf[strip][pos].b = 0x80 >> tail;
            }
            pos = ((head + NUM_PIXELS) - max_tail) % NUM_PIXELS;
            pix_buf[strip][pos].b = 0;

            // 6th strip: ludicrous speed purple
            strip = 5;
            head = (uint32_t)(i) % NUM_PIXELS;
            for (tail = 0; tail < max_tail; tail++) {
                pos = ((head + NUM_PIXELS) - tail) % NUM_PIXELS;
                pix_buf[strip][pos].b = 0x80 >> tail;
                pix_buf[strip][pos].r = 0x80 >> tail;
            }
            pos = ((head + NUM_PIXELS) - max_tail) % NUM_PIXELS;
            pix_buf[strip][pos].b = 0;
            pix_buf[strip][pos].r = 0;

            // Transpose framebuffer to bit planes and send to PIO
            fill_bit_planes(bit_planes_buf[current], pix_buf, current);
            sem_acquire_blocking(&reset_delay_complete_sem);
            output_strings_dma(bit_planes_buf[current], NUM_PIXELS * 3);

            // Swap double buffer
            current ^= 1;
        }
    }
}