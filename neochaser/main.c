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
#include "hardware/uart.h"

#include "ws2812_parallel.pio.h"

/*******************************************************************************
 * Defines
 */

// Settings (general)
#define DEBUG                   1   // 1 to print, 0 to silence
#define STATUS_LED_PIN          25  // Status LED pin

// Settings (WS2812b superstrips)
#define WS2812_PIN_BASE         6   // Which pin to start PIO outputs at
#define NUM_SUPERSTRIPS         2   // Number of chained strips
#define STRIPS_PER_SUPERSTRIP   3   // Number of strips in 1 chained strip
#define PIXELS_PER_STRIP        150 // Number of pixels in a single strip
#define PIXELS_PER_METER        60  // Number of pixels per meter in a strip
#define DMA_CHANNEL             0   // Bit plane DMA channel

// Settings (TFMini Plus)
#define TFMINI_UART_ID          uart1
#define BAUD_RATE               115200
#define UART_TX_PIN             4
#define UART_RX_PIN             5
#define TFMINI_TIMEOUT          50000   // Microseconds

// Constants (WS2812b superstrips)
#define BITS_PER_BYTE           8   // Well, duh.
#define DMA_CB_CHANNEL          1   // DMA chain channel
#define NUM_COLORS              3   // Red, green, blue
#define WS2812_OK               0
#define WS2812_ERROR            -1

// Constants (TFMini Plus)
#define TFMINI_BUF_LEN          6
#define TFMINI_OK               0
#define TFMINI_ERROR            -1
#define CM_PER_M                100 // Centimeters in a meter

// Calculated values (WS2812b superstrips)
#define NUM_STRIPS              NUM_SUPERSTRIPS * STRIPS_PER_SUPERSTRIP
#define NUM_PIXELS              NUM_STRIPS * PIXELS_PER_STRIP
#define DMA_CHANNEL_MASK        (1u << DMA_CHANNEL)
#define DMA_CB_CHANNEL_MASK     (1u << DMA_CB_CHANNEL)
#define DMA_CHANNELS_MASK       (DMA_CHANNEL_MASK | DMA_CB_CHANNEL_MASK)

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

// Way to store output of TFMini
typedef struct {
    uint16_t distance;
    uint16_t strength;
    uint16_t temperature;
} TFMini;

/*******************************************************************************
 * Globals
 */

// Set of pointers to double buffer for sending values to DMA
static uintptr_t fragment_start[(NUM_PIXELS * NUM_COLORS) + 1];

// Posted when it is safe to output a new set of values
static struct semaphore reset_delay_complete_sem;

// Alarm handle for handling delay
alarm_id_t reset_delay_alarm_id;

// Buffer for holding pixel values
static pixel_t pix_buf[NUM_STRIPS][NUM_PIXELS];

// Double buffer containing "bit planes"
static value_bits_t bit_planes_buf[2][NUM_PIXELS * NUM_COLORS];

// Double buffer pointer
static uint8_t current = 0;

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
 * Custom functions: WS2812b strip control
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

// Set pixel to give RGB value
int set_pixel(  uint8_t superstrip, 
                uint32_t pixel, 
                uint8_t r, 
                uint8_t g, 
                uint8_t b) {

    uint32_t strip;
    uint32_t pos;

    // Check to make sure our indices are not out of bounds
    if ((superstrip >= NUM_SUPERSTRIPS) || 
        (pixel >= (STRIPS_PER_SUPERSTRIP * PIXELS_PER_STRIP))) {
        return WS2812_ERROR;
    }

    // Convert to strip and pixel (in that strip)
    strip = (superstrip * STRIPS_PER_SUPERSTRIP) + (pixel / PIXELS_PER_STRIP);
    pos = pixel % PIXELS_PER_STRIP;

    // Store in buffer
    pix_buf[strip][pos].r = r;
    pix_buf[strip][pos].g = g;
    pix_buf[strip][pos].b = b;

    return WS2812_OK;
}

// Display buffer to WS2812b strips
void show() {

    // Transpose framebuffer to bit planes and send to PIO
    fill_bit_planes(bit_planes_buf[current], pix_buf, current);
    sem_acquire_blocking(&reset_delay_complete_sem);
    output_strings_dma(bit_planes_buf[current], NUM_PIXELS * 3);

    // Swap double buffer
    current ^= 1;
}

/*******************************************************************************
 * Custom functions: TFMini Plus control
 */

// State machine to parse UART messages from TFMini
int tfmini_rx(  char *buf, 
                uint8_t buf_len_max, 
                uart_inst_t *uart, 
                uint32_t timeout_us) {
    
    char c;
    uint8_t state = 0;
    uint8_t sof_counter = 0;
    bool receiving = true;
    uint8_t buf_idx = 0;
    uint32_t checksum = 0;
    
    while (receiving == true) {

        // Wait for RX buffer to have some data, timeout otherwise
        if (uart_is_readable_within_us(uart, timeout_us) != true){
            printf("TIMEOUT\r\n");
            return TFMINI_ERROR;
        }

        // Get character
        c = uart_getc(uart);

        // State machine to receive frame
        switch(state) {

            // Find start of frame
            case 0:
                if (c == 0x59) {
                    sof_counter++;
                }
                if (sof_counter >= 2) {
                    sof_counter = 0;
                    state = 1;
                }
                break;
            
            // Fill buffer
            case 1:
                buf[buf_idx] = c;
                buf_idx++;
                if (buf_idx > buf_len_max) {
                    printf("BUFFER OVERFLOW\r\n");
                    return TFMINI_ERROR;
                }
                if (buf_idx >= 6) { // Only 6 bytes in TFMini payload
                    state = 2;
                }
                break;
            
            // Checksum calculation (lower 8 bits of sum of received bytes)
            case 2:
                checksum = 0x59 + 0x59; // SOF bytes are static
                for (int i = 0; i < buf_idx; i++) {
                    checksum += buf[i];
                }
                if ((checksum & 0xFF) != c) {
                    printf("BAD CHECKSUM\r\n");
                    return TFMINI_ERROR;    
                } else {
                    state = 0;
                    receiving = false;
                }
                break;

            // Reset state machine if we somehow end up here
            default:
                state = 0;
                break;
        }
    }

    return TFMINI_OK;
}

// Update the TFMini struct with all the yummy data
int tfmini_update(TFMini *tfmini, uart_inst_t *uart, uint32_t timeout_us) {

    char buf[TFMINI_BUF_LEN];

    // Get data
    if (tfmini_rx( buf, TFMINI_BUF_LEN, uart, timeout_us) != TFMINI_OK) {
        return TFMINI_ERROR;
    }

    // Parse data
    tfmini->distance = buf[0] + (buf[1] << 8);
    tfmini->strength = buf[2] + (buf[3] << 8);
    tfmini->temperature = buf[4] + (buf[5] << 8);

    return TFMINI_OK;
}

/*******************************************************************************
 * Main
 */

int main() {

    char buf[20];
    PIO pio;
    int sm;
    uint offset;
    TFMini tfmini;
    uint16_t gnd_dist = 0;
    uint32_t prev_pos = 0;
    uint32_t pos = 0;

    // Initialize status LED pin
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);

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

    // Clear pixel display
    for (int i = 0; i < NUM_SUPERSTRIPS; i++) {
        for (int j = 0; j < STRIPS_PER_SUPERSTRIP * PIXELS_PER_STRIP; j++) {
            set_pixel(i, j, 0, 0, 0);
        }
    }
    show();

    // Initialize TFMini UART
    uart_init(TFMINI_UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    tfmini.distance = 0;
    tfmini.strength = 0;
    tfmini.temperature = 0;

    // Superloop
    while (1) {

        // Get distance from TFMini
        if (TFMINI_OK != tfmini_update( &tfmini, 
                                        TFMINI_UART_ID, 
                                        TFMINI_TIMEOUT)) {
            printf("ERROR: Could not receive data from TFMini\r\n");
        }

        // Calculate distance from point on floor below TFMini
        // %%%TODO
        gnd_dist = tfmini.distance;

        // Calculate which pixel is above target
        pos = (gnd_dist * PIXELS_PER_METER) / CM_PER_M;
#if DEBUG
        printf("Dist: %i cm, Gnd dist: %i cm, Pos:%i\r\n", 
                tfmini.distance,
                gnd_dist,
                pos);
#endif

        // // Clear previous pixels
        // %%%TODO: this is causing UART Rx to fail (buf overflow? Missed frames?)
        // set_pixel(0, prev_pos, 0, 0, 0);
        // set_pixel(1, prev_pos, 0, 0, 0);

        // // Show current location
        // set_pixel(0, pos, 0, 0, 255);
        // set_pixel(1, pos, 0, 0, 255);
        // show();

        // // Remember previous position
        // prev_pos = pos;
    }
}