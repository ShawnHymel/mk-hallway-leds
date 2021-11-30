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
#include "hardware/irq.h"

#include "ws2812_parallel.pio.h"

/*******************************************************************************
 * Defines
 */

// Settings (general)
#define DEBUG                   0   // 1 to print, 0 to silence
#define MODE_BUTTON_PIN         16  // Mode button pin
#define STATUS_LED_PIN          25  // Status LED pin
#define SENSOR_HEIGHT           200 // Distance (cm) from ground to sensor
#define SENSE_DISTANCE          700 // Length (cm) of hallway
#define CUTOFF_DISTANCE         650 // Stop tracking after this distance (cm)
#define GND_OFFSET              -10 // Offset (cm)
#define FADE                    3   // Amount of fade each step
#define DEBOUNCE_DELAY          40000   // Microseconds

// Settings (WS2812b superstrips)
#define WS2812_PIN_BASE         6   // Which pin to start PIO outputs at
#define NUM_SUPERSTRIPS         1   // Number of chained strips
#define STRIPS_PER_SUPERSTRIP   3   // Number of strips in 1 chained strip
#define PIXELS_PER_STRIP        150 // Number of pixels in a single strip
#define PIXELS_PER_METER        60  // Number of pixels per meter in a strip
#define DMA_CHANNEL             0   // Bit plane DMA channel

// Settings (TFMini Plus)
#define TFMINI_UART_ID          uart1
#define BAUD_RATE               115200
#define UART_TX_PIN             4
#define UART_RX_PIN             5
#define TFMINI_TIMEOUT          10000   // Microseconds

// Constants (WS2812b superstrips)
#define BITS_PER_BYTE           8   // Well, duh.
#define DMA_CB_CHANNEL          1   // DMA chain channel
#define NUM_COLORS              3   // Red, green, blue
#define WS2812_OK               0
#define WS2812_ERROR            -1

// Constants (TFMini Plus)
#define TFMINI_BUF_LEN          6
#define CM_PER_M                100 // Centimeters in a meter

// Calculated values (WS2812b superstrips)
#define NUM_STRIPS              NUM_SUPERSTRIPS * STRIPS_PER_SUPERSTRIP
#define NUM_PIXELS              NUM_STRIPS * PIXELS_PER_STRIP
#define PIXELS_PER_SUPERSTRIP   STRIPS_PER_SUPERSTRIP * PIXELS_PER_STRIP
#define DMA_CHANNEL_MASK        (1u << DMA_CHANNEL)
#define DMA_CB_CHANNEL_MASK     (1u << DMA_CB_CHANNEL)
#define DMA_CHANNELS_MASK       (DMA_CHANNEL_MASK | DMA_CB_CHANNEL_MASK)

/*******************************************************************************
 * Structs and enums
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

// TFMini error codes
typedef enum {
    TFMINI_OK,              // 0
    TFMINI_ERROR_TIMEOUT,   // 1
    TFMINI_ERROR_BUFOVR,    // 2
    TFMINI_ERROR_CHKSUM,    // 3
    TFMINI_ERROR_UNKNOWN    // 4
} TFMiniStatus;

// Operating modes
typedef enum {
    MODE_OFF = 0,
    MODE_CHASE,
    MODE_ON,
    NUM_MODES
} OpMode;

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
static pixel_t pix_buf[NUM_STRIPS][PIXELS_PER_STRIP];

// Double buffer containing "bit planes"
static value_bits_t bit_planes_buf[2][NUM_PIXELS * NUM_COLORS];

// Double buffer pointer
static uint8_t current = 0;

// Global TFMini buffer and such
static char tfmini_buf[TFMINI_BUF_LEN];
static uint8_t tfmini_status;
static volatile TFMini tfmini;
static volatile TFMiniStatus tfmini_status = TFMINI_OK;

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
                        pixel_t pix_buf[NUM_STRIPS][PIXELS_PER_STRIP], 
                        uint8_t current) {
    
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

pixel_t get_pixel(  uint8_t superstrip, 
                    int pixel) {
    
    pixel_t pix = {.r = 0, .g = 0, .b = 0};
    uint32_t strip;
    uint32_t pos;

    // Check to make sure our indices are not out of bounds
    if ((superstrip >= NUM_SUPERSTRIPS) || 
        (pixel >= PIXELS_PER_SUPERSTRIP)) {
        return pix;
    }

    // Don't do anything with negative pixel positions
    if (pixel < 0) {
        return pix;
    }

    // Convert to strip and pixel (in that strip)
    strip = (superstrip * STRIPS_PER_SUPERSTRIP) + (pixel / PIXELS_PER_STRIP);
    pos = pixel % PIXELS_PER_STRIP;

    // Get values
    pix.r = pix_buf[strip][pos].r;
    pix.g = pix_buf[strip][pos].g;
    pix.b = pix_buf[strip][pos].b;

    return pix;
}

// Set pixel to give RGB value
int set_pixel(  uint8_t superstrip, 
                int pixel, 
                uint8_t r, 
                uint8_t g, 
                uint8_t b) {

    uint32_t strip;
    uint32_t pos;

    // Check to make sure our indices are not out of bounds
    if ((superstrip >= NUM_SUPERSTRIPS) || 
        (pixel >= PIXELS_PER_SUPERSTRIP)) {
        return WS2812_ERROR;
    }

    // Ignore negative pixel positions
    if (pixel < 0) {
        return WS2812_OK;
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

// Interrupt handler
void on_tfmini_rx() {

    static uint8_t c;
    static uint8_t state = 0;
    static uint8_t sof_counter = 0;
    static bool receiving = true;
    static uint8_t buf_idx = 0;
    static uint32_t checksum = 0;

    // Keep reading from UART while data is available
    while (uart_is_readable(TFMINI_UART_ID)) {

        // Get a character
        c = uart_getc(TFMINI_UART_ID);

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
                tfmini_buf[buf_idx] = c;
                buf_idx++;
                if (buf_idx == TFMINI_BUF_LEN) {
                    buf_idx = 0;
                    state = 2;
                } else if (buf_idx > TFMINI_BUF_LEN) {
                    buf_idx = 0;
                    tfmini_status = TFMINI_ERROR_BUFOVR;
                    state = 0;
                }
                break;
            
            // End of frame
            case 2:

                // Calculate checksum
                checksum = 0x59 + 0x59; // SOF bytes are static
                for (int i = 0; i < TFMINI_BUF_LEN; i++) {
                    checksum += tfmini_buf[i];
                }

                // If last byte of checksum matches rx character, save data
                if ((checksum & 0xFF) == c) {
                    tfmini.distance = tfmini_buf[0] + (tfmini_buf[1] << 8);
                    tfmini.strength = tfmini_buf[2] + (tfmini_buf[3] << 8);
                    tfmini.temperature = tfmini_buf[4] + (tfmini_buf[5] << 8);
                    tfmini_status = TFMINI_OK;
                } else {
                    tfmini_status = TFMINI_ERROR_CHKSUM;
                }
                state = 0;
                break;

            // Reset state machine if we somehow end up here
            default:
                tfmini_status = TFMINI_ERROR_UNKNOWN;
                state = 0;
                break;
        }
    }
}

void tfmini_init() {

    // Set up our UART with a dummy baud rate
    uart_init(TFMINI_UART_ID, 2400);

    // Configure pins with UART functionality
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Calculate the real baud rate from the requested amount
    uart_set_baudrate(TFMINI_UART_ID, BAUD_RATE);

    // Disable CTS/RTS
    uart_set_hw_flow(TFMINI_UART_ID, false, false);

    // Set our data format
    uart_set_format(TFMINI_UART_ID, 8, 1, UART_PARITY_NONE);

    // Disable FIFOs, as we're handling each received character in an ISR
    uart_set_fifo_enabled(TFMINI_UART_ID, false);

    // Set up RX interrupt and interrupt handler
    int UART_IRQ = (TFMINI_UART_ID == uart0 ? UART0_IRQ : UART1_IRQ);
    irq_set_exclusive_handler(UART_IRQ, on_tfmini_rx);
    irq_set_enabled(UART_IRQ, true);

    // Enable the UART interrupt (RX only)
    uart_set_irq_enables(TFMINI_UART_ID, true, false);
}

/*******************************************************************************
 * Main
 */

int main() {

    PIO pio;
    int sm;
    uint offset;
    int gnd_dist = 0;
    int pos = 0;
    pixel_t pix;
    absolute_time_t timestamp;
    int64_t time_diff;
    uint16_t fps;
    float angle;
    uint8_t mode_btn_reading = 1;
    uint8_t mode_btn_state = 1;
    uint8_t mode_btn_state_prev = 1;
    absolute_time_t last_debounce;
    uint8_t mode_flag;
    OpMode mode = MODE_OFF;
    uint8_t static_disp_flag = 0;

    // Initialize status LED pin
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);

    // Initialize mode button
    gpio_init(MODE_BUTTON_PIN);
    gpio_set_dir(MODE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MODE_BUTTON_PIN);

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
        for (int j = 0; j < PIXELS_PER_SUPERSTRIP; j++) {
            set_pixel(i, j, 0, 0, 0);
        }
    }
    show();

    // Calculate angle of sensor
    angle = atan(SENSE_DISTANCE / SENSOR_HEIGHT);

    // Initialize TFMini UART
    tfmini_init();

    // Superloop
    timestamp = get_absolute_time();
    while (1) {

        // Button debounce
        mode_btn_reading = gpio_get(MODE_BUTTON_PIN);
        if (mode_btn_reading != mode_btn_state_prev) {
            last_debounce = timestamp;
        }
        if (absolute_time_diff_us(last_debounce, get_absolute_time()) > 
            DEBOUNCE_DELAY) {
            if (mode_btn_reading != mode_btn_state) {
                mode_btn_state = mode_btn_reading;
                if (mode_btn_state == 0) {
                    mode_flag = 1;
                }
            }
        }
        mode_btn_state_prev = mode_btn_state;

        // If mode flag is set, switch to new mode
        if (mode_flag == 1) {
            mode++;
            if (mode == NUM_MODES) {
                mode = MODE_OFF;
            }
            static_disp_flag = 1;
            mode_flag = 0;
        }

        // Check on status of TFMini
        if (tfmini_status != TFMINI_OK) {
#if DEBUG
            printf("ERROR: TFMini error code: %i\r\n", tfmini_status);
#endif
        }
        
        // Do display stuff depending on mode
        switch (mode) {

            // Turn off everything
            case MODE_OFF:
                for (int i = 0; i < NUM_SUPERSTRIPS; i++) {
                    for (int j = 0; j < PIXELS_PER_SUPERSTRIP; j++) {
                        set_pixel(i, j, 0, 0, 0);
                    }
                }
                if (static_disp_flag == 1) {
                    static_disp_flag = 0;
                    show();
                }
                break;

            // Chase animation
            case MODE_CHASE:

                // Fade out pixels
                for (int i = 0; i < NUM_SUPERSTRIPS; i++) {
                    for (int j = 0; j < PIXELS_PER_SUPERSTRIP; j++) {
                        pix = get_pixel(i, j);
                        pix.r = ((int16_t)pix.r-FADE) < 0 ? 0 : (pix.r-FADE);
                        pix.g = ((int16_t)pix.g-FADE) < 0 ? 0 : (pix.g-FADE);
                        pix.b = ((int16_t)pix.b-FADE) < 0 ? 0 : (pix.b-FADE);
                        set_pixel(i, j, pix.r, pix.g, pix.b);
                    }
                }
                
                // Calculate distance from point on floor below TFMini
                gnd_dist = (tfmini.distance * sin(angle)) + GND_OFFSET;

                // Calculate which pixel is above target
                pos = (gnd_dist * PIXELS_PER_METER) / CM_PER_M;

                // Only draw new pixels if within desired range
                if (gnd_dist <= CUTOFF_DISTANCE) {
                    for (int i = -2; i < 3; i++) {
                        set_pixel(0, pos + i, 50, 50, 255);
                    }
                }
                show();
                break;

            // Turn all the lights on
            case MODE_ON:
                for (int i = 0; i < NUM_SUPERSTRIPS; i++) {
                    for (int j = 0; j < PIXELS_PER_SUPERSTRIP; j++) {
                        set_pixel(i, j, 63, 61, 55);
                    }
                }
                if (static_disp_flag == 1) {
                    static_disp_flag = 0;
                    show();
                }
                break;

            default:
                break;
        }

        // Show debugging info
#if DEBUG
        time_diff = absolute_time_diff_us(timestamp, get_absolute_time());
        fps = 1000000 / time_diff;
        printf("Dist: %i cm, Gnd dist: %i cm, Pos:%i, FPS:%u\r\n", 
        tfmini.distance,
        gnd_dist,
        pos,
        fps);
        timestamp = get_absolute_time();
#endif
    }
}