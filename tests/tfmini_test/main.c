#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// Settings
#define TFMINI_UART_ID  uart1
#define BAUD_RATE       115200
#define UART_TX_PIN     4
#define UART_RX_PIN     5
#define TFMINI_TIMEOUT  10000   // Microseconds

#define TFMINI_BUF_LEN  6
#define TFMINI_OK       0
#define TFMINI_ERROR    -1

typedef struct {
    uint16_t distance;
    uint16_t strength;
    uint16_t temperature;
} TFMini;

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

int main() {

    TFMini tfmini;
    const uint led_pin = 25;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    stdio_init_all();
    printf("TFMini Plus Test\r\n");

    // Initialize TFMini UART
    uart_init(TFMINI_UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Loop forever
    while (true) {
        if (TFMINI_OK != tfmini_update( &tfmini, 
                                        TFMINI_UART_ID, 
                                        TFMINI_TIMEOUT)) {
            printf("ERROR: Could not receive data from TFMini\r\n");
        }
        printf("Dist: %i\r\n", tfmini.distance);
        // printf("Str:  %i\r\n", tfmini.strength);
        // printf("Temp: %i\r\n", tfmini.temperature);
        // printf("---\r\n");
    }
}