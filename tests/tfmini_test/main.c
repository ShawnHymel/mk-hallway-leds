#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

// Settings
#define TFMINI_UART_ID  uart1
#define BAUD_RATE       115200
#define UART_TX_PIN     4
#define UART_RX_PIN     5
#define TFMINI_TIMEOUT  10000   // Microseconds

#define TFMINI_BUF_LEN  6

// Struct to store TFMini received data
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

// Global TFMini buffer and struct
static char tfmini_buf[TFMINI_BUF_LEN];
static uint8_t tfmini_status;
static TFMini tfmini;
static volatile TFMiniStatus tfmini_status = TFMINI_OK;

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

int main() {

    
    const uint led_pin = 25;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    stdio_init_all();
    printf("TFMini Plus Test\r\n");

    // Initialize TFMini UART
    tfmini_init();
    printf("TFMini initialized\r\n");

    // Loop forever
    while (true) {
        if (tfmini_status != TFMINI_OK) {
            printf("TFMINI ERROR: %i\r\n", tfmini_status);
        }
        printf("Dist: %i cm\r\n", tfmini.distance);
        sleep_ms(10);
    }
}