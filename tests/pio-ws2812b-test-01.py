# Example using PIO to drive a set of WS2812 LEDs.

import array, time
from machine import Pin
import rp2

# Configure the number of WS2812 LEDs.
NUM_LEDS = 150
PIN_NUM = 13
brightness = 0.2

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()


# Create the StateMachine with the ws2812 program, outputting on pin
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))

# Start the StateMachine, it will wait for data on its FIFO.
sm.active(1)

# Array to hold pixel color values
ar = array.array("I", [0 for _ in range(NUM_LEDS)])

# Push color values to state machine
def show(fb):
    for i in range(NUM_LEDS):
        sm.put(fb[i], 8)

# Test: one pixel on
while True:
    r = int(255 * brightness)
    g = int(127 * brightness)
    b = int(0 * brightness)
    print("Writing:", r, g, b)
    ar[1] = (g << 16) + (r << 8) + b
    show(ar)
    time.sleep(0.1)