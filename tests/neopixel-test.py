# Make sure the LED strips are working

import board
import digitalio
import time

import neopixel

# Pins
neo_0_pin = board.GP13

# Number of pixels on each strip (60 LEDs per meter)
neo_0_num_pixels = 150

# Onboard LED for debugging
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Initialize NeoPixel strips
neo_0_pixels = neopixel.NeoPixel(neo_0_pin, neo_0_num_pixels)
neo_0_pixels.brightness = 1.0

# Light up every pixel in the strip(s), toggle LED, say something
while True:
    for i in range(neo_0_num_pixels):
        if i == 0:
            neo_0_pixels[neo_0_num_pixels - 1] = (0, 0, 0)
        if i > 0:
            neo_0_pixels[i - 1] = (0, 0, 0)
        neo_0_pixels[i] = (255, 255, 255)
        neo_0_pixels.show()
    led.value = not led.value
    print("boop")