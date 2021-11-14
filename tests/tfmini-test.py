# Tests TFmini (or TFmini Plus)

import board
import busio
import digitalio
import time

import adafruit_tfmini

# Pins
tx_1_pin = board.GP4   # White wire
rx_1_pin = board.GP5   # Green wire

# Use hardware uart
uart = busio.UART(tx=tx_1_pin, rx=rx_1_pin)

# Simplest use, connect with the uart bus object
tfmini = adafruit_tfmini.TFmini(uart)

# You can put in 'short' or 'long' distance mode
#tfmini.mode = adafruit_tfmini.MODE_SHORT
#print("Now in mode", tfmini.mode)

while True:
    print("Distance: %d cm (strength %d, mode %x)" %
          (tfmini.distance, tfmini.strength, tfmini.mode))
    time.sleep(0.1)