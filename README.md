# Mystic Krewe - Hallway LEDs

Project to control the NeoPixel strips mounted in the hallway of the PinChurch.

## Getting Started

You must install the Raspberry Pi Pico C/C++ SDK. Follow the setup instructions in the [Pico Getting Started Guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf). If you are using Windows, I recommend following [my guide here](https://shawnhymel.com/2096/how-to-set-up-raspberry-pi-pico-c-c-toolchain-on-windows-with-vs-code/).

Clone or download this repository.

```
git clone https://github.com/ShawnHymel/mk-hallway-leds
```

Run cmake to configure the build environment:

```
cd mk-hallway-leds/neochaser
mkdir build
cd build
cmake ..
```

Note: if you are using Windows, you will want to use `cmake -G "MinGW Makefiles" ..` instead.

Build the project:

```
make
```

Put the Pico into bootloader mode by pressing and holding the BOOTSEL button while plugging in the USB cable. This should force the Pico to enumerate on your computer as a mass storage device.

Copy **neochaser.uf2** to the root directory of the Pico drive that was mounted. The Pico should automatically restart and begin running the program.

You can clean the compiled binaries and build environment by simply removing the *build* directory we created.

Apply power and press the button to cycle through operating modes.

## About the Code

> :warning: **WARNING!** Never set all of the LEDs to white and more than half brightness (128 out of 255). The wiring and power supply cannot handle that amount of current. It is a **fire hazard**.

All of the code you need should be in *main.c*. I was too lazy to break it apart into libraries.

Distance measurements from the TFMini are updated every time a full message is received from the sensor over UART. This is handled by the `on_tfmini_rx()` interrupt service routine. You should never need to call this function or poll the distance sensor--it is handled in the background.

Modes are simply cycled in order whenever the pushbutton (attached to pin 16) is pressed. by default, the modes are:

 * `MODE_OFF`: All LEDs are off
 * `MODE_CHASE`: LEDs perform a chase animation following the nearest object to the distance sensor
 * `MODE_ON`: Turn all LEDs on at some low power (white)

The distance from the TFMini Plus to the nearest measured object is stored in `tfmini.distance`. This is a `uint16_t` value that gives the distance in centimeters (cm). See the `MODE_CHASE` pattern for an example on calculating the distance from the ground below the TFMini to the nearest object.

You set the color/brightness of a pixel with the following function:

```
int set_pixel(uint8_t superstrip, int pixel, uint8_t r, uint8_t g, uint8_t b);
```

* `superstrip` should always be 0. It is a holdover from when the code used to support separate, parallel NeoPixel strips. The 2 strips are connected to the same data pin, so the patterns are mirrored.
* `pixel` is the pixel position, starting from 0 (first pixel on the strip where the data line feeds in). 
* `r` is the red value (0..255)
* `g` is the green value (0..255)
* `b` is the blue value (0..255)


The function returns `WS2812_ERROR` if you tried to index a pixel out of bounds or `WS2812_OK` otherwise.

Make sure to call `show()` during your animation when you want to update the pattern on the NeoPixel strips. This function writes out the frame buffer (that you modified by calling `set_pixel()`) to the actual NeoPixel strips.

## Adding a Custom Pattern

Let's create a simple chasing pattern and add it to our program.

First, add the name of the mode to the `OpMode` enum. Make sure it is above `NUM_MODES` (which is needed for the mode cycling process). For example, I'll call my mode `MODE_RUNNER`:

```
// Operating modes
typedef enum {
    MODE_OFF = 0,
    MODE_CHASE,
    MODE_ON,
    MODE_RUNNER,
    NUM_MODES
} OpMode;
```

Next, create your animation in the main superloop as a mode under `switch(mode)`. Here's one possible way of implementing our custom runner animation. Note that we use the `pos` variable, which was declared before the superloop. It is used in the `MODE_CHASE` animation, but we can also use it here to remember the position of the on LED.

```c
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

    // *** My custom runner code ***

    // Simple runner animation
    case MODE_RUNNER:

        // Set pixel to red
        set_pixel(0, pos, 255, 0, 0);

        // Write framebuffer to NeoPixel strips
        show();

        // Update position counter and wrap
        pos++;
        if (pos >= PIXELS_PER_SUPERSTRIP) {
            pos = 0;
        }

        // Wait before updating the strips again
        // Note: keep this below 40 ms, otherwise, the button will lag
        sleep_ms(15);

        // Exit the switch/case statement
        break;

    // *** End my custom runner code ***

    default:
        break;
```

## License

Unless otherwise specified, source code is licensed under the BSD-3-Clause license.

Copyright 2022 Shawn Hymel

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.