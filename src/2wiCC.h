/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 KNfLrPn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdint.h>
#include "ws2812.pio.h"

// Controller HID report structure.
typedef struct
{
	uint16_t Button; // 16 buttons;
	uint8_t HAT;	 // HAT switch; one nibble w/ unused nibble
	uint8_t LX;		 // Left  Stick X
	uint8_t LY;		 // Left  Stick Y
	uint8_t RX;		 // Right Stick X
	uint8_t RY;		 // Right Stick Y
	uint8_t VendorSpec;
} USB_ControllerReport_Input_t;

// Action state
enum {
	A_RT,   // real-time
	A_PLAY, // play from buffer
};

// Serial control information
enum {
    C_IDLE,        // nothing happening
    C_ACTIVATED,   // activated by command character
    C_Q,           // receiving a controller state for queue
    C_I,           // receiving an immediate controller state
	C_F,           // request for queue buffer fill amount
	C_M,           // mode change
	C_R,           // request to read from record buffer 
	C_D            // receiving a new delay value
};

#define CMD_CHAR '+'

#define CON_BUF_SIZE 1024

//--------------------------------------------------------------------
// NeoPixel control
//--------------------------------------------------------------------
#define IS_RGBW false

static inline void debug_pixel(uint32_t pixel_grb)
{
	pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline void feedback_pixel(uint32_t pixel_grb)
{
	pio_sm_put_blocking(pio0, 2, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
	return ((uint32_t)(r) << 8) |
		   ((uint32_t)(g) << 16) |
		   (uint32_t)(b);
}

void hid_task(void);
void parse_usb(uint8_t const *current_usb_buf, uint16_t len);