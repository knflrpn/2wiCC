#include <stdint.h>
#include "ws2812.pio.h"

// Controller HID report structure.
typedef struct {
	uint16_t Button; // 16 buttons;
	uint8_t  HAT;    // HAT switch; one nibble w/ unused nibble
	uint8_t  LX;     // Left  Stick X
	uint8_t  LY;     // Left  Stick Y
	uint8_t  RX;     // Right Stick X
	uint8_t  RY;     // Right Stick Y
	uint8_t  VendorSpec;
} USB_ControllerReport_Input_t;

void parse_usb(uint8_t const *current_usb_buf, uint16_t len);
void hid_task(void);

//--------------------------------------------------------------------
// NeoPixel control
//--------------------------------------------------------------------
#define IS_RGBW false

static inline void debug_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline void feedback_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 2, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}
