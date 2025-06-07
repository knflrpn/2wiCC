#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "2wiCC.h"

#include "tusb.h"
#include "usb_descriptors.h"
#include "procon_functions.c"

// SPI Defines
// Use SPI 0 and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19


int64_t alarm_callback(alarm_id_t id, void *user_data) {
    static uint8_t heartbeat = 1;

    if ((heartbeat%4) == 1) {
        controller_state.button_x = 1;
    } else {
        controller_state.button_x = 0;
    }
    if ((heartbeat%4) == 3) {
        controller_state.button_a = 1;
    } else {
        controller_state.button_a = 0;
    }
//    controller_state.button_a = !controller_state.button_a;
    debug_pixel(urgb_u32(controller_state.button_a, 1, 0));
    heartbeat++;

    add_alarm_in_ms(500, alarm_callback, NULL, false);
    return 0;
}



int main()
{
    stdio_init_all();
    uart_init(uart0, 115200);
    printf("Hi.\n");

    // SPI initialization. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // Set up debug neopixel
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, 0, offset, 16, 800000, IS_RGBW);
    debug_pixel(urgb_u32(1, 1, 0));
    
    add_alarm_in_ms(2000, alarm_callback, NULL, false);

    // Set up USB
    tusb_init();

    set_neutral_con();

    while (true) {
        tud_task();
        hid_task();
    }
}

//--------------------------------------------------------------------
// Device callbacks
//--------------------------------------------------------------------

// Invoked when device is mounted
void tud_mount_cb(void)
{
    last_report_time = to_ms_since_boot(get_absolute_time());
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

// Invoked when sent REPORT successfully to host
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
    (void)instance;
    (void)len;
    if(special_report_pending) {
        if ((report[0] == usb_special_buf[0]) && (report[1] == usb_special_buf[1])) {
            // Just sent the special report.
            special_report_pending = false;
            special_report_queued = false;
        }
    }
//    printf("Done.\n");
}

// Invoked when received GET_REPORT control request
// Nothing to do here; can just STALL (return 0)
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    printf("grcb\n");
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    if (buffer[0] != 0x00)
        parse_usb(buffer, bufsize);
/*
    char pbuffer[50];
    sprintf(pbuffer, "Report inst %X id %X: %X %X\n", instance, report_id, buffer[0], buffer[1]);
    printf(pbuffer);
    */
}

//--------------------------------------------------------------------
// USB HID
//--------------------------------------------------------------------

void parse_usb(uint8_t const *current_usb_buf, uint16_t len)
{

    uint8_t cmd = current_usb_buf[0];

    switch (cmd)
    {
    case kReportIdOutput01:
        output_report_0x01(current_usb_buf, usb_special_buf);
        special_report_pending = true;
//        printf("r01\n");
        break;
    case kReportIdOutput10:
        output_report_0x10(current_usb_buf, usb_special_buf);
//        special_report_pending = true;
//        printf("r10\n");
        break;

    case kUsbReportIdOutput80:
        output_report_0x80(current_usb_buf, usb_special_buf);
        special_report_pending = true;
//        printf("r80\n");
        break;

    case kReportIdInput30:
        //default:
        input_report_0x30(current_usb_buf, usb_norm_buf);
        special_report_pending = false;
        printf("Default.\n");
        break;
    }
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    printf("r\n");

}

void hid_task(void)
{
    // report controller data
    if (tud_hid_ready())
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        if (current_time - last_report_time > 250){
            printf("Timeout\n");
            // Offer initial connection report
            const uint8_t response_h[] = {
                0x81, 0x01, 0x00, 0x03, 0xc1, 0xc9, 0x3e, 0xe9, 0xb6, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ......>......... */
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ................ */
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ................ */
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ................ */
            };
            memcpy(usb_special_buf, response_h, sizeof(response_h));
            special_report_pending = true;
            tud_hid_report(usb_special_buf[0], &usb_special_buf[1], 0x3F);
            last_report_time = current_time;
        }

        if (!special_report_pending) {
            if (polling_mode == 0x30) { // 0x30 is full polling mode
                // Generate a normal report
                input_report_0x30(0, usb_norm_buf);
                tud_hid_report(usb_norm_buf[0], &usb_norm_buf[1], 0x3F);
                last_report_time = current_time;
            }
        } else {
            if (!special_report_queued) {
                tud_hid_report(usb_special_buf[0], &usb_special_buf[1], 0x3F);
                last_report_time = current_time;
                special_report_queued = true;
            }
        }
    }
}

