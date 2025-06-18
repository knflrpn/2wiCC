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

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "2wiCC.h"

#include "tusb.h"
#include "usb_descriptors.h"
#include "procon_functions.c"

ControllerData_t con_buffer[CON_BUF_SIZE];
uint16_t conbuf_head, conbuf_tail = 0;
uint8_t action_mode = A_RT;
bool usb_connected = false;

//--------------------------------------------------------------------
// USB HID
//--------------------------------------------------------------------
void parse_usb(uint8_t const *current_usb_buf, uint16_t len)
{

	uint8_t cmd = current_usb_buf[0];

	switch (cmd)
	{
	case 0x01:
		// Endpoint 0x01 is used for special requests (reading from
		// flash, changing modes, etc.).
		output_report_0x01(current_usb_buf, usb_special_buf);
		special_report_pending = true;
		break;
	case 0x10:
		// Endpoint 0x10 is used to transmit rumble data.
		// output_report_0x10(current_usb_buf, usb_special_buf);
		// Endpoint 0x10 is unused here, so don't set special_report_pending,
		// because there won't be a response to clear it.
		break;

	case 0x80:
		// Endpoint 0x80 is used for communication control
		// (ask for MAC address, set SPI baud rate)
		output_report_0x80(current_usb_buf, usb_special_buf);
		special_report_pending = true;
		break;

	case 0x30:
		// Endpoint 0x30 is used for standard controller state.
		report_input_full(current_usb_buf, usb_norm_buf);
        usb_connected = true; // If it got this far, it's definitely connected.
		break;
	}
	uint32_t current_time = to_ms_since_boot(get_absolute_time());
}

void hid_task(void)
{
	// report controller data
	if (tud_hid_ready())
	{
		uint32_t current_time = to_ms_since_boot(get_absolute_time());

		if (current_time - last_report_time > 250) // It's been too long -- try to connect
		{
			// Offer initial connection report
			const uint8_t response[] = {
				// Hello, I'm a pro controller (0x03) with KNfLrPn's pro controller's MAC address.
				0x81, 0x01, 0x00, 0x03, 0x64, 0xb6, 0xc9, 0x3e, 0x91, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			};
			memcpy(usb_special_buf, response, sizeof(response));
			special_report_pending = true;
			special_report_queued = false; // overwrite any previous report
			tud_hid_report(usb_special_buf[0], &usb_special_buf[1], 0x3F);
			last_report_time = current_time;
		}

		if (!special_report_pending)
		{
			if (polling_mode == 0x30)
			{ // 0x30 is full polling mode
				// Generate a normal report
				report_input_full(0, usb_norm_buf);
				tud_hid_report(usb_norm_buf[0], &usb_norm_buf[1], 0x3F);
				last_report_time = current_time;
			}
		}
		else
		{
			if (!special_report_queued)
			{
				tud_hid_report(usb_special_buf[0], &usb_special_buf[1], 0x3F);
				last_report_time = current_time;
				special_report_queued = true;
			}
		}
	}
}

//--------------------------------------------------------------------
// USB device callbacks
//--------------------------------------------------------------------

// Invoked when device is mounted
void tud_mount_cb(void)
{
	last_report_time = to_ms_since_boot(get_absolute_time());
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    usb_connected = false;
}

// Invoked when usb bus is suspended
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

// Invoked when report successfully to host
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
	(void)instance;
	(void)len;
	if (special_report_pending)
	{
		// Check if the sent report matches the pending special report
		if ((report[0] == usb_special_buf[0]) && (report[1] == usb_special_buf[1]))
		{
			// Just sent the special report.
			special_report_pending = false;
			special_report_queued = false;
		}
		else
		{
		}
	}
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
	return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
	if (buffer[0] != 0x00)
		parse_usb(buffer, bufsize);
}


//--------------------------------------------------------------------
// 2wiCC Communication and Behavior
//--------------------------------------------------------------------

/* Each time a character is received, process it.
 *  Uses state in cmd_state to track what is happening.
 */
void on_uart_rx()
{
    static int cmd_state = C_IDLE;
    #define CMD_STR_LEN 64
    static char cmd_str[CMD_STR_LEN];  // incoming command string
    static uint8_t uart_count = 0;     // tail pointer
    static uint8_t cmd_str_ind = 0;    // index into command string

    while (uart_is_readable(uart0))
    {
        uint8_t ch = uart_getc(uart0);

        // hard force new action on command character
        if (ch == CMD_CHAR)
        {
            // reset command string
            cmd_str_ind = 0;
            memset(cmd_str, 0, CMD_STR_LEN); // Fill the string with null
            uart_count = 0;
        }
        // parse the command on newline
        else if ((ch == '\r') || (ch == '\n'))
        {

            // ID self
            if (strncmp(cmd_str, "ID ", 3) == 0)
            {
                uart_puts(uart0, "+2wiCC \r\n");
            }

            // Get version
            if (strncmp(cmd_str, "VER ", 4) == 0)
            {
                uart_puts(uart0, "+VER 1.0\r\n");
            }

            // Add to queue
            if (strncmp(cmd_str, "Q ", 2) == 0)
            {
                //add_to_queue(cmd_str + 2);
            }

            // Set VSYNC delay
            if (strncmp(cmd_str, "VSD ", 4) == 0)
            {
                //set_frame_delay(cmd_str + 4);
            }

            // Start recording
            if (strncmp(cmd_str, "REC ", 4) == 0)
            {
                /*
                if (cmd_str[4] == '1') {
                    rec_head = 0;
                    recording_wrap = false;
                    memcpy(&(rec_data_buff[rec_head]), &current_con, sizeof(USB_ControllerReport_Input_t));
                    rec_rle_buff[rec_head] = 1;
                    recording = true;
                } else {
                    recording = false;
                }
                */
            }

            // Get USB connection status
            if (strncmp(cmd_str, "GCS ", 4) == 0)
            {
                if (usb_connected)
                    uart_puts(uart0, "+GCS 1\r\n");
                else
                    uart_puts(uart0, "+GCS 0\r\n");
            }

            // Get queue buffer fullness
            if (strncmp(cmd_str, "GQF ", 4) == 0)
            {
                //uart_resp_int("GQF", get_queue_fill());
            }

            // Get recording buffer fullness
            if (strncmp(cmd_str, "GRF ", 4) == 0)
            {
                /*
                // If recording has wrapped, it is full
                if (recording_wrap) {
                    uart_resp_int("GRF", (unsigned int)(REC_BUFF_LEN));
                } else {
                    uart_resp_int("GRF", (unsigned int)(rec_head));
                }
                */
            }
            // Get recording buffer remaining
            if (strncmp(cmd_str, "GRR ", 4) == 0)
            {
                /*
                // If recording has wrapped, it is empty
                if (recording_wrap) {
                    uart_resp_int("GRR", (unsigned int)(0));
                } else {
                    uart_resp_int("GRR", (unsigned int)(REC_BUFF_LEN - rec_head));
                }
                */
            }
            // Get total recording buffer size
            if (strncmp(cmd_str, "GRB ", 4) == 0)
            {
                /*
                // If recording has wrapped, it is empty
                uart_resp_int("GRB", (unsigned int)(REC_BUFF_LEN));
                */
            }

            // Retrieve recording
            if (strncmp(cmd_str, "GR ", 3) == 0)
            {
                /*
                if (cmd_str[3] == '0') {
                    // Start at beginning
                    if (recording_wrap) {
                        // If wrapped, oldest value is just in front of head
                        stream_head = (rec_head + 1) % REC_BUFF_LEN;
                    } else {
                        stream_head = 0;
                    }
                }
                send_recording();
                if (stream_head == rec_head)
                {
                    // end of stream, entire recording has been sent
                    uart_puts(uart0, "+GR 0\r\n");
                }
                else
                {
                    // end of stream but more is pending
                    uart_puts(uart0, "+GR 1\r\n");
                }
                */
            }

            // Enable / disable vsync synchronization
            if (strncmp(cmd_str, "VSYNC ", 6) == 0)
            {
                /*
                if (cmd_str[6] == '1')
                {
                    vsync_en = true;
                    // Set up GPIO interrupt
                    gpio_set_irq_enabled_with_callback(VSYNC_IN_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
                    vsync_count = 0;
                }
                else if (cmd_str[6] == '0')
                {
                    vsync_en = false;
                    // Disable GPIO interrupt
                    gpio_set_irq_enabled(VSYNC_IN_PIN, GPIO_IRQ_EDGE_RISE, false);
                    alarm_in_us(16666); // set an alarm 1/60s in the future
                }
                else {
                    if (vsync_en)
                        uart_puts(uart0, "+VSYNC 1\r\n");
                    else
                        uart_puts(uart0, "+VSYNC 0\r\n");
                }
                */
            }

            // Enable / disable LED
            if (strncmp(cmd_str, "LED ", 4) == 0)
            {
                /*
                if (cmd_str[4] == '1')
                {
                    led_on = true;
                }
                else
                {
                    led_on = false;
                }
                */
            }

            memset(cmd_str, 0, CMD_STR_LEN); // Clear command; if it wasn't valid, it never will be
        }
        else
        {
            // add chars to the string
            if (cmd_str_ind < (CMD_STR_LEN))
            {
                cmd_str[cmd_str_ind] = ch;
                cmd_str_ind++;
                uart_count++;
            }
        }
    }
}


//--------------------------------------------------------------------
// Functionality
//--------------------------------------------------------------------
/* Alarm interrupt handler.
   This happens once per game frame, and is used to update the USB data.
*/
/*
static void alarm_irq(void)
{
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << 0);
//    if (!vsync_en)
    {
        alarm_in_us(16667); // set an alarm 1/60s in the future
//        vsync_count++;
    }
}
*/
/* Set up an alarm in the future.
 */
/*
static void alarm_in_us(uint32_t delay_us)
{
    // Enable the interrupt for the alarm
    hw_set_bits(&timer_hw->inte, 1u << 0);
    // Set irq handler for alarm irq
    irq_set_exclusive_handler(TIMER_IRQ_0, alarm_irq);
    // Enable the alarm irq
    irq_set_enabled(TIMER_IRQ_0, true);
    // Enable interrupt in block and at processor
    uint64_t target = timer_hw->timerawl + delay_us;

    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[0] = (uint32_t)target;
}
*/
//--------------------------------------------------------------------
// GPIO code
//--------------------------------------------------------------------

/* GPIO interrupt handler
 */
void gpio_callback(uint gpio, uint32_t events)
{
    // set up an interrupt in the future to change controller data
//    alarm_in_us(frame_delay_us);
//    vsync_count++;
}

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    static uint8_t heartbeat = 1;

    if ((heartbeat%4) == 1) {
        controller_state.button_x = 1;
    } else {
        controller_state.button_x = 0;
    }
    if ((heartbeat%4) == 3) {
        controller_state.button_y = 1;
    } else {
        controller_state.button_y = 0;
    }
//    controller_state.button_a = !controller_state.button_a;
    debug_pixel(urgb_u32(controller_state.button_y, 1, 0));
    heartbeat++;

    add_alarm_in_ms(500, alarm_callback, NULL, false);
    return 0;
}

int main()
{
	stdio_init_all();
    // Set up UART with a 2wiCC standard baud rate.
    uart_init(uart0, 921600u);
    // Set the TX and RX pins
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    // Turn off UART flow control CTS/RTS
    uart_set_hw_flow(uart0, false, false);
    // Set data format: 8N1
    uart_set_format(uart0, 8, 1, 0);
    // Turn on FIFO
    uart_set_fifo_enabled(uart0, false);
    // Set up an RX interrupt
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    // Eable the UART to send interrupts (on RX only)
    uart_set_irq_enables(uart0, true, false);

	// Set up debug neopixel
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &ws2812_program);
	ws2812_program_init(pio, 0, offset, 16, 800000, IS_RGBW);
	debug_pixel(urgb_u32(0, 0, 1));

	add_alarm_in_ms(2000, alarm_callback, NULL, false);

	// Set up USB
	tusb_init();

	set_neutral_con();

	while (true)
	{
		tud_task();
		hid_task();
	}
}
