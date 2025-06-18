#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
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

bool usb_connected = 0;

int64_t alarm_callback(alarm_id_t id, void *user_data)
{
	static uint8_t heartbeat = 1;

	if ((heartbeat % 4) == 1)
	{
		con_buffer[conbuf_tail].digital.button_x = 1;
	}
	else
	{
		con_buffer[conbuf_tail].digital.button_x = 0;
	}
	if ((heartbeat % 4) == 3)
	{
		con_buffer[conbuf_tail].digital.button_y = 1;
	}
	else
	{
		con_buffer[conbuf_tail].digital.button_y = 0;
	}
	//    con_buffer[conbuf_tail].button_a = !con_buffer[conbuf_tail].button_a;
	debug_pixel(urgb_u32(con_buffer[conbuf_tail].digital.button_y, 1, 0));
	heartbeat++;

	add_alarm_in_ms(500, alarm_callback, NULL, false);
	return 0;
}

/*
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

			}

			// Get USB connection status
			if (strncmp(cmd_str, "GCS ", 4) == 0)
			{
				/*
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

			}
			// Get total recording buffer size
			if (strncmp(cmd_str, "GRB ", 4) == 0)
			{
				/*
				// If recording has wrapped, it is empty
				uart_resp_int("GRB", (unsigned int)(REC_BUFF_LEN));

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

			}

*/

//--------------------------------------------------------------------
// UART Commands
//--------------------------------------------------------------------
bool command_ready = false;
char cmd_str[CMD_STR_LEN];

// Add digital controller state to the queue
static void cmd_qd(const char *arg)
{
	//add_queue_digital(arg);
}

// Respond with identification
static void cmd_id(const char *arg)
{
	uart_puts(uart0, "+2wiCC\r\n");
}

// Respond with firmware version
static void cmd_ver(const char *arg)
{
	uart_puts(uart0, "+VER 1.0\r\n");
}

// Get USB connection status
static void cmd_gcs(const char *arg)
{
	if (usb_connected)
		uart_puts(uart0, "+GCS 1\r\n");
	else
		uart_puts(uart0, "+GCS 0\r\n");
}

// Echo the sent argument (loopback testing)
static void cmd_echo(const char *arg)
{
	uart_puts(uart0, "+ECHO ");
	uart_puts(uart0, arg);
}

static const command_t commands[] = {
	// Make sure the trailing space is present.
	{"ID ", cmd_id},
	{"VER ", cmd_ver},
	{"GCS ", cmd_gcs},
	{"ECHO ", cmd_echo},
};

// Each time a character is received, process it.
void on_uart_rx()
{
	static int cmd_state = C_IDLE;
	static char cmd_buf[CMD_STR_LEN]; // incoming command string
	static uint8_t incoming_idx = 0;  // index into incoming string

	while (uart_is_readable(uart0))
	{
		uint8_t ch = uart_getc(uart0);

		// hard force new action on command character
		if (ch == CMD_CHAR)
		{
			// reset command string
			incoming_idx = 0;
		}
		// parse the command on newline
		else if ((ch == '\r') || (ch == '\n'))
		{
			if (incoming_idx > 1)
			{
				// Copy to the command string
				strncpy(cmd_str, cmd_buf, incoming_idx);
				// Ensure null termination
				cmd_str[incoming_idx + 1] = '\0';
			}
			incoming_idx = 0;
			// Flag the parser
			command_ready = true;
		}
		else
		{
			// add chars to the string
			if (incoming_idx < (CMD_STR_LEN - 1)) // Never fill the final null terminator
			{
				cmd_buf[incoming_idx++] = ch;
				incoming_idx++;
			}
		}
	}
}

void core1_task()
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

	while (1)
	{
		// Handle incoming UART comms
		if (command_ready)
		{
			// Check against the list of commands
			for (uint8_t i = 0; i < ARRAY_SIZE(commands); i++)
			{
				if (strncmp(cmd_str, commands[i].name, strlen(commands[i].name)) == 0)
				{
					const char *arg = cmd_str + strlen(commands[i].name);
					commands[i].fn(arg);
					break;
				}
			}
			memset(cmd_str, 0, CMD_STR_LEN);
			command_ready = false;
		}
		sleep_ms(1);
	}
}

int main()
{
	// Start second core (handles comms)
	multicore_launch_core1(core1_task);

	// Set up debug neopixel
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &ws2812_program);
	ws2812_program_init(pio, 0, offset, 16, 800000, IS_RGBW);
	debug_pixel(urgb_u32(0, 0, 1));

	add_alarm_in_ms(2000, alarm_callback, NULL, false);

	// Set up USB
	tusb_init();

	current_controller_data = &con_buffer[conbuf_tail];
	set_neutral_con(&con_buffer[conbuf_tail]);

	while (true)
	{
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
	usb_connected = false;
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
	if (special_report_pending)
	{
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
// USB HID
//--------------------------------------------------------------------

void parse_usb(uint8_t const *current_usb_buf, uint16_t len)
{

	uint8_t cmd = current_usb_buf[0];

	switch (cmd)
	{
	case 0x01:
		output_report_0x01(current_usb_buf, usb_special_buf);
		special_report_pending = true;
		break;
	case 0x10:
		output_report_0x10(current_usb_buf, usb_special_buf);
		//        special_report_pending = true;
		break;

	case 0x80:
		output_report_0x80(current_usb_buf, usb_special_buf);
		special_report_pending = true;
		break;

	case 0x30:
		// default:
		input_report_0x30(current_usb_buf, usb_norm_buf);
		special_report_pending = false;
		usb_connected = true;
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

		if (current_time - last_report_time > 250)
		{
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

		if (!special_report_pending)
		{
			if (polling_mode == 0x30)
			{ // 0x30 is full polling mode
				// Generate a normal report
				input_report_0x30(0, usb_norm_buf);
				tud_hid_report(usb_norm_buf[0], &usb_norm_buf[1], 0x3F);
				last_report_time = current_time;
				//                printf("n");
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
