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

ControllerData_t con_data;
ControllerDigital_t digital_buffer[CON_BUF_SIZE];
ControllerAnalog_t analog_buffer[CON_BUF_SIZE];
uint16_t conbuf_head, conbuf_tail = 0;
uint8_t global_imu[36];

uint8_t play_mode = A_RT;

static bool usb_connected = false;

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

// Convert 2 hex characters into a byte in a pretty unsafe way.
uint8_t hex2byte(const char* ch) {
	int val = 0, tval = 0;
	// Convert hex digits to number
	for (uint8_t i=0; i<2; i++) {
		val = val<<4;     // Shift to next nibble
		tval = ch[i]-'0'; // Convert ascii number to its value
		if (tval > 9) tval -= ('A'-'0'-10); // Convert A-F
		val += tval;
	}
	return val;
}

/* Respond with an integer encoded in hex, starting with + and a header, ending with newline.
 */
void uart_resp_int(const char *header, unsigned int msg)
{
    char msgstr[5];
    sprintf(msgstr, "%04X", msg);
	uart_putc(uart0, '+');
	uart_puts(uart0, header);
	uart_putc(uart0, ' ');
	uart_puts(uart0, msgstr);
	uart_putc(uart0, '\r');
	uart_putc(uart0, '\n');
}

// Add digital controller state to the queue
static void cmd_queuedigital(const char* arg) {
	// Three bytes of button data
	uint8_t* asbytes = (uint8_t*)&digital_buffer[conbuf_head];
	asbytes[0] = hex2byte(arg+0);
	asbytes[1] = hex2byte(arg+2);
	asbytes[2] = hex2byte(arg+4);
	// No analog data, so make it neutral
	set_neutral_analog(&analog_buffer[conbuf_head]);
	// Move the head
	conbuf_head = (conbuf_head+1)%CON_BUF_SIZE;
}

// Add full controller state to the queue
static void cmd_queuefull(const char* arg) {
	// Three bytes of button data
	uint8_t* dig_asbytes = (uint8_t*)&digital_buffer[conbuf_head];
	dig_asbytes[0] = hex2byte(arg+0);
	dig_asbytes[1] = hex2byte(arg+2);
	dig_asbytes[2] = hex2byte(arg+4);
	// Six bytes of stick data
	uint8_t* analog_asbytes = (uint8_t*)&analog_buffer[conbuf_head];
	analog_asbytes[0] = hex2byte(arg+6);
	analog_asbytes[1] = hex2byte(arg+8);
	analog_asbytes[2] = hex2byte(arg+10);
	analog_asbytes[3] = hex2byte(arg+12);
	analog_asbytes[4] = hex2byte(arg+14);
	analog_asbytes[5] = hex2byte(arg+16);
	// Move the head
	conbuf_head = (conbuf_head+1)%CON_BUF_SIZE;
}

// Respond with identification
static void cmd_id(const char* arg) {
	uart_puts(uart0, "+2wiCC\r\n");
}

// Respond with firmware version
static void cmd_ver(const char* arg) {
	uart_puts(uart0, "+VER 1.0\r\n");
}

// Get USB connection status
static void cmd_getconnectionstatus(const char* arg) {
	if (usb_connected)
		uart_puts(uart0, "+GCS 1\r\n");
	else
		uart_puts(uart0, "+GCS 0\r\n");
}

static void cmd_setplaymode(const char* arg) {
	if (strncmp(arg, "RT", 5) == 0) {
		play_mode = A_RT;
	}
	if (strncmp(arg, "BUF", 5) == 0) {
		play_mode = A_BUF;
	}
}

// Get the queue fullness
static void cmd_getqueueremaining(const char* arg) {
	uint16_t free_amt = (conbuf_tail + CON_BUF_SIZE - conbuf_head - 1) % CON_BUF_SIZE;
	uart_resp_int("GQF", free_amt);
}

// Get the size of the queue
static void cmd_getqueuesize(const char* arg) {
	uart_resp_int("GQS", CON_BUF_SIZE);
}

// Echo the sent argument (loopback testing)
static void cmd_echo(const char* arg) {
	uart_puts(uart0, "+ECHO ");
	uart_puts(uart0, arg);
}

static const command_t commands[] = {
	// Make sure the trailing space is present.
	{"QD ", cmd_queuedigital},
	{"QF ", cmd_queuefull},
	{"SPM ", cmd_setplaymode},
	{"GQR", cmd_getqueueremaining},
	{"GQS", cmd_getqueuesize},
	{"ID ", cmd_id},
	{"VER ", cmd_ver},
	{"GCS ", cmd_getconnectionstatus},
	{"ECHO ", cmd_echo},
};

// Each time a character is received, process it.
static void on_uart_rx()
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
				// Flag the parser
				command_ready = true;
			}
			incoming_idx = 0;
		}
		else
		{
			// add chars to the string
			if (incoming_idx < (CMD_STR_LEN - 1)) // Never fill the final null terminator
			{
				cmd_buf[incoming_idx++] = ch;
			}
		}
	}
}

/* Alarm interrupt handler.
   This happens once per game frame, and is used to update the USB data.
*/
static void alarm_irq(void)
{
	static uint16_t heartbeat = 0;
	static uint8_t collision_count = 0;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << 0);
	alarm_in_us(16666);

	if (collision_count > 5) {
		// Ran out of buffer data.  Set neutral controller
		// and realtime mode.
		play_mode = A_RT;
		cmd_queuedigital("000000");
	}

	if (play_mode == A_RT) {
		// Point to the latest state
		conbuf_tail = (conbuf_head-1)%CON_BUF_SIZE;
	}
	if (play_mode == A_BUF) {
		// Move the tail up if possible
		uint16_t nextpos = (conbuf_tail+1)%CON_BUF_SIZE;
		if (nextpos != conbuf_head) {
			conbuf_tail = nextpos;
			collision_count = 0;
		} else {
			collision_count++;
		}
	}
	// Insert updated state into data
	insert_constate_to_condata(&con_data, &digital_buffer[conbuf_tail], &analog_buffer[conbuf_tail]);

	heartbeat++;
	uint8_t hb = ((heartbeat % 64) == 0) * 4 | ((heartbeat % 64) == 11) * 32;
    debug_pixel(urgb_u32(hb, 0, usb_connected * 4));

}

/* Set up an alarm in the future.
 */
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
					const char* arg = cmd_str + strlen(commands[i].name);
					commands[i].fn(arg);
					cmd_str[0] = '\0';
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

	alarm_in_us(200000);

	// Set up USB
	tusb_init();

	current_controller_data = &con_data;
	set_neutral_analog(&analog_buffer[conbuf_tail]);
	digital_buffer[conbuf_tail].charging_grip = 1;
	conbuf_head = 1;

	while (true)
	{
		tud_task();
		hid_task();
		usb_connected = tud_connected();
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
	case 0x01: // subcommand
		output_report_0x01(current_usb_buf, usb_special_buf);
		special_report_pending = true;
		break;
	case 0x10: // rumble only
		// Doesn't do anything. Don't set special report pending, because
		// it won't get cleared, because there's no response.
		output_report_0x10(current_usb_buf, usb_special_buf);
		break;

	case 0x80: // basic HID
		output_report_0x80(current_usb_buf, usb_special_buf);
		special_report_pending = true;
		break;

	case 0x30: // normal controller state
		tud_hid_report(0x30, &con_data, 0x3F);
		special_report_pending = false;
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
				// Hi, I'm a pro controller with KNfLrPn's controller's MAC 
				0x81, 0x01, 0x00, 0x03, 0xe6, 0x91, 0x3e, 0xc9, 0xb5, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
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
				insert_constate_to_condata(&con_data, &digital_buffer[conbuf_tail], &analog_buffer[conbuf_tail]);
				tud_hid_report(0x30, &con_data, 0x3F);
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
