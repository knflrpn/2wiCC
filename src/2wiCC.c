#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "2wiCC.h"

#include "tusb.h"
#include "usb_descriptors.h"
#include "procon_functions.h"
#include "procon_data.h"
#include "status_messages.h"

// Watchdog
#include "hardware/watchdog.h"
#define WATCHDOG_TIMEOUT_MS 500

ControllerDataReport_t con_report;
ControllerData_t *con_data = &con_report.controller_data;
ControllerDigital_t digital_buffer[CON_BUF_SIZE];
ControllerAnalog_t analog_buffer[CON_BUF_SIZE];
uint16_t conbuf_head = 0, conbuf_tail = 0;
uint8_t imu_pose_data[12]; // Single IMU sample: 6 bytes accel + 6 bytes gyro

uint8_t play_mode = A_RT;

#define FRAME_TIME_US 16666 // 60 FPS
static uint32_t next_frame_time = 0;
static bool vsync_en = false;
static uint32_t frame_delay_us = 10000; // delay from vsync to con state update

static bool usb_connected = false;
static bool led_on = true;
static bool send_rumble = true;

ControllerDigital_t rec_digital_buff[REC_BUFF_SIZE];
ControllerAnalog_t rec_analog_buff[REC_BUFF_SIZE];
uint8_t rec_rle_buff[REC_BUFF_SIZE]; // run length encoding buffer
uint16_t rec_head = 0;
uint16_t stream_head = 0;
bool recording = false;
bool recording_wrap = false;

//--------------------------------------------------------------------
// Function forward declarations
//--------------------------------------------------------------------

// Utility functions
uint8_t hex2byte(const char *ch);
void uart_resp_int(const char *header, uint16_t msg);
static bool safe_memcpy(void *dst, const void *src, size_t len)
{
	if (!dst || !src)
		return false;
	memcpy(dst, src, len);
	return true;
}

// UART command handlers
static void cmd_queuedigital(const char *arg);
static void cmd_queuefull(const char *arg);
static void cmd_id(const char *arg);
static void cmd_ver(const char *arg);
static void cmd_getconnectionstatus(const char *arg);
static void cmd_setplaymode(const char *arg);
static void cmd_getqueueremaining(const char *arg);
static void cmd_getqueuesize(const char *arg);
static void cmd_echo(const char *arg);
static void cmd_vsync_en(const char *arg);
static void cmd_set_frame_delay(const char *cstr);
static void cmd_recording(const char *arg);
static void cmd_getrecordingfullness(const char *arg);
static void cmd_getrecordingremaining(const char *arg);
static void cmd_getrecordingbuffersize(const char *arg);
static void cmd_getrecording(const char *arg);

// UART communication
static void on_uart_rx(void);

// Timer and interrupt handlers
static void alarm_irq(void);
static uint32_t alarm_in_us(uint32_t delay_us);
static void alarm_at_us(uint32_t delay_us);
void gpio_callback(uint gpio, uint32_t events);

// Core tasks
void core1_task(void);
int main(void);

// USB HID functions
void parse_usb(uint8_t const *current_usb_buf, uint16_t len);
void hid_task(void);
void update_imu_data_in_report(void);

// Recording functions
static bool are_controllers_equal(const ControllerDigital_t *dig1, const ControllerAnalog_t *ana1,
								  const ControllerDigital_t *dig2, const ControllerAnalog_t *ana2);
static void send_recording_entry(uint16_t index);
static void update_recording(void);

void update_rumble_state(uint8_t const *usb_buf)
{
	// usb_buf layout:
	//   [0]           = timing
	//   [1..4]        = left  rumble block
	//   [5..8]        = right rumble block
	// Within each 4‑byte block (at usb_buf[base..base+3]):
	//   [base+1]      = HF_amp (mask off lsb)
	//   [base+3]      = LF_amp (mask off top two bits)
	// The two words seem always identical for procon except when neutral,
	// in which case they swap between left and right depending on the
	// timing byte with the other word being all 0.

	static uint8_t last_rumble = 0;

	const uint16_t hf_amp = usb_buf[2] >> 1;

	// Also going to reduce precision since it's not needed
	const uint8_t lf_amp = (usb_buf[4] & 0x3F) >> 1;

	// Pack HF in high byte, LF in low byte
	uint32_t rumble = ((hf_amp & 0xFF) << 8) | (lf_amp & 0xFF);

	// Only send if LF has changed.  HF is too chaotic to include.
	if (lf_amp != last_rumble)
	{
		if (send_rumble)
			status_msg_send_with_data(MSG_USB_RUMBLE, rumble);
		last_rumble = lf_amp;
	}
}

//--------------------------------------------------------------------
// UART Commands
//--------------------------------------------------------------------
typedef struct
{
	char commands[CMD_RING_BUFFER_SIZE][CMD_STR_LEN];
	volatile uint8_t head;
	volatile uint8_t tail;
} cmd_ring_buffer_t;
static cmd_ring_buffer_t cmd_buffer = {0};

static bool cmd_buffer_is_full(void)
{
	// Buffer is full when advancing head would equal tail
	return ((cmd_buffer.head + 1) % CMD_RING_BUFFER_SIZE) == cmd_buffer.tail;
}

static bool cmd_buffer_is_empty(void)
{
	// Buffer is empty when head equals tail
	return cmd_buffer.head == cmd_buffer.tail;
}

static bool cmd_buffer_push(const char *cmd)
{
	if (cmd_buffer_is_full())
	{
		return false; // Buffer full, command dropped
	}

	// Copy command to the head position
	strncpy(cmd_buffer.commands[cmd_buffer.head], cmd, CMD_STR_LEN - 1);
	cmd_buffer.commands[cmd_buffer.head][CMD_STR_LEN - 1] = '\0'; // Ensure null termination

	// Advance head (no count update needed)
	cmd_buffer.head = (cmd_buffer.head + 1) % CMD_RING_BUFFER_SIZE;

	return true; // Command successfully added
}

static bool cmd_buffer_pop(char *cmd_out)
{
	if (cmd_buffer_is_empty())
	{
		return false; // No commands available
	}

	// Copy command from tail position
	strcpy(cmd_out, cmd_buffer.commands[cmd_buffer.tail]);

	// Advance tail (no count update needed)
	cmd_buffer.tail = (cmd_buffer.tail + 1) % CMD_RING_BUFFER_SIZE;

	return true; // Command successfully retrieved
}

// Convert 2 hex characters into a byte.
uint8_t hex2byte(const char *ch)
{
	int val = 0, tval = 0;
	// Convert hex digits to number
	for (uint8_t i = 0; i < 2; i++)
	{
		// Enhanced validation
		if (!isxdigit(ch[i]))
		{
			return 0;
		}

		val = val << 4;		// Shift to next nibble
		tval = ch[i] - '0'; // Convert ascii number to its value
		if (tval > 9)
			tval -= ('A' - '0' - 10); // Convert A-F
		val += tval;
	}
	return val;
}

/* Respond with a 16-bit integer encoded in hex, starting with + and a header, ending with newline.
 */
void uart_resp_int(const char *header, uint16_t msg)
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
static void cmd_queuedigital(const char *arg)
{
	if (!arg || strlen(arg) < 6)
		return; // Not enough data

	// Calculate what the new head position would be
	uint16_t new_head = (conbuf_head + 1) % CON_BUF_SIZE;

	// Check if advancing the head would overwrite the tail
	if (new_head == conbuf_tail)
	{
		// Buffer is full - cannot add new data without overwriting
		return;
	}

	// Three bytes of button data
	uint8_t *asbytes = (uint8_t *)&digital_buffer[conbuf_head];
	asbytes[0] = hex2byte(arg + 0);
	asbytes[1] = hex2byte(arg + 2);
	asbytes[2] = hex2byte(arg + 4);

	// No analog data, so make it neutral
	set_neutral_analog(&analog_buffer[conbuf_head]);

	// Move the head
	conbuf_head = new_head;
}

// Add full controller state to the queue
static void cmd_queuefull(const char *arg)
{
	if (!arg || strlen(arg) < 18)
		return; // Not enough data

	// Calculate what the new head position would be
	uint16_t new_head = (conbuf_head + 1) % CON_BUF_SIZE;

	// Check if advancing the head would overwrite the tail
	if (new_head == conbuf_tail)
	{
		// Buffer is full - cannot add new data without overwriting
		return;
	}

	// Three bytes of button data
	uint8_t *dig_asbytes = (uint8_t *)&digital_buffer[conbuf_head];
	dig_asbytes[0] = hex2byte(arg + 0);
	dig_asbytes[1] = hex2byte(arg + 2);
	dig_asbytes[2] = hex2byte(arg + 4);
	// Six bytes of stick data
	uint8_t *analog_asbytes = (uint8_t *)&analog_buffer[conbuf_head];
	analog_asbytes[0] = hex2byte(arg + 6);
	analog_asbytes[1] = hex2byte(arg + 8);
	analog_asbytes[2] = hex2byte(arg + 10);
	analog_asbytes[3] = hex2byte(arg + 12);
	analog_asbytes[4] = hex2byte(arg + 14);
	analog_asbytes[5] = hex2byte(arg + 16);
	// Move the head
	conbuf_head = new_head;
}

// Respond with identification
static void cmd_id(const char *arg)
{
	uart_puts(uart0, "+2wiCC\r\n");
}

// Respond with firmware version
static void cmd_ver(const char *arg)
{
	uart_puts(uart0, "+VER ");
	uart_puts(uart0, VERSION_NUMBER);
	uart_puts(uart0, "\r\n");
}

// Get USB connection status
static void cmd_getconnectionstatus(const char *arg)
{
	if (usb_connected)
		uart_puts(uart0, "+GCS 1\r\n");
	else
		uart_puts(uart0, "+GCS 0\r\n");
}

static void cmd_setplaymode(const char *arg)
{
	if (strncmp(arg, "RT", 2) == 0)
	{
		play_mode = A_RT;
	}
	if (strncmp(arg, "BUF", 3) == 0)
	{
		play_mode = A_BUF;
	}
}

// Get the queue space remaining
static void cmd_getqueueremaining(const char *arg)
{
	uint16_t free_amt = (conbuf_tail + CON_BUF_SIZE - conbuf_head - 1) % CON_BUF_SIZE;
	uart_resp_int("GQR", free_amt);
}

// Get the size of the queue
static void cmd_getqueuesize(const char *arg)
{
	uart_resp_int("GQS", CON_BUF_SIZE);
}

// Echo the sent argument (loopback testing)
static void cmd_echo(const char *arg)
{
	uart_puts(uart0, "+ECHO ");
	uart_puts(uart0, arg);
}

// Enable or disable vsync synchronization
static void cmd_vsync_en(const char *arg)
{
	if (arg[0] == '1')
	{
		vsync_en = true;
		// Set up GPIO interrupt
		gpio_set_irq_enabled_with_callback(VSYNC_IN_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
	}
	else if (arg[0] == '0')
	{
		vsync_en = false;
		// Disable GPIO interrupt
		gpio_set_irq_enabled(VSYNC_IN_PIN, GPIO_IRQ_EDGE_RISE, false);
		// Resume timer-based updates
		next_frame_time = alarm_in_us(FRAME_TIME_US);
	}
	else if (arg[0] == '?')
	{
		uart_puts(uart0, "+VSYNC ");
		uart_putc(uart0, vsync_en ? '1' : '0');
		uart_puts(uart0, "\r\n");
	}
}

static void cmd_set_frame_delay(const char *cstr)
{
	// Check for null pointer
	if (cstr == NULL)
	{
		return; // error - no parameter provided
	}

	uint32_t result = 0;

	// Parse exactly 4 hex characters
	for (int i = 0; i < 4; i++)
	{
		char c = cstr[i];

		// Check if character is valid hex digit
		if (!isxdigit(c))
		{
			return; // error - invalid hex character
		}

		// Convert character to hex value
		uint32_t digit;
		if (c >= '0' && c <= '9')
		{
			digit = c - '0';
		}
		else if (c >= 'A' && c <= 'F')
		{
			digit = c - 'A' + 10;
		}
		else if (c >= 'a' && c <= 'f')
		{
			digit = c - 'a' + 10;
		}
		else
		{
			return; // error - invalid hex character
		}

		// Shift previous result and add new digit
		result = (result << 4) | digit;
	}

	// Set the global variable
	frame_delay_us = result;
	return; // success
}

/* Start/stop recording
 */
static void cmd_recording(const char *arg)
{
	if (arg[0] == '1')
	{
		// Start recording
		rec_head = 0;
		recording_wrap = false;
		// Initialize with current controller state
		safe_memcpy(&rec_digital_buff[rec_head], &digital_buffer[conbuf_tail], sizeof(ControllerDigital_t));
		safe_memcpy(&rec_analog_buff[rec_head], &analog_buffer[conbuf_tail], sizeof(ControllerAnalog_t));
		rec_rle_buff[rec_head] = 1;
		recording = true;
		uart_puts(uart0, "+REC 1\r\n");
	}
	else
	{
		recording = false;
		uart_puts(uart0, "+REC 0\r\n");
	}
}

/* Get recording buffer remaining space
 */
static void cmd_getrecordingremaining(const char *arg)
{
	uint16_t remaining;
	if (recording_wrap)
	{
		remaining = 0;
	}
	else
	{
		remaining = REC_BUFF_SIZE - rec_head;
	}
	uart_resp_int("GRR", remaining);
}

/* Get total recording buffer size
 */
static void cmd_getrecordingbuffersize(const char *arg)
{
	uart_resp_int("GRS", REC_BUFF_SIZE);
}

/* Send recording data
 */
static void cmd_getrecording(const char *arg)
{
	if (arg[0] == '0')
	{
		// Start at beginning
		if (recording_wrap)
		{
			// If wrapped, oldest value is just in front of head
			stream_head = (rec_head + 1) % REC_BUFF_SIZE;
		}
		else
		{
			stream_head = 0;
		}
	}

	// Send up to 20 entries at a time to avoid overwhelming UART
	for (uint8_t i = 0; i < 20 && stream_head != rec_head; i++)
	{
		send_recording_entry(stream_head);
		stream_head = (stream_head + 1) % REC_BUFF_SIZE;
	}

	// Send current entry if we've reached the head
	if (stream_head == rec_head)
	{
		send_recording_entry(stream_head);
		uart_puts(uart0, "+GR 0\r\n"); // End of stream
	}
	else
	{
		uart_puts(uart0, "+GR 1\r\n"); // More data pending
	}
}

static void cmd_setled(const char *arg)
{
	led_on = arg[0] != '0';
	if (!led_on)
		debug_pixel(urgb_u32(0, 0, 0));
}

static void cmd_setrumble(const char *arg)
{
	send_rumble = arg[0] != '0';
}

/* Set IMU data - 12 bytes (6 accel + 6 gyro)
 */
static void cmd_setimu(const char *arg)
{
	if (!arg || strlen(arg) < 24)
		return; // Not enough data - need 24 hex chars for 12 bytes

	// Parse 12 bytes of IMU data (6 accel + 6 gyro)
	for (int i = 0; i < 12; i++)
	{
		imu_pose_data[i] = hex2byte(arg + (i * 2));
	}
}

/* Add full controller state including IMU to the queue
 */
static void cmd_queuefull_imu(const char *arg)
{
	if (!arg || strlen(arg) < 42)
		return; // Not enough data - need 42 hex chars total

	// Insert normal controller state.
	cmd_queuefull(arg);

	// Twelve bytes of IMU data
	for (int i = 0; i < 12; i++)
	{
		imu_pose_data[i] = hex2byte(arg + 18 + (i * 2));
	}
}

static const command_t commands[] = {
	// Make sure the trailing space is present.
	{"QF ", cmd_queuefull, 3},
	{"QD ", cmd_queuedigital, 3},
	{"QFI ", cmd_queuefull_imu, 4},
	{"SI ", cmd_setimu, 3},
	{"ID ", cmd_id, 3},
	{"VER ", cmd_ver, 4},
	{"SPM ", cmd_setplaymode, 4},
	{"GQR ", cmd_getqueueremaining, 4},
	{"GQS ", cmd_getqueuesize, 4},
	{"GCS ", cmd_getconnectionstatus, 4},
	{"VSYNC ", cmd_vsync_en, 6},
	{"VSD ", cmd_set_frame_delay, 4},
	{"REC ", cmd_recording, 4},
	{"GRR ", cmd_getrecordingremaining, 4},
	{"GRS ", cmd_getrecordingbuffersize, 4},
	{"GR ", cmd_getrecording, 3},
	{"LED ", cmd_setled, 4},
	{"RMBL ", cmd_setrumble, 5},
	{"ECHO ", cmd_echo, 5},
};

// Process each incoming character
static void on_uart_rx()
{
	static char incoming_cmd[CMD_STR_LEN]; // Buffer for building incoming command
	static uint8_t incoming_idx = 0;	   // Index into incoming string

	while (uart_is_readable(uart0))
	{
		uint8_t ch = uart_getc(uart0);

		// Hard force new action on command character
		if (ch == CMD_CHAR)
		{
			// Reset command string
			incoming_idx = 0;
		}
		// Parse the command on newline
		else if ((ch == '\r') || (ch == '\n'))
		{
			if (incoming_idx > 1)
			{
				// Null terminate the incoming command
				if (incoming_idx >= CMD_STR_LEN)
					incoming_idx = CMD_STR_LEN - 1;
				incoming_cmd[incoming_idx] = '\0';

				// Try to add command to ring buffer
				cmd_buffer_push(incoming_cmd);
			}
			incoming_idx = 0;
		}
		else
		{
			// Add chars to the string
			if (incoming_idx < (CMD_STR_LEN - 1)) // Never fill the final null terminator
			{
				incoming_cmd[incoming_idx++] = ch;
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
	// If in realtime mode, set a timeout.
	if (!vsync_en)
	{
		next_frame_time += FRAME_TIME_US;
		alarm_at_us(next_frame_time);
	}

	// Feed the watchdog
	watchdog_update();

	if (collision_count > 20)
	{
		// Ran out of buffer data.  Set neutral controller
		// and realtime mode to fall back to a harmless
		// state.
		cmd_queuedigital("000000");
		play_mode = A_RT;
	}

	if (play_mode == A_RT)
	{
		// Point to the latest state
		conbuf_tail = (conbuf_head + CON_BUF_SIZE - 1) % CON_BUF_SIZE;
		collision_count = 0;
	}
	else if (play_mode == A_BUF)
	{
		// Move the tail up if possible
		uint16_t nextpos = (conbuf_tail + 1) % CON_BUF_SIZE;
		if (nextpos != conbuf_head)
		{
			conbuf_tail = nextpos;
			collision_count = 0;
		}
		else
		{
			collision_count++;
		}
	}
	// Insert updated state into data
	insert_constate_to_condata(con_data, &digital_buffer[conbuf_tail], &analog_buffer[conbuf_tail]);
	update_imu_data_in_report();

	// If recording, insert into recording buffer
	if (recording)
		update_recording();

	heartbeat++;
	if (led_on)
	{
		uint8_t cycle_pos = heartbeat % 64;
		uint8_t intensity = 0;

		switch (cycle_pos)
		{
		case 0:
			intensity = 20;
			break;
		case 1:
			intensity = 5;
			break;
		case 11:
			intensity = 200;
			break;
		case 12:
			intensity = 50;
			break;
		case 13:
			intensity = 10;
			break;
		}

		debug_pixel(urgb_u32(intensity, 0, usb_connected * 4));
	}
}

/* Set up an alarm in the future.
 * Returns the timer value that the alarm is set to.
 */
static uint32_t alarm_in_us(uint32_t delay_us)
{
	// Enable the alarm irq
	irq_set_enabled(TIMER_IRQ_0, true);

	// Write the lower 32 bits of the target time to the alarm,
	// which will arm it
	uint32_t target = timer_hw->timerawl + delay_us;
	timer_hw->alarm[0] = (uint32_t)target;

	return target;
}

/* Set up an alarm at a specific time.
 */
static void alarm_at_us(uint32_t time_us)
{
	// Enable the alarm irq
	irq_set_enabled(TIMER_IRQ_0, true);

	// Write the lower 32 bits of the target time to the alarm, which
	// will arm it
	timer_hw->alarm[0] = time_us;
}

static volatile bool core1_initialized = false;
void core1_task()
{
	stdio_init_all();
	// Set up UART with a 2wiCC standard baud rate.
	uart_init(uart0, 460800u);
	// Configure the TX and RX pins
	gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(uart0, UART_TX_PIN));
	gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(uart0, UART_RX_PIN));

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

	// Set irq handler for alarm irq
	irq_set_exclusive_handler(TIMER_IRQ_0, alarm_irq);
	// Enable timer interrupts for the alarm
	hw_set_bits(&timer_hw->inte, 1u << 0);
	// Start timer-based controller updates
	next_frame_time = alarm_in_us(20000);
	core1_initialized = true;

	// Initialize watchdog for this core
	if (watchdog_caused_reboot())
	{
		uart_puts(uart0, "+ERR WATCHDOG\r\n");
	}
	watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);

	while (1)
	{
		// Handle incoming UART commands from ring buffer
		char current_cmd[CMD_STR_LEN];

		// Process all available commands in the buffer
		while (cmd_buffer_pop(current_cmd))
		{
			// Check against the list of commands
			bool command_found = false;
			for (uint8_t i = 0; i < ARRAY_SIZE(commands); i++)
			{
				if (strncmp(current_cmd, commands[i].name, commands[i].name_len) == 0)
				{
					const char *arg = current_cmd + commands[i].name_len;
					commands[i].fn(arg);
					command_found = true;
					break;
				}
			}
		}

		// Process status messages from core 0
		status_msg_process_queue();

		sleep_ms(5);
	}
}

void update_imu_data_in_report(void)
{
	if (imu_enabled)
	{
		// Copy the current IMU data to the report
		// The Switch expects 3 IMU samples (12 bytes each = 36 bytes total)
		for (int sample = 0; sample < 3; sample++)
		{
			safe_memcpy(&con_report.imu_data[sample * 12], imu_pose_data, 12);
		}
	}
	else
	{
		// Clear IMU data when disabled
		memset(con_report.imu_data, 0, sizeof(con_report.imu_data));
	}
}

int main()
{
	// Set up debug neopixel
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &ws2812_program);
	ws2812_program_init(pio, 0, offset, 16, 800000, IS_RGBW);
	debug_pixel(urgb_u32(16, 16, 16));

	// Initialize controller state.
	cmd_queuefull_imu("000000000880000880000000000000000000000000");
	// Initialize inter-core message queue
	status_msg_init();

	// Start second core (handles comms)
	multicore_launch_core1(core1_task);
	// Wait for core 1
	while (!core1_initialized)
		;

	// Set up USB
	tusb_init();

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
		// Note: don't set special report pending, because this report
		// doesn't send a response
		output_report_0x10(current_usb_buf, usb_special_buf);
		update_rumble_state(current_usb_buf + 1);
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
			output_mac_addr(usb_special_buf, usb_special_buf);
			special_report_pending = true;
			tud_hid_report(usb_special_buf[0], &usb_special_buf[1], 0x3F);
			last_report_time = current_time;
		}

		if (!special_report_pending)
		{
			if (polling_mode == 0x30)
			{ // 0x30 is full polling mode
				// Generate a normal report
				insert_constate_to_condata(con_data, &digital_buffer[conbuf_tail], &analog_buffer[conbuf_tail]);
				update_imu_data_in_report();
				tud_hid_report(0x30, &con_report, 0x3F);
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

/* GPIO interrupt handler
 */
void gpio_callback(uint gpio, uint32_t events)
{
	// set up an interrupt in the future to change controller data
	alarm_in_us(frame_delay_us);
}

//--------------------------------------------------------------------
// Recording helper functions
//--------------------------------------------------------------------

/* Check if two controller states are equal for RLE compression
 */
static bool are_controllers_equal(const ControllerDigital_t *dig1, const ControllerAnalog_t *ana1,
								  const ControllerDigital_t *dig2, const ControllerAnalog_t *ana2)
{
	// Compare digital data
	if (memcmp(dig1, dig2, sizeof(ControllerDigital_t)) != 0)
		return false;

	// Compare analog data
	if (memcmp(ana1, ana2, sizeof(ControllerAnalog_t)) != 0)
		return false;

	return true;
}

/* Send a single recording entry via UART
 */
static void send_recording_entry(uint16_t index)
{
	static char msgstr[5];

	// Header
	uart_puts(uart0, "+R ");

	// Digital data (3 bytes as hex)
	uint8_t *dig_bytes = (uint8_t *)&rec_digital_buff[index];
	sprintf(msgstr, "%02X", dig_bytes[0]);
	uart_puts(uart0, msgstr);
	sprintf(msgstr, "%02X", dig_bytes[1]);
	uart_puts(uart0, msgstr);
	sprintf(msgstr, "%02X", dig_bytes[2]);
	uart_puts(uart0, msgstr);

	// Analog data (6 bytes as hex)
	uint8_t *ana_bytes = (uint8_t *)&rec_analog_buff[index];
	for (int i = 0; i < 6; i++)
	{
		sprintf(msgstr, "%02X", ana_bytes[i]);
		uart_puts(uart0, msgstr);
	}

	// RLE count
	uart_puts(uart0, "x");
	sprintf(msgstr, "%02X", rec_rle_buff[index]);
	uart_puts(uart0, msgstr);

	// Termination
	uart_puts(uart0, "\r\n");
}

/* Update recording buffer - call this in alarm_irq() function
 */
static void update_recording()
{
	// Get current controller state
	ControllerDigital_t *current_dig = &digital_buffer[conbuf_tail];
	ControllerAnalog_t *current_ana = &analog_buffer[conbuf_tail];

	// Implement run-length encoding
	if ((rec_rle_buff[rec_head] < 240) &&
		are_controllers_equal(&rec_digital_buff[rec_head], &rec_analog_buff[rec_head],
							  current_dig, current_ana))
	{
		// Same state, increment RLE count
		rec_rle_buff[rec_head]++;
	}
	else
	{
		// State changed or max RLE reached, move to next buffer entry
		rec_head++;
		if (rec_head >= REC_BUFF_SIZE)
		{
			rec_head = 0;
			recording_wrap = true;
		}

		// Copy new state to record buffer
		safe_memcpy(&rec_digital_buff[rec_head], current_dig, sizeof(ControllerDigital_t));
		safe_memcpy(&rec_analog_buff[rec_head], current_ana, sizeof(ControllerAnalog_t));
		rec_rle_buff[rec_head] = 1;
	}
}