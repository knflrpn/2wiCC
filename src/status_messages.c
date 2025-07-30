// status_messages.c
#include "status_messages.h"
#include "pico/multicore.h"
#include "2wiCC.h" // For UART functions
#include <stdio.h>

static volatile uint32_t rumble_data = 0;

void status_msg_send(status_msg_type_t type)
{
	// Send just the message type (upper 8 bits = type, lower 24 bits = 0)
	uint32_t msg = ((uint32_t)type << 24);
	multicore_fifo_push_timeout_us(msg, 1);
}

void status_msg_send_with_data(status_msg_type_t type, uint32_t data)
{
	// Send message type with data (upper 8 bits = type, lower 24 bits = data)
	uint32_t msg = ((uint32_t)type << 24) | (data & 0xFFFFFF);
	multicore_fifo_push_timeout_us(msg, 1);
}

void status_msg_process_queue(void)
{
	char msgstr[32];

	// Process all available messages
	while (multicore_fifo_rvalid())
	{
		uint32_t msg = multicore_fifo_pop_blocking();
		status_msg_type_t type = (status_msg_type_t)(msg >> 24);
		uint32_t data = msg & 0xFFFFFF;

		switch (type)
		{
		case MSG_USB_RUMBLE:
			if (data != rumble_data) {
				rumble_data = data;
				sprintf(msgstr, "+RMBL %06X\r\n", data);
				uart_puts(uart0, msgstr);
			}
			break;

		case MSG_USB_LIGHTS:
			sprintf(msgstr, "+LGHT %02X\r\n", data);
			uart_puts(uart0, msgstr);
			break;
		}
	}
}