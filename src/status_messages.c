// status_messages.c
#include "status_messages.h"
#include "pico/util/queue.h"
#include "2wiCC.h" // For UART functions
#include <stdio.h>

// Queue for status messages
static queue_t status_queue;
static volatile uint32_t rumble_data = 0;

void status_msg_init(void)
{
	// Initialize queue to hold status_msg_t structures
	// 16 messages should be plenty for most use cases
	queue_init(&status_queue, sizeof(status_msg_t), 64);
}

void status_msg_send(status_msg_type_t type)
{
	status_msg_t msg = {
		.type = type,
		.data = 0
	};
	
	// Non-blocking add - if queue is full, message is dropped
	queue_try_add(&status_queue, &msg);
}

void status_msg_send_with_data(status_msg_type_t type, uint32_t data)
{
	status_msg_t msg = {
		.type = type,
		.data = data & 0xFFFFFF  // Preserve original 24-bit data limit
	};

	// Non-blocking add - if queue is full, message is dropped
	queue_try_add(&status_queue, &msg);
}

void status_msg_process_queue(void)
{
	char msgstr[32];
	status_msg_t msg;

	// Process all available messages
	while (queue_try_remove(&status_queue, &msg))
	{
		switch (msg.type)
		{
		case MSG_USB_RUMBLE:
			if (msg.data != rumble_data) {
				rumble_data = msg.data;
				sprintf(msgstr, "+RMBL %06X\r\n", msg.data);
				uart_puts(uart0, msgstr);
			}
			break;

		case MSG_USB_LIGHTS:
			sprintf(msgstr, "+LGHT %02X\r\n", msg.data);
			uart_puts(uart0, msgstr);
			break;
		}
	}
}