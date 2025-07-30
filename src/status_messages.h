// status_messages.h
#ifndef STATUS_MESSAGES_H
#define STATUS_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

// Message types
typedef enum
{
	MSG_USB_RUMBLE = 0x10,
	MSG_USB_LIGHTS = 0x30,
	MSG_USB_HOME_LIGHT = 0x38,
	MSG_USB_IMU_ENABLE = 0x40,
	MSG_USB_IMU_SENSITIVITY = 0x41,
	MSG_USB_VIBRATION = 0x48,
	MSG_USB_REPORT_MODE = 0x03,
	MSG_USB_DEVICE_INFO = 0x02,
} status_msg_type_t;

// Function declarations
void status_msg_send(status_msg_type_t type);
void status_msg_send_with_data(status_msg_type_t type, uint32_t data);
void status_msg_process_queue(void);

#endif // STATUS_MESSAGES_H