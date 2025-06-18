#include "usb_descriptors.h"
#include "procon_functions.h"

static struct ControllerData *controller_state;
uint8_t usb_special_buf[0x40]; // Buffer for special messages
uint8_t usb_norm_buf[0x40];	   // Buffer for normal messages
uint8_t polling_mode = 0;
uint8_t imu_enabled = 0;
bool special_report_pending = false;
bool special_report_queued = false;
static uint8_t tick = 0;
uint32_t last_report_time = 0;

void set_neutral_con()
{
	controller_state->analog[0] = 0xFF;
	controller_state->analog[1] = 0xF7;
	controller_state->analog[2] = 0x7F;
	controller_state->analog[3] = 0xFF;
	controller_state->analog[4] = 0xF7;
	controller_state->analog[5] = 0x7F;
	controller_state->charging_grip = 1;
}

/* see https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/spi_flash_notes.md */

/* Gets a value from the controller's non-volatile memory */
void spi_read(uint16_t addr, uint8_t len, uint8_t *buffer)
{
	uint8_t *data = NULL;
	switch (addr & 0xF000)
	{
	case 0x6000:
		// Factory Configuration and Calibration
		data = (uint8_t *)&spi0x6000[addr & 0xFF];
		break;
	case 0x2000:
		// Pairing info
		data = (uint8_t *)&spi0x2000[addr & 0x7F];
		break;

	default:
		break;
	}

	if (data)
		memcpy(buffer, data, len);
	else
		memset(buffer, 0xFF, len);
}

void spi_write(uint16_t addr, uint8_t len, uint8_t const *buffer)
{
	// Not implemented
}

void spi_erase(uint16_t addr, uint8_t len)
{
	// Not implemented
}

/* Inserts the common controller data into the provided report */
static void fill_input_report(struct ControllerData *controller_data)
{
	memcpy(controller_data, controller_state, sizeof(struct ControllerData));

	controller_data->timestamp = tick;
	controller_data->battery_level = battery_level_charging | battery_level_full;
	controller_data->connection_info = /* 0xe; */ 0x1;
	controller_data->rumble_input_report = 0x70;

	tick += 3;
}

/* 0x30 is the full report with IMU data. */
static void input_report_0x30(uint8_t const *usb_in, uint8_t *usb_out_buf)
{
	// report ID
	usb_out_buf[0x00] = 0x30;

	fill_input_report((struct ControllerData *)&usb_out_buf[0x01]);
}

/* Used to ignore controller internal UART commands */
static void output_passthrough(uint8_t const *usb_in, uint8_t *usb_out_buf)
{
	const uint8_t response_h[] = {0x81, usb_in[1], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	memcpy(usb_out_buf, response_h, sizeof(response_h));
}

/* Response to 80 01. Sends connection status and MAC data */
static void output_mac_addr(uint8_t const *usb_in, uint8_t *usb_out_buf)
{
	memcpy(usb_out_buf, mac_response, sizeof(mac_response));
}

/* Used to ignore controller internal UART handshaking */
static void output_handshake(uint8_t const *usb_in, uint8_t *usb_out_buf)
{
	memset(usb_out_buf, 0, 0x40);
	usb_out_buf[0] = 0x81;
	usb_out_buf[1] = kSubTypeHandshake;
}

/* Used to ignore baud rate change */
static void output_baudrate(uint8_t const *usb_in, uint8_t *usb_out_buf)
{
	memset(usb_out_buf, 0, 0x40);
	usb_out_buf[0] = 0x81;
	usb_out_buf[1] = 0x03;
}

/* Used to ignore request to switch to bluetooth */
static void output_enable_usb_timeout(uint8_t const *usb_in, uint8_t *usb_out_buf)
{
	memset(usb_out_buf, 0, 0x40);
	usb_out_buf[0] = 0x81;
	usb_out_buf[1] = kSubTypeDisableUsbTimeout;
}

/* Used to ignore request to remain on USB (going to anyway) */
static void output_disable_usb_timeout(uint8_t const *usb_in, uint8_t *usb_out_buf)
{
	memset(usb_out_buf, 0, 0x40);
	usb_out_buf[0] = 0x81;
	usb_out_buf[1] = kSubTypeEnableUsbTimeout;
}

/* Check which action has been sent to the 0x80 endpoint */
static void output_report_0x80(uint8_t const *buf, uint8_t *usb_out_buf)
{
	switch (buf[1])
	{
	case 0x01: // mac addr
		output_mac_addr(buf, usb_out_buf);
		break;
		// handshake//baudrate
	case 0x02:
		output_handshake(buf, usb_out_buf);
		break;
	case 0x03:
		output_baudrate(buf, usb_out_buf);
		break;
	// usb timeout
	case 0x04:
		output_disable_usb_timeout(buf, usb_out_buf);
		break;
	case 0x05:
		output_enable_usb_timeout(buf, usb_out_buf);
		break;
	case 0x91:
	case 0x92:
		output_passthrough(buf, usb_out_buf);
		break;
	default:
		output_passthrough(buf, usb_out_buf);
		break;
	}
}

/* An unknown subcommand was sent to the 0x01 endpoint. */
static void output_report_0x01_unknown_subcmd(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct Report81Response *resp = (struct Report81Response *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;
	fill_input_report(&resp->controller_data);
	resp->subcommand_ack = 0x80;
	resp->subcommand = buf[10];
}

/* 0x01 subcommand 0x08: Set shipment low power state.
   This does nothing to this device, so just send a dummy response. */
static void output_report_0x01_0x08_lowpower_state(uint8_t const *buf, uint8_t *usb_out_buf)
{
	unsigned char rawData[64] = {
		0x21,0x06,0x8E,0x84,0x00,0x12,0x01,0x18,
		0x80,0x01,0x18,0x80,0x80,0x80,0x08,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	};
	static int iii = 0;
	rawData[0x01] = iii++;
	memcpy(usb_out_buf, rawData, sizeof(rawData));
}

/* 0x01 subcommand 0x02: Request device info. */
static void output_report_0x01_get_device_info(uint8_t const *buf, uint8_t *usb_out_buf)
{
	memcpy(usb_out_buf, device_info_response, 0x40);
	fill_input_report((struct ControllerData *)&usb_out_buf[0x01]);
}

/* 0x01 subcommand 0x03: Set input report mode.  This is expected to set mode to
   standard full mode (0x30). */
static void output_report_0x01_set_report_mode(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct Report81Response *resp = (struct Report81Response *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;
	// acknowledge
	resp->subcommand_ack = 0x80;
	resp->subcommand = 0x03;

	fill_input_report(&resp->controller_data);

	polling_mode = buf[11];
}

/* 0x01 subcommand 0x04: Trigger buttons elapsed time.  Not sure what this should do,
   so just send a canned response. */
static void output_report_0x01_trigger_elapsed(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct Report81Response *resp = (struct Report81Response *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;

	// acknowledge
	resp->subcommand_ack = 0x83;
	resp->subcommand = 0x04;

	fill_input_report(&resp->controller_data);

	const uint8_t resp_[] = {
		0x21, 0x0A, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80,
		0x83, 0x04, 0x00, 0xCC, 0x00, 0xEE, 0x00, 0xFF, 0x00, 0x00, 0x00};

	memcpy(usb_out_buf, resp_, sizeof(resp_));
}

/* 0x01 subcommand 0x10: Read from SPI flash. */
static void output_report_0x01_readspi(uint8_t const *buf, uint8_t *usb_out_buf)
{

	struct SpiReadReport *resp = (struct SpiReadReport *)&usb_out_buf[0x01];
	uint16_t addr = buf[kSubCommandDataOffset] | (uint16_t)buf[kSubCommandDataOffset + 1] << 8;
	uint8_t len = buf[kSubCommandDataOffset + 4];

	memset(usb_out_buf, 0x00, 0x40);
	usb_out_buf[0x00] = 0x21;

	fill_input_report(&resp->controller_data);

	resp->subcommand_ack = 0x90;
	resp->subcommand = 0x10;
	resp->addr = addr;

	spi_read(addr, len, resp->spi_data);
}

/* 0x01 subcommand 0x11: Write to SPI flash. This will do nothing. */
static void output_report_0x01_writespi(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct ResponseX81 *resp = (struct ResponseX81 *)&usb_out_buf[0x01];
	uint16_t addr = *(uint16_t *)(&buf[kSubCommandDataOffset]);
	uint8_t len = buf[kSubCommandDataOffset + 4];

	// report ID
	usb_out_buf[0x00] = 0x21;
	fill_input_report(&resp->controller_data);

	spi_write(addr, len, &buf[kSubCommandDataOffset + 5]);
}

/* 0x01 subcommand 0x12: Erase SPI flash. This will do nothing. */
static void output_report_0x01_erasespi(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct ResponseX81 *resp = (struct ResponseX81 *)&usb_out_buf[0x01];
	uint16_t addr = *(uint16_t *)(&buf[kSubCommandDataOffset]);
	uint8_t len = buf[kSubCommandDataOffset + 4];

	// report ID
	usb_out_buf[0x00] = 0x21;
	fill_input_report(&resp->controller_data);
	spi_erase(addr, len);
}

/* 0x01 subcommand 0x30: Set controller lights. Might use for something later.*/
static void output_report_0x01_set_lights(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct ResponseX81 *resp = (struct ResponseX81 *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;
	fill_input_report(&resp->controller_data);
}

/* 0x01 subcommand 0x38: Set home button light. Might use for something later.*/
static void output_report_0x01_set_homelight(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct ResponseX81 *resp = (struct ResponseX81 *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;
	fill_input_report(&resp->controller_data);
}

/* 0x01 subcommand 0x40: Turn on or off IMU.*/
static void output_report_0x01_enable_imu(uint8_t const *buf, uint8_t *usb_out_buf)
{
	imu_enabled = buf[11];

	struct ResponseX81 *resp = (struct ResponseX81 *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;
	fill_input_report(&resp->controller_data);
}

/* 0x01 subcommand 0x41: Set IMU sensitivity.*/
static void output_report_0x01_set_imu_sensitivity(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct ResponseX81 *resp = (struct ResponseX81 *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;
	fill_input_report(&resp->controller_data);
}

/* 0x01 subcommand 0x48: Enable vibration. This will do nothing.*/
static void output_report_0x01_set_vibration(uint8_t const *buf, uint8_t *usb_out_buf)
{
	struct ResponseX81 *resp = (struct ResponseX81 *)&usb_out_buf[0x01];
	// report ID
	usb_out_buf[0x00] = 0x21;

	resp->subcommand_ack = 0x80;
	resp->subcommand = 0x48;

	fill_input_report(&resp->controller_data);
}

/* 0x01 subcommand 0x01: Manual BT pairing. This will do nothing.*/
static void output_report_0x01_bt_pairing(uint8_t const *buf, uint8_t *usb_out_buf)
{
	uint8_t pairing_type = buf[11] /* data->pairing.type */;

	uint8_t *data = (uint8_t *)bt_data_01;
	switch (pairing_type)
	{
	case 1:
		data = (uint8_t *)bt_data_01;
		break;
	case 2:
		data = (uint8_t *)bt_data_02;
		break;
	default:
	case 3:
		data = (uint8_t *)bt_data_03;
		break;
	}
	memcpy(usb_out_buf, data, 0x40);
	fill_input_report((struct ControllerData *)&usb_out_buf[0x01]);
}

/* Endpoint 0x10 does nothing here. */
static void output_report_0x10(uint8_t const *buf, uint8_t *usb_out_buf)
{
	/** nothing **/
}

/* Reroute the various subcommands of the 0x01 endpoint. */
static void output_report_0x01(uint8_t const *buf, uint8_t *usb_out_buf)
{
	uint8_t subCmd = buf[10];

	switch (subCmd)
	{
	case 0x01:
		output_report_0x01_bt_pairing(buf, usb_out_buf);
		break;
	case 0x02: // get device info
		output_report_0x01_get_device_info(buf, usb_out_buf);
		break;
	case 0x03: // Set input report mode
		output_report_0x01_set_report_mode(buf, usb_out_buf);
		break;
	case 0x04:
		output_report_0x01_trigger_elapsed(buf, usb_out_buf);
		break;
	case 0x08: // Set low power state (nothing here)
		output_report_0x01_0x08_lowpower_state(buf, usb_out_buf);
		break;
	case 0x10: // Read SPI flash
		output_report_0x01_readspi(buf, usb_out_buf);
		break;
	case 0x11: // Write SPI flash
		output_report_0x01_writespi(buf, usb_out_buf);
		break;
	case 0x12: // Erase SPI flash
		output_report_0x01_erasespi(buf, usb_out_buf);
		break;
	case 0x30: // Set Lights
		output_report_0x01_set_lights(buf, usb_out_buf);
		break;
	case 0x38: // Set Home Light
		output_report_0x01_set_homelight(buf, usb_out_buf);
		break;
	case 0x40: // Enable Imu
		output_report_0x01_enable_imu(buf, usb_out_buf);
		break;
	case 0x41: // Set Imu Sensitivity
		output_report_0x01_set_imu_sensitivity(buf, usb_out_buf);
		break;
	case 0x48: // Enable Vibration
		output_report_0x01_set_vibration(buf, usb_out_buf);
		break;
	case 0x00:
	case 0x33:
	default:
		output_report_0x01_unknown_subcmd(buf, usb_out_buf);
		break;
	}
}
