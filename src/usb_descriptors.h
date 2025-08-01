/*
 * The MIT License (MIT)
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
 */

#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

#include <stdint.h>

extern char shared_buf[0x40];

#define TUD_HID_REPORT_DESC_PROCON(...)                                                              \
	HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                                                          \
		HID_LOGICAL_MIN(0),                                                                          \
		HID_USAGE(HID_USAGE_DESKTOP_JOYSTICK),                                                       \
		HID_COLLECTION(HID_COLLECTION_APPLICATION), /* report ID 0x30 */                             \
		0x85, 0x30,                                                                                  \
		HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                                                      \
		HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),                                                       \
		HID_USAGE_MIN(1),                                                                            \
		HID_USAGE_MAX(10),                                                                           \
		HID_LOGICAL_MIN(0),                                                                          \
		HID_LOGICAL_MAX(1),                                                                          \
		HID_REPORT_SIZE(1),                                                                          \
		HID_REPORT_COUNT(10),                                                                        \
		HID_UNIT_EXPONENT(0),                                                                        \
		HID_UNIT(0),                                                                                 \
		HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                           \
		HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),                                                       \
		HID_USAGE_MIN(11),                                                                           \
		HID_USAGE_MAX(14),                                                                           \
		HID_LOGICAL_MIN(0),                                                                          \
		HID_LOGICAL_MAX(1),                                                                          \
		HID_REPORT_SIZE(1),                                                                          \
		HID_REPORT_COUNT(4),                                                                         \
		HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                           \
		HID_REPORT_SIZE(1),                                                                          \
		HID_REPORT_COUNT(2),                                                                         \
		HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE), /* Usage */                           \
		0x0B, 0x01, 0x00, 0x01, 0x00,                                                                \
		HID_COLLECTION(HID_COLLECTION_PHYSICAL), /* Usage */                                         \
		0x0B, 0x30, 0x00, 0x01, 0x00,                                                                \
		0x0B, 0x31, 0x00, 0x01, 0x00,                                                                \
		0x0B, 0x32, 0x00, 0x01, 0x00,                                                                \
		0x0B, 0x35, 0x00, 0x01, 0x00,                                                                \
		HID_LOGICAL_MIN(0), /* Logical max 65534 */                                                  \
		0x27, 0xFF, 0xFF, 0x00, 0x00,                                                                \
		HID_REPORT_SIZE(16),                                                                         \
		HID_REPORT_COUNT(4),                                                                         \
		HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                           \
		HID_COLLECTION_END, /* Usage */                                                              \
		0x0B, 0x39, 0x00, 0x01, 0x00,                                                                \
		HID_LOGICAL_MIN(0),                                                                          \
		HID_LOGICAL_MAX(7),                                                                          \
		HID_PHYSICAL_MIN(0), /* Physical Maximum (315) */                                            \
		0x46, 0x3B, 0x01,                                                                            \
		HID_UNIT(0x14),                                                                              \
		HID_REPORT_SIZE(4),                                                                          \
		HID_REPORT_COUNT(1),                                                                         \
		HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                           \
		HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),                                                       \
		HID_USAGE_MIN(15),                                                                           \
		HID_USAGE_MAX(18),                                                                           \
		HID_LOGICAL_MIN(0),                                                                          \
		HID_LOGICAL_MAX(1),                                                                          \
		HID_REPORT_SIZE(1),                                                                          \
		HID_REPORT_COUNT(4),                                                                         \
		HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),                                           \
		HID_REPORT_SIZE(8),                                                                          \
		HID_REPORT_COUNT(52),                                                                        \
		HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE),                                       \
		HID_USAGE_PAGE_N(0xFF00, 2),                                                                 \
		0x85, 0x21,                                                 /* Report IN 0x21 */             \
		HID_USAGE(0x01),                                                                             \
		HID_REPORT_SIZE(8),                                                                          \
		HID_REPORT_COUNT(63),                                                                        \
		HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE),                                       \
		0x85, 0x81,                                                 /* Report IN 0x81 */             \
		HID_USAGE(0x02),                                                                             \
		HID_REPORT_SIZE(8),                                                                          \
		HID_REPORT_COUNT(63),                                                                        \
		HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE),                                       \
		0x85, 0x01,                                                 /* Report OUT 0x01 */            \
		HID_USAGE(0x03),                                                                             \
		HID_REPORT_SIZE(8),                                                                          \
		HID_REPORT_COUNT(63),                                                                        \
		HID_OUTPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE | HID_VOLATILE),                       \
		0x85, 0x10,                                                 /* Report OUT 0x10 */            \
		HID_USAGE(0x04),                                                                             \
		HID_REPORT_SIZE(8),                                                                          \
		HID_REPORT_COUNT(63),                                                                        \
		HID_OUTPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE | HID_VOLATILE),                       \
		0x85, 0x80,                                                 /* Report OUT 0x80 */            \
		HID_USAGE(0x05),                                                                             \
		HID_REPORT_SIZE(8),                                                                          \
		HID_REPORT_COUNT(63),                                                                        \
		HID_OUTPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE | HID_VOLATILE),                       \
		0x85, 0x82,                                                 /* Report OUT 0x82 */            \
		HID_USAGE(0x06),                                                                             \
		HID_REPORT_SIZE(8),                                                                          \
		HID_REPORT_COUNT(63),                                                                        \
		HID_OUTPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE | HID_VOLATILE),                       \
		HID_COLLECTION_END

enum
{
	ENDPOINT_HID_IN = 0x81,
	ENDPOINT_HID_OUT = 0x01,
	ENDPOINT_CDC_COMM_IN = 0x83,
	ENDPOINT_CDC_DATA_IN = 0x82,
	ENDPOINT_CDC_DATA_OUT = 0x02
};

#define kMaxInputReportSizeBytes 64

#define kSubCmdOffset 10

#define ACK 0x80
#define NACK 0x00

enum voltage_level
{
	voltage_level_critical = 0x528,
	voltage_level_low = 0x5C0,
	voltage_level_medium = 0x600,
	voltage_level_full = 0x690,
};
enum battery_level
{
	battery_level_charging = 0x1,
	battery_level_empty = 0x0,
	battery_level_critical = 0x2,
	battery_level_low = 0x04,
	battery_level_medium = 0x6,
	battery_level_full = 0x8,
};

enum joycon_connection
{
	joycon_connection_bt = 0x03 << 1,
	joycon_connection_usb = 0,
};

// Sub-types of the 0x80 output report, used for initialization.
static const uint8_t kSubTypeRequestMac = 0x01;
static const uint8_t kSubTypeHandshake = 0x02;
static const uint8_t kSubTypeBaudRate = 0x03;
static const uint8_t kSubTypeDisableUsbTimeout = 0x04;
static const uint8_t kSubTypeEnableUsbTimeout = 0x05;

// UART subcommands.
static const uint8_t kSubCommandSetInputReportMode = 0x03;
static const uint8_t kSubCommandReadSpi = 0x10;
static const uint8_t kSubCommandSetPlayerLights = 0x30;
static const uint8_t kSubCommand33 = 0x33;
static const uint8_t kSubCommandSetHomeLight = 0x38;
static const uint8_t kSubCommandEnableImu = 0x40;
static const uint8_t kSubCommandSetImuSensitivity = 0x41;
static const uint8_t kSubCommandEnableVibration = 0x48;

// SPI memory regions.
static const uint16_t kSpiImuCalibrationAddress = 0x6020;
static const uint32_t kSpiImuCalibrationSize = 24;
static const uint16_t kSpiAnalogStickCalibrationAddress = 0x603d;
static const uint32_t kSpiAnalogStickCalibrationSize = 18;
static const uint16_t kSpiImuHorizontalOffsetsAddress = 0x6080;
static const uint32_t kSpiImuHorizontalOffsetsSize = 6;
static const uint16_t kSpiAnalogStickParametersAddress = 0x6086;
static const uint32_t kSpiAnalogStickParametersSize = 18;

// Byte index for the first byte of subcommand data in 0x80 output reports.
static const uint32_t kSubCommandDataOffset = 11;
// Byte index for the first byte of SPI data in SPI read responses.
#define kSpiDataOffset 20

// Values for the |device_type| field reported in the MAC reply.
static const uint8_t kUsbDeviceTypeChargingGripNoDevice = 0x00;
static const uint8_t kUsbDeviceTypeChargingGripJoyConL = 0x01;
static const uint8_t kUsbDeviceTypeChargingGripJoyConR = 0x02;
static const uint8_t kUsbDeviceTypeProController = 0x03;

static const uint32_t kMaxRetryCount = 3;

static const uint32_t kMaxVibrationEffectDurationMillis = 100;

// Initialization parameters.
static const uint8_t kGyroSensitivity2000Dps = 0x03;
static const uint8_t kAccelerometerSensitivity8G = 0x00;
static const uint8_t kGyroPerformance208Hz = 0x01;
static const uint8_t kAccelerometerFilterBandwidth100Hz = 0x01;
static const uint8_t kPlayerLightPattern1 = 0x01;

// Parameters for the "strong" and "weak" components of the dual-rumble effect.
static const double kVibrationFrequencyStrongRumble = 141.0;
static const double kVibrationFrequencyWeakRumble = 182.0;
static const double kVibrationAmplitudeStrongRumbleMax = 0.9;
static const double kVibrationAmplitudeWeakRumbleMax = 0.1;

static const int kVibrationFrequencyHzMin = 41;
static const int kVibrationFrequencyHzMax = 1253;
static const int kVibrationAmplitudeMax = 1000;

#pragma pack(1)
typedef struct ControllerDigital
{
	// Note: list here is from lsb to msb within byte, first byte to last byte
	// btn right - 8bit
	uint8_t button_y : 1;
	uint8_t button_x : 1;
	uint8_t button_b : 1;
	uint8_t button_a : 1;
	uint8_t button_right_sr : 1;
	uint8_t button_right_sl : 1;
	uint8_t button_r : 1;
	uint8_t button_zr : 1;
	// btn common - 8bit
	uint8_t button_minus : 1;
	uint8_t button_plus : 1;
	uint8_t button_thumb_r : 1;
	uint8_t button_thumb_l : 1;
	uint8_t button_home : 1;
	uint8_t button_capture : 1;
	uint8_t dummy : 1;
	uint8_t charging_grip : 1;
	// btn left - 8 bit
	uint8_t dpad_down : 1;
	uint8_t dpad_up : 1;
	uint8_t dpad_right : 1;
	uint8_t dpad_left : 1;
	uint8_t button_left_sr : 1;
	uint8_t button_left_sl : 1;
	uint8_t button_l : 1;
	uint8_t button_zl : 1;
} ControllerDigital_t;

#pragma pack(1)
typedef struct ControllerAnalog
{
	// analog - 48
	uint8_t analog[6];
} ControllerAnalog_t;

#pragma pack(1)
typedef struct ControllerData
{
	uint8_t timestamp;
	uint8_t connection_info : 4;
	uint8_t battery_level : 4;
	struct ControllerDigital digital; // 3 bytes
	struct ControllerAnalog analog;  // 6 bytes
	uint8_t rumble_input_report;
} ControllerData_t;

// In standard full input report mode, controller data is reported with IMU data
// in reports with ID 0x30.
#pragma pack(1)
typedef struct ControllerDataReport
{
	struct ControllerData controller_data; // 12 bytes
	uint8_t imu_data[36];
	uint8_t padding[kMaxInputReportSizeBytes - 49 /* 36 + 12 +1(reportid ?) */];
} ControllerDataReport_t;

// Responses to SPI read requests are sent in reports with ID 0x21. These
// reports also include controller data.
#pragma pack(1)
struct SpiReadReport
{
	struct ControllerData controller_data; // 12 bytes
	uint8_t subcommand_ack;				   // 0x90
	uint8_t subcommand;					   // 0x10
	/*
	uint8_t addrl;
	uint8_t addrh;
	uint8_t padding[2]; // 0x00 0x00
	*/
	uint32_t addr;
	uint8_t length;
	uint8_t spi_data[kMaxInputReportSizeBytes - kSpiDataOffset];
};

#pragma pack(1)
struct ResponseX81
{
	struct ControllerData controller_data; // 12 bytes
	uint8_t subcommand_ack;				   // 0x90
	uint8_t subcommand;					   // 0x10
	uint8_t data[kMaxInputReportSizeBytes - 16];
};

#pragma pack(1)
struct UsbInputReport81
{
	uint8_t subtype;
	uint8_t data[kMaxInputReportSizeBytes - 2];
};

#pragma pack(1)
struct MacAddressReport
{
	uint8_t subtype; // 0x01
	uint8_t padding;
	uint8_t device_type;
	uint8_t mac_data[6];
	uint8_t padding2[kMaxInputReportSizeBytes - 10];
};

#pragma pack(1)
struct brcm_hdr
{
	uint8_t cmd;
	uint8_t timer;
	uint8_t rumble_l[4];
	uint8_t rumble_r[4];
};

#pragma pack(1)
struct Report81Response
{
	struct ControllerData controller_data; // 12 bytes
	uint8_t subcommand_ack;				   // 0x90
	uint8_t subcommand;					   // 0x10
	union
	{
		struct
		{
			uint8_t arg0;
			uint8_t arg1;
		} cmd_args;
		struct
		{
			uint16_t voltage;
		} cmd_0x50;

		struct
		{
			uint16_t firmware_version;
			uint8_t device_type;
			uint8_t unk_0;
			uint8_t mac[6];
			uint8_t unk_1;
			uint8_t use_spi_colors;
		} cmd_0x02;

		struct
		{
			uint16_t unk0;
			uint8_t unk1;
		} cmd_0x00;

		struct
		{
			// uint8_t length;
			uint16_t address;
			uint8_t spi_data[0x1D];
		} cmd_0x10;

		uint8_t data[kMaxInputReportSizeBytes - 16];
	};
};

#pragma pack(1)
struct subcommand
{
	uint8_t report_counter;
	uint8_t unk[0x09];
	uint8_t subcommand;

	union
	{
		struct
		{
			uint16_t addr;
			uint8_t len;
		} spi_read;

		struct
		{
			uint8_t type;
			uint8_t host_bt_addr[0x06];
		} pairing;
	};
};

#pragma pack(1)
struct brcm_cmd_01
{
	uint8_t subcmd;
	union
	{
		struct
		{
			uint32_t offset;
			uint8_t size;
		} spi_data;

		struct
		{
			uint8_t arg1;
			uint8_t arg2;
		} subcmd_arg;

		struct
		{
			uint8_t mcu_cmd;
			uint8_t mcu_subcmd;
			uint8_t mcu_mode;
		} subcmd_21_21;

		struct
		{
			uint8_t mcu_cmd;
			uint8_t mcu_subcmd;
			uint8_t no_of_reg;
			uint16_t reg1_addr;
			uint8_t reg1_val;
			uint16_t reg2_addr;
			uint8_t reg2_val;
			uint16_t reg3_addr;
			uint8_t reg3_val;
			uint16_t reg4_addr;
			uint8_t reg4_val;
			uint16_t reg5_addr;
			uint8_t reg5_val;
			uint16_t reg6_addr;
			uint8_t reg6_val;
			uint16_t reg7_addr;
			uint8_t reg7_val;
			uint16_t reg8_addr;
			uint8_t reg8_val;
			uint16_t reg9_addr;
			uint8_t reg9_val;
		} subcmd_21_23_04;

		struct
		{
			uint8_t mcu_cmd;
			uint8_t mcu_subcmd;
			uint8_t mcu_ir_mode;
			uint8_t no_of_frags;
			uint16_t mcu_major_v;
			uint16_t mcu_minor_v;
		} subcmd_21_23_01;
	};
};

#endif /* USB_DESCRIPTORS_H_ */
