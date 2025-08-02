// procon_functions.h
#ifndef PROCON_FUNCTIONS_H
#define PROCON_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include <hardware/flash.h>
#include "usb_descriptors.h"

// Function declarations
void set_neutral_analog(ControllerAnalog_t *analogstate);
void insert_constate_to_condata(ControllerData_t *condata, ControllerDigital_t *condigital, ControllerAnalog_t *conanalog);
void spi_read(uint16_t addr, uint8_t len, uint8_t *buffer);
void spi_write(uint16_t addr, uint8_t len, uint8_t const *buffer);
void spi_erase(uint16_t addr, uint8_t len);
void output_report_0x01(uint8_t const *buf, uint8_t *usb_out_buf);
void output_report_0x10(uint8_t const *buf, uint8_t *usb_out_buf);
void output_report_0x80(uint8_t const *buf, uint8_t *usb_out_buf);
void output_mac_addr(uint8_t const *usb_in, uint8_t *usb_out_buf);

// External variable declarations
extern uint8_t usb_special_buf[0x40];
extern uint8_t usb_norm_buf[0x40];
extern uint8_t polling_mode;
extern uint8_t imu_enabled;
extern bool special_report_pending;
extern bool special_report_queued;
extern uint32_t last_report_time;

#endif // PROCON_FUNCTIONS_H