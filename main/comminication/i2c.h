#ifndef I2C_H
#define I2C_H

#include "driver/i2c.h"
#include <stdio.h>

#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define I2C_HIGH_SPEED 400000

void i2c_master_init(i2c_port_t port_num, uint8_t sda_pin_num, uint8_t scl_pin_num, uint32_t speed, uint8_t gpio_pull_up);
void i2c_read_bytes(i2c_port_t port_num, uint8_t slave_addr, uint8_t register_read_from, uint8_t* buffer, size_t size);
void i2c_write_byte(i2c_port_t port_num, uint8_t slave_addr, uint8_t reg, uint8_t value);
int8_t i2c_read_byte(i2c_port_t port_num, uint8_t slave_addr, uint8_t register_read_from);

#endif