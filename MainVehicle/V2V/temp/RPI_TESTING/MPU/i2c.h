/*
 * I2C.h
 *
 * Author: Hady Abdelhady
 */
#ifndef __I2C__
#define __I2C__

// #include "utils.h"
#include <linux/i2c-dev.h>
#include "STDTYPE.h"
#define I2C_DEVICE "/dev/i2c-1"

extern char buffer1[40]; // Initialize buffer for receiving string 1
extern char buffer2[40]; // Initialize buffer for receiving string 2
extern char buffer3[40]; // Initialize buffer for receiving string 3
extern char data[120];   // Initialize buffer for concatenated data

extern sint16 i2c_fd; // File descriptor for I2C device

void i2c_init(int slave_address);
void i2c_send_data(uint8 *data, size_t len);
void i2c_receive_data(uint8 *buffer, size_t len);
uint8 i2c_receive_byte(uint8 last);
void i2c_send_byte(uint8 data);
void i2c_close();
#endif
