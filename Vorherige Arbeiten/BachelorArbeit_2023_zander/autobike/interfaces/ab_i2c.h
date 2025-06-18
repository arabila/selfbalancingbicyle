#ifndef __AB_I2C_H__
#define __AB_I2C_H__

#include <stdint.h>




typedef int i2c_fd_t;  // i2c file descriptor type

// opens the connection to the device on the corresponding bus.
i2c_fd_t i2c_device_open(char* bus, long address);

// closes the connection to the i2c device
void i2c_device_close(i2c_fd_t fd);

// writes the num_bytes to the i2c device including the address in the first byte of the buffer. 
// The first byte in data is the register address the following bytes are the data.
// So the amount of user data is (num_bytes - 1)
void i2c_device_write(i2c_fd_t fd, const uint8_t *data, int num_bytes);

// reads (num_bytes -1) user data from the i2c device. The register address of the first read byte
// has to be in data[0]. Returned user data will start from data[1]...
void i2c_device_read(i2c_fd_t fd, uint8_t *data, int num_bytes);

#endif // __AB_I2C_H__