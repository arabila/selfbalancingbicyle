#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "ab_i2c.h"
#include "../ab_error_handling.h"

i2c_fd_t i2c_device_open(char* bus, long address)
{
    i2c_fd_t fd;
    
    if ((fd = open(bus, O_RDWR)) < 0) {
        AB_ERROR(-1, "Bus open failed.", AB_ERR_ABORT);
    }
}

void i2c_device_close(i2c_fd_t fd)
{
    close(fd);
}

void i2c_device_write(i2c_fd_t fd, const uint8_t *data, int num_bytes)
{
    if (write(fd, data, num_bytes) != num_bytes) {
        AB_ERROR(-1, "write to device failed", AB_ERR_ABORT);
    }
}

void i2c_device_read(i2c_fd_t fd, uint8_t *data, int num_bytes){
    i2c_device_write(fd, data, 1);
    if (read(fd, data + 1, num_bytes - 1) != num_bytes - 1) {
        AB_ERROR(-1, "reading from device failed", AB_ERR_ABORT);
    }
}