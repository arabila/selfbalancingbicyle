#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "../ab_error_handling.h"
#include "ab_md49.h"

#define CMD             0x00
#define GET_SPEED1      0x21
#define GET_SPEED2      0x22
#define GET_ENC1        0x23
#define GET_ENC2        0x24
#define GET_ENCS        0x25
#define GET_VOLTS       0x26
#define GET_CUR1        0x27
#define GET_CUR2        0x28
#define GET_VER         0x29
#define GET_ACC         0x2A
#define GET_MODE        0x2B
#define GET_VI          0x2C
#define GET_ERROR       0x2D
#define SET_SPEED1      0x31
#define SET_SPEED2      0x32
#define SET_ACC         0x33
#define SET_MODE        0x34
#define RESET_ENCS      0x35
#define DISABLE_REG     0x36
#define ENABLE_REG      0x37
#define DISABLE_TO      0x38
#define ENABLE_TO       0x39




uart_fd_t md49_device_open(char* port, char *uart_pin1, char *uart_pin2){
    
    char buffer[62];
    sprintf(buffer, "config-pin %s uart", uart_pin1);
    system(buffer);
    sprintf(buffer, "config-pin %s uart", uart_pin2);
    system(buffer);
    
    uart_fd_t fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        AB_ERROR(-1, "port open Failed", AB_ERR_ABORT);
    }
    struct termios options;
    tcgetattr(fd, &options);
    cfsetspeed(&options, B38400);
    cfmakeraw(&options);
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    usleep(10000);
    return fd;
}

void md49_device_close(uart_fd_t fd)
{
    close(fd);
}

void md49_device_write(uart_fd_t fd, uint8_t command)
{
    uint8_t buffer[] = {0x00, command};
    if (write(fd, buffer, 2) != 2) {
        AB_ERROR(-1, "write to device failed", AB_ERR_ABORT);
    }
}

void md49_device_read(uart_fd_t fd, uint8_t *data, int data_size)
{
    if (read(fd, data, data_size) != data_size){
        AB_ERROR(-1, "reading from device failed.", AB_ERR_ABORT);
    }

}

void md49_device_reset(uart_fd_t fd){
    md49_device_write(fd, RESET_ENCS);
    usleep(5000);
}

int32_t md49_get_encoder1(uart_fd_t fd){
    md49_device_write(fd, GET_ENC1);
    uint8_t data[4];
    md49_device_read(fd, data, 4);
    // conversion of the endianness and from unsigned to a signed integer.
    int32_t encoder_val = (int32_t) ((uint32_t)data[0]<<24 | (uint32_t)data[1]<<16 | (uint32_t)data[2]<<8 | data[3]);
    return encoder_val;
}