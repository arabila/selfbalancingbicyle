#ifndef __AB_MD49_H__
#define __AB_MD49_H__

#include <stdint.h>

typedef int uart_fd_t;

// opens the connection to the md49 device using uart communication.
// uart 4 port is "/dev/ttyO4" on the Beaglebone Black. 
// currently pin "p9.11" and "p9.13" are in use.
// the names of the pins have to be given in the style of "p<number>.<number>"
uart_fd_t md49_device_open(char* port, char *uart_pin1, char *uart_pin2);

// closes the connection to the md49 device.
void md49_device_close(uart_fd_t fd);

// writes the given one byte command to the md49 device.
void md49_device_write(uart_fd_t fd, uint8_t command);

// reads data_size amount of bytes from the md49 and writes it into the data array
void md49_device_read(uart_fd_t fd, uint8_t *data, int data_size);

// returns one int32 value, which represents the encoder value of encoder 1.
int32_t md49_get_encoder1(uart_fd_t fd);

// resets the values of the encoders to 0. has a built in delay of 5000 us.
void md49_device_reset(uart_fd_t fd);

#endif // __AB_MD49_H__