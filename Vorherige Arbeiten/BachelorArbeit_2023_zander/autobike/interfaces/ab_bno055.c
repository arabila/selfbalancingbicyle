#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "ab_i2c.h"
#include "ab_bno055.h"
#include "../ab_error_handling.h"




i2c_fd_t bno055_device_open(char* bus, long address, char *i2c_pin1, char *i2c_pin2){
    
    char buffer[62];
    sprintf(buffer, "config-pin %s i2c", i2c_pin1);
    system(buffer);
    sprintf(buffer, "config-pin %s i2c", i2c_pin2);
    system(buffer);
    
    i2c_fd_t fd = i2c_device_open(bus, address);
    
    if (ioctl(fd, I2C_SLAVE, address) != 0) {
		AB_ERROR(-1, "ioctl bno055 failed.", AB_ERR_ABORT);
	}

	
	// this is a communication test to verify if the write operation can be executed.
	uint8_t data[] = {0x00};
	i2c_device_write(fd, data, sizeof(data) / sizeof(uint8_t));
	
	return fd;   
    
}


void bno055_device_close(i2c_fd_t fd)
{
	i2c_device_close(fd);
}


void bno055_device_burst_read(i2c_fd_t fd, uint8_t *data, int data_size)
{
	i2c_device_read(fd, data, data_size);
}



void bno055_device_write_byte(i2c_fd_t fd, uint8_t register_address, uint8_t value)
{
	uint8_t buffer[] = {register_address, value};
	
	i2c_device_write(fd, buffer, 2);
}

uint8_t bno055_device_read_byte(i2c_fd_t fd, uint8_t register_address)
{
	uint8_t buffer[2] = {register_address, 0};
	
	i2c_device_read(fd, buffer, 2);
	
	return buffer[1];
}


uint16_t bno055_make_16bit(uint8_t lsb, uint8_t msb)
{
	return (uint16_t) msb << 8 | (uint16_t) lsb;
}

uint16_t bno055_make_16bit_from_byte_buffer(uint8_t * buffer)
{
	return (uint16_t) buffer[1] << 8 | (uint16_t) buffer[0];
}

uint16_t bno055_device_read_16bit(i2c_fd_t fd, uint8_t register_address)
{
	uint8_t buffer[3] = {register_address, 0, 0};
	
	i2c_device_read(fd, buffer, 3);
	
	return bno055_make_16bit_from_byte_buffer(&buffer[1]);
}


void bno055_set_page0(i2c_fd_t fd)
{
	bno055_device_write_byte(fd, 0x07, 0x00);
}

uint8_t bno055_get_mode(i2c_fd_t fd)
{
	return bno055_device_read_byte(fd, 0x3D) & 0x0F;
}

void bno055_set_mode(i2c_fd_t fd, uint8_t mode)
{
	bno055_set_page0(fd);
	
	if (mode == bno055_get_mode(fd))
	{
		return;
	}
	
	bno055_device_write_byte(fd, 0x3D, mode);
	
	// Changing modes takes time depending on the state.
	usleep(mode ? 9000 : 21000);
}

i2c_fd_t bno055_soft_reset(i2c_fd_t fd)
{
	bno055_set_page0(fd);
	
	uint8_t buffer[2] = { 0x3F, 0 };
	
	i2c_device_read(fd, buffer, 2);
	buffer[1] |= 0x20;
	i2c_device_write(fd, buffer, 2);
	
	bno055_device_close(fd);

	usleep(575000);  // MIN 0.55s needed for reset!
	
	return -1;
}

int16_t make_int16(uint8_t msb, uint8_t lsb)
{
	// uint16_t cast to extend space, ignoring the sign bit.
	// int16_t cast to correctly interpret the sign bit.
	return (int16_t) (((uint16_t) msb)<<8 | lsb);
}


void bno055_get_euler(i2c_fd_t fd, f_euler_degrees_t *euler, float angle_unit_factor)
{
	//device should be in page 0 and imu mode
	
	uint8_t data[7];
	
	data[0] = 0x1A; // Address of first Euler register
	
	bno055_device_burst_read(fd, data, 7);
	
	euler->heading = make_int16(data[2], data[1]) / angle_unit_factor;
	euler->roll    = make_int16(data[4], data[3]) / angle_unit_factor;
	euler->pitch   = make_int16(data[6], data[5]) / angle_unit_factor;
}