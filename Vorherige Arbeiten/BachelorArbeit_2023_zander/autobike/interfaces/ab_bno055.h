#ifndef __BNO055_H__
#define __BNO055_H__

// could be interesting for gavity vector/linear accelaration vector/quaternions. 
// typedef struct int_coordinates {
//     int         x;
//     int         y;
//     int         z;
//     int         w;
// } int_coordinates_t;

typedef struct f_euler_degrees {
    float       heading;
    float       roll;
    float       pitch;
} f_euler_degrees_t;


// opening the connection to the I2C device on the corresponding bus
i2c_fd_t bno055_device_open(char* bus, long address, char *i2c_pin1, char *i2c_pin2);

// closing the connection to bno055 device
void bno055_device_close(i2c_fd_t fd);

// reads (data_size -1) user data from the i2c device. The register address of the first read byte
// has to be in data[0]. Returned user data will start from data[1]...
void bno055_device_burst_read(i2c_fd_t fd, uint8_t *data, int data_size);

// Allows one byte to be written to the devices register
void bno055_device_write_byte(i2c_fd_t fd, uint8_t register_address, uint8_t value);

// Reads the byte, which the register holds and returns it
uint8_t bno055_device_read_byte(i2c_fd_t fd, uint8_t register_address);

// Sets the register page to 0. This is needed for config modes and for reading sensor data
void bno055_set_page0(i2c_fd_t fd);

// Sets the mode of the bno055 sensor
void bno055_set_mode(i2c_fd_t fd, uint8_t mode);

// resets the bno055 through writing a bit into a register. returns the file descriptor
// of the newly opened connection. There is also a 0.575s delay before the connection can be reastablished.
i2c_fd_t bno055_soft_reset(i2c_fd_t fd);

// gets the euler angles and stores them in the f_euler_degrees struct. For this 
// page0 of the bno055 has to be selected and imu mode active.
void bno055_get_euler(i2c_fd_t fd, f_euler_degrees_t *euler, float angle_unit_factor);





#endif //__BNO055_H__

