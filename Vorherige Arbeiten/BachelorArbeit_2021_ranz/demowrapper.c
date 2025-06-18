/* ---------------------------------------------------------------*
 * file:        demowrapper.c                                     *
 * purpose:     wraps central program to balance a bicycle        *
 *           using the BNO055 sensor and steering                 *
 *           via the EM49 DC motor                                *
 *           and its controller board MD49                        *
 *                                                                *
 * return:      0 on success, and -1 on errors.                   *
 *                                                                *
 * requires:    I2C headers (libi2c-dev)                          *
 *          GPIOD header (libgpiod-dev)                           *
 *                                                                *
 * compile:    gcc -o demowrapper demowrapper.c -lm -lgpiod       *
 *                                                                *
 * author:      agr                                               *
 * ---------------------------------------------------------------*/

// information MD49 at http://www.robot-electronics.co.uk/htm/md49tech.htm

#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<fcntl.h>
#include<unistd.h>
#include<string.h>
#include<termios.h>
#include<errno.h>
#include<sys/ioctl.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>
#define _USE_MATH_DEFINES
#include<math.h>
#include<gpiod.h>



#include "bicycle.h"

/* ------------------------------------------------------------ *
 * function prototypes for motor control                        *
 * ------------------------------------------------------------ */
int set_up_uart(int);
int write_bytes_uart(int, int);
int read_bytes_uart(int, int);
int set_MD49_motor_mode(int, char);
char get_MD49_motor_mode(int);
int drive_MD49_motor(int);
int stop_MD49_motor(int);
int get_MD49_speed(int);
int get_MD49_steering_angle(int);
int set_MD49_steering_angle(int, float, int);
int drive_MD49_motor_until_encoder(int, int, int);
int calc_MD49_encoder_from_angle(float);
int reset_MD49_encoder(int);
int get_MD49_encoder(int);

/* ------------------------------------------------------------ *
 * function prototypes for BNO055                               *
 * ------------------------------------------------------------ */
int get_i2c(char*, char*);
int get_mode(int);
int set_mode(int, operation_mode_t newmode);
int set_axis_mode(int);
int get_eul(int, struct euler_angles*);
float get_roll(int);
float get_pitch(int);
float get_yaw(int);
double get_gyro_x(int);
double get_gyro_y(int);
double get_gyro_z(int);
int set_page0(int);
int set_page1(int);
int set_interrupt(int);

/* ------------------------------------------------------------ *
 * function declaration for control loop                        *
 * ------------------------------------------------------------ */
float calculate_feedback(float, float, float);
float calculate_next_angle(float, float);

/* ------------------------------------------------------------ *
 * global variables and constants                               *
 * ------------------------------------------------------------ */
int verbose = 1; 						// used for printf debugging
char sensor_address[256] = "0x28";		// address of the bno055
char i2c_bus[256] = I2CBUS;				// get the used i2c device from bicycle.h
char port_Name[256] = UART;			    // get the used uart device from bicycle.h
char serialBuffer[10] = {0};  			// serial buffer for UART
int direction = 1;						// clockwise = -1 needs negative speed
										// counterclockwise = 1, needs positive speed
float delta = 0;                        // current steering angle
signed char speed = 0;                  // current speed

const float encoder_ratio = 980.0;     	// Encoder counts per output shaft turn
const float gears_motor = 30.0;			// number of teeth input gear
const float gears_steering = 44.0;		// number of teeth output gear
const float pi = M_PI;                  // get pi from math.h
const float full_circle = 360;          // decides unit deg(360.00) or rad(2pi) for motor
const float encoder_to_angle = full_circle/(encoder_ratio * (gears_steering/gears_motor));
const float steering_to_encoder_ratio = ((encoder_ratio * (gears_steering/gears_motor)) / full_circle);

const float angle_unit = 16.0;          // defines the angle unit (datasheet p.35) for imu
                                        // set 16.0 for degrees, 900.0 for rad

union encoder_32 {						// to store the encoder values
    int value;
    char bytes[4];
} current_encoder, last_encoder;

int32_t encoder_result;



/* ------------------------------------------------------------ *
 * main sets up uart, i2c and starts the main control loop      *
 * ------------------------------------------------------------ */
int main(int argc, char *argv[]){

	// file descriptors for communication with sensor and motor controller
    int i2c_fd = 0;
    int uart_fd = 0;
    char mode = 0;

    // set up i2c to talk to bno055
    i2c_fd = get_i2c(i2c_bus, sensor_address);
    // set up i2c to talk to MD49
    uart_fd = set_up_uart(uart_fd);


    int res = -1;
    int toggle = 30;

    // settin motor mode to mode 1: -128(full speed reverse/clockwise), 0(stop), +128(full forward/counterclockwise)
    res = set_MD49_motor_mode(uart_fd, 0x01);
    usleep(1000);
    mode = get_MD49_motor_mode(uart_fd);

    //remap axes to config 'P0' (datasheet p.27)
    set_page0(i2c_fd);
    set_axis_mode(i2c_fd);

    // set operation mode of bno055 to imu
    operation_mode_t new_mode;
    new_mode = imu;
   	res = set_mode(i2c_fd, new_mode);

    res = reset_MD49_encoder(uart_fd);
    usleep(1000);

    float current_steering_angle = 0.0;
    float next_angle = 0.0;
    float k_phi = 0.5;
    float k_m = 1.0;
    float feedback_size;

    float last_roll = 0.0;
    float current_roll = 0.0;
    float current_pitch = 0.0;
    float current_yaw = 0.0;

    // set the turn limit
    float threshold = 22.50;
    int max_encoder = 0;
    max_encoder = calc_MD49_encoder_from_angle(threshold);


    float difference = last_roll - current_roll;

    // set global speed
    speed = 25;
    usleep(1000000);

    current_roll = get_roll(i2c_fd);
    last_roll = current_roll;

    do{
        // next_angle is delta_0
        current_roll = get_roll(i2c_fd);
    	next_angle = - (k_phi) * current_roll;
        res = set_MD49_steering_angle(uart_fd, next_angle, max_encoder);
        last_roll = current_roll;
        toggle --;

    }while(toggle > 0); 


    return(res);

}

/* ------------------------------------------------------------ *
 * function declarations for motor control                      *
 * ------------------------------------------------------------ */

 /* ------------------------------------------------------------ *
 * @brief   sets up UART BBB P9.11,P9.13 -> /dev/ttyO4            
 * @param   uart_fd
 *          uart file descriptor
 * @return  uart_fd
 *          uart file descriptor
 * ------------------------------------------------------------ */
int set_up_uart(int uart_fd){

    char *port_Name = "/dev/ttyO4";							    // Name of the UART4 on BeagleBone
    struct termios options;                                 	// Port options


    uart_fd = open(port_Name, O_RDWR | O_NOCTTY); 				// Open port for read and write not making it a controlling terminal
    
    if (uart_fd == -1){
            perror("openPort: Unable to open port ");	
    }
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B38400);                          	// Set baud rate to 38400
    cfsetospeed(&options, B38400);
    cfmakeraw(&options);
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);

    usleep(10000);                                          	// Sleep for UART to power up and set options

    return uart_fd;

}

  /* ------------------------------------------------------------ *
 * @brief   writes var count number of bytes into the serial buffer           
 * @param   uart_fd
 *          uart file descriptor
 * @param   count
 *          int number of bytes to be written
 * @return  0 if process is successful 
 * ------------------------------------------------------------ */
int write_bytes_uart(int file_descriptor, int count) {
    
    if ((write(file_descriptor, serialBuffer, count)) == -1) {	// Send data out
    	printf("Error: UART write failure in function write_bytes_uart");
        perror("Error writing on UART");
        close(file_descriptor);										// Close port if there is an error
        return(-1);
    }

    return 0;
}

/* ------------------------------------------------------------ *
 * @brief   reads var count number of bytes from the serial buffer           
 * @param   uart_fd
 *          uart file descriptor
 * @param   count
 *          int number of bytes to be written
 * @return  0 if process is successful 
 * ------------------------------------------------------------ */
int read_bytes_uart(int file_descriptor, int count) {
    
    if (read(file_descriptor, serialBuffer, count) == -1) {		// Read back data into buf[]
    	printf("Error: UART read failure in function read_bytes_uart");
        perror("Error reading reading from UART");
        close(file_descriptor);										// Close port if there is an error
        return(-1);
    }

    return 0;
}

/* ------------------------------------------------------------ *
 * @brief   sets motor mode to mode var mode          
 * @param   uart_fd
 *          uart file descriptor
 * @param   mode
 *          char describig motor mode
 * @return  0 if process is successful 
 * ------------------------------------------------------------ */
int set_MD49_motor_mode(int file_descriptor, char mode) {

	int res = 0;

    serialBuffer[0] = CMD;
    serialBuffer[1] = SET_MODE;
    serialBuffer[2] = mode;

    res = write_bytes_uart(file_descriptor, 3);
    if(verbose == 1) printf("set MD49 to mode: %d \n", mode);

    return res;
}

 /* ------------------------------------------------------------ *
 * @brief   reads current motor mode          
 * @param   uart_fd
 *          uart file descriptor
 * @return  mode
 *          char describig motor mode
 * ------------------------------------------------------------ */
char get_MD49_motor_mode(int file_descriptor) {

	int mode = 0;
    int res = 0;

    serialBuffer[0] = CMD;
    serialBuffer[1] = GET_MODE;

    res = write_bytes_uart(file_descriptor, 2);
    res = read_bytes_uart(file_descriptor, 1);

    mode = serialBuffer[0];

    if(verbose == 1) printf("got MD49 mode: %d \n", mode);

    return mode;

}

 /* ------------------------------------------------------------ *
 * @brief   sets speed of motor 1 to global var speed         
 * @param   uart_fd
 *          uart file descriptor
 * @return  0 if process is successful 
 * ------------------------------------------------------------ */
int drive_MD49_motor(int file_descriptor) {

	int res = 0;

    serialBuffer[0] = CMD;
    serialBuffer[1] = SET_SPEED1;
    serialBuffer[2] = speed;
    
    res = write_bytes_uart(file_descriptor, 3);
    if(verbose == 1) printf("driving with speed %d \n", speed);
    return res;
}

 /* ------------------------------------------------------------ *
 * @brief   sets speed of motor 1 to 0         
 * @param   uart_fd
 *          uart file descriptor
 * @return  0 if process is successful 
 * ------------------------------------------------------------ */
int stop_MD49_motor(int file_descriptor) {

	int res = 0;

    signed char stop = 0;

    serialBuffer[0] = CMD;
    serialBuffer[1] = SET_SPEED1;
    serialBuffer[2] = stop;
    
    res = write_bytes_uart(file_descriptor, 3);
    if(verbose == 1) printf("stopped motor \n");
    return res;

}


 /* ------------------------------------------------------------ *
 * @brief   sends request to fill buffer with current speed and returns the speed        
 * @param   uart_fd
 *          uart file descriptor
 * @return  speed
 *          signed char              
 * ------------------------------------------------------------ */
int get_MD49_speed(int file_descriptor){

    int res = 0;

    serialBuffer[0] = CMD;
    serialBuffer[1] = GET_SPEED1;

    res = write_bytes_uart(file_descriptor, 2);
    res = read_bytes_uart(file_descriptor, 1);

    signed char raw_speed;
    raw_speed = serialBuffer[0];
    if(verbose == 1) printf("raw speed: %x \n", raw_speed);

    int speed = (int) raw_speed;
    if(verbose == 1) printf("int speed: %d \n", speed);

    return speed;

}

/* ------------------------------------------------------------ *
 * @brief   sends request get_encdoer and calculates angle from encoder value        
 * @param   uart_fd
 *          uart file descriptor
 * @return  0 if process is successful               
 * ------------------------------------------------------------ */
int get_MD49_steering_angle(int file_descriptor){

    int res = 0;

    float fp_current_encoder;
    float current_steering_angle = 0.0;
    float current_motor_angle = 0.0;

    res = get_MD49_encoder(file_descriptor);

    fp_current_encoder = (float) encoder_result;
    current_steering_angle = fp_current_encoder * encoder_to_angle;
    current_motor_angle = fp_current_encoder * (full_circle / encoder_ratio);

    if(verbose == 1) printf("pulled steering_angle: encoder %f to motor angle %f, steering angle %f\n", fp_current_encoder, current_motor_angle, current_steering_angle);

    delta = current_steering_angle;
    return res;

}

 /* ------------------------------------------------------------ *
 * @brief   receives steering angle and sets it        
 * @param   uart_fd
 *          uart file descriptor
 * @param   next_steering_angle
 *          float
 * @return  0 if process is successful             
 * ------------------------------------------------------------ */
int set_MD49_steering_angle(int file_descriptor, float next_steering_angle, int max_encoder){

	int res = 0;
	int next_encoder = 0;
    int current_encoder = 0;

    res = get_MD49_steering_angle(file_descriptor);
    next_encoder = calc_MD49_encoder_from_angle(next_steering_angle);
    usleep(1000);
    
    if(current_encoder > next_encoder){

        speed = - (abs(speed));
        
    }else if(current_encoder < next_encoder){

        speed = (abs(speed));

    }else{

        return res;

    }

    if(verbose == 1) printf("set steering angle %i from requested angle %f with speed %d\n", next_encoder, next_steering_angle, speed);

    next_encoder = calc_MD49_encoder_from_angle(next_steering_angle);
    res = drive_MD49_motor_until_encoder(file_descriptor, next_encoder, max_encoder);
    
    return res;

}

 /* ------------------------------------------------------------ *
 * @brief   transfers angle to encoder value      
 * @param   next_steering_angle
 *          float
 * @return  next_encoder
 *          int             
 * ------------------------------------------------------------ */
int calc_MD49_encoder_from_angle(float next_steering_angle){

	int next_encoder = 0;
	float fp_next_encoder = 0.0;

    fp_next_encoder = - next_steering_angle * steering_to_encoder_ratio;
    next_encoder = (int) fp_next_encoder;
    
    //if(verbose == 1) printf("calculated encoder: %i from requested angle %f \n", next_encoder, next_steering_angle);

	return next_encoder;
}

/* ------------------------------------------------------------ *
 * @brief   drives motor until var limit encoder value is reached
 * @param   uart_fd
 *          uart file descriptor
 * @param   limit
 *          int
 * @return  0 if process is successful              
 * ------------------------------------------------------------ */
int drive_MD49_motor_until_encoder(int file_descriptor, int limit, int max_encoder){

	int res = 0;

	if(verbose == 1) printf("drive_MD49_motor_until_encoder limit %d, speed %d \n", limit, speed);


	do{
		
        res = drive_MD49_motor(file_descriptor);
		res = get_MD49_encoder(file_descriptor);

		if(verbose == 1) printf("driving with encoder: %d limit: %d\n", current_encoder.value, limit);

        if(speed < 0){						// if mototr is rotating clockwise
			
            if(encoder_result <= limit){

				res = stop_MD49_motor(file_descriptor);
				if(verbose == 1) printf("(encoder_result <= limit):stopped driving with encoder: %d \n", current_encoder.value);
				 return res;
                }
        }else{									// if mototr is rotating counterclockwise
            if(encoder_result >=  limit){

                res = stop_MD49_motor(file_descriptor);
                if(verbose == 1) printf("(encoder_result >=  limit):stopped driving with encoder: %d \n", current_encoder.value);
                return res;
            }
    	}

    }while(encoder_result != limit);

    return res;

}

/* ------------------------------------------------------------ *
 * @brief   resets the encoder registers to zero
 * @param   uart_fd
 *          uart file descriptor
 * @return  0 if process is successful              
 * ------------------------------------------------------------ */
int reset_MD49_encoder(int file_descriptor) {

	int res = 0;

    serialBuffer[0] = CMD;
    serialBuffer[1] = RESET_ENCS;

    res = write_bytes_uart(file_descriptor, 2);
    if(verbose == 1) printf("reset encoder registers \n");
    return res;

}


 /* ------------------------------------------------------------ *
 * @brief   request the current encoder value and reads it into global encoder struct
 * @param   uart_fd
 *          uart file descriptor
 * @return  0 if process is successful              
 * ------------------------------------------------------------ */
int get_MD49_encoder(int file_descriptor) {

    int res = 0;
    current_encoder.value = 0;

    serialBuffer[2]= 0;

    serialBuffer[0] = CMD;
    serialBuffer[1] = GET_ENC1;

    res = write_bytes_uart(file_descriptor, 2);
    res = read_bytes_uart(file_descriptor, 4);

    /*
    if(verbose == 1) {
    	printf("serialBuffer[0]: %x \n", serialBuffer[0]);
    	printf("serialBuffer[1]: %x \n", serialBuffer[1]);
    	printf("serialBuffer[2]: %x \n", serialBuffer[2]);
    	printf("serialBuffer[3]: %x \n", serialBuffer[3]);
		fflush(stdout);					// Flush output to ensure that data is displayed on the screen
	}
    */
	

    current_encoder.bytes[0] = serialBuffer[3];
    encoder_result =  serialBuffer[0] << 24ul;
    current_encoder.bytes[1] = serialBuffer[2];
    encoder_result +=  serialBuffer[1] << 16ul;
    current_encoder.bytes[2] = serialBuffer[1];
    encoder_result +=  serialBuffer[2] << 8ul;
    current_encoder.bytes[3] = serialBuffer[0];
    encoder_result +=  serialBuffer[3];

    if(verbose == 1) {
    	//printf("encoder 1 : %d \n", current_encoder.value);
    	printf("encoder_result : %d \n", encoder_result);
		fflush(stdout);					// Flush output to ensure that data is displayed on the screen
	}

	return res;                                                              
}


/* ------------------------------------------------------------ *
 * function declaration for BNO055                              
 * ------------------------------------------------------------ */


/* ------------------------------------------------------------ *
 * @brief   sets up I2C BBB P9.19,P9.20 -> /dev/i2c-2             
 * @param   i2cbus
 *          i2c device name
 * @param   i2caddr
 *          i2c address
 * @param   i2cfd
 *          i2c file descriptor
 * ------------------------------------------------------------ */
int get_i2c(char *i2cbus, char *i2caddr){

    int i2cfd;

    if((i2cfd = open(i2cbus, O_RDWR)) < 0) {
          printf("Error failed to open I2C bus [%s].\n", i2cbus);
          exit(-1);
    }
    if(verbose == 1) printf("Debug: I2C bus device: [%s]\n", i2cbus);
    /* --------------------------------------------------------- *
    * Set I2C device (BNO055 I2C address is  0x28)              *
    * --------------------------------------------------------- */
    int addr = (int)strtol(i2caddr, NULL, 16);
    if(verbose == 1) printf("Debug: Sensor address: [0x%02X]\n", addr);

    if(ioctl(i2cfd, I2C_SLAVE, addr) != 0) {
          printf("Error can't find sensor at address [0x%02X].\n", addr);
          exit(-1);
    }
    /* --------------------------------------------------------- *
    * I2C communication test is the only way to confirm success *
    * --------------------------------------------------------- */
    char reg = BNO055_CHIP_ID_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
        printf("Error: I2C write failure register [0x%02X], sensor addr [0x%02X]?\n", reg, addr);
        exit(-1);
    }

    return i2cfd;

}

/* ------------------------------------------------------------ *
 * @brief   reads and returns sensor operational mode register 0x3D             
 * @param   i2cfd
 *          i2c file descriptor
 * @return  data & 0x0F
 *          lowest 4 bits of register
 * ------------------------------------------------------------ */
int get_mode(int i2cfd) {

    int reg = BNO055_OPR_MODE_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
        printf("Error: I2C write failure in function get_mode for register 0x%02X\n", reg);
        return(-1);
    }

    unsigned int data = 0;
    if(read(i2cfd, &data, 1) != 1) {
        printf("Error: I2C read failure in function get_mode for register data 0x%02X\n", reg);
        return(-1);
    }

    if(verbose == 1) printf("Debug: Operation Mode: [0x%02X]\n", data & 0x0F);

    return(data & 0x0F);  // only return the lowest 4 bits
}

 /* ------------------------------------------------------------ *
 * @brief   set the sensor operational mode register 0x3D            
 * @param   i2cfd
 *          i2c file descriptor
 * @param   newmode
 *          mode as operation_mode_t
 * @return  0 if process is successful
 * ------------------------------------------------------------ */
int set_mode(int i2cfd, operation_mode_t newmode) {

    set_page0(i2cfd);
    
    char data[2] = {0};
    data[0] = BNO055_OPR_MODE_ADDR;
    operation_mode_t oldmode = get_mode(i2cfd);

    if(oldmode == newmode) return(0); // if new mode is the same

    //set to config mode
    else if(oldmode > 0 && newmode > 0) {  // switch to "config" first
        data[1] = 0x0;
        if(verbose == 1) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
        if(write(i2cfd, data, 2) != 2) {
            printf("Error: I2C write failure in function set_mode for register 0x%02X\n", data[0]);
            return(-1);
        }
        /* --------------------------------------------------------- *
         * switch time: any->config needs 7ms + small buffer = 10ms  *
         * --------------------------------------------------------- */
        usleep(10 * 1000);
       }

    data[1] = newmode;
    if(verbose == 1) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
    if(write(i2cfd, data, 2) != 2) {
        printf("Error: I2C write failure in function set_mode for register 0x%02X\n", data[0]);
        return(-1);
    }

    /* --------------------------------------------------------- *
     * switch time: config->any needs 19ms + small buffer = 25ms *
     * --------------------------------------------------------- */
    usleep(25 * 1000);

    if(verbose == 1) printf("Did: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);

    if(get_mode(i2cfd) == newmode) return(0);
    else return(-1);
}

  /* ------------------------------------------------------------ *
 * @brief   remaps the axes fitting the bike            
 * @param   i2cfd
 *          i2c file descriptor
 * @return  0 id process is successful
 * ------------------------------------------------------------ */
int set_axis_mode(int i2cfd) {

    // change to page 0
    set_page0(i2cfd);
    
    // set to config mode
    char mode[2] = {0};
    mode[0] = BNO055_OPR_MODE_ADDR;
    mode[1] = 0x0;
    
    if(write(i2cfd, mode, 2) != 2) {
            printf("Error: I2C write failure in function set_mode for register 0x%02X\n", mode[0]);
            return(-1);
        }
    /* --------------------------------------------------------- *
     * switch time: any->config needs 7ms + small buffer = 10ms  *
     * --------------------------------------------------------- */
    usleep(10 * 1000);

    char data[2] = {0};
    data[0] = BNO055_AXIS_REMAP_CONFIG_ADDR;
    data[1] = BNO055_AXIS_CONFIG;
    if(verbose == 1) printf("Debug: write axis remap config: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
    if(write(i2cfd, data, 2) != 2) {
        printf("Error: I2C write failure for register 0x%02X\n", data[0]);
        return(-1);
    }
    data[0] = BNO055_AXIS_REMAP_SIGN_ADDR;
    data[1] = BNO055_AXIS_SIGN;
    if(verbose == 1) printf("Debug: write axis sign config: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
    if(write(i2cfd, data, 2) != 2) {
        printf("Error: I2C write failure for register 0x%02X\n", data[0]);
        return(-1);
    }

    return(0);
}

   /* ------------------------------------------------------------ *
 * @brief   reads euler_angles into the global struct            
 * @param   i2cfd
 *          i2c file descriptor
 * @param   *bnod_ptr
 *          pointer to euler angles struct
 * @return  0 id process is successful
 * ------------------------------------------------------------ */
int get_eul(int i2cfd, struct euler_angles *bnod_ptr) {

    char reg = BNO055_EULER_H_LSB_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
        printf("Error: I2C write failure in function get_eul for register 0x%02X\n", reg);
        return(-1);
    }

    //if(verbose == 1) printf("Debug: I2C read 6 bytes starting at register 0x%02X\n", reg);

    unsigned char euler_angles_data[6] = {0, 0, 0, 0, 0, 0};

    if(read(i2cfd, euler_angles_data, 6) != 6) {
        printf("Error: I2C read failure in function get_eul for register data 0x%02X\n", reg);
        return(-1);
    }

    int16_t buf = ((int16_t)euler_angles_data[1] << 8) | euler_angles_data[0];
    if(verbose == 1) printf("Debug: Euler Orientation H: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", euler_angles_data[0], euler_angles_data[1],buf);
    bnod_ptr->eul_head = (double) buf / angle_unit;

    buf = ((int16_t)euler_angles_data[3] << 8) | euler_angles_data[2];
    if(verbose == 1) printf("Debug: Euler Orientation R: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", euler_angles_data[2], euler_angles_data[3],buf);
    bnod_ptr->eul_roll = (double) buf / angle_unit;

    buf = ((int16_t)euler_angles_data[5] << 8) | euler_angles_data[4];
    if(verbose == 1) printf("Debug: Euler Orientation P: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", euler_angles_data[4], euler_angles_data[5],buf);
    bnod_ptr->eul_pitc = (double) buf / angle_unit;

    return(0);
}

/* ------------------------------------------------------------ *
 * @brief   returns the roll in  var angle_unit unit            
 * @param   i2cfd
 *          i2c file descriptor
 * @return  roll
 *          float
 * ------------------------------------------------------------ */
float get_roll(int i2cfd) {

    float roll;
    char reg = BNO055_EULER_H_LSB_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
            printf("Error: I2C write failure in function get_roll for register 0x%02X\n", reg);
            return(-1);
    }
    //if(verbose == 1) printf("Debug: I2C read 6 bytes starting at register 0x%02X\n", reg);

    unsigned char euler_angles_data[6] = {0, 0, 0, 0, 0, 0};

    if(read(i2cfd, euler_angles_data, 6) != 6) {
            printf("Error: I2C read failure in function get_roll for register data 0x%02X\n", reg);
            return(-1);
    }

    int16_t buf = ((int16_t)euler_angles_data[3] << 8) | euler_angles_data[2];

    roll = (float) buf / angle_unit;

    if(verbose == 1) printf("roll is: %f \n", roll); 

    return roll;
}

 /* ------------------------------------------------------------ *
 * @brief   returns the pitch in  var angle_unit unit            
 * @param   i2cfd
 *          i2c file descriptor
 * @return  pitch
 *          float
 * ------------------------------------------------------------ */
float get_pitch(int i2cfd) {

    float pitch;
    char reg = BNO055_EULER_H_LSB_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
            printf("Error: I2C write failure in function get_roll for register 0x%02X\n", reg);
            return(-1);
    }
    //if(verbose == 1) printf("Debug: I2C read 6 bytes starting at register 0x%02X\n", reg);

    unsigned char euler_angles_data[6] = {0, 0, 0, 0, 0, 0};

    if(read(i2cfd, euler_angles_data, 6) != 6) {
            printf("Error: I2C read failure in function get_roll for register data 0x%02X\n", reg);
            return(-1);
    }

    int16_t buf = ((int16_t)euler_angles_data[5] << 8) | euler_angles_data[4];

    pitch = (float) buf / angle_unit;

    if(verbose == 1) printf("pitch is: %f \n", pitch); 

    return pitch;
}

 /* ------------------------------------------------------------ *
 * @brief   returns the yaw in  var angle_unit unit            
 * @param   i2cfd
 *          i2c file descriptor
 * @return  yaw
 *          float
 * ------------------------------------------------------------ */
float get_yaw(int i2cfd) {

    float yaw;
    char reg = BNO055_EULER_H_LSB_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
            printf("Error: I2C write failure in function get_roll for register 0x%02X\n", reg);
            return(-1);
    }
    //if(verbose == 1) printf("Debug: I2C read 6 bytes starting at register 0x%02X\n", reg);

    unsigned char euler_angles_data[6] = {0, 0, 0, 0, 0, 0};

    if(read(i2cfd, euler_angles_data, 6) != 6) {
            printf("Error: I2C read failure in function get_roll for register data 0x%02X\n", reg);
            return(-1);
    }

    int16_t buf = ((int16_t)euler_angles_data[1] << 8) | euler_angles_data[0];

    yaw = (float) buf / angle_unit;

    if(verbose == 1) printf("yaw is: %f \n", yaw); 

    return yaw;
}

/* ------------------------------------------------------------ *
 * @brief   return gyro rate along the x axis           
 * @param   i2cfd
 *          i2c file descriptor
 * @return  gyro_x
 *          double
 * ------------------------------------------------------------ */
double get_gyro_x(int i2cfd){

    double gyro_x;

    char reg = BNO055_GYRO_DATA_X_LSB_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
       printf("Error: I2C write failure in function get_gyro_x for register 0x%02X\n", reg);
       return(-1);
    }

    char data[6] = {0};
    if(read(i2cfd, data, 6) != 6) {
          printf("Error: I2C read failure in function get_gyro_x for register data 0x%02X\n", reg);
          return(-1);
    }

    int16_t buf = ((int16_t)data[1] << 8) | data[0];
    //if(verbose == 1) printf("Debug: Gyroscope Data X: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[0], data[1],buf);
    
    gyro_x = (double) buf / angle_unit;
    if(verbose == 1) printf("gyro_x is: %f \n", gyro_x);

    return(gyro_x);

}

/* ------------------------------------------------------------ *
 * @brief   return gyro rate along the y axis           
 * @param   i2cfd
 *          i2c file descriptor
 * @return  gyro_y
 *          double
 * ------------------------------------------------------------ */
double get_gyro_y(int i2cfd){

    double gyro_y;

    char reg = BNO055_GYRO_DATA_Y_LSB_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
       printf("Error: I2C write failure in function get_gyro_y  for register 0x%02X\n", reg);
       return(-1);
    }

    char data[6] = {0};
    if(read(i2cfd, data, 6) != 6) {
          printf("Error: I2C read in function get_gyro_y failure for register data 0x%02X\n", reg);
          return(-1);
    }

    int16_t buf = ((int16_t)data[5] << 8) | data[4];
    //if(verbose == 1) printf("Debug: Gyroscope Data y: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);
    
    gyro_y = (double) buf / angle_unit;
    if(verbose == 1) printf("gyro_y is: %f \n", gyro_y);

    return(gyro_y);

}

/* ------------------------------------------------------------ *
 * @brief   return gyro rate along the z axis           
 * @param   i2cfd
 *          i2c file descriptor
 * @return  gyro_z
 *          double
 * ------------------------------------------------------------ */
double get_gyro_z(int i2cfd){

    double gyro_z;

    char reg = BNO055_GYRO_DATA_Z_LSB_ADDR;
    if(write(i2cfd, &reg, 1) != 1) {
       printf("Error: I2C write failure in function get_gyro_z  for register 0x%02X\n", reg);
       return(-1);
    }

    char data[6] = {0};
    if(read(i2cfd, data, 6) != 6) {
          printf("Error: I2C read in function get_gyro_z failure for register data 0x%02X\n", reg);
          return(-1);
    }

    int16_t buf = ((int16_t)data[5] << 8) | data[4];
    //if(verbose == 1) printf("Debug: Gyroscope Data Z: LSB [0x%02X] MSB [0x%02X] INT16 [%d]\n", data[4], data[5],buf);
    
    gyro_z = (double) buf / angle_unit;
    if(verbose == 1) printf("gyro_z is: %f \n", gyro_z);

    return(gyro_z);

}

/* ------------------------------------------------------------ *
 * @brief   sets register page to 0           
 * @param   i2cfd
 *          i2c file descriptor
 * @return  0 if process is successful
 * ------------------------------------------------------------ */
int set_page0(int i2cfd) {

    char data[2] = {0};
   data[0] = BNO055_PAGE_ID_ADDR;
   data[1] = 0x0;
   if(verbose == 1) printf("Debug: write page-ID: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
   if(write(i2cfd, data, 2) != 2) {
      printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      return(-1);
   }
   return(0);
}

/* ------------------------------------------------------------ *
 * @brief   sets register page to 1           
 * @param   i2cfd
 *          i2c file descriptor
 * @return  0 if process is successful
 * ------------------------------------------------------------ */
int set_page1(int i2cfd) {
   char data[2] = {0};
   data[0] = BNO055_PAGE_ID_ADDR;
   data[1] = 0x1;
   if(verbose == 1) printf("Debug: write page-ID: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
   if(write(i2cfd, data, 2) != 2) {
      printf("Error: I2C write failure for register 0x%02X\n", data[0]);
      return(-1);
   }
   return(0);
}

/* ------------------------------------------------------------ *
 * @brief   sets an Gyroscope Any Motion Interrupt (datasheet p.48)
 *           on x axis with threshold  default value 0xA      
 * @param   i2cfd
 *          i2c file descriptor
 * @return  0 if process is successful
 * ------------------------------------------------------------ */
int set_interrupt(int i2cfd) {

    //must be  on page 0 to change mode
    set_page0(i2cfd);
    // must be in config mode
    char data[2] = {0};
    data[0] = BNO055_OPR_MODE_ADDR;
    data[1] = 0x0;
    if(verbose == 1) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
    if(write(i2cfd, data, 2) != 2) {
        printf("Error: I2C write failure for register 0x%02X\n", data[0]);
        return(-1);
    }
    /* --------------------------------------------------------- *
     * switch time: any->config needs 7ms + small buffer = 10ms  *
     * --------------------------------------------------------- */
    usleep(10 * 1000);

    //must be  on page 1 get interrupt registers
     set_page1(i2cfd);
    //Enable GYR AM NIT
    data[0] = BNO055_INT_ADDR;
     // bit 2 needs to be set for Gyro AM INT
    data[1] = 0x4;
    if(verbose == 1) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
    if(write(i2cfd, data, 2) != 2) {
            printf("Error: I2C write failure for register 0x%02X\n", data[0]);
            return(-1);
    }
    // route GYR AM INT to pin
    data[0] = BNO055_INT_MASK_ADDR;
    // bit 2 needs to be set for Gyro AM INT
    data[1] = 0x4;
    if(verbose == 1) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
    if(write(i2cfd, data, 2) != 2) {
            printf("Error: I2C write failure for register 0x%02X\n", data[0]);
            return(-1);
    }

    // specify which axis should be used
    data[0] = BNO055_GYRO_INTR_SETING_ADDR;
    // bit 0 needs to be set for x axis
    // bit 7 needs to be set for filtering
    data[1] = 0x81;
    if(verbose == 1) printf("Debug: Write opr_mode: [0x%02X] to register [0x%02X]\n", data[1], data[0]);
    if(write(i2cfd, data, 2) != 2) {
                printf("Error: I2C write failure for register 0x%02X\n", data[0]);
                return(-1);
    }

    set_page0(i2cfd);
    return 0;


}

/* ------------------------------------------------------------ *
 * function declaration for control loop                        *
 * ------------------------------------------------------------ */

/* ------------------------------------------------------------ *
 * calculate_feedback()             							*
 * patch function tries to approximate P Controll without model *
 * depends on 	abs(current_roll) - abs(last_roll)				*
 * ------------------------------------------------------------ */
float calculate_feedback(float k_p, float last_roll, float current_roll){

	float error_size = abs(last_roll) - abs(current_roll);

	float feedback_size = k_p;

	if(error_size = 0.0){

		feedback_size = 1.0;

	}else if(error_size < 0.0){

		feedback_size = 1.0 + k_p;
	
	}else{

		feedback_size = k_p - 1.0;
	}

	if(verbose == 1) printf("Set feedback_size to: %f \n", k_p);

	return feedback_size;

}

/* ------------------------------------------------------------ *
 * calculate_next_delta()             							*
 * place to implement the calculation of modeled delta 			*
 * now only return delta = current_roll	* feedback_size			*
 * ------------------------------------------------------------ */
float calculate_next_delta(float current_roll){

	float next_delta = 0.0;
    next_delta = current_roll;

	if(verbose == 1) printf("Set next_delta to: %f \n", next_delta);
	return next_delta;

}

/* ------------------------------------------------------------ *
 * calculate_delta_gain()             							*
 * actual proportional gain 			                        *
 * now only return delta = current_roll	* feedback_size			*
 * ------------------------------------------------------------ */
float calculate_delta_gain(float next_delta, float feedback_size){

	float next_angle = 0.0;
    next_angle = next_delta * feedback_size;

	if(verbose == 1) printf("Set next_angle to: %f \n", next_angle);
	return next_angle;

}
