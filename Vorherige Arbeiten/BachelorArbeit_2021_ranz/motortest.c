/* ---------------------------------------------------------------*
 * file:        motortest.c                                       *
 * purpose:     motortest for the EM49 DC motor                   *
 *                and its controller board MD49                   *
 *                                                                *
 * return:      0 on success, and -1 on errors.                   *
 *                                                                *
 * requires:    GPIOD header (libgpiod-dev)                       *
 *                                                                *
 * compile:    gcc -o motortest motortest.c -lm -lgpiod           *
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
#include<math.h>
#include<gpiod.h>


#include "motortest.h"

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
int set_MD49_steering_angle(int, float);
int drive_MD49_motor_until_encoder(int, int);
int calc_MD49_encoder_from_angle(float);
int reset_MD49_encoder(int);
int get_MD49_encoder(int);
float calculate_feedback(float, float, float);
float calculate_next_angle(float, float);


/* ------------------------------------------------------------ *
 * global variables and constants                               *
 * ------------------------------------------------------------ */
int verbose = 1; 						// used for printf debugging
char port_Name[256] = UART;			    // get the used uart device from motortest.h
char serialBuffer[10] = {0};  			// serial buffer for UART
int direction = 1;						// clockwise = -1 needs negative speed
										// counterclockwise = 1, needs positive speed
float delta = 0;                        // current steering angle
signed char speed = 0;                  // current speed

const float encoder_ratio = 980.0;     	// Encoder counts per output shaft turn
const float gears_motor = 30.0;			// number of teeth input gear
const float gears_steering = 44.0;		// number of teeth output gear
const float full_circle = 360.0;
const float encoder_to_angle = full_circle/(encoder_ratio * (gears_steering/gears_motor));
const float steering_to_encoder_ratio = ((encoder_ratio * (gears_steering/gears_motor)) / full_circle);

union encoder_32 {						// to store the encoder values
    int value;
    char bytes[4];
} current_encoder, last_encoder;

int32_t encoder_result;


/* ------------------------------------------------------------ *
 * @brief   sets up uart, i2c and starts the main control loop    
 * @return  0 if process is successful                        
 * ------------------------------------------------------------ */
int main(int argc, char *argv[]){

	// file descriptors for communication with motor controller
    int uart_fd = 0;
    float encoder_test = 0.0;
    float steering_angle_test = 45.0;
    char mode = 0;

    // set up i2c to talk to MD49
    uart_fd = set_up_uart(uart_fd);


    int res = -1;

    // settin motor mode to mode 1: -128(full speed reverse), 0(stop), +128(full forward)
    res = set_MD49_motor_mode(uart_fd, 0x01);
    usleep(1000);
    mode = get_MD49_motor_mode(uart_fd);

    speed = 50;
    res = reset_MD49_encoder(uart_fd);

    encoder_test = 175.0;


    
    // to test forward and backward driving

    ///* 
    drive_MD49_motor(uart_fd);
    usleep(1000000);
    speed = - speed;
    drive_MD49_motor(uart_fd);
    usleep(1000000);
    speed = - speed;
    //*/

    // to test get encoder

    //*
    usleep(1000000);
    res = reset_MD49_encoder(uart_fd);
    res = get_MD49_encoder(uart_fd);
    drive_MD49_motor(uart_fd);
    usleep(1000);
    res = get_MD49_encoder(uart_fd);
    speed = - speed;
    res = reset_MD49_encoder(uart_fd);
    drive_MD49_motor(uart_fd);
    usleep(1000);
    res = get_MD49_encoder(uart_fd);
    res = reset_MD49_encoder(uart_fd); 
    //*/

    // to test drive until encoder
         
    //*
    usleep(1000000);
    res = drive_MD49_motor_until_encoder(uart_fd, encoder_test);
    encoder_test = - encoder_test;
    speed = - speed;
    res = drive_MD49_motor_until_encoder(uart_fd, 0.0);
    encoder_test = - encoder_test;
    speed = - speed;
    res = drive_MD49_motor_until_encoder(uart_fd, encoder_test);
    encoder_test = - encoder_test;
    speed = - speed;
    res = drive_MD49_motor_until_encoder(uart_fd, encoder_test);
    encoder_test = - encoder_test;
    speed = - speed;  
    //*/

    // to test set steering angle
        
    ///*
    res = reset_MD49_encoder(uart_fd);
    res = set_MD49_steering_angle(uart_fd, steering_angle_test);
    res = get_MD49_steering_angle(uart_fd);
    usleep(1000000);
    res = set_MD49_steering_angle(uart_fd, 0.0);
    res = get_MD49_steering_angle(uart_fd);
    usleep(1000000);
    res = set_MD49_steering_angle(uart_fd, - steering_angle_test);
    res = get_MD49_steering_angle(uart_fd);
    //*/

    stop_MD49_motor(uart_fd);
    usleep(1000000);
         

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
int set_MD49_steering_angle(int file_descriptor, float next_steering_angle){

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
    res = drive_MD49_motor_until_encoder(file_descriptor, next_encoder);
    
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
int drive_MD49_motor_until_encoder(int file_descriptor, int limit){

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
