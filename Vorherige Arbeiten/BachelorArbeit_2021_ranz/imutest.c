/* ---------------------------------------------------------------*
 * file:        imutest.c                                         *
 * purpose:     testing the bno055                                *
 *                                                                *
 * return:      0 on success, and -1 on errors.                   *
 *                                                                *
 * requires:    I2C headers (libi2c-dev)                          *
 *              GPIOD header (libgpiod-dev)                           *
 *                                                                *
 * compile:    gcc -o imutest imutest.c -lm -lgpiod               *
 *                                                                *
 * author:      agr                                               *
 * ---------------------------------------------------------------*/

// referenced datasheet at https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

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



#include "imutest.h"


/* ------------------------------------------------------------ *
 * function prototypes for BNO055                               
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
 * global variables and constants                               
 * ------------------------------------------------------------ */
int verbose = 1; 						// used for printf debugging
char sensor_address[256] = "0x28";		// address of the bno055
char i2c_bus[256] = I2CBUS;				// get the used i2c device from bicycle.h
float angle_unit = 900.0;                // defines the angle unit (datasheet p.35)
                                        // set 16.0 for degrees, 900.0 for rad/s


/* ------------------------------------------------------------ *
 * @brief   sets up uart, i2c and starts the main control loop    
 * @return  0 if process is successful                        
 * ------------------------------------------------------------ */
int main(int argc, char *argv[]){

	// file descriptors for communication with sensor and motor controller
    int i2c_fd = 0;
    char mode = 0;

    // set up i2c to talk to bno055
    i2c_fd = get_i2c(i2c_bus, sensor_address);


    int res = -1;
    int toggle = 30;


    float last_roll = 0.0;
    double gyro_x = 0.0;
    float current_roll = 0.0;
    float current_pitch = 0.0;
    float current_yaw = 0.0;
    double current_gyro_x = 0.0;
    double current_gyro_y = 0.0;
    double current_gyro_z = 0.0;
    float threshold = 45.0;
    set_page0(i2c_fd);

    //remap axes to PO
    set_axis_mode(i2c_fd);

    // set operation mode of bno055 to imu
    operation_mode_t new_mode;
    new_mode = imu;
   	res = set_mode(i2c_fd, new_mode);
    

    do{
    	current_roll = get_roll(i2c_fd);
        current_pitch = get_pitch(i2c_fd);
        current_yaw = get_yaw(i2c_fd);
        current_gyro_x = get_gyro_x(i2c_fd);
        current_gyro_y = get_gyro_y(i2c_fd);
        current_gyro_z = get_gyro_z(i2c_fd);

    }while(toggle >= 0);

    return(res);

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
