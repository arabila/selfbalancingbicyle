/* ---------------------------------------------------------------*
 * file:        imutest.h                                         *
 * purpose:     testing the bno055                                *
 *                                                                *
 * return:      0 on success, and -1 on errors.                   *
 *                                                                *
 * requires:    I2C headers (libi2c-dev)                          *
 *          GPIOD header (libgpiod-dev)                           *
 *                                                                *
 * compile:    gcc -o imutest imutest.c -lm -lgpiod               *
 *                                                                *
 * author:      agr                                               *
 * ---------------------------------------------------------------*/

#ifndef IMUTEST_H_
#define IMUTEST_H_

#define I2CBUS                  "/dev/i2c-2"

/* ------------------------------------------------------------ *
 * Bosch BNO0555 Registers                                      *
 * ------------------------------------------------------------ */

#define BNO055_CHIP_ID_ADDR     0x00
#define BNO055_MODE_IMU         0x08

/* Page ID register, for page switching */
#define BNO055_PAGE_ID_ADDR  0x07

#define BNO055_OPR_MODE_ADDR    0x3D
#define BNO055_PWR_MODE_ADDR    0x3E

#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_TEMP_SOURCE_ADDR 0x40

/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR      0x1A
#define BNO055_EULER_H_MSB_ADDR      0x1B
#define BNO055_EULER_R_LSB_ADDR      0x1C
#define BNO055_EULER_R_MSB_ADDR      0x1D
#define BNO055_EULER_P_LSB_ADDR      0x1E
#define BNO055_EULER_P_MSB_ADDR      0x1F

/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR     0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR     0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR     0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR     0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR     0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR     0x19

/* Registers to set Gyrosscope Any Motion Interrupt */
/* Interrupt registers*/
#define BNO055_INT_MASK_ADDR                0X0F
#define BNO055_INT_ADDR                     0X10
#define BNO055_GYRO_INTR_SETING_ADDR        0X17
#define BNO055_GYRO_ANY_MOTION_THRES_ADDR   0X1E
#define BNO055_GYRO_ANY_MOTION_SET_ADDR     0X1F

/*Interrupt status registers*/
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_POS      2
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_MSK      0X04
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_LEN      1
#define BNO055_INTR_STAT_GYRO_ANY_MOTION_REG      BNO055_INTR_STAT_ADDR

/* Axis mapping registers and value for remap P0*/
#define BNO055_AXIS_REMAP_CONFIG_ADDR     0x41
#define BNO055_AXIS_REMAP_SIGN_ADDR       0x42
#define BNO055_AXIS_CONFIG                0x21
#define BNO055_AXIS_SIGN                  0x04

/* ------------------------------------------------------------ *
 * global variables                                             *
 * ------------------------------------------------------------ */
//int i2cfd;       // I2C file descriptor
int verbose;     // debug flag, 0 = normal, 1 = debug mode

struct euler_angles{
   double eul_head;  // Euler heading data
   double eul_roll;  // Euler roll data
   double eul_pitc;  // Euler picth data
};


/* ------------------------------------------------------------ *
 * Operations and power mode, name to value translation         *
 * ------------------------------------------------------------ */
typedef enum {
   config   = 0x00,
   acconly  = 0x01,
   magonly  = 0x02,
   gyronly  = 0x03,
   accmag   = 0x04,
   accgyro  = 0x05,
   maggyro  = 0x06,
   amg      = 0x07,
   imu      = 0x08,
   compass  = 0x09,
   m4g      = 0x0A,
   ndof     = 0x0B,
   ndof_fmc = 0x0C
} operation_mode_t;

typedef enum {
   normal  = 0x00,
   low     = 0x01,
   suspend = 0x02
} power_mode_t;

#endif /* IMUTEST_H_ */
