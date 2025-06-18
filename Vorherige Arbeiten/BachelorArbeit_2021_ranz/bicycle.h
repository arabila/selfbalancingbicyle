/* ---------------------------------------------------------------*
 * file:        bicycle.c                                         *
 * purpose:     central programm to balance a bicycle using the   *
 *           BNO055 sensor and steering via the EM49 DC motor     *
 *           and its controller board MD49                        *
 *                                                                *
 * return:      0 on success, and -1 on errors.                   *
 *                                                                *
 * requires:    I2C headers (libi2c-dev)                          *
 *          GPIOD header (libgpiod-dev)                           *
 *                                                                *
 * compile:    gcc -o bicycle bicycle.c -lm -lgpiod               *
 *                                                                *
 * author:      agr                                               *
 * ---------------------------------------------------------------*/

#ifndef BICYCLE_H_
#define BICYCLE_H_

#define I2CBUS                  "/dev/i2c-2"
#define UART                    "/dev/ttyO4"

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
 * MD49 Registers                                      *
 * ------------------------------------------------------------ */
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

typedef enum {
   clockwise = -1,  // needs negative speed
   counterclockwise = 1, // needs positiv speed
} motor_direction_t;

typedef struct{
    motor_direction_t direction;
}motor_direction;

#endif /* BICYCLE_H_ */
