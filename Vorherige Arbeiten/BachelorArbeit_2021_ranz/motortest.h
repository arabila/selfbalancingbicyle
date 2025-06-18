/* ---------------------------------------------------------------*
 * file:        motortest.c                                       *
 * purpose:     motortest for the EM49 DC motor                   *
 *                and its controller board MD49                   *
 *                                                                *
 * return:      0 on success, and -1 on errors.                   *
 *                                                                *
 * requires:    GPIOD header (libgpiod-dev)                       *
 *                                                                *
 *                                                                *
 * compile:    gcc -o motortest motortest.c -lm -lgpiod           *
 *                                                                *
 * author:      agr                                               *
 * ---------------------------------------------------------------*/

#ifndef MOTORTEST_H_
#define MOTORTEST_H_

// if using pins p9.11 p9.13
#define UART                    "/dev/ttyO4"

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

// struct euler_angles{
//    double eul_head;  // Euler heading data
//    double eul_roll;  // Euler roll data
//    double eul_pitc;  // Euler picth data
// };


/* ------------------------------------------------------------ *
 * diretion name to value translation                           *
 * ------------------------------------------------------------ */

typedef enum {
   clockwise = -1,  // needs negative speed
   counterclockwise = 1, // needs positiv speed
} motor_direction_t;

typedef struct{
    motor_direction_t direction;
}motor_direction;

#endif /* MOTORTEST_H_ */
