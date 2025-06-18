#ifndef __AB_BTS7960_H__
#define __AB_BTS7960_H__

#include "ab_io.h"
//To turn the motor both the left and the right enable bin have to be set to 
//High.

//This module has all necessary functions to handle the bts7960 motorcontroller. 

//typedef int bts7960_fd_t;
typedef struct bts7960_fd {
    gpio_fd_t       left_enable;
    gpio_fd_t       right_enable;
    pwm_fd_t        left_pwm;
    pwm_fd_t        right_pwm;
} bts7960_fd_t;

// this function opens and starts the device. since the bts7960 needs two gpio signals and 
// two pwm pins, the connected ones have to be set upon function call.
void bts7960_device_open(bts7960_fd_t *fd, char *left_enable_pin, char *right_enable_pin,
                         char *left_pwm_chip, char *left_pwm_channel, char *left_pwm_pin,
                         char *right_pwm_chip, char *right_pwm_channel, char *right_pwm_pin);

//Positive speeds are a right turn and negative speeds are a left turn. The speed value has to be
//between -100000 and 100000 which basically set the duty cycle of either the left or the right
//pwm pin. 
void bts7960_turn(bts7960_fd_t *fd, int speed);

// sets the values of the pwm pins and pwm_enable to 0. It does not disable the gpio pins.
void bts7960_disable_all(bts7960_fd_t *fd);

// closes the device and disables gpio pins and pwm pins.
void bts7960_device_close(bts7960_fd_t *fd);


#endif // __AB_BTS7960_H__