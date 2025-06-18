#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "ab_bts7960.h"



void bts7960_device_open(bts7960_fd_t *fd, char *left_enable_pin, char *right_enable_pin,
                         char *left_pwm_chip, char *left_pwm_channel, char *left_pwm_pin,
                         char *right_pwm_chip, char *right_pwm_channel, char *right_pwm_pin)
{
    fd->left_enable = gpio_open(left_enable_pin, "out");
    gpio_enable(&fd->left_enable);
    
    fd->right_enable = gpio_open(right_enable_pin, "out");
    gpio_enable(&fd->right_enable);
    
    pwm_open(&fd->left_pwm, left_pwm_chip, left_pwm_channel, left_pwm_pin, 100000);
    pwm_open(&fd->right_pwm, right_pwm_chip, right_pwm_channel, right_pwm_pin, 100000);
}



void bts7960_turn(bts7960_fd_t *fd, int speed)
{
    if(speed>0){
        pwm_disable(&fd->left_pwm);
        pwm_enable(&fd->right_pwm);
        pwm_set_duty_cycle(&fd->right_pwm, speed);
    }else{
        pwm_disable(&fd->right_pwm);
        pwm_enable(&fd->left_pwm);
        pwm_set_duty_cycle(&fd->left_pwm, abs(speed));
    }
}

void bts7960_disable_all(bts7960_fd_t *fd){
    pwm_disable(&fd->right_pwm);
    pwm_disable(&fd->left_pwm);
}

void bts7960_device_close(bts7960_fd_t *fd){
    bts7960_disable_all(fd);
    pwm_close(&fd->left_pwm);
    pwm_close(&fd->right_pwm);
    gpio_close(&fd->left_enable);
    gpio_close(&fd->right_enable);
}