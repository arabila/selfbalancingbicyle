#ifndef __AB_HNM_H__
#define __AB_HNM_H__


//typedef int bts7960_fd_t;
#include <sys/mman.h>

#include "ab_io.h"


typedef struct hnm_fd {
    pwm_fd_t  hnm_pwm;
    int     mem_fd;
    void    * map_base;
    off_t   target;
} hnm_fd_t;

// Opens the connection to the hnm. The pwm pin is initialized
void hnm_device_open(hnm_fd_t *fd, char *pwm_chip, char *pwm_channel, char *pwm_pin);

// sets the duty_cycle of the pwm signal to 0 and also sets enable_pwm to 0
void hnm_disable_pwm(hnm_fd_t *fd);

// only changes the enable_pwm to 1, but does not change the duty_cycle
void hnm_enable_pwm(hnm_fd_t *fd);

// sets the duty cycle of the pwm pin. has to be between 0 and 100000 ns
void hnm_set_pwm(hnm_fd_t *fd, int speed);

// sets duty_cycle and pwm enable to 0 and closes the hnm_fd
void hnm_device_close(hnm_fd_t *fd);


// enables the speed reading by starting the PRU which must be loaded with the pru_bike_speed
// code which counts the pulse changes to the pru0 input pin p8.15 and writes that to memory.
// This function creates a memory map so that the 32 bit unsigned integer value can be read
// by the hnm_speed function.
void hnm_enable_speed_reading(hnm_fd_t *fd);

// unmaps the memory stops the execution of the pru code and closes the file descriptor.
void hnm_disable_speed_reading(hnm_fd_t *fd);

// uses the pulses per turn, wheel diameter and pru sample time to calculate the bike back 
// wheel speed in m/s. If a value like the wheel diameter changes the constants in the 
// beginning of ab_hnm.h have to be changed and the code compiled again.
float hnm_speed(hnm_fd_t *fd);

// uses the pulses per turn and wheel diameter calculate the distance traveled by the bike 
// in m.
float hnm_distance(hnm_fd_t *fd);

#endif // __AB_HNM_H__