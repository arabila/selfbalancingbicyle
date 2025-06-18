#ifndef __AB_IO_H__
#define __AB_IO_H__

#include <stdio.h>
#include <stdlib.h>
#include "../ab_error_handling.h"


typedef struct pwm_fd {
    int       pwm_duty_cycle;
    int       pwm_enable;
} pwm_fd_t;

typedef struct gpio_fd {
    int gpio_value;
    int gpio_read;
    char *direction;
} gpio_fd_t;

typedef struct adc_fd {
    int adc_file;
} adc_fd_t;


// writes the given data to the path. gives an error if path couldn't be opened
// or the data wasn't able to be written to that path.
void write_to_file(char *path, char *data, int error);

//opens the connection to one pwm pin. pwm pin is important for pwm chips that can send pwm
// over different pins of the Beaglebone Black. pwm enable will be 1 but duty cycle 0.
void pwm_open(pwm_fd_t *fd, char *pwm_chip, char *pwm_channel, char *pwm_pin, int pwm_period);

//Sets the pwm enable as well as the duty cycle to 0.
void pwm_disable(pwm_fd_t *fd);

//Only sets pwm enable to 1.
void pwm_enable(pwm_fd_t *fd);

//pwm duty cycle will be set to duty_cycle. pwm enable has to be set to 1.
void pwm_set_duty_cycle(pwm_fd_t *fd, int duty_cycle);

//pin is disabled and the duty cycle is set to 0.
void pwm_close(pwm_fd_t *fd);

//#################### GPIO SECTION #########################

// sets the direction of the gpio pin and saves the path of the gpio pin value as it's fd.
// the value is not initialized.
gpio_fd_t gpio_open(char *gpio_pin, char *gpio_direction);

// sets the value of the gpio pin to 1. This function does not yet check if a pin is an 
// output or input pin.
void gpio_enable(gpio_fd_t *fd);

// sets the value of the gpio pin to 0. This function does not yet check if a pin is an 
// output or input pin.
void gpio_disable(gpio_fd_t *fd);

// writes 0 to the value of the pin regardles of input or output mode. The gpio_fd is then closed.
// The direction of the pin stays unchanged.
void gpio_close(gpio_fd_t *fd);

// returns either 0 or 1 depending on the value of the pin
int gpio_in_read(gpio_fd_t *fd);


//#################### ADC SECTION #########################
// opens the connection to the pin analog 0-7.
adc_fd_t adc_open(char * adc_pin);

// reads the raw value of the analog signal max voltage is 1.8V and the precision is 12 bit
// that gives a range of 0-4095.
int adc_get_raw(adc_fd_t *fd);

// closes the fd to the raw input of the pin.
void adc_close(adc_fd_t *fd);


#endif // __AB_IO_H__