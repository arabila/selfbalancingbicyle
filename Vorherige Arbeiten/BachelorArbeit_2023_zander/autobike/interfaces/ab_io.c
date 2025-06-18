#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <sys/time.h>


#include "ab_io.h"

#define MAX_TEXT_LENTGH     64 //the max number of bytes that is being written to a fd.

#define PWM_CHIP_EXPORT     "/sys/class/pwm/pwmchip%s/export" //number of the chip missing
#define PWM_PATH            "/sys/class/pwm/pwm-%s:%s" // number of the chip and number of the pwm pin
#define PWM_PERIOD          "/period"
#define PWM_10KH            "100000" //period in nanoseconds
#define PWM_DUTY_CYCLE      "/duty_cycle"
#define PWM_ENABLE          "/enable"
#define PWM_CONFIG_PIN      "config-pin %s pwm"

#define GPIO_PATH           "/sys/class/gpio/gpio%s" //number of the pin missing
#define GPIO_VALUE          "/value"
#define GPIO_OUT            "out"
#define GPIO_IN             "in"
#define GPIO_DIRECTION      "/direction"

#define ADC_PATH            "/sys/bus/iio/devices/iio:device0/in_voltage%s_raw" //number of the adc unit has to be given (0-7)

#define SAFE_WRITE(fd, data, data_len, error)   if(write(fd, data, data_len) != data_len){ \
                                                   AB_ERROR(error, "Failed to write.", AB_ERR_ABORT);}
#define SAFE_OPEN(fd, path, error) if((fd = open(path, O_WRONLY | O_NOCTTY)) < 0){ \
                                        AB_ERROR(error, "Failed to open.", AB_ERR_ABORT);}

long long last_time_value = 1500;



void write_to_file(char *path, char *data, int error)
{
    int fd;
    SAFE_OPEN(fd, path, error);
    
    int data_size = strlen(data);
    
    SAFE_WRITE(fd, data, data_size, error);
    
    close(fd);
}


void pwm_open(pwm_fd_t *fd, char *pwm_chip, char *pwm_channel, char *pwm_pin, int pwm_period)
{
    char path_buf[MAX_TEXT_LENTGH];
    char data_buf[MAX_TEXT_LENTGH];
    
    //configuring the Beaglebone Black pin.
    sprintf(data_buf, PWM_CONFIG_PIN, pwm_pin);
    system(data_buf);
    
    
    // setting the period of the pwm and enabling it.
    sprintf(path_buf, PWM_PATH PWM_PERIOD, pwm_chip, pwm_channel);
    sprintf(data_buf, "%d", pwm_period);
    write_to_file(path_buf, data_buf, -100);


    sprintf(path_buf, PWM_PATH PWM_ENABLE, pwm_chip, pwm_channel);
    SAFE_OPEN(fd->pwm_enable, path_buf, -102);
    SAFE_WRITE(fd->pwm_enable, "1", 1, -103);

    
    sprintf(path_buf, PWM_PATH PWM_DUTY_CYCLE, pwm_chip, pwm_channel);
    SAFE_OPEN(fd->pwm_duty_cycle, path_buf, -104);
    SAFE_WRITE(fd->pwm_duty_cycle, "0", 1, -105);

}

void pwm_disable(pwm_fd_t *fd){
    SAFE_WRITE(fd->pwm_duty_cycle, "0", 1, -106);
    SAFE_WRITE(fd->pwm_enable, "0", 1, -107);
}

void pwm_enable(pwm_fd_t *fd){
    SAFE_WRITE(fd->pwm_enable, "1", 1, -108);
}



void pwm_set_duty_cycle(pwm_fd_t *fd, int duty_cycle)
{
    char data_buf[MAX_TEXT_LENTGH];
    sprintf(data_buf, "%d", duty_cycle);
    int data_size = strlen(data_buf);
    
    SAFE_WRITE(fd->pwm_duty_cycle, data_buf, data_size, -109);
}


void pwm_close(pwm_fd_t *fd){
    pwm_disable(fd);
    close(fd->pwm_duty_cycle);
    close(fd->pwm_enable);
}

//#################### GPIO SECTION #########################


gpio_fd_t gpio_open(char *gpio_pin, char *gpio_direction){
    char path_buf[MAX_TEXT_LENTGH];
    char data_buf[MAX_TEXT_LENTGH];
    gpio_fd_t fd;
    
    if (strcmp(gpio_direction, "out") == 0){
        sprintf(path_buf, GPIO_PATH GPIO_DIRECTION, gpio_pin);
        sprintf(data_buf, gpio_direction);
        write_to_file(path_buf, data_buf, -200);
        
        sprintf(path_buf, GPIO_PATH GPIO_VALUE, gpio_pin);
        SAFE_OPEN(fd.gpio_value, path_buf, -201);
        fd.direction = "out";
        return fd;
    } else if (strcmp(gpio_direction, "in") == 0){
        sprintf(path_buf, GPIO_PATH GPIO_DIRECTION, gpio_pin);
        sprintf(data_buf, gpio_direction);
        write_to_file(path_buf, data_buf, -200);
        
        sprintf(path_buf, GPIO_PATH GPIO_VALUE, gpio_pin);
        
        if((fd.gpio_read = open(path_buf, O_RDONLY)) < 0){
            AB_ERROR(-200, "Failed to open.", AB_ERR_ABORT);
        }
        fd.direction = "in";
        return fd;
    } else{
        AB_ERROR(-200, "Failed to initiate gpio pin.", AB_ERR_ABORT);
    }
    
}

void gpio_enable(gpio_fd_t *fd){
    if(strcmp(fd->direction, "out") != 0){
        AB_ERROR(-202, "Failed to enable pin.", AB_ERR_ABORT);
    }
    SAFE_WRITE(fd->gpio_value, "1", 1, -202);
}

void gpio_disable(gpio_fd_t *fd){
    if(strcmp(fd->direction,"out") != 0){
        AB_ERROR(-202, "Failed to enable pin.", AB_ERR_ABORT);
    }
    SAFE_WRITE(fd->gpio_value, "0", 1, -203);
}

int gpio_in_read(gpio_fd_t *fd){
    
    if(strcmp(fd->direction,"in") != 0){
        AB_ERROR(-202, "Failed to enable pin.", AB_ERR_ABORT);
    }
    char value;
    lseek(fd->gpio_read, 0, SEEK_SET);
    if (read(fd->gpio_read, &value, 1) != 1){
        AB_ERROR(-202, "Failed to read from pin.", AB_ERR_ABORT);
    }
    
    if (value == '1') {
        return 1;
    } else if(value == '0') {
        return 0;
    } else {
        return -1;
    }
}


void gpio_close(gpio_fd_t *fd){
    
    if (strcmp(fd->direction, "out") == 0){
        gpio_disable(fd);
        close(fd->gpio_value);
    } else if (strcmp(fd->direction, "in") == 0){
        close(fd->gpio_read);
    }
}


//#################### ADC SECTION #########################


adc_fd_t adc_open(char * adc_pin){

    char path_buf[MAX_TEXT_LENTGH];
    char data_buf[MAX_TEXT_LENTGH];
    adc_fd_t fd;
    
    sprintf(path_buf, ADC_PATH, adc_pin);

    if((fd.adc_file = open(path_buf, O_RDONLY)) < 0){
        AB_ERROR(-400, "Failed to open.", AB_ERR_ABORT);
    }
    
    return fd;
}

int adc_get_raw(adc_fd_t *fd){
    char value[7] = {'\0'};
    lseek(fd->adc_file, 0, SEEK_SET);
    if (read(fd->adc_file, &value, 6) < 1 ){
        AB_ERROR(-202, "Failed to read adc from pin.", AB_ERR_ABORT);
    }
    return atoi(value);
}

void adc_close(adc_fd_t *fd){
    close(fd->adc_file);
}