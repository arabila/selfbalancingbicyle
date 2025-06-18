#include "ab_remote.h"

remote_fd_t remote_open(char* adc_pin_1, char* adc_pin_2){
    remote_fd_t fd;
    fd.a_in_steering = adc_open(adc_pin_1);
    fd.a_in_throttle = adc_open(adc_pin_2);
    return fd;
}

int remote_get_steering(remote_fd_t *fd){
    int raw_steering_value = adc_get_raw(&fd->a_in_steering);
    if(raw_steering_value > 0 && raw_steering_value < 200){
        return -1; // the controller is not connected
    }
    else if(raw_steering_value > 1000 && raw_steering_value < 1200){
        return 1; // the controller steers left
    }
    else if(raw_steering_value > 450 && raw_steering_value < 650){
        return 2; // the controller steers right
    }
    else if(raw_steering_value > 650 && raw_steering_value < 1000){
        return 0; // the controller steers straight
    }
    else{
        return -2; // impossible reading.
    }
}

int remote_get_throttle(remote_fd_t *fd){
    int raw_throttle_value = adc_get_raw(&fd->a_in_throttle);
    if (remote_get_steering(fd)<0){
        return -1; // remote is not connected
    } else{
        if(raw_throttle_value > 1000 && raw_throttle_value < 1200){
            return 1; // the controller throttle
        }
        else if(raw_throttle_value > 450 && raw_throttle_value < 650){
            return 2; // the controller reverse
        }
        else if(raw_throttle_value > 650 && raw_throttle_value < 1000){
            return 0; // the controller no throttle
        }
        else{
            return -2; // impossible reading.
        }
    }
}







void remote_close(remote_fd_t *fd){
    adc_close(&fd->a_in_steering);
    adc_close(&fd->a_in_throttle);
}