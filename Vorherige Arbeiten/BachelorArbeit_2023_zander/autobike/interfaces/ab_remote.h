#ifndef __AB_REMOTE_H__
#define __AB_REMOTE_H__

#include "ab_io.h"

typedef struct remote_fd {
    adc_fd_t a_in_steering;
    adc_fd_t a_in_throttle;
} remote_fd_t;

// open a connection to the remote
remote_fd_t remote_open(char* adc_pin_1, char* adc_pin_2);

// read out the steering value -2 to 2 for different values.
int remote_get_steering(remote_fd_t *fd);

// read out the throttle value -2 to 2 for different values.
int remote_get_throttle(remote_fd_t *fd);

//close the remote
void remote_close(remote_fd_t *fd);


#endif // __AB_REMOTE_H__