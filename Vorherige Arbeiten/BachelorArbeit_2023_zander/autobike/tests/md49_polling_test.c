#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <time.h>
#include <sys/time.h>

#include "../interfaces/ab_md49.h"
#include "../ab_error_handling.h"


// thread md49 read variables.
uart_fd_t md49_fd;


void closing_all_devices(){
    printf("Closing of all devices started.");
    fflush(stdout);
    md49_device_close(md49_fd);
    printf("all devices closed successfully.");
    fflush(stdout);
}


int main(int argc, char* argv[])
{
    
    
    if (signal(SIGINT, closing_all_devices) == SIG_ERR){
        printf("Signal handler failed.");
        fflush(stdout);
        
        return -1;
    }

    // creating the necessary logging files.
    struct timeval precise_time;
    struct timeval time_value;
    long long ms_since_start;   
    gettimeofday(&precise_time, NULL);


    md49_fd = md49_device_open("/dev/ttyO4", "p9.11", "p9.13");
    
    int encoder_value;


    printf("time, encoder\n");
    for(int i=0; i<5; i++){
        // get the encoder value
        encoder_value = -1 * md49_get_encoder1(md49_fd); // negative to enable smooth operation with pid controller.

        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));
 
        
        // writing the values
        printf("%lld, %d\n",
                ms_since_start,
                encoder_value);
        fflush(stdout);
    }

    md49_device_close(md49_fd);
    return 0;
}