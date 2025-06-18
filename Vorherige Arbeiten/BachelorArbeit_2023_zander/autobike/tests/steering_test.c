#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <time.h>
#include <sys/time.h>

#include "../interfaces/ab_bts7960.h"
#include "../interfaces/ab_md49.h"
#include "../interfaces/ab_pid.h"
#include "../ab_error_handling.h"


// thread md49 read variables.
uart_fd_t md49_fd;
bts7960_fd_t bts_fd;


void safe_exit(){
    system("/var/lib/cloud9/fahrradprojekt/autobike/all_pin_default.sh");
    printf("Error occured, pins have been reset.");
    fflush(stdout);
}


void closing_all_devices(){
    printf("Closing of all devices started.");
    fflush(stdout);
    bts7960_device_close(&bts_fd);
    md49_device_close(md49_fd);
    printf("all devices closed successfully.");
    fflush(stdout);
}


int main(int argc, char* argv[])
{

    if(atexit(safe_exit) != 0){
        printf("Exit handler failed.");
        fflush(stdout);
        return -1;
    }
    
    
    if (signal(SIGINT, closing_all_devices) == SIG_ERR){
        printf("Signal handler failed.");
        fflush(stdout);
        
        return -1;
    }

    // creating the necessary logging files.
    time_t local_time_initializer;
    struct tm local_time;
    struct timeval precise_time;
    
    float md49_setpoint;


    local_time_initializer = time(NULL);
    local_time = *localtime(&local_time_initializer);
    gettimeofday(&precise_time, NULL);
    
    pid_val_t steering_pid;

    md49_fd = md49_device_open("/dev/ttyO4", "p9.11", "p9.13");
    
    bts7960_device_open(&bts_fd, "49", "49", "1", "1", "p9.22", "1", "0", "p9.21");
    
    steering_pid = pid_open(1000.0, 0.0, 0.0, -100000, 100000, -3000, 3000);


    int encoder_value;
    int new_steering_value;
    int setpoint_change = 1;

    struct timeval time_value;
    long long ms_since_start;
    
    for(int i=0; i<1500; i++){
        // get the encoder value
        encoder_value = -1 * md49_get_encoder1(md49_fd); // negative to enable smooth operation with pid controller.
        if (encoder_value>400 || encoder_value<-400){
            bts7960_device_close(&bts_fd);
        }
        
        if(!(md49_setpoint == setpoint_change * 100)){
            md49_setpoint = md49_setpoint + setpoint_change;
        }
        
        if (i%300 == 0){
            setpoint_change = setpoint_change * -1;  
            // if (md49_setpoint==100){
            //     md49_setpoint = -100;
            // } else{
            //     md49_setpoint =  100;
            // }
        }
        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));
        
        md49_setpoint;

        new_steering_value = (int)pid_output(&steering_pid, md49_setpoint, (float)encoder_value, ms_since_start);
        bts7960_turn(&bts_fd, new_steering_value);
        
 
        
        // writing the values
        printf("time: %lld, encoder: %d, encoder setpoint: %4.1f, steering value: %d\n",
                ms_since_start,
                encoder_value,
                md49_setpoint,
                new_steering_value);
        fflush(stdout);
        usleep(10000);
    }
    
    bts7960_device_close(&bts_fd);
    md49_device_close(md49_fd);
    
    return 0;
}