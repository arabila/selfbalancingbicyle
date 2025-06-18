#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>

#include "../interfaces/ab_hnm.h"
#include "../interfaces/ab_pid.h"
#include "../interfaces/ab_remote.h"

hnm_fd_t testi_hnm;

void safe_exit(){
    system("/var/lib/cloud9/fahrradprojekt/autobike/all_pin_default.sh");
    printf("Error occured, pins have been reset.");
    fflush(stdout);
}


void closing_all_devices(){
    printf("Closing of all devices started.");
    fflush(stdout);
    hnm_device_close(&testi_hnm);
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
    
    time_t local_time_initializer;
    struct tm local_time;
    struct timeval precise_time;
   
    remote_fd_t test_remote = remote_open("1", "0");
    
    
    printf("enableing speed reading\n");
    fflush(stdout);
    hnm_enable_speed_reading(&testi_hnm);
    sleep(1);
    printf("speed reading enabled\n");
    fflush(stdout);
        
    hnm_device_open(&testi_hnm, "4", "0", "p9.14");
    // seems to work alright although it is pretty slow
    // pid_val_t testi_hnm_pid = pid_open(2500.0f, 7000.0f, -180.0f, 0.0f, 60000.0f, 0.0f, 60000.0f);
    pid_val_t testi_hnm_pid = pid_open(2500.0f, 7000.0f, -180.0f, 0.0f, 60000.0f, 0.0f, 60000.0f);
    // pid_val_t testi_hnm_pid = pid_open(70000.0f, 2000.0f, 2000.0f, 0.0f, 60000.0f, 0.0f, 60000.0f);

    hnm_set_pwm(&testi_hnm, 31000);
    int testi_pwm = 0;
    float desired_speed = 0.0f;
    
    local_time_initializer = time(NULL);
    local_time = *localtime(&local_time_initializer);
    gettimeofday(&precise_time, NULL);
    
    struct timeval time_value;
    long long ms_since_start;
    
    
    for(int i = 0;i<100;i++){
        if(remote_get_throttle(&test_remote) == 1){
            desired_speed = 4.0f;
        } else if(remote_get_throttle(&test_remote) == 2){
            desired_speed = 7.0f;
        } else{
            desired_speed = 0.0f;
        }
        
        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));
        
        
        testi_pwm = pid_output(&testi_hnm_pid, desired_speed, hnm_speed(&testi_hnm), ms_since_start);
        hnm_set_pwm(&testi_hnm, testi_pwm);
        printf("speed of bike: %4.3f m/s\n", hnm_speed(&testi_hnm));
        fflush(stdout);
        usleep(200000);
    }
    hnm_disable_speed_reading(&testi_hnm);
    hnm_device_close(&testi_hnm);
    
    return 0;

}