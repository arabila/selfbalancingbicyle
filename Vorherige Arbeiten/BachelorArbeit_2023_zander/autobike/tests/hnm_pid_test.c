#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

//#include "libpruio/pruio.h"
#include <time.h>
#include <sys/time.h>


#include "../interfaces/ab_hnm.h"
#include "../interfaces/ab_pid.h"
#include "../ab_error_handling.h"

#define LOGGING
#define LOGGING_FILE_PATH "/var/lib/cloud9/fahrradprojekt/autobike/logging/logging_files/"
#define PIN_DEFAULT_FILE_PATH "/var/lib/cloud9/fahrradprojekt/autobike/all_pin_default.sh"


// creating the necessary logging files.
time_t local_time_initializer;
struct tm local_time;
struct timeval precise_time;


// general global variables
hnm_fd_t hnm_fd; // used for closing connection in the closing all devices function
pid_val_t hnm_pid;
float setpoint_speed;


void * thread_hnm_read(void * p){
        //return;
    usleep(200000); // a small delay for the pru do read the speed.
    
    int remote_speed_copy;
    float speed_value;
    int new_pwm_value;
    
    #ifdef LOGGING
    //create logging file
    char filename[128];
    strftime(filename, sizeof(filename), LOGGING_FILE_PATH "%Y-%m-%d__%H-%M-%S_hnm_thread.csv", &local_time);
    FILE *logging_file = fopen(filename, "w");
    if (logging_file == NULL) {
        AB_ERROR(-1, "Opening file did not work", AB_ERR_ABORT);
    }
    fprintf(logging_file, "time, current speed, speed_setpoint, hnm pwm, prop_term, inte_term, deriv_term, error\n");
    #endif //LOGGING
    struct timeval time_value;
    long long ms_since_start;
    int pid_hnm_value;
    
    for(;;){

        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));
        
        speed_value = hnm_speed(&hnm_fd);
        
        if(setpoint_speed >= 1.9){
            new_pwm_value = (int)(1820 * setpoint_speed + 23923);
        } else{
            new_pwm_value = 0;
        }
    
        pid_hnm_value = (int)pid_output(&hnm_pid, setpoint_speed, speed_value, ms_since_start);
        new_pwm_value += pid_hnm_value;
        if(new_pwm_value < 0){
            new_pwm_value = 0;
        } else if(new_pwm_value > 55000){
            new_pwm_value = 55000;
        }
    
        
        hnm_set_pwm(&hnm_fd, new_pwm_value);
        
        #ifdef LOGGING
        // writing the values into the logging file
        fprintf(logging_file, "%lld, %4.2f, %4.1f, %d, %8.2f, %8.2f, %8.2f, %8.2f\n",
                ms_since_start,
                speed_value,
                setpoint_speed,
                new_pwm_value,
                hnm_pid.proportional_term,
                hnm_pid.integral_term,
                hnm_pid.derivative_term,
                hnm_pid.error_history[hnm_pid.history_counter]);
        #endif //LOGGING
        
        #ifndef LOGGING
        // writing the values into the logging file
        printf("%lld, %4.2f, %4.1f, %d, %8.2f, %8.2f, %8.2f, %8.2f\n",
                ms_since_start,
                speed_value,
                setpoint_speed,
                new_pwm_value,
                hnm_pid.proportional_term,
                hnm_pid.integral_term,
                hnm_pid.derivative_term,
                hnm_pid.error_history[hnm_pid.history_counter]);
        #endif //LOGGING
        
        
        usleep(150000);
    }
}

void safe_exit(){
    system(PIN_DEFAULT_FILE_PATH);
    printf("\npins have been reset.\n");
    fflush(stdout);
}


void closing_all_devices(){
    printf("Closing of all devices started.\n");
    fflush(stdout);
    hnm_device_close(&hnm_fd);
    hnm_disable_speed_reading(&hnm_fd);
    printf("all devices closed successfully.\n");
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

    
//  LOGGING
    local_time_initializer = time(NULL);
    local_time = *localtime(&local_time_initializer);
    gettimeofday(&precise_time, NULL);
    


    hnm_device_open(&hnm_fd, "4", "0", "p9.14");
    hnm_enable_speed_reading(&hnm_fd);
    
    
    //hnm_pid = pid_open(2500.0f, 7000.0f, -180.0f, 0.0f, 60000.0f, 0.0f, 60000.0f); // Hiermit hat es ungefaer funktioniert.
    //hnm_pid = pid_open(2200.0f, 3000.0f, -1200.0f, -20000.0f, 20000.0f, 0.0f, 60000.0f);
    hnm_pid = pid_open(0.0f, 1600.0f, 0.0f, -20000.0f, 20000.0f, -1000.0f, 10000.0f);
    
    //hnm_set_pwm(&hnm_fd, 31000);
    
    pthread_t t_hnm; 


    
    if (pthread_create(&t_hnm, NULL, &thread_hnm_read, NULL))
    {
        printf("ERROR creating t_hnm\n");
        fflush(stdout);
    }



    
    setpoint_speed = 0.0f;
    usleep(500000);
    setpoint_speed = 3.3f;
    sleep(10);
    setpoint_speed = 0.0f;
    sleep(5);
    
    
    /*
    setpoint_speed = 0.0f;
    usleep(500000);
    setpoint_speed = 2.0f;
    sleep(6);
    setpoint_speed = 5.0f;
    sleep(6);
    setpoint_speed = 2.0f;
    sleep(10);
    setpoint_speed = 0.0f;
    sleep(5);
    */
    

    pthread_cancel(t_hnm);


    pthread_join(t_hnm, NULL);

    
    printf("threads have ended.....................\n");
    fflush(stdout);
    
    hnm_device_close(&hnm_fd);
    hnm_disable_speed_reading(&hnm_fd);

    printf("All devices closed.\n");
    fflush(stdout);
    
    return 0;
    
    printf("Done.\n");
}