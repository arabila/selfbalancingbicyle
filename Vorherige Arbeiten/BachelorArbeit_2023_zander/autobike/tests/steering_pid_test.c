#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>

#include <time.h>
#include <sys/time.h>

#include "../interfaces/ab_bts7960.h"
#include "../interfaces/ab_md49.h"
#include "../interfaces/ab_pid.h"
#include "../ab_error_handling.h"


#define LOGGING
#define LOGGING_FILE_PATH "/var/lib/cloud9/fahrradprojekt/autobike/logging/logging_files/"
#define PIN_DEFAULT_FILE_PATH "/var/lib/cloud9/fahrradprojekt/autobike/all_pin_default.sh"


// thread md49 read variables.
uart_fd_t md49_fd;
bts7960_fd_t bts_fd;


// creating the necessary logging files.
time_t local_time_initializer;
struct tm local_time;
struct timeval precise_time;


float md49_setpoint;

// thread md49 read variables.
uart_fd_t md49_fd;
bts7960_fd_t bts_fd;

pid_val_t steering_pid;



void safe_exit(){
    system("/var/lib/cloud9/fahrradprojekt/autobike/all_pin_default.sh");
    printf("pins have been reset.");
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



void * thread_md49_read(void * p)
{

    int encoder_value;
    float setpoint_variable_copy;
    int new_steering_value;
    
    #ifdef LOGGING
    //create logging file
    char filename[128];
    strftime(filename, sizeof(filename), LOGGING_FILE_PATH "%Y-%m-%d__%H-%M-%S_md49_thread.csv", &local_time);
    FILE *logging_file = fopen(filename, "w");
    if (logging_file == NULL) {
        AB_ERROR(-1, "Opening file did not work", AB_ERR_ABORT);
    }
    fprintf(logging_file, "time, encoder, encoder_setpoint, steering_value\n");
    #endif //LOGGING
    struct timeval time_value;
    long long ms_since_start;
    
    for(;;){
        // get the encoder value
        encoder_value = -1 * md49_get_encoder1(md49_fd); // negative to enable smooth operation with pid controller.
        if (encoder_value>400 || encoder_value<-400){
            bts7960_device_close(&bts_fd);
        }
        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));
        
        
        new_steering_value = (int)pid_output(&steering_pid, md49_setpoint, (float)encoder_value, ms_since_start);
        bts7960_turn(&bts_fd, new_steering_value);
        
        
        #ifdef LOGGING
        // writing the values into the logging file
        fprintf(logging_file, "%lld, %d, %4.1f, %d\n",
                ms_since_start,
                encoder_value,
                md49_setpoint,
                new_steering_value);
        #endif //LOGGING
        
        
        #ifndef LOGGING
        printf("%lld, %d, %4.1f, %d\n",
                ms_since_start,
                encoder_value,
                md49_setpoint,
                new_steering_value);
        #endif //LOGGING
        
        // testet with this usleep(1000);
        
        usleep(1000);
    }
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


    local_time_initializer = time(NULL);
    local_time = *localtime(&local_time_initializer);
    gettimeofday(&precise_time, NULL);
    
    
    md49_fd = md49_device_open("/dev/ttyO4", "p9.11", "p9.13");
    md49_device_reset(md49_fd);
    usleep(50000);
    bts7960_device_open(&bts_fd, "49", "49", "1", "1", "p9.22", "1", "0", "p9.21");
    
    //steering_pid = pid_open(1000.0, 300.0, 40.0, -100000, 100000, -20000, 20000);
    steering_pid = pid_open(850.0, 300.0, 30.0, -100000, 100000, -20000, 20000);
    
    
    
    pthread_t t_md49;

    
    if (pthread_create(&t_md49, NULL, &thread_md49_read, NULL))
    {
        printf("ERROR creating t_md49\n");
        fflush(stdout);
    }


    //##################### steering pid test sequence #########################
    for(int i=0; i<10; i++){
    md49_setpoint = 0;
    for(md49_setpoint; md49_setpoint>-200; md49_setpoint--){usleep(5000);}
    for(md49_setpoint; md49_setpoint<0; md49_setpoint++){usleep(5000);}
    md49_setpoint = 0;
    sleep(10);
    }
    
    
    /*
    md49_setpoint = 0;
    for(md49_setpoint; md49_setpoint>-100; md49_setpoint--){usleep(5000);}
    for(md49_setpoint; md49_setpoint<100; md49_setpoint++){usleep(5000);}
    for(md49_setpoint; md49_setpoint>-100; md49_setpoint--){usleep(5000);}
    for(md49_setpoint; md49_setpoint<0; md49_setpoint++){usleep(5000);}
    */

    /*
    md49_setpoint = 0;
    usleep(50000);
    md49_setpoint = 250;
    sleep(1);
    md49_setpoint = 0;
    usleep(700000);
    md49_setpoint = -250;
    usleep(700000);
    md49_setpoint = 0;
    usleep(700000);
    */
    
    
    pthread_cancel(t_md49);



    pthread_join(t_md49, NULL);

    
    printf("threads have ended.....................\n");
    fflush(stdout);

    bts7960_device_close(&bts_fd);
    md49_device_close(md49_fd);
    
    printf("All devices closed.\n");
    fflush(stdout);
    
    return 0;
    
    printf("Done.\n");
}