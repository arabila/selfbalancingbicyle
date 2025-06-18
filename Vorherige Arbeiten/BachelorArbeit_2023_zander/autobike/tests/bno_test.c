#include <stdio.h>
#include <unistd.h>
#include <signal.h>

//#include "libpruio/pruio.h"
#include <time.h>
#include <sys/time.h>

#include "../interfaces/ab_i2c.h"
#include "../interfaces/ab_bno055.h"
#include "../interfaces/ab_bts7960.h"
#include "../interfaces/ab_pid.h"


#define  DEGREE_FACTOR  16.0

i2c_fd_t bno055_fd;

i2c_fd_t bno055_autobike_init()
{

    i2c_fd_t fd = bno055_device_open("/dev/i2c-2", 0x28, "p9.19", "p9.20"); 
    
    bno055_set_page0(fd);
    bno055_set_mode(fd, 0x00);      // config mode
    
    // TODO: Check whether the next two statements are needed and wanted
    bno055_device_write_byte(fd, 0x41, 0x21);  // map axis orientation
    bno055_device_write_byte(fd, 0x42, 0x04);  // map axis sign

    uint8_t unit_sel = bno055_device_read_byte(fd, 0x38);
    unit_sel &= 0x7B;  // unit selection euler angles in degrees and fusion data output format to windows;
    bno055_device_write_byte(fd, 0x38, unit_sel);

    bno055_set_mode(fd, 0x08);      // imu mode
    
    return fd;
}



void closing_all_devices(){
    printf("Closing of all devices started.");
    fflush(stdout);
    bno055_device_close(bno055_fd);
    printf("all devices closed successfully.");
    fflush(stdout);
}


//#include <fcntl.h>

int main(int argc, char* argv[])
{

    if (signal(SIGINT, closing_all_devices) == SIG_ERR){
        printf("Signal handler failed.");
        fflush(stdout);
        
        return -1;
    }


    // thread bno055 read variables.
    f_euler_degrees_t euler_angles;
    
    pid_val_t angle_to_steering_pid;
    
    float md49_setpoint;

    // creating the necessary logging files.
    // and time reference for the pid controller.
    time_t local_time_initializer;
    struct tm local_time;
    struct timeval precise_time; 

    local_time_initializer = time(NULL);
    local_time = *localtime(&local_time_initializer);
    gettimeofday(&precise_time, NULL);

    
    
    bno055_fd = bno055_autobike_init();

    
    // geschwindigkeit: 31000 ok bergauf mit :angle_to_steering_pid = pid_open(20, 0, 2.3, -300, 300, -3, 3); fuer d = 2 auch gut
    angle_to_steering_pid = pid_open(20, 0, 2.3, -300, 300, -3, 3); 

    
    i2c_fd_t bno055_fd_copy = bno055_fd;
    f_euler_degrees_t temp_euler_angles;
    int angle_to_steering_value;
    
    
    struct timeval time_value;
    long long ms_since_start;
    
    
    for(;;){      //(int i=0; i<200; i++){ // If the loop is only meant to run for a couple of seconds)
        
        bno055_get_euler(bno055_fd_copy, &temp_euler_angles, DEGREE_FACTOR);
        
        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));

        euler_angles = temp_euler_angles;

        angle_to_steering_value = (int)pid_output(&angle_to_steering_pid, 0.0f, (float)temp_euler_angles.roll, ms_since_start);

        md49_setpoint = angle_to_steering_value;
        
        // writing the values into the logging file
        
        printf("milliseconds: %lld, heading: %8.2f, roll: %8.2f, pitch: %8.2f, steering value: %d\n", // , %8.2f, %8.2f, %8.2f, %8.2f\n",
                ms_since_start,
                temp_euler_angles.heading,
                temp_euler_angles.roll,
                temp_euler_angles.pitch,
                angle_to_steering_value);
                //angle_to_steering_pid.proportional_term,
                //angle_to_steering_pid.integral_term,
                //angle_to_steering_pid.derivative_term,
                //angle_to_steering_pid.error_history[angle_to_steering_pid.history_counter]);
        fflush(stdout);
        
        usleep(200000);
    }
    
    bno055_device_close(bno055_fd);
    return 0;
}