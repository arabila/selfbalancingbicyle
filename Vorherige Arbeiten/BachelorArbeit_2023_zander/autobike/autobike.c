#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

//#include "libpruio/pruio.h"
#include <time.h>
#include <sys/time.h>

#include "interfaces/ab_i2c.h"
#include "interfaces/ab_bno055.h"
#include "interfaces/ab_bts7960.h"
#include "interfaces/ab_md49.h"
#include "interfaces/ab_hnm.h"
#include "interfaces/ab_pid.h"
#include "interfaces/ab_remote.h"
#include "ab_error_handling.h"

#define LOGGING
#define LOGGING_FILE_PATH "/var/lib/cloud9/fahrradprojekt/autobike/logging/logging_files/"
#define PIN_DEFAULT_FILE_PATH "/var/lib/cloud9/fahrradprojekt/autobike/all_pin_default.sh"
//#define DUMMY_THREAD

#define  DEGREE_FACTOR  16.0

//PID Angle to Steering
#define ANGLE_PID_KP                    10.0f
#define ANGLE_PID_KI                    0.0f
#define ANGLE_PID_KD                    2.2f 
#define ANGLE_PID_OUTPUT_MIN            -150.0f
#define ANGLE_PID_OUTPUT_MAX            150.0f
#define ANGLE_PID_INTEGRAL_MIN          -60.0f
#define ANGLE_PID_INTEGRAL_MAX          60.0f

#define STEERING_MAX_ANGLE_OFFSET       14.0f
#define STEERING_ANGLE_OFFSET_STEPP     0.05f
#define STEERING_MAX_OFFSET             0.0f
#define STEERING_OFFSET_STEPP           0.0f

#define BNO_THREAD_WAIT                 5000
    
//PID Steering PID
#define MD49_PID_KP                     850.0f
#define MD49_PID_KI                     300.0f
#define MD49_PID_KD                     30.0f
#define MD49_PID_OUTPUT_MIN             -100000.0f
#define MD49_PID_OUTPUT_MAX             100000.0f
#define MD49_PID_INTEGRAL_MIN           -20000.0f
#define MD49_PID_INTEGRAL_MAX           20000.0f

#define MD49_THREAD_WAIT                1000

//PID HNM
#define HNM_PID_KP                      0.0f
#define HNM_PID_KI                      1600.0f
#define HNM_PID_KD                      0.0f
#define HNM_PID_OUTPUT_MIN              -20000.0f
#define HNM_PID_OUTPUT_MAX              20000.0f
#define HNM_PID_INTEGRAL_MIN            -1000.0f
#define HNM_PID_INTEGRAL_MAX            10000.0f

#define HNM_PWM_MULTIPLIER              1820
#define HNM_PWM_ADDITION                23923

#define HNM_REMOTE_SPEED                3.1f 

#define HNM_THREAD_WAIT                 150000





#ifdef DUMMY_THREAD
int iDummyThreadParam;
#endif




//#ifdef LOGGING
// creating the necessary logging files.
time_t local_time_initializer;
struct tm local_time;
struct timeval precise_time;
//#endif //LOGGING


// general global variables
hnm_fd_t hnm_fd; // used for closing connection in the closing all devices function
pid_val_t hnm_pid;

// thread bno055 read variables.
i2c_fd_t bno055_fd;
f_euler_degrees_t euler_angles;
pthread_mutex_t euler_mutex = PTHREAD_MUTEX_INITIALIZER;

pid_val_t angle_to_steering_pid;

remote_fd_t bike_remote_fd;
int remote_speed;
int remote_direction;
pthread_mutex_t remote_speed_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t remote_direction_mutex = PTHREAD_MUTEX_INITIALIZER;


float md49_setpoint;
pthread_mutex_t md49_setpoint_mutex = PTHREAD_MUTEX_INITIALIZER;

// thread md49 read variables.
uart_fd_t md49_fd;
bts7960_fd_t bts_fd;

int md49_encoder_value;
pthread_mutex_t encoder_mutex = PTHREAD_MUTEX_INITIALIZER;

pid_val_t steering_pid;

// thread watchdog variables
pthread_mutex_t watch_encoder_mutex = PTHREAD_MUTEX_INITIALIZER;
int watch_encoder = 1;
pthread_mutex_t watch_bno_mutex = PTHREAD_MUTEX_INITIALIZER;
int watch_bno = 0;




void * thread_bno055_read(void * p)
{
    i2c_fd_t bno055_fd_copy = bno055_fd;
    f_euler_degrees_t temp_euler_angles;
    int angle_to_steering_value;
    
    
    #ifdef LOGGING
    //create logging file
    char filename[128];
    strftime(filename, sizeof(filename), LOGGING_FILE_PATH "%Y-%m-%d__%H-%M-%S_bno_thread.csv", &local_time);
    FILE *logging_file = fopen(filename, "w");
    if (logging_file == NULL) {
        AB_ERROR(-1, "Opening file did not work", AB_ERR_ABORT);
    }
    fprintf(logging_file, "time, heading, roll, pitch, steering_setpoint, P term (%4.2f), I term(%4.2f), D term(%4.2f), error\n",
                angle_to_steering_pid.Kp, angle_to_steering_pid.Ki, angle_to_steering_pid.Kd);
    #endif //LOGGING
    
    struct timeval time_value;
    long long ms_since_start;
    
    int temp_remote_direction;
    int temp_remote_speed;
    
    
    // Parameters of the steering. the stepp decides how fast the change in angle happens.
    float angle_offset = 0.0f;
    float steering_offset = 0.0f;
    float steering_offset_stepp = STEERING_OFFSET_STEPP;
    float angle_offset_stepp = STEERING_ANGLE_OFFSET_STEPP;
    float max_angle_offset = STEERING_MAX_ANGLE_OFFSET;
    float max_steering_offset = STEERING_MAX_OFFSET;
    
    
    bno055_get_euler(bno055_fd_copy, &temp_euler_angles, DEGREE_FACTOR);
    float absolute_straight = temp_euler_angles.heading;
    

    
    for(;;)
    {
        bno055_get_euler(bno055_fd_copy, &temp_euler_angles, DEGREE_FACTOR);
        
        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));

        pthread_mutex_lock(&euler_mutex);
        euler_angles = temp_euler_angles;
        pthread_mutex_unlock(&euler_mutex);
        
        
        // remote steering section here only steering is important.
        // speed is important in hnm control thread.
        temp_remote_direction = remote_get_steering(&bike_remote_fd);
        temp_remote_speed = remote_get_throttle(&bike_remote_fd);
        
        pthread_mutex_lock(&remote_direction_mutex);
        remote_direction = temp_remote_direction;
        pthread_mutex_unlock(&remote_direction_mutex);
        
        pthread_mutex_lock(&remote_speed_mutex);
        remote_speed = temp_remote_speed;
        pthread_mutex_unlock(&remote_speed_mutex);
    
        
        //TODO turn this into a function and also correct the logging of the offset values
        //TODO the following code is for gradually turning the bike
        
        if(temp_remote_direction == -1){
            AB_ERROR(-1, "Connection to the remot lost.", AB_ERR_ABORT);
        }
        if(temp_remote_direction == 2){  // turning right
            if(steering_offset < max_steering_offset){
                steering_offset = steering_offset + steering_offset_stepp;
            }
            if(angle_offset > - max_angle_offset){
                angle_offset = angle_offset - angle_offset_stepp; 
            }
            angle_to_steering_value = (int)(steering_offset+pid_output(&angle_to_steering_pid, angle_offset, (float)temp_euler_angles.roll, ms_since_start));
        } else if(temp_remote_direction == 1){ // turning left
            if(steering_offset > - max_steering_offset){
                steering_offset = steering_offset - steering_offset_stepp;
            }
            if(angle_offset < max_angle_offset){
                angle_offset = angle_offset + angle_offset_stepp; 
            }
            angle_to_steering_value = (int)(steering_offset+pid_output(&angle_to_steering_pid, angle_offset, (float)temp_euler_angles.roll, ms_since_start));
        } else{
            if(steering_offset < 0){
                //steering_offset = steering_offset + steering_offset_stepp;
            } else if (steering_offset > 0){
                //steering_offset = steering_offset - steering_offset_stepp;
            }
            if(angle_offset > 0.0f){
                //angle_offset = angle_offset - angle_offset_stepp; 
            } else if (angle_offset < 0.0f){
                //angle_offset = angle_offset + angle_offset_stepp;
            }
            angle_to_steering_value = (int)(steering_offset+pid_output(&angle_to_steering_pid, angle_offset, (float)temp_euler_angles.roll, ms_since_start));
        }

        
        pthread_mutex_lock(&md49_setpoint_mutex);
        md49_setpoint = angle_to_steering_value;
        pthread_mutex_unlock(&md49_setpoint_mutex);
        
        pthread_mutex_lock(&watch_bno_mutex);
        watch_bno = 1;
        pthread_mutex_unlock(&watch_bno_mutex);
        
        
        
        #ifdef LOGGING
        // writing the values into the logging file, changed steering offset to float


        fprintf(logging_file, "%lld, %8.2f, %8.2f, %8.2f, %d, %8.2f, %8.2f, %8.2f, %8.2f, %3.2f, %3.2f, %d, %d\n",
                ms_since_start,
                temp_euler_angles.heading,
                temp_euler_angles.roll,
                temp_euler_angles.pitch,
                angle_to_steering_value,
                angle_to_steering_pid.proportional_term,
                angle_to_steering_pid.integral_term,
                angle_to_steering_pid.derivative_term,
                angle_to_steering_pid.error_history[angle_to_steering_pid.history_counter],
                steering_offset,
                angle_offset,
                temp_remote_direction,
                temp_remote_speed);
        #endif //LOGGING
        
        
        usleep(BNO_THREAD_WAIT);
    }
}


void safe_exit();


void * thread_md49_read(void * p)
{
    uart_fd_t md49_fd_copy = md49_fd;
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
        
        // update the global encoder value
        pthread_mutex_lock(&encoder_mutex);
        md49_encoder_value = encoder_value;
        pthread_mutex_unlock(&encoder_mutex);
        // get a copy of the setpoint for the steering
        pthread_mutex_lock(&md49_setpoint_mutex);
        setpoint_variable_copy = md49_setpoint;
        pthread_mutex_unlock(&md49_setpoint_mutex);
        
        new_steering_value = (int)pid_output(&steering_pid, setpoint_variable_copy, (float)encoder_value, ms_since_start);
        bts7960_turn(&bts_fd, new_steering_value);
        
        pthread_mutex_lock(&watch_encoder_mutex);
        watch_encoder = 1;
        pthread_mutex_unlock(&watch_encoder_mutex);
        
        
        #ifdef LOGGING
        // writing the values into the logging file
        fprintf(logging_file, "%lld, %d, %4.1f, %d\n",
                ms_since_start,
                encoder_value,
                setpoint_variable_copy,
                new_steering_value);
        #endif //LOGGING
        
        usleep(MD49_THREAD_WAIT);
    }
}

void * thread_hnm_read(void * p){
    usleep(200000); // a small delay for the pru do read the speed.
    
    int remote_speed_copy;
    float speed_value;
    float setpoint_speed;
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
        // get the remote reading and set the speed accordingly.
        pthread_mutex_lock(&remote_speed_mutex);
        remote_speed_copy = remote_speed;
        pthread_mutex_unlock(&remote_speed_mutex);
        
        if(remote_speed_copy == 1){
            setpoint_speed = HNM_REMOTE_SPEED;
            new_pwm_value = 31000;
        } else{
            setpoint_speed = 0.0f;
            new_pwm_value = 0;
        }

        gettimeofday(&time_value, NULL);
        ms_since_start = ((time_value.tv_sec*1000000+time_value.tv_usec) - (precise_time.tv_sec*1000000 + precise_time.tv_usec));
        
        speed_value = hnm_speed(&hnm_fd);
        
        if(setpoint_speed >= 1.9){
            new_pwm_value = (int)(HNM_PWM_MULTIPLIER * setpoint_speed + HNM_PWM_ADDITION);
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
        
        usleep(HNM_THREAD_WAIT);
    }
}



void * thread_watchdog(void * p)
{
    int encoder_working = 0;
    int bno_working = 0;
    int time_working = 1;
    struct timeval time_value;
    gettimeofday(&time_value, NULL);
    long long previous_time = 20000000000L;//((time_value.tv_sec * 1000000) + time_value.tv_usec);
    long long current_time = 0;
    long long time_difference = 0;
    
    sleep(1);
    
    int i;
    for(;;){
        fflush(stdout);
        // check if too much time has passed.
        gettimeofday(&time_value, NULL);
        current_time = ((time_value.tv_sec * 1000000L) + time_value.tv_usec);
        time_difference =  current_time - previous_time;
        previous_time = current_time;
        if(time_difference > 100000L){
            time_working = 0;
            printf("time was the problem\n\n\n\n");
            fflush(stdout);
        }
        
        // make a copy of the status of the watch_bno and watch_encoder values to see if they have been updated.
        pthread_mutex_lock(&watch_bno_mutex);
        bno_working = watch_bno;
        pthread_mutex_unlock(&watch_bno_mutex);
        pthread_mutex_lock(&watch_encoder_mutex);
        encoder_working = watch_encoder;
        pthread_mutex_unlock(&watch_encoder_mutex);
        
        if(bno_working && encoder_working && time_working){
            pthread_mutex_lock(&watch_bno_mutex);
            watch_bno = 0;
            pthread_mutex_unlock(&watch_bno_mutex);
            pthread_mutex_lock(&watch_encoder_mutex);
            watch_encoder = 0;
            pthread_mutex_unlock(&watch_encoder_mutex);
        } else{
            printf("Watchdog found failure\n");
            fflush(stdout);
            safe_exit();
        }
        
        usleep(30000);
    }
}


void * thread_print(void * p)
{
    
    f_euler_degrees_t temp_euler_angles;
    int temp_encoder_value;
    float temp_encoder_setpoint;
    for(;;){
        usleep(100000);
        temp_euler_angles = euler_angles;
        temp_encoder_value = md49_encoder_value;
        temp_encoder_setpoint = md49_setpoint;
        
    
    printf("head:%8.2f   roll:%8.2f   pitch:%8.2f  encoder:%d   enc_setpoint:%8.2f   \r", 
                //i / 10.0,   %8.1fs
                temp_euler_angles.heading,
                temp_euler_angles.roll,
                temp_euler_angles.pitch,
                temp_encoder_value,
                temp_encoder_setpoint)
                ;
                
	    fflush(stdout);
    }
}



    

i2c_fd_t bno055_autobike_init()
{

    i2c_fd_t fd = bno055_device_open("/dev/i2c-2", 0x28, "p9.19", "p9.20"); 
    
    bno055_set_page0(fd);
    bno055_set_mode(fd, 0x00);      // config mode
    
    bno055_device_write_byte(fd, 0x41, 0x21);  // map axis orientation
    bno055_device_write_byte(fd, 0x42, 0x04);  // map axis sign

    uint8_t unit_sel = bno055_device_read_byte(fd, 0x38);
    unit_sel &= 0x7B;  // unit selection euler angles in degrees and fusion data output format to windows;
    bno055_device_write_byte(fd, 0x38, unit_sel);

    bno055_set_mode(fd, 0x08);      // imu mode
    
    return fd;
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
    bts7960_device_close(&bts_fd);
    md49_device_close(md49_fd);
    bno055_device_close(bno055_fd);
    hnm_disable_speed_reading(&hnm_fd);
    remote_close(&bike_remote_fd);
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


    local_time_initializer = time(NULL);
    local_time = *localtime(&local_time_initializer);
    gettimeofday(&precise_time, NULL);

    
    bno055_fd = bno055_autobike_init();
    
    md49_fd = md49_device_open("/dev/ttyO4", "p9.11", "p9.13");
    md49_device_reset(md49_fd);
    usleep(50000);
    
    bts7960_device_open(&bts_fd, "49", "49", "1", "1", "p9.22", "1", "0", "p9.21");
    
    steering_pid = pid_open(MD49_PID_KP, MD49_PID_KI, MD49_PID_KD, MD49_PID_OUTPUT_MIN, MD49_PID_OUTPUT_MAX, MD49_PID_INTEGRAL_MIN, MD49_PID_INTEGRAL_MAX);
    
    angle_to_steering_pid = pid_open(ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, ANGLE_PID_OUTPUT_MIN, ANGLE_PID_OUTPUT_MAX, ANGLE_PID_INTEGRAL_MIN, ANGLE_PID_INTEGRAL_MAX); 
    
    bike_remote_fd = remote_open("1", "0");
    hnm_device_open(&hnm_fd, "4", "0", "p9.14");
    hnm_enable_speed_reading(&hnm_fd);
    

    hnm_pid = pid_open(HNM_PID_KP, HNM_PID_KI, HNM_PID_KD, HNM_PID_OUTPUT_MIN, HNM_PID_OUTPUT_MAX, HNM_PID_INTEGRAL_MIN, HNM_PID_INTEGRAL_MAX);

    
    pthread_t t_bno055, t_print, t_md49, t_watchdog, t_hnm; // , t_time
    
  
    
    if (pthread_create(&t_bno055, NULL, &thread_bno055_read, NULL))
    {
        printf("ERROR creating t_bno055\n");
        fflush(stdout);
    }
    
    if (pthread_create(&t_print, NULL, &thread_print, NULL))
    {
        printf("ERROR creating t_print\n");
        fflush(stdout);
    }
    
    if (pthread_create(&t_md49, NULL, &thread_md49_read, NULL))
    {
        printf("ERROR creating t_md49\n");
        fflush(stdout);
    }
    
    if (pthread_create(&t_hnm, NULL, &thread_hnm_read, NULL))
    {
        printf("ERROR creating t_hnm\n");
        fflush(stdout);
    }
    
    if (pthread_create(&t_watchdog, NULL, &thread_watchdog, NULL))
    {
        printf("ERROR creating t_watchdog\n");
        fflush(stdout);
    }
    
    
    sleep(120);
    pthread_cancel(t_print);
    pthread_cancel(t_bno055);
    pthread_cancel(t_md49);
    pthread_cancel(t_watchdog);
    pthread_cancel(t_hnm);
    
    
    
    
    pthread_join(t_bno055, NULL);
    pthread_join(t_print, NULL);
    pthread_join(t_md49, NULL);
    pthread_join(t_hnm, NULL);
    pthread_join(t_watchdog, NULL);
    
    
    printf("threads have ended.....................\n");
    fflush(stdout);
    
    hnm_device_close(&hnm_fd);
    bts7960_device_close(&bts_fd);
    md49_device_close(md49_fd);
    bno055_device_close(bno055_fd);
    hnm_disable_speed_reading(&hnm_fd);
    remote_close(&bike_remote_fd);
    
    printf("All devices closed.\n");
    fflush(stdout);
    
    return 0;
    
    printf("Done.\n");
}