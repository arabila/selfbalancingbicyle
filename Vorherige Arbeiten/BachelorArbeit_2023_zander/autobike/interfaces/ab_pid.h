#ifndef __PID_H__
#define __PID_H__

#define HISTORY_LEN 5

typedef struct pid_val {
    float derivative_history[HISTORY_LEN];
    float error_history[HISTORY_LEN];
    int   history_counter;
    long long time_history[HISTORY_LEN];
    
    //Test for acceleration dampening
    float output_history[HISTORY_LEN];
    float max_accel;
    
    float proportional_term;
    float integral_term;// = 0;
    float derivative_term;// = 0;
    float Kp;
    float Ki;
    float Kd;

    float output_min; // minimum output value
    float output_max;
    float integral_min;
    float integral_max;
    
    
} pid_val_t;

// opens the pid controller
pid_val_t pid_open(float Kp, float Ki, float Kd, float output_min,
                   float output_max, float integral_min, float integral_max);

// calculates the next value of the controller. The current time has to be given in
// microseconds since the start of the program.
float pid_output(pid_val_t *pid, float setpoint, float process_variable, long long current_time);

#endif //__PID_H__
