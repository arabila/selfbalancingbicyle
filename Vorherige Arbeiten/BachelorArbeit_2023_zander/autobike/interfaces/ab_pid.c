#include <stdlib.h>
#include "ab_pid.h"


pid_val_t pid_open(float Kp, float Ki, float Kd, float output_min,
                   float output_max, float integral_min, float integral_max){
    pid_val_t pid_controller;
    
    for(int i=0; i<HISTORY_LEN; i++){
        pid_controller.error_history[i] = 0;
        pid_controller.derivative_history[i] = 0;
    }
    for(int i=0, k=HISTORY_LEN; i<HISTORY_LEN; i++, k--){
        pid_controller.time_history[i] = k;
    }
    

    
    pid_controller.history_counter = 0;
    
    pid_controller.proportional_term = 0.0f;
    pid_controller.integral_term = 0.0f;
    pid_controller.derivative_term = 0.0f;

    pid_controller.Kp = Kp;
    pid_controller.Ki = Ki;
    pid_controller.Kd = Kd;

    pid_controller.output_min = output_min;
    pid_controller.output_max = output_max;
    pid_controller.integral_min = integral_min;
    pid_controller.integral_max = integral_max;
    
    return pid_controller;
}


float pid_output(pid_val_t *pid, float setpoint, float process_variable, long long current_time){
    pid->error_history[pid->history_counter] = setpoint - process_variable;
    pid->time_history[pid->history_counter] = current_time;
    
    pid->proportional_term = pid->Kp * (pid->error_history[pid->history_counter]);
    
    pid->integral_term += pid->Ki * pid->error_history[pid->history_counter] * ((pid->time_history[pid->history_counter] - pid->time_history[(pid->history_counter + (HISTORY_LEN -1))%HISTORY_LEN])/1000000.0f);
    // anti windup for the integral term
    if(pid->integral_term < pid->integral_min){
        pid->integral_term = pid->integral_min;
    } else if(pid->integral_term > pid->integral_max){
        pid->integral_term = pid->integral_max;
    }
    
    // derivative term
    pid->derivative_history[pid->history_counter] = ((pid->error_history[pid->history_counter] - pid->error_history[((pid->history_counter + 1) % HISTORY_LEN)]) / 
                                ((pid->time_history[pid->history_counter] - pid->time_history[((pid->history_counter + 1) % HISTORY_LEN)]) / 1000000.0f));
    pid->derivative_term = 0;
    for(int i=0; i<HISTORY_LEN; i++){
        pid->derivative_term += pid->derivative_history[i];
    }
    pid->derivative_term = (pid->derivative_term / HISTORY_LEN) * pid->Kd;
    
    
    
    float output = pid->proportional_term + pid->integral_term + pid->derivative_term;
    

    if(output < pid->output_min){
        output = pid->output_min;
    } else if(output > pid->output_max){
        output = pid->output_max;
    }
    
    pid->output_history[pid->history_counter] = output;
    
    if(pid->history_counter>=HISTORY_LEN-1){
        pid->history_counter = 0;
    } else{
        pid->history_counter++;
    }
    
    return output;
}

