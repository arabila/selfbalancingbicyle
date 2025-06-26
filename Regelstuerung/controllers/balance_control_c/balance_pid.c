/*
 * balance_pid.c
 * 
 * PID-Controller-Implementierung für die Fahrradbalancierung
 * Adaptiert von der autobike-Implementierung (Jonah Zander, 2023)
 */

#include "balance_pid.h"
#include <string.h>

pid_controller_t pid_init(float Kp, float Ki, float Kd, 
                         float output_min, float output_max,
                         float integral_min, float integral_max) {
    
    pid_controller_t pid;
    
    // Historie initialisieren
    for (int i = 0; i < HISTORY_LEN; i++) {
        pid.error_history[i] = 0.0;
        pid.derivative_history[i] = 0.0;
        pid.time_history[i] = 0;
    }
    
    pid.history_counter = 0;
    
    // PID-Terme zurücksetzen
    pid.proportional_term = 0.0;
    pid.integral_term = 0.0;
    pid.derivative_term = 0.0;
    
    // Parameter setzen
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    
    // Begrenzungen setzen
    pid.output_min = output_min;
    pid.output_max = output_max;
    pid.integral_min = integral_min;
    pid.integral_max = integral_max;
    
    return pid;
}

float pid_compute(pid_controller_t *pid, float setpoint, 
                  float process_variable, long long current_time_us) {
    
    // Aktueller Fehler
    float error = setpoint - process_variable;
    pid->error_history[pid->history_counter] = error;
    pid->time_history[pid->history_counter] = current_time_us;
    
    // P-Term (Proportional)
    pid->proportional_term = pid->Kp * error;
    
    // I-Term (Integral) mit Anti-Windup
    int prev_index = (pid->history_counter + HISTORY_LEN - 1) % HISTORY_LEN;
    float dt = (pid->time_history[pid->history_counter] - pid->time_history[prev_index]) / 1000000.0; // Mikrosekunden zu Sekunden
    
    if (dt > 0.0 && dt < 1.0) {  // Plausibilitätsprüfung für dt
        pid->integral_term += pid->Ki * error * dt;
        
        // Anti-Windup: Integral-Term begrenzen
        if (pid->integral_term < pid->integral_min) {
            pid->integral_term = pid->integral_min;
        } else if (pid->integral_term > pid->integral_max) {
            pid->integral_term = pid->integral_max;
        }
    }
    
    // D-Term (Differential) mit gleitendem Durchschnitt
    if (dt > 0.0) {
        float current_derivative = (error - pid->error_history[prev_index]) / dt;
        pid->derivative_history[pid->history_counter] = current_derivative;
        
        // Gleitender Durchschnitt über die Historie berechnen
        float derivative_sum = 0.0;
        for (int i = 0; i < HISTORY_LEN; i++) {
            derivative_sum += pid->derivative_history[i];
        }
        pid->derivative_term = pid->Kd * (derivative_sum / HISTORY_LEN);
    }
    
    // Gesamtausgabe berechnen
    float output = pid->proportional_term + pid->integral_term + pid->derivative_term;
    
    // Ausgabe begrenzen
    if (output < pid->output_min) {
        output = pid->output_min;
    } else if (output > pid->output_max) {
        output = pid->output_max;
    }
    
    // History-Counter aktualisieren (zirkulärer Puffer)
    pid->history_counter = (pid->history_counter + 1) % HISTORY_LEN;
    
    return output;
}

void pid_reset(pid_controller_t *pid) {
    // Alle Historie-Werte zurücksetzen
    for (int i = 0; i < HISTORY_LEN; i++) {
        pid->error_history[i] = 0.0;
        pid->derivative_history[i] = 0.0;
        pid->time_history[i] = 0;
    }
    
    pid->history_counter = 0;
    
    // PID-Terme zurücksetzen
    pid->proportional_term = 0.0;
    pid->integral_term = 0.0;
    pid->derivative_term = 0.0;
}

void pid_update_parameters(pid_controller_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    // Optional: Integral-Term zurücksetzen bei Parameteränderung
    // um Sprünge zu vermeiden
    if (Ki == 0.0) {
        pid->integral_term = 0.0;
    }
} 