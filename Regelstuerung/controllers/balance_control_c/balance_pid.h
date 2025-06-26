/*
 * balance_pid.h
 * 
 * PID-Controller-Implementierung für die Fahrradbalancierung
 * Adaptiert von der autobike-Implementierung (Jonah Zander, 2023)
 */

#ifndef BALANCE_PID_H
#define BALANCE_PID_H

#define HISTORY_LEN 5
#define ROLL_FILTER_SIZE 10

typedef struct {
    // Historie für D-Term-Berechnung
    float derivative_history[HISTORY_LEN];
    float error_history[HISTORY_LEN];
    int history_counter;
    long long time_history[HISTORY_LEN];
    
    // PID-Terme
    float proportional_term;
    float integral_term;
    float derivative_term;
    
    // PID-Parameter
    float Kp;  // Proportional-Verstärkung
    float Ki;  // Integral-Verstärkung  
    float Kd;  // Differential-Verstärkung
    
    // Begrenzungen
    float output_min;     // Minimaler Ausgabewert
    float output_max;     // Maximaler Ausgabewert
    float integral_min;   // Minimaler Integral-Term (Anti-Windup)
    float integral_max;   // Maximaler Integral-Term (Anti-Windup)
    
} pid_controller_t;

/**
 * Initialisiert einen PID-Controller
 * 
 * @param Kp              Proportional-Verstärkung
 * @param Ki              Integral-Verstärkung
 * @param Kd              Differential-Verstärkung
 * @param output_min      Minimaler Ausgabewert
 * @param output_max      Maximaler Ausgabewert
 * @param integral_min    Minimaler Integral-Term
 * @param integral_max    Maximaler Integral-Term
 * @return                Initialisierter PID-Controller
 */
pid_controller_t pid_init(float Kp, float Ki, float Kd, 
                         float output_min, float output_max,
                         float integral_min, float integral_max);

/**
 * Berechnet den PID-Ausgabewert
 * 
 * @param pid             Pointer auf PID-Controller
 * @param setpoint        Sollwert
 * @param process_variable Istwert (Prozessvariable)
 * @param current_time_us Aktuelle Zeit in Mikrosekunden
 * @return                PID-Ausgabewert
 */
float pid_compute(pid_controller_t *pid, float setpoint, 
                  float process_variable, long long current_time_us);

/**
 * Setzt den PID-Controller zurück (alle Historie-Werte auf 0)
 * 
 * @param pid Pointer auf PID-Controller
 */
void pid_reset(pid_controller_t *pid);

/**
 * Aktualisiert die PID-Parameter zur Laufzeit
 * 
 * @param pid Pointer auf PID-Controller
 * @param Kp  Neue Proportional-Verstärkung
 * @param Ki  Neue Integral-Verstärkung
 * @param Kd  Neue Differential-Verstärkung
 */
void pid_update_parameters(pid_controller_t *pid, float Kp, float Ki, float Kd);

#endif // BALANCE_PID_H 