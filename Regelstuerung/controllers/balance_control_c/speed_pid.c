/*
 * speed_pid.c
 */

#include "speed_pid.h"
#include <math.h>

speed_pid_controller_t speed_pid_init(
    float Kp,
    float Ki,
    float Kd,
    float max_reduction,
    float integral_min,
    float integral_max
) {
    speed_pid_controller_t ctrl;
    // Wir nutzen die generische PID mit Outputbegrenzung auf [-max_reduction, 0]
    ctrl.pid = pid_init(
        Kp,
        Ki,
        Kd,
        -fabsf(max_reduction),
        0.0f,
        integral_min,
        integral_max
    );
    ctrl.max_reduction = fabsf(max_reduction);
    return ctrl;
}

float speed_pid_compute(
    speed_pid_controller_t *ctrl,
    float base_speed,
    float min_speed,
    float max_speed,
    float steer_command,
    long long current_time_us
) {
    // Einfache P-Regelung: Reduktion = -Kp * |steer_command|
    float steer_intensity = fabsf(steer_command); // 0..1
    float reduction = -ctrl->pid.Kp * steer_intensity;
    if (reduction < -ctrl->max_reduction) reduction = -ctrl->max_reduction;
    if (reduction > 0.0f) reduction = 0.0f;

    float target_speed = base_speed + reduction; // Reduktion ist negativ

    // Begrenzen auf [min_speed, max_speed]
    if (target_speed < min_speed) target_speed = min_speed;
    if (target_speed > max_speed) target_speed = max_speed;

    return target_speed;
}


