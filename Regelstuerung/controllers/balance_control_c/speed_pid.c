/*
 * speed_pid.c
 */

#include "speed_pid.h"
#include <math.h>
#include <stdio.h>

speed_pid_controller_t speed_pid_init(
    float Kp,
    float max_reduction
) {
    speed_pid_controller_t ctrl;
    ctrl.Kp = Kp;
    ctrl.max_reduction = fabsf(max_reduction);
    return ctrl;
}

float speed_pid_compute(
    speed_pid_controller_t *ctrl,
    float base_speed,
    float min_speed,
    float max_speed,
    float steer_command
) {
    // Einfache P-Reduktion: je weiter |steer_command| von 0 entfernt, desto langsamer
    float steer_intensity = fabsf(steer_command * 10.0f);
    printf("steer_intensity: %f\n", steer_intensity);
    printf("steer_command: %f\n", steer_command);

    float reduction = -ctrl->Kp * steer_intensity; //

    printf("reduction: %f\n", reduction);
    if (reduction < -ctrl->max_reduction) reduction = -ctrl->max_reduction;
    if (reduction > 0.0f) reduction = 0.0f;

    float target_speed = base_speed + reduction; // Reduktion ist negativ
    if (target_speed < min_speed) target_speed = min_speed;
    if (target_speed > max_speed) target_speed = max_speed;
    return target_speed;
}


