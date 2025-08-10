/*
 * speed_pid.h
 *
 * Einfache Geschwindigkeitsregelung, die die Sollgeschwindigkeit
 * proportional zur Größe des Vision-Lenkbefehls reduziert.
 *
 * Idee: Je größer |steer_command| (0..1), desto stärker die Reduktion
 * von der Basisgeschwindigkeit in Richtung Minimalgeschwindigkeit.
 */

#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "balance_pid.h"

typedef struct {
    pid_controller_t pid;      // verwendet die generische PID-Implementierung
    float max_reduction;       // maximal mögliche Geschwindigkeitsreduktion (rad/s)
} speed_pid_controller_t;

/**
 * Initialisiert den Speed-PID-Controller.
 *
 * @param Kp               Proportional-Verstärkung
 * @param Ki               Integral-Verstärkung
 * @param Kd               Differential-Verstärkung
 * @param max_reduction    Maximale Reduktion der Geschwindigkeit (rad/s)
 * @param integral_min     Minimaler Integrationswert (Anti-Windup)
 * @param integral_max     Maximaler Integrationswert (Anti-Windup)
 * @return                 Initialisierte Controller-Struktur
 */
speed_pid_controller_t speed_pid_init(
    float Kp,
    float Ki,
    float Kd,
    float max_reduction,
    float integral_min,
    float integral_max
);

/**
 * Berechnet die Zielgeschwindigkeit aus Basisgeschwindigkeit und dem aktuellen
 * Vision-Lenkbefehl. Größere |steer_command| → höhere Reduktion.
 *
 * @param ctrl             Speed-PID-Controller
 * @param base_speed       Basisgeschwindigkeit (rad/s)
 * @param min_speed        Minimale Geschwindigkeit (rad/s)
 * @param max_speed        Maximale Geschwindigkeit (rad/s)
 * @param steer_command    Vision-Lenkbefehl in [-1, 1]
 * @param current_time_us  Zeitbasis (µs)
 * @return                 Zielgeschwindigkeit (rad/s), begrenzt auf [min_speed, max_speed]
 */
float speed_pid_compute(
    speed_pid_controller_t *ctrl,
    float base_speed,
    float min_speed,
    float max_speed,
    float steer_command,
    long long current_time_us
);

#endif // SPEED_PID_H


