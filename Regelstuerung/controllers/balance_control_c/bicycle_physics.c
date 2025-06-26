/*
 * bicycle_physics.c
 * 
 * Implementierung der erweiterten Fahrradphysik für realistische Simulation
 * 
 * Diese Datei implementiert alle fehlenden Physik-Effekte, die für eine
 * realistische Fahrrad-Simulation notwendig sind:
 * 
 * 1. Laterale Reifenkräfte (Seitenkräfte)
 * 2. Aerodynamischer Widerstand 
 * 3. Rollwiderstand
 * 4. Gyroskopmommente
 * 5. Sensorrauschen und -verzögerung
 * 6. Umwelteinflüsse (Wind, Straßenneigung)
 */

#include "bicycle_physics.h"
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ============================================================================
// HILFSFUNKTIONEN FÜR MATHEMATIK UND ZUFALLSZAHLEN
// ============================================================================

/**
 * Berechnet Vektor-Länge (Magnitude)
 */
static float vector_magnitude(const float vec[3]) {
    return sqrtf(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}

/**
 * Normalisiert einen Vektor (macht ihn zur Länge 1)
 */
static void vector_normalize(float vec[3], const float input[3]) {
    float mag = vector_magnitude(input);
    if (mag > 1e-6f) {
        vec[0] = input[0] / mag;
        vec[1] = input[1] / mag;
        vec[2] = input[2] / mag;
    } else {
        vec[0] = vec[1] = vec[2] = 0.0f;
    }
}

/**
 * Gaussches Rauschen (Box-Muller-Transformation)
 */
static float gaussian_noise(float sigma) {
    static int have_spare = 0;
    static float spare;
    
    if (have_spare) {
        have_spare = 0;
        return spare * sigma;
    }
    
    have_spare = 1;
    static float u, v, mag;
    do {
        u = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        v = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        mag = u*u + v*v;
    } while (mag >= 1.0f || mag == 0.0f);
    
    mag = sqrtf(-2.0f * logf(mag) / mag);
    spare = v * mag;
    return u * mag * sigma;
}

/**
 * Begrenzt einen Wert zwischen min und max
 */
static float clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// ============================================================================
// HAUPTFUNKTIONEN DER PHYSIK-SIMULATION
// ============================================================================

int bicycle_physics_init(bicycle_physics_t *physics, WbNodeRef robot_node, int timestep) {
    if (!physics || !robot_node) {
        printf("ERROR: Ungültige Parameter für bicycle_physics_init\n");
        return -1;
    }
    
    // Struktur nullsetzen
    memset(physics, 0, sizeof(bicycle_physics_t));
    
    // Webots-Referenzen setzen
    physics->robot_node = robot_node;
    
    // Versuche Rad-Nodes zu finden (optional, falls verfügbar)
    physics->rear_wheel_node = wb_supervisor_node_get_from_def("rear_wheel");
    physics->front_wheel_node = wb_supervisor_node_get_from_def("front_wheel");
    
    // Integrationsschritt berechnen
    physics->integration_step = timestep / 1000.0f; // ms zu s
    
    // Umgebungsparameter initialisieren (ruhige Bedingungen)
    physics->environment.wind_speed = 0.0f;
    physics->environment.wind_direction = 0.0f;
    physics->environment.wind_turbulence = 0.1f;
    physics->environment.road_slope = 0.0f;
    physics->environment.road_roughness = 0.0f;
    
    // Sensorsimulation initialisieren
    physics->sensor_sim.buffer_index = 0;
    physics->sensor_sim.buffer_filled = false;
    memset(physics->sensor_sim.imu_buffer, 0, sizeof(physics->sensor_sim.imu_buffer));
    
    // Zufallsgenerator initialisieren
    srand((unsigned int)time(NULL));
    
    physics->initialized = true;
    physics->last_update_time = wb_robot_get_time();
    
    printf("PHYSIK: Fahrradphysik initialisiert (dt=%.3fs)\n", physics->integration_step);
    
    // Prüfe ob Supervisor-Funktionen verfügbar sind
    const double *test_vel = wb_supervisor_node_get_velocity(robot_node);
    if (test_vel == NULL) {
        printf("WARNUNG: Supervisor-Modus nicht aktiv - externe Kräfte werden nicht angewendet\n");
    } else {
        printf("PHYSIK: Supervisor-Modus aktiv - externe Kräfte verfügbar\n");
    }
    
    return 0;
}

void bicycle_physics_update_state(bicycle_physics_t *physics) {
    if (!physics || !physics->initialized) return;
    
    // Aktuelle Zeit für Berechnungen
    double current_time = wb_robot_get_time();
    physics->state.timestamp = current_time;
    
    // Geschwindigkeit vom Supervisor holen (falls verfügbar)
    const double *velocity = wb_supervisor_node_get_velocity(physics->robot_node);
    if (velocity != NULL) {
        physics->state.velocity[0] = (float)velocity[0];
        physics->state.velocity[1] = (float)velocity[1];
        physics->state.velocity[2] = (float)velocity[2];
    } else {
        // Fallback: Geschwindigkeit auf 0 setzen wenn nicht verfügbar
        memset(physics->state.velocity, 0, sizeof(physics->state.velocity));
    }
    
    // Winkelgeschwindigkeit schätzen (da wb_supervisor_node_get_angular_velocity nicht verfügbar)
    // Verwende numerische Ableitung der Geschwindigkeit
    static float last_velocity[3] = {0, 0, 0};
    static double last_time = 0.0;
    static bool first_run = true;
    
    if (!first_run && (current_time - last_time) > 0.001) {
        float dt = (float)(current_time - last_time);
        physics->state.angular_velocity[0] = (physics->state.velocity[0] - last_velocity[0]) / dt;
        physics->state.angular_velocity[1] = (physics->state.velocity[1] - last_velocity[1]) / dt;
        physics->state.angular_velocity[2] = (physics->state.velocity[2] - last_velocity[2]) / dt;
        
        // Begrenze auf realistische Werte
        for (int i = 0; i < 3; i++) {
            if (physics->state.angular_velocity[i] > 10.0f) physics->state.angular_velocity[i] = 10.0f;
            if (physics->state.angular_velocity[i] < -10.0f) physics->state.angular_velocity[i] = -10.0f;
        }
    } else {
        memset(physics->state.angular_velocity, 0, sizeof(physics->state.angular_velocity));
    }
    
    // Werte für nächste Iteration speichern
    memcpy(last_velocity, physics->state.velocity, sizeof(last_velocity));
    last_time = current_time;
    first_run = false;
    
    // Slip-Winkel berechnen (Verhältnis von Quer- zu Längsgeschwindigkeit)
    float v_x = physics->state.velocity[0];
    float v_y = physics->state.velocity[1];
    
    if (fabsf(v_x) > 0.1f) { // Nur bei ausreichender Vorwärtsgeschwindigkeit
        physics->state.slip_angle = atanf(v_y / v_x);
    } else {
        physics->state.slip_angle = 0.0f;
    }
    
    // Begrenze Slip-Winkel auf realistische Werte (±30°)
    physics->state.slip_angle = clamp(physics->state.slip_angle, -0.524f, 0.524f);
    
    // Radwinkelgeschwindigkeit schätzen (vereinfacht)
    float speed = vector_magnitude(physics->state.velocity);
    physics->state.wheel_angular_velocity = speed / WHEEL_RADIUS;
    
    physics->last_update_time = current_time;
}

void bicycle_physics_calculate_forces(bicycle_physics_t *physics) {
    if (!physics || !physics->initialized) return;
    
    // Alle Kräfte zurücksetzen
    memset(&physics->forces, 0, sizeof(physics->forces));
    
    // 1. AERODYNAMISCHER WIDERSTAND
    bicycle_physics_aerodynamic_drag(physics->state.velocity, physics->forces.drag_force);
    
    // 2. LATERALE REIFENKRÄFTE (Seitenkräfte)
    float normal_force = BICYCLE_MASS * GRAVITY; // Vereinfacht: gesamte Gewichtskraft
    float lateral_force_magnitude = bicycle_physics_tire_lateral_force(
        physics->state.slip_angle, normal_force);
    
    // Laterale Kraft wirkt senkrecht zur Fahrtrichtung
    physics->forces.lateral_force[0] = 0.0f; // Keine Kraft in Längsrichtung
    physics->forces.lateral_force[1] = lateral_force_magnitude; // Kraft in Querrichtung
    physics->forces.lateral_force[2] = 0.0f; // Keine vertikale Kraft
    
    // 3. ROLLWIDERSTAND
    physics->forces.rolling_resistance_torque = bicycle_physics_rolling_resistance(
        physics->state.wheel_angular_velocity, normal_force);
    
    // 4. GYROSKOPMOMMENTE
    bicycle_physics_gyroscopic_effects(
        physics->state.wheel_angular_velocity,
        physics->state.angular_velocity,
        physics->forces.gyroscopic_torque);
    
    // 5. UMWELTEINFLÜSSE
    bicycle_physics_environment_effects(physics);
}

void bicycle_physics_apply_forces(bicycle_physics_t *physics) {
    if (!physics || !physics->initialized || !physics->robot_node) return;
    
    // Prüfe ob Supervisor-Funktionen verfügbar sind
    const double *test_vel = wb_supervisor_node_get_velocity(physics->robot_node);
    if (test_vel == NULL) {
        // Supervisor nicht verfügbar - keine Kräfte anwendbar
        return;
    }
    
    // 1. AERODYNAMISCHER WIDERSTAND anwenden
    float total_drag[3];
    for (int i = 0; i < 3; i++) {
        total_drag[i] = physics->forces.drag_force[i] + physics->forces.crosswind_force[i];
    }
    
    if (vector_magnitude(total_drag) > 0.01f) { // Nur wenn signifikante Kraft
        double drag_force_d[3] = {
            (double)total_drag[0],
            (double)total_drag[1], 
            (double)total_drag[2]
        };
        wb_supervisor_node_add_force(physics->robot_node, drag_force_d, false);
    }
    
    // 2. LATERALE REIFENKRÄFTE anwenden
    if (fabsf(physics->forces.lateral_force[1]) > 0.1f) { // Nur bei signifikanter Seitenkraft
        double lateral_force_d[3] = {
            (double)physics->forces.lateral_force[0],
            (double)physics->forces.lateral_force[1],
            (double)physics->forces.lateral_force[2]
        };
        
        // Kraft am Schwerpunkt anwenden (vereinfacht, da _with_offset nicht verfügbar)
        wb_supervisor_node_add_force(physics->robot_node, lateral_force_d, false);
    }
    
    // 3. GYROSKOPMOMMENTE anwenden (falls implementiert)
    if (vector_magnitude(physics->forces.gyroscopic_torque) > 0.01f) {
        double gyro_torque_d[3] = {
            (double)physics->forces.gyroscopic_torque[0],
            (double)physics->forces.gyroscopic_torque[1],
            (double)physics->forces.gyroscopic_torque[2]
        };
        wb_supervisor_node_add_torque(physics->robot_node, gyro_torque_d, false);
    }
    
    // HINWEIS: Rollwiderstandsmoment wird direkt am Motor angewendet,
    // nicht über wb_supervisor_node_add_torque, da es spezifisch das Rad betrifft
}

float bicycle_physics_simulate_imu(bicycle_physics_t *physics, float true_roll) {
    if (!physics || !physics->initialized) return true_roll;
    
    // 1. VERZÖGERUNG simulieren (FIFO-Puffer)
    physics->sensor_sim.imu_buffer[physics->sensor_sim.buffer_index] = true_roll;
    physics->sensor_sim.buffer_index = (physics->sensor_sim.buffer_index + 1) % IMU_DELAY_SAMPLES;
    
    float delayed_roll;
    if (!physics->sensor_sim.buffer_filled) {
        // Puffer noch nicht voll - verwende aktuellen Wert
        delayed_roll = true_roll;
        if (physics->sensor_sim.buffer_index == 0) {
            physics->sensor_sim.buffer_filled = true;
        }
    } else {
        // Hole verzögerten Wert
        delayed_roll = physics->sensor_sim.imu_buffer[physics->sensor_sim.buffer_index];
    }
    
    // 2. RAUSCHEN hinzufügen
    float noise = gaussian_noise(IMU_NOISE_SIGMA);
    float noisy_roll = delayed_roll + noise;
    
    // 3. PLAUSIBILITÄTSPRÜFUNG - verhindere unrealistische Sprünge
    static float last_output = 0.0f;
    static bool first_call = true;
    
    if (!first_call) {
        float max_change = 0.1f; // Maximale Änderung pro Zeitschritt (rad)
        float change = noisy_roll - last_output;
        
        if (fabsf(change) > max_change) {
            // Begrenze die Änderung
            if (change > max_change) {
                noisy_roll = last_output + max_change;
            } else if (change < -max_change) {
                noisy_roll = last_output - max_change;
            }
        }
    }
    
    last_output = noisy_roll;
    first_call = false;
    
    return noisy_roll;
}

// ============================================================================
// SPEZIFISCHE PHYSIK-BERECHNUNGEN
// ============================================================================

float bicycle_physics_tire_lateral_force(float slip_angle, float normal_force) {
    // Vereinfachtes Pacejka-Modell für Seitenkräfte
    // Fy = D * sin(C * atan(B * slip_angle))
    // Wo: B = Steifigkeitsfaktor, C = Formfaktor, D = Spitzenfaktor
    
    float B = 10.0f;    // Steifigkeitsfaktor (typisch 8-15)
    float C = 1.3f;     // Formfaktor (typisch 1.2-1.4)
    float D = normal_force * TIRE_FRICTION_COEFF; // Maximale Seitenkraft
    
    // Berechnung der Seitenkraft
    float lateral_force = D * sinf(C * atanf(B * slip_angle));
    
    // Für kleine Slip-Winkel: lineare Näherung
    if (fabsf(slip_angle) < 0.1f) {
        lateral_force = TIRE_STIFFNESS * slip_angle;
    }
    
    return lateral_force;
}

void bicycle_physics_aerodynamic_drag(const float velocity[3], float drag_force[3]) {
    float speed = vector_magnitude(velocity);
    
    if (speed < 0.1f) {
        // Bei sehr geringer Geschwindigkeit: kein Luftwiderstand
        memset(drag_force, 0, 3 * sizeof(float));
        return;
    }
    
    // Luftwiderstandskraft: F = 0.5 * rho * Cd * A * v²
    float drag_magnitude = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA * speed * speed;
    
    // Kraft wirkt entgegengesetzt zur Bewegungsrichtung
    float velocity_unit[3];
    vector_normalize(velocity_unit, velocity);
    
    drag_force[0] = -drag_magnitude * velocity_unit[0];
    drag_force[1] = -drag_magnitude * velocity_unit[1];
    drag_force[2] = -drag_magnitude * velocity_unit[2];
}

float bicycle_physics_rolling_resistance(float wheel_velocity, float normal_force) {
    // Rollwiderstandsmoment: M = Crr * N * r
    float resistance_force = ROLLING_RESISTANCE_COEFF * normal_force;
    float resistance_torque = resistance_force * WHEEL_RADIUS;
    
    // Richtung: entgegengesetzt zur Drehrichtung
    if (wheel_velocity > 0.01f) {
        return -resistance_torque;
    } else if (wheel_velocity < -0.01f) {
        return resistance_torque;
    } else {
        return 0.0f; // Kein Widerstand bei stillstehendem Rad
    }
}

void bicycle_physics_gyroscopic_effects(float wheel_angular_vel, const float frame_angular_vel[3], float gyro_torque[3]) {
    // Gyroskopmomment: M = I * omega_wheel × omega_frame
    // Für Fahrradräder ist der Gyroskopeeffekt hauptsächlich um die Y-Achse spürbar
    
    float wheel_inertia = 0.002f; // Trägheitsmoment des Rades um Rotationsachse
    
    // Vereinfachte Berechnung: Hauptsächlich Stabilisierung um Roll-Achse
    float omega_x = frame_angular_vel[0]; // Roll-Rate
    float omega_z = frame_angular_vel[2]; // Yaw-Rate
    
    // Gyroskopmomment wirkt stabilisierend
    gyro_torque[0] = -wheel_inertia * wheel_angular_vel * omega_z; // Roll-Stabilisierung
    gyro_torque[1] = wheel_inertia * wheel_angular_vel * omega_x;  // Pitch-Moment
    gyro_torque[2] = 0.0f; // Yaw wird nicht direkt beeinflusst
    
    // Begrenze Mommente auf realistische Werte
    for (int i = 0; i < 3; i++) {
        gyro_torque[i] = clamp(gyro_torque[i], -5.0f, 5.0f);
    }
}

void bicycle_physics_environment_effects(bicycle_physics_t *physics) {
    // SEITENWIND-EFFEKTE
    if (physics->environment.wind_speed > 0.1f) {
        float wind_velocity[3];
        
        // Windgeschwindigkeit in Weltkoordinaten umrechnen
        float cos_dir = cosf(physics->environment.wind_direction);
        float sin_dir = sinf(physics->environment.wind_direction);
        
        wind_velocity[0] = physics->environment.wind_speed * cos_dir;
        wind_velocity[1] = physics->environment.wind_speed * sin_dir;
        wind_velocity[2] = 0.0f;
        
        // Relative Windgeschwindigkeit (Wind - Fahrradgeschwindigkeit)
        float relative_wind[3];
        for (int i = 0; i < 3; i++) {
            relative_wind[i] = wind_velocity[i] - physics->state.velocity[i];
        }
        
        float rel_wind_speed = vector_magnitude(relative_wind);
        
        if (rel_wind_speed > 0.1f) {
            // Seitenwindkraft berechnen
            float side_drag_magnitude = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * SIDE_AREA * 
                                      rel_wind_speed * rel_wind_speed;
            
            // Kraft in Richtung des relativen Windes
            float wind_unit[3];
            vector_normalize(wind_unit, relative_wind);
            
            physics->forces.crosswind_force[0] = side_drag_magnitude * wind_unit[0];
            physics->forces.crosswind_force[1] = side_drag_magnitude * wind_unit[1];
            physics->forces.crosswind_force[2] = 0.0f;
            
            // Turbulenz hinzufügen
            if (physics->environment.wind_turbulence > 0.0f) {
                for (int i = 0; i < 2; i++) {
                    float turbulence = gaussian_noise(physics->environment.wind_turbulence * 
                                                    side_drag_magnitude * 0.1f);
                    physics->forces.crosswind_force[i] += turbulence;
                }
            }
        }
    }
    
    // STRAZENNEIGUNG (vereinfacht - gravitationelle Hangabtriebskraft)
    if (fabsf(physics->environment.road_slope) > 0.001f) {
        float slope_force = BICYCLE_MASS * GRAVITY * sinf(physics->environment.road_slope);
        
        // Kraft wirkt in Fahrtrichtung (positiv = bergab, negativ = bergauf)
        if (vector_magnitude(physics->state.velocity) > 0.1f) {
            float velocity_unit[3];
            vector_normalize(velocity_unit, physics->state.velocity);
            
            // Addiere zur Luftwiderstandskraft (da sie beide am Massenschwerpunkt wirken)
            physics->forces.drag_force[0] += slope_force * velocity_unit[0];
            physics->forces.drag_force[1] += slope_force * velocity_unit[1];
        }
    }
}

void bicycle_physics_set_environment(bicycle_physics_t *physics, float wind_speed, float wind_direction, float turbulence) {
    if (!physics) return;
    
    physics->environment.wind_speed = clamp(wind_speed, 0.0f, 20.0f);         // Max 20 m/s
    physics->environment.wind_direction = wind_direction;
    physics->environment.wind_turbulence = clamp(turbulence, 0.0f, 1.0f);    // 0-100%
    
    printf("PHYSIK: Umwelt gesetzt - Wind: %.1f m/s @ %.1f°, Turbulenz: %.1f%%\n",
           wind_speed, wind_direction * 180.0f / M_PI, turbulence * 100.0f);
}

// ============================================================================
// HAUPTSIMULATIONSSCHLEIFE UND HILFSFUNKTIONEN
// ============================================================================

float bicycle_physics_step(bicycle_physics_t *physics, float true_roll) {
    if (!physics || !physics->initialized) return true_roll;
    
    // 1. Physik-Zustand aktualisieren
    bicycle_physics_update_state(physics);
    
    // 2. Kräfte berechnen
    bicycle_physics_calculate_forces(physics);
    
    // 3. Kräfte anwenden
    bicycle_physics_apply_forces(physics);
    
    // 4. Sensor-Simulation (Rauschen + Verzögerung)
    float simulated_roll = bicycle_physics_simulate_imu(physics, true_roll);
    
    return simulated_roll;
}

void bicycle_physics_debug_print(const bicycle_physics_t *physics) {
    if (!physics || !physics->initialized) return;
    
    printf("\n=== FAHRRAD-PHYSIK DEBUG ===\n");
    printf("Geschwindigkeit: [%.2f, %.2f, %.2f] m/s (|v|=%.2f)\n",
           physics->state.velocity[0], physics->state.velocity[1], physics->state.velocity[2],
           vector_magnitude(physics->state.velocity));
    printf("Slip-Winkel: %.3f rad (%.1f°)\n", physics->state.slip_angle, 
           physics->state.slip_angle * 180.0f / M_PI);
    printf("Luftwiderstand: [%.2f, %.2f, %.2f] N\n",
           physics->forces.drag_force[0], physics->forces.drag_force[1], physics->forces.drag_force[2]);
    printf("Seitenkraft: [%.2f, %.2f, %.2f] N\n",
           physics->forces.lateral_force[0], physics->forces.lateral_force[1], physics->forces.lateral_force[2]);
    printf("Rollwiderstand: %.3f Nm\n", physics->forces.rolling_resistance_torque);
    printf("Gyroskopmomment: [%.3f, %.3f, %.3f] Nm\n",
           physics->forces.gyroscopic_torque[0], physics->forces.gyroscopic_torque[1], physics->forces.gyroscopic_torque[2]);
    printf("Wind: %.1f m/s @ %.1f°\n", physics->environment.wind_speed,
           physics->environment.wind_direction * 180.0f / M_PI);
    printf("==========================\n\n");
}

void bicycle_physics_cleanup(bicycle_physics_t *physics) {
    if (!physics) return;
    
    // Struktur zurücksetzen
    memset(physics, 0, sizeof(bicycle_physics_t));
    
    printf("PHYSIK: Fahrradphysik-System bereinigt\n");
} 