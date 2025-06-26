/*
 * bicycle_physics.h
 * 
 * Erweiterte Fahrradphysik-Simulationen für realistisches Verhalten
 * 
 * Implementiert:
 * - Laterale Reifenkräfte (Pacejka-Modell)
 * - Aerodynamischer Widerstand
 * - Rollwiderstand
 * - Sensorrauschen und Verzögerungen
 * - Geschwindigkeitsabhängige Effekte
 * 
 * Autor: Balance Control System
 * Version: 1.0
 */

#ifndef BICYCLE_PHYSICS_H
#define BICYCLE_PHYSICS_H

#include <webots/supervisor.h>
#include <stdbool.h>

// Physikalische Konstanten
#define AIR_DENSITY 1.225f              // Luftdichte kg/m³
#define GRAVITY 9.81f                   // Erdbeschleunigung m/s²
#define WHEEL_RADIUS 0.055f             // Radradius in m
#define BICYCLE_MASS 3.5f               // Gesamtmasse in kg
#define BICYCLE_HEIGHT 0.6f             // Fahrradhöhe in m
#define FRONTAL_AREA 0.3f               // Stirnfläche in m²

// Aerodynamische Parameter
#define DRAG_COEFFICIENT 0.9f           // Luftwiderstandsbeiwert
#define SIDE_AREA 0.4f                  // Seitenfläche für Querwind m²

// Reifenparameter
#define TIRE_STIFFNESS 2000.0f          // Reifensteifigkeit N/rad
#define ROLLING_RESISTANCE_COEFF 0.005f // Rollwiderstandsbeiwert
#define TIRE_FRICTION_COEFF 0.8f        // Reibungskoeffizient Reifen-Boden

// Sensorfehler-Parameter
#define IMU_NOISE_SIGMA 0.01f           // Standardabweichung IMU-Rauschen (rad)
#define IMU_DELAY_SAMPLES 2             // Verzögerung in Samples (10ms bei 5ms timestep)

// Strukturen für Physik-Zustand
typedef struct {
    float velocity[3];                  // Aktuelle Geschwindigkeit [x,y,z] m/s
    float angular_velocity[3];          // Winkelgeschwindigkeit [x,y,z] rad/s
    float roll_angle;                   // Aktueller Roll-Winkel rad
    float slip_angle;                   // Slip-Winkel rad
    float wheel_angular_velocity;       // Radwinkelgeschwindigkeit rad/s
    double timestamp;                   // Zeitstempel für Berechnungen
} bicycle_state_t;

typedef struct {
    float drag_force[3];                // Luftwiderstandskraft [x,y,z] N
    float lateral_force[3];             // Laterale Reifenkraft [x,y,z] N
    float rolling_resistance_torque;    // Rollwiderstandsmoment Nm
    float gyroscopic_torque[3];         // Gyroskopmomemente [x,y,z] Nm
    float crosswind_force[3];           // Seitenwindkraft [x,y,z] N
} bicycle_forces_t;

typedef struct {
    float imu_buffer[IMU_DELAY_SAMPLES]; // Verzögerungspuffer für IMU
    int buffer_index;                    // Aktueller Pufferindex
    bool buffer_filled;                  // Puffer vollständig gefüllt?
    float last_values[3];                // Letzte Werte für Sprungdetektion
} sensor_simulation_t;

typedef struct {
    float wind_speed;                    // Windgeschwindigkeit m/s
    float wind_direction;                // Windrichtung rad (0 = Gegenwind)
    float wind_turbulence;               // Turbulenzintensität 0-1
    float road_slope;                    // Straßenneigung rad
    float road_roughness;                // Straßenrauheit 0-1
} environment_t;

// Physik-Hauptstruktur
typedef struct {
    bicycle_state_t state;
    bicycle_forces_t forces;
    sensor_simulation_t sensor_sim;
    environment_t environment;
    
    // Webots-Referenzen
    WbNodeRef robot_node;
    WbNodeRef rear_wheel_node;
    WbNodeRef front_wheel_node;
    
    // Interne Berechnungsgrößen
    bool initialized;
    double last_update_time;
    float integration_step;
} bicycle_physics_t;

/**
 * Initialisiert das Fahrradphysik-System
 * 
 * @param physics     Pointer auf Physik-Struktur
 * @param robot_node  Webots Robot-Node
 * @param timestep    Simulationszeitschritt in ms
 * @return           0 bei Erfolg, -1 bei Fehler
 */
int bicycle_physics_init(bicycle_physics_t *physics, WbNodeRef robot_node, int timestep);

/**
 * Aktualisiert den Physik-Zustand basierend auf aktuellen Sensordaten
 * 
 * @param physics Pointer auf Physik-Struktur
 */
void bicycle_physics_update_state(bicycle_physics_t *physics);

/**
 * Berechnet alle externen Kräfte basierend auf aktuellem Zustand
 * 
 * @param physics Pointer auf Physik-Struktur
 */
void bicycle_physics_calculate_forces(bicycle_physics_t *physics);

/**
 * Wendet berechnete Kräfte auf das Webots-Modell an
 * 
 * @param physics Pointer auf Physik-Struktur
 */
void bicycle_physics_apply_forces(bicycle_physics_t *physics);

/**
 * Simuliert Sensorfehler und gibt "realistischen" IMU-Wert zurück
 * 
 * @param physics    Pointer auf Physik-Struktur
 * @param true_roll  Wahrer Roll-Winkel vom Simulator
 * @return          Roll-Winkel mit Rauschen und Verzögerung
 */
float bicycle_physics_simulate_imu(bicycle_physics_t *physics, float true_roll);

/**
 * Berechnet laterale Reifenkraft basierend auf Slip-Winkel (vereinfachtes Pacejka-Modell)
 * 
 * @param slip_angle Slip-Winkel in rad
 * @param normal_force Normalkraft in N
 * @return Laterale Kraft in N
 */
float bicycle_physics_tire_lateral_force(float slip_angle, float normal_force);

/**
 * Berechnet aerodynamischen Widerstand
 * 
 * @param velocity Geschwindigkeitsvektor [x,y,z] m/s
 * @param drag_force Ausgabe: Widerstandskraft [x,y,z] N
 */
void bicycle_physics_aerodynamic_drag(const float velocity[3], float drag_force[3]);

/**
 * Berechnet Rollwiderstandsmoment
 * 
 * @param wheel_velocity Radgeschwindigkeit rad/s
 * @param normal_force Normalkraft in N
 * @return Rollwiderstandsmoment in Nm
 */
float bicycle_physics_rolling_resistance(float wheel_velocity, float normal_force);

/**
 * Berechnet Gyroskopmomemente der rotierenden Räder
 * 
 * @param wheel_angular_vel Radwinkelgeschwindigkeit rad/s
 * @param frame_angular_vel Rahmenwinkelgeschwindigkeit [x,y,z] rad/s
 * @param gyro_torque Ausgabe: Gyroskopmommente [x,y,z] Nm
 */
void bicycle_physics_gyroscopic_effects(float wheel_angular_vel, const float frame_angular_vel[3], float gyro_torque[3]);

/**
 * Simuliert Umwelteinflüsse (Wind, Straßenneigung, etc.)
 * 
 * @param physics Pointer auf Physik-Struktur
 */
void bicycle_physics_environment_effects(bicycle_physics_t *physics);

/**
 * Setzt Umweltparameter
 * 
 * @param physics Pointer auf Physik-Struktur
 * @param wind_speed Windgeschwindigkeit m/s
 * @param wind_direction Windrichtung rad (0 = Gegenwind)
 * @param turbulence Turbulenzintensität 0-1
 */
void bicycle_physics_set_environment(bicycle_physics_t *physics, float wind_speed, float wind_direction, float turbulence);

/**
 * Hauptfunktion: Komplette Physik-Simulation für einen Zeitschritt
 * 
 * @param physics Pointer auf Physik-Struktur
 * @param true_roll Wahrer Roll-Winkel für Sensorsimulation
 * @return Simulierter Roll-Winkel mit Sensorfehlern
 */
float bicycle_physics_step(bicycle_physics_t *physics, float true_roll);

/**
 * Debug-Ausgabe des aktuellen Physik-Zustands
 * 
 * @param physics Pointer auf Physik-Struktur
 */
void bicycle_physics_debug_print(const bicycle_physics_t *physics);

/**
 * Bereinigt Physik-System
 * 
 * @param physics Pointer auf Physik-Struktur
 */
void bicycle_physics_cleanup(bicycle_physics_t *physics);

#endif // BICYCLE_PHYSICS_H 