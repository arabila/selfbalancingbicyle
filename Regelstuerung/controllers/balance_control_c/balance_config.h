/*
 * balance_config.h
 * 
 * Konfigurationsstrukturen für den Balance-Controller
 * Ermöglicht das Laden von Parametern aus JSON-Dateien für GUI-Integration
 */

#ifndef BALANCE_CONFIG_H
#define BALANCE_CONFIG_H

// PID-Parameter für Angle-to-Steering Regelung
typedef struct {
    float Kp;                // Proportional-Verstärkung
    float Ki;                // Integral-Verstärkung
    float Kd;                // Differential-Verstärkung
    float output_min;        // Minimaler Ausgabewert (rad)
    float output_max;        // Maximaler Ausgabewert (rad)
    float integral_min;      // Minimaler Integral-Term
    float integral_max;      // Maximaler Integral-Term
} balance_angle_pid_t;

// Geschwindigkeitsregelung
typedef struct {
    float base_speed;        // Basis-Geschwindigkeit (rad/s)
    float min_speed;         // Minimale Geschwindigkeit (rad/s)
    float max_speed;         // Maximale Geschwindigkeit (rad/s)
    float stability_reduction; // Faktor für Geschwindigkeitsreduktion bei Instabilität (0.0-1.0)
} balance_speed_control_t;

// Mechanische Begrenzungen
typedef struct {
    float max_handlebar_angle; // Maximaler Lenkwinkel (rad)
    float max_roll_angle;      // Maximaler Roll-Winkel für Plausibilitätsprüfung (grad)
} balance_mechanical_limits_t;

// System-Einstellungen
typedef struct {
    int enable_logging;        // Logging aktivieren (0/1)
    int enable_preview;        // Preview-Display aktivieren (0/1)
    int config_reload_interval; // Konfiguration alle N Schritte neu laden
    int filter_size;           // Größe des Roll-Winkel-Filters
} balance_system_t;

// Hauptkonfiguration
typedef struct {
    balance_angle_pid_t angle_pid;
    balance_speed_control_t speed_control;
    balance_mechanical_limits_t mechanical_limits;
    balance_system_t system;
} balance_config_t;

/**
 * Lädt Konfiguration aus JSON-Datei
 * 
 * @param config   Pointer auf Konfigurationsstruktur
 * @param filename Pfad zur JSON-Konfigurationsdatei
 * @return         0 bei Erfolg, -1 bei Fehler
 */
int balance_config_load(balance_config_t *config, const char *filename);

/**
 * Speichert Konfiguration in JSON-Datei
 * 
 * @param config   Pointer auf Konfigurationsstruktur
 * @param filename Pfad zur JSON-Konfigurationsdatei
 * @return         0 bei Erfolg, -1 bei Fehler
 */
int balance_config_save(const balance_config_t *config, const char *filename);

/**
 * Setzt Standard-Konfigurationswerte
 * Basiert auf den bewährten autobike-Parametern
 * 
 * @param config Pointer auf Konfigurationsstruktur
 */
void balance_config_set_defaults(balance_config_t *config);

/**
 * Validiert Konfigurationswerte auf Plausibilität
 * 
 * @param config Pointer auf Konfigurationsstruktur
 * @return       0 wenn gültig, -1 wenn ungültige Werte gefunden
 */
int balance_config_validate(const balance_config_t *config);

/**
 * Gibt Konfiguration auf der Konsole aus
 * 
 * @param config Pointer auf Konfigurationsstruktur
 */
void balance_config_print(const balance_config_t *config);

#endif // BALANCE_CONFIG_H 