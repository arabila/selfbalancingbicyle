/*
 * balance_logging.h
 * 
 * Logging-System für den Balance-Controller
 * Zeichnet Regelungsdaten für Analyse und Tuning auf
 */

#ifndef BALANCE_LOGGING_H
#define BALANCE_LOGGING_H

#include <stdio.h>

// Erweiterte Datenstruktur für einen Logeintrag (inkl. Vision-Control-Daten)
typedef struct {
    // Balance-Controller-Daten
    double timestamp;           // Zeitstempel in Sekunden
    float roll_angle;          // Aktueller Roll-Winkel in Grad
    float steering_output;     // Lenkwinkel-Ausgabe in Radiant
    float target_speed;        // Zielgeschwindigkeit in rad/s
    float p_term;              // P-Term des Balance-PID-Controllers
    float i_term;              // I-Term des Balance-PID-Controllers
    float d_term;              // D-Term des Balance-PID-Controllers
    float error;               // Aktueller Balance-Regelfehler
    float stability_factor;    // Stabilitätsfaktor (0.0 = stabil, 1.0 = instabil)
    
    // Vision-Controller-Daten
    float vision_error;        // Vision-Fehler (Abweichung von erkannter Straße)
    float vision_steer_command; // Lenkbefehl vom Vision-System (-1.0 bis +1.0)
    float vision_speed_command; // Geschwindigkeitsbefehl vom Vision-System (0.0 bis 1.0)
    float vision_p_term;       // P-Term des Vision-PID-Controllers
    float vision_i_term;       // I-Term des Vision-PID-Controllers
    float vision_d_term;       // D-Term des Vision-PID-Controllers
    int vision_active;         // Vision-System aktiv (1) oder inaktiv (0)
    float vision_mask_coverage; // Straßenerkennung-Abdeckung in Prozent (0-100)
} balance_log_data_t;

// Logger-Struktur
typedef struct {
    FILE *csv_file;            // CSV-Datei für Datenaufzeichnung
    char filename[256];        // Dateiname der Log-Datei
    int log_counter;           // Anzahl der Logeinträge
    int is_initialized;        // Initialisierungsstatus
} balance_logger_t;

/**
 * Initialisiert den Logger
 * 
 * @param logger    Pointer auf Logger-Struktur
 * @param log_dir   Verzeichnis für Log-Dateien
 * @return          0 bei Erfolg, -1 bei Fehler
 */
int balance_logging_init(balance_logger_t *logger, const char *log_dir);

/**
 * Schreibt Daten in die Log-Datei
 * 
 * @param logger Pointer auf Logger-Struktur
 * @param data   Pointer auf Datenstruktur
 * @return       0 bei Erfolg, -1 bei Fehler
 */
int balance_logging_write(balance_logger_t *logger, const balance_log_data_t *data);

/**
 * Schließt den Logger und die Datei
 * 
 * @param logger Pointer auf Logger-Struktur
 */
void balance_logging_close(balance_logger_t *logger);

/**
 * Fügt einen Kommentar zur Log-Datei hinzu
 * 
 * @param logger  Pointer auf Logger-Struktur
 * @param comment Kommentar-Text
 */
void balance_logging_add_comment(balance_logger_t *logger, const char *comment);

/**
 * Erstellt eine neue Log-Datei (z.B. nach Crash)
 * 
 * @param logger    Pointer auf Logger-Struktur
 * @param log_dir   Verzeichnis für Log-Dateien
 * @param suffix    Suffix für Dateiname (z.B. "_crash")
 * @return          0 bei Erfolg, -1 bei Fehler
 */
int balance_logging_create_new_file(balance_logger_t *logger, const char *log_dir, const char *suffix);

#endif // BALANCE_LOGGING_H 