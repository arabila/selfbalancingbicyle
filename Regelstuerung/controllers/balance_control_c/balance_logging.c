/*
 * balance_logging.c
 * 
 * Logging-System für den Balance-Controller
 * Schreibt Regelungsdaten in CSV-Dateien für Analyse
 */

#include "balance_logging.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

// Hilfsfunktion: Verzeichnis erstellen falls nicht vorhanden
static int create_directory(const char *path) {
    struct stat st = {0};
    if (stat(path, &st) == -1) {
        if (mkdir(path, 0755) != 0) {
            if (errno != EEXIST) {
                printf("Fehler beim Erstellen des Verzeichnisses '%s': %s\n", path, strerror(errno));
                return -1;
            }
        }
    }
    return 0;
}

// Hilfsfunktion: Zeitstempel für Dateinamen generieren
static void generate_timestamp_string(char *buffer, size_t buffer_size) {
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(buffer, buffer_size, "%Y%m%d_%H%M%S", tm_info);
}

int balance_logging_init(balance_logger_t *logger, const char *log_dir) {
    if (logger == NULL || log_dir == NULL) {
        return -1;
    }
    
    // Logger-Struktur initialisieren
    logger->csv_file = NULL;
    logger->log_counter = 0;
    logger->is_initialized = 0;
    
    // Log-Verzeichnis erstellen
    if (create_directory(log_dir) != 0) {
        return -1;
    }
    
    // Dateiname mit Zeitstempel generieren
    char timestamp[32];
    generate_timestamp_string(timestamp, sizeof(timestamp));
    snprintf(logger->filename, sizeof(logger->filename), 
             "%s/balance_log_%s.csv", log_dir, timestamp);
    
    // CSV-Datei öffnen
    logger->csv_file = fopen(logger->filename, "w");
    if (logger->csv_file == NULL) {
        printf("Fehler beim Öffnen der Log-Datei '%s': %s\n", logger->filename, strerror(errno));
        return -1;
    }
    
    // Erweiterten CSV-Header schreiben (inkl. Vision-Control-Daten)
    fprintf(logger->csv_file, 
            "timestamp,roll_angle,steering_output,final_steer,target_speed,p_term,i_term,d_term,error,stability_factor,"
            "vision_error,vision_steer_command,vision_speed_command,vision_p_term,vision_i_term,vision_d_term,"
            "vision_active,vision_mask_coverage\n");
    fflush(logger->csv_file);
    
    logger->is_initialized = 1;
    printf("Logging initialisiert: %s\n", logger->filename);
    
    return 0;
}

int balance_logging_write(balance_logger_t *logger, const balance_log_data_t *data) {
    if (logger == NULL || data == NULL || !logger->is_initialized || logger->csv_file == NULL) {
        return -1;
    }
    
    // Erweiterte Daten in CSV-Format schreiben (inkl. Vision-Control-Daten)
    fprintf(logger->csv_file, 
            "%.6f,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.6f,%.3f,%.3f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.2f\n",
            data->timestamp,
            data->roll_angle,
            data->steering_output,
            data->final_steer,
            data->target_speed,
            data->p_term,
            data->i_term,
            data->d_term,
            data->error,
            data->stability_factor,
            data->vision_error,
            data->vision_steer_command,
            data->vision_speed_command,
            data->vision_p_term,
            data->vision_i_term,
            data->vision_d_term,
            data->vision_active,
            data->vision_mask_coverage);
    
    logger->log_counter++;
    
    // Alle 100 Einträge flushen für Echtzeit-Monitoring
    if (logger->log_counter % 100 == 0) {
        fflush(logger->csv_file);
    }
    
    return 0;
}

void balance_logging_close(balance_logger_t *logger) {
    if (logger == NULL || !logger->is_initialized) {
        return;
    }
    
    if (logger->csv_file != NULL) {
        // Abschließenden Kommentar schreiben
        fprintf(logger->csv_file, "# Log beendet. Einträge: %d\n", logger->log_counter);
        fclose(logger->csv_file);
        logger->csv_file = NULL;
    }
    
    printf("Logging beendet. %d Einträge in '%s' geschrieben\n", 
           logger->log_counter, logger->filename);
    
    logger->is_initialized = 0;
}

void balance_logging_add_comment(balance_logger_t *logger, const char *comment) {
    if (logger == NULL || comment == NULL || !logger->is_initialized || logger->csv_file == NULL) {
        return;
    }
    
    // Kommentar mit Zeitstempel hinzufügen
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    
    fprintf(logger->csv_file, "# %s: %s\n", timestamp, comment);
    fflush(logger->csv_file);
    
    printf("Log-Kommentar hinzugefügt: %s\n", comment);
}

int balance_logging_create_new_file(balance_logger_t *logger, const char *log_dir, const char *suffix) {
    if (logger == NULL || log_dir == NULL) {
        return -1;
    }
    
    // Aktuelle Datei schließen
    if (logger->is_initialized && logger->csv_file != NULL) {
        balance_logging_add_comment(logger, "Neue Log-Datei wird erstellt");
        fclose(logger->csv_file);
    }
    
    // Neue Datei mit Suffix erstellen
    char timestamp[32];
    generate_timestamp_string(timestamp, sizeof(timestamp));
    
    if (suffix != NULL && strlen(suffix) > 0) {
        snprintf(logger->filename, sizeof(logger->filename), 
                 "%s/balance_log_%s%s.csv", log_dir, timestamp, suffix);
    } else {
        snprintf(logger->filename, sizeof(logger->filename), 
                 "%s/balance_log_%s.csv", log_dir, timestamp);
    }
    
    // Neue CSV-Datei öffnen
    logger->csv_file = fopen(logger->filename, "w");
    if (logger->csv_file == NULL) {
        printf("Fehler beim Öffnen der neuen Log-Datei '%s': %s\n", logger->filename, strerror(errno));
        logger->is_initialized = 0;
        return -1;
    }
    
    // Erweiterten CSV-Header schreiben (inkl. Vision-Control-Daten)
    fprintf(logger->csv_file, 
            "timestamp,roll_angle,steering_output,final_steer,target_speed,p_term,i_term,d_term,error,stability_factor,"
            "vision_error,vision_steer_command,vision_speed_command,vision_p_term,vision_i_term,vision_d_term,"
            "vision_active,vision_mask_coverage\n");
    fflush(logger->csv_file);
    
    logger->log_counter = 0;
    logger->is_initialized = 1;
    
    printf("Neue Log-Datei erstellt: %s\n", logger->filename);
    
    return 0;
} 