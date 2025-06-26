/*
 * balance_config.c
 * 
 * Konfigurationsmanagement für den Balance-Controller
 * Vereinfachte JSON-Parsing-Implementierung für Webots
 */

#include "balance_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Vereinfachtes JSON-Parsing für spezifische Parameter
static float parse_float_from_json(const char *json_content, const char *key, float default_value) {
    char search_pattern[256];
    snprintf(search_pattern, sizeof(search_pattern), "\"%s\":", key);
    
    char *pos = strstr(json_content, search_pattern);
    if (pos != NULL) {
        pos += strlen(search_pattern);
        
        // Überspringe Leerzeichen
        while (*pos == ' ' || *pos == '\t') pos++;
        
        float value;
        if (sscanf(pos, "%f", &value) == 1) {
            return value;
        }
    }
    
    return default_value;
}

static int parse_int_from_json(const char *json_content, const char *key, int default_value) {
    char search_pattern[256];
    snprintf(search_pattern, sizeof(search_pattern), "\"%s\":", key);
    
    char *pos = strstr(json_content, search_pattern);
    if (pos != NULL) {
        pos += strlen(search_pattern);
        
        // Überspringe Leerzeichen
        while (*pos == ' ' || *pos == '\t') pos++;
        
        int value;
        if (sscanf(pos, "%d", &value) == 1) {
            return value;
        }
    }
    
    return default_value;
}

int balance_config_load(balance_config_t *config, const char *filename) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        printf("Konfigurationsdatei '%s' nicht gefunden\n", filename);
        return -1;
    }
    
    // Datei einlesen
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    char *json_content = malloc(file_size + 1);
    if (json_content == NULL) {
        fclose(file);
        return -1;
    }
    
    fread(json_content, 1, file_size, file);
    json_content[file_size] = '\0';
    fclose(file);
    
    // Erst Standardwerte setzen
    balance_config_set_defaults(config);
    
    // Dann JSON-Werte parsen (überschreibt Standardwerte)
    
    // Angle PID Parameter
    config->angle_pid.Kp = parse_float_from_json(json_content, "angle_Kp", config->angle_pid.Kp);
    config->angle_pid.Ki = parse_float_from_json(json_content, "angle_Ki", config->angle_pid.Ki);
    config->angle_pid.Kd = parse_float_from_json(json_content, "angle_Kd", config->angle_pid.Kd);
    config->angle_pid.output_min = parse_float_from_json(json_content, "angle_output_min", config->angle_pid.output_min);
    config->angle_pid.output_max = parse_float_from_json(json_content, "angle_output_max", config->angle_pid.output_max);
    config->angle_pid.integral_min = parse_float_from_json(json_content, "angle_integral_min", config->angle_pid.integral_min);
    config->angle_pid.integral_max = parse_float_from_json(json_content, "angle_integral_max", config->angle_pid.integral_max);
    
    // Speed Control Parameter
    config->speed_control.base_speed = parse_float_from_json(json_content, "base_speed", config->speed_control.base_speed);
    config->speed_control.min_speed = parse_float_from_json(json_content, "min_speed", config->speed_control.min_speed);
    config->speed_control.max_speed = parse_float_from_json(json_content, "max_speed", config->speed_control.max_speed);
    config->speed_control.stability_reduction = parse_float_from_json(json_content, "stability_reduction", config->speed_control.stability_reduction);
    
    // Mechanical Limits
    config->mechanical_limits.max_handlebar_angle = parse_float_from_json(json_content, "max_handlebar_angle", config->mechanical_limits.max_handlebar_angle);
    config->mechanical_limits.max_roll_angle = parse_float_from_json(json_content, "max_roll_angle", config->mechanical_limits.max_roll_angle);
    
    // System Settings
    config->system.enable_logging = parse_int_from_json(json_content, "enable_logging", config->system.enable_logging);
    config->system.enable_preview = parse_int_from_json(json_content, "enable_preview", config->system.enable_preview);
    config->system.config_reload_interval = parse_int_from_json(json_content, "config_reload_interval", config->system.config_reload_interval);
    config->system.filter_size = parse_int_from_json(json_content, "filter_size", config->system.filter_size);
    
    free(json_content);
    
    // Konfiguration validieren
    if (balance_config_validate(config) != 0) {
        printf("Warnung: Ungültige Konfigurationswerte gefunden, verwende Standardwerte\n");
        balance_config_set_defaults(config);
    }
    
    printf("Konfiguration aus '%s' geladen\n", filename);
    return 0;
}

int balance_config_save(const balance_config_t *config, const char *filename) {
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        printf("Fehler: Kann Konfigurationsdatei '%s' nicht schreiben\n", filename);
        return -1;
    }
    
    fprintf(file, "{\n");
    fprintf(file, "    \"balance_control\": {\n");
    
    // Angle PID Parameter
    fprintf(file, "        \"angle_pid\": {\n");
    fprintf(file, "            \"angle_Kp\": %.6f,\n", config->angle_pid.Kp);
    fprintf(file, "            \"angle_Ki\": %.6f,\n", config->angle_pid.Ki);
    fprintf(file, "            \"angle_Kd\": %.6f,\n", config->angle_pid.Kd);
    fprintf(file, "            \"angle_output_min\": %.6f,\n", config->angle_pid.output_min);
    fprintf(file, "            \"angle_output_max\": %.6f,\n", config->angle_pid.output_max);
    fprintf(file, "            \"angle_integral_min\": %.6f,\n", config->angle_pid.integral_min);
    fprintf(file, "            \"angle_integral_max\": %.6f\n", config->angle_pid.integral_max);
    fprintf(file, "        },\n");
    
    // Speed Control Parameter
    fprintf(file, "        \"speed_control\": {\n");
    fprintf(file, "            \"base_speed\": %.6f,\n", config->speed_control.base_speed);
    fprintf(file, "            \"min_speed\": %.6f,\n", config->speed_control.min_speed);
    fprintf(file, "            \"max_speed\": %.6f,\n", config->speed_control.max_speed);
    fprintf(file, "            \"stability_reduction\": %.6f\n", config->speed_control.stability_reduction);
    fprintf(file, "        },\n");
    
    // Mechanical Limits
    fprintf(file, "        \"mechanical_limits\": {\n");
    fprintf(file, "            \"max_handlebar_angle\": %.6f,\n", config->mechanical_limits.max_handlebar_angle);
    fprintf(file, "            \"max_roll_angle\": %.6f\n", config->mechanical_limits.max_roll_angle);
    fprintf(file, "        },\n");
    
    // System Settings
    fprintf(file, "        \"system\": {\n");
    fprintf(file, "            \"enable_logging\": %d,\n", config->system.enable_logging);
    fprintf(file, "            \"enable_preview\": %d,\n", config->system.enable_preview);
    fprintf(file, "            \"config_reload_interval\": %d,\n", config->system.config_reload_interval);
    fprintf(file, "            \"filter_size\": %d\n", config->system.filter_size);
    fprintf(file, "        }\n");
    
    fprintf(file, "    }\n");
    fprintf(file, "}\n");
    
    fclose(file);
    printf("Konfiguration in '%s' gespeichert\n", filename);
    return 0;
}

void balance_config_set_defaults(balance_config_t *config) {
    // Angle PID Parameter (basierend auf autobike.c)
    config->angle_pid.Kp = 10.0f;          // ANGLE_PID_KP
    config->angle_pid.Ki = 0.0f;            // ANGLE_PID_KI (deaktiviert)
    config->angle_pid.Kd = 2.2f;            // ANGLE_PID_KD
    config->angle_pid.output_min = -0.3f;   // ~-17° in Radiant
    config->angle_pid.output_max = 0.3f;    // ~+17° in Radiant
    config->angle_pid.integral_min = -60.0f; // ANGLE_PID_INTEGRAL_MIN
    config->angle_pid.integral_max = 60.0f;  // ANGLE_PID_INTEGRAL_MAX
    
    // Speed Control Parameter
    config->speed_control.base_speed = 5.0f;        // Basis-Geschwindigkeit
    config->speed_control.min_speed = 3.0f;         // Mindestgeschwindigkeit für Stabilität
    config->speed_control.max_speed = 8.0f;         // Maximale Geschwindigkeit
    config->speed_control.stability_reduction = 0.5f; // 50% Reduktion bei max. Lenkwinkel
    
    // Mechanical Limits
    config->mechanical_limits.max_handlebar_angle = 0.32f; // ~18° in Radiant
    config->mechanical_limits.max_roll_angle = 45.0f;      // 45° maximaler Roll-Winkel
    
    // System Settings
    config->system.enable_logging = 1;         // Logging aktiviert
    config->system.enable_preview = 1;         // Preview aktiviert
    config->system.config_reload_interval = 10; // Alle 10 Schritte neu laden
    config->system.filter_size = 5;            // 5-Punkt gleitender Durchschnitt
}

int balance_config_validate(const balance_config_t *config) {
    // PID-Parameter Plausibilitätsprüfung
    if (config->angle_pid.Kp < 0.0f || config->angle_pid.Kp > 100.0f) return -1;
    if (config->angle_pid.Ki < 0.0f || config->angle_pid.Ki > 50.0f) return -1;
    if (config->angle_pid.Kd < 0.0f || config->angle_pid.Kd > 50.0f) return -1;
    
    // Ausgabebegrenzungen
    if (config->angle_pid.output_min >= config->angle_pid.output_max) return -1;
    if (fabs(config->angle_pid.output_min) > M_PI || fabs(config->angle_pid.output_max) > M_PI) return -1;
    
    // Geschwindigkeitsparameter
    if (config->speed_control.min_speed <= 0.0f || config->speed_control.min_speed > config->speed_control.max_speed) return -1;
    if (config->speed_control.base_speed < config->speed_control.min_speed || 
        config->speed_control.base_speed > config->speed_control.max_speed) return -1;
    if (config->speed_control.stability_reduction < 0.0f || config->speed_control.stability_reduction > 1.0f) return -1;
    
    // Mechanische Begrenzungen
    if (config->mechanical_limits.max_handlebar_angle <= 0.0f || config->mechanical_limits.max_handlebar_angle > M_PI/2) return -1;
    if (config->mechanical_limits.max_roll_angle <= 0.0f || config->mechanical_limits.max_roll_angle > 90.0f) return -1;
    
    // System-Parameter
    if (config->system.config_reload_interval < 1 || config->system.config_reload_interval > 1000) return -1;
    if (config->system.filter_size < 1 || config->system.filter_size > 20) return -1;
    
    return 0; // Alle Parameter sind gültig
}

void balance_config_print(const balance_config_t *config) {
    printf("\n=== Balance Controller Konfiguration ===\n");
    
    printf("Angle PID:\n");
    printf("  Kp: %.3f, Ki: %.3f, Kd: %.3f\n", 
           config->angle_pid.Kp, config->angle_pid.Ki, config->angle_pid.Kd);
    printf("  Output: %.3f bis %.3f rad (%.1f° bis %.1f°)\n",
           config->angle_pid.output_min, config->angle_pid.output_max,
           config->angle_pid.output_min * 180.0f / M_PI, config->angle_pid.output_max * 180.0f / M_PI);
    printf("  Integral: %.1f bis %.1f\n", 
           config->angle_pid.integral_min, config->angle_pid.integral_max);
    
    printf("Speed Control:\n");
    printf("  Base: %.1f rad/s, Min: %.1f rad/s, Max: %.1f rad/s\n",
           config->speed_control.base_speed, config->speed_control.min_speed, config->speed_control.max_speed);
    printf("  Stability Reduction: %.1f%%\n", config->speed_control.stability_reduction * 100.0f);
    
    printf("Mechanical Limits:\n");
    printf("  Max Handlebar: %.3f rad (%.1f°)\n", 
           config->mechanical_limits.max_handlebar_angle,
           config->mechanical_limits.max_handlebar_angle * 180.0f / M_PI);
    printf("  Max Roll: %.1f°\n", config->mechanical_limits.max_roll_angle);
    
    printf("System:\n");
    printf("  Logging: %s, Preview: %s\n",
           config->system.enable_logging ? "Ein" : "Aus",
           config->system.enable_preview ? "Ein" : "Aus");
    printf("  Config Reload: alle %d Schritte, Filter: %d Punkte\n",
           config->system.config_reload_interval, config->system.filter_size);
    
    printf("=======================================\n\n");
} 