/*
 * balance_control_c.c
 * 
 * Selbstbalancierender Fahrrad-Controller für Webots
 * Basiert auf der autobike-Implementierung von Jonah Zander (2023)
 * 
 * Implementiert eine Roll-Winkel basierte PID-Regelung zur Fahrradstabilisierung
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/display.h>
#include <webots/camera.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

#include "balance_pid.h"
#include "balance_config.h"
#include "balance_logging.h"
#include "bicycle_physics.h"

// Globale Device-Handles
static WbDeviceTag handlebars_motor;
static WbDeviceTag wheel_motor;
static WbDeviceTag imu_sensor;
static WbDeviceTag display_device;
static WbDeviceTag command_receiver;
static WbDeviceTag status_emitter;
// static WbDeviceTag camera_device;  // Für zukünftige Erweiterungen
static WbNodeRef robot_node;

// Erweiterte Vision-Command-Struktur für IPC (inkl. Debug-Daten)
typedef struct {
    float steer_command;     // Lenkbefehl von Vision-Controller (-1.0 bis +1.0)
    float speed_command;     // Geschwindigkeitsbefehl von Vision-Controller (0.0 bis 1.0)
    int valid;               // Kommando gültig (1) oder nicht (0)
    float vision_error;      // Vision-Fehler (Abweichung von Straße)
    float vision_p_term;     // P-Term des Vision-PID-Controllers
    float vision_i_term;     // I-Term des Vision-PID-Controllers  
    float vision_d_term;     // D-Term des Vision-PID-Controllers
    float mask_coverage;     // Straßenerkennung-Abdeckung in Prozent
} vision_command_t;

// Balance-Status-Struktur für IPC
typedef struct {
    float roll_angle;        // Aktueller Roll-Winkel (rad)
    float steering_output;   // Aktueller Lenkwinkel (rad)
    float current_speed;     // Aktuelle Geschwindigkeit (rad/s)
    float stability_factor;  // Stabilitätsfaktor (0.0-1.0)
} balance_status_t;

// Vision-Command-Status (erweitert)
static vision_command_t last_vision_command = {0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static double last_command_time = 0.0;

// Controller-Zustand
static pid_controller_t angle_pid;
static balance_config_t config;
static balance_logger_t logger;
static bicycle_physics_t bicycle_physics;

// Roll-Winkel-Filter
static float roll_angle_history[ROLL_FILTER_SIZE];
static int roll_history_index = 0;
static int roll_history_filled = 0;

// Timing
static struct timeval start_time;
static int config_reload_counter = 0;

// Funktionsprototypen
static void init_devices(int timestep);
static void load_and_apply_config(void);
static float get_filtered_roll_angle(void);
static void update_display(float roll_angle, float steering_output, float speed);
static void handle_keyboard_input(void);
static long long get_time_microseconds(void);
static void print_status(float roll_angle, float steering_output, float speed);
static int receive_vision_command(vision_command_t *command);
static void send_balance_status(const balance_status_t *status);

int main(int argc __attribute__((unused)), char **argv __attribute__((unused))) {
    // Webots initialisieren
    wb_robot_init();
    
    int timestep = (int)wb_robot_get_basic_time_step();
    printf("Balance Control C - Timestep: %d ms\n", timestep);
    
    // Startzeit für Timing-Messungen
    gettimeofday(&start_time, NULL);
    
    // Devices initialisieren
    init_devices(timestep);
    
    // Konfiguration laden und PID initialisieren
    load_and_apply_config();
    
    // Logging initialisieren
    balance_logging_init(&logger, "../../Monitoring");
    
    // Erweiterte Fahrradphysik initialisieren
    if (bicycle_physics_init(&bicycle_physics, robot_node, timestep) != 0) {
        printf("FEHLER: Konnte Fahrradphysik nicht initialisieren\n");
        wb_robot_cleanup();
        return 1;
    }
    
    // Umgebungsparameter setzen (Beispiel: leichter Seitenwind)
    bicycle_physics_set_environment(&bicycle_physics, 2.0f, 0.5f, 0.2f); // 2 m/s Seitenwind, 20% Turbulenz
    
    printf("=== Balance Control C gestartet ===\n");
    printf("Angle PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", 
           config.angle_pid.Kp, config.angle_pid.Ki, config.angle_pid.Kd);
    printf("Speed: Base=%.1f, Min=%.1f, Max=%.1f\n",
           config.speed_control.base_speed, config.speed_control.min_speed, config.speed_control.max_speed);
    printf("Max Handlebar Angle: %.3f rad (%.1f°)\n", 
           config.mechanical_limits.max_handlebar_angle, 
           config.mechanical_limits.max_handlebar_angle * 180.0 / M_PI);
    printf("DEBUG: vision_command_t Größe: %zu Bytes\n", sizeof(vision_command_t));
    printf("DEBUG: balance_status_t Größe: %zu Bytes\n", sizeof(balance_status_t));
    printf("Drücke ESC zum Beenden\n");
    printf("=============================\n\n");
    
    // Hauptregelschleife
    while (wb_robot_step(timestep) != -1) {
        
        // Konfiguration periodisch neu laden (alle 100ms)
        if (++config_reload_counter >= (100 / timestep)) {
            load_and_apply_config();
            config_reload_counter = 0;
        }
        
        // Keyboard-Input verarbeiten
        handle_keyboard_input();
        
        // 1. Roll-Winkel messen und filtern
        float true_roll_angle = get_filtered_roll_angle();
        
        // 2. ERWEITERTE PHYSIK-SIMULATION durchführen
        // Berechnet externe Kräfte und simuliert realistische Sensorfehler
        float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
        
        // Verwende simulierten Roll-Winkel für realistischere Regelung
        float roll_angle = simulated_roll_angle;
        
        // 3. PID-Regelung: Roll-Winkel → Lenkwinkel
        long long current_time = get_time_microseconds();
        float steering_output = pid_compute(&angle_pid, 0.0, roll_angle, current_time);
        
        // 4. Lenkwinkel begrenzen
        if (steering_output > config.mechanical_limits.max_handlebar_angle) 
            steering_output = config.mechanical_limits.max_handlebar_angle;
        else if (steering_output < -config.mechanical_limits.max_handlebar_angle)
            steering_output = -config.mechanical_limits.max_handlebar_angle;
        
        // 5. Vision-Commands empfangen und verarbeiten
        vision_command_t vision_cmd;
        int vision_cmd_received = receive_vision_command(&vision_cmd);
        
        // 6. Zwei-Ebenen-Regelung: Vision + Balance
        float final_steer = steering_output;  // Balance-PID-Ausgang
        float target_speed = config.speed_control.base_speed;
        
        // Stabilitätsfaktor für alle Regelungspfade berechnen
        float stability_factor = fabs(steering_output) / config.mechanical_limits.max_handlebar_angle;
        
        if (vision_cmd_received && (wb_robot_get_time() - last_command_time) < 0.5) {
            // Vision-Command ist aktuell (< 500ms alt)
            // Kombiniere Balance-Korrektur mit Vision-Lenkbefehl
            float vision_steer = vision_cmd.steer_command * config.mechanical_limits.max_handlebar_angle;
            
            // Gewichtete Kombination: 70% Vision, 30% Balance-Korrektur
            final_steer = 0.7f * vision_steer + 0.3f * steering_output;
            
            // Geschwindigkeit von Vision-Controller übernehmen
            target_speed = config.speed_control.min_speed + 
                          vision_cmd.speed_command * (config.speed_control.max_speed - config.speed_control.min_speed);
            
            printf("VISION: Steer=%.3f, Speed=%.2f | Balance=%.3f → Final=%.3f\n", 
                   vision_steer, target_speed, steering_output, final_steer);
        } else {
            // Kein aktueller Vision-Command → Nur Balance-Regelung
            float speed_reduction = stability_factor * config.speed_control.stability_reduction;
            target_speed = config.speed_control.base_speed * (1.0 - speed_reduction);
        }
        
        // Finale Begrenzungen
        if (final_steer > config.mechanical_limits.max_handlebar_angle) 
            final_steer = config.mechanical_limits.max_handlebar_angle;
        else if (final_steer < -config.mechanical_limits.max_handlebar_angle)
            final_steer = -config.mechanical_limits.max_handlebar_angle;
            
        if (target_speed < config.speed_control.min_speed) 
            target_speed = config.speed_control.min_speed;
        else if (target_speed > config.speed_control.max_speed)
            target_speed = config.speed_control.max_speed;
        
        // 6. ROLLWIDERSTAND am Motor anwenden (aus Physik-Simulation)
        float roll_resistance_torque = bicycle_physics.forces.rolling_resistance_torque;
        if (fabs(roll_resistance_torque) > 0.001f) {
            // Rollwiderstand als zusätzliches Bremsmoment anwenden
            // (Vereinfacht: reduziere Zielgeschwindigkeit entsprechend)
            float resistance_speed_reduction = fabs(roll_resistance_torque) * 0.1f; // Skalierungsfaktor
            target_speed = target_speed * (1.0f - resistance_speed_reduction);
            if (target_speed < 0.1f) target_speed = 0.1f; // Mindestgeschwindigkeit
        }
        
        // 7. Motoren ansteuern
        wb_motor_set_position(handlebars_motor, final_steer);
        wb_motor_set_velocity(wheel_motor, target_speed);  // Positive Geschwindigkeit für Vorwärtsfahrt
        
        // 8. Balance-Status an Vision-Controller senden
        balance_status_t status = {
            .roll_angle = roll_angle,
            .steering_output = final_steer,
            .current_speed = target_speed,
            .stability_factor = fabs(steering_output) / config.mechanical_limits.max_handlebar_angle
        };
        send_balance_status(&status);
        
        // 8. Erweiterte Logging-Daten (inklusive Physik-Informationen + Vision-Daten)
        if (config.system.enable_logging) {
            balance_log_data_t log_data = {
                // Balance-Controller-Daten
                .timestamp = wb_robot_get_time(),
                .roll_angle = roll_angle,
                .steering_output = steering_output,
                .target_speed = target_speed,
                .p_term = angle_pid.proportional_term,
                .i_term = angle_pid.integral_term,
                .d_term = angle_pid.derivative_term,
                .error = angle_pid.error_history[angle_pid.history_counter],
                .stability_factor = stability_factor,
                
                // Vision-Controller-Daten (erweiterte Daten vom IPC)
                .vision_error = vision_cmd_received ? last_vision_command.vision_error : 0.0f,
                .vision_steer_command = vision_cmd_received ? vision_cmd.steer_command : 0.0f,
                .vision_speed_command = vision_cmd_received ? vision_cmd.speed_command : 0.0f,
                .vision_p_term = vision_cmd_received ? last_vision_command.vision_p_term : 0.0f,
                .vision_i_term = vision_cmd_received ? last_vision_command.vision_i_term : 0.0f,
                .vision_d_term = vision_cmd_received ? last_vision_command.vision_d_term : 0.0f,
                .vision_active = vision_cmd_received && (wb_robot_get_time() - last_command_time) < 0.5 ? 1 : 0,
                .vision_mask_coverage = vision_cmd_received ? last_vision_command.mask_coverage : 0.0f
            };
            balance_logging_write(&logger, &log_data);
            
            // ZUSÄTZLICH: Physik-Debug-Ausgabe alle 5 Sekunden
            static int physics_debug_counter = 0;
            if (++physics_debug_counter >= (5000 / timestep)) { // 5 Sekunden
                printf("\n=== PHYSIK-STATUS ===\n");
                printf("Wahrer Roll-Winkel: %.2f° | Simuliert: %.2f°\n", 
                       true_roll_angle * 180.0f / M_PI, roll_angle * 180.0f / M_PI);
                printf("Slip-Winkel: %.2f° | Geschwindigkeit: %.2f m/s\n",
                       bicycle_physics.state.slip_angle * 180.0f / M_PI,
                       sqrtf(bicycle_physics.state.velocity[0]*bicycle_physics.state.velocity[0] + 
                             bicycle_physics.state.velocity[1]*bicycle_physics.state.velocity[1]));
                printf("Seitenkraft: %.1f N | Luftwiderstand: %.1f N\n",
                       bicycle_physics.forces.lateral_force[1],
                       sqrtf(bicycle_physics.forces.drag_force[0]*bicycle_physics.forces.drag_force[0] + 
                             bicycle_physics.forces.drag_force[1]*bicycle_physics.forces.drag_force[1]));
                printf("Rollwiderstand: %.3f Nm\n", bicycle_physics.forces.rolling_resistance_torque);
                printf("==================\n\n");
                physics_debug_counter = 0;
            }
        }
        
        // 9. Display und Status-Updates
        update_display(roll_angle, steering_output, target_speed);
        
        // Status-Ausgabe alle 1000ms (1 Sekunde)
        static int status_counter = 0;
        if (++status_counter >= (1000 / timestep)) {
            print_status(roll_angle, steering_output, target_speed);
            status_counter = 0;
        }
    }
    
    // Cleanup
    balance_logging_close(&logger);
    bicycle_physics_cleanup(&bicycle_physics);
    wb_robot_cleanup();
    printf("\nBalance Control C - Beendet\n");
    
    return 0;
}

static void init_devices(int timestep) {
    // Motoren
    handlebars_motor = wb_robot_get_device("handlebars motor");
    wheel_motor = wb_robot_get_device("motor::crank");
    
    if (handlebars_motor == 0 || wheel_motor == 0) {
        fprintf(stderr, "Fehler: Motoren nicht gefunden!\n");
        exit(1);
    }
    
    // Motor-Initialisierung
    wb_motor_set_position(handlebars_motor, 0.0);
    wb_motor_set_position(wheel_motor, INFINITY);  // Geschwindigkeitsmodus
    wb_motor_set_velocity(wheel_motor, 5.0);  // Positive Startgeschwindigkeit für Vorwärtsfahrt
    
    // IMU Sensor
    imu_sensor = wb_robot_get_device("imu");
    if (imu_sensor == 0) {
        fprintf(stderr, "Fehler: IMU Sensor nicht gefunden!\n");
        exit(1);
    }
    wb_inertial_unit_enable(imu_sensor, timestep * 2);  // IMU mit 10ms statt 5ms
    
    // Keyboard
    wb_keyboard_enable(timestep);
    
    // Display (optional)
    display_device = wb_robot_get_device("display");
    if (display_device != 0) {
        wb_display_attach_camera(display_device, wb_robot_get_device("camera"));
        wb_display_set_color(display_device, 0x00FF00);
        wb_display_set_font(display_device, "Arial", 16, true);
    }
    
    // IPC: Receiver für Vision-Commands
    command_receiver = wb_robot_get_device("command_rx");
    if (command_receiver == 0) {
        fprintf(stderr, "Fehler: Command Receiver nicht gefunden!\n");
        exit(1);
    }
    wb_receiver_enable(command_receiver, timestep);
    
    // IPC: Emitter für Balance-Status
    status_emitter = wb_robot_get_device("status_tx");
    if (status_emitter == 0) {
        fprintf(stderr, "Fehler: Status Emitter nicht gefunden!\n");
        exit(1);
    }
    
    // Robot-Node für Velocity-Messung
    robot_node = wb_supervisor_node_get_self();
    if (robot_node == NULL) {
        printf("Warnung: Supervisor-Funktionen nicht verfügbar\n");
    } else {
        printf("Supervisor-Funktionen verfügbar - Überwache Fahrrad-Orientierung\n");
    }
    
    // Roll-Winkel-Filter initialisieren
    for (int i = 0; i < ROLL_FILTER_SIZE; i++) {
        roll_angle_history[i] = 0.0;
    }
    
    printf("Device-Initialisierung abgeschlossen\n");
    printf("IMU Sensor mit %dms Sampling initialisiert\n", timestep * 2);
}

static void load_and_apply_config(void) {
    // Konfiguration aus JSON laden
    if (balance_config_load(&config, "../../GUI/balance_config.json") != 0) {
        printf("Warnung: Konfigurationsdatei nicht gefunden, verwende Standardwerte\n");
        balance_config_set_defaults(&config);
    }
    
    // PID-Controller mit neuen Parametern initialisieren
    angle_pid = pid_init(
        config.angle_pid.Kp,
        config.angle_pid.Ki,
        config.angle_pid.Kd,
        config.angle_pid.output_min,
        config.angle_pid.output_max,
        config.angle_pid.integral_min,
        config.angle_pid.integral_max
    );
}

static float get_filtered_roll_angle(void) {
    const double *quaternion = wb_inertial_unit_get_quaternion(imu_sensor);
    
    if (quaternion == NULL) {
        printf("DEBUG: Quaternion ist NULL!\n");
        return 0.0;  // Fallback bei Sensor-Fehler
    }
    
    // Quaternion zu Euler-Winkel (Roll um X-Achse)
    double w = quaternion[0];
    double x = quaternion[1]; 
    double y = quaternion[2];
    double z = quaternion[3];
    
    // DEBUG: Quaternion-Werte ausgeben (nur alle 40 Zyklen = 200ms)
    static int debug_counter = 0;
    if (++debug_counter >= 40) {
        printf("DEBUG: Quaternion [w=%.6f, x=%.6f, y=%.6f, z=%.6f]\n", w, x, y, z);
        debug_counter = 0;
    }
    
    // Normalisierung (Sicherheitscheck)
    double norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm > 0.0001) {
        w /= norm; x /= norm; y /= norm; z /= norm;
    } else {
        printf("DEBUG: Quaternion-Norm zu klein: %.6f\n", norm);
        return 0.0;
    }
    
    // Alternative Roll-Winkel-Berechnung (robuster)
    // Roll (X-Achse) - Fahrrad neigt sich seitlich
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll_rad = atan2(sinr_cosp, cosr_cosp);
    float roll_deg = (float)(roll_rad * 180.0 / M_PI);
    
    // KORREKTUR: Fahrrad ist um 180° gedreht, daher Roll-Winkel korrigieren
    // Wenn das Fahrrad aufrecht steht (aber um 180° gedreht), sollte roll_deg = 0° sein
    // Bei 180°-Rotation ist der "aufrechte" Zustand bei ±180°, nicht bei 0°
    if (roll_deg > 90.0) {
        roll_deg = 180.0 - roll_deg;  // 180° -> 0°, 170° -> 10°, etc.
    } else if (roll_deg < -90.0) {
        roll_deg = -180.0 - roll_deg; // -180° -> 0°, -170° -> -10°, etc.
    }
    // Für Werte zwischen -90° und 90° bleibt roll_deg unverändert
    
    // Plausibilitätscheck: Maximale Änderung pro Zeitschritt begrenzen
    static float last_roll = 0.0;
    static int first_run = 1;
    
    if (!first_run) {
        float max_change_per_5ms = 5.0;  // Reduziert auf 5° pro 5ms
        float change = roll_deg - last_roll;
        
        if (fabs(change) > max_change_per_5ms) {
            printf("DEBUG: Unrealistische Änderung erkannt: %.2f° -> %.2f° (Δ=%.2f°)\n", 
                   last_roll, roll_deg, change);
            // Begrenze die Änderung
            if (change > max_change_per_5ms) {
                roll_deg = last_roll + max_change_per_5ms;
            } else if (change < -max_change_per_5ms) {
                roll_deg = last_roll - max_change_per_5ms;
            }
            printf("DEBUG: Korrigiert auf: %.2f°\n", roll_deg);
        }
    }
    
    last_roll = roll_deg;
    first_run = 0;
    
    // DEBUG: Rohe Roll-Winkel-Werte (nur alle 40 Zyklen)
    static int debug_counter2 = 0;
    if (++debug_counter2 >= 40) {
        printf("DEBUG: Raw Roll = %.3f°, Filtered will be applied\n", roll_deg);
        debug_counter2 = 0;
    }
    
    // KEINE automatische Aufrichtung oder Positionsänderung!
    // Lass das Fahrrad in seiner ursprünglichen Position
    
    // Gleitender Durchschnitt für Stabilität
    roll_angle_history[roll_history_index] = roll_deg;
    roll_history_index = (roll_history_index + 1) % ROLL_FILTER_SIZE;
    if (!roll_history_filled && roll_history_index == 0) {
        roll_history_filled = 1;
    }
    
    float filtered_roll = 0.0;
    int count = roll_history_filled ? ROLL_FILTER_SIZE : roll_history_index;
    for (int i = 0; i < count; i++) {
        filtered_roll += roll_angle_history[i];
    }
    return (count > 0) ? filtered_roll / count : 0.0;
}

static void update_display(float roll_angle, float steering_output, float speed) {
    if (display_device == 0) return;
    
    // Display löschen
    wb_display_set_alpha(display_device, 0.0);
    wb_display_fill_rectangle(display_device, 0, 0, 
                             wb_display_get_width(display_device), 
                             wb_display_get_height(display_device));
    wb_display_set_alpha(display_device, 1.0);
    
    // Status-Text
    char status_text[256];
    snprintf(status_text, sizeof(status_text), 
             "Roll: %6.2f° | Steering: %6.2f° | Speed: %5.2f",
             roll_angle, steering_output * 180.0 / M_PI, speed);
    
    wb_display_draw_text(display_device, status_text, 10, 10);
    
    // PID-Terme anzeigen
    char pid_text[256];
    snprintf(pid_text, sizeof(pid_text),
             "P: %6.2f | I: %6.2f | D: %6.2f",
             angle_pid.proportional_term, angle_pid.integral_term, angle_pid.derivative_term);
    
    wb_display_draw_text(display_device, pid_text, 10, 30);
}

static void handle_keyboard_input(void) {
    int key = wb_keyboard_get_key();
    
    switch (key) {
        case 27:  // ESC key
            printf("ESC gedrückt - Beende Controller\n");
            wb_robot_cleanup();
            exit(0);
            break;
            
        case 'r':
        case 'R':
            // Konfiguration neu laden
            printf("Konfiguration wird neu geladen...\n");
            load_and_apply_config();
            break;
            
        case 's':
        case 'S':
            // Status ausgeben
            printf("\n=== Status ===\n");
            printf("Angle PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", 
                   config.angle_pid.Kp, config.angle_pid.Ki, config.angle_pid.Kd);
            printf("Speed: Base=%.1f, Min=%.1f, Max=%.1f\n",
                   config.speed_control.base_speed, config.speed_control.min_speed, config.speed_control.max_speed);
            printf("===============\n\n");
            break;
            
        case 'p':
        case 'P':
            // Physik-Debug-Ausgabe
            bicycle_physics_debug_print(&bicycle_physics);
            break;
            
        case 'w':
        case 'W':
            {
                // Wind-Simulation umschalten
                static float wind_modes[] = {0.0f, 2.0f, 5.0f, 10.0f}; // Windgeschwindigkeiten
                static int wind_mode_index = 0;
                wind_mode_index = (wind_mode_index + 1) % 4;
                float wind_speed = wind_modes[wind_mode_index];
                bicycle_physics_set_environment(&bicycle_physics, wind_speed, 1.5708f, 0.3f); // Seitenwind
                printf("Wind-Simulation: %.1f m/s Seitenwind\n", wind_speed);
                break;
            }
            
        case 'e':
        case 'E':
            // Umgebungsparameter zurücksetzen
            bicycle_physics_set_environment(&bicycle_physics, 0.0f, 0.0f, 0.1f);
            printf("Umgebungsparameter zurückgesetzt\n");
            break;
    }
}

static long long get_time_microseconds(void) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    
    return ((current_time.tv_sec - start_time.tv_sec) * 1000000LL) + 
           (current_time.tv_usec - start_time.tv_usec);
}

static void print_status(float roll_angle, float steering_output, float speed) {
    // Geschwindigkeit vom Supervisor-Node holen (falls verfügbar)
    float actual_speed = speed;  // Fallback
    
    if (robot_node != NULL) {
        const double *velocity = wb_supervisor_node_get_velocity(robot_node);
        if (velocity != NULL) {
            actual_speed = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]) * 3.6; // km/h
        }
    }
    
    printf("Roll: %6.2f° | Steering: %6.2f° | Speed: %5.2f km/h | P:%6.2f I:%6.2f D:%6.2f\r",
           roll_angle, 
           steering_output * 180.0 / M_PI, 
           actual_speed,
           angle_pid.proportional_term,
           angle_pid.integral_term,
           angle_pid.derivative_term);
    fflush(stdout);
}

static int receive_vision_command(vision_command_t *command) {
    static int call_counter = 0;
    if (++call_counter % 100 == 0) {
        printf("DEBUG: receive_vision_command aufgerufen - Aufruf #%d, Queue-Länge: %d\n", 
               call_counter, wb_receiver_get_queue_length(command_receiver));
    }
    
    if (wb_receiver_get_queue_length(command_receiver) > 0) {
        const void *data = wb_receiver_get_data(command_receiver);
        if (data != NULL) {
            int data_size = wb_receiver_get_data_size(command_receiver);
            printf("DEBUG: IPC empfangen - Datengröße: %d Bytes, erwartet: %zu Bytes\n", 
                   data_size, sizeof(vision_command_t));
            
            memcpy(command, data, sizeof(vision_command_t));
            wb_receiver_next_packet(command_receiver);
            
            // Debug: Alle empfangenen Werte anzeigen
            printf("DEBUG: Empfangen - steer=%.3f, speed=%.3f, valid=%d, v_error=%.3f, v_p=%.3f, v_i=%.3f, v_d=%.3f, mask=%.2f\n",
                   command->steer_command, command->speed_command, command->valid,
                   command->vision_error, command->vision_p_term, command->vision_i_term, 
                   command->vision_d_term, command->mask_coverage);
            
            // Plausibilitätsprüfung
            if (command->steer_command >= -1.0f && command->steer_command <= 1.0f &&
                command->speed_command >= 0.0f && command->speed_command <= 1.0f &&
                command->valid == 1) {
                
                last_vision_command = *command;
                last_command_time = wb_robot_get_time();
                printf("DEBUG: Vision-Command AKZEPTIERT\n");
                return 1;  // Gültiges Kommando empfangen
            } else {
                printf("WARNUNG: Ungültiges Vision-Command empfangen: steer=%.3f, speed=%.3f, valid=%d\n",
                       command->steer_command, command->speed_command, command->valid);
            }
        }
    }
    return 0;  // Kein neues Kommando
}

static void send_balance_status(const balance_status_t *status) {
    static int send_counter = 0;
    
    // Sende Status nur alle 25 Zyklen (25 * 2ms = 50ms → 20 Hz)
    if (++send_counter >= 25) {
        wb_emitter_send(status_emitter, status, sizeof(balance_status_t));
        send_counter = 0;
    }
} 