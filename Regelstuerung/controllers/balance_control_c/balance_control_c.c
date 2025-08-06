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
 #include <webots/display.h>
 #include <webots/camera.h>
 #include <webots/receiver.h>
 #include <webots/emitter.h>
#include <webots/position_sensor.h>
 
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <math.h>
 #include <time.h>
 #include <unistd.h>
 #include <sys/time.h>
 #include <stdbool.h>
 
 #include "balance_pid.h"
 #include "balance_config.h"
 #include "balance_logging.h"
 
 // Globale Device-Handles
static WbDeviceTag handlebars_motor;
static WbDeviceTag wheel_motor;
static WbDeviceTag imu_sensor;
static WbDeviceTag display_device;
static WbDeviceTag command_receiver;
static WbDeviceTag status_emitter;
static WbDeviceTag handlebars_sensor;
static WbDeviceTag rear_wheel_sensor;
// static WbDeviceTag camera_device;  // Für zukünftige Erweiterungen
static WbNodeRef robot_node;
 
 // --- Vision-Parameter ---
 // (Alte Filter-Parameter werden nicht mehr verwendet - einfache Speicherung implementiert)
 
 
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
 
 // Vision-Command-Status (vereinfacht)
static vision_command_t last_vision_command = {0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static double last_command_time = 0.0;
 
 // Controller-Zustand
 static pid_controller_t angle_pid;
 static balance_config_t config;
 static balance_logger_t logger;
 
 // Roll-Winkel-Filter
 static float roll_angle_history[ROLL_FILTER_SIZE];
 static int roll_history_index = 0;
 static int roll_history_filled = 0;
 
 // Timing
static struct timeval start_time;
static int config_reload_counter = 0;

// Regelungsstabilisierung - verhindert Aktivierung bis genug Zeit vergangen ist
static bool control_enabled = false;
static double simulation_start_time = 0.0;
 
 // Funktionsprototypen
static void init_devices(int timestep);
static void load_and_apply_config(void);
static float get_filtered_roll_angle(void);
static void update_display(float roll_angle, float steering_output, float speed);
// Keyboard-Input wurde entfernt
static long long get_time_microseconds(void);
static void print_status(float roll_angle, float steering_output, float speed);
static int receive_vision_command(vision_command_t *command);
static void send_balance_status(const balance_status_t *status);
static bool check_roll_angle_stability(float roll_angle, double current_time);
static double compute_rear_wheel_omega(double angle_now);
static double rear_wheel_kmh_from_omega(double omega);
 
 int main(int argc __attribute__((unused)), char **argv __attribute__((unused))) {
     // Webots initialisieren
     wb_robot_init();
     
     int timestep = (int)wb_robot_get_basic_time_step();
     printf("Balance Control C - Timestep: %d ms\n", timestep);
     
         // Startzeit für Timing-Messungen
    gettimeofday(&start_time, NULL);
    
    // Simulationsstart-Zeit speichern für Regelungsverzögerung
    simulation_start_time = wb_robot_get_time();
     
    // Devices initialisieren
    init_devices(timestep); // Motor, IMU, Display, Command Receiver, Status Emitter, Robot Node

    // Konfiguration laden und PID initialisieren
    load_and_apply_config();
     
     // Logging initialisieren
     balance_logging_init(&logger, "../../Monitoring");
     
     // Physik-Simulation wurde entfernt - direkte Regelung ohne zusätzliche Physik-Effekte
     
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

        //--------------------------------
        // 0. Handlebar- und Raddrehwinkel auslesen
        //--------------------------------
        double handlebar_angle = wb_position_sensor_get_value(handlebars_sensor);
        printf("Handlebar angle: %.2f rad\n", handlebar_angle);
        
        // Raddrehwinkel auslesen
        double rear_wheel_angle = wb_position_sensor_get_value(rear_wheel_sensor);
        double omega = compute_rear_wheel_omega(rear_wheel_angle); // rad/s
        double v_kmh = rear_wheel_kmh_from_omega(omega);           // km/h
        printf("Rear wheel: ω = %.2f rad/s, v = %.2f km/h\n", omega, v_kmh);


        //--------------------------------
        // 1. Roll-Winkel messen und filtern
        //--------------------------------
        float roll_angle = get_filtered_roll_angle();
        printf("Amine: roll_angle = %.2f\n", roll_angle);
        
        //--------------------------------
        // 1.5. ZEITCHECK - Regelung erst nach 0.25 Sekunden aktivieren
        //--------------------------------
        double current_webots_time = wb_robot_get_time();
        if (!control_enabled) {
            control_enabled = check_roll_angle_stability(roll_angle, current_webots_time);
            if (control_enabled) {
                printf("STABILISIERUNG: Regelung aktiviert nach %.3f Sekunden\n", 
                       current_webots_time - simulation_start_time);
            }
        }
        
        //--------------------------------
        // 2. PID-Regelung: Roll-Winkel → Lenkwinkel (nur wenn Regelung aktiviert)
        //--------------------------------
        long long current_time = get_time_microseconds();
        float steering_output = 0.0f; // Default: Kein Lenkausschlag
        
        if (control_enabled) {
            steering_output = -pid_compute(&angle_pid, 0.0, roll_angle, current_time);
        }
         
         //--------------------------------
         // 3. Lenkwinkel begrenzen
         //--------------------------------
         if (steering_output > config.mechanical_limits.max_handlebar_angle) 
             steering_output = config.mechanical_limits.max_handlebar_angle;
         else if (steering_output < -config.mechanical_limits.max_handlebar_angle)
             steering_output = -config.mechanical_limits.max_handlebar_angle;
         
         //--------------------------------
         // 4. Vision-Commands empfangen und verarbeiten
         //--------------------------------
         vision_command_t vision_cmd;
         int vision_cmd_received = receive_vision_command(&vision_cmd);
         
        // ========================================
        // 5. VEREINFACHTE ZWEI-EBENEN-REGELUNG 
        // ========================================
        float final_steer = steering_output; 
        float target_speed = config.speed_control.base_speed;
        
        // Vision-Timeout prüfen (konfigurierbar)
        double vision_time = wb_robot_get_time();
        bool vision_active = control_enabled && config.vision_integration.enable_vision && 
                            (vision_time - last_command_time) < config.vision_integration.vision_timeout_seconds;
        
        if (control_enabled) {
            final_steer = steering_output; 
            target_speed = config.speed_control.base_speed;
            
            if (vision_active && last_vision_command.valid) { 
                // STATISCHE GEWICHTUNG aus Konfiguration
                float vision_weight = config.vision_integration.vision_weight; //Vision Anteil
                float balance_weight = config.vision_integration.balance_weight; //Balance Anteil
                
                // Vision-Lenkung berechnen
                float vision_steer = last_vision_command.steer_command * config.mechanical_limits.max_handlebar_angle; //TODO: Wieso nochmal begrenzen?
                
                // EINFACHE KOMBINATION: Statische Gewichtung
                final_steer = vision_weight * -vision_steer + balance_weight * steering_output;
                
                // Geschwindigkeit von Vision-Controller übernehmen
                target_speed = config.speed_control.min_speed + // TODO: Nur balance speed!
                              last_vision_command.speed_command * (config.speed_control.max_speed - config.speed_control.min_speed);
                
                if (vision_cmd_received) {
                    printf("VISION: Vision=%.3f, Balance=%.3f → Final=%.3f (Weights: V=%.1f%%, B=%.1f%%)\n",
                           vision_steer, steering_output, final_steer, 
                           vision_weight*100, balance_weight*100);
                }
            } else {
                // Vision inaktiv → Nur Balance-Regelung
                final_steer = steering_output;
                target_speed = config.speed_control.base_speed;
                
                static double last_timeout_msg = 0.0;
                if (last_command_time > 0.0 && (vision_time - last_timeout_msg) > 2.0) {
                    printf("VISION: Timeout/Inaktiv - Nur Balance-Regelung aktiv\n");
                    last_timeout_msg = vision_time;
                }
            }
        } else {
            // Regelung noch nicht aktiviert - sichere Werte verwenden
            final_steer = 0.0f;
            target_speed = config.speed_control.min_speed;
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

         
         printf("Amine: target_speed: %.2f\n", target_speed);
         printf("Amine: final_steer: %.2f\n", final_steer);
         // 7. Motoren ansteuern
         wb_motor_set_position(handlebars_motor, final_steer);
         wb_motor_set_velocity(wheel_motor, target_speed);  // Positive Geschwindigkeit für Vorwärtsfahrt
         
        // 8. Balance-Status an Vision-Controller senden
        float stability_factor = fabs(steering_output) / config.mechanical_limits.max_handlebar_angle;
        balance_status_t status = {
            .roll_angle = roll_angle,
            .steering_output = final_steer,
            .current_speed = target_speed,
            .stability_factor = stability_factor
        };
        send_balance_status(&status);
        
        // 8. Erweiterte Logging-Daten (inklusive Physik-Informationen + Vision-Daten)
        if (config.system.enable_logging) {
            balance_log_data_t log_data = {
                // Balance-Controller-Daten
                .timestamp = wb_robot_get_time(),
                .roll_angle = roll_angle,
                .steering_output = steering_output,
                .final_steer = final_steer,
                .target_speed = target_speed,
                .actual_speed_kmh = (float)v_kmh,
                .actual_handlebar_angle = (float)handlebar_angle,
                .p_term = angle_pid.proportional_term,
                .i_term = angle_pid.integral_term,
                .d_term = angle_pid.derivative_term,
                .error = angle_pid.error_history[angle_pid.history_counter],
                .stability_factor = stability_factor,
                
                // Vision-Controller-Daten (vereinfacht)
                .vision_error = vision_active ? last_vision_command.vision_error : 0.0f,
                .vision_steer_command = vision_active ? last_vision_command.steer_command : 0.0f,
                .vision_speed_command = vision_active ? last_vision_command.speed_command : 0.0f,
                .vision_p_term = vision_active ? last_vision_command.vision_p_term : 0.0f,
                .vision_i_term = vision_active ? last_vision_command.vision_i_term : 0.0f,
                .vision_d_term = vision_active ? last_vision_command.vision_d_term : 0.0f,
                .vision_active = vision_active ? 1 : 0,
                .vision_mask_coverage = vision_active ? last_vision_command.mask_coverage : 0.0f
            };
            balance_logging_write(&logger, &log_data);
             
             // ZUSÄTZLICH: Physik-Debug-Ausgabe alle 5 Sekunden
             static int physics_debug_counter = 0;
             if (++physics_debug_counter >= (5000 / timestep)) {
                 printf("PHYSIK-DEBUG: Kräfte und Momente - alle 5s\n");
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
     } // Ende der while-Schleife
     
     // Cleanup
     balance_logging_close(&logger);
     // Physik-System wurde entfernt
     wb_robot_cleanup();
     
     return 0;
 }
 
 static void init_devices(int timestep) {
     // Motoren
     handlebars_motor = wb_robot_get_device("handlebars motor");
     wheel_motor = wb_robot_get_device("motor::wheel");
     
     if (handlebars_motor == 0 || wheel_motor == 0) {
         fprintf(stderr, "Fehler: Motoren nicht gefunden!\n");
         exit(1);
     }
     
    // Motor-Initialisierung
    wb_motor_set_position(handlebars_motor, 0.0); //
    wb_motor_set_position(wheel_motor, INFINITY);  // Geschwindigkeitsmodus

    // Lenkwinkelsensor
    handlebars_sensor = wb_robot_get_device("handlebars sensor");
    rear_wheel_sensor = wb_robot_get_device("rear wheel sensor");

    if (handlebars_sensor == 0 || rear_wheel_sensor == 0) {
        fprintf(stderr, "Fehler: Handlebar-Sensor nicht gefunden!\n");
        exit(1);
    }
    wb_position_sensor_enable(handlebars_sensor, timestep);
    wb_position_sensor_enable(rear_wheel_sensor, timestep);

    // IMU Sensor
    imu_sensor = wb_robot_get_device("imu");
     if (imu_sensor == 0) {
         fprintf(stderr, "Fehler: IMU Sensor nicht gefunden!\n");
         exit(1);
     }
     wb_inertial_unit_enable(imu_sensor, timestep * 2);  // IMU mit 10ms statt 5ms
     
         // Keyboard wurde entfernt
     
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
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
    
    if (rpy == NULL) {
        printf("DEBUG: RPY ist NULL!\n");
        return 0.0;  // Fallback bei Sensor-Fehler
    }
    
    // Roll- und Yaw-Winkel auslesen
    double roll_rad = rpy[0];
    double yaw = rpy[2];

         // DEBUG: RPY-Werte ausgeben (reduziert auf alle 200 Zyklen = 1000ms)
     static int debug_counter = 0;
     if (++debug_counter >= 200) {
         printf("DEBUG: RPY [roll=%.6f, pitch=%.6f, yaw=%.6f]\n", rpy[0], rpy[1], rpy[2]);
         debug_counter = 0;
     }

    // Roll-Vorzeichen an Fahrtrichtung koppeln
    if (cos(yaw) < 0.0) {
        roll_rad = -roll_rad;
    }

    float roll_deg = (float)(roll_rad * 180.0 / M_PI);
    
     // Plausibilitätscheck: Maximale Änderung pro Zeitschritt begrenzen
     static float last_roll = 0.0;
     static int first_run = 1;
     
     if (!first_run) {
                   float max_change_per_5ms = 15.0;  // Erhöht auf 15° pro 5ms für schnellere Reaktion
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
     
           // DEBUG: Rohe Roll-Winkel-Werte (reduziert auf alle 200 Zyklen)
      static int debug_counter2 = 0;
      if (++debug_counter2 >= 200) {
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
 
 // Keyboard-Input komplett entfernt
 
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
                 
                 printf("DEBUG: Vision-Command AKZEPTIERT - Steer=%.3f, Error=%.3f, Coverage=%.1f%%\n",
                        command->steer_command, command->vision_error, command->mask_coverage);
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

static bool check_roll_angle_stability(float roll_angle, double current_time) {
    (void)roll_angle; // Parameter nicht verwendet
    
    // Einfacher Zeitcheck: Regelung nach 0.1 Sekunden aktivieren (reduziert von 0.25s)
    // Grund: IMU-Ausschlag in den ersten ~0.05s umgehen, aber schneller reagieren
    double elapsed_time = current_time - simulation_start_time;
    return elapsed_time >= 0.2;
} 

static double compute_rear_wheel_omega(double angle_now) {
    static int initialized = 0;
    static double prev_angle = 0.0;
    static double prev_time  = 0.0;  // s (Webots-Zeit)
    static double omega_filt = 0.0;
  
    double t = wb_robot_get_time(); // s
    if (!initialized) {
      prev_angle = angle_now;
      prev_time  = t;
      initialized = 1;
      return 0.0;
    }
  
    double dt = t - prev_time;
    if (dt <= 0.0) return omega_filt;
  
    // Δθ (bei ungebundenen Gelenken meist kontinuierlich; unwrap schadet nicht)
    double dtheta = angle_now - prev_angle;
    while (dtheta >  M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
  
    double omega = dtheta / dt;       // rad/s
  
    // leichte Glättung gegen Rauschen
    const double alpha = 0.2;         // 0..1 (höher = weniger Glättung)
    omega_filt = alpha * omega + (1.0 - alpha) * omega_filt;
  
    prev_angle = angle_now;
    prev_time  = t;
    return omega_filt;
  }
  
  static double rear_wheel_kmh_from_omega(double omega) {
    const double R = 0.45;            // m (dein Rad-Radius aus boundingObject)
    return omega * R * 3.6;           // km/h
  }