/*
 * Little Bicycle V2 Controller - C Implementation
 * Based on Jonah Zander's Bachelor Thesis (2023)
 * 
 * Self-balancing bicycle control using PID regulation
 * Main control loop: Roll angle -> Steering angle
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/keyboard.h>
#include <webots/supervisor.h>
#include <webots/gps.h>

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Zeitkonstanten
#define TIME_STEP 16
#define HISTORY_LEN 5

// PID-Parameter für Winkel -> Lenkung (aus Zanders Arbeit)
#define ANGLE_PID_KP 10.0f
#define ANGLE_PID_KI 0.0f
#define ANGLE_PID_KD 2.2f
#define ANGLE_PID_OUTPUT_MIN -150.0f
#define ANGLE_PID_OUTPUT_MAX 150.0f
#define ANGLE_PID_INTEGRAL_MIN -60.0f
#define ANGLE_PID_INTEGRAL_MAX 60.0f

// Fahrradgeschwindigkeit
#define MAX_SPEED 6.0
#define MIN_SPEED 3.0

// Lenkwinkel-Grenzen
#define HANDLEBAR_MAX_ANGLE 0.1920  // 11° in Radiant
#define DEGREE_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEGREE (180.0 / M_PI)

// PID-Controller Struktur
typedef struct {
    float error_history[HISTORY_LEN];
    float derivative_history[HISTORY_LEN];
    int history_counter;
    double time_history[HISTORY_LEN];
    
    float proportional_term;
    float integral_term;
    float derivative_term;
    
    float Kp, Ki, Kd;
    float output_min, output_max;
    float integral_min, integral_max;
} PIDController;

// Globale Variablen
static WbDeviceTag wheel_motor, handlebar_motor;
static WbDeviceTag inertial_unit;
static WbDeviceTag camera;
static WbDeviceTag display;
static WbDeviceTag gps;
static WbNodeRef robot_node;

static PIDController angle_pid;

// Funktion zum Initialisieren des PID-Controllers
void pid_init(PIDController *pid, float kp, float ki, float kd, 
              float output_min, float output_max, 
              float integral_min, float integral_max) {
    
    // Historie initialisieren
    for(int i = 0; i < HISTORY_LEN; i++) {
        pid->error_history[i] = 0.0f;
        pid->derivative_history[i] = 0.0f;
        pid->time_history[i] = 0.0;
    }
    
    pid->history_counter = 0;
    pid->proportional_term = 0.0f;
    pid->integral_term = 0.0f;
    pid->derivative_term = 0.0f;
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

// PID-Ausgabe berechnen
float pid_output(PIDController *pid, float setpoint, float process_variable, double current_time) {
    // Fehler berechnen
    pid->error_history[pid->history_counter] = setpoint - process_variable;
    pid->time_history[pid->history_counter] = current_time;
    
    // Proportional-Term
    pid->proportional_term = pid->Kp * pid->error_history[pid->history_counter];
    
    // Integral-Term (mit Zeitdifferenz)
    int prev_index = (pid->history_counter + HISTORY_LEN - 1) % HISTORY_LEN;
    double dt = (pid->time_history[pid->history_counter] - pid->time_history[prev_index]) / 1000.0;
    
    if(dt > 0) {
        pid->integral_term += pid->Ki * pid->error_history[pid->history_counter] * dt;
        
        // Anti-Windup für Integral-Term
        if(pid->integral_term < pid->integral_min) {
            pid->integral_term = pid->integral_min;
        } else if(pid->integral_term > pid->integral_max) {
            pid->integral_term = pid->integral_max;
        }
    }
    
    // Derivative-Term (geglättet über Historie)
    if(dt > 0) {
        pid->derivative_history[pid->history_counter] = 
            (pid->error_history[pid->history_counter] - pid->error_history[prev_index]) / dt;
    }
    
    // Mittelwert der Ableitungen für Glättung
    pid->derivative_term = 0.0f;
    for(int i = 0; i < HISTORY_LEN; i++) {
        pid->derivative_term += pid->derivative_history[i];
    }
    pid->derivative_term = (pid->derivative_term / HISTORY_LEN) * pid->Kd;
    
    // Gesamte PID-Ausgabe
    float output = pid->proportional_term + pid->integral_term + pid->derivative_term;
    
    // Ausgabe begrenzen
    if(output < pid->output_min) {
        output = pid->output_min;
    } else if(output > pid->output_max) {
        output = pid->output_max;
    }
    
    // Historie aktualisieren
    pid->history_counter = (pid->history_counter + 1) % HISTORY_LEN;
    
    return output;
}

// Initialisierung aller Devices
void initialize_devices() {
    // Motoren
    wheel_motor = wb_robot_get_device("motor::crank");
    handlebar_motor = wb_robot_get_device("handlebars motor");
    
    wb_motor_set_position(wheel_motor, INFINITY);
    wb_motor_set_velocity(wheel_motor, MAX_SPEED);
    wb_motor_set_position(handlebar_motor, 0.0);
    
    // Inertial Unit für Orientierung
    inertial_unit = wb_robot_get_device("inertial unit");
    if(inertial_unit == 0) {
        printf("Warnung: Inertial Unit nicht gefunden, verwende GPS für Orientierung\n");
        gps = wb_robot_get_device("gps");
        if(gps != 0) {
            wb_gps_enable(gps, TIME_STEP);
        }
    } else {
        wb_inertial_unit_enable(inertial_unit, TIME_STEP);
    }
    
    // Kamera und Display
    camera = wb_robot_get_device("camera"); 
    display = wb_robot_get_device("display");
    
    if(camera != 0) {
        wb_camera_enable(camera, TIME_STEP * 4);
    }
    
    // Supervisor für Roboter-Node
    robot_node = wb_supervisor_node_get_self();
    
    // Tastatur aktivieren
    wb_keyboard_enable(TIME_STEP);
    
    printf("Alle Devices initialisiert\n");
}

// Roll-Winkel aus Supervisor-Orientierung extrahieren
float get_roll_angle() {
    if(robot_node == 0) return 0.0f;
    
    const double *orientation = wb_supervisor_node_get_orientation(robot_node);
    if(orientation == NULL) return 0.0f;
    
    // Rotationsmatrix in Euler-Winkel umwandeln
    // Roll-Winkel aus Rotationsmatrix extrahieren
    float roll = atan2(orientation[7], orientation[8]) * RAD_TO_DEGREE;
    
    return roll;
}

// Alternative: Roll-Winkel aus Inertial Unit
float get_roll_angle_from_imu() {
    if(inertial_unit == 0) return get_roll_angle();
    
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    if(rpy == NULL) return get_roll_angle();
    
    return (float)(rpy[0] * RAD_TO_DEGREE);  // Roll in Grad
}

// Geschwindigkeit basierend auf Roboter-Geschwindigkeit
float get_robot_speed() {
    if(robot_node == 0) return 0.0f;
    
    const double *velocity = wb_supervisor_node_get_velocity(robot_node);
    if(velocity == NULL) return 0.0f;
    
    // Geschwindigkeitsvektor -> Skalare Geschwindigkeit
    double speed = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2]);
    return (float)(speed * 3.6);  // m/s -> km/h
}

// Status auf Display anzeigen
void display_status(float roll_angle, float steering_angle, float speed, double time) {
    if(display == 0) return;
    
    wb_display_set_color(display, 0x00FF00);
    wb_display_set_font(display, "Verdana", 12, true);
    
    char status_text[256];
    snprintf(status_text, sizeof(status_text), 
             "Zeit: %.1fs | Roll: %.2f° | Lenkung: %.2f° | Speed: %.1f km/h", 
             time, roll_angle, steering_angle * RAD_TO_DEGREE, speed);
    
    wb_display_draw_text(display, status_text, 10, 10);
    
    // PID-Werte anzeigen
    snprintf(status_text, sizeof(status_text),
             "PID: P=%.2f I=%.2f D=%.2f", 
             angle_pid.proportional_term, angle_pid.integral_term, angle_pid.derivative_term);
    wb_display_draw_text(display, status_text, 10, 30);
}

// Hauptsteuerungsschleife
void control_bicycle() {
    double current_time = wb_robot_get_time() * 1000.0;  // ms
    
    // Roll-Winkel messen (Soll: 0°, geradeaus)
    float roll_angle = get_roll_angle_from_imu();
    
    // PID-Regelung: Soll-Roll-Winkel = 0°
    float steering_output = pid_output(&angle_pid, 0.0f, roll_angle, current_time);
    
    // Lenkwinkel berechnen (Zanders Logik angepasst)
    float steering_angle = steering_output * 0.001f;  // Skalierung anpassen
    
    // Lenkwinkel begrenzen
    if(steering_angle > HANDLEBAR_MAX_ANGLE) {
        steering_angle = HANDLEBAR_MAX_ANGLE;
    } else if(steering_angle < -HANDLEBAR_MAX_ANGLE) {
        steering_angle = -HANDLEBAR_MAX_ANGLE;
    }
    
    // Geschwindigkeit basierend auf Stabilität anpassen
    float speed = MAX_SPEED;
    float abs_roll = fabs(roll_angle);
    if(abs_roll > 5.0f) {  // Bei großer Neigung langsamer fahren
        speed = MAX_SPEED - (abs_roll - 5.0f) * 0.2f;
        if(speed < MIN_SPEED) speed = MIN_SPEED;
    }
    
    // Motoren ansteuern
    wb_motor_set_position(handlebar_motor, -steering_angle);  // Negativ für korrekte Richtung
    wb_motor_set_velocity(wheel_motor, speed);
    
    // Status anzeigen
    display_status(roll_angle, steering_angle, get_robot_speed(), current_time / 1000.0);
    
    // Debug-Ausgabe
    if((int)current_time % 1000 < TIME_STEP) {  // Jede Sekunde
        printf("Roll: %.2f° | Steering: %.2f° | Speed: %.1f | PID: P=%.2f I=%.2f D=%.2f\n",
               roll_angle, steering_angle * RAD_TO_DEGREE, speed,
               angle_pid.proportional_term, angle_pid.integral_term, angle_pid.derivative_term);
    }
}

// Tastatureingaben verarbeiten
void process_keyboard() {
    int key = wb_keyboard_get_key();
    
    if(key >= 0) {
        switch(key) {
            case WB_KEYBOARD_UP:    // Geschwindigkeit erhöhen
                wb_motor_set_velocity(wheel_motor, MAX_SPEED + 2.0);
                break;
            case WB_KEYBOARD_DOWN:  // Geschwindigkeit verringern  
                wb_motor_set_velocity(wheel_motor, MIN_SPEED);
                break;
            case WB_KEYBOARD_LEFT:  // Manuell links lenken
                wb_motor_set_position(handlebar_motor, HANDLEBAR_MAX_ANGLE);
                break;
            case WB_KEYBOARD_RIGHT: // Manuell rechts lenken
                wb_motor_set_position(handlebar_motor, -HANDLEBAR_MAX_ANGLE);
                break;
            case ' ':  // Leertaste: Zurück zur automatischen Regelung
                printf("Automatische Regelung aktiviert\n");
                break;
            case 'R':  // Reset
                printf("PID-Controller zurückgesetzt\n");
                pid_init(&angle_pid, ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD,
                         ANGLE_PID_OUTPUT_MIN, ANGLE_PID_OUTPUT_MAX,
                         ANGLE_PID_INTEGRAL_MIN, ANGLE_PID_INTEGRAL_MAX);
                break;
        }
    }
}

int main(int argc, char **argv) {
    // Webots-Robot initialisieren
    wb_robot_init();
    
    printf("=== Little Bicycle V2 Controller (C) ===\n");
    printf("Basierend auf Jonah Zanders Bachelorarbeit 2023\n");
    printf("PID-Parameter: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", 
           ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD);
    printf("Steuerung: Pfeiltasten für manuell, Leertaste für automatisch, R für Reset\n\n");
    
    // Devices initialisieren
    initialize_devices();
    
    // PID-Controller initialisieren
    pid_init(&angle_pid, ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD,
             ANGLE_PID_OUTPUT_MIN, ANGLE_PID_OUTPUT_MAX,
             ANGLE_PID_INTEGRAL_MIN, ANGLE_PID_INTEGRAL_MAX);
    
    // Hauptschleife
    while(wb_robot_step(TIME_STEP) != -1) {
        process_keyboard();
        control_bicycle();
    }
    
    wb_robot_cleanup();
    return 0;
} 