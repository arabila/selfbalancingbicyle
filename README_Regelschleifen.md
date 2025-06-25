# Regelschleifen-Analyse: Selbstbalancierendes Fahrrad

## Übersicht

Diese Dokumentation analysiert die Regelschleifen aus der BachelorArbeit 2023 (Jonah Zander) und erklärt deren Umsetzung für Webots. Die autobike-Implementierung verwendet eine **mehrstufige Kaskaderegelung** für die Fahrradbalancierung.

## 1. Systemarchitektur

### 1.1 Hardware-Aufbau (Original)
```
[BNO055 IMU] → [BeagleBone Black] → [MD49 Motorcontroller] → [Lenkmotor]
                     ↓
[Fernbedienung] → [Geschwindigkeitsmotor (HNM)]
```

### 1.2 Webots-Aufbau (Simulation)
```
[IMU Sensor] → [Webots Controller] → [Handlebars Motor]
                     ↓
[Keyboard Input] → [Wheel Motor]
```

## 2. Regelschleifen-Detail-Analyse

### 2.1 Hauptregelschleife: Angle-to-Steering PID

**Zweck**: Konvertiert Roll-Winkel in Lenkwinkel-Sollwert für die Stabilisierung.

#### 2.1.1 Parameter (aus autobike.c)
```c
#define ANGLE_PID_KP                    10.0f
#define ANGLE_PID_KI                    0.0f
#define ANGLE_PID_KD                    2.2f 
#define ANGLE_PID_OUTPUT_MIN            -150.0f
#define ANGLE_PID_OUTPUT_MAX            150.0f
#define ANGLE_PID_INTEGRAL_MIN          -60.0f
#define ANGLE_PID_INTEGRAL_MAX          60.0f
```

#### 2.1.2 Funktionsweise
```c
// Aus autobike.c, Zeile ~200
angle_to_steering_value = (int)(steering_offset + 
    pid_output(&angle_to_steering_pid, angle_offset, 
               (float)temp_euler_angles.roll, ms_since_start));
```

**Eingabe**: 
- `angle_offset`: Gewünschter Roll-Winkel (normalerweise 0°)
- `temp_euler_angles.roll`: Aktueller Roll-Winkel vom BNO055

**Ausgabe**: 
- Lenkwinkel-Sollwert in Encoder-Schritten

#### 2.1.3 Regelverhalten
- **P-Anteil (Kp=10.0)**: Starke Reaktion auf Roll-Abweichung
- **I-Anteil (Ki=0.0)**: Deaktiviert - verhindert Aufschaukeln
- **D-Anteil (Kd=2.2)**: Dämpft schnelle Bewegungen

### 2.2 Sekundäre Regelschleife: Steering Position PID

**Zweck**: Regelt die physische Lenkposition basierend auf Sollwert.

#### 2.2.1 Parameter
```c
#define MD49_PID_KP                     850.0f
#define MD49_PID_KI                     300.0f
#define MD49_PID_KD                     30.0f
#define MD49_PID_OUTPUT_MIN             -100000.0f
#define MD49_PID_OUTPUT_MAX             100000.0f
```

#### 2.2.2 Funktionsweise
```c
// Aus autobike.c, thread_md49_read()
new_steering_value = (int)pid_output(&steering_pid, 
                                    setpoint_variable_copy, 
                                    (float)encoder_value, 
                                    ms_since_start);
bts7960_turn(&bts_fd, new_steering_value);
```

**Eingabe**:
- `setpoint_variable_copy`: Sollwert von Angle-to-Steering PID
- `encoder_value`: Aktuelle Lenkposition (MD49 Encoder)

**Ausgabe**:
- PWM-Signal für BTS7960 Motorcontroller

### 2.3 Tertiäre Regelschleife: Speed Control (HNM)

**Zweck**: Geschwindigkeitsregelung für Vorwärtsfahrt.

#### 2.3.1 Parameter
```c
#define HNM_PID_KP                      0.0f
#define HNM_PID_KI                      1600.0f
#define HNM_PID_KD                      0.0f
#define HNM_REMOTE_SPEED                3.1f 
```

#### 2.3.2 Funktionsweise
```c
// Vereinfachte Geschwindigkeitsregelung
if(remote_speed_copy == 1){
    setpoint_speed = HNM_REMOTE_SPEED;
    new_pwm_value = 31000;
} else{
    setpoint_speed = 0.0f;
    new_pwm_value = 0;
}
```

## 3. Threading-Architektur

### 3.1 Original-Threads (autobike.c)

| Thread | Frequenz | Funktion |
|--------|----------|----------|
| `thread_bno055_read` | 200Hz (5ms) | IMU-Daten lesen, Angle-PID |
| `thread_md49_read` | 1000Hz (1ms) | Lenkung regeln, Steering-PID |
| `thread_hnm_read` | 6.7Hz (150ms) | Geschwindigkeit regeln |
| `thread_watchdog` | 33Hz (30ms) | System-Überwachung |
| `thread_print` | 10Hz (100ms) | Status-Ausgabe |

### 3.2 Datenfluss
```
BNO055 Thread:
  IMU → Roll-Winkel → Angle-PID → Lenkwinkel-Sollwert
                                         ↓
MD49 Thread:
  Encoder ← Lenkwinkel-Sollwert → Steering-PID → Motor-PWM

HNM Thread:
  Speed-Sensor → Speed-PID → Antriebsmotor-PWM
```

## 4. Webots-Umsetzung

### 4.1 Vereinfachte Architektur

Da Webots die Hardware-Komplexität abstrahiert, können wir die Regelschleifen vereinfachen:

```c
// Webots-Hauptschleife (vereinfacht)
while (wb_robot_step(timestep) != -1) {
    // 1. Roll-Winkel messen
    float roll_angle = get_filtered_roll_angle();
    
    // 2. Angle-to-Steering PID (direkter Zugriff auf Motor)
    float steering_output = pid_compute(&angle_pid, 0.0, roll_angle, current_time);
    
    // 3. Direkte Motoransteuerung (keine Encoder-Regelung nötig)
    wb_motor_set_position(handlebars_motor, steering_output);
    
    // 4. Geschwindigkeitsanpassung basierend auf Stabilität
    float speed = base_speed * (1.0 - abs(steering_output) * 0.5);
    wb_motor_set_velocity(wheel_motor, speed);
}
```

### 4.2 PID-Controller-Implementierung

```c
typedef struct {
    float derivative_history[HISTORY_LEN];
    float error_history[HISTORY_LEN];
    int history_counter;
    long long time_history[HISTORY_LEN];
    
    float proportional_term;
    float integral_term;
    float derivative_term;
    float Kp, Ki, Kd;
    
    float output_min, output_max;
    float integral_min, integral_max;
} pid_controller_t;

float pid_compute(pid_controller_t *pid, float setpoint, 
                  float process_variable, long long current_time_us) {
    
    // Fehler berechnen
    float error = setpoint - process_variable;
    pid->error_history[pid->history_counter] = error;
    pid->time_history[pid->history_counter] = current_time_us;
    
    // P-Term
    pid->proportional_term = pid->Kp * error;
    
    // I-Term mit Anti-Windup
    float dt = (pid->time_history[pid->history_counter] - 
                pid->time_history[(pid->history_counter + HISTORY_LEN - 1) % HISTORY_LEN]) / 1000000.0;
    pid->integral_term += pid->Ki * error * dt;
    
    // Integral-Begrenzung
    if (pid->integral_term < pid->integral_min) 
        pid->integral_term = pid->integral_min;
    else if (pid->integral_term > pid->integral_max) 
        pid->integral_term = pid->integral_max;
    
    // D-Term (gleitender Durchschnitt über Historie)
    if (dt > 0) {
        float current_derivative = (error - 
            pid->error_history[(pid->history_counter + HISTORY_LEN - 1) % HISTORY_LEN]) / dt;
        pid->derivative_history[pid->history_counter] = current_derivative;
        
        float derivative_sum = 0;
        for(int i = 0; i < HISTORY_LEN; i++) {
            derivative_sum += pid->derivative_history[i];
        }
        pid->derivative_term = pid->Kd * (derivative_sum / HISTORY_LEN);
    }
    
    // Ausgabe berechnen und begrenzen
    float output = pid->proportional_term + pid->integral_term + pid->derivative_term;
    if (output < pid->output_min) output = pid->output_min;
    else if (output > pid->output_max) output = pid->output_max;
    
    // History-Counter aktualisieren
    pid->history_counter = (pid->history_counter + 1) % HISTORY_LEN;
    
    return output;
}
```

### 4.3 IMU-Datenverarbeitung

```c
float get_filtered_roll_angle(void) {
    const double *quaternion = wb_inertial_unit_get_quaternion(imu_sensor);
    
    // Quaternion zu Euler-Winkel (Roll um X-Achse)
    double w = quaternion[0];
    double x = quaternion[1]; 
    double y = quaternion[2];
    double z = quaternion[3];
    
    // Roll-Winkel berechnen
    double roll_rad = atan2(2 * (w * x + y * z), w*w - x*x - y*y + z*z);
    float roll_deg = (float)(roll_rad * 180.0 / M_PI);
    
    // Begrenzung auf realistische Werte
    if (roll_deg > 45.0) roll_deg = 45.0;
    else if (roll_deg < -45.0) roll_deg = -45.0;
    
    // Gleitender Durchschnitt für Stabilität
    static float roll_history[HISTORY_LEN] = {0};
    static int roll_index = 0;
    
    roll_history[roll_index] = roll_deg;
    roll_index = (roll_index + 1) % HISTORY_LEN;
    
    float filtered_roll = 0;
    for(int i = 0; i < HISTORY_LEN; i++) {
        filtered_roll += roll_history[i];
    }
    return filtered_roll / HISTORY_LEN;
}
```

## 5. Konfiguration über GUI

### 5.1 JSON-Konfigurationsdatei
```json
{
    "balance_control": {
        "angle_pid": {
            "Kp": 10.0,
            "Ki": 0.0,
            "Kd": 2.2,
            "output_min": -0.3,
            "output_max": 0.3,
            "integral_min": -60.0,
            "integral_max": 60.0
        },
        "speed_control": {
            "base_speed": 5.0,
            "min_speed": 3.0,
            "max_speed": 8.0,
            "stability_factor": 0.5
        },
        "system": {
            "max_handlebar_angle": 0.32,
            "max_roll_angle": 45.0,
            "filter_size": 5,
            "enable_logging": true
        }
    }
}
```

### 5.2 GUI-Integration
Die bestehende GUI kann erweitert werden, um die Balance-Parameter zu konfigurieren:

```python
# Erweiterung der GUI um Balance-Parameter
balance_params = {
    "Angle PID Kp": {"value": 10.0, "min": 0.0, "max": 50.0},
    "Angle PID Ki": {"value": 0.0, "min": 0.0, "max": 10.0},
    "Angle PID Kd": {"value": 2.2, "min": 0.0, "max": 10.0},
    "Base Speed": {"value": 5.0, "min": 1.0, "max": 10.0},
    "Max Handlebar Angle": {"value": 0.32, "min": 0.1, "max": 0.5}
}
```

## 6. Vorteile der Webots-Umsetzung

### 6.1 Vereinfachungen
- **Keine Hardware-Abstraktionsschicht**: Direkter Zugriff auf Motoren
- **Keine Encoder-Regelung**: Position direkt setzbar
- **Keine Threading-Komplexität**: Synchrone Hauptschleife
- **Keine Watchdog-Mechanismen**: Webots-Stabilität

### 6.2 Zusätzliche Möglichkeiten
- **Perfekte Sensoren**: Kein Rauschen, keine Ausfälle
- **Reproduzierbare Tests**: Identische Startbedingungen
- **Visualisierung**: 3D-Darstellung der Regelungsdynamik
- **Parameterstudien**: Schnelle Iteration verschiedener PID-Werte

## 7. Implementierungsschritte

### 7.1 Phase 1: Basis-Balancierung
1. PID-Controller in C implementieren
2. IMU-Datenverarbeitung
3. Direkte Motor-Ansteuerung
4. Grundlegende Stabilisierung

### 7.2 Phase 2: GUI-Integration
1. JSON-Konfigurationsdatei
2. Laufzeit-Parameterwechsel
3. Monitoring und Logging
4. Visualisierung der PID-Terme

### 7.3 Phase 3: Erweiterte Features
1. Adaptive Parameteranpassung
2. Störungsbehandlung
3. Performance-Optimierung
4. Crash-Erkennung und -Recovery

## 8. Theoretische Grundlagen

### 8.1 Fahrrad-Dynamik
Das selbstbalancierende Fahrrad ist ein **umgekehrtes Pendel** mit folgenden Eigenschaften:

- **Instabile Ruhelage**: Ohne Regelung fällt das Fahrrad um
- **Gyroskopische Effekte**: Raddrehung stabilisiert bei Geschwindigkeit
- **Lenkkopplung**: Lenkung erzeugt Zentrifugalkraft für Aufrichtung

### 8.2 Regelungstheorie
```
Störung (Wind, Unebenheiten)
         ↓
Sollwert (0°) → [PID] → Lenkwinkel → [Fahrrad] → Roll-Winkel
         ↑                                           ↓
         ←←←←←←←← Rückführung ←←←←←←←←←←←←←←←←←←←←←←←←
```

**Kritische Aspekte**:
- **Stabilität**: System darf nicht aufschaukeln
- **Geschwindigkeit**: Regelung muss schnell genug sein
- **Robustheit**: Funktioniert auch bei Störungen

## 9. Debugging und Tuning

### 9.1 Typische Probleme
- **Aufschaukeln**: Kp zu hoch oder Ki zu hoch
- **Träge Reaktion**: Kp zu niedrig oder Kd zu hoch
- **Oszillation**: Kd zu niedrig oder Systemverzögerung

### 9.2 Tuning-Strategie
1. **Nur P-Regler**: Kp erhöhen bis Oszillation
2. **D-Anteil hinzufügen**: Kd für Dämpfung
3. **I-Anteil vorsichtig**: Nur bei bleibender Regelabweichung

### 9.3 Monitoring
```csv
timestamp,roll_angle,steering_output,speed,P_term,I_term,D_term,error
1.234,2.5,-0.15,4.8,25.0,0.0,-3.2,2.5
1.244,1.8,-0.12,4.9,18.0,0.0,-2.1,1.8
```

Diese Daten ermöglichen die Analyse des Regelverhaltens und gezieltes Tuning.

---

**Zusammenfassung**: Die autobike-Implementierung verwendet eine bewährte Kaskaderegelung, die sich durch Vereinfachung hervorragend für Webots eignet. Die Fokussierung auf Roll-Winkel-basierte Balancierung eliminiert die Komplexität der Spurverfolgung und ermöglicht eine robuste, konfigurierbare Lösung. 