# Balance-Regelung für Selbstbalancierendes Fahrrad - Technische Dokumentation

## Überblick

Das Balance-Control-System implementiert eine hochentwickelte Roll-Winkel-basierte PID-Regelung für ein selbstbalancierendes Fahrrad in der Webots-Simulationsumgebung. Das System basiert auf der bewährten autobike-Architektur von Jonah Zander (BachelorArbeit 2023) und wurde um erweiterte Physik-Simulation und umfassende Monitoring-Funktionen erweitert.

## Systemarchitektur

### Regelkreis-Übersicht

```
[IMU Sensor] → [Roll-Winkel-Filter] → [Angle PID] → [Handlebars Motor]
      ↓                ↑                    ↓              ↓
[Physik-Simulation] ← [Sensor-Simulation]  ↓         [Wheel Motor]
      ↓                                     ↓              ↓
[Umwelt-Effekte] → [Störgrößen] → [Extended Physics] → [Webots-Welt]
                                           ↓
[JSON Config] ← [GUI] → [Live Monitoring] → [CSV Logger]
```

### Datenfluss im Detail

1. **Sensorerfassung**: IMU liefert Quaternion-Daten
2. **Winkelberechnung**: Quaternion → Roll-Winkel (gefiltert)
3. **Physik-Simulation**: Erweiterte Kräfte und Sensorfehler
4. **PID-Regelung**: Roll-Winkel → Lenkwinkel-Sollwert
5. **Geschwindigkeitsadaption**: Stabilität → adaptive Geschwindigkeit
6. **Motoransteuerung**: Position (Lenkung) + Geschwindigkeit (Antrieb)
7. **Monitoring**: Kontinuierliche Datenaufzeichnung

## Dateistruktur und Funktionen

### Hauptdateien

#### `balance_control_c.c` (19KB, 491 Zeilen)
**Zweck**: Hauptsteuerung und zentrale Regelschleife

**Kernfunktionen**:
- **Webots-Integration**: Initialisierung aller Devices (IMU, Motoren, Display)
- **Hauptregelschleife**: 
  ```c
  while (wb_robot_step(timestep) != -1) {
      // 1. Roll-Winkel messen und filtern
      float true_roll_angle = get_filtered_roll_angle();
      
      // 2. Erweiterte Physik-Simulation
      float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
      
      // 3. PID-Regelung: Roll-Winkel → Lenkwinkel
      float steering_output = pid_compute(&angle_pid, 0.0, simulated_roll_angle, current_time);
      
      // 4. Geschwindigkeitsregelung basierend auf Stabilität
      float stability_factor = fabs(steering_output) / max_handlebar_angle;
      float target_speed = base_speed * (1.0 - stability_reduction * stability_factor);
      
      // 5. Motoren ansteuern
      wb_motor_set_position(handlebars_motor, steering_output);
      wb_motor_set_velocity(wheel_motor, target_speed);
  }
  ```

**Wichtige Funktionen**:
- `init_devices()`: Webots-Device-Initialisierung
- `get_filtered_roll_angle()`: IMU-Datenverarbeitung mit Glättungsfilter
- `load_and_apply_config()`: Dynamisches Nachladen der Konfiguration
- `handle_keyboard_input()`: Erweiterte Tastatursteuerung
- `update_display()`: Webots-Display-Updates
- `print_status()`: Debug-Ausgaben

**Erweiterte Features**:
- **Adaptive Konfiguration**: JSON-Config wird alle 100ms neu geladen
- **Physik-Integration**: Vollständige Integration der erweiterten Physik
- **Keyboard-Steuerung**: P (Physik-Debug), W (Wind-Simulation), E (Reset)
- **Multi-Level-Logging**: Status, Debug und Physik-Informationen

### PID-Regelung

#### `balance_pid.h` (2.5KB, 84 Zeilen)
**Zweck**: PID-Controller-Definitionen und API

**Datenstruktur**:
```c
typedef struct {
    // Historie für D-Term-Berechnung
    float derivative_history[HISTORY_LEN];
    float error_history[HISTORY_LEN];
    long long time_history[HISTORY_LEN];
    
    // PID-Terme
    float proportional_term;
    float integral_term;
    float derivative_term;
    
    // Parameter
    float Kp, Ki, Kd;
    
    // Begrenzungen (Anti-Windup)
    float output_min, output_max;
    float integral_min, integral_max;
} pid_controller_t;
```

**API-Funktionen**:
- `pid_init()`: PID-Controller-Initialisierung
- `pid_compute()`: Hauptberechnung mit Anti-Windup
- `pid_reset()`: Historie zurücksetzen
- `pid_update_parameters()`: Laufzeit-Parameter-Update

#### `balance_pid.c` (3.8KB, 126 Zeilen)
**Zweck**: PID-Controller-Implementierung

**Algorithmus**:
```c
float pid_compute(pid_controller_t *pid, float setpoint, float process_variable, long long current_time_us) {
    float error = setpoint - process_variable;
    
    // P-Term
    pid->proportional_term = pid->Kp * error;
    
    // I-Term mit Anti-Windup
    pid->integral_term += pid->Ki * error * dt;
    if (pid->integral_term > pid->integral_max) pid->integral_term = pid->integral_max;
    if (pid->integral_term < pid->integral_min) pid->integral_term = pid->integral_min;
    
    // D-Term mit Filterung über Historie
    pid->derivative_term = pid->Kd * derivative_filtered;
    
    float output = pid->proportional_term + pid->integral_term + pid->derivative_term;
    
    // Output-Begrenzung
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}
```

**Besonderheiten**:
- **Historie-basierter D-Term**: Filterung über 5 Samples zur Rauschunterdrückung
- **Anti-Windup-Schutz**: Begrenzte Integral-Terme
- **Mikrosekunden-Timing**: Präzise dt-Berechnung
- **Plausibilitätsprüfung**: Zeitsprung-Erkennung

### Konfigurationssystem

#### `balance_config.h` (3.0KB, 93 Zeilen)
**Zweck**: Konfigurationsstrukturen und API

**Strukturen**:
```c
// PID-Parameter
typedef struct {
    float Kp, Ki, Kd;                // PID-Parameter  
    float output_min, output_max;    // Ausgabebegrenzung
    float integral_min, integral_max; // Anti-Windup-Schutz
} balance_angle_pid_t;

// Geschwindigkeitsregelung
typedef struct {
    float base_speed;        // Basis-Geschwindigkeit (rad/s)
    float min_speed;         // Minimale Geschwindigkeit
    float max_speed;         // Maximale Geschwindigkeit  
    float stability_reduction; // Reduktionsfaktor bei Instabilität
} balance_speed_control_t;

// Mechanische Begrenzungen
typedef struct {
    float max_handlebar_angle; // Maximaler Lenkwinkel (rad)
    float max_roll_angle;      // Roll-Winkel-Limit für Plausibilität
} balance_mechanical_limits_t;

// System-Einstellungen
typedef struct {
    int enable_logging;        // Logging aktivieren
    int enable_preview;        // Display-Updates
    int config_reload_interval; // Config-Reload-Takt
    int filter_size;           // Roll-Winkel-Filter-Größe
} balance_system_t;

// Hauptkonfiguration
typedef struct {
    balance_angle_pid_t angle_pid;
    balance_speed_control_t speed_control;
    balance_mechanical_limits_t mechanical_limits;
    balance_system_t system;
} balance_config_t;
```

**API-Funktionen**:
- `balance_config_load()`: JSON-Datei → Struktur
- `balance_config_save()`: Struktur → JSON-Datei
- `balance_config_set_defaults()`: Bewährte Standard-Parameter
- `balance_config_validate()`: Plausibilitätsprüfung
- `balance_config_print()`: Debug-Ausgabe

#### `balance_config.c` (12KB, 255 Zeilen)
**Zweck**: JSON-basiertes Konfigurationssystem

**Features**:
- **JSON-Parser**: Robuste JSON-Verarbeitung mit Fehlerbehandlung
- **Standard-Parameter**: Autobike-bewährte Einstellungen
- **Validierung**: Umfassende Plausibilitätsprüfung
- **GUI-Integration**: Bidirektionale JSON-Kommunikation
- **Hot-Reload**: Laufzeit-Konfigurationsupdate

**Beispiel-Konfiguration**:
```json
{
  "angle_pid": {
    "Kp": 10.0,
    "Ki": 0.0, 
    "Kd": 2.2,
    "output_min": -0.32,
    "output_max": 0.32
  },
  "speed_control": {
    "base_speed": 5.0,
    "min_speed": 3.0,
    "max_speed": 8.0,
    "stability_reduction": 0.5
  }
}
```

### Logging-System

#### `balance_logging.h` (2.4KB, 77 Zeilen)
**Zweck**: Logging-API und Datenstrukturen

**Log-Datenstruktur**:
```c
typedef struct {
    double timestamp;           // Zeitstempel in Sekunden
    float roll_angle;          // Roll-Winkel in Grad
    float steering_output;     // Lenkwinkel-Ausgabe in Radiant
    float target_speed;        // Zielgeschwindigkeit in rad/s
    float p_term;              // P-Term des PID
    float i_term;              // I-Term des PID
    float d_term;              // D-Term des PID
    float error;               // Regelfehler
    float stability_factor;    // Stabilitätsfaktor (0.0-1.0)
} balance_log_data_t;
```

**Logger-Struktur**:
```c
typedef struct {
    FILE *csv_file;            // CSV-Datei-Handle
    char filename[256];        // Dateiname mit Zeitstempel
    int log_counter;           // Anzahl Logeinträge
    int is_initialized;        // Initialisierungsstatus
} balance_logger_t;
```

#### `balance_logging.c` (5.5KB, 180 Zeilen)
**Zweck**: CSV-basiertes Logging-System

**Funktionen**:
- `balance_logging_init()`: Logger-Initialisierung mit Zeitstempel-Datei
- `balance_logging_write()`: Thread-sicheres CSV-Schreiben
- `balance_logging_close()`: Sicheres Datei-Schließen
- `balance_logging_add_comment()`: Kommentare für Events
- `balance_logging_create_new_file()`: Neue Datei nach Crash

**CSV-Format**:
```csv
timestamp,roll_angle,steering_output,target_speed,p_term,i_term,d_term,error,stability_factor
0.000000,-1.234,0.042,5.000,0.123,-0.001,0.019,-1.234,0.131
```

**Features**:
- **Automatische Dateinamen**: `balance_log_YYYYMMDD_HHMMSS.csv`
- **Puffer-Management**: Regelmäßiges Flush für Datensicherheit
- **Crash-Recovery**: Neue Datei bei Neustart
- **Performance**: Minimaler Overhead in Regelschleife

### Erweiterte Physik-Simulation

#### `bicycle_physics.h` (6.9KB, 210 Zeilen)
**Zweck**: Erweiterte Fahrradphysik-API

**Physik-Zustand**:
```c
typedef struct {
    float velocity[3];                  // Geschwindigkeit [x,y,z] m/s
    float angular_velocity[3];          // Winkelgeschwindigkeit [x,y,z] rad/s
    float roll_angle;                   // Roll-Winkel rad
    float slip_angle;                   // Slip-Winkel rad
    float wheel_angular_velocity;       // Radwinkelgeschwindigkeit rad/s
} bicycle_state_t;
```

**Kraftberechnung**:
```c
typedef struct {
    float drag_force[3];                // Luftwiderstandskraft [x,y,z] N
    float lateral_force[3];             // Laterale Reifenkraft [x,y,z] N
    float rolling_resistance_torque;    // Rollwiderstandsmoment Nm
    float gyroscopic_torque[3];         // Gyroskopmommente [x,y,z] Nm
    float crosswind_force[3];           // Seitenwindkraft [x,y,z] N
} bicycle_forces_t;
```

**Sensor-Simulation**:
```c
typedef struct {
    float imu_buffer[IMU_DELAY_SAMPLES]; // Verzögerungspuffer
    int buffer_index;                    // Pufferindex
    bool buffer_filled;                  // Vollständig gefüllt?
    float last_values[3];                // Sprungdetektion
} sensor_simulation_t;
```

#### `bicycle_physics.c` (21KB, 537 Zeilen)
**Zweck**: Vollständige Physik-Implementierung

**Implementierte Physik-Effekte**:

1. **Laterale Reifenkräfte** (Pacejka-Modell):
   ```c
   float bicycle_physics_tire_lateral_force(float slip_angle, float normal_force) {
       float B = 10.0f;  // Steifigkeitsfaktor
       float C = 1.3f;   // Formfaktor  
       float D = TIRE_FRICTION_COEFF * normal_force;  // Spitzenkraft
       
       // Für kleine Winkel: lineare Näherung
       if (fabsf(slip_angle) < 0.1f) {
           return TIRE_STIFFNESS * slip_angle;
       }
       // Vollständiges Pacejka-Modell
       return D * sinf(C * atanf(B * slip_angle));
   }
   ```

2. **Aerodynamischer Widerstand**:
   ```c
   void bicycle_physics_aerodynamic_drag(const float velocity[3], float drag_force[3]) {
       float speed = sqrtf(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
       float drag_magnitude = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA * speed * speed;
       
       if (speed > 0.01f) {
           drag_force[0] = -drag_magnitude * velocity[0] / speed;
           drag_force[1] = -drag_magnitude * velocity[1] / speed;
       }
   }
   ```

3. **Rollwiderstand**:
   ```c
   float bicycle_physics_rolling_resistance(float wheel_velocity, float normal_force) {
       return ROLLING_RESISTANCE_COEFF * normal_force * WHEEL_RADIUS * 
              (wheel_velocity > 0 ? 1.0f : -1.0f);
   }
   ```

4. **Gyroskopmommente**:
   ```c
   void bicycle_physics_gyroscopic_effects(float wheel_angular_vel, const float frame_angular_vel[3], float gyro_torque[3]) {
       float wheel_inertia = 0.002f;  // kg⋅m²
       gyro_torque[0] = -wheel_inertia * wheel_angular_vel * frame_angular_vel[2]; // Roll-Stabilisierung
       gyro_torque[1] = wheel_inertia * wheel_angular_vel * frame_angular_vel[0];  // Pitch-Moment
   }
   ```

5. **IMU-Sensorsimulation**:
   ```c
   float bicycle_physics_simulate_imu(bicycle_physics_t *physics, float true_roll) {
       // Verzögerung über FIFO-Puffer
       physics->sensor_sim.imu_buffer[physics->sensor_sim.buffer_index] = true_roll;
       int delayed_index = (physics->sensor_sim.buffer_index + 1) % IMU_DELAY_SAMPLES;
       float delayed_roll = physics->sensor_sim.imu_buffer[delayed_index];
       
       // Gausssches Rauschen hinzufügen
       float noise = gaussian_noise(IMU_NOISE_SIGMA);
       return delayed_roll + noise;
   }
   ```

**Umwelt-Simulation**:
- **Seitenwind**: Variable Geschwindigkeit und Richtung
- **Turbulenz**: Stochastische Störungen
- **Straßenneigung**: Gravitationelle Effekte
- **Interaktive Steuerung**: Tastatur-gesteuerte Parameter

### Build-System

#### `Makefile` (2.0KB, 71 Zeilen)
**Zweck**: Webots-kompatibles Build-System

**Struktur**:
```makefile
# Webots-spezifische Konfiguration (macOS)
WEBOTS_HOME ?= /Applications/Webots.app/Contents
WEBOTS_INCLUDE = -I$(WEBOTS_HOME_PATH)/include/controller/c
WEBOTS_LIB = -L$(WEBOTS_HOME_PATH)/lib/controller -lController

# Compiler-Einstellungen
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g
LDFLAGS = -lm

# Quell- und Zieldateien
SOURCES = balance_control_c.c balance_pid.c balance_config.c balance_logging.c bicycle_physics.c
OBJECTS = $(SOURCES:.c=.o)
TARGET = balance_control_c
```

**Build-Targets**:
- `make all`: Standard-Build mit Optimierung
- `make debug`: Debug-Version mit Symbolen
- `make release`: Release-Version mit maximaler Optimierung
- `make clean`: Bereinigung aller generierten Dateien
- `make help`: Hilfe-Ausgabe

**Abhängigkeiten**:
```makefile
balance_control_c.o: balance_control_c.c balance_pid.h balance_config.h balance_logging.h bicycle_physics.h
balance_pid.o: balance_pid.c balance_pid.h
balance_config.o: balance_config.c balance_config.h
balance_logging.o: balance_logging.c balance_logging.h
bicycle_physics.o: bicycle_physics.c bicycle_physics.h
```

## Regelungsalgorithmus im Detail

### Roll-Winkel-Berechnung

```c
static float get_filtered_roll_angle(void) {
    // IMU-Quaternion auslesen
    const double *quaternion = wb_inertial_unit_get_quaternion(imu_sensor);
    double w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];
    
    // Quaternion → Roll-Winkel
    float roll_rad = atan2(2*(w*x + y*z), w*w - x*x - y*y + z*z);
    float roll_deg = roll_rad * 180.0f / M_PI;
    
    // Plausibilitätsprüfung
    if (fabs(roll_deg) > config.mechanical_limits.max_roll_angle) {
        printf("WARNUNG: Unplausibler Roll-Winkel: %.1f°\n", roll_deg);
        return roll_angle_history[(roll_history_index + ROLL_FILTER_SIZE - 1) % ROLL_FILTER_SIZE];
    }
    
    // Gleitender Mittelwert-Filter
    roll_angle_history[roll_history_index] = roll_deg;
    roll_history_index = (roll_history_index + 1) % ROLL_FILTER_SIZE;
    
    float filtered_roll = 0.0f;
    int count = roll_history_filled ? ROLL_FILTER_SIZE : roll_history_index;
    for (int i = 0; i < count; i++) {
        filtered_roll += roll_angle_history[i];
    }
    
    return filtered_roll / count;
}
```

### Adaptive Geschwindigkeitsregelung

```c
// Stabilität basierend auf Lenkwinkel bewerten
float stability_factor = fabs(steering_output) / config.mechanical_limits.max_handlebar_angle;

// Geschwindigkeit reduzieren bei Instabilität
float speed_reduction = stability_factor * config.speed_control.stability_reduction;
float target_speed = config.speed_control.base_speed * (1.0f - speed_reduction);

// Begrenzungen anwenden
if (target_speed < config.speed_control.min_speed) 
    target_speed = config.speed_control.min_speed;
else if (target_speed > config.speed_control.max_speed)
    target_speed = config.speed_control.max_speed;

// Rollwiderstand berücksichtigen
float roll_resistance_torque = bicycle_physics.forces.rolling_resistance_torque;
if (fabs(roll_resistance_torque) > 0.001f) {
    float resistance_speed_reduction = fabs(roll_resistance_torque) * 0.1f;
    target_speed = target_speed * (1.0f - resistance_speed_reduction);
    if (target_speed < 0.1f) target_speed = 0.1f;
}
```

## Monitoring und Debugging

### Physik-Debug-Ausgabe

```c
// Physik-Debug alle 5 Sekunden
if (++physics_debug_counter >= (5000 / timestep)) {
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
```

### Tastatur-Steuerung

```c
static void handle_keyboard_input(void) {
    int key = wb_keyboard_get_key();
    
    switch (key) {
        case WB_KEYBOARD_ESCAPE:
            printf("ESC gedrückt - Controller wird beendet\n");
            wb_robot_cleanup();
            exit(0);
            break;
            
        case 'R':
        case 'r':
            printf("Konfiguration wird neu geladen...\n");
            load_and_apply_config();
            break;
            
        case 'P':
        case 'p':
            printf("=== PHYSIK-DEBUG ===\n");
            bicycle_physics_debug_print(&bicycle_physics);
            break;
            
        case 'W':
        case 'w':
            // Wind-Simulation umschalten: 0 → 2 → 5 → 10 m/s
            static float wind_levels[] = {0.0f, 2.0f, 5.0f, 10.0f};
            static int wind_index = 0;
            wind_index = (wind_index + 1) % 4;
            bicycle_physics_set_environment(&bicycle_physics, wind_levels[wind_index], M_PI/2, 0.3f);
            printf("Seitenwind: %.1f m/s\n", wind_levels[wind_index]);
            break;
            
        case 'E':
        case 'e':
            // Umgebung zurücksetzen
            bicycle_physics_set_environment(&bicycle_physics, 0.0f, 0.0f, 0.0f);
            printf("Umgebungsparameter zurückgesetzt\n");
            break;
    }
}
```

## Installation und Verwendung

### Build-Prozess

```bash
# Ins Verzeichnis wechseln
cd Regelstuerung/controllers/balance_control_c/

# Kompilieren
make clean
make

# Debug-Version (mit Symbolen)
make debug

# Release-Version (optimiert)
make release
```

### Konfiguration

```bash
# Standard-Konfiguration erstellen
./balance_control_c --create-config

# Mit GUI konfigurieren
cd ../../GUI/
python balance_controller_gui.py
```

### Webots-Integration

1. **Welt laden**: `Regelstuerung/worlds/Little Bicycle V2.wbt`
2. **Controller auswählen**: `balance_control_c`
3. **Simulation starten**: Play-Button oder Strg+0

### Parameter-Tuning

**PID-Parameter (Autobike-bewährt)**:
```json
{
  "angle_pid": {
    "Kp": 10.0,    // Hauptregelparameter
    "Ki": 0.0,     // Meist deaktiviert
    "Kd": 2.2      // Dämpfung
  }
}
```

**Systematisches Tuning**:
1. **Nur P-Regler**: Kp von 1.0 bis 15.0 erhöhen
2. **D-Anteil hinzufügen**: Kd für Dämpfung (1.0-5.0)
3. **I-Anteil vorsichtig**: Nur bei bleibender Abweichung (0.1-1.0)

## Technische Besonderheiten

### Performance-Optimierungen

- **Effizienter PID**: Nur notwendige Berechnungen pro Zyklus
- **Smart Filtering**: Adaptive Filter-Größe basierend auf Stabilität
- **Lazy Loading**: JSON-Config nur bei Änderungen neu laden
- **Buffered Logging**: Minimaler I/O-Overhead

### Robustheit-Features

- **Plausibilitätsprüfung**: Sensor-Werte werden validiert
- **Graceful Degradation**: System läuft auch bei Teilausfällen
- **Recovery-Mechanismen**: Automatische Wiederherstellung nach Fehlern
- **Bounds-Checking**: Alle Parameter werden begrenzt

### Realismus-Verbesserungen

- **Physik-Simulation**: 5 zusätzliche Physik-Effekte
- **Sensor-Realismus**: IMU-Rauschen und Verzögerung
- **Umwelt-Einflüsse**: Wind, Neigung, Turbulenz
- **Dynamische Effekte**: Geschwindigkeitsabhängige Kräfte

## Troubleshooting

### Häufige Probleme

| Problem | Symptom | Lösung |
|---------|---------|---------|
| Fahrrad fällt sofort | Roll-Winkel > 45° | Kp erhöhen (5-15) |
| Oszillation | Hin- und Herpendeln | Kp reduzieren, Kd erhöhen |
| Träge Reaktion | Langsame Aufrichtung | Kd reduzieren |
| JSON-Fehler | Config nicht geladen | Syntax prüfen, Pfad validieren |
| Build-Fehler | Compilation failed | Webots-Pfad, GCC-Version prüfen |

### Debug-Strategien

1. **Status-Ausgabe**: Taste 'S' für aktuellen Zustand
2. **Physik-Debug**: Taste 'P' für Kraftanalyse  
3. **Live-Monitoring**: GUI für Echtzeit-Plots
4. **CSV-Analyse**: Log-Dateien in Excel/Python auswerten
5. **Parameter-Sweep**: Systematisches Testen verschiedener Werte

## Lizenz und Referenzen

**Basiert auf**:
- Autobike-Implementierung von Jonah Zander (BachelorArbeit 2023)
- Webots Controller C API
- Erweiterte Physik-Modelle (Pacejka, Aerodynamik)

**Lizenz**: Open Source (Apache 2.0)
**Entwickelt für**: Masterarbeit - Selbstbalancierendes Fahrrad 