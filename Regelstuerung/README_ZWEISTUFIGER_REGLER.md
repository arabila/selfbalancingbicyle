# Zweistufiger Regler - Selbstbalancierendes Fahrrad

## Übersicht

Das selbstbalancierende Fahrrad verwendet eine **zweistufige Reglerarchitektur** bestehend aus:

1. **Balance Controller (C)** - Ultraschnelle Roll-Winkel-Stabilisierung (500 Hz)
2. **Vision Controller (Python)** - Langsamere pfadbasierte Lenkung (20 Hz)

Diese Architektur ermöglicht es, sowohl die kritische Balance-Regelung in Echtzeit zu gewährleisten als auch komplexe Vision-basierte Pfadplanung durchzuführen.

## Architektur-Übersicht

```
┌─────────────────────────────────────────────────────────────────┐
│                    FAHRRAD-SYSTEM                                │
├─────────────────────────────────────────────────────────────────┤
│  IMU Sensor → Roll-Winkel                                       │
│  Kamera → Straßenbild                                           │
│  Motoren ← Lenkung & Geschwindigkeit                            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                 ZWEISTUFIGER REGLER                              │
├─────────────────┬───────────────────────────────────────────────┤
│  BALANCE        │  VISION CONTROLLER (Python)                   │
│  CONTROLLER     │  - YOLO Straßenerkennung                      │
│  (C)            │  - Pfadplanung                                │
│  - PID Balance  │  - Vision-PID                                 │
│  - Motor Control│  - Laufzeit: 50ms (20 Hz)                    │
│  - IPC Handler  │                                               │
│  - Laufzeit: 2ms├──────────────────────────────────────────────┤
│    (500 Hz)     │              IPC KOMMUNIKATION                │
│                 │  Vision Commands ←→ Balance Status             │
└─────────────────┴───────────────────────────────────────────────┘
```

## 1. Balance Controller (C) - `balance_control_c.c`

### 1.1 Hauptfunktionen

Der Balance Controller ist das **Herzstück** des Systems und läuft mit **2ms Timestep (500 Hz)** für maximale Reaktionsgeschwindigkeit.

#### Kernaufgaben:
- **Roll-Winkel-Messung** über IMU-Sensor
- **PID-basierte Balance-Regelung**
- **Motorsteuerung** (Lenkung + Antrieb)
- **IPC-Kommunikation** mit Vision Controller
- **Erweiterte Fahrradphysik-Simulation**

### 1.2 Detaillierter Ablauf (alle 2ms)

#### Schritt 1: Sensordatenerfassung
```c
// 1. Roll-Winkel von IMU-Sensor lesen
float true_roll_angle = get_filtered_roll_angle();

// 2. Erweiterte Physik-Simulation anwenden
float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
float roll_angle = simulated_roll_angle;
```

**Was passiert hier:**
- IMU-Sensor liefert Quaternion-Daten
- Umwandlung zu Roll-Winkel in Grad
- Korrektur für 180°-Fahrrad-Rotation
- Gleitender Durchschnitt über mehrere Messwerte
- Physik-Simulation fügt realistische Störungen hinzu (Wind, Rollwiderstand)

#### Schritt 2: Balance-PID-Regelung
```c
// 3. PID-Regelung: Roll-Winkel → Lenkwinkel
long long current_time = get_time_microseconds();
float steering_output = -pid_compute(&angle_pid, 0.0, roll_angle, current_time);

// 4. Lenkwinkel begrenzen
if (steering_output > config.mechanical_limits.max_handlebar_angle) 
    steering_output = config.mechanical_limits.max_handlebar_angle;
```

**Was passiert hier:**
- **Sollwert:** 0° (aufrecht)
- **Istwert:** Aktueller Roll-Winkel
- **PID-Ausgabe:** Lenkwinkel in Radiant
- **Begrenzung:** Mechanische Limits des Lenkers

#### Schritt 3: Vision-Commands empfangen
```c
// 5. Vision-Commands empfangen und verarbeiten
vision_command_t vision_cmd;
int vision_cmd_received = receive_vision_command(&vision_cmd);
```

**Vision-Command-Struktur:**
```c
typedef struct {
    float steer_command;     // Lenkbefehl (-1.0 bis +1.0)
    float speed_command;     // Geschwindigkeitsbefehl (0.0 bis 1.0)
    int valid;               // Kommando gültig?
    float vision_error;      // Vision-Fehler
    float vision_p_term;     // P-Term des Vision-PID
    float vision_i_term;     // I-Term des Vision-PID
    float vision_d_term;     // D-Term des Vision-PID
    float mask_coverage;     // Straßenerkennung in %
} vision_command_t;
```

#### Schritt 4: Zwei-Ebenen-Regelung
```c
// 6. Zwei-Ebenen-Regelung: Vision + Balance
float final_steer = steering_output;  // Balance-PID-Ausgang
float target_speed = config.speed_control.base_speed;

if (vision_active && last_vision_command.valid) {
    // Vision-Lenkung berechnen
    float vision_steer = vision_to_use->steer_command * config.mechanical_limits.max_handlebar_angle;
    
    // Gewichtung basierend auf Maskenabdeckung
    float weight = 0.6f * (vision_to_use->mask_coverage / 100.0f);
    
    // Kombination aus Vision und Balance
    final_steer = weight * vision_steer + (1.0f - weight) * steering_output;
}
```

**Regelungs-Logik:**
1. **Balance hat immer Priorität** - Grundstabilität wird immer gewährleistet
2. **Vision überlagert Balance** - Je besser die Straßenerkennung, desto mehr Vision-Einfluss
3. **Gewichtung durch Mask-Coverage** - Gute Erkennung = mehr Vision-Anteil
4. **Automatischer Fallback** - Bei Vision-Ausfall nur Balance-Regelung

#### Schritt 5: Motoransteuerung
```c
// 7. Motoren ansteuern
wb_motor_set_position(handlebars_motor, final_steer);    // Lenkung
wb_motor_set_velocity(wheel_motor, target_speed);        // Geschwindigkeit
```

#### Schritt 6: Status-Rückmeldung
```c
// 8. Balance-Status an Vision-Controller senden
balance_status_t status = {
    .roll_angle = roll_angle,
    .steering_output = final_steer,
    .current_speed = target_speed,
    .stability_factor = fabs(steering_output) / config.mechanical_limits.max_handlebar_angle
};
send_balance_status(&status);
```

### 1.3 Konfiguration

Der Balance Controller lädt seine Parameter aus `../../GUI/balance_config.json`:

```json
{
  "angle_pid": {
    "Kp": 15.0,    // Proportional-Anteil
    "Ki": 0.5,     // Integral-Anteil  
    "Kd": 2.0      // Differential-Anteil
  },
  "speed_control": {
    "base_speed": 5.0,
    "min_speed": 2.0,
    "max_speed": 8.0
  },
  "mechanical_limits": {
    "max_handlebar_angle": 0.4  // ~23° max Lenkwinkel
  }
}
```

---

## 2. Vision Controller (Python) - `vision_control_py.py`

### 2.1 Hauptfunktionen

Der Vision Controller läuft mit **50ms Timestep (20 Hz)** und fokussiert sich auf **Pfadplanung und Straßenerkennung**.

#### Kernaufgaben:
- **YOLO-basierte Straßensegmentierung**
- **Vision-PID für Pfadverfolgung**
- **Geschwindigkeitsadaption**
- **IPC-Kommunikation** mit Balance Controller

### 2.2 Detaillierter Ablauf (alle 50ms)

#### Schritt 1: Kamerabild erfassen
```python
# Kamerabild holen
img_bytes = self.camera.getImage()
frame = np.frombuffer(img_bytes, dtype=np.uint8).reshape((height, width, 4))
frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
```

#### Schritt 2: YOLO-Straßenerkennung
```python
def get_vision_error_yolo(self, frame):
    # YOLO-Vorhersage
    results = self.yolo_model.predict(
        source=frame,
        conf=0.5,
        max_det=5,
        show=False,
        verbose=False
    )
    
    # Suche nach Straßen-Klasse (ID 2 = 'street_main')
    if r.boxes is not None and len(r.boxes.cls) > 0:
        street_indices = [idx for idx, cls in enumerate(r.boxes.cls) if cls == 2]
        
        if street_indices:
            # Berechne Mittelpunkt der Straßen-Bounding-Boxes
            x_centers = []
            for idx in street_indices:
                xyxy = r.boxes.xyxy[idx]
                x_center = float((xyxy[0] + xyxy[2]) / 2.0)
                x_centers.append(x_center)
            
            avg_x_center = sum(x_centers) / len(x_centers)
            frame_center = frame.shape[1] / 2
            error = (frame_center - avg_x_center) / frame.shape[1]  # Normiert
```

**YOLO-Pipeline:**
1. **Frame an YOLO** - Aktuelle Kamerabild
2. **Objekterkennung** - Erkennung verschiedener Straßenklassen
3. **Segmentierung** - Pixel-genaue Straßenmaske
4. **Mittelpunkt-Berechnung** - Schwerpunkt der erkannten Straße
5. **Error-Berechnung** - Abweichung zur Bildmitte (normiert auf -1 bis +1)

#### Schritt 3: Intelligentes Fallback-System
```python
def _use_last_valid_values(self):
    """Verwendet die letzten gültigen Werte mit langsamem Ausblenden"""
    self.no_detection_counter += 1
    
    # Wenn wir zu lange keine Erkennung hatten, blende langsam aus
    if self.no_detection_counter > self.max_no_detection_steps:
        decay_factor = self.error_decay_factor ** (self.no_detection_counter - self.max_no_detection_steps)
        error = self.last_valid_error * decay_factor
```

**Fallback-Logik:**
- **Letzte gültige Werte speichern** - Bei erfolgreicher Erkennung
- **Graduelles Ausblenden** - Langsame Reduzierung bei fehlender Erkennung
- **Sanfter Übergang** - Verhindert abrupte Sprünge
- **Kompletter Fallback** - Nach zu langer Zeit ohne Erkennung

#### Schritt 4: Vision-PID-Regelung
```python
def vision_pid_control(self, error, dt):
    # P-Term
    p_term = self.vision_kp * error
    
    # I-Term mit Anti-Windup
    self.vision_integral += error * dt
    integral_limit = 0.5
    self.vision_integral = max(-integral_limit, min(integral_limit, self.vision_integral))
    i_term = self.vision_ki * self.vision_integral
    
    # D-Term
    d_error = (error - self.vision_last_error) / dt if dt > 0 else 0.0
    d_term = self.vision_kd * d_error
    
    # Gesamtausgang
    output = p_term + i_term + d_term
    output = max(-self.max_steer, min(self.max_steer, output))
```

**PID-Parameter:**
- **Kp = 20:** Starke Reaktion auf Abweichungen
- **Ki = 0:** Kein Integral-Anteil (vermeidet Überschwingen)
- **Kd = 0:** Kein Differential-Anteil (vermeidet Rauschen)

#### Schritt 5: Geschwindigkeitsadaption
```python
# Geschwindigkeitsanpassung basierend auf Fehler
speed_reduction = min(abs(error) * 2.0, 0.4)  # Max 40% Reduktion
speed_cmd = self.base_speed - speed_reduction
speed_cmd = max(self.min_speed, min(self.max_speed, speed_cmd))
```

**Geschwindigkeits-Logik:**
- **Basis-Geschwindigkeit:** 0.5 (50% von Maximum)
- **Error-basierte Reduktion:** Je größer der Fehler, desto langsamer
- **Maximal 40% Reduktion** - Verhindert zu langsame Fahrt
- **Min/Max Limits** - Sicherheitsgrenzen

#### Schritt 6: Command-Übertragung
```python
def send_vision_command(self, steer_cmd, speed_cmd):
    command_data = struct.pack('ffifffff', 
                             steer_cmd, speed_cmd, 1,
                             self.vision_error, self.vision_p_term, 
                             self.vision_i_term, self.vision_d_term, 
                             self.vision_mask_coverage)
    
    self.command_emitter.send(command_data)
```

---

## 3. IPC-Kommunikation

### 3.1 Datenstrukturen

#### Vision → Balance (Command)
```c
typedef struct {
    float steer_command;     // -1.0 bis +1.0
    float speed_command;     // 0.0 bis 1.0  
    int valid;               // 1 = gültig
    float vision_error;      // Erkannter Pfadfehler
    float vision_p_term;     // PID P-Term
    float vision_i_term;     // PID I-Term
    float vision_d_term;     // PID D-Term
    float mask_coverage;     // Straßenerkennung in %
} vision_command_t;
```

#### Balance → Vision (Status)  
```c
typedef struct {
    float roll_angle;        // Aktueller Roll-Winkel (rad)
    float steering_output;   // Aktueller Lenkwinkel (rad)
    float current_speed;     // Aktuelle Geschwindigkeit (rad/s)
    float stability_factor;  // Stabilitätsfaktor (0.0-1.0)
} balance_status_t;
```

### 3.2 Kommunikationsfrequenz

- **Vision → Balance:** 20 Hz (alle 50ms)
- **Balance → Vision:** 20 Hz (gesendet alle 25 * 2ms = 50ms)
- **Balance interne Regelung:** 500 Hz (alle 2ms)

---

## 4. Gesamter Regelkreis - Schritt für Schritt

### 4.1 Initialisierung (Einmalig)

1. **Balance Controller startet** (C-Programm)
   - Lädt Konfiguration aus JSON
   - Initialisiert IMU, Motoren, IPC
   - Startet 500Hz Hauptschleife

2. **Vision Controller startet** (Python-Programm)  
   - Lädt YOLO-Modell
   - Initialisiert Kamera, IPC
   - Startet 20Hz Hauptschleife

### 4.2 Laufzeit-Regelkreis

#### Jeder Balance-Zyklus (2ms):
```
┌─ Balance Controller (C) ─────────────────────────────────────┐
│ 1. IMU → Roll-Winkel messen                                │
│ 2. Physik-Simulation anwenden                              │
│ 3. Balance-PID berechnen                                   │
│ 4. Vision-Command empfangen (falls verfügbar)              │
│ 5. Balance + Vision kombinieren                            │
│ 6. Motoren ansteuern                                       │
│ 7. Status senden (alle 25 Zyklen)                         │
└─────────────────────────────────────────────────────────────┘
```

#### Jeder Vision-Zyklus (50ms):
```  
┌─ Vision Controller (Python) ────────────────────────────────┐
│ 1. Kamerabild erfassen                                     │
│ 2. YOLO Straßenerkennung                                   │
│ 3. Vision-Error berechnen                                  │
│ 4. Vision-PID berechnen                                    │
│ 5. Geschwindigkeit adaptieren                              │
│ 6. Command an Balance senden                               │
│ 7. Balance-Status empfangen                                │
└─────────────────────────────────────────────────────────────┘
```

### 4.3 Zeitlicher Ablauf (Beispiel)

```
Zeit    Balance Controller (2ms)           Vision Controller (50ms)
────    ────────────────────────          ────────────────────────
0ms     Zyklus 1: IMU→PID→Motor           
2ms     Zyklus 2: IMU→PID→Motor
4ms     Zyklus 3: IMU→PID→Motor
...
48ms    Zyklus 25: IMU→PID→Motor→Status
50ms    Zyklus 26: Vision-CMD→PID→Motor    Zyklus 1: Kamera→YOLO→CMD
52ms    Zyklus 27: IMU→PID→Motor
...
100ms   Zyklus 51: Vision-CMD→PID→Motor    Zyklus 2: Kamera→YOLO→CMD
```

---

## 5. Sicherheits- und Robustheitsmechanismen

### 5.1 Balance Controller

1. **IMU-Ausfallsicherheit:**
   - Plausibilitätsprüfung der Roll-Winkel
   - Begrenzung maximaler Änderungen pro Zeitschritt
   - Gleitender Durchschnitt zur Rauschfilterung

2. **Vision-Timeout-Behandlung:**
   - Automatischer Fallback bei Vision-Ausfall
   - Weiterhin stabile Balance-Regelung
   - Geschwindigkeitsreduktion bei Instabilität

3. **Motor-Sicherheit:**
   - Mechanische Lenkwinkel-Begrenzung
   - Geschwindigkeits-Min/Max-Limits
   - Sanfte Übergänge zwischen Zuständen

### 5.2 Vision Controller

1. **YOLO-Ausfallsicherheit:**
   - Fallback auf einfache Kantenerkennung
   - Nutzung letzter gültiger Erkennungen
   - Graduelles Ausblenden bei längeren Ausfällen

2. **Erkennungsvalidierung:**
   - Plausibilitätsprüfung der YOLO-Ergebnisse
   - Speicherung letzter gültiger Werte
   - Mask-Coverage als Qualitätsmetrik

3. **PID-Stabilität:**
   - Anti-Windup für Integral-Term
   - Ausgangsbegrenzung
   - Sanfte Parameter-Übergänge

---

## 6. Konfiguration und Tuning

### 6.1 Balance-PID-Parameter (`balance_config.json`)

```json
{
  "angle_pid": {
    "Kp": 15.0,    // ↑ = schnellere Reaktion, aber Überschwingen
    "Ki": 0.5,     // ↑ = eliminiert Offset, aber Instabilität  
    "Kd": 2.0      // ↑ = dämpft Schwingungen, aber Rauschen
  }
}
```

**Tuning-Tipps:**
- **Kp zu hoch:** Überschwingen, Instabilität
- **Kp zu niedrig:** Langsame Reaktion, Offset
- **Ki zu hoch:** Oszillationen, Überschwingen
- **Kd zu hoch:** Rauschen-Verstärkung

### 6.2 Vision-Parameter (Code)

```python
# PID-Parameter für Vision
self.vision_kp = 20   # Lenkstärke
self.vision_ki = 0    # Meist 0 (vermeidet Überschwingen)
self.vision_kd = 0    # Meist 0 (vermeidet Rauschen)

# Geschwindigkeitsparameter  
self.base_speed = 0.5     # Grundgeschwindigkeit
self.min_speed = 0.3      # Minimum bei großen Fehlern
self.max_speed = 0.6      # Maximum bei perfekter Spur
```

---

## 7. Debugging und Monitoring

### 7.1 Console-Output

**Balance Controller:**
```
Roll: -2.34° | Steering: 12.1° | Speed: 4.2 km/h | P: 1.23 I: 0.45 D: 0.67
VISION: VisionSteer=0.123, Balance=0.089 → Final=0.106 (weight=0.45, coverage=67.3%)
```

**Vision Controller:**
```
VISION: Error=-0.234 | Steer=-0.123 | Speed=0.45 | PID=[P: 4.68 I: 0.00 D: 0.00] | Balance: Roll=-2.3° Stab=0.23
```

### 7.2 Keyboard-Shortcuts

**Balance Controller:**
- `ESC` - Beenden
- `R` - Konfiguration neu laden  
- `S` - Status ausgeben
- `P` - Physik-Debug
- `W` - Wind-Simulation umschalten
- `E` - Umgebung zurücksetzen

**Vision Controller:**
- `ESC` - Beenden
- `V` - Vision an/aus
- `R` - Vision-PID zurücksetzen

### 7.3 Log-Dateien

Erweiterte Logging-Daten werden gespeichert in:
- `Monitoring/balance_log_YYYYMMDD_HHMMSS.csv`

**Logged werden:**
- Balance-Controller: Roll-Winkel, PID-Terme, Motorwerte
- Vision-Controller: Error, PID-Terme, Mask-Coverage
- Physik-Simulation: Kräfte, Geschwindigkeiten
- Timing: Zykluszeiten, IPC-Latenz

---

## 8. Erweiterte Features

### 8.1 Physik-Simulation

Der Balance Controller enthält eine erweiterte Physik-Simulation:

```c
// Umgebungsparameter setzen
bicycle_physics_set_environment(&bicycle_physics, 2.0f, 0.5f, 0.2f); 
// 2 m/s Seitenwind, 20% Turbulenz

// Physik-Step durchführen  
float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
```

**Simulierte Effekte:**
- **Seitenwind** - Störkräfte seitlich
- **Luftwiderstand** - Geschwindigkeitsabhängig
- **Rollwiderstand** - Reibungsverluste
- **Gyroskopische Effekte** - Rad-Stabilisierung
- **Sensorrauschen** - Realistische IMU-Fehler

### 8.2 Dual-Kamera-System

Das System nutzt zwei Kameras:

1. **Fahrrad-Kamera** - Am Fahrrad montiert, bewegt sich mit
2. **Vision Controller Kamera** - Folgt dem Fahrrad, bessere Übersicht

```python
def _update_camera_position(self):
    # Vision Controller Kamera-Position an Fahrrad anpassen
    bike_pos = self.bicycle.getPosition()
    bike_rotation = self.bicycle.getField('rotation').getSFRotation()
    
    self.camera_transform_node.getField('translation').setSFVec3f(bike_pos)
    self.camera_transform_node.getField('rotation').setSFRotation(bike_rotation)
```

---

## 9. Troubleshooting

### 9.1 Häufige Probleme

**Balance Controller startet nicht:**
- Prüfe Webots-Devices: IMU, Motoren, IPC-Geräte
- Prüfe `balance_config.json` Syntax
- Prüfe Supervisor-Modus in Webots

**Vision Controller findet keine Straße:**
- Prüfe YOLO-Modell Pfad
- Prüfe Kamera-Bildqualität
- Prüfe YOLO-Klassen (ID 2 = 'street_main')

**IPC-Kommunikation funktioniert nicht:**
- Prüfe Device-Namen in .wbt-Datei
- Prüfe Struct-Größen in beiden Controllern
- Prüfe Webots-Version Kompatibilität

**Fahrrad oszilliert:**
- Balance-PID Kp reduzieren
- Vision-PID Kp reduzieren  
- Geschwindigkeit reduzieren

### 9.2 Performance-Optimierung

**Balance Controller:**
- IMU-Sampling-Rate anpassen
- Filter-Größe optimieren
- IPC-Frequenz reduzieren

**Vision Controller:**
- YOLO-Confidence-Threshold erhöhen
- Kamera-Auflösung reduzieren
- Vision-Frequenz reduzieren

---

## 10. Zusammenfassung

Die **zweistufige Reglerarchitektur** bietet optimale Balance zwischen:

✅ **Stabilität** - 500Hz Balance-Regelung garantiert Aufrechterhaltung  
✅ **Intelligenz** - YOLO-basierte Pfadfindung für autonome Navigation  
✅ **Robustheit** - Mehrfache Fallback-Mechanismen  
✅ **Performance** - Optimierte Lastverteilung zwischen C und Python  
✅ **Erweiterbarkeit** - Modulare Architektur für zusätzliche Features  

Das System demonstriert erfolgreich, wie komplexe Regelungsaufgaben durch intelligente Architektur-Entscheidungen lösbar sind.