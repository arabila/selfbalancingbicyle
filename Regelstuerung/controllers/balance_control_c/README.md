# Balance Control C - Selbstbalancierender Fahrrad-Controller

## Übersicht

Dieser C-Controller implementiert eine Roll-Winkel basierte PID-Regelung für ein selbstbalancierendes Fahrrad in Webots. Die Implementierung basiert auf der bewährten autobike-Architektur von Jonah Zander (BachelorArbeit 2023) und fokussiert sich ausschließlich auf die Balancierung.

## Features

- **Roll-Winkel basierte Regelung**: Direkte Umsetzung der autobike PID-Parameter
- **GUI-Integration**: Laufzeit-Konfiguration über JSON-Dateien
- **Echtzeit-Monitoring**: CSV-Logging für Analyse und Tuning
- **Adaptive Geschwindigkeit**: Automatische Geschwindigkeitsanpassung basierend auf Stabilität
- **Robuste Implementierung**: Umfangreiche Fehlerbehandlung und Validierung

## Architektur

```
[IMU Sensor] → [Roll-Winkel-Filter] → [Angle PID] → [Handlebars Motor]
                                           ↓
[JSON Config] ← [GUI] → [Live Monitoring] → [CSV Logger]
```

### Regelschleife

1. **IMU-Datenerfassung**: Quaternion → Roll-Winkel (gefiltert)
2. **PID-Regelung**: Roll-Winkel → Lenkwinkel-Sollwert
3. **Motoransteuerung**: Direkte Position/Geschwindigkeit
4. **Monitoring**: Kontinuierliche Datenaufzeichnung

## Installation

### Voraussetzungen

- **Webots**: Version R2023a oder neuer
- **GCC**: C99-kompatibel
- **Python 3**: Für GUI (tkinter, matplotlib, pandas)

### 1. Controller kompilieren

```bash
cd Regelstuerung/controllers/balance_control_c/
make clean
make
```

### 2. GUI-Abhängigkeiten installieren

```bash
cd ../../GUI/
pip install matplotlib pandas
```

### 3. Verzeichnisstruktur überprüfen

```
Regelstuerung/
├── controllers/
│   └── balance_control_c/
│       ├── balance_control_c.c      # Hauptcontroller
│       ├── balance_pid.c/.h         # PID-Implementierung
│       ├── balance_config.c/.h      # Konfigurationssystem
│       ├── balance_logging.c/.h     # Logging-System
│       ├── Makefile                 # Build-System
│       └── README.md               # Diese Datei
├── GUI/
│   ├── balance_config.json         # Konfigurationsdatei
│   └── balance_controller_gui.py   # GUI-Anwendung
└── Monitoring/                     # Log-Dateien (automatisch erstellt)
```

## Nutzung

### 1. GUI starten

```bash
cd Regelstuerung/GUI/
python balance_controller_gui.py
```

### 2. Parameter konfigurieren

In der GUI können folgende Parameter angepasst werden:

#### **Angle PID Parameter**
- **Kp (10.0)**: Proportional-Verstärkung - Hauptregelparameter
- **Ki (0.0)**: Integral-Verstärkung - meist deaktiviert für Stabilität
- **Kd (2.2)**: Differential-Verstärkung - Dämpfung

#### **Geschwindigkeitsregelung**
- **Base Speed (5.0 rad/s)**: Standard-Fahrgeschwindigkeit
- **Min Speed (3.0 rad/s)**: Mindestgeschwindigkeit für Gyroskop-Stabilität
- **Max Speed (8.0 rad/s)**: Maximale Geschwindigkeit
- **Stability Reduction (0.5)**: Geschwindigkeitsreduktion bei Instabilität

#### **Mechanische Grenzen**
- **Max Handlebar Angle (0.32 rad)**: Maximaler Lenkwinkel (~18°)
- **Max Roll Angle (45°)**: Plausibilitätsgrenze für Roll-Winkel

### 3. Webots-Simulation starten

1. Webots öffnen
2. Welt laden: `Regelstuerung/worlds/Little Bicycle V2.wbt`
3. Controller auswählen: `balance_control_c`
4. Simulation starten

### 4. Monitoring

- **Live-Monitoring**: In der GUI unter "Monitoring" → "Live-Plot starten"
- **Log-Dateien**: Automatisch in `Regelstuerung/Monitoring/`
- **CSV-Format**: `timestamp,roll_angle,steering_output,target_speed,p_term,i_term,d_term,error,stability_factor`

## Presets

Die GUI bietet vorkonfigurierte Einstellungen:

### **Autobike Original**
Bewährte Parameter aus der BachelorArbeit 2023:
- Kp=10.0, Ki=0.0, Kd=2.2
- Konservative Geschwindigkeiten

### **Konservativ**
Sichere Einstellungen für erste Tests:
- Niedrigere Verstärkungen
- Langsamere Geschwindigkeiten

### **Aggressiv**
Schnelle, responsive Einstellungen:
- Höhere Verstärkungen
- I-Anteil aktiviert
- Höhere Geschwindigkeiten

## Parameter-Tuning

### Systematisches Vorgehen

1. **Nur P-Regler**: Kp erhöhen bis leichte Oszillation
2. **D-Anteil hinzufügen**: Kd für Dämpfung optimieren
3. **I-Anteil vorsichtig**: Nur bei bleibender Regelabweichung

### Troubleshooting

| Problem | Mögliche Ursache | Lösung |
|---------|------------------|---------|
| Fahrrad fällt sofort um | Kp zu niedrig | Kp erhöhen auf 5-15 |
| Starke Oszillation | Kp zu hoch oder Kd zu niedrig | Kp reduzieren, Kd erhöhen |
| Träge Reaktion | Kd zu hoch | Kd reduzieren |
| Permanente Schieflage | Ki erforderlich | Ki vorsichtig auf 0.1-1.0 |
| Aufschaukeln | Ki zu hoch | Ki reduzieren oder deaktivieren |

## Keyboard-Steuerung

Während der Simulation:

- **ESC**: Controller beenden
- **R**: Konfiguration neu laden
- **S**: Status ausgeben
- **P**: Physik-Debug-Informationen anzeigen (neu)
- **W**: Wind-Simulation umschalten: 0 → 2 → 5 → 10 m/s Seitenwind (neu)
- **E**: Umgebungsparameter zurücksetzen (Wind aus) (neu)

## Technische Details

### PID-Implementierung

```c
// Simplified PID computation
float error = setpoint - process_variable;
float P = Kp * error;
float I += Ki * error * dt;  // with anti-windup
float D = Kd * (error - last_error) / dt;  // with filtering
float output = P + I + D;  // with limits
```

### IMU-Datenverarbeitung

```c
// Quaternion to Roll angle
roll_rad = atan2(2*(w*x + y*z), w*w - x*x - y*y + z*z);
roll_deg = roll_rad * 180.0 / M_PI;
// + filtering + limits
```

### Geschwindigkeitsadaption

```c
// Speed reduction based on instability
stability_factor = fabs(steering_output) / max_handlebar_angle;
target_speed = base_speed * (1.0 - stability_reduction * stability_factor);
```

## Entwicklung

### Build-Targets

```bash
make           # Standard-Build
make debug     # Debug-Version mit Symbolen
make release   # Optimierte Release-Version
make clean     # Bereinigung
make help      # Hilfe anzeigen
```

### Code-Struktur

- **balance_control_c.c**: Hauptschleife und Device-Management
- **balance_pid.c**: PID-Controller-Implementierung
- **balance_config.c**: JSON-Konfigurationssystem
- **balance_logging.c**: CSV-Logging und Monitoring
- **bicycle_physics.c**: Erweiterte Fahrradphysik-Simulation (neu)
  - Laterale Reifenkräfte (Pacejka-Modell)
  - Aerodynamischer Widerstand
  - Rollwiderstand
  - Gyroskopmommente
  - IMU-Sensorsimulation (Rauschen + Verzögerung)
  - Umwelteinflüsse (Wind, Straßenneigung)

### Debugging

Debug-Ausgaben aktivieren:
```bash
make debug
# Zusätzliche Konsolenausgaben verfügbar
```

## Lizenz

Dieses Projekt basiert auf der autobike-Implementierung von Jonah Zander (BachelorArbeit 2023) und ist für Forschungs- und Bildungszwecke konzipiert.

## Kontakt

Bei Fragen oder Problemen:
- Prüfen Sie die Log-Dateien in `Regelstuerung/Monitoring/`
- Kontrollieren Sie die Konfiguration in `balance_config.json`
- Verwenden Sie das GUI-Monitoring für Live-Analyse

---

**Viel Erfolg beim Balancieren! 🚴‍♂️** 