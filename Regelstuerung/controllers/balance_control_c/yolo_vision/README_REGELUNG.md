 # Regelung für Selbstbalancierendes Fahrrad mit YOLO-Vision

## Überblick

Dieses System implementiert eine Vision-basierte Regelung für ein selbstbalancierendes Fahrrad in der Webots-Simulationsumgebung. Die Regelung kombiniert Computer Vision (YOLO-Segmentierung) mit modernen Regelungsverfahren (PID und MPC) für autonome Spurhaltung und Pfadverfolgung.

## Systemarchitektur (Test)

### 1. Vision-System (YOLO-Segmentierung)
Das Vision-System basiert auf YOLOv8-Segmentierung und identifiziert folgende Klassen:
- **meadow** (Wiese): Umgebung
- **obstacle** (Hindernis): Zu vermeidende Objekte  
- **street_main** (Hauptstraße): Primärer Fahrbereich
- **street_side** (Seitenstraße): Sekundärer Fahrbereich

### 2. Regelungsverfahren

Das System implementiert zwei Regelungsansätze:

#### PID-Controller (`yolo_vision.py`)
- **Einfache Spurhaltung** basierend auf Querabweichung
- **Echtzeitfähig** für Webots-Integration
- **Parameter**: Kp=0.01, Ki=0.02, Kd=0.0001

#### MPC-Controller (`model_predictive_speed_and_steer_control.py`)
- **Prädiktive Regelung** mit Zeithorizont
- **Optimale Pfadverfolgung** unter Berücksichtigung von Constraints
- **Fahrzeugdynamik** wird explizit modelliert

## Dateienstruktur und Zusammenhänge

### Hauptprogramme

#### `yolo_vision.py`
**Zweck**: Hauptsteuerung mit PID-Regelung

**Funktionalität**:
- Lädt YOLO-Modell (`runs/segment/train/weights/best.pt`)
- Verarbeitet Kamerabilder aus Webots
- Führt Segmentierung durch und berechnet Querabweichung
- Implementiert PID-Controller für Lenkwinkel-Berechnung
- Steuert Fahrradmotoren (Lenkung und Antrieb)

**Wichtige Parameter**:
```python
maxS = 5      # Maximale Geschwindigkeit
minS = 3      # Minimale Geschwindigkeit  
hMax = 0.19   # Maximaler Lenkwinkel (Rad)
Kp, Ki, Kd = 0.01, 0.02, 0.0001  # PID-Parameter
```

#### `yolo_vision_mps.py`
**Zweck**: Apple Silicon optimierte Version (Metal Performance Shaders)
- Identische Funktionalität wie `yolo_vision.py`
- Verwendet `device="mps"` für Apple M1/M2 Chips

#### `model_predictive_speed_and_steer_control.py`
**Zweck**: Erweiterte MPC-Regelung für Pfadverfolgung

**Funktionalität**:
- Implementiert Model Predictive Control (MPC)
- Optimiert Geschwindigkeit und Lenkwinkel über Zeithorizont
- Berücksichtigt Fahrzeugdynamik und Constraints
- Verwendet konvexe Optimierung (cvxpy)

**MPC-Parameter**:
```python
T = 2                    # Zeithorizont
TARGET_SPEED = 10.0      # Zielgeschwindigkeit [m/s]  
MAX_STEER = 45°          # Maximaler Lenkwinkel
WB = 2.5                 # Radstand [m]
```

### Konfigurationsdateien

#### `data.yaml`
Definiert YOLO-Trainingskonfiguration:
```yaml
nc: 4
names: ['meadow', 'obstacle', 'street_main', 'street_side']
```
- Trainingsdatenpfade
- 4 Segmentierungsklassen
- Roboflow-Integration für Dataset-Management

#### `requirements.txt`
Spezifiziert Python-Abhängigkeiten:
```
torch~=2.1.2
ultralytics~=8.0.236
opencv-python~=4.9.0.80
cvxpy~=1.5.1
```

### Modelle und Gewichte

- **`yolov8n.pt`**: Nano-Modell (schnell, weniger genau)
- **`yolov8s-seg.pt`**: Small-Segmentierungsmodell  
- **`yolov8x-seg.pt`**: Extra-Large-Modell (langsam, sehr genau)
- **`custom_yolo.engine`**: TensorRT-optimiertes Modell
- **`runs/segment/train/weights/best.pt`**: Trainiertes benutzerdefiniertes Modell

### Hilfsdateien

#### `resources/`
- **`cubic_spline_planner.py`**: Pfadplanung mit kubischen Splines
- **`mpc_controller.py`**: Modularer MPC-Controller
- **`__init__.py`**: Python-Package-Initialisierung

#### Konfigurationsdateien für Bildverarbeitung
- **`conf_compressed_img.csv`**: Konfiguration für komprimierte Bilder
- **`conf_stretched_img.csv`**: Konfiguration für gestreckte Bilder  
- **`conf_cropped_img.csv`**: Konfiguration für zugeschnittene Bilder

## Regelungsverfahren im Detail

### PID-Controller (yolo_vision.py)

1. **Fehlerberechnung**: 
   ```python
   # Segmentierung identifiziert 'street_main'-Bereich
   street_indices = [idx for idx, cls in enumerate(r.boxes.cls) if cls == 2]
   avg_x_center = sum(x_centers) / len(x_centers)
   error = (frame_center - avg_x_center)  # Querabweichung
   ```

2. **PID-Regelung**:
   ```python
   P = error
   I = I * 2/3 + P * dt          # Integral mit Decay
   D = D * 0.5 + (P - oldP) / dt # Differenzial mit Glättung
   output = Kp * P + Ki * I + Kd * D
   ```

3. **Ausgabe**:
   - Lenkwinkel: begrenzt auf ±0.19 rad
   - Geschwindigkeit: adaptive Anpassung basierend auf Fehlergröße

### MPC-Controller (model_predictive_speed_and_steer_control.py)

1. **Zustandsvektor**: `x = [x, y, v, yaw]`
2. **Eingangvektor**: `u = [acceleration, steering_angle]`
3. **Optimierungsproblem**:
   - Minimiere Kosten über Zeithorizont T
   - Berücksichtige Fahrzeugdynamik
   - Respektiere Constraints (max. Geschwindigkeit, Lenkwinkel)

4. **Kostenfunktion**:
   ```python
   J = Σ(x'*Q*x + u'*R*u + Δu'*Rd*Δu) + x_final'*Qf*x_final
   ```

## Installation & Setup

### Systemanforderungen
- Python 3.9+ 
- CUDA (optional, für GPU-Beschleunigung)
- Webots R2023b oder neuer

### Installation

1. **Python-Umgebung erstellen**:
   ```bash
   conda create -n yolo_vision python=3.11 -y
   conda activate yolo_vision
   ```

2. **Abhängigkeiten installieren**:
   ```bash
   pip install -r requirements.txt
   ```

3. **CUDA-Setup** (optional):
   ```bash
   pip install -r requirements_cuda.txt
   ```

## Verwendung

### PID-basierte Regelung
```bash
python yolo_vision.py
```

### Apple Silicon Version
```bash
python yolo_vision_mps.py
```

### MPC-basierte Regelung  
```bash
python model_predictive_speed_and_steer_control.py
```

## Systemparameter Tuning

### PID-Parameter anpassen
```python
# In yolo_vision.py
Kp = 0.01    # Proportional: Reaktion auf aktuellen Fehler
Ki = 0.02    # Integral: Eliminiert stationären Fehler  
Kd = 0.0001  # Differenzial: Dämpft Schwingungen
```

### MPC-Parameter anpassen
```python
# In model_predictive_speed_and_steer_control.py
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # Zustandskosten [x, y, v, yaw]
R = np.diag([0.01, 0.01])           # Eingabekosten [accel, steer]
T = 2                               # Prädiktionshorizont [s]
```

## Funktionsweise der Regelschleife

### Regelkreis PID-System

1. **Bildaufnahme**: Webots-Kamera erfasst Umgebungsbild
2. **YOLO-Segmentierung**: Identifikation von Straßenabschnitten
3. **Fehlerberechnung**: Abweichung von Straßenmitte bestimmen
4. **PID-Regelung**: Berechnung von Lenkwinkel und Geschwindigkeit
5. **Motoransteuerung**: Übertragung an Webots-Motoren
6. **Wiederholung** in jedem Zeitschritt

### Datenfluss

```
Kamera → YOLO → Fehlerberechnung → PID → Motoren → Fahrrad → Kamera
  ↑                                                              ↓
  └─────────────────── Regelkreis ──────────────────────────────┘
```

## Monitoring & Debugging

### Visualisierung
- Kamerabild mit Segmentierungsmaske: `cv2.imshow()`
- Webots-Display: Geschwindigkeit und Status
- Matplotlib-Plots: Pfadverfolgung (MPC)

### Log-Dateien
- **`Benchmark.csv`**: Performance-Metriken
- **`conf_*.csv`**: Bildkonfigurationsdaten
- **`ndarray.txt`**: Numerische Ausgabedaten

## Erweiterte Funktionen

### Modell-Training
```bash
# YOLO-Modell mit eigenem Dataset trainieren
yolo segment train data=data.yaml model=yolov8s-seg.pt epochs=100
```

### TensorRT-Optimierung
```bash
# Modell für GPU-Inferenz optimieren  
yolo export model=best.pt format=engine
```

## Fehlerbehebung

### Häufige Probleme

1. **CUDA nicht verfügbar**: 
   - Überprüfe CUDA-Installation: `torch.cuda.is_available()`
   - Fallback auf CPU-Modus automatisch

2. **Webots-Verbindung fehlgeschlagen**:
   - Überprüfe Controller-Konfiguration in .wbt-Datei
   - Sicherstelle korrekte Geräte-Namen

3. **YOLO-Modell lädt nicht**:
   - Überprüfe Pfad zu `best.pt`
   - Lade alternatives Modell: `yolov8n.pt`

### Performance-Optimierung

- **GPU verwenden**: `device="0"` in YOLO-Predict
- **Modellgröße reduzieren**: nano (n) statt extra-large (x)
- **Bildauflösung anpassen**: Kleinere Bilder = schnellere Inferenz
- **TensorRT verwenden**: `.engine`-Dateien für NVIDIA-GPUs

## Wissenschaftlicher Hintergrund

Dieses System implementiert moderne Ansätze der autonomen Fahrzeugführung:

- **Computer Vision**: Semantische Segmentierung für Umgebungswahrnehmung
- **Regelungstheorie**: PID für einfache, MPC für optimale Regelung
- **Echtzeitsysteme**: Integration in Webots-Simulationsumgebung
- **Machine Learning**: Trainierte YOLO-Modelle für robuste Objekterkennung

## Technische Besonderheiten

### Adaptive Geschwindigkeitsregelung
```python
# Geschwindigkeit wird basierend auf Kurvenstärke angepasst
speed = maxS - abs(pid_value) * 4
if speed < minS:
    speed = minS
```

### Robuste PID-Implementation
- Integral-Windup-Schutz durch Decay-Faktor
- Differenzial-Filterung zur Rauschreduzierung
- Saturierung der Ausgangswerte

### Multi-Robot-Unterstützung
Das System unterstützt mehrere Fahrräder gleichzeitig:
- `Little Bicycle 1`, `Little Bicycle 2`, `Little Bicycle V2`
- Individuelles Monitoring pro Fahrzeug

## Lizenz

Dieses Projekt basiert auf verschiedenen Open-Source-Komponenten:
- YOLOv8: AGPL-3.0 License
- Webots: Apache License 2.0  
- PyTorch: BSD License 