# YOLO Vision Regelungssystem für Selbstbalancierendes Fahrrad

## 🚴 Überblick

Dieses Projekt implementiert ein intelligentes Regelungssystem für ein selbstbalancierendes Fahrrad, das Computer Vision (YOLO-Segmentierung) mit klassischen Regelungsalgorithmen kombiniert. Das System erkennt Straßenabschnitte in Echtzeit und steuert das Fahrrad autonom entlang der erkannten Fahrbahn.

## 🏗️ Systemarchitektur

### Hauptkomponenten

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Webots        │    │  YOLO Vision    │    │  Regelung       │
│   Simulation    │───▶│  Processing     │───▶│  (PID/MPC)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        ▲                                              │
        │              ┌─────────────────┐             │
        └──────────────│  Motor Control  │◀────────────┘
                       └─────────────────┘
```

### Datenfluss

1. **Kameraerfassung**: Webots-Kamera liefert Echtzeitbilder der Umgebung
2. **Bildverarbeitung**: YOLO-Segmentierung identifiziert Straßenabschnitte, Hindernisse und andere Objekte
3. **Fehlerberechnung**: Abweichung von der Straßenmitte wird als Regelfehler berechnet
4. **Regelungsalgorithmus**: PID- oder MPC-Controller berechnet Lenk- und Geschwindigkeitsbefehle
5. **Motoransteuerung**: Berechnete Werte werden an Webots-Motoren übertragen

## 📁 Dateistruktur und Funktionen

### Haupt-Controller

#### `yolo_vision.py`
- **Zweck**: Hauptsteuerung mit YOLO + PID-Regelung
- **Funktionen**:
  - Webots-Integration (Kamera, Motoren, Sensoren)
  - YOLO-Modell für Straßensegmentierung
  - PID-Controller für Lenkwinkel und Geschwindigkeit
  - Echtzeitvisualisierung der Segmentierungsmasken
- **Regelparameter**:
  ```python
  maxS = 5        # Max. Geschwindigkeit
  minS = 3        # Min. Geschwindigkeit  
  hMax = 0.19     # Max. Lenkwinkel
  Kp, Ki, Kd = 0.01, 0.02, 0.0001  # PID-Konstanten
  ```

#### `yolo_vision_mps.py`
- **Zweck**: Apple Silicon optimierte Version (Metal Performance Shaders)
- **Unterschiede**: 
  - Verwendet `device = "mps"` für Apple M1/M2 Chips
  - Identische Funktionalität wie Standard-Version

### Erweiterte Regelungsalgorithmen

#### `model_predictive_speed_and_steer_control.py`
- **Zweck**: Model Predictive Control (MPC) für Bahnverfolgung
- **Algorithmus**: Iterative lineare MPC für Geschwindigkeit und Lenkung
- **Parameter**:
  ```python
  NX = 4          # Zustandsvektor [x, y, v, yaw]
  NU = 2          # Eingabevektor [acceleration, steering]
  T = 2           # Vorhersagehorizont
  TARGET_SPEED = 10.0   # Zielgeschwindigkeit [m/s]
  MAX_STEER = 45°       # Max. Lenkwinkel
  ```
- **Kostenfunktion**: Optimiert Bahnverfolgung, Energieverbrauch und Komfort

#### `resources/mpc_controller.py`
- **Zweck**: Modularer MPC-Controller für Integration
- **Features**: Vereinfachte MPC-Implementierung für Echtzeitanwendungen

#### `resources/cubic_spline_planner.py`
- **Zweck**: Bahnplanung mit kubischen Splines
- **Funktion**: Generiert glatte Referenztrajektorien für MPC

### YOLO-Konfiguration

#### `data.yaml`
```yaml
nc: 4  # Anzahl Klassen
names: ['meadow', 'obstacle', 'street_main', 'street_side']
```
- **Klassen**:
  - `meadow`: Grasflächen/Natur
  - `obstacle`: Hindernisse 
  - `street_main`: Hauptfahrbahn (primäres Ziel)
  - `street_side`: Nebenfahrbahn

#### YOLO-Modelle
- `yolov8n.pt`: Nano-Modell (schnell, weniger genau)
- `yolov8s-seg.pt`: Small-Modell (ausgewogen)
- `yolov8x-seg.pt`: Extra-Large-Modell (langsam, sehr genau)
- `custom_yolo.engine`: Optimiertes TensorRT-Modell für GPU

### Konfigurationsdateien

#### `conf_compressed_img.csv`
- Konfiguration für komprimierte Bilderverarbeitung
- Parameter für Bildqualität und Verarbeitungsgeschwindigkeit

#### `conf_stretched_img.csv` / `conf_cropped_img.csv`
- Bildvorverarbeitungsparameter für verschiedene Kamera-Modi

## 🔄 Regelschleifen

### PID-Regelung (Standard)

```python
def pid_controller(error, dt):
    global P, I, D, oldP
    P = error                           # Proportional
    I = I * 2/3 + P * dt               # Integral (mit Dämpfung)
    D = D * 0.5 + (P - oldP) / dt      # Differential (mit Filterung)
    oldP = P
    return Kp * P + Ki * I + Kd * D
```

**Regelkreis**:
1. Bildaufnahme von Webots-Kamera
2. YOLO-Segmentierung → Straße erkennen
3. Fehlerberechnung: Abstand zur Straßenmitte
4. PID-Controller → Lenkwinkel berechnen
5. Geschwindigkeit anpassen basierend auf Kurvenstärke
6. Motoransteuerung in Webots

### MPC-Regelung (Erweitert)

**Zustandsvektor**: `x = [x, y, v, yaw]`
**Eingabevektor**: `u = [acceleration, steering]`

**Optimierungsproblem**:
```
minimize: Σ(||x-x_ref||²_Q + ||u||²_R + ||Δu||²_Rd)
subject to: x_{k+1} = f(x_k, u_k)
           u_min ≤ u_k ≤ u_max
```

## 🖥️ Setup und Installation

### Systemanforderungen
- Python 3.9+
- Webots Robotik-Simulator
- CUDA-fähige GPU (optional, für bessere Performance)

### Installation

#### Basis-Setup
```bash
# Conda-Umgebung erstellen
conda create -n yolo_vision python=3.11 -y
conda activate yolo_vision

# Dependencies installieren
pip install -r requirements.txt
```

#### GPU-Setup (NVIDIA)
```bash
# CUDA installieren (siehe README für Details)
pip install -r requirements_cuda.txt
```

#### Apple Silicon (M1/M2)
```bash
# PyTorch mit Metal-Support
conda install pytorch torchvision torchaudio -c pytorch-nightly -c apple
pip install ultralytics opencv-python
```

### Konfiguration

1. **YOLO-Modell wählen**:
   ```python
   # In yolo_vision.py
   model = YOLO('runs/segment/train/weights/best.pt')  # Eigenes Modell
   # oder
   model = YOLO('yolov8s-seg.pt')  # Vortrainiertes Modell
   ```

2. **PID-Parameter anpassen**:
   ```python
   Kp, Ki, Kd = 0.01, 0.02, 0.0001  # Für sanfte Regelung
   # oder
   Kp, Ki, Kd = 0.05, 0.01, 0.001   # Für aggressive Regelung
   ```

## 🚀 Verwendung

### Standard-Modus (PID)
```bash
cd /path/to/yolo_vision
python yolo_vision.py
```

### Apple Silicon
```bash
python yolo_vision_mps.py
```

### MPC-Modus (Standalone-Test)
```bash
python model_predictive_speed_and_steer_control.py
```

## 📊 Monitoring und Debugging

### Visualisierung
- Echtzeit-Kamerabild mit Segmentierungsmaske
- Webots-Simulation mit Geschwindigkeits-/Positionsanzeige
- PID-Parameter Live-Tuning möglich

### Datenaufzeichnung
- `Benchmark.csv`: Performance-Metriken
- `output_samples/`: Gespeicherte Kamerabilder mit Annotationen
- `Monitoring/`: Logdateien für Regelungsparameter

### Debugging-Tipps
1. **Kamera-Kalibrierung prüfen**: Bildqualität und Belichtung
2. **YOLO-Confidence anpassen**: `conf=0.5` in `model.predict()`
3. **PID-Tuning**: Schrittweise Kp erhöhen, dann Ki und Kd
4. **Geschwindigkeitslimits**: Bei Instabilität maxS reduzieren

## 🔧 Erweiterte Funktionen

### Custom YOLO-Training
```bash
# Eigenes Dataset trainieren
yolo segment train data=data.yaml model=yolov8s-seg.pt epochs=100
```

### TensorRT-Optimierung
```bash
# Modell für GPU optimieren
yolo export model=best.pt format=engine device=0
```

### Mehrfahrzeug-Simulation
- Unterstützung für mehrere Fahrräder (`Little Bicycle 1`, `Little Bicycle 2`, etc.)
- Individuelles Monitoring pro Fahrzeug

## 🔬 Algorithmus-Details

### YOLO-Segmentierung
- **Eingabe**: 640x640 RGB-Bild
- **Ausgabe**: Segmentierungsmasken + Confidence-Scores
- **Nachverarbeitung**: Maskenfusion, Zentroid-Berechnung

### Fehlerberechnung
```python
# Straßenmitte finden
street_indices = [idx for idx, cls in enumerate(r.boxes.cls) if cls == 2]
avg_x_center = sum(x_centers) / len(x_centers)
error = (frame_center - avg_x_center)  # Querabweichung
```

### Adaptive Geschwindigkeitsregelung
```python
# Geschwindigkeit basierend auf Kurvenstärke
speed = maxS - abs(pid_value) * 4
speed = max(speed, minS)  # Mindestgeschwindigkeit halten
```

## 📈 Performance-Optimierung

### GPU-Beschleunigung
- YOLO-Inferenz auf GPU: ~10-50ms
- CPU-Inferenz: ~100-500ms
- Apple MPS: ~20-80ms

### Modell-Auswahl
| Modell | Geschwindigkeit | Genauigkeit | Empfehlung |
|--------|----------------|-------------|------------|
| YOLOv8n | 🟢 Sehr schnell | 🟡 Mittel | Echtzeit-Tests |
| YOLOv8s | 🟢 Schnell | 🟢 Gut | Produktiv |
| YOLOv8x | 🟡 Langsam | 🟢 Sehr gut | Offline-Analyse |

## 🐛 Häufige Probleme

### CUDA-Probleme
```bash
# CUDA-Verfügbarkeit prüfen
python -c "import torch; print(torch.cuda.is_available())"
```

### Modell-Ladefehler
- Pfad zu `best.pt` überprüfen
- Fallback auf vortrainierte Modelle verwenden

### Webots-Integration
- Kamera-Device-Name überprüfen: `'camera'`
- Motor-Device-Namen: `'motor::crank'`, `'handlebars motor'`

## 📚 Weiterführende Ressourcen

- [Ultralytics YOLOv8 Dokumentation](https://docs.ultralytics.com/)
- [Webots Robotik-Simulator](https://cyberbotics.com/)
- [Model Predictive Control Tutorial](https://web.stanford.edu/class/ee364b/)
- [PID-Controller Tuning Guide](https://en.wikipedia.org/wiki/PID_controller#Manual_tuning)

## 🤝 Entwicklung und Beitrag

### Code-Struktur befolgen
- Kommentare auf Deutsch
- Funktionsdokumentation mit Docstrings
- Konsistente Variablennamen

### Testing
```bash
# Basis-Tests
python -m pytest tests/
```

### Integration neuer Features
1. Fork des Repositories
2. Feature-Branch erstellen
3. Tests implementieren
4. Pull Request erstellen

---

**Entwickelt für Masterarbeit - Selbstbalancierendes Fahrrad mit KI-basierter Pfadverfolgung** 