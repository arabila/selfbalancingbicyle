# Fahrrad Datenblatt - Little Bicycle V2

## Übersicht
Dieses Dokument enthält alle technischen Spezifikationen des selbstbalancierenden Fahrrads "Little Bicycle V2" aus der Webots-Simulation.

---

## 🚲 **Rahmen (Frame)**
- **Gewicht**: 4 kg
- **Massenschwerpunkt**: [0, -0.1, 0.32] m
- **Trägheitsmatrix**: [0.08, 0.05, 0.07] kg⋅m²
- **Skalierung**: 0.0064 (alle Achsen)
- **Material**: CAD-Modell (frame.obj)

---

## ⚙️ **Hinterrad (Rear Wheel)**
- **Radius**: 0.45 m
- **Breite**: 0.06 m
- **Gewicht**: 2 kg
- **Massenschwerpunkt**: [0, 0, 0] m
- **Trägheitsmatrix**: [0.1, 0.1, 0.2] kg⋅m²
- **Position**: [0, 0.5312, 0] m
- **Dämpfungskonstante**: 0.8 N⋅m⋅s/rad
- **Haftreibung**: 0.1
- **Kontaktmaterial**: "wheel"
- **Skalierung**: [0.004, 0.0081, 0.0081]

### Motor Spezifikationen
- **Name**: "motor::wheel"
- **Maximales Drehmoment**: 120 N⋅m
- **Positionssensor**: "rear wheel sensor"

---

## 🛞 **Vorderrad (Front Wheel)**
- **Radius**: 0.45 m
- **Breite**: 0.06 m
- **Gewicht**: 2 kg
- **Massenschwerpunkt**: [0, 0, 0] m
- **Trägheitsmatrix**: [0.1, 0.1, 0.2] kg⋅m²
- **Position**: [0, -0.2368, -0.6528] m (relativ zur Gabel)
- **Dämpfungskonstante**: 0.8 N⋅m⋅s/rad
- **Haftreibung**: 0.1
- **Kontaktmaterial**: "wheel"
- **Skalierung**: [0.004, 0.0081, 0.0081]

---

## 🔧 **Lenker und Gabel (Handlebars & Fork)**
- **Gewicht**: 2.5 kg
- **Massenschwerpunkt**: [0, 0, 0] m
- **Trägheitsmatrix**: [0.06, 0.07, 0.12] kg⋅m²
- **Drehachse**: [0, 0, 1] (Z-Achse)
- **Ankerpunkt**: [0, -0.52352, 0.6528] m
- **Dämpfungskonstante**: 0.3 N⋅m⋅s/rad
- **Haftreibung**: 0.1
- **Skalierung**: 0.0064 (alle Achsen)

### Lenker Motor
- **Name**: "handlebars motor"
- **Maximales Drehmoment**: 25 N⋅m
- **Positionssensor**: "handlebars sensor"

---

## 🚴 **Kurbel und Pedale (Crank & Pedals)**

### Kurbel (Crank)
- **Gewicht**: 4 kg
- **Massenschwerpunkt**: [0, 0, 0] m
- **Trägheitsmatrix**: [0.015, 0.0236, 0.038] kg⋅m²
- **Dämpfungskonstante**: 0.2 N⋅m⋅s/rad
- **Haftreibung**: 0.05
- **Skalierung**: 0.0064 (alle Achsen)

### Kurbel Motor
- **Name**: "motor::crank"
- **Maximales Drehmoment**: 80 N⋅m

### Pedale (je Pedal)
- **Gewicht**: 0.32 kg (pro Pedal)
- **Massenschwerpunkt**: [0, 0, 0] m
- **Abmessungen**: 0.128 × 0.128 × 0.0384 m
- **Position Links**: [-0.24, -0.192, 0] m
- **Position Rechts**: [0.24, 0.192, 0] m
- **Skalierung**: 0.0064 (alle Achsen)

---

## 📷 **Sensoren und Kameras**

### Hauptkamera (Bicycle Camera)
- **Position**: [0.96, -1.1, 0.95] m
- **Sichtfeld**: 2 rad (≈ 114.6°)
- **Auflösung**: 480 × 320 Pixel
- **Anti-Aliasing**: Aktiviert

### Vision Camera (Supervisor)
- **Position**: [7.641, -23.709, 0.349] m
- **Sichtfeld**: 2 rad (≈ 114.6°)
- **Auflösung**: 640 × 360 Pixel
- **Höhenversatz**: [0, -0.8, 1.3] m

### Inertial Measurement Unit (IMU)
- **Name**: "imu"
- **Position**: [0, 0, 0.32] m
- **Rotation**: [1, 0, 0, 0] (keine Rotation)

---

## 📡 **Kommunikation**

### Receiver (Empfänger)
- **Name**: "command_rx"
- **Kanal**: 1
- **Puffergröße**: 64 Bytes

### Emitter (Sender)
- **Name**: "status_tx"
- **Kanal**: 2
- **Puffergröße**: 64 Bytes

---

## 🌪️ **Aerodynamik (Fluid Immersion)**
Alle Fahrradkomponenten haben folgende Luftwiderstandseigenschaften:
- **Fluid**: Luft ("air")
- **Referenzfläche**: "xyz-projected area"
- **Luftwiderstandskoeffizienten**: [1, 1, 1]
- **Drehmoment-Widerstandskoeffizienten**: [0.05, 0.05, 0.05]

---

## 🌍 **Umgebung**

### Luft (Air Fluid)
- **Name**: "air"
- **Dichte**: 1.225 kg/m³
- **Viskosität**: 1.81×10⁻⁵ Pa⋅s
- **Bounding Box**: 100 × 100 × 20 m

### Bodenkontakt
- **Material 1**: "wheel"
- **Material 2**: "ground"
- **Coulomb-Reibung**: 0.8
- **Rückprall**: 0.3

---

## 🎮 **Controller**
- **Balance Controller**: "balance_control_c"
- **Vision Controller**: "vision_control_py"
- **Supervisor**: Aktiviert

---

## 📊 **Gesamtspezifikationen**
- **Gesamtgewicht**: ~12.82 kg (4 + 2 + 2 + 2.5 + 4 + 0.64 kg)
- **Radstand**: ~1.1848 m (berechnet aus Positionen)
- **Maximale Motorleistung**: 225 N⋅m (kombiniert)
- **Zeitschritt**: 2 ms
- **Skalierungsfaktor**: 0.0064 (Modell zu Real)

---

## 📝 **Anmerkungen**
- Alle Massen sind explizit definiert (density = -1)
- Das Fahrrad verwendet CAD-Modelle (.obj Dateien) für realistische Darstellung
- Die Simulation läuft mit einem Basis-Zeitschritt von 2 ms
- Das Fahrrad ist als Supervisor-Robot konfiguriert für erweiterte Kontrolle

---

*Erstellt basierend auf der Webots-Weltdatei: S-Kurve.wbt*
