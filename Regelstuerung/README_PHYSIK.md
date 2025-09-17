# Physik-Simulation in Webots - ODE Engine und Fahrradmodellierung

Dieses Dokument erklärt die physikalische Modellierung des selbstbalancierenden Fahrrads in der Webots-Simulationsumgebung, basierend auf der **Open Dynamics Engine (ODE)** und der Hauptsimulationswelt `S-Kurve.wbt`.

## 🔬 Webots Physik-Engine (ODE)

### Open Dynamics Engine - Grundlagen
Webots verwendet die **Open Dynamics Engine (ODE)** für realistische Physiksimulation:
- **Rigid Body Dynamics**: Präzise Simulation starrer Körper mit Masse und Trägheit
- **Constraint-based Simulation**: Gelenke und Verbindungen zwischen Körpern
- **Collision Detection**: Kontinuierliche Kollisionserkennung für Boden-Kontakt
- **Numerical Integration**: Runge-Kutta Integration für stabile Simulation

### Zeitschritt und Genauigkeit
```wbt
WorldInfo {
  basicTimeStep 2
}
```
- **2 ms Zeitschritt** = 500 Hz Simulationsfrequenz
- Hochauflösende Physiksimulation für präzise Regelung
- Optimales Verhältnis zwischen Genauigkeit und Rechenaufwand
- Ermöglicht Echtzeit-fähige Balance-Regelung

## 🌍 S-Kurve.wbt - Hauptsimulationswelt

### Weltaufbau und Umgebung
```wbt
RectangleArena {
  translation 0 0 -0.1
  contactMaterial "ground"
  floorSize 64 100
  floorTileSize 64 100
  floorAppearance PBRAppearance {
    baseColorMap ImageTexture {
      url ["textures/S-Kurve.png"]
    }
    roughness 1
    metalness 0
  }
}
```

#### Boden und Kontakteigenschaften
- **Große Fahrbahn**: 64m × 100m für ausgedehnte Testfahrten
- **S-Kurven-Textur**: Visueller Pfad für Computer Vision System
- **Realistische Oberflächeneigenschaften**: Rauheit und Metallgehalt für Beleuchtung

### Physikalische Kontaktmodellierung
```wbt
contactProperties [
  ContactProperties {
    material1 "wheel"
    material2 "ground"
    coulombFriction [0.8]
    bounce 0.3
  }
]
```
- **Coulomb-Reibung**: μ = 0.8 (realistischer Reifen-Asphalt-Kontakt)
- **Rückprall-Koeffizient**: 0.3 (gedämpfte Kollisionen)
- **Material-spezifische Eigenschaften**: Differenzierung zwischen Reifen und Boden

## 🚲 Fahrradphysik - Detaillierte Modellierung

### Aerodynamische Simulation
```wbt
DEF AIR Fluid {
  name "air"
  density 1.225
  viscosity 1.81e-05
  boundingObject Box {
    size 100 100 20
  }
}
```

#### Luftwiderstand und Immersion
```wbt
immersionProperties [
  ImmersionProperties {
    fluidName "air"
    referenceArea "xyz-projected area"
    dragForceCoefficients 1 1 1
    dragTorqueCoefficients 0.05 0.05 0.05
  }
]
```
- **Realistische Luftdichte**: 1.225 kg/m³ (Standardbedingungen)
- **Viskosität**: 1.81×10⁻⁵ Pa⋅s (dynamische Luftviskosität)
- **3D-Luftwiderstand**: Kraft- und Drehmoment-Koeffizienten für alle Achsen

### Fahrradkomponenten - Physikalische Eigenschaften

#### Hauptrahmen (Frame)
```wbt
physics Physics {
  density -1
  mass 4
  centerOfMass [0 -0.1 0.32]
  inertiaMatrix [0.08 0.05 0.07, 0 0 0]
}
```
- **Explizite Masse**: 4 kg (density = -1 deaktiviert automatische Berechnung)
- **Schwerpunkt**: [0, -0.1, 0.32] m (realistisch nach hinten-unten versetzt)
- **Trägheitsmatrix**: Realistische Werte für Roll-, Nick- und Gierträgheit

#### Räder - Dynamische Modellierung
```wbt
boundingObject Pose {
  rotation 0 1 0 1.5708
  children [
    Cylinder {
      height 0.06
      radius 0.45
      subdivision 72
    }
  ]
}
```
- **Zylindrische Kollisionsgeometrie**: Vereinfacht aber ausreichend präzise
- **Realistische Abmessungen**: 45 cm Radius, 6 cm Breite
- **Hohe Subdivision**: 72 Segmente für glatte Rollbewegung

### Gelenk-Modellierung (HingeJoint)

#### Lenkungsgelenk
```wbt
DEF Handlebars_and_Fork HingeJoint {
  jointParameters HingeJointParameters {
    axis 0 0 1
    anchor 0 -0.52352 0.6528
    dampingConstant 0.3
    staticFriction 0.1
  }
}
```
- **Rotationsachse**: Z-Achse (vertikale Lenkachse)
- **Dämpfung**: 0.3 N⋅m⋅s/rad (realistische Lenkungsdämpfung)
- **Reibung**: 0.1 (Lager-Reibung)

#### Motor-Integration
```wbt
device [
  RotationalMotor {
    name "handlebars motor"
    maxTorque 25
  }
  PositionSensor {
    name "handlebars sensor"
  }
]
```
- **Maximales Drehmoment**: 25 N⋅m (ausreichend für Lenkbewegungen)
- **Positionssensor**: Präzise Winkelerfassung für Regelung

## 📡 Controller-Kommunikation in der Physik

### IPC-Kommunikation über Webots
```wbt
Receiver {
  name "command_rx"
  channel 1
  bufferSize 64
}
Emitter {
  name "status_tx"
  channel 2
  bufferSize 64
}
```
- **Bidirektionale Kommunikation**: Empfang von Vision-Befehlen, Senden von Status
- **Puffergröße**: 64 Bytes für strukturierte Datenpakete
- **Kanaltrennung**: Separate Kanäle für verschiedene Datenströme

### Supervisor-System für erweiterte Physik
```wbt
Supervisor {
  children [
    DEF VISION_CAMERA_TRANSFORM Pose {
      translation 7.641 -23.709 0.349
      children [
        Camera {
          fieldOfView 2
          width 640
          height 360
        }
      ]
    }
  ]
}
```
- **Externe Kamera-Perspektive**: Überwachung des Fahrrads von außen
- **Supervisor-Rechte**: Zugriff auf Weltkoordinaten und Objektpositionen
- **Vision-Integration**: Kamera für Computer Vision Algorithmen

## 🔧 Erweiterte Physik-Implementierung (C-Code)

### Fahrradphysik-Modul
```c
// Erweiterte Fahrradphysik initialisieren
bicycle_physics_init(&bicycle_physics, robot_node, timestep);
// Beispielhafte Umgebungsparameter (Seitenwind)
bicycle_physics_set_environment(&bicycle_physics, 2.0f, 0.5f, 0.2f);
```
Initialisiert erweiterte Physik-Simulation mit:
- **Windgeschwindigkeit**: 2.0 m/s
- **Windrichtung**: 0.5 rad
- **Turbulenz**: 0.2 (20% Schwankung)

### Aerodynamische Kräfte
```c
// Luftwiderstandskraft F = 0.5 * rho * Cd * A * v²
float drag_magnitude = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA * speed * speed;
```
- **Physikalisch korrekte Formel**: Quadratische Geschwindigkeitsabhängigkeit
- **Realistische Konstanten**: Luftdichte, Widerstandsbeiwert, Stirnfläche

### Reifenmodellierung (Pacejka)
```c
// Pacejka-Modell für seitliche Reifenkräfte
float lateral_force = D * sinf(C * atanf(B * slip_angle));
```
- **Magic Formula**: Industriestandard für Reifenmodellierung
- **Slip-Winkel-Abhängigkeit**: Realistische Seitenkraft-Charakteristik

### Gyroskopische Effekte
```c
// Gyroskopmoment zur Stabilisierung
gyro_torque[0] = -wheel_inertia * wheel_angular_vel * omega_z;
```
- **Rotierende Räder**: Gyroskopisches Moment für natürliche Stabilisierung
- **Physikalisch korrekt**: Kreuzprodukt aus Winkelgeschwindigkeiten

## 📊 Sensor-Simulation

### IMU mit realistischen Eigenschaften
```c
// Sensor-Simulation mit Verzögerung und Rauschen
physics->sensor_sim.imu_buffer[physics->sensor_sim.buffer_index] = true_roll;
float noise = gaussian_noise(IMU_NOISE_SIGMA);
```
- **Sensor-Verzögerung**: Pufferung für realistische Latenz
- **Gauß'sches Rauschen**: Simuliert reale Sensoreigenschaften
- **Kalibrierbare Parameter**: Anpassbar an verschiedene IMU-Typen

### Physik-Simulation Schritt für Schritt
```c
// Physik-Simulation durchführen
float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
```
Pro Simulationsschritt werden berechnet:
1. **Aerodynamische Kräfte** (Luft- und Seitenwiderstand)
2. **Reifenkräfte** (Pacejka-Modell)
3. **Gyroskopische Momente** (rotierende Räder)
4. **Umgebungseinflüsse** (Wind, Störungen)
5. **Sensor-Simulation** (Rauschen, Verzögerung)

## 🎯 Physikalische Validierung

Die Implementierung wurde validiert durch:
- **Vergleich mit Hardware-Prototyp**: Ähnliches Verhalten in Grundszenarien
- **Physikalische Plausibilität**: Korrekte Reaktion auf Störungen
- **Parametervariation**: Robustheit gegenüber Modellparametern
- **Grenzwertanalyse**: Verhalten bei extremen Bedingungen

---

**Fazit**: Die ODE-basierte Physik-Simulation in Webots ermöglicht eine realistische und präzise Modellierung des selbstbalancierenden Fahrrads. Die Kombination aus Webots-nativer Physik und erweiterten C-Modulen bietet die notwendige Genauigkeit für die Entwicklung und Validierung von Regelungsalgorithmen.