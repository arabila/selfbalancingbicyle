# Detaillierte README: Physik-Implementierung des Selbstbalancierenden Fahrrads

## Inhaltsverzeichnis

1. [Überblick](#überblick)
2. [Physikalische Grundlagen](#physikalische-grundlagen)
3. [Webots-Welt-Datei (Little Bicycle V2.wbt)](#webots-welt-datei)
4. [Balance-Controller (C)](#balance-controller-c)
5. [Vision-Controller (Python)](#vision-controller-python)
6. [Erweiterte Physik-Simulation](#erweiterte-physik-simulation)
7. [PID-Regelung](#pid-regelung)
8. [Sensorsimulation](#sensorsimulation)
9. [Zusammenfassung](#zusammenfassung)

---

## Überblick

Dieses Projekt implementiert ein selbstbalancierendes Fahrrad in Webots mit einer dualen Controller-Architektur:

- **Balance-Controller (C)**: Ultraschnelle Balance-Regelung (2ms/500Hz)
- **Vision-Controller (Python)**: Langsamere Vision-basierte Pfadplanung (50ms/20Hz)
- **Erweiterte Physik-Simulation**: Realistische Fahrradphysik mit aerodynamischen Effekten

### Architektur-Übersicht

```
┌─────────────────────┐    ┌─────────────────────┐
│  Vision Controller  │    │  Balance Controller │
│     (Python)        │    │        (C)          │
│     20 Hz           │◄──►│      500 Hz         │
└─────────────────────┘    └─────────────────────┘
           │                         │
           ▼                         ▼
┌─────────────────────┐    ┌─────────────────────┐
│  YOLO Segmentation  │    │   PID Controller    │
│  Path Planning      │    │  Roll → Steering    │
└─────────────────────┘    └─────────────────────┘
           │                         │
           ▼                         ▼
┌─────────────────────────────────────────────────┐
│            Webots Fahrrad-Simulation            │
│        + Erweiterte Physik-Simulation           │
└─────────────────────────────────────────────────┘
```

---

## Physikalische Grundlagen

### Fahrradstabilität

Ein Fahrrad ist ein **inhärent instabiles System**. Die Stabilität entsteht durch:

1. **Gyroskopmommente**: Rotierende Räder erzeugen stabilisierende Kräfte
2. **Caster-Effekt**: Vorderrad-Geometrie sorgt für selbstlenkende Eigenschaften
3. **Aktive Regelung**: PID-Controller gleicht Instabilitäten aus

### Fundamentale Gleichungen

#### Roll-Stabilität
```
M_roll = J_roll * α_roll + m * g * h * sin(θ_roll)
```
Wo:
- `M_roll`: Roll-Moment
- `J_roll`: Trägheitsmoment um Roll-Achse
- `α_roll`: Roll-Winkelbeschleunigung
- `m`: Masse
- `g`: Erdbeschleunigung
- `h`: Höhe des Schwerpunkts
- `θ_roll`: Roll-Winkel

#### Lenkwinkel-Dynamik
```
δ_optimal = K_p * θ_roll + K_d * ω_roll
```
Wo:
- `δ_optimal`: Optimaler Lenkwinkel
- `K_p`: Proportionalverstärkung
- `K_d`: Differentialverstärkung
- `ω_roll`: Roll-Winkelgeschwindigkeit

### Kräfte und Momente

1. **Gravitationskraft**: `F_g = m * g` (wirkt vertikal nach unten)
2. **Zentrifugalkraft**: `F_c = m * v² / r` (bei Kurvenfahrt)
3. **Gyroskopmomment**: `M_gyro = I_wheel * ω_wheel × ω_frame`
4. **Luftwiderstand**: `F_drag = 0.5 * ρ * C_d * A * v²`

---

## Webots-Welt-Datei (Little Bicycle V2.wbt)

### Hauptstruktur

```vrml
DEF BICYCLE Robot {
  translation -2.5291 -3.22077 -0.0492097
  rotation 0.0022654579487756074 -0.00020923791275967243 0.9999974119565402 3.1414438733326446
  children [
    DEF frame Transform { ... }
    DEF rear_wheel HingeJoint { ... }
    DEF crank HingeJoint { ... }
    DEF Handlebars_and_Fork HingeJoint { ... }
    Camera { ... }
    InertialUnit { ... }
    Receiver { ... }
    Emitter { ... }
  ]
  physics Physics {
    density -1
    mass 3.5
    centerOfMass [0 0 0.05]
    inertiaMatrix [0.1 0.1 0.05 0 0 0]
  }
}
```

### Physik-Analyse der Hauptkomponenten

#### 1. Fahrradrahmen (Frame)
**Physikalische Eigenschaften:**
```vrml
physics Physics {
  density -1        // Automatische Dichteberechnung
  mass 3.5         // Gesamtmasse 3.5 kg
  centerOfMass [0 0 0.05]  // Schwerpunkt 5cm über Boden
  inertiaMatrix [0.1 0.1 0.05 0 0 0]  // Trägheitstensor
}
```

**Physikalische Bedeutung:**
- **Masse (3.5 kg)**: Realistische Masse für ein leichtes Fahrrad
- **Schwerpunkt (5cm hoch)**: Niedriger Schwerpunkt für bessere Stabilität
- **Trägheitsmoment**: 
  - `I_xx = 0.1 kg⋅m²` (Roll-Trägheit)
  - `I_yy = 0.1 kg⋅m²` (Pitch-Trägheit)  
  - `I_zz = 0.05 kg⋅m²` (Yaw-Trägheit, niedriger für bessere Lenkbarkeit)

#### 2. Hinterrad (Rear Wheel)
```vrml
DEF rear_wheel HingeJoint {
  jointParameters HingeJointParameters {
    anchor 0 0.083 0          // Rotationsachse
    dampingConstant 0.1       // Dämpfung
    staticFriction 0.1        // Statische Reibung
  }
  device [
    RotationalMotor {
      name "motor::wheel"
      maxVelocity 200         // Max. 200 rad/s
      multiplier 2            // Getriebe-Übersetzung
    }
  ]
  endPoint Solid {
    contactMaterial "wheel"   // Kontaktmaterial definiert
    boundingObject Cylinder {
      height 0.015           // Radbreite 1.5cm
      radius 0.055           // Radius 5.5cm
    }
    physics Physics {
      mass 0.8               // Radmasse 0.8 kg
      inertiaMatrix [0.001 0.001 0.001 0 0 0]
    }
  }
}
```

**Physikalische Bedeutung:**
- **Radradius (5.5cm)**: Bestimmt Rollwiderstand und Geschwindigkeit
- **Radmasse (0.8 kg)**: Gyroskopmomment `I_wheel = 0.001 kg⋅m²`
- **Dämpfung (0.1)**: Simuliert Lagerverluste
- **Getriebe-Übersetzung (2:1)**: Verdoppelt Motormoment

#### 3. Vorderrad mit Lenkung (Handlebars and Fork)
```vrml
DEF Handlebars_and_Fork HingeJoint {
  jointParameters HingeJointParameters {
    axis 0 0 1              // Lenkachse (Z-Achse)
    anchor 0 -0.0818 0.102  // Lenkachse Position
    dampingConstant 0.2     // Lenkdämpfung
    staticFriction 0.1      // Reibung
  }
  device [
    RotationalMotor {
      name "handlebars motor"
      maxTorque 2           // Max. 2 Nm Lenkmoment
    }
  ]
}
```

**Physikalische Bedeutung:**
- **Lenkachse**: Vertikale Achse für Steering
- **Lenkdämpfung (0.2)**: Verhindert Lenkflattern
- **Lenkmoment (2 Nm)**: Ausreichend für Fahrradlenkung
- **Nachlauf**: Geometrische Selbstzentrierung

#### 4. Kontaktphysik
```vrml
ContactProperties {
  material1 "wheel"
  material2 "ground"
  coulombFriction [0.8]     // Reibungskoeffizient
  bounce 0.1                // Rückprall-Koeffizient
}
```

**Physikalische Bedeutung:**
- **Reibungskoeffizient (0.8)**: Gute Haftung zwischen Reifen und Straße
- **Rückprall (0.1)**: Minimale Elastizität für realistische Dämpfung

---

## Balance-Controller (C)

### Hauptregelschleife

```c
// Hauptregelschleife - Läuft mit 500 Hz (2ms Zeitschritt)
while (wb_robot_step(timestep) != -1) {
    // 1. Roll-Winkel messen und filtern
    float true_roll_angle = get_filtered_roll_angle();
    
    // 2. ERWEITERTE PHYSIK-SIMULATION durchführen
    float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
    
    // 3. PID-Regelung: Roll-Winkel → Lenkwinkel
    float steering_output = -pid_compute(&angle_pid, 0.0, simulated_roll_angle, current_time);
    
    // 4. Zwei-Ebenen-Regelung: Vision + Balance
    float final_steer = combine_vision_and_balance(steering_output, vision_commands);
    
    // 5. Motoren ansteuern
    wb_motor_set_position(handlebars_motor, final_steer);
    wb_motor_set_velocity(wheel_motor, target_speed);
}
```

### IMU-Datenverarbeitung

```c
static float get_filtered_roll_angle(void) {
    const double *quaternion = wb_inertial_unit_get_quaternion(imu_sensor);
    
    // Quaternion zu Euler-Winkel (Roll um X-Achse)
    // Reihenfolge in Webots: (x, y, z, w)
    double x = quaternion[0];
    double y = quaternion[1];
    double z = quaternion[2];
    double w = quaternion[3];
    
    // Normalisierung (Sicherheitscheck)
    double norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm > 0.0001) {
        w /= norm; x /= norm; y /= norm; z /= norm;
    }
    
    // Roll-Winkel-Berechnung
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll_rad = atan2(sinr_cosp, cosr_cosp);
    float roll_deg = (float)(roll_rad * 180.0 / M_PI);
    
    // KORREKTUR: Fahrrad ist um 180° gedreht
    if (roll_deg > 90.0) {
        roll_deg = 180.0 - roll_deg;
    } else if (roll_deg < -90.0) {
        roll_deg = -180.0 - roll_deg;
    }
    
    // Gleitender Durchschnitt für Stabilität
    roll_angle_history[roll_history_index] = roll_deg;
    roll_history_index = (roll_history_index + 1) % ROLL_FILTER_SIZE;
    
    float filtered_roll = 0.0;
    int count = roll_history_filled ? ROLL_FILTER_SIZE : roll_history_index;
    for (int i = 0; i < count; i++) {
        filtered_roll += roll_angle_history[i];
    }
    return (count > 0) ? filtered_roll / count : 0.0;
}
```

**Physikalische Bedeutung:**
- **Quaternion → Euler**: Umwandlung der 3D-Orientierung in Roll-Winkel
- **Normalisierung**: Korrekt für Rundungsfehler
- **180°-Korrektur**: Fahrrad ist kopfüber in der Simulation
- **Gleitender Durchschnitt**: Rauschunterdrückung (Tiefpassfilter)

### Geschwindigkeitsregelung

```c
// Geschwindigkeitsanpassung basierend auf Stabilität
float stability_factor = fabs(steering_output) / config.mechanical_limits.max_handlebar_angle;
float speed_reduction = stability_factor * config.speed_control.stability_reduction;
float target_speed = config.speed_control.base_speed * (1.0 - speed_reduction);

// Rollwiderstand berücksichtigen
float roll_resistance_torque = bicycle_physics.forces.rolling_resistance_torque;
if (fabs(roll_resistance_torque) > 0.001f) {
    float resistance_speed_reduction = fabs(roll_resistance_torque) * 0.1f;
    target_speed = target_speed * (1.0f - resistance_speed_reduction);
}
```

**Physikalische Bedeutung:**
- **Stabilitätsfaktor**: Maß für Instabilität (0 = stabil, 1 = maximale Auslenkung)
- **Geschwindigkeitsreduktion**: Langsamer fahren bei Instabilität erhöht Kontrolle
- **Rollwiderstand**: Physikalischer Widerstand reduziert effektive Geschwindigkeit

---

*Fortsetzung folgt in Teil 2...* 