# Detaillierte README: Physik-Implementierung des Selbstbalancierenden Fahrrads - Teil 2

## Vision-Controller (Python)

### Hauptarchitektur

```python
class VisionController:
    def __init__(self):
        # Dual-Kamera-System
        self.camera = self.robot.getDevice('camera')  # Vision Controller Kamera
        self.camera_transform_node = self.robot.getFromDef('VISION_CAMERA_TRANSFORM')
        
        # YOLO-Modell für Straßenerkennung
        self._init_yolo()
        
        # Vision-PID-Parameter
        self.vision_kp = 50    # Proportionalverstärkung
        self.vision_ki = 0     # Integral deaktiviert
        self.vision_kd = 0     # Differential deaktiviert
        
        # Geschwindigkeitsparameter
        self.base_speed = 0.5  # Basis-Geschwindigkeit
        self.min_speed = 0.3   # Mindestgeschwindigkeit
        self.max_speed = 0.6   # Maximalgeschwindigkeit
```

### Kamera-Positionierung

```python
def _update_camera_position(self):
    """Aktualisiert die Vision Controller Kamera-Position relativ zum Fahrrad"""
    if not (self.bicycle and self.camera_transform_node):
        return
        
    try:
        # Fahrrad-Position und -Rotation abrufen
        bike_pos = self.bicycle.getPosition()
        bike_rotation = self.bicycle.getField('rotation').getSFRotation()
        
        # Vision Controller Kamera-Transform exakt an Fahrradposition setzen
        translation_field = self.camera_transform_node.getField('translation')
        translation_field.setSFVec3f(bike_pos)
        
        # Fahrrad-Rotation auf Vision Controller Transform übertragen
        rotation_field = self.camera_transform_node.getField('rotation')
        rotation_field.setSFRotation(bike_rotation)
        
    except Exception as e:
        print(f"Vision Controller Kamera-Positionierung-Fehler: {e}")
```

**Physikalische Bedeutung:**
- **Dual-Kamera-System**: Eine Kamera am Fahrrad, eine am Vision Controller
- **Dynamische Positionierung**: Kamera folgt dem Fahrrad in Echtzeit
- **Koordinatentransformation**: Weltkoordinaten → Fahrradkoordinaten

### YOLO-basierte Straßenerkennung

```python
def get_vision_error_yolo(self, frame):
    """YOLO-basierte Straßenerkennung und Fehlerberechnung"""
    if not self.yolo_model:
        return self._use_last_valid_values()
    
    try:
        # YOLO-Vorhersage mit Konfidenz-Schwelle
        results = self.yolo_model.predict(
            source=frame,
            conf=0.5,        # 50% Konfidenz-Schwelle
            max_det=5,       # Maximal 5 Objekte
            show=False,
            verbose=False
        )
        
        # Segmentierungsmasken verarbeiten
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        if results[0].masks is not None:
            for seg in results[0].masks.data:
                seg = seg.cpu().numpy()
                seg_resized = cv2.resize(seg, (frame.shape[1], frame.shape[0]))
                mask = np.maximum(mask, seg_resized.astype(np.uint8))
        
        # Straßen-Klasse (ID 2) suchen
        if results[0].boxes is not None:
            street_indices = [idx for idx, cls in enumerate(results[0].boxes.cls) if cls == 2]
            
            if street_indices:
                # Mittelpunkt der Straßen-Bounding-Boxes berechnen
                x_centers = []
                for idx in street_indices:
                    xyxy = results[0].boxes.xyxy[idx]
                    x_center = float((xyxy[0] + xyxy[2]) / 2.0)
                    x_centers.append(x_center)
                
                avg_x_center = sum(x_centers) / len(x_centers)
                frame_center = frame.shape[1] / 2
                error = (frame_center - avg_x_center) / frame.shape[1]  # Normiert
                
                # Gültige Werte speichern
                self._store_valid_values(error, mask, mask_coverage)
                return error, mask
        
        # Keine Straße erkannt - verwende letzte gültige Werte
        return self._use_last_valid_values()
        
    except Exception as e:
        print(f"YOLO-Fehler: {e}")
        return self._use_last_valid_values()
```

**Physikalische Bedeutung:**
- **Segmentierung**: Pixelgenaue Straßenerkennung
- **Schwerpunkt-Berechnung**: Geometrische Mittelpunkt-Bestimmung
- **Normierter Fehler**: Abweichung von der Bildmitte (-0.5 bis +0.5)
- **Fehlerbehandlung**: Robuste Verarbeitung bei Erkennungsfehlern

### Vision-PID-Regelung

```python
def vision_pid_control(self, error, dt):
    """PID-Controller für Vision-basierte Lenkung"""
    # P-Term (Proportional)
    p_term = self.vision_kp * error
    
    # I-Term mit Anti-Windup
    self.vision_integral += error * dt
    integral_limit = 0.5
    self.vision_integral = max(-integral_limit, min(integral_limit, self.vision_integral))
    i_term = self.vision_ki * self.vision_integral
    
    # D-Term (Differential)
    d_error = (error - self.vision_last_error) / dt if dt > 0 else 0.0
    d_term = self.vision_kd * d_error
    
    # PID-Terme für IPC-Übertragung speichern
    self.vision_p_term = p_term
    self.vision_i_term = i_term
    self.vision_d_term = d_term
    self.vision_error = error
    
    # Gesamtausgang
    output = p_term + i_term + d_term
    
    # Ausgang begrenzen
    output = max(-self.max_steer, min(self.max_steer, output))
    
    # Für nächsten Zyklus
    self.vision_last_error = error
    
    return output, p_term, i_term, d_term
```

**Physikalische Bedeutung:**
- **Proportionalterm**: Direkte Reaktion auf Abweichung
- **Integralterm**: Korrektur bleibender Regelabweichung
- **Differentialterm**: Dämpfung schneller Änderungen
- **Anti-Windup**: Verhindert Integral-Überschwingen

---

## Erweiterte Physik-Simulation

### Hauptphysik-Struktur

```c
typedef struct {
    bicycle_state_t state;           // Aktueller Zustand
    bicycle_forces_t forces;         // Berechnete Kräfte
    sensor_simulation_t sensor_sim;  // Sensorfehler-Simulation
    environment_t environment;       // Umgebungsbedingungen
    
    // Webots-Referenzen
    WbNodeRef robot_node;
    WbNodeRef rear_wheel_node;
    WbNodeRef front_wheel_node;
    
    // Berechnungsparameter
    bool initialized;
    double last_update_time;
    float integration_step;
} bicycle_physics_t;
```

### Laterale Reifenkräfte (Pacejka-Modell)

```c
float bicycle_physics_tire_lateral_force(float slip_angle, float normal_force) {
    // Vereinfachtes Pacejka-Modell für Seitenkräfte
    // Fy = D * sin(C * atan(B * slip_angle))
    
    float B = 10.0f;    // Steifigkeitsfaktor (typisch 8-15)
    float C = 1.3f;     // Formfaktor (typisch 1.2-1.4)
    float D = normal_force * TIRE_FRICTION_COEFF; // Maximale Seitenkraft
    
    // Berechnung der Seitenkraft
    float lateral_force = D * sinf(C * atanf(B * slip_angle));
    
    // Für kleine Slip-Winkel: lineare Näherung
    if (fabsf(slip_angle) < 0.1f) {
        lateral_force = TIRE_STIFFNESS * slip_angle;
    }
    
    return lateral_force;
}
```

**Physikalische Bedeutung:**
- **Pacejka-Modell**: Bewährtes Reifenmodell aus der Fahrzeugtechnik
- **Slip-Winkel**: Winkel zwischen Fahrtrichtung und Radrichtung
- **Steifigkeitsfaktor B**: Bestimmt Linearität der Kraft-Winkel-Beziehung
- **Formfaktor C**: Beeinflusst Kurvenverlauf
- **Spitzenfaktor D**: Maximale Seitenkraft bei gegebenem Normalkraft

### Aerodynamischer Widerstand

```c
void bicycle_physics_aerodynamic_drag(const float velocity[3], float drag_force[3]) {
    float speed = vector_magnitude(velocity);
    
    if (speed < 0.1f) {
        memset(drag_force, 0, 3 * sizeof(float));
        return;
    }
    
    // Luftwiderstandskraft: F = 0.5 * rho * Cd * A * v²
    float drag_magnitude = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA * speed * speed;
    
    // Kraft wirkt entgegengesetzt zur Bewegungsrichtung
    float velocity_unit[3];
    vector_normalize(velocity_unit, velocity);
    
    drag_force[0] = -drag_magnitude * velocity_unit[0];
    drag_force[1] = -drag_magnitude * velocity_unit[1];
    drag_force[2] = -drag_magnitude * velocity_unit[2];
}
```

**Physikalische Bedeutung:**
- **Widerstandsgleichung**: Klassische Aerodynamik-Formel
- **Luftdichte ρ**: 1.225 kg/m³ bei Normalbedingungen
- **Widerstandsbeiwert Cd**: 0.9 für Fahrrad + Fahrer
- **Stirnfläche A**: 0.3 m² für kompakte Fahrradgeometrie
- **Quadratische Geschwindigkeitsabhängigkeit**: Charakteristisch für Luftwiderstand

### Rollwiderstand

```c
float bicycle_physics_rolling_resistance(float wheel_velocity, float normal_force) {
    // Rollwiderstandsmoment: M = Crr * N * r
    float resistance_force = ROLLING_RESISTANCE_COEFF * normal_force;
    float resistance_torque = resistance_force * WHEEL_RADIUS;
    
    // Richtung: entgegengesetzt zur Drehrichtung
    if (wheel_velocity > 0.01f) {
        return -resistance_torque;
    } else if (wheel_velocity < -0.01f) {
        return resistance_torque;
    } else {
        return 0.0f; // Kein Widerstand bei stillstehendem Rad
    }
}
```

**Physikalische Bedeutung:**
- **Rollwiderstandskoeffizient**: 0.005 für Fahrradreifen auf Asphalt
- **Normalkraft**: Gewichtskraft des Fahrrads
- **Moment-Berechnung**: Kraft × Hebelarm (Radradius)
- **Geschwindigkeitsunabhängigkeit**: Charakteristisch für Rollwiderstand

### Gyroskopmommente

```c
void bicycle_physics_gyroscopic_effects(float wheel_angular_vel, const float frame_angular_vel[3], float gyro_torque[3]) {
    // Gyroskopmomment: M = I * omega_wheel × omega_frame
    float wheel_inertia = 0.002f; // Trägheitsmoment des Rades
    
    // Vereinfachte Berechnung: Hauptsächlich Stabilisierung um Roll-Achse
    float omega_x = frame_angular_vel[0]; // Roll-Rate
    float omega_z = frame_angular_vel[2]; // Yaw-Rate
    
    // Gyroskopmomment wirkt stabilisierend
    gyro_torque[0] = -wheel_inertia * wheel_angular_vel * omega_z; // Roll-Stabilisierung
    gyro_torque[1] = wheel_inertia * wheel_angular_vel * omega_x;  // Pitch-Moment
    gyro_torque[2] = 0.0f; // Yaw wird nicht direkt beeinflusst
    
    // Begrenze Mommente auf realistische Werte
    for (int i = 0; i < 3; i++) {
        gyro_torque[i] = clamp(gyro_torque[i], -5.0f, 5.0f);
    }
}
```

**Physikalische Bedeutung:**
- **Gyroskopmomment**: Kreuzprodukt aus Rad- und Rahmen-Winkelgeschwindigkeit
- **Trägheitsmoment**: 0.002 kg⋅m² für Fahrradrad
- **Stabilisierungseffekt**: Gyroskopmomment wirkt Roll-Bewegung entgegen
- **Präzession**: Lenkbewegung erzeugt Roll-Moment

### Umwelteinflüsse

```c
void bicycle_physics_environment_effects(bicycle_physics_t *physics) {
    // SEITENWIND-EFFEKTE
    if (physics->environment.wind_speed > 0.1f) {
        float wind_velocity[3];
        
        // Windgeschwindigkeit in Weltkoordinaten
        float cos_dir = cosf(physics->environment.wind_direction);
        float sin_dir = sinf(physics->environment.wind_direction);
        
        wind_velocity[0] = physics->environment.wind_speed * cos_dir;
        wind_velocity[1] = physics->environment.wind_speed * sin_dir;
        wind_velocity[2] = 0.0f;
        
        // Relative Windgeschwindigkeit (Wind - Fahrradgeschwindigkeit)
        float relative_wind[3];
        for (int i = 0; i < 3; i++) {
            relative_wind[i] = wind_velocity[i] - physics->state.velocity[i];
        }
        
        float rel_wind_speed = vector_magnitude(relative_wind);
        
        if (rel_wind_speed > 0.1f) {
            // Seitenwindkraft berechnen
            float side_drag_magnitude = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * SIDE_AREA * 
                                      rel_wind_speed * rel_wind_speed;
            
            // Kraft in Richtung des relativen Windes
            float wind_unit[3];
            vector_normalize(wind_unit, relative_wind);
            
            physics->forces.crosswind_force[0] = side_drag_magnitude * wind_unit[0];
            physics->forces.crosswind_force[1] = side_drag_magnitude * wind_unit[1];
            physics->forces.crosswind_force[2] = 0.0f;
            
            // Turbulenz hinzufügen
            if (physics->environment.wind_turbulence > 0.0f) {
                for (int i = 0; i < 2; i++) {
                    float turbulence = gaussian_noise(physics->environment.wind_turbulence * 
                                                    side_drag_magnitude * 0.1f);
                    physics->forces.crosswind_force[i] += turbulence;
                }
            }
        }
    }
}
```

**Physikalische Bedeutung:**
- **Relative Windgeschwindigkeit**: Berücksichtigt Eigengeschwindigkeit des Fahrrads
- **Seitenfläche**: 0.4 m² für Querwind-Angriffsfläche
- **Turbulenz**: Zufällige Schwankungen der Windkraft
- **Gaussches Rauschen**: Realistische Windböen-Simulation

---

## PID-Regelung

### PID-Controller-Struktur

```c
typedef struct {
    // Historie für D-Term-Berechnung
    float derivative_history[HISTORY_LEN];
    float error_history[HISTORY_LEN];
    long long time_history[HISTORY_LEN];
    int history_counter;
    
    // PID-Terme
    float proportional_term;
    float integral_term;
    float derivative_term;
    
    // Parameter
    float Kp;  // Proportional-Verstärkung
    float Ki;  // Integral-Verstärkung  
    float Kd;  // Differential-Verstärkung
    
    // Begrenzungen
    float output_min, output_max;      // Ausgangsbegrenzung
    float integral_min, integral_max;  // Anti-Windup
} pid_controller_t;
```

### PID-Berechnung

```c
float pid_compute(pid_controller_t *pid, float setpoint, float process_variable, long long current_time_us) {
    // Aktueller Fehler
    float error = setpoint - process_variable;
    pid->error_history[pid->history_counter] = error;
    pid->time_history[pid->history_counter] = current_time_us;
    
    // P-Term (Proportional)
    pid->proportional_term = pid->Kp * error;
    
    // I-Term (Integral) mit Anti-Windup
    int prev_index = (pid->history_counter + HISTORY_LEN - 1) % HISTORY_LEN;
    float dt = (pid->time_history[pid->history_counter] - pid->time_history[prev_index]) / 1000000.0;
    
    if (dt > 0.0 && dt < 1.0) {
        pid->integral_term += pid->Ki * error * dt;
        
        // Anti-Windup: Integral-Term begrenzen
        if (pid->integral_term < pid->integral_min) {
            pid->integral_term = pid->integral_min;
        } else if (pid->integral_term > pid->integral_max) {
            pid->integral_term = pid->integral_max;
        }
    }
    
    // D-Term (Differential) mit gleitendem Durchschnitt
    if (dt > 0.0) {
        float current_derivative = (error - pid->error_history[prev_index]) / dt;
        pid->derivative_history[pid->history_counter] = current_derivative;
        
        // Gleitender Durchschnitt über die Historie
        float derivative_sum = 0.0;
        for (int i = 0; i < HISTORY_LEN; i++) {
            derivative_sum += pid->derivative_history[i];
        }
        pid->derivative_term = pid->Kd * (derivative_sum / HISTORY_LEN);
    }
    
    // Gesamtausgabe
    float output = pid->proportional_term + pid->integral_term + pid->derivative_term;
    
    // Ausgabe begrenzen
    if (output < pid->output_min) {
        output = pid->output_min;
    } else if (output > pid->output_max) {
        output = pid->output_max;
    }
    
    return output;
}
```

**Physikalische Bedeutung:**
- **P-Term**: Proportional zum aktuellen Fehler - sorgt für schnelle Reaktion
- **I-Term**: Integral des Fehlers - eliminiert bleibende Regelabweichung
- **D-Term**: Ableitung des Fehlers - dämpft Schwingungen
- **Anti-Windup**: Verhindert Integrator-Überschwingen
- **Historien-Filter**: Glättet D-Term gegen Sensorrauschen

### Parametrierung

```c
void balance_config_set_defaults(balance_config_t *config) {
    // Angle PID Parameter (basierend auf autobike.c)
    config->angle_pid.Kp = 10.0f;          // Proportionalverstärkung
    config->angle_pid.Ki = 0.0f;            // Integral deaktiviert
    config->angle_pid.Kd = 2.2f;            // Differentialverstärkung
    config->angle_pid.output_min = -0.3f;   // ~-17° Lenkwinkel
    config->angle_pid.output_max = 0.3f;    // ~+17° Lenkwinkel
    config->angle_pid.integral_min = -60.0f;
    config->angle_pid.integral_max = 60.0f;
    
    // Geschwindigkeitsparameter
    config->speed_control.base_speed = 5.0f;        // 5 rad/s Basis
    config->speed_control.min_speed = 3.0f;         // Mindestgeschwindigkeit
    config->speed_control.max_speed = 8.0f;         // Maximalgeschwindigkeit
    config->speed_control.stability_reduction = 0.5f; // 50% Reduktion
}
```

**Physikalische Bedeutung:**
- **Kp = 10**: Starke Proportionalreaktion für schnelle Korrektur
- **Ki = 0**: Integral deaktiviert (verhindert Schwingungen)
- **Kd = 2.2**: Dämpfung gegen Überreaktionen
- **Ausgangsbegrenzung**: ±17° entspricht realistischen Lenkwinkeln
- **Geschwindigkeitsadaption**: Langsamere Fahrt bei Instabilität

---

## Sensorsimulation

### IMU-Sensorfehler

```c
float bicycle_physics_simulate_imu(bicycle_physics_t *physics, float true_roll) {
    // 1. VERZÖGERUNG simulieren (FIFO-Puffer)
    physics->sensor_sim.imu_buffer[physics->sensor_sim.buffer_index] = true_roll;
    physics->sensor_sim.buffer_index = (physics->sensor_sim.buffer_index + 1) % IMU_DELAY_SAMPLES;
    
    float delayed_roll;
    if (!physics->sensor_sim.buffer_filled) {
        delayed_roll = true_roll;
        if (physics->sensor_sim.buffer_index == 0) {
            physics->sensor_sim.buffer_filled = true;
        }
    } else {
        // Hole verzögerten Wert
        delayed_roll = physics->sensor_sim.imu_buffer[physics->sensor_sim.buffer_index];
    }
    
    // 2. RAUSCHEN hinzufügen
    float noise = gaussian_noise(IMU_NOISE_SIGMA);
    float noisy_roll = delayed_roll + noise;
    
    // 3. PLAUSIBILITÄTSPRÜFUNG
    static float last_output = 0.0f;
    static bool first_call = true;
    
    if (!first_call) {
        float max_change = 0.1f; // Maximale Änderung pro Zeitschritt
        float change = noisy_roll - last_output;
        
        if (fabsf(change) > max_change) {
            if (change > max_change) {
                noisy_roll = last_output + max_change;
            } else if (change < -max_change) {
                noisy_roll = last_output - max_change;
            }
        }
    }
    
    last_output = noisy_roll;
    first_call = false;
    
    return noisy_roll;
}
```

**Physikalische Bedeutung:**
- **Verzögerung**: Realistische Sensor-Latenz (10ms bei 2 Samples)
- **Gaussches Rauschen**: Elektronisches Rauschen (σ = 0.01 rad)
- **Plausibilitätsprüfung**: Verhindert unrealistische Sprünge
- **FIFO-Puffer**: Simulates realistic signal processing delay

### Gaussches Rauschen

```c
static float gaussian_noise(float sigma) {
    static int have_spare = 0;
    static float spare;
    
    if (have_spare) {
        have_spare = 0;
        return spare * sigma;
    }
    
    have_spare = 1;
    static float u, v, mag;
    do {
        u = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        v = 2.0f * ((float)rand() / RAND_MAX) - 1.0f;
        mag = u*u + v*v;
    } while (mag >= 1.0f || mag == 0.0f);
    
    mag = sqrtf(-2.0f * logf(mag) / mag);
    spare = v * mag;
    return u * mag * sigma;
}
```

**Physikalische Bedeutung:**
- **Box-Muller-Transformation**: Erzeugt normalverteilte Zufallszahlen
- **Standardabweichung σ**: Charakterisiert Rauschstärke
- **Zweier-Erzeugung**: Algorithmus erzeugt zwei Werte gleichzeitig
- **Realistische Verteilung**: Entspricht echtem Sensorrauschen

---

## Zusammenfassung

### Physikalische Modelle

1. **Fahrradkörper**: Starrkörper mit realistischen Trägheitsmomenten
2. **Reifen-Boden-Kontakt**: Pacejka-Modell für Seitenkräfte
3. **Aerodynamik**: Quadratischer Luftwiderstand + Seitenwind
4. **Gyroskopmommente**: Stabilisierende Effekte rotierender Räder
5. **Sensorfehler**: Verzögerung, Rauschen, Quantisierung

### Regelungsarchitektur

1. **Balance-Controller**: Ultraschnelle Roll-Stabilisierung (500 Hz)
2. **Vision-Controller**: Pfadplanung mit YOLO-Segmentierung (20 Hz)
3. **Zwei-Ebenen-Regelung**: Balance + Vision kombiniert
4. **Adaptive Geschwindigkeit**: Stabilität-basierte Geschwindigkeitsregelung

### Realitätsnähe

- **Physikalische Konstanten**: Realistische Werte für Masse, Trägheit, Reibung
- **Umwelteinflüsse**: Wind, Turbulenz, Straßenneigung
- **Sensorfehler**: Verzögerung, Rauschen, Drift
- **Mechanische Grenzen**: Realistische Lenkwinkel und Geschwindigkeiten

Die Implementierung stellt eine hochrealistische Simulation eines selbstbalancierenden Fahrrads dar, die alle wesentlichen physikalischen Effekte berücksichtigt und durch moderne Regelungstechnik stabilisiert wird.

---

*Ende der detaillierten Physik-Dokumentation* 