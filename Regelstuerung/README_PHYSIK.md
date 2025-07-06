# Physikübersicht der "Regelstuerung"

Dieses Dokument fasst die physikalischen Aspekte des Projekts zusammen und zeigt anhand ausgewählter Codeschnipsel, wie die Simulation in Webots umgesetzt wird.

## 1. World-Datei `Little Bicycle V2.wbt`

```wbt
WorldInfo {
  basicTimeStep 2
}
```
Der Zeitschritt von 2 ms ermöglicht eine Regelfrequenz von 500 Hz und damit eine sehr feine Auflösung der Physiksimulation.

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
Über diese Geräte empfängt der Balance‑Controller die Vision‑Kommandos und sendet seinen Status zurück.

## 2. `controllers/balance_control_c/balance_control_c.c`

```c
// Erweiterte Fahrradphysik initialisieren
bicycle_physics_init(&bicycle_physics, robot_node, timestep);
// Beispielhafte Umgebungsparameter (Seitenwind)
bicycle_physics_set_environment(&bicycle_physics, 2.0f, 0.5f, 0.2f);
```
Initialisiert das Physik‑Modul und definiert Windgeschwindigkeit, -richtung und Turbulenz.

```c
// Physik-Simulation durchführen
float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
```
Pro Simulationsschritt werden Kräfte berechnet (Luft-, Seiten- und Rollwiderstand etc.) und ein gefilterter Rollwinkel erzeugt.

```c
// Rollwiderstand beeinflusst die Zielgeschwindigkeit
float roll_resistance_torque = bicycle_physics.forces.rolling_resistance_torque;
float resistance_speed_reduction = fabs(roll_resistance_torque) * 0.1f;
```
Das berechnete Rollwiderstandsmoment reduziert die Vortriebsleistung und simuliert Energieverluste.

## 3. `controllers/balance_control_c/bicycle_physics.c`

```c
// Luftwiderstandskraft F = 0.5 * rho * Cd * A * v²
float drag_magnitude = 0.5f * AIR_DENSITY * DRAG_COEFFICIENT * FRONTAL_AREA * speed * speed;
```
Berechnet den aerodynamischen Widerstand abhängig von Geschwindigkeit und Frontalfläche.

```c
// Pacejka-Modell für seitliche Reifenkräfte
float lateral_force = D * sinf(C * atanf(B * slip_angle));
```
Erzeugt Seitenkräfte basierend auf dem Slip‑Winkel und dem Gewicht des Rades.

```c
// Gyroskopmoment zur Stabilisierung
gyro_torque[0] = -wheel_inertia * wheel_angular_vel * omega_z;
```
Modelliert das stabilisierende Moment der rotierenden Räder.

```c
// Sensor-Simulation mit Verzögerung und Rauschen
physics->sensor_sim.imu_buffer[physics->sensor_sim.buffer_index] = true_roll;
float noise = gaussian_noise(IMU_NOISE_SIGMA);
```
Imu‑Messwerte werden gepuffert und verrauscht, um reale Sensoreffekte nachzubilden.

## 4. `controllers/vision_control_py/vision_control_py.py`

```python
# PID-Regler für die Vision-Lenkung
p_term = self.vision_kp * error
self.vision_integral += error * dt
d_error = (error - self.vision_last_error) / dt
```
Der Python‑Controller berechnet aus der Bildabweichung einen Lenkbefehl, der später mit dem Balance‑Regler verrechnet wird.

```python
# IPC-Übertragung des Vision-Kommandos
command_data = struct.pack('ffifffff',
                           steer_cmd, speed_cmd, 1,
                           self.vision_error, self.vision_p_term,
                           self.vision_i_term, self.vision_d_term,
                           self.vision_mask_coverage)
self.command_emitter.send(command_data)
```
Überträgt Lenk- und Geschwindigkeitsvorgaben inkl. PID‑Terme an den C‑Controller.

## 5. `test_integration.py`

```python
command_data = struct.pack('ffi', steer, speed, 1)
command_emitter.send(command_data)
```
Dieses Skript prüft die Kommunikation der Controller und simuliert einfache Befehle.

---

Diese Übersicht zeigt, wie in den einzelnen Dateien physikalische Effekte modelliert und über IPC zwischen den Regelkreisen ausgetauscht werden.
