# Analyse der Regelungssteuerung

Dieses Dokument fasst die wichtigsten Erkenntnisse aus der bestehenden Implementierung des selbstbalancierenden Fahrrads zusammen und definiert eine mögliche Problemstellung für die Masterarbeit.

## Überblick
Das Repository enthält eine umfangreiche Controller-Implementierung in `Regelstuerung/controllers/balance_control_c` und das zugehörige World-File `Regelstuerung/worlds/Normal_World_normallBike copy.wbt`. Die Regelung basiert auf einer PID-Schleife, die über Sensoren und eine erweiterte Physik-Simulation Rückmeldung erhält. Ziel ist es, das Fahrrad in der Webots-Umgebung stabil zu halten und gleichzeitig äußere Einflüsse realistisch abzubilden.

## Controller-Struktur
Der Controller wird in C umgesetzt. Die technische Dokumentation beschreibt einen Regelkreis, der IMU-Daten, ein physikalisches Modell sowie eine adaptive Konfiguration nutzt:

```
[IMU Sensor] → [Roll-Winkel-Filter] → [Angle PID] → [Handlebars Motor]
      ↓                ↑                    ↓              ↓
[Physik-Simulation] ← [Sensor-Simulation]  ↓         [Wheel Motor]
      ↓                                     ↓              ↓
[Umwelt-Effekte] → [Störgrößen] → [Extended Physics] → [Webots-Welt]
                                         ↓
[JSON Config] ← [GUI] → [Live Monitoring] → [CSV Logger]
```
(Quelle: `README_BALANCE_REGELUNG.md` Zeilen 1–18)

Die Hauptregelschleife verarbeitet die Sensordaten, berechnet den PID-Ausgang und setzt Motorbefehle:

```c
while (wb_robot_step(timestep) != -1) {
    // 1. Roll-Winkel messen und filtern
    float true_roll_angle = get_filtered_roll_angle();
    // 2. Erweiterte Physik-Simulation
    float simulated_roll_angle = bicycle_physics_step(&bicycle_physics, true_roll_angle);
    // 3. PID-Regelung
    float steering_output = pid_compute(&angle_pid, 0.0, simulated_roll_angle, current_time);
    // 4. Adaptive Geschwindigkeitsregelung
    float stability_factor = fabs(steering_output) / max_handlebar_angle;
    float target_speed = base_speed * (1.0 - stability_reduction * stability_factor);
    // 5. Motoren ansteuern
    wb_motor_set_position(handlebars_motor, steering_output);
    wb_motor_set_velocity(wheel_motor, target_speed);
}
```
(Quelle: `README_BALANCE_REGELUNG.md` Zeilen 40–60)

Weitere Komponenten sind ein konfigurierbares Logging-System und ein JSON-basiertes Parameter-Management. Der Controller ermöglicht eine Echtzeit-Anpassung der Regelparameter sowie Debug-Ausgaben über die Tastatur.

## Analyse der Weltdatei
Im World-File wird das Fahrrad mit allen physikalischen Eigenschaften und Sensoren definiert. Der Roboterknoten beginnt ab Zeile 48:

```
DEF BICYCLE Robot {
  translation -17.5476 -19.4694 0.326267
  rotation 0.0022 -0.0002 0.9999 3.1414
  children [
    DEF frame Transform {
      scale 0.0064 0.0064 0.0064
      children [
        CadShape {
          url [ "obj/Little Bicycle V2 - Frame/frame.obj" ]
        }
      ]
    }
    ...
```
(Quelle: `Normal_World_normallBike copy.wbt` Zeilen 48–61)

Das Modell enthält mehrere `HingeJoint`‑Knoten für Räder, Lenker und Kurbel, jeweils mit Motoren und passenden `Physics`‑Attributen (Masse, Trägheitsmatrix usw.). Zusätzlich sind Kamera, Display und Inertial Unit eingebettet, sodass der Controller auf diese Sensoren zugreifen kann.

Am Ende des Robot-Knotens ist der Controller `balance_control_c` sowie der Supervisor `vision_control_py` verbunden, wodurch eine Kombination aus Regelung und visueller Spurverfolgung entsteht.

## Problemstellung
Die aktuelle Implementierung bietet bereits eine funktionsfähige Balance-Regelung mit erweiterter Physik. Für die Masterarbeit lässt sich daraus folgende Problemstellung ableiten:

**Wie kann ein selbstbalancierendes Fahrrad in Webots so geregelt werden, dass es auch unter realitätsnahen Störgrößen (Wind, unebener Untergrund, Verzögerungen in Sensoren) stabil fährt?**

* Teilziele:
  * Validierung der vorhandenen Physik-Modelle und ggf. Erweiterung (z. B. Reifenmodell, Windmodelle).
  * Optimierung des PID-Reglers und Untersuchung alternativer Regelansätze (LQR, modellbasierte Verfahren).
  * Analyse der Sensordaten (IMU, Kamera) und Verbesserung der Filterung/Verzögerungskompensation.
  * Systematische Experimente mit unterschiedlichen World-Einstellungen, um Grenzen der Stabilität zu bestimmen.

Diese Problemstellung dient als Ausgangspunkt für weitere Experimente und könnte in der Masterarbeit vertieft werden.

