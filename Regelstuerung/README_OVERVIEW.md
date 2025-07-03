# Regelung in `Regelstuerung`

Dieses Dokument fasst die beiden Regelkreise des Projekts zusammen und zeigt, wie sie über IPC
miteinander kommunizieren. Es werden dabei konkrete Codestellen aus den Controllern erwähnt.

## 1. Balance-Controller (`C`)
Der schnelle Balance-Regler befindet sich in
`controllers/balance_control_c/balance_control_c.c`. Er liest den Roll-Winkel aus dem IMU,
führt eine erweiterte Physik-Simulation durch und regelt den Lenkwinkel per PID.

Wichtige Ausschnitte der Hauptschleife:

```c
/* balance_control_c.c */
140  // 1. Roll-Winkel messen und filtern
141  float true_roll_angle = get_filtered_roll_angle();
...
160  // 5. Vision-Commands empfangen und verarbeiten
161  vision_command_t vision_cmd;
162  int vision_cmd_received = receive_vision_command(&vision_cmd);
164  // 6. Zwei-Ebenen-Regelung: Vision + Balance
165  float final_steer = steering_output;  // Balance-PID-Ausgang
166  float target_speed = config.speed_control.base_speed;
168  if (vision_cmd_received && (wb_robot_get_time() - last_command_time) < 0.5) {
171      float vision_steer = vision_cmd.steer_command * config.mechanical_limits.max_handlebar_angle;
174      final_steer = 0.7f * vision_steer + 0.3f * steering_output;
176      target_speed = config.speed_control.min_speed +
177            vision_cmd.speed_command * (config.speed_control.max_speed -
            config.speed_control.min_speed);
180      printf("VISION: Steer=%.3f, Speed=%.2f | Balance=%.3f -> Final=%.3f\n",
             vision_steer, target_speed, steering_output, final_steer);
```

Ohne Vision-Befehl reduziert der Controller die Geschwindigkeit abhängig vom Stabilitätsfaktor.
Danach werden Motorbefehle gesetzt und der aktuelle Status an den Python-Controller übertragen:

```c
210  // 7. Motoren ansteuern
211  wb_motor_set_position(handlebars_motor, final_steer);
212  wb_motor_set_velocity(wheel_motor, target_speed);
214  // 8. Balance-Status an Vision-Controller senden
215  balance_status_t status = {
216      .roll_angle = roll_angle,
217      .steering_output = final_steer,
218      .current_speed = target_speed,
219      .stability_factor = fabs(steering_output) / config.mechanical_limits.max_handlebar_angle
220  };
221  send_balance_status(&status); // Senden erfolgt alle 50ms
```

Die Funktion `send_balance_status` (ab Zeile 597) verschickt den Status über einen `Emitter`.

## 2. Vision-Controller (`Python`)
Der Vision-Regler befindet sich in
`controllers/vision_control_py/vision_control_py.py`. Er nutzt eine Kamera
(ggf. mit YOLO) zur Wegerkennung und sendet Lenk- und Geschwindigkeitsbefehle an den
Balance-Controller.

Empfang des Balance-Status:

```python
178  def receive_balance_status(self):
180      if self.status_receiver.getQueueLength() > 0:
182          data = self.status_receiver.getData()
183          status = struct.unpack('ffff', data[:16])
186          self.last_balance_status = {
              'roll_angle': status[0],
              'steering_output': status[1],
              'current_speed': status[2],
              'stability_factor': status[3],
              'timestamp': self.robot.getTime()
          }
```

Versand des Vision-Kommandos:

```python
202  def send_vision_command(self, steer_cmd, speed_cmd):
206      steer_cmd = max(-1.0, min(1.0, steer_cmd))
207      speed_cmd = max(0.0, min(1.0, speed_cmd))
210      command_data = struct.pack('ffi', steer_cmd, speed_cmd, 1)
211      self.command_emitter.send(command_data)
```

In der Hauptschleife (ab Zeile 446) wird der Vision-Fehler berechnet und per PID in einen
Lenkbefehl umgewandelt, anschließend wird `send_vision_command` aufgerufen.
`update_display` (ab Zeile 363) blendet Fehler, Steuerbefehle und den empfangenen
Balance-Status im Kamerabild ein.

## 3. Monitoring

Der Balance-Controller legt in `Regelstuerung/Monitoring` CSV-Dateien an.
Eine Datei beginnt mit folgenden Spalten:

```
timestamp,roll_angle,steering_output,target_speed,p_term,i_term,d_term,error,stability_factor
```

Beispielzeilen aus `balance_log_20250702_161315.csv`:

```
0.002000,-0.006,0.000000,4.995,0.000000,0.000000,0.000000,0.000,0.000
27.502000,-0.001,0.000000,4.995,0.000000,0.000000,-0.000000,0.000,0.000
```

Bedeutung der Spalten:
- **timestamp** – Zeit seit Simulationsstart in Sekunden
- **roll_angle** – gefilterter Roll-Winkel des Fahrrads (Grad)
- **steering_output** – berechneter Lenkwinkel (Rad)
- **target_speed** – aktuelle Antriebsgeschwindigkeit (rad/s)
- **p_term**, **i_term**, **d_term** – Beiträge des Balance-PID
- **error** – Regelfehler des PID
- **stability_factor** – Maß für die Instabilität (0 = stabil)

Diese Daten dienen der Analyse und dem Tuning der Regelparameter.
