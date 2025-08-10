"""Integrierter Controller für ein selbstfahrendes Fahrrad mit YOLO-Segmentierung."""

import os
import cv2
import torch
from ultralytics import YOLO
import numpy as np
from controller import Supervisor
import matplotlib.pyplot as plt

# ------------------------------------------------------------
# 1) Webots Setup
# ------------------------------------------------------------
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Fahrradmotoren laden und konfigurieren
whemotor = robot.getDevice('motor::crank')
whemotor.setPosition(float('inf'))   # Endlosdrehung
whemotor.setVelocity(0)             # Start mit 0 Umdrehung

hndmotor = robot.getDevice('handlebars motor')
hndmotor.setPosition(0)

# Keyboard-Eingaben (optional, falls du manuell eingreifen willst)
robot.keyboard.enable(timestep)
keyboard = robot.getKeyboard()

# Kamera holen und aktivieren
camera = robot.getDevice('camera')
camera.enable(timestep)

# ------------------------------------------------------------
# 2) YOLO-Modell laden
# ------------------------------------------------------------
# Pfad zu deinen YOLO-Gewichten anpassen
device = "mps" if torch.backends.mps.is_available() else "cpu"
model = YOLO('runs/segment/train/weights/last.pt').to(device)
print(f"Verwendetes Gerät: {device}")

# ------------------------------------------------------------
# 3) Parameter für Fahrradsteuerung
# ------------------------------------------------------------
maxS = 5    # Maximale Geschwindigkeit
minS = 3    # Minimale Geschwindigkeit
hMax = 0.19 # Maximaler Lenkwinkel (Rad)

# PID-Regler Konstanten
Kp, Ki, Kd = 0.01, 0.02, 0.0001

# PID-Zwischenspeicher
P, I, D, oldP = 0, 0, 0, 0

# Maximale gemessene Geschwindigkeit
maxV = 0

def pid_controller(error, dt):
    """Einfache PID-Regelung, die Lenkwinkel & Geschwindigkeit bestimmt."""
    global P, I, D, oldP
    P = error
    I = I * 2/3 + P * dt
    D = D * 0.5 + (P - oldP) / dt
    oldP = P
    return Kp * P + Ki * I + Kd * D

def get_error_from_yolo(frame):
    """
    Wende YOLO-Segmentierung an und finde 'street_main'-Maske, um daraus einen Fehler (z.B. Querabweichung) zu bestimmen.
    """
    # YOLO-Vorhersage auf das Frame
    results = model.predict(
        source=frame, 
        device=device, 
        conf=0.5,   
        max_det=5,  
        show=False, 
        verbose=False
    )
    if not results:
        return 0.0, np.zeros(frame.shape[:2], dtype=np.uint8)

    # Falls mehrere Ergebnisse -> nimm das erste
    r = results[0]

    # Initialisiere leere Maske
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    # Prüfen, ob Segmentierungsdaten vorhanden sind
    if r.masks is not None and r.masks.data is not None:
        for seg in r.masks.data:  # Annahme: YOLO gibt Segmentierungsdaten zurück
            seg = seg.cpu().numpy()  # Konvertiere in NumPy
            # Skalierung der Maske auf die Größe des Frames
            seg_resized = cv2.resize(seg, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
            mask = np.maximum(mask, seg_resized.astype(np.uint8))  # Überlagerung

    # Suche nach Klassen-ID 2 = 'street_main' (ggf. an dein Label anpassen)
    street_indices = [idx for idx, cls in enumerate(r.boxes.cls) if cls == 2]
    if not street_indices:
        # Keine Straße erkannt -> kein Fehler
        return 0.0, mask

    # Berechne Bounding-Box-Mitte
    x_centers = []
    for idx in street_indices:
        xyxy = r.boxes.xyxy[idx]  # [x1, y1, x2, y2]
        x_center = float((xyxy[0] + xyxy[2]) / 2.0)
        x_centers.append(x_center)

    avg_x_center = sum(x_centers) / len(x_centers)
    frame_center = frame.shape[1] / 2
    error = (frame_center - avg_x_center)  # Querabweichung

    return error, mask

# ------------------------------------------------------------
# 4) Zusätzliche Funktionen aus dem Originalcode
# ------------------------------------------------------------

def hms(sec):
    """Konvertiert Sekunden in das Format Stunden:Minuten:Sekunden."""
    h = sec // 3600
    m = sec % 3600 // 60
    s = sec % 3600 % 60
    tm = f'{h:02d}:{m:02d}:{s:02d}'
    return tm

def print_status(velocity):
    """Zeigt den aktuellen Status des Fahrrads auf dem Display an, einschließlich Name, Geschwindigkeit und maximale Geschwindigkeit."""
    global maxV
    if velocity > maxV:
        maxV = (velocity + maxV) / 2  # Durchschnittswert zur Glättung

    timer = int(robot.getTime())
    strP = hms(timer)

    # Initialisiere vpos mit einem Standardwert
    vpos = 0.0

    if robot.getName() == 'Little Bicycle 1':
        vpos = 0.93
        strP = f'Time: {strP:s}'
        robot.setLabel(0, strP, 0, 0.97, 0.06, 0x000000, 0, 'Lucida Console')  # Zeigt die Zeit an
    elif robot.getName() == 'Little Bicycle 2':
        vpos = 0.89
    elif robot.getName() == 'Little Bicycle V2':
        vpos = 0.89  # Kann angepasst werden für verschiedene Fahrradmodelle
    else:
        vpos = 0.85  # Standardwert für unerwartete Namen

    # Erstellt den Status-String mit Name, aktueller und maximaler Geschwindigkeit
    strP = f'Robot: {robot.getName():s}   Speed: {velocity:5.2f} km/h   Max {maxV:5.2f} km/h'
    # Setzt das Label auf dem Display mit dem Status-String
    robot.setLabel(1, strP, 0, vpos, 0.06, 0x000000, 0, 'Lucida Console')   

# ------------------------------------------------------------
# 5) Haupt-Loop
# ------------------------------------------------------------
while robot.step(timestep) != -1:
    # Lese Kamera-Bild
    img_bytes = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    frame = np.frombuffer(img_bytes, dtype=np.uint8).reshape((height, width, 4))

    # RGBA -> BGR konvertieren
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

    # 1) Fehler und Maske aus YOLO-Segmentierung bestimmen
    error, mask = get_error_from_yolo(frame_bgr)

    # Maske in BGR umwandeln, um sie auf das Bild zu legen
    mask_bgr = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
    overlay = cv2.addWeighted(frame_bgr, 0.6, mask_bgr, 0.4, 0)

    # Optional: Anzeige der kombinierten Maske
    cv2.imshow("Kamerabild mit Maske", overlay)
    cv2.waitKey(1)

    # 2) PID-Controller aufrufen -> Lenkwinkel berechnen
    dt = timestep / 1000.0  # Umrechnung von Millisekunden in Sekunden
    pid_value = pid_controller(error, dt)

    # Begrenze Lenkwinkel
    steer_angle = pid_value
    if steer_angle > hMax:
        steer_angle = hMax
    if steer_angle < -hMax:
        steer_angle = -hMax

    # 3) Geschwindigkeit anpassen
    speed = maxS - abs(pid_value) * 4
    if speed < minS:
        speed = minS

    # 4) Werte an Webots-Motoren übergeben
    hndmotor.setPosition(steer_angle)
    whemotor.setVelocity(speed)

    # 5) Geschwindigkeit berechnen und Status aktualisieren
    velo = robot.getSelf().getVelocity()
    velocity = (velo[0]**2 + velo[1]**2 + velo[2]**2)**0.5
    velocity = velocity * 3.6  # Umrechnung von m/s in km/h

    print_status(velocity)

    # Optional: Keyboard-Steuerung (falls du manuell eingreifen möchtest)
    key = keyboard.getKey()
    if key == 315:  # Pfeil nach oben
        whemotor.setVelocity(maxS)
    elif key == 314:  # Pfeil nach links
        hndmotor.setPosition(hMax)
    elif key == 316:  # Pfeil nach rechts
        hndmotor.setPosition(-hMax)
    elif key == 317:  # Pfeil nach unten (optional: Geschwindigkeit reduzieren)
        whemotor.setVelocity(minS)
    # Du kannst weitere Tastencodes hinzufügen, falls nötig

    # Optional: Weitere Steuerlogik oder Debugging-Ausgaben
    # print(f"Error: {error:.2f}, Steer: {steer_angle:.2f}, Speed: {speed:.2f}")

# Ende der Simulation