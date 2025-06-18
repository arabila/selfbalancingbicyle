"""little_bicycle_P_V2.1 controller."""

import cv2
import numpy as np
from controller import Supervisor

# Initialisiert den Supervisor, der erweiterte Kontrollfunktionen in der Simulation bietet
robot = Supervisor()

# Holt den Zeitschritt der aktuellen Welt und konvertiert ihn in einen Integer-Wert
timestep = int(robot.getBasicTimeStep())

# Referenz auf den eigenen Roboterknoten in der Simulation
robotNode = robot.getSelf()

# Vorschau-Modus für die Maskenanzeige (1 aktiviert die Vorschau)
preview = 1  # Mask Preview 1

# PID-Regler Konstanten
Kp = 0.01  # Proportionaler Anteil
Ki = 0.02  # Integraler Anteil
Kd = 0.0001  # Differenzieller Anteil

# Initialisierung der PID-Reglervariablen
P = 0
I = 0
D = 0
oldP = 0
PID = 0

# Fahrradgetrieb-Geschwindigkeitseinstellungen
maxS = 5  # Maximale Geschwindigkeit
minS = 3  # Minimale Geschwindigkeit

# Lenkwinkel-Einstellungen
hMax = 0.1920  # Maximale Lenkradstellung in Radiant (entspricht ca. 11°)
hndB = 0  # Startposition des Lenkers (zentriert)
maxV = 0  # Maximale gemessene Geschwindigkeit

# Zugriff auf das Hinterrad-Motorgerät und Konfiguration
whemotor = robot.getDevice('motor::crank')
whemotor.setPosition(float('inf'))  # Setzt den Motor in den Geschwindigkeitsmodus (unendliche Position)
whemotor.setVelocity(maxS)  # Setzt die anfängliche Geschwindigkeit des Hinterrads

# Zugriff auf das Lenk-Motorgerät und Konfiguration
hndmotor = robot.getDevice('handlebars motor')
hndmotor.setPosition(0)  # Setzt den Lenkwinkel auf die Mitte

# Aktiviert die Tastatursteuerung mit dem definierten Zeitschritt
robot.keyboard.enable(timestep)
robot.keyboard = robot.getKeyboard()

# Initialisiert die Kamera und setzt die Bildrate (Viertel des Zeitschritts)
camera = robot.getDevice('camera')
camera.enable(timestep*4)

# Initialisiert das Display und verbindet es mit der Kamera
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)  # Setzt die Zeichenfarbe auf Grün
display.setFont('Verdana', 16, True)  # Setzt die Schriftart und -größe

# Startet den OpenCV-Fenster-Thread, falls die Vorschau aktiviert ist
if preview == 1:
    cv2.startWindowThread()

def getError(act_error):
    """
    Berechnet den Fehler zwischen der gewünschten Position und der erkannten Position im Kamerabild.
    
    Args:
        act_error (float): Der aktuelle Fehlerwert.
    
    Returns:
        float: Der berechnete Fehler basierend auf der Bildverarbeitung.
    """
    error_P = act_error

    # Holt das aktuelle Kamerabild und formatiert es als NumPy-Array
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    
    # Höhe und Breite des Bildes ermitteln
    h = img.shape[0]
    w = img.shape[1]

    # Setzt den Zielpunkt in x und y (z.B. die Bildmitte leicht nach links verschoben)
    xSet = int(w / 2) - 10
    ySet = 205

    # Erstellt eine leere Maske und definiert das Polygon der interessierenden Zone (Straßenbereich)
    mask = np.zeros((h, w), dtype=np.uint8)
    pts = np.array([[[90, 200], [390, 200], [410, 210], [70, 210]]])  # Bereich für das Fahrrad
    cv2.fillPoly(mask, pts, 255)  # Füllt das Polygon in der Maske mit Weiß (255)
    zone = cv2.bitwise_and(img, img, mask=mask)  # Wendet die Maske auf das Bild an
    
    # Konvertiert das Bild von BGR zu HSV-Farbraum
    hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)
    
    # Definiert die unteren und oberen Grenzen für die Farbfilterung im HSV-Raum
    dark_color = np.array([75, 0, 0])
    bght_color = np.array([179, 255, 255])
    Kernel = np.ones((5, 5), np.uint8)  # Kernel für Morphologische Operationen

    # Erstellt eine Binärmaske, die nur die gewünschten Farben enthält
    mask0 = cv2.inRange(hsv, dark_color, bght_color)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, Kernel)  # Schließt kleine Löcher in der Maske
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, Kernel)   # Entfernt kleine Objekte aus der Maske

    # Zeigt die Maske im Vorschaufenster, falls aktiviert
    if preview == 1:
        cv2.imshow("preview", mask0)
        cv2.waitKey(1)

    try:
        # Findet alle Konturen in der binären Maske
        cnts0, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    except:
        # Falls keine Konturen gefunden werden, gibt den aktuellen Fehler zurück
        return error_P
    else:
        try:
            # Wählt die größte Kontur basierend auf der Fläche aus
            largest_contour = max(cnts0, key=cv2.contourArea)
        except:
            # Falls keine Konturen vorhanden sind, gibt den aktuellen Fehler zurück
            return error_P
        else:
            try:
                # Berechnet das umgebende Rechteck der größten Kontur
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = int(x + w / 2)  # Berechnet die x-Koordinate des Mittelpunkts der Kontur
                
                # Aktualisiert das Display mit den Markierungen
                display.setAlpha(0.0)
                display.fillRectangle(0, 0, display.width, display.height)
                display.setAlpha(1.0)
                display.drawLine(center_x - 20, ySet, center_x + 20, ySet)  # Horizontale Linie
                display.drawLine(center_x, ySet - 20, center_x, ySet + 20)  # Vertikale Linie
                display.fillOval(center_x, ySet, 3, 3)  # Zeichnet einen kleinen Kreis am Mittelpunkt
            except:
                # Bei Fehlern während der Zeichnung wird der aktuelle Fehler zurückgegeben
                return error_P
            else:
                # Gibt die Differenz zwischen dem Zielpunkt und dem erkannten Mittelpunkt zurück
                return xSet - center_x 

def keyCtrl():
    """
    Verarbeitet die Tastatureingaben zur manuellen Steuerung des Fahrrads.
    """
    # Holt die aktuell gedrückte Taste
    key = robot.keyboard.getKey()
    
    # Wenn die Pfeiltaste nach oben (315) gedrückt wird, setzt die Geschwindigkeit auf maxS
    if key == 315:
        bcyS = maxS
    elif key < 0:
        # Wenn keine Taste gedrückt ist, verringert sich die Geschwindigkeit um 5%
        bcyS = bcyS * 95 / 100

    # Wenn die Pfeiltaste nach links (314) gedrückt wird, lenkt nach links
    if key == 314:
        hndB = hMax  # Linke maximale Lenkposition
    # Wenn die Pfeiltaste nach rechts (316) gedrückt wird, lenkt nach rechts
    elif key == 316:
        hndB = -hMax  # Rechte maximale Lenkposition
    else:
        hndB = 0  # Lenker zentrieren, wenn keine Pfeiltaste gedrückt ist
    
    # Setzt die Geschwindigkeit des Hinterrads basierend auf der berechneten Geschwindigkeit
    whemotor.setVelocity(bcyS)

    # Setzt die Position des Lenkers basierend auf der berechneten Lenkwinkelposition
    hndmotor.setPosition(hndB)

def hms(sec):
    """
    Konvertiert Sekunden in das Format Stunden:Minuten:Sekunden.

    Args:
        sec (int): Anzahl der Sekunden.

    Returns:
        str: Formatierte Zeit als String.
    """
    h = sec // 3600
    m = sec % 3600 // 60
    s = sec % 3600 % 60
    tm = f'{h:02d}:{m:02d}:{s:02d}'
    return tm

def printStatus():
    """
    Zeigt den aktuellen Status des Fahrrads auf dem Display an, einschließlich Name, Geschwindigkeit und maximale Geschwindigkeit.
    """
    global maxV
    # Holt die aktuelle Geschwindigkeit des Roboters als Vektor
    velo = robotNode.getVelocity()
    
    # Berechnet den Betrag der Geschwindigkeit (Skalierung auf km/h)
    velocity = (velo[0]**2 + velo[1]**2 + velo[2]**2)**0.5
    velocity = velocity * 3.6  # Umrechnung von m/s in km/h
    
    # Aktualisiert die maximale Geschwindigkeit, falls die aktuelle Geschwindigkeit höher ist
    if velocity > maxV:
        maxV = (velocity + maxV) / 2  # Durchschnittswert zur Glättung
    
    # Holt die aktuelle Simulationszeit in Sekunden und formatiert sie
    timer = int(robot.getTime())
    strP = hms(timer)
    
    # Initialisiert die vertikale Position für die Anzeige mit einem Standardwert
    vpos = 0.0
    
    # Bestimmt die Position der Anzeige basierend auf dem Namen des Roboters
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

# Hauptschleife der Simulation
while robot.step(timestep) != -1:

    # Berechnet den aktuellen Fehler basierend auf der Bildverarbeitung
    P = getError(P)
    # Aktualisiert den Integralanteil des PID-Reglers
    I = I * 2 / 3 + P * timestep / 1000
    # Aktualisiert den Differentialanteil des PID-Reglers
    D = D * 0.5 + (P - oldP) / timestep * 1000
    
    # Berechnet den PID-Reglerausgang
    PID = Kp * P + Ki * I + Kd * D
    oldP = P  # Aktualisiert den alten Fehlerwert

    # Berechnet die neue Lenkwinkelposition basierend auf dem PID-Ausgang
    hndB = hMax - abs(PID)
    hndB = hndB + PID
    # Beschränkt den Lenkwinkel auf die maximalen Werte
    if hndB > hMax:
        hndB = hMax
    elif hndB < -hMax:
        hndB = -hMax
    
    # Setzt die Fahrradgeschwindigkeit basierend auf dem PID-Ausgang
    bcyS = maxS
    bcyS = bcyS - abs(PID * 4)  # Reduziert die Geschwindigkeit bei größeren Abweichungen
    if bcyS < minS:
        bcyS = minS  # Stellt sicher, dass die Geschwindigkeit nicht unter den Minimalwert fällt

    # Setzt die Position des Lenkers und die Geschwindigkeit des Hinterrads
    hndmotor.setPosition(hndB)
    whemotor.setVelocity(bcyS)

    # Aktualisiert die Statusanzeige auf dem Display
    printStatus()
    
    pass  # Platzhalter, kann entfernt oder für zusätzliche Logik verwendet werden