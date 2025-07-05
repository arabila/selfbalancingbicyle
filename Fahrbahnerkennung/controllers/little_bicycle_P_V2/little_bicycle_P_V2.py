"""little_bicycle_P_V2.1 controller mit realistischen physikalischen Eigenschaften."""

import cv2
import numpy as np
import math
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
maxS = 30  # Maximale Geschwindigkeit
minS = 20  # Minimale Geschwindigkeit für Stabilität

# Lenkwinkel-Einstellungen
hMax = 0.1920  # Maximale Lenkradstellung in Radiant (entspricht ca. 11°)
hndB = 0  # Startposition des Lenkers (zentriert)
maxV = 0  # Maximale gemessene Geschwindigkeit

# Physikalische Eigenschaften des Fahrrads
bicycle_mass = 15.0  # Masse des Fahrrads in kg
total_mass = bicycle_mass  # Gesamtmasse ohne Fahrer

# Luftwiderstand
air_density = 1.225  # Luftdichte in kg/m³
drag_coefficient = 0.5  # Luftwiderstandskoeffizient des Fahrrads (reduziert, da kein Fahrer)
frontal_area = 0.3  # Frontfläche in m² (reduziert, da kein Fahrer)

# Rollwiderstand
rolling_resistance_coefficient = 0.009  # Rollwiderstand auf Asphalt (erhöht für runde Reifen)
g = 9.81  # Erdbeschleunigung in m/s²

# Reifenhaftung
tire_grip_coefficient = 0.75  # Haftungskoeffizient der Reifen (reduziert für runde Reifen)
max_safe_cornering_speed = 0  # Wird dynamisch berechnet

# Massenträgheit für realistische Beschleunigung und Abbremsung
moment_of_inertia = 0.15  # Trägheitsmoment des Rads in kg·m²
acceleration_factor = 0.85  # Faktor für realistische Beschleunigung
deceleration_factor = 0.92  # Faktor für realistische Verzögerung

# Aktuelle Geschwindigkeit und Beschleunigung
current_speed = 0.0  # m/s
current_acceleration = 0.0  # m/s²
last_speed = 0.0
target_speed = 5.0  # Zielgeschwindigkeit (initial auf 5 gesetzt, damit das Fahrrad sofort losfährt)

# Zugriff auf das Hinterrad-Motorgerät und Konfiguration
whemotor = robot.getDevice('motor::crank')
whemotor.setPosition(float('inf'))  # Setzt den Motor in den Geschwindigkeitsmodus (unendliche Position)
whemotor.setVelocity(5.0)  # Startet mit einer initialen Geschwindigkeit von 5

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

# GPS für Positionsbestimmung
gps = None
try:
    gps = robot.getDevice('gps')
    gps.enable(timestep)
except:
    print("GPS nicht verfügbar, Geländeanpassungen werden ohne GPS-Daten berechnet")

# Neigungssensor für die Erkennung des Geländes
inertial_unit = None
try:
    inertial_unit = robot.getDevice('inertial unit')
    inertial_unit.enable(timestep)
except:
    print("Neigungssensor nicht verfügbar, Geländeanpassungen deaktiviert")

# Startet den OpenCV-Fenster-Thread, falls die Vorschau aktiviert ist
if preview == 1:
    cv2.startWindowThread()

# Umgebungseffekte
weather_conditions = {
    'clear': {'wind_strength': 0.0, 'road_grip_modifier': 1.0},
    'light_rain': {'wind_strength': 2.0, 'road_grip_modifier': 0.8},
    'heavy_rain': {'wind_strength': 4.0, 'road_grip_modifier': 0.6},
    'windy': {'wind_strength': 6.0, 'road_grip_modifier': 0.9}
}

# Aktuelle Wetterbedingung (kann zur Laufzeit geändert werden)
current_weather = 'clear'

# Windeffekte
wind_direction = 0.0  # In Radiant (0 = Wind von vorne, PI = Wind von hinten)
wind_variability = 0.2  # Wie stark variiert der Wind
last_wind_change = 0.0  # Letzte Zeit, zu der sich der Wind geändert hat
wind_change_interval = 5.0  # Intervall für Windrichtungs-/stärkenänderungen

# Selbstbalancierungs-Parameter
balance_pid_kp = 2.0      # Proportionaler Faktor für Balanceregler (deutlich erhöht für bessere Reaktion auf runde Reifen)
balance_pid_ki = 0.05     # Integraler Faktor für Balanceregler (erhöht für besseren Ausgleich der dauerhaften Neigung)
balance_pid_kd = 0.8      # Differentieller Faktor für Balanceregler (erhöht für bessere Dämpfung bei rundem Reifenprofil)
balance_error = 0.0       # Aktueller Fehler in der Balance
balance_integral = 0.0    # Integraler Fehler der Balance
balance_derivative = 0.0  # Ableitung des Fehlers
last_balance_error = 0.0  # Vorheriger Balancefehler

# Schwerpunkt-Parameter
center_of_mass_height = 0.6  # Höhe des Schwerpunkts in Metern (erhöht für realistischeren Schwerpunkt)
gravity_effect = 0.3         # Einfluss der Schwerkraft auf das Kippen (verstärkt für runde Reifen)

# Parameter für zufällige Störungen
last_disturbance_time = 0.0  # Zeit der letzten Störung
disturbance_interval = 15.0  # Durchschnittliches Zeitintervall zwischen Störungen
max_disturbance = 0.2        # Maximale Stärke der Störung
current_disturbance = 0.0    # Aktuelle aktive Störung
disturbance_decay = 0.95     # Abklingfaktor der Störung pro Zeitschritt

def calculate_air_resistance(speed):
    """
    Berechnet den Luftwiderstand basierend auf der aktuellen Geschwindigkeit.
    
    Args:
        speed (float): Aktuelle Geschwindigkeit in m/s.
    
    Returns:
        float: Luftwiderstandskraft in Newton.
    """
    # Formel: F_drag = 0.5 * ρ * v² * C_d * A
    return 0.5 * air_density * speed**2 * drag_coefficient * frontal_area

def calculate_rolling_resistance(speed):
    """
    Berechnet den Rollwiderstand basierend auf der aktuellen Geschwindigkeit.
    Angepasst für runde Reifen mit schmalerer Kontaktfläche.
    
    Args:
        speed (float): Aktuelle Geschwindigkeit in m/s.
    
    Returns:
        float: Rollwiderstandskraft in Newton.
    """
    # Formel: F_r = C_r * m * g
    # Leicht erhöht bei höheren Geschwindigkeiten
    speed_factor = 1.0 + 0.015 * speed  # Erhöhung um 1.5% pro m/s (stärker für runde Reifen)
    
    # Zusätzlicher Faktor für die reduzierte Kontaktfläche runder Reifen
    round_tire_factor = 1.15  # 15% höherer Rollwiderstand durch Punktkontakt
    
    return rolling_resistance_coefficient * total_mass * g * speed_factor * round_tire_factor

def calculate_cornering_grip(steering_angle):
    """
    Berechnet die maximale sichere Kurvengeschwindigkeit basierend auf dem Lenkwinkel.
    Berücksichtigt das runde Reifenprofil mit reduzierter Seitenhaftung.
    
    Args:
        steering_angle (float): Aktueller Lenkwinkel in Radiant.
    
    Returns:
        float: Maximale sichere Kurvengeschwindigkeit in m/s.
    """
    # Kleinere Winkel bedeuten größere Kurvenradien und höhere sichere Geschwindigkeiten
    # Vermeidet Division durch Null bei geradem Fahrrad
    if abs(steering_angle) < 0.01:
        return float('inf')
    
    # Schätzt den Kurvenradius basierend auf dem Lenkwinkel
    # Vereinfachte Formel: R ≈ wheelbase / sin(steering_angle)
    wheelbase = 0.9  # Radstand des Fahrrads in Metern
    curve_radius = wheelbase / math.sin(abs(steering_angle) + 0.01)
    
    # Reduzierter Haftungskoeffizient für Kurven mit runden Reifen
    # Runde Reifen haben weniger seitliche Stabilität in Kurven
    cornering_grip_reduction = 0.85  # 15% weniger Kurvenhaftung
    effective_grip = tire_grip_coefficient * cornering_grip_reduction
    
    # Berechnet die maximale Kurvengeschwindigkeit basierend auf Haftung und Zentripetalkraft
    # v = √(μ * g * r), wobei μ der Haftungskoeffizient ist
    max_speed = math.sqrt(effective_grip * g * curve_radius)
    
    return max_speed

def adjust_for_terrain():
    """
    Passt die Fahrradleistung basierend auf dem Gelände an.
    
    Returns:
        float: Anpassungsfaktor für die Motorleistung.
    """
    terrain_factor = 1.0  # Standardwert auf flachem Gelände
    
    # Wenn GPS und Neigungssensor verfügbar sind, Geländeneigung berechnen
    if inertial_unit:
        roll, pitch, _ = inertial_unit.getRollPitchYaw()
        
        # Bergauf (positive Neigung) erhöht den Widerstand
        if pitch > 0.01:
            # Berechnet zusätzlichen Widerstand durch Steigung
            # F_g = m * g * sin(θ)
            grade_resistance = total_mass * g * math.sin(pitch)
            
            # Reduziert die Leistung entsprechend der Steigung
            terrain_factor = 1.0 / (1.0 + grade_resistance / (total_mass * g))
        
        # Bergab (negative Neigung) verringert den Widerstand und erhöht die Geschwindigkeit
        elif pitch < -0.01:
            # Berechnet negative Widerstandskraft (Beschleunigung durch Schwerkraft)
            grade_assistance = total_mass * g * math.sin(-pitch)
            
            # Erhöht die Leistung entsprechend des Gefälles
            terrain_factor = 1.0 + grade_assistance / (total_mass * g)
    
    return terrain_factor

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

def calculate_wind_effects(dt, speed):
    """
    Berechnet die Auswirkungen des Windes auf das Fahrrad.
    
    Args:
        dt (float): Zeitschritt in Sekunden.
        speed (float): Aktuelle Geschwindigkeit in m/s.
    
    Returns:
        tuple: (Zusätzliche seitliche Kraft in Newton, Zusätzlicher Frontwiderstand in Newton)
    """
    global wind_direction, last_wind_change
    
    # Aktuelle Windstärke basierend auf Wetterlage
    wind_strength = weather_conditions[current_weather]['wind_strength']
    
    # Ändert den Wind in zufälligen Intervallen
    current_time = robot.getTime()
    if current_time - last_wind_change > wind_change_interval:
        # Ändert Windrichtung und -stärke leicht
        wind_direction += (np.random.random() - 0.5) * wind_variability
        # Sorgt dafür, dass der Wind im sinnvollen Bereich bleibt
        wind_direction = wind_direction % (2 * math.pi)
        # Aktualisiert die Zeit der letzten Änderung
        last_wind_change = current_time
    
    # Berechnet Komponenten des Windes (Seiten- und Frontwind)
    side_wind = wind_strength * math.sin(wind_direction)
    front_wind = wind_strength * math.cos(wind_direction)
    
    # Seiten- und Frontkraft des Windes
    # Die Auswirkung hängt von der Geschwindigkeit ab - bei höherer Geschwindigkeit ist die Auswirkung größer
    side_force = side_wind * 0.5 * air_density * frontal_area * (1.0 + speed / 5.0)
    front_resistance = front_wind * 0.5 * air_density * frontal_area
    
    # Gibt die berechneten Kräfte zurück
    return side_force, front_resistance

def apply_weather_effects():
    """
    Wendet Wettereffekte auf die Fahreigenschaften an.
    
    Returns:
        float: Modifizierter Haftungskoeffizient basierend auf Wetterbedingungen.
    """
    # Holt den Haftungsmodifikator für die aktuelle Wetterlage
    grip_modifier = weather_conditions[current_weather]['road_grip_modifier']
    
    # Modifiziert den Basis-Haftungskoeffizienten
    modified_grip = tire_grip_coefficient * grip_modifier
    
    return modified_grip

def update_balance(dt):
    """
    Berechnet die Lenkbewegung, die nötig ist, um das Fahrrad zu balancieren.
    Berücksichtigt das runde Reifenprofil, das die Balance schwieriger macht.
    
    Args:
        dt (float): Zeitschritt in Sekunden.
    
    Returns:
        float: Lenkwinkelkorrektur für die Balance.
    """
    global balance_error, balance_integral, balance_derivative, last_balance_error
    
    # Bei einem realen selbstbalancierenden Fahrrad würde hier der Neigungswinkel gemessen
    # In dieser Simulation verwenden wir den Neigungssensor, falls verfügbar
    roll = 0.0
    if inertial_unit:
        roll, pitch, _ = inertial_unit.getRollPitchYaw()
        # Debug-Ausgabe für den Balancestatus
        if robot.getTime() % 5 < 0.1:  # Alle 5 Sekunden ausgeben
            print(f"Balance-Status: Roll={roll:.4f}, Pitch={pitch:.4f}, Balance-Fehler={balance_error:.4f}, Balance-Korrektur={balance_pid_kp * balance_error:.4f}")

    # Berechnung des Effekts der Schwerkraft auf das Kippen
    # Bei höherer Geschwindigkeit ist das Fahrrad stabiler (Gyroskopeffekt)
    speed_stability_factor = 1.0 / (1.0 + current_speed * 0.8)  # Verstärkter Stabilisierungseffekt bei höherer Geschwindigkeit
    
    # Berechne Gravitationseffekt auf den Schwerpunkt mit Berücksichtigung des runden Reifenprofils
    # Ein rundes Reifenprofil führt zu einem verstärkten Kippeffekt
    tire_roundness_factor = 1.5  # Faktor für rundes Reifenprofil (erhöht den Kippeffekt)
    gravity_tilt = math.sin(roll) * gravity_effect * center_of_mass_height * speed_stability_factor * tire_roundness_factor
    
    # Füge den Gravitationseffekt zum Balancefehler hinzu
    balance_error = roll + gravity_tilt
    
    # PID-Regelung für die Balance
    balance_integral += balance_error * dt
    # Verhindert zu großes Anwachsen des Integralterms (Anti-Windup)
    balance_integral = max(-0.7, min(0.7, balance_integral))  # Vergrößerter Bereich für stärkere Korrekturen
    
    # Berechnung der Ableitung des Fehlers
    balance_derivative = (balance_error - last_balance_error) / dt
    last_balance_error = balance_error
    
    # Zusätzlicher Dämpfungsfaktor speziell für runde Reifen
    damping_factor = 1.2  # Verstärkte Dämpfung für rundes Reifenprofil
    
    # PID-Ausgang: Positive Neigung erfordert Lenkung nach links (positiv)
    # und umgekehrt, um das Fahrrad zu balancieren
    balance_correction = (balance_pid_kp * balance_error +
                         balance_pid_ki * balance_integral +
                         balance_pid_kd * balance_derivative * damping_factor)
    
    # Verstärkte Begrenzung der Korrektur für runde Reifen
    max_correction = hMax * 0.7  # Erlaubt stärkere Korrekturen, aber nicht bis zum Maximum
    return max(-max_correction, min(max_correction, balance_correction))

def updatePhysics(dt, target_speed, current_steering):
    """
    Aktualisiert die physikalischen Parameter des Fahrrads basierend auf physikalischen Gesetzen.
    
    Args:
        dt (float): Zeitschritt in Sekunden.
        target_speed (float): Zielgeschwindigkeit in m/s.
        current_steering (float): Aktueller Lenkwinkel in Radiant.
        
    Returns:
        tuple: (neue_geschwindigkeit, neuer_lenkwinkel)
    """
    global current_speed, current_acceleration, last_speed
    
    # Berechnet die aktuellen Widerstandskräfte
    air_resistance = calculate_air_resistance(current_speed)
    rolling_resistance = calculate_rolling_resistance(current_speed)
    
    # Berechnet Wind- und Wettereffekte
    side_wind_force, front_wind_resistance = calculate_wind_effects(dt, current_speed)
    
    # Wendet Wettereffekte auf den Haftungskoeffizienten an
    effective_grip = apply_weather_effects()
    
    # Anpassung der Haftung für runde Reifenprofile
    # Runde Reifen haben weniger Kontaktfläche mit dem Boden und daher geringere Haftung
    tire_roundness_grip_factor = 0.85  # Reduzierter Faktor für runde Reifen (85% der normalen Haftung)
    effective_grip *= tire_roundness_grip_factor
    
    # Berechnet die gesamte Widerstandskraft (inkl. Windwiderstand)
    total_resistance = air_resistance + rolling_resistance + front_wind_resistance
    
    # Passt die Leistung basierend auf dem Gelände an
    terrain_factor = adjust_for_terrain()
    
    # Berechnet die Beschleunigung oder Verzögerung basierend auf der Zielgeschwindigkeit
    if target_speed > current_speed:
        # Beschleunigung mit Trägheit
        pedal_force = 80.0 * terrain_factor  # Pedalkraft in Newton
        acceleration = (pedal_force - total_resistance) / total_mass
        
        # Begrenzt die Beschleunigung auf ein realistisches Maximum
        acceleration = min(acceleration, 2.0)  # max 2 m/s²
        
        # Verzögert die Reaktion durch Massenträgheit
        current_acceleration = acceleration * acceleration_factor
    else:
        # Verzögerung durch Widerstände und Bremsen
        brake_force = 120.0  # Bremskraft in Newton
        deceleration = (total_resistance + brake_force) / total_mass
        
        # Begrenzt die Verzögerung auf ein realistisches Maximum
        deceleration = min(deceleration, 3.0)  # max 3 m/s²
        
        # Verzögert die Reaktion durch Massenträgheit
        current_acceleration = -deceleration * deceleration_factor
    
    # Aktualisiert die Geschwindigkeit basierend auf der Beschleunigung
    new_speed = current_speed + current_acceleration * dt
    
    # Stellt sicher, dass die Geschwindigkeit nicht negativ wird
    new_speed = max(0.0, new_speed)
    
    # Stellt sicher, dass die Minimalgeschwindigkeit eingehalten wird, 
    # außer wenn explizit gestoppt wird (target_speed nahe 0)
    if target_speed > 0.5 and new_speed < minS:
        new_speed = minS
        print(f"Minimalgeschwindigkeit {minS} m/s erzwungen für Stabilität")
    
    # Berechnet die maximale sichere Kurvengeschwindigkeit mit angepasstem Haftungskoeffizient
    max_safe_cornering_speed = calculate_cornering_grip(current_steering)
    max_safe_cornering_speed *= math.sqrt(effective_grip / tire_grip_coefficient)  # Anpassung an Wetterbedingungen
    
    # Wenn die berechnete Geschwindigkeit die sichere Kurvengeschwindigkeit überschreitet,
    # reduziere die Geschwindigkeit
    if new_speed > max_safe_cornering_speed:
        # Reduziert die Geschwindigkeit zur sicheren Kurvengeschwindigkeit
        slip_factor = max_safe_cornering_speed / new_speed
        new_speed = new_speed * (0.5 + 0.5 * slip_factor)  # Sanftere Anpassung
    
    # Berechnet Lenkwinkeländerung durch Seitenwind
    wind_steering_effect = side_wind_force * 0.005  # Skalierungsfaktor für Lenkeffekt
    new_steering = current_steering + wind_steering_effect * dt
    
    # Berechnet die Lenkkorrektur für die Balance
    balance_correction = update_balance(dt)
    
    # Wendet zufällige Störungen an
    disturbance = apply_random_disturbance(dt)
    
    # Wendet die Balancekorrektur und Störungen an
    new_steering = new_steering + balance_correction + disturbance
    
    # Beschränkt den Lenkwinkel auf die maximalen Werte
    if new_steering > hMax:
        new_steering = hMax
    elif new_steering < -hMax:
        new_steering = -hMax
    
    # Speichert die aktuelle Geschwindigkeit für den nächsten Zeitschritt
    last_speed = current_speed
    current_speed = new_speed
    
    # Konvertiert die Geschwindigkeit für den Motor (Annahme: 1 m/s = 10 Einheiten)
    motor_speed = new_speed * 10.0
    
    # Berechnet die aktuelle Gierrate basierend auf Geschwindigkeit und Lenkwinkel
    # Bei höheren Geschwindigkeiten ist der Lenkeffekt sensibler
    steering_sensitivity = 1.0 - min(0.5, new_speed / 15.0)  # Reduziert Sensitivität bei höheren Geschwindigkeiten
    new_steering = new_steering * steering_sensitivity
    
    return motor_speed, new_steering

def keyCtrl():
    """
    Verarbeitet die Tastatureingaben zur manuellen Steuerung des Fahrrads.
    
    Returns:
        tuple: (zielgeschwindigkeit, lenkwinkel)
    """
    global target_speed
    
    # Holt die aktuell gedrückte Taste
    key = robot.keyboard.getKey()
    
    # Standardwerte
    bcyS = target_speed
    hndB = 0
    
    # Wenn die Pfeiltaste nach oben (315) gedrückt wird, erhöht die Zielgeschwindigkeit
    if key == 315:
        bcyS = min(maxS, target_speed + 0.5)  # Erhöht die Geschwindigkeit, begrenzt auf maxS
        print(f"Taste HOCH gedrückt - Zielgeschwindigkeit: {bcyS:.2f}")
    # Wenn die Pfeiltaste nach unten (317) gedrückt wird, verringert die Zielgeschwindigkeit
    elif key == 317:
        # Verhindert, dass die Geschwindigkeit unter das Minimum fällt
        if target_speed > minS + 1.0:
            bcyS = max(minS, target_speed - 1.0)  # Verringert die Geschwindigkeit, nicht unter minS
        else:
            # Wenn wir schon nahe der Minimalgeschwindigkeit sind, stoppen wir komplett
            bcyS = 0
        print(f"Taste RUNTER gedrückt - Zielgeschwindigkeit: {bcyS:.2f}")
    elif key < 0:
        # Wenn keine Taste gedrückt ist, verringert sich die Geschwindigkeit leicht
        bcyS = max(0, target_speed * 0.98)  # Natürliche Verzögerung

    # Wenn die Pfeiltaste nach links (314) gedrückt wird, lenkt nach links
    if key == 314:
        hndB = hMax  # Linke maximale Lenkposition
        print(f"Taste LINKS gedrückt - Lenkwinkel: {hndB:.2f}")
    # Wenn die Pfeiltaste nach rechts (316) gedrückt wird, lenkt nach rechts
    elif key == 316:
        hndB = -hMax  # Rechte maximale Lenkposition
        print(f"Taste RECHTS gedrückt - Lenkwinkel: {hndB:.2f}")
    
    # Zeigt bei Taste 5 den aktuellen Status
    if key == ord('5'):
        print(f"STATUS: Geschwindigkeit={current_speed:.2f} m/s, Ziel={target_speed:.2f}, Lenkwinkel={hndB:.2f}")
        
    # Aktualisiert die Zielgeschwindigkeit
    target_speed = bcyS
    
    return bcyS, hndB

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
    
    # Zeigt physikalische Informationen an
    physics_info = f'Terrain: {adjust_for_terrain():3.2f}   Grip: {apply_weather_effects():3.2f}   Air Res: {calculate_air_resistance(current_speed):3.1f}N'
    robot.setLabel(2, physics_info, 0, vpos - 0.04, 0.05, 0x000000, 0, 'Lucida Console')
    
    # Zeigt Wetterinformationen an
    side_force, front_res = calculate_wind_effects(timestep/1000.0, current_speed)
    weather_info = f'Weather: {current_weather}   Wind: {weather_conditions[current_weather]["wind_strength"]:3.1f} m/s   Side Force: {side_force:3.1f}N'
    robot.setLabel(3, weather_info, 0, vpos - 0.08, 0.05, 0x000000, 0, 'Lucida Console')

    # Zeigt Gleichgewichtsinformationen an
    if inertial_unit:
        roll, _, _ = inertial_unit.getRollPitchYaw()
        balance_info = f'Balance: Roll: {roll:3.2f}   Korrektur: {update_balance(timestep/1000.0):3.2f}   Störung: {current_disturbance:3.2f}'
        robot.setLabel(4, balance_info, 0, vpos - 0.12, 0.05, 0x000000, 0, 'Lucida Console')
        
        # Zusätzliche Anzeige für runde Reifen
        tire_info = f'Runde Reifen: Grip: {tire_grip_coefficient * 0.85:3.2f}   Roll-Wid.: {calculate_rolling_resistance(current_speed):3.1f}N'
        robot.setLabel(5, tire_info, 0, vpos - 0.16, 0.05, 0x000000, 0, 'Lucida Console')

def apply_random_disturbance(dt):
    """
    Erzeugt zufällige Störungen, die das Gleichgewicht des Fahrrads beeinflussen.
    Diese simulieren Bodenunebenheiten, Seitenwind-Böen oder andere unerwartete Einflüsse.
    
    Args:
        dt (float): Zeitschritt in Sekunden.
    
    Returns:
        float: Störeffekt, der auf den Lenkwinkel angewendet werden soll.
    """
    global last_disturbance_time, current_disturbance
    
    current_time = robot.getTime()
    
    # Reduziere die aktuelle Störung mit der Zeit (Abklingen)
    current_disturbance *= disturbance_decay
    
    # Prüft, ob es Zeit für eine neue Störung ist
    if current_time - last_disturbance_time > disturbance_interval:
        # Zufallskomponente: Nicht bei jedem Intervall tritt eine Störung auf
        if np.random.random() < 0.3:  # 30% Chance auf Störung
            # Neue Störung generieren
            current_disturbance = (np.random.random() * 2 - 1) * max_disturbance
            last_disturbance_time = current_time
            
            # Log der Störung
            print(f"Störung aufgetreten! Stärke: {current_disturbance:3.2f}")
    
    return current_disturbance

# Hauptschleife der Simulation
while robot.step(timestep) != -1:
    # In den ersten 10 Sekunden sicherstellen, dass das Fahrrad fährt
    if robot.getTime() < 10.0:
        whemotor.setVelocity(max(5.0, minS))  # Mindestens minS oder 5.0, was immer größer ist
        
    # Berechnet den aktuellen Fehler basierend auf der Bildverarbeitung für autonome Steuerung
    P = getError(P)
    
    # Aktualisiert den Integralanteil des PID-Reglers
    I = I * 2 / 3 + P * timestep / 1000
    
    # Aktualisiert den Differentialanteil des PID-Reglers
    D = D * 0.5 + (P - oldP) / timestep * 1000
    
    # Berechnet den PID-Reglerausgang
    PID = Kp * P + Ki * I + Kd * D
    oldP = P  # Aktualisiert den alten Fehlerwert

    # Berechnet die neue Lenkwinkelposition basierend auf dem PID-Ausgang
    hndB = PID
    
    # Beschränkt den Lenkwinkel auf die maximalen Werte
    if hndB > hMax:
        hndB = hMax
    elif hndB < -hMax:
        hndB = -hMax
    
    # Holt Tastatureingaben für manuelle Steuerung
    target_velocity, keyboard_steering = keyCtrl()
    
    # Wenn Tastatureingaben vorhanden sind, überschreiben sie die autonome Steuerung
    if abs(keyboard_steering) > 0:
        hndB = keyboard_steering
    
    # Zeitschritt in Sekunden für physikalische Berechnungen
    dt = timestep / 1000.0
    
    # Aktualisiert die physikalischen Parameter
    motor_speed, hndB = updatePhysics(dt, target_velocity / 10.0, hndB)
    
    # Setzt die Geschwindigkeit des Hinterrads basierend auf der berechneten Geschwindigkeit
    whemotor.setVelocity(motor_speed)

    # Setzt die Position des Lenkers basierend auf der berechneten Lenkwinkelposition
    hndmotor.setPosition(hndB)
    
    # Zeigt den Status des Fahrrads an
    printStatus()

    # Ändert das Wetter, wenn bestimmte Tasten gedrückt werden
    key = robot.keyboard.getKey()
    if key == ord('1'):  # Taste 1: Klares Wetter
        current_weather = 'clear'
    elif key == ord('2'):  # Taste 2: Leichter Regen
        current_weather = 'light_rain'
    elif key == ord('3'):  # Taste 3: Starker Regen
        current_weather = 'heavy_rain'
    elif key == ord('4'):  # Taste 4: Windig
        current_weather = 'windy'

    # Selbstbalancierungssystem ist immer aktiv
    # Die Balancekorrektur wird später in updatePhysics angewendet