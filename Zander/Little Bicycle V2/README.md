# Little Bicycle V2 - Selbstbalancierendes Fahrrad

## Übersicht

Dieses Projekt implementiert einen C-Controller für ein selbstbalancierendes Fahrrad in der Webots-Simulation. Die Regelung basiert auf der Bachelorarbeit von Jonah Zander (2023) und verwendet einen PID-Regler zur Stabilisierung des Fahrrads.

## Regelungskonzept

### Hauptregelung
- **Eingangsgröße**: Roll-Winkel (Neigungswinkel des Fahrrads)
- **Ausgangsgröße**: Lenkwinkel  
- **Regelziel**: Roll-Winkel = 0° (aufrechte Position)

### PID-Parameter (für niedrige Geschwindigkeit optimiert)
- **Kp = 25.0** - Proportionalverstärkung (erhöht für mehr Reaktivität)
- **Ki = 0.5** - Integralverstärkung (hinzugefügt für Stabilität)
- **Kd = 8.0** - Differentialverstärkung (erhöht für Dämpfung)

### Funktionsweise
1. **Sensordaten**: Roll-Winkel wird über Inertial Unit gemessen
2. **PID-Regelung**: Berechnung des Stellsignals für Lenkung
3. **Aktuierung**: Lenkmotor wird entsprechend angesteuert
4. **Geschwindigkeitsanpassung**: Bei großer Neigung wird Geschwindigkeit reduziert

## Dateien

- `worlds/Little Bicycle V2.wbt` - Webots-Simulationsumgebung
- `controllers/little_bicycle_P_V2/little_bicycle_P_V2.c` - C-Controller
- `controllers/little_bicycle_P_V2/Makefile` - Build-System

## Steuerung

### Automatisch
- Das Fahrrad balanciert sich selbstständig aus
- PID-Regler korrigiert automatisch Neigungen

### Manuell (Tastatur)
- **↑**: Geschwindigkeit erhöhen
- **↓**: Geschwindigkeit verringern  
- **←**: Manuell links lenken
- **→**: Manuell rechts lenken
- **Leertaste**: Zurück zur automatischen Regelung
- **R**: PID-Controller zurücksetzen

## Technische Details

### Sensoren
- **InertialUnit**: Messung von Roll-, Pitch- und Yaw-Winkeln
- **Camera**: Kamerabild für erweiterte Regelungsalgorithmen
- **GPS**: Geschwindigkeitsmessung

### Aktuatoren  
- **Lenkmotor**: Steuerung des Lenkwinkels (±11°)
- **Antriebsmotor**: Geschwindigkeitsregelung (3-6 km/h)

### Regelungsparameter
```c
#define ANGLE_PID_KP 10.0f
#define ANGLE_PID_KI 0.0f  
#define ANGLE_PID_KD 2.2f
#define HANDLEBAR_MAX_ANGLE 0.1920  // 11° in Radiant
#define MAX_SPEED 6.0
#define MIN_SPEED 3.0
```

## Build und Ausführung

1. **Webots öffnen**
2. **World-Datei laden**: `worlds/Little Bicycle V2.wbt`
3. **Simulation starten**

Der C-Controller wird automatisch von Webots kompiliert und gestartet.

### Wichtige Hinweise
- Die ursprüngliche Python-Datei wurde als `little_bicycle_P_V2.py.backup` gesichert
- Der C-Controller wurde erfolgreich kompiliert (`little_bicycle_P_V2` Executable)
- Webots verwendet jetzt automatisch den C-Controller
- Bei Kompilierungsproblemen: `make clean && make` im Controller-Verzeichnis ausführen

### Fehlerbehebungen
✅ **Webots World-Datei**: "scale" Node-Fehler behoben (Transform-Wrapper hinzugefügt)  
✅ **Controller-Kompilierung**: Makefile für macOS/Webots optimiert  
✅ **Header-Pfade**: Korrekte Webots-Include-Pfade konfiguriert  
✅ **Fahrrad-Instabilität**: Realistische Physik-Parameter hinzugefügt (Masse: 15kg, Schwerpunkt: 12cm hoch)  
✅ **Sensor-Integration**: InertialUnit für Roll-Winkel-Messung aktiviert  
✅ **PID-Optimierung**: Parameter für niedrige Geschwindigkeit (2.0 km/h) angepasst

## Ausgabe

Der Controller gibt folgende Informationen aus:
- Roll-Winkel in Grad
- Lenkwinkel in Grad  
- Aktuelle Geschwindigkeit
- PID-Terms (P, I, D)

Beispiel-Ausgabe:
```
Roll: -2.34° | Steering: 5.67° | Speed: 5.2 | PID: P=23.40 I=0.00 D=-1.15
```

## Basiert auf

- **Bachelorarbeit**: Jonah Zander, 2023
- **Original Implementierung**: BeagleBone-basiertes System mit echten Sensoren
- **Simulation**: Webots-Adaptation der Regelungsalgorithmen

## Weiterentwicklung

Mögliche Erweiterungen:
- Bildverarbeitung für Spurverfolgung
- Erweiterte Geschwindigkeitsregelung
- Hinderniserkennung
- Machine Learning-basierte Verbesserungen 