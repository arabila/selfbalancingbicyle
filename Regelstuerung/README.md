# Regelsteuerung - Projektübersicht

Dieses Verzeichnis enthält die vollständige Implementierung des selbstbalancierenden Fahrrads mit videogestützter Pfadverfolgung in der **Webots-Simulationsumgebung**. Das System basiert auf einer innovativen Zwei-Controller-Architektur, die Echtzeit-Stabilisierung mit Computer Vision kombiniert.

## 📁 Verzeichnisstruktur

### 🎮 **controllers/**
Kernimplementierung der Regelungsalgorithmen
- **`balance_control_c/`**: Hochperformanter C-Controller für Echtzeit-Stabilisierung
  - Erweiterte Fahrradphysik-Simulation mit Luftwiderstand und Windkräften
  - PID-Kaskadenregelung für Roll-Stabilisierung und Lenkungsregelung
  - IPC-Kommunikation für Integration mit Vision-Controller
  - Kontinuierliche Datenaufzeichnung für Analyse und Tuning

- **`balance_control_c_onlyBalance/`**: Vereinfachte Version nur für Balance-Regelung
  - Fokus auf reine Stabilisierung ohne Vision-Integration
  - Ideal für Wind-Störungsszenarien und Grundlagen-Tests

- **`vision_control_py/`**: Python-basierter Computer Vision Controller
  - YOLO-Integration für Pfaderkennung und Objektdetektion
  - Querregelung mit konfigurierbaren P/PD-Reglern
  - Echtzeitfähige Bildverarbeitung (30 FPS)
  - Supervisor-Funktionalität für Koordination beider Controller

### 🌍 **worlds/**
Webots-Simulationswelten für verschiedene Testszenarien
- **`S-Kurve.wbt`**: Hauptsimulation mit kombinierter Balance- und Vision-Regelung
  - Komplexe S-förmige Pfadverläufe für Navigation
  - Supervisor-System mit Vision-Camera
  - IPC-Kommunikation zwischen beiden Controllern
  
- **`Balance_wind.wbt`**: Spezielle Windstörungsszenarien
  - Multiple Luftströmungen mit verschiedenen Geschwindigkeiten
  - Fokus auf reine Balance-Regelung unter Störeinflüssen
  - Vereinfachtes Setup ohne Vision-Controller

- **`obj/`**: 3D-CAD-Modelle aller Fahrradkomponenten
- **`textures/`**: Bodentexturen und Pfadmarkierungen für Vision-System

### 📊 **Monitoring/**
Automatische Datenerfassung und Analyse
- **CSV-Logs**: Kontinuierliche Aufzeichnung aller Regelgrößen
- **Zeitstempel-basierte Daten**: Roll/Pitch/Yaw, PID-Terme, Motorstellwerte
- **Performance-Metriken**: Zykluszeiten, Fehlerstatistiken, Stabilitätsindikatoren
- **Experimentelle Auswertung**: Basis für wissenschaftliche Analyse

### 🖥️ **GUI/**
Echtzeit-Monitoring und Parametertuning
- **Tkinter-Interface**: Benutzerfreundliche grafische Oberfläche
- **Live-Visualisierung**: Sensordaten, Regelgrößen und Systemstatus
- **Parameter-Tuning**: Dynamische Anpassung der PID-Parameter
- **Konfigurationsdateien**: Persistente Speicherung der Einstellungen

### 🔧 **protos/**
Benutzerdefinierte Webots-Komponenten
- **`Supervisor.proto`**: Erweiterte Supervisor-Funktionalität
- **`SandyGround.proto`**: Spezielle Bodeneigenschaften für Tests
- **`UnevenTerrain.proto`**: Unebenes Terrain für Stabilitätstests

## 📋 Dokumentation

### Technische Spezifikationen
- **`README_PHYSIK.md`**: Detaillierte Physik-Implementierung und ODE-Engine
- **`Bicycle_Specifications_README.md`**: Vollständige Fahrrad-Spezifikationen
- **`README_OVERVIEW.md`**: Systemarchitektur und Controller-Integration

### Entwicklungsdokumentation  
- **`DETAILED_COMPARISON_README.md`**: Hardware vs. Simulation Vergleich
- **`README_ZWEI_CONTROLLER_INTEGRATION.md`**: Controller-Integrationsstrategie
- **`VERIFICATION_CHECKLIST.md`**: Qualitätssicherung und Validierung

### Analyse und Vergleiche
- **`README_Regelschleifen.md`**: Detaillierte Regelkreis-Analyse
- **`README_ZWEISTUFIGER_REGLER.md`**: Zweistufige Regelungsarchitektur
- **`DETAILED_COMPARISON_README.md`**: Leistungsvergleiche verschiedener Ansätze

## 🚀 Schnellstart

### 1. Kombinierte Regelung (Standard)
```bash
# Webots öffnen und S-Kurve.wbt laden
./run_balance_controller.sh    # Terminal 1
./run_vision_controller.sh     # Terminal 2
./run_gui.sh                   # Terminal 3 (optional)
```

### 2. Nur Balance-Regelung (Wind-Tests)
```bash
# Webots öffnen und Balance_wind.wbt laden  
./run_balance_controller.sh    # Nur Balance-Controller
```

### 3. Integration testen
```bash
python test_integration.py     # IPC-Kommunikation prüfen
```

## 🎯 Experimentelle Szenarien

| Szenario | World-Datei | Controller | Fokus |
|----------|-------------|------------|-------|
| **S-Kurven Navigation** | S-Kurve.wbt | Balance + Vision | Pfadverfolgung |
| **Windstörungen** | Balance_wind.wbt | Nur Balance | Störungsrobustheit |
| **Kurvenfahrten** | S-Kurve.wbt | Balance + Vision | Querregelung |
| **Höhenprofile** | Varianten | Balance + Vision | Steigungen/Gefälle |

## 📈 Datenauswertung

Alle experimentellen Daten werden automatisch im `Monitoring/`-Verzeichnis gespeichert und können mit Standard-Tools (Python/Matlab) analysiert werden. Die GUI bietet zusätzlich Echtzeit-Visualisierung für Live-Tuning und Debugging.

---

**Hinweis**: Dieses Verzeichnis repräsentiert den aktuellen Stand der Masterarbeit "Auf dem Weg zum autonom fahrenden Fahrrad" und bietet eine vollständige Entwicklungsumgebung für autonome Fahrrad-Navigation.
