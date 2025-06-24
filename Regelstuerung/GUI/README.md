# 🚴‍♂️ Selbstbalancierendes Fahrrad - GUI Kontrollzentrum

Eine moderne tkinter-basierte GUI zur Steuerung und Überwachung eines selbstbalancierenden Fahrrads.

## ✨ Features

- **Reglereinstellungen**: Echtzeit-Anpassung der PID-Parameter über Slider und Textfelder
- **Monitoring**: Visualisierung der Fahrdaten mit interaktiven Diagrammen
- **Datenaufzeichnung**: Automatische CSV-Erstellung bei jeder Fahrt
- **Zurücksetzen**: Systemreset und Datenbereinigung
- **Modernes Design**: Native macOS-Optik mit aqua-Theme

## ⚙️ Installation

### Voraussetzungen

- macOS (getestet auf macOS 14.0+)
- Homebrew

### Automatische Installation

```bash
# In das GUI-Verzeichnis wechseln
cd Regelstuerung/GUI

# Installationsskript ausführbar machen
chmod +x install_dependencies.sh

# Abhängigkeiten installieren
./install_dependencies.sh
```

### Manuelle Installation

```bash
# Homebrew installieren (falls nicht vorhanden)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Python3 und tkinter installieren
brew install python-tk

# Virtuelle Umgebung erstellen
python3 -m venv venv
source venv/bin/activate

# Python-Pakete installieren
pip install -r requirements.txt
```

## 🚀 Verwendung

### GUI starten

```bash
# Virtuelle Umgebung aktivieren
source venv/bin/activate

# GUI starten
python bicycle_controller_gui.py
```

### Tabs Overview

#### 1. Reglereinstellungen
- **PID-Parameter**: Kp, Ki, Kd über Slider oder Textfelder anpassen
- **Geschwindigkeit**: Maximale und minimale Geschwindigkeit konfigurieren
- **Lenkung**: Maximaler Lenkwinkel in Radiant
- **Anzeige**: Kamera-Vorschau aktivieren/deaktivieren

#### 2. Monitoring
- **CSV-Dateien**: Liste aller aufgezeichneten Fahrten
- **Diagramme**: Interaktive Visualisierung der Fahrdaten
  - PID-Reglerausgabe (P, I, D, PID)
  - Fahrdynamik (Geschwindigkeit, Lenkwinkel)
  - Regelfehler über Zeit

#### 3. Zurücksetzen
- **Standardwerte**: Alle Parameter auf Werkseinstellungen zurücksetzen
- **Monitoring-Daten**: Alle CSV-Dateien löschen

## 📊 Datenformat

Die CSV-Dateien enthalten folgende Spalten:
- `timestamp`: Zeitstempel der Messung
- `Kp`, `Ki`, `Kd`: Aktuelle PID-Parameter
- `P`, `I`, `D`, `PID`: Reglerkomponenten und -ausgabe
- `speed`: Geschwindigkeit in km/h
- `handlebar_angle`: Lenkwinkel in Radiant
- `error`: Regelfehler

## 🔧 Konfiguration

Die Konfiguration wird in `controller_config.json` gespeichert:

```json
{
    "Kp": 0.029,
    "Ki": 0.035,
    "Kd": 0.0005,
    "maxS": 20.0,
    "minS": 10.0,
    "hMax": 0.25,
    "preview": 1
}
```

## 📁 Dateien

- `bicycle_controller_gui.py`: Hauptanwendung
- `controller_config.json`: Konfigurationsdatei
- `requirements.txt`: Python-Abhängigkeiten
- `install_dependencies.sh`: Installationsskript
- `../Monitoring/`: Ordner für CSV-Dateien

## 🔧 Entwicklung

### Code-Struktur

```
BicycleControllerGUI/
├── __init__()              # Initialisierung
├── configure_styles()      # Theme-Konfiguration
├── load_config()          # JSON-Konfiguration laden
├── save_config()          # JSON-Konfiguration speichern
├── create_controller_tab() # Reglereinstellungen-Tab
├── create_monitoring_tab() # Monitoring-Tab
├── create_reset_tab()     # Zurücksetzen-Tab
└── auto_save()            # Automatisches Speichern
```

### Anpassungen

- **Neue Parameter**: In `load_config()` und `controller_config.json` hinzufügen
- **Diagramme**: In `plot_selected_csv()` erweitern
- **Themes**: In `configure_styles()` anpassen

## 🛠️ Fehlerbehebung

### Häufige Probleme

1. **Import-Fehler**: Virtuelle Umgebung aktivieren
2. **tkinter nicht verfügbar**: `brew install python-tk`
3. **Matplotlib-Fehler**: Backend-Konfiguration prüfen
4. **CSV-Dateien nicht gefunden**: Monitoring-Ordner existiert

### Logs

```bash
# Python-Logs anzeigen
python -c "import tkinter; print('tkinter verfügbar')"
python -c "import matplotlib; print('matplotlib verfügbar')"
python -c "import pandas; print('pandas verfügbar')"
```

## 📜 Lizenz

MIT License - Siehe LICENSE-Datei für Details.

## 🤝 Beitragen

1. Fork erstellen
2. Feature-Branch erstellen (`git checkout -b feature/amazing-feature`)
3. Änderungen committen (`git commit -m 'Add amazing feature'`)
4. Branch pushen (`git push origin feature/amazing-feature`)
5. Pull Request erstellen

## 👥 Autoren

- **Projekt**: Selbstbalancierendes Fahrrad
- **GUI**: Moderne tkinter-Implementierung
- **Monitoring**: Datenaufzeichnung und -visualisierung 