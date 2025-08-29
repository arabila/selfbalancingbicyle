# Path Visualization System - Implementierungsdokumentation

## Überblick

Das Path Visualization System für das Self-Balancing Bicycle wurde erfolgreich implementiert und bietet automatische Pfad-Visualisierung mit Live-Tracking und Galerie-Funktionalität.

## Implementierte Features

### ✅ 1. Erweiterte Datenerfassung
- **Position & Orientierung**: Erfassung von X, Y, Z-Koordinaten und Euler-Winkeln (Yaw, Pitch, Roll)
- **CSV-Logging**: Erweiterte Spalten in `balance_log_data_t` Struktur
- **Supervisor-Integration**: Nutzung der Webots Supervisor API für Positionsdaten

### ✅ 2. Koordinaten-Mapping System
- **Weltkoordinaten → Pixelkoordinaten**: Präzise Transformation basierend auf Weltgrenzen
- **Konfigurierbare Weltgrenzen**: Anpassbar für verschiedene Webots-Welten
- **Clipping & Normalisierung**: Robuste Behandlung von Randwerten

### ✅ 3. Live Pfad-Visualisierung
- **Echtzeit-Updates**: Kontinuierliche Aktualisierung während der Simulation
- **Hintergrund-Karten**: Integration von Webots-Texturen als Hintergrund
- **Matplotlib-Integration**: Professionelle Visualisierung mit Start/End-Markierungen

### ✅ 4. Automatischer Export
- **PNG-Export**: Hochauflösende Pfad-Bilder (1200x900px) mit Metadaten
- **JSON-Metadaten**: Strukturierte Speicherung von Lauf-Informationen
- **Automatische Benennung**: Zeitstempel-basierte Dateibenennung

### ✅ 5. Galerie-System
- **Übersichtsliste**: Tabellarische Darstellung aller gespeicherten Läufe
- **Sortierung**: Chronologische Sortierung (neueste zuerst)
- **Datei-Management**: Löschen, Öffnen und Organisieren von Läufen

### ✅ 6. GUI-Integration
- **Neuer Tab**: "Fahrtstrecke" im Balance Controller GUI
- **Live-Panel**: Oberer Bereich für Live-Visualisierung
- **Galerie-Panel**: Unterer Bereich für gespeicherte Läufe
- **Intuitive Bedienung**: Buttons für alle Hauptfunktionen

## Architektur

### Datenfluss
```
Webots Simulation → Balance Controller (C) → CSV-Logging → Path Monitor → Live Visualization
                                                      ↓
                                              Automatic Export → Gallery System
```

### Komponenten

#### 1. **Balance Controller (C)**
- `balance_logging.h/c`: Erweiterte Logging-Funktionalität
- `balance_control_c.c`: Position-Erfassung über Supervisor API
- Neue Spalten: `pos_x`, `pos_y`, `pos_z`, `yaw`, `pitch`, `roll_world`

#### 2. **Path Visualizer (Python)**
- `path_visualizer.py`: Kern-Klassen `PathVisualizer` und `PathMonitor`
- Koordinaten-Transformation und Rendering
- Export-Funktionalität für PNG und JSON

#### 3. **GUI Integration**
- `balance_controller_gui.py`: Neuer "Fahrtstrecke" Tab
- Live-Monitoring mit 1-Sekunden-Updates
- Galerie-Management mit TreeView

## Verwendung

### 1. Simulation starten
1. Webots mit S-Kurve.wbt öffnen
2. Balance Controller GUI starten: `./launch_gui.sh`
3. Zum "Fahrtstrecke" Tab wechseln

### 2. Live-Monitoring
1. **"🎯 Monitoring Starten"** klicken
2. Simulation in Webots starten
3. Pfad wird live in der Visualisierung gezeichnet

### 3. Pfad speichern
1. **"💾 Pfad Speichern"** klicken nach Beendigung der Simulation
2. Automatischer Export als PNG + JSON
3. Eintrag erscheint in der Galerie

### 4. Galerie verwalten
- **Doppelklick** auf Eintrag öffnet das Bild
- **"📁 Galerie Öffnen"** öffnet den Ordner im Finder
- **"🗑 Auswahl Löschen"** entfernt gewählte Einträge

## Dateipfade

### Galerie-Verzeichnis
```
Regelstuerung/Monitoring/path_gallery/
├── path_20250829_130410.png      # Pfad-Bild
├── metadata_20250829_130410.json # Metadaten
├── path_20250829_140225.png
└── metadata_20250829_140225.json
```

### CSV-Logs
```
Regelstuerung/Monitoring/
├── balance_log_20250829_124319.csv  # Mit Position-Daten
└── ...
```

## Konfiguration

### Weltgrenzen anpassen
In `path_visualizer.py`, Methode `_get_world_bounds()`:
```python
bounds_map = {
    "S-Kurve": {"min_x": -32, "max_x": 32, "min_y": -50, "max_y": 50},
    "Neue_Welt": {"min_x": -100, "max_x": 100, "min_y": -80, "max_y": 80}
}
```

### Hintergrund-Texturen
In `path_visualizer.py`, Methode `_get_map_image_path()`:
```python
texture_map = {
    "S-Kurve": "../worlds/textures/S-Kurve.png",
    "Neue_Welt": "../worlds/textures/Neue_Welt.png"
}
```

## Startpunkt-Korrektur

### Problem behoben
Das ursprüngliche System markierte den ersten aufgezeichneten Datenpunkt als "Start", was nicht korrekt war, da:
- Die Datenaufzeichnung oft erst nach Simulationsbeginn startet
- Das Fahrrad bereits gefahren sein könnte, bevor Logging aktiviert wird

### Lösung implementiert
- **Tatsächlicher Startpunkt**: Wird aus der Webots .wbt-Datei extrahiert (`translation` Feld)
- **S-Kurve Startposition**: (2.348, -43.730) basierend auf `translation 2.34802 -43.7299 0.378952`
- **Duale Markierung**: 
  - Grüner Punkt (groß): Echter Welt-Startpunkt
  - Gelber Punkt (klein): Aufzeichnungs-Beginn (nur wenn > 1m Unterschied)

### Konfiguration neuer Welten
In `path_visualizer.py`, Methode `_get_world_start_position()`:
```python
start_positions = {
    "S-Kurve": (2.348, -43.730),        # Aus S-Kurve.wbt
    "Neue_Welt": (x_pos, y_pos),        # Aus Neue_Welt.wbt translation
}
```

## Technische Details

### CSV-Format (erweitert)
```csv
timestamp,roll_angle,steering_output,final_steer,target_speed,actual_speed_kmh,actual_handlebar_angle,p_term,i_term,d_term,error,stability_factor,pos_x,pos_y,pos_z,yaw,pitch,roll_world,vision_error,vision_steer_command,vision_speed_command,vision_p_term,vision_i_term,vision_d_term,vision_active,vision_mask_coverage
```

### JSON-Metadaten
```json
{
  "csv_file": "balance_log_20250829_124319.csv",
  "total_points": 1500,
  "duration_seconds": 30.0,
  "world_name": "S-Kurve",
  "start_position": [2.34, -43.73],
  "end_position": [8.12, -15.42],
  "created_at": "2025-08-29T13:04:10.123456",
  "world_bounds": {...},
  "export_timestamp": "2025-08-29T13:04:15.789012"
}
```

### Performance
- **Live-Updates**: 1 Hz (jede Sekunde)
- **CSV-Verarbeitung**: Inkrementell, nur neue Zeilen
- **Speicherverbrauch**: Minimal durch streaming processing
- **Export-Zeit**: ~2-3 Sekunden für 1500 Punkte

## Erweiterungsmöglichkeiten

### Geplante Features
- [ ] **Multi-World-Support**: Automatische Welt-Erkennung
- [ ] **3D-Visualisierung**: Höhenprofil-Darstellung
- [ ] **Vergleichsmodus**: Mehrere Läufe überlagern
- [ ] **Export-Formate**: SVG, PDF-Export
- [ ] **Statistiken**: Geschwindigkeits-, Beschleunigungs-Analyse

### API-Erweiterungen
- [ ] **REST-API**: Remote-Zugriff auf Galerie
- [ ] **WebSocket**: Live-Streaming für Web-Interface
- [ ] **Plugin-System**: Benutzerdefinierte Visualisierungen

## Fehlerbehebung

### Häufige Probleme

1. **"Keine CSV-Dateien gefunden"**
   - Prüfen ob Simulation läuft und Logging aktiviert ist
   - Monitoring-Verzeichnis überprüfen: `Regelstuerung/Monitoring/`

2. **"Pfad wird nicht angezeigt"**
   - CSV-Datei auf `pos_x`, `pos_y` Spalten prüfen
   - Balance Controller neu kompilieren: `make clean && make`

3. **"Hintergrund-Karte fehlt"**
   - Textur-Pfad prüfen: `Regelstuerung/worlds/textures/S-Kurve.png`
   - Welt-Name in PathVisualizer konfigurieren

### Debug-Informationen
- Console-Output beachten für detaillierte Fehlermeldungen
- Log-Dateien in `Regelstuerung/Monitoring/` prüfen
- GUI-Status-Anzeige im "Fahrtstrecke" Tab

## Fazit

Das Path Visualization System ist vollständig implementiert und getestet. Es bietet eine intuitive Benutzeroberfläche für Live-Pfad-Verfolgung und automatische Archivierung aller Fahrten. Das System ist erweiterbar und kann leicht an neue Webots-Welten angepasst werden.

**Status**: ✅ **Vollständig implementiert und funktionsfähig**

---
*Erstellt am: 29. August 2025*  
*Letzte Aktualisierung: 29. August 2025*
