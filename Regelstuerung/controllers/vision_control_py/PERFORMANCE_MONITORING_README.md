# Performance Monitoring - Rechenzeit & Echtzeitfähigkeit

## Überblick

Das Performance-Monitoring-System wurde in den Vision Controller integriert, um detaillierte Messungen der Rechenzeiten verschiedener Komponenten zu ermöglichen. Dies ist besonders wichtig für die Analyse der Echtzeitfähigkeit in Webots im Vergleich zum realen Fahrrad.

## Gemessene Komponenten

### 1. YOLO-Berechnung (`yolo_computation`)
- **Was wird gemessen**: Zeit für YOLO-Inferenz inkl. Vorverarbeitung
- **Typische Werte**: 20-100ms je nach Hardware und Modellgröße
- **Bedeutung**: Hauptflaschenhals bei Vision-basierter Steuerung

### 2. MPC-Controller (`mpc_computation`)
- **Was wird gemessen**: Zeit für Model Predictive Control Optimierung
- **Typische Werte**: 5-50ms je nach Horizont und Komplexität
- **Bedeutung**: Kritisch für Echtzeitfähigkeit der Regelung

### 3. Fallback-Vision (`fallback_vision`)
- **Was wird gemessen**: HSV-Filterung, Morphologie und Konturerkennung
- **Typische Werte**: 1-10ms
- **Bedeutung**: Backup-System wenn YOLO nicht verfügbar

### 4. Kamera-Verarbeitung (`camera_processing`)
- **Was wird gemessen**: Bildabruf und Konvertierung von Webots
- **Typische Werte**: 1-5ms
- **Bedeutung**: Webots-spezifischer Overhead

### 5. Display-Update (`display_update`)
- **Was wird gemessen**: OpenCV-Anzeige und Overlay-Rendering
- **Typische Werte**: 1-10ms
- **Bedeutung**: Visualisierung (kann bei Bedarf deaktiviert werden)

### 6. Physik-Engine (`physics_step`)
- **Was wird gemessen**: Webots-Simulationsschritt
- **Typische Werte**: 2-10ms je nach Komplexität
- **Bedeutung**: Simulationsoverhead

### 7. Gesamtschleife (`total_loop`)
- **Was wird gemessen**: Kompletter Steuerungszyklus
- **Typische Werte**: 50-200ms
- **Bedeutung**: Gesamtperformance des Systems

## Hardware-Informationen

Das System sammelt automatisch folgende Hardware-Informationen:

- **CPU**: Modell, Anzahl Kerne/Threads, Taktfrequenz
- **GPU**: Typ (CUDA/MPS/CPU), falls YOLO verwendet wird
- **RAM**: Gesamtspeicher und Auslastung
- **OS**: Betriebssystem und Version

## Echtzeitfähigkeit

### Echtzeitfaktor-Berechnung
```
Echtzeitfaktor = Simulationszeit / Reale Zeit
```

- **≥ 1.0**: Simulation läuft in Echtzeit oder schneller ✓
- **0.8-0.95**: Simulation läuft etwas langsamer ⚠
- **< 0.8**: Simulation läuft deutlich langsamer ✗

### Zielwerte für Echtzeitfähigkeit
- **Vision Controller**: 20 Hz (50ms Zykluszeit)
- **Balance Controller**: 500 Hz (2ms Zykluszeit)
- **Gesamtsystem**: Mindestens 0.95x Echtzeitfaktor

## Ausgabe-Beispiel

```
================================================================================
PERFORMANCE MONITOR - RECHENZEIT & ECHTZEITFÄHIGKEIT
================================================================================
Hardware: Apple M1 Pro
CPU: 8 Kerne, 8 Threads, 3.2 GHz
GPU: Apple Metal Performance Shaders
RAM: 16.0 GB
OS: Darwin 21.6.0

ECHTZEITFÄHIGKEIT:
Simulation Zeit: 120.5s
Reale Zeit: 125.2s
Echtzeitfaktor: 0.96x ✓

KOMPONENTEN-TIMING (Mittelwerte der letzten 100 Messungen):
--------------------------------------------------------------------------------
Komponente           Anzahl   Mittel     Min        Max        Std       
--------------------------------------------------------------------------------
yolo_computation     45       42.15      38.20      65.30      4.25      
mpc_computation      45       18.50      15.10      25.80      2.10      
fallback_vision      0        0.00       0.00       0.00       0.00      
camera_processing    45       2.30       1.80       3.50       0.40      
display_update       45       5.20       3.10       8.90       1.20      
physics_step         0        0.00       0.00       0.00       0.00      
total_loop           45       68.15      58.20      103.50     7.85      

SYSTEM-AUSLASTUNG:
CPU: 25.3%
RAM: 45.2%
================================================================================
```

## JSON-Report

Zusätzlich zur Konsolen-Ausgabe wird ein detaillierter JSON-Report gespeichert:

```json
{
  "timestamp": "2025-09-13T17:30:45.123456",
  "hardware_info": {
    "cpu_model": "Apple M1 Pro",
    "cpu_cores": 8,
    "cpu_threads": 8,
    "cpu_freq": 3.2,
    "memory_total": 16.0,
    "gpu_info": "Apple Metal Performance Shaders",
    "os": "Darwin 21.6.0"
  },
  "simulation_info": {
    "total_steps": 2400,
    "simulation_duration": 125.2,
    "realtime_factor": 0.96
  },
  "timing_statistics": {
    "yolo_computation": {
      "count": 45,
      "avg": 42.15,
      "min": 38.20,
      "max": 65.30,
      "std": 4.25
    }
  }
}
```

## Integration

Das Performance-Monitoring ist automatisch in den Vision Controller integriert:

1. **Automatischer Start**: Beginnt mit der ersten Simulation
2. **Kontinuierliche Messung**: Alle Komponenten werden gemessen
3. **Regelmäßige Reports**: Alle 100 Simulationsschritte
4. **Automatisches Speichern**: JSON-Report beim Beenden

## Konfiguration

- **Report-Intervall**: Standardmäßig alle 100 Steps (änderbar in `PerformanceMonitor.__init__()`)
- **Speicher-Limit**: Letzte 1000 Messungen pro Komponente
- **Datei-Output**: Automatisch mit Zeitstempel

## Verwendung für Thesis

Die gesammelten Daten können direkt für die Thesis-Tabellen verwendet werden:

### Hardware-Spezifikation Tabelle
| Komponente | Spezifikation |
|------------|---------------|
| CPU | {cpu_model} |
| Kerne/Threads | {cpu_cores}/{cpu_threads} |
| Taktfrequenz | {cpu_freq} GHz |
| GPU | {gpu_info} |
| RAM | {memory_total} GB |
| OS | {os} |

### Rechenzeit Tabelle
| Komponente | Mittelwert (ms) | Min (ms) | Max (ms) | Std (ms) |
|------------|-----------------|----------|----------|----------|
| YOLO | {yolo_avg} | {yolo_min} | {yolo_max} | {yolo_std} |
| MPC | {mpc_avg} | {mpc_min} | {mpc_max} | {mpc_std} |
| Vision Fallback | {fallback_avg} | {fallback_min} | {fallback_max} | {fallback_std} |
| Gesamtschleife | {total_avg} | {total_min} | {total_max} | {total_std} |

### Echtzeitfähigkeit Tabelle
| Simulation Zeit (s) | Reale Zeit (s) | Echtzeitfaktor | Status |
|---------------------|----------------|----------------|---------|
| {sim_time} | {real_time} | {rt_factor}x | {status} |

## Troubleshooting

### Häufige Probleme

1. **Import-Fehler psutil**: `pip install psutil>=5.9.0`
2. **Langsame Performance**: GPU-Beschleunigung prüfen
3. **Hohe CPU-Last**: Display-Updates reduzieren
4. **Speicher-Probleme**: Report-Intervall vergrößern

### Performance-Optimierung

1. **YOLO**: Kleineres Modell verwenden
2. **MPC**: Horizont reduzieren (T=2 statt T=3)
3. **Display**: OpenCV-Fenster schließen
4. **Vision**: Bildauflösung reduzieren
