# Vision Controller mit MPC-Regelung

## Überblick

Der Vision-Controller wurde von PID-Regelung auf **Model Predictive Control (MPC)** umgestellt. Dies bietet eine vorausschauende, optimale Regelung für die Pfadverfolgung basierend auf YOLO-Straßenerkennung.

## Änderungen

### 🔄 Ersetzt
- **PID-Regelung** → **MPC-Regelung**
- Einfache Proportional-Integral-Differential-Kontrolle → Optimierungsbasierte prädiktive Kontrolle

### ✨ Neue Features
- **Prädiktive Trajektorienplanung**: Berücksichtigt zukünftige Fahrzeugzustände
- **Fahrzeugdynamik-Modell**: Bicycle-Modell für realistische Vorhersagen
- **Optimierungsbasierte Kontrolle**: Minimiert Kosten unter Berücksichtigung von Randbedingungen
- **Intelligenter Fallback**: Automatischer Wechsel zu einfacher Regelung wenn MPC nicht verfügbar

## Architektur

### MPC-Controller (`VisionMPCController`)
```python
Zustandsvektor: [x, y, v, yaw]  # Position, Geschwindigkeit, Orientierung
Eingänge: [accel, steer]        # Beschleunigung, Lenkwinkel
Prädiktionshorizont: 3 Schritte # Echtzeitoptimiert
```

### Kostenmatrizen
- **R**: Eingangskostenmatrix (penalisiert große Kontrolleingänge)
- **Rd**: Eingangsdifferenzkostenmatrix (penalisiert abrupte Änderungen)
- **Q**: Zustandskostenmatrix (penalisiert Abweichungen vom Referenzpfad)
- **Qf**: Terminale Kostenmatrix (Endzustand)

## Installation

### Neue Abhängigkeiten
```bash
pip install cvxpy>=1.3.0
```

### Vollständige Installation
```bash
cd Regelstuerung/controllers/vision_control_py/
pip install -r requirements.txt
```

## Verwendung

### Starten des Controllers
```bash
cd Regelstuerung/controllers/vision_control_py/
python vision_control_py.py
```

### Test der MPC-Funktionalität
```bash
python test_mpc.py
```

## Parameter

### MPC-Parameter
```python
# Fahrzeugmodell
WB = 1.2                    # Radstand [m]
DT = 0.1                    # Zeitschritt [s]

# Begrenzungen
MAX_STEER = 30°             # Maximaler Lenkwinkel
MAX_SPEED = 8.0             # Maximale Geschwindigkeit [m/s]
MAX_ACCEL = 2.0             # Maximale Beschleunigung [m/s²]

# Optimierung
T = 3                       # Prädiktionshorizont
MAX_ITER = 3                # Maximale Iterationen
```

### Kostengewichtung
```python
R = [0.01, 0.01]           # Eingangskostengewichtung
Rd = [0.01, 1.0]           # Eingangsdifferenzkostengewichtung
Q = [1.0, 1.0, 0.5, 0.5]   # Zustandskostengewichtung
```

## Funktionsweise

### 1. Referenzpfad-Generierung
- Basiert auf Vision-Fehler (Querabweichung von Straßenmitte)
- Erzeugt Solltrajectorie mit sanfter Korrektur
- Berücksichtigt aktuelle Geschwindigkeit und Orientierung

### 2. MPC-Optimierung
- Löst Optimierungsproblem mit `cvxpy`
- Minimiert Kosten unter Berücksichtigung von Randbedingungen
- Berücksichtigt Fahrzeugdynamik und physikalische Grenzen

### 3. Fallback-Mechanismus
- Automatischer Wechsel zu einfacher P-Regelung wenn MPC fehlschlägt
- Robuste Behandlung von Optimierungsfehlern
- Kontinuierliche Kontrolle auch bei Problemen

## IPC-Kommunikation

### Datenformat bleibt kompatibel
```c
typedef struct {
    float steer_command;     // [-1.0, 1.0]
    float speed_command;     // [0.0, 1.0]
    int valid;               // 1 = gültig
    float vision_error;      // Querabweichung
    float vision_p_term;     // MPC-Äquivalent zu P-Term
    float vision_i_term;     // 0.0 (nicht verwendet)
    float vision_d_term;     // MPC-Äquivalent zu D-Term
    float mask_coverage;     // Straßenerkennung [%]
} vision_command_t;
```

## Vorteile des MPC-Ansatzes

### 🎯 Verbesserte Regelgüte
- **Prädiktive Kontrolle**: Berücksichtigt zukünftige Auswirkungen
- **Optimale Trajektorien**: Minimiert Kosten global, nicht nur lokal
- **Randbedingungen**: Explizite Behandlung von Geschwindigkeits- und Lenkbegrenzungen

### 🔄 Robustheit
- **Modellbasiert**: Berücksichtigt Fahrzeugdynamik explizit
- **Störungsunterdrückung**: Bessere Handhabung von Sensorrauschen
- **Graceful Degradation**: Fallback bei Optimierungsfehlern

### ⚡ Echtzeitfähigkeit
- **Reduzierter Horizont**: T=3 für schnelle Berechnung
- **Weniger Iterationen**: MAX_ITER=3 für niedrige Latenz
- **Intelligenter Fallback**: Sofortiger Wechsel bei Zeitüberschreitung

## Debugging

### Tastatursteuerung
- **V**: Vision aktivieren/deaktivieren
- **R**: MPC-Controller zurücksetzen
- **ESC**: Controller beenden

### Debug-Ausgaben
```
MPC: Error=0.123 | Steer=0.456 | Speed=0.789 | 
     MPC=[P:1.23 I:0.00 D:4.56] | 
     Balance: Roll=2.3° Stab=0.45
```

## Fehlerbehebung

### Häufige Probleme

#### 1. CVXPY nicht installiert
```
⚠ CVXPY nicht verfügbar - Fallback auf PID-Regelung
```
**Lösung**: `pip install cvxpy>=1.3.0`

#### 2. MPC-Optimierung fehlgeschlagen
```
⚠ MPC-Lösung nicht optimal - Fallback
```
**Lösung**: Parameter überprüfen, Referenzpfad validieren

#### 3. Langsame Performance
```
MPC-Optimierung dauert >50ms
```
**Lösung**: Horizont reduzieren (T=2), Iterationen begrenzen

## Vergleich PID vs. MPC

| Aspekt | PID | MPC |
|--------|-----|-----|
| **Komplexität** | Niedrig | Hoch |
| **Rechenaufwand** | Minimal | Moderat |
| **Regelgüte** | Gut | Optimal |
| **Prädiktivität** | Keine | Ja |
| **Randbedingungen** | Nachgelagert | Explizit |
| **Tuning** | Einfach | Komplex |
| **Robustheit** | Hoch | Sehr hoch |

## Nächste Schritte

### 🔧 Optimierungen
- [ ] Adaptive Kostengewichtung basierend auf Fahrsituation
- [ ] Erweiterte Fahrzeugmodelle (Reifenmodell, Aerodynamik)
- [ ] Online-Parameteridentifikation

### 📊 Monitoring
- [ ] MPC-Performance-Metriken
- [ ] Optimierungszeit-Überwachung
- [ ] Trajektorien-Visualisierung

### 🚀 Erweiterte Features
- [ ] Mehrere Referenzpfade (Spurwechsel)
- [ ] Hindernisumfahrung
- [ ] Geschwindigkeitsprofile

## Lizenz

Gleiche Lizenz wie das Hauptprojekt. 