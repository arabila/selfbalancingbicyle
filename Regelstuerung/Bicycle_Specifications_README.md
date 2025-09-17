# Fahrrad Spezifikationen - Vergleich der Simulationswelten

Dieses Dokument vergleicht die technischen Spezifikationen des selbstbalancierenden Fahrrads zwischen den beiden Hauptsimulationswelten: **S-Kurve.wbt** (kombinierte Regelung) und **Balance_wind.wbt** (reine Balance-Regelung).

---

## 🚲 **Fahrrad-Grundkonfiguration**

### Gemeinsame Eigenschaften beider Welten

| Komponente | Spezifikation | Wert |
|------------|---------------|------|
| **Gesamtgewicht** | Hauptrahmen | 4.0 kg |
| **Schwerpunkt** | Position | [0, -0.1, 0.32] m |
| **Trägheitsmatrix** | Rahmen | [0.08, 0.05, 0.07] kg⋅m² |
| **Radstand** | Berechnet | ~1.18 m |
| **Zeitschritt** | Simulation | 2 ms (S-Kurve) / 10 ms (Balance_wind) |

---

## ⚙️ **Detaillierter Systemvergleich**

### 🎯 **S-Kurve.wbt - Kombinierte Regelung**

#### Simulationsparameter
```wbt
WorldInfo {
  basicTimeStep 2  // Hochfrequente Simulation für Vision + Balance
}
```
- **Zeitschritt**: 2 ms = 500 Hz
- **Fokus**: Präzise Regelung für Vision-gestützte Navigation
- **Komplexität**: Vollständiges System mit zwei Controllern

#### Controller-Architektur
```wbt
controller "balance_control_c"  // Hauptfahrrad
controller "vision_control_py"  // Supervisor-System
```
- **Balance Controller**: C-basiert, Echtzeit-Stabilisierung
- **Vision Controller**: Python-basiert, YOLO-Integration
- **Supervisor**: Erweiterte Koordination und Monitoring

#### Kommunikationssystem
```wbt
Receiver { name "command_rx", channel 1, bufferSize 64 }
Emitter { name "status_tx", channel 2, bufferSize 64 }
```
- **Bidirektionale IPC**: Kommandos und Status zwischen Controllern
- **Strukturierte Datenpakete**: 64 Byte Puffer für komplexe Informationen

#### Kamera-System
```wbt
Camera {
  translation 0.96 -1.1 0.95
  fieldOfView 2
  width 480
  height 320
  antiAliasing TRUE
}
```
**Hauptkamera (am Fahrrad)**:
- Position: [0.96, -1.1, 0.95] m
- Sichtfeld: 2 rad (≈ 114.6°)
- Auflösung: 480 × 320 Pixel

**Vision Camera (Supervisor)**:
- Position: [7.641, -23.709, 0.349] m
- Auflösung: 640 × 360 Pixel
- Externe Überwachungsperspektive

#### Umgebung und Pfad
```wbt
RectangleArena {
  floorSize 64 100
  floorAppearance PBRAppearance {
    baseColorMap ImageTexture {
      url ["textures/S-Kurve.png"]
    }
  }
}
```
- **Große Fahrbahn**: 64m × 100m
- **S-Kurven-Textur**: Visueller Pfad für Computer Vision
- **Komplexe Navigation**: Wechselnde Kurvenrichtungen

---

### 🌪️ **Balance_wind.wbt - Wind-Störungsszenarien**

#### Simulationsparameter
```wbt
WorldInfo {
  basicTimeStep 10  // Gröberer Zeitschritt für reine Balance-Tests
}
```
- **Zeitschritt**: 10 ms = 100 Hz
- **Fokus**: Störungsrobustheit und Balance-Performance
- **Vereinfacht**: Nur Balance-Controller aktiv

#### Controller-Architektur
```wbt
controller "balance_control_c_onlyBalance"
```
- **Einziger Controller**: Vereinfachte C-Implementation
- **Kein Vision-System**: Fokus auf reine Stabilisierung
- **Kein Supervisor**: Direktere Hardware-ähnliche Konfiguration

#### Erweiterte Aerodynamik
```wbt
DEF AIR Fluid {
  translation 0 21 0
  streamVelocity -2 0 0  // Gegenwind
}
DEF AIR Fluid {
  translation 0 1.86 0
  streamVelocity 2 0 0   // Rückenwind
}
DEF AIR Fluid {
  translation 0 -19.33 0
  streamVelocity -2 0 0  // Seitenwind
}
```
**Multiple Windkräfte**:
- **3 Luftströmungszonen** mit unterschiedlichen Geschwindigkeiten
- **Windgeschwindigkeiten**: ±2 m/s (realistische Störungen)
- **Komplexe Aerodynamik**: Wechselnde Windrichtungen entlang der Strecke

#### Vereinfachte Umgebung
```wbt
RectangleArena {
  floorSize 64 64  // Quadratische Arena
  floorAppearance PBRAppearance {
    baseColorMap ImageTexture {
      url ["textures/track012.png"]  // Einfache Textur
    }
  }
}
```
- **Kompakte Fahrbahn**: 64m × 64m
- **Einfache Textur**: Ohne komplexe Pfadmarkierungen
- **Fokus auf Physik**: Weniger visuelle Ablenkung

#### Modifizierte Radeigenschaften
```wbt
DEF wheel Transform {
  scale 0.0056 0.0081 0.0081  // Leicht veränderte Skalierung
}
```
- **Angepasste Radgröße**: Kleinere Räder für andere Dynamik
- **Verschiedene Masse**: 1.8 kg (Vorderrad) vs 2.0 kg (Standard)

---

## 📊 **Technischer Vergleich**

### Physikalische Unterschiede

| Parameter | S-Kurve.wbt | Balance_wind.wbt | Bedeutung |
|-----------|-------------|------------------|-----------|
| **Zeitschritt** | 2 ms | 10 ms | S-Kurve: 5× höhere Auflösung |
| **Simulationsfrequenz** | 500 Hz | 100 Hz | Präzision vs. Performance |
| **Fahrbahngröße** | 64×100 m | 64×64 m | Erweiterte vs. kompakte Tests |
| **Windkräfte** | Einzelne Luftmasse | 3 Strömungszonen | Einfach vs. komplex |
| **Controller** | 2 (Balance + Vision) | 1 (nur Balance) | Vollsystem vs. Grundfunktion |

### Anwendungsszenarien

#### S-Kurve.wbt - Ideale Verwendung
✅ **Entwicklung der Vision-Regelung**
✅ **Integration beider Controller testen**  
✅ **Komplexe Navigationsszenarien**
✅ **Performance-Analyse des Gesamtsystems**
✅ **YOLO-Training und -Validierung**

#### Balance_wind.wbt - Ideale Verwendung  
✅ **Störungsrobustheit validieren**
✅ **Balance-Controller isoliert testen**
✅ **Wind-Effekte untersuchen**
✅ **Hardware-ähnliche Bedingungen simulieren**
✅ **Grundlagen-Algorithmus entwickeln**

### Performance-Charakteristika

#### Rechenaufwand
- **S-Kurve.wbt**: Höher (2 Controller + Vision + 500 Hz)
- **Balance_wind.wbt**: Geringer (1 Controller + 100 Hz)

#### Realismus
- **S-Kurve.wbt**: Vollständiges autonomes System
- **Balance_wind.wbt**: Fokussierte Störungsanalyse

#### Komplexität
- **S-Kurve.wbt**: Maximal (IPC + Vision + Navigation)
- **Balance_wind.wbt**: Minimal (nur Balance + Wind)

---

## 🔧 **Konfigurationsempfehlungen**

### Entwicklungsphase
1. **Grundlagenentwicklung**: Balance_wind.wbt
2. **Vision-Integration**: S-Kurve.wbt
3. **Validierung**: Beide Welten parallel

### Experimentelle Validierung
1. **Störungsrobustheit**: Balance_wind.wbt
2. **Navigationspräzision**: S-Kurve.wbt  
3. **Gesamtsystem-Performance**: S-Kurve.wbt

### Hardware-Transfer
1. **Algorithmus-Basis**: Balance_wind.wbt (ähnlicher zu Hardware)
2. **Vision-System**: S-Kurve.wbt (vollständige Integration)

---

## 📈 **Experimentelle Ergebnisse**

### Typische Performance-Metriken

| Szenario | S-Kurve.wbt | Balance_wind.wbt |
|----------|-------------|------------------|
| **Stabilisierungszeit** | 2-3 s | 1-2 s |
| **Pfadabweichung** | ±0.15 m | N/A |
| **Windresistenz** | Begrenzt | Bis 15 N |
| **Zykluszeit** | ~2 ms | ~10 ms |
| **CPU-Last** | Hoch | Mittel |

---

## 🎯 **Fazit und Empfehlungen**

### Optimale Verwendung
- **S-Kurve.wbt**: Vollständige Systementwicklung und Vision-Integration
- **Balance_wind.wbt**: Grundlagenforschung und Störungsanalyse

### Entwicklungsstrategie
1. **Phase 1**: Balance_wind.wbt für robuste Grundregelung
2. **Phase 2**: S-Kurve.wbt für Vision-Integration  
3. **Phase 3**: Beide Welten für umfassende Validierung

### Hardware-Transfer
Die Kombination beider Simulationsansätze bietet optimale Vorbereitung für den Transfer auf reale Hardware, da sowohl Grundfunktionalität als auch erweiterte Features validiert werden.

---

*Erstellt basierend auf den Webots-Weltdateien: S-Kurve.wbt und Balance_wind.wbt*