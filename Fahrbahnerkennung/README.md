# Selbstfahrendes, selbstbalancierendes Fahrrad mit realistischer Physik

Diese Simulation eines selbstfahrenden, selbstbalancierenden Fahrrads in Webots enthält realistische physikalische Eigenschaften, die das Fahrverhalten beeinflussen.

## Physikalische Eigenschaften

### Fahrrad-Eigenschaften
- **Masse**: Das Fahrrad hat eine realistische Masse von 15 kg, was die Trägheit beeinflusst.
- **Massenträgheitsmoment**: Realistische Beschleunigung und Verzögerung durch Trägheitseffekte.
- **Luftwiderstand**: Berechnet nach der Formel F_drag = 0.5 * ρ * v² * C_d * A, wobei:
  - ρ: Luftdichte (1,225 kg/m³)
  - v: Geschwindigkeit
  - C_d: Luftwiderstandskoeffizient (0,5)
  - A: Frontfläche (0,3 m²)
- **Rollwiderstand**: Reibungswiderstand der Reifen auf dem Untergrund, steigt leicht mit der Geschwindigkeit.
- **Reifenhaftung**: Sicheres Kurvenfahren hängt vom Lenkwinkel und der Haftung ab.
- **Selbstbalancierend**: Das Fahrrad benötigt keinen Fahrer, da es sich selbst ausbalanciert.

### Umgebungseigenschaften
- **Gelände**: Hügel, Steigungen und Gefälle beeinflussen die Fahrleistung.
- **Wettereffekte**: Verschiedene Wetterbedingungen mit unterschiedlichen Auswirkungen:
  - Klar: Optimale Bedingungen
  - Leichter Regen: Verringerte Haftung (80%) und leichter Wind
  - Starker Regen: Stark verringerte Haftung (60%) und stärkerer Wind
  - Windig: Starke Seitenwinde, die die Lenkung beeinflussen

## Steuerung

### Autonomer Modus
- Die Kamera erkennt die Spur und steuert das Fahrrad autonom.
- PID-Regler für präzise Spurhaltung.

### Manuelle Steuerung
- **Pfeiltasten**: Lenkung (links/rechts) und Beschleunigung (hoch/runter)
- **Zifferntasten**:
  - 1: Klares Wetter
  - 2: Leichter Regen
  - 3: Starker Regen
  - 4: Windig

## Realistische Effekte

- **Kurvengeschwindigkeit**: In Kurven wird die Geschwindigkeit automatisch reduziert, um ein Ausrutschen zu vermeiden.
- **Windeffekte**: Seitliche Windkräfte beeinflussen die Lenkung, Gegenwind erhöht den Widerstand.
- **Untergrundneigung**: Bergauf reduziert die Geschwindigkeit, bergab beschleunigt das Fahrrad.
- **Dynamische Wetterbedingungen**: Zufällig variierende Windrichtung und -stärke.

## Anzeigen

Die Simulation zeigt in Echtzeit diverse physikalische Parameter an:
- Geschwindigkeit und Maximalgeschwindigkeit
- Geländeeigenschaften und Haftung
- Luftwiderstand und andere Widerstandskräfte
- Aktuelle Wetterbedingungen und Windeffekte

# Selbstbalancierungsfunktionen

## Selbstbalancierung
- **PID-Balance-Controller**: Hält das Fahrrad durch automatische Lenkbewegungen aufrecht.
- **Schwerpunktmodellierung**: Realistische Modellierung des Schwerpunkts und der Gravitationseffekte.
- **Gyroskopische Effekte**: Bei höheren Geschwindigkeiten wird das Fahrrad stabiler.
- **Zufällige Störungen**: Simulation von Bodenunebenheiten und äußeren Einflüssen, die das Gleichgewicht stören.

## Physikalische Visualisierung
- **Balance-Anzeige**: Echtzeit-Anzeige des Neigungswinkels und der Balancekorrektur.
- **Störungsanzeige**: Aktuelle Stärke zufälliger Störungen wird in Echtzeit angezeigt. 