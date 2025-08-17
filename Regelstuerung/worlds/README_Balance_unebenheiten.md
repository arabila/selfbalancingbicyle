## Balance_unebenheiten.wbt – Unebenes Terrain und Stabilität

Dieses Szenario nutzt das `UnevenTerrain`-PROTO, um eine Strecke mit Höhenprofil zu erzeugen. Die Ränder sind flach (für sicheren Ein-/Auslauf), die Mitte weist Unebenheiten (Höhenänderung) auf. Der Balance-Regler ist so parametriert, dass das Fahrrad trotz Bodenunebenheiten stabil bleibt.

### Terrain-Setup (Auszug aus der Welt)
- Größe: 64 × 64 m (`size 64 64 1`, Höhe max. ≈ 1 m)
- Auflösung: 128 × 128 Punkte (`xDimension 128`, `yDimension 128`)
- Ränder: `flatBounds TRUE` (Ränder werden abgeflacht)
- Perlin-Rauschen: `perlinNOctaves 0` (glatte Kuppe ohne Perlinrauschen)
- Zufallssaat: `randomSeed 0` (≤ 0 → zufälliger Seed; bei 0 wird ein Seed zur Laufzeit gewählt)
- Untergrund: Textur `textures/track012.png`, Reibung Rad–Boden `coulombFriction 0.8`
- Regler: `balance_control_c_onlyBalance`

### Bildliche Skizze (Top-Down/Profil vereinfacht)
[ Rand (flach) ]  ->  [ Mitte (Hügel/Uneben) ]  ->  [ Rand (flach) ]








