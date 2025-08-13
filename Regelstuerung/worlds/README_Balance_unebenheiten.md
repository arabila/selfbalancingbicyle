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

### Relevante physikalische Parameter
- Aerodynamik (am Fahrrad/Teilen): `dragForceCoefficients 1 1 1`, `dragTorqueCoefficients 0.05 0.05 0.05`
- Fahrrad: Gesamtmasse 14 kg, Schwerpunkt z ≈ 0.32 m

### Anpassung der Schwierigkeit
- Mehr Unebenheit/rauer: `perlinNOctaves` auf 2–4 erhöhen (fügt Perlin-Rauschen hinzu)
- Höhere Amplitude: `size`-Z-Komponente erhöhen (z. B. `size 64 64 2` ≈ 2 m Höhe)
- Ränder nicht abflachen: `flatBounds FALSE`
- Reproduzierbar: `randomSeed` > 0 setzen (z. B. `randomSeed 42`)

### Nutzung
- Öffne `Regelstuerung/worlds/Balance_unebenheiten.wbt` in Webots und simuliere.
- Das Fahrrad durchläuft flache Randzonen und die unebene Mittelzone; der Regler zeigt die Stabilität auf wechselndem Untergrund.

