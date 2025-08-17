## Balance_wind.wbt – Drei Windkanäle und Stabilität

Diese Welt teilt die Strecke in drei Windkanal-Abschnitte mit abwechselnden, entgegengesetzten Windrichtungen. Das Fahrrad (Regler: `balance_control_c_onlyBalance`) ist dafür stabil ausgelegt.

### Abschnitte und Windrichtungen
- Abschnitt 1 (oben): 0.5 m/s in −x-Richtung (`streamVelocity -0.5 0 0`)
- Abschnitt 2 (mitte): 0.5 m/s in +x-Richtung (`streamVelocity 0.5 0 0`)
- Abschnitt 3 (unten): 0.5 m/s in −x-Richtung (`streamVelocity -0.5 0 0`)

Die drei Bereiche entstehen durch `Fluid`-Boxen (je 64 × 21 × 20 m) entlang der y-Achse versetzt: y ≈ 21, 1.86, −19.33.

### Einfache Skizze der Strecke
[ Abschnitt 1 ]  <—— 0.5 m/s (−x)
[ Abschnitt 2 ]  ——> 0.5 m/s (+x)
[ Abschnitt 3 ]  <—— 0.5 m/s (−x)

### Relevante Einstellungen
- Luft: `density 1.225 kg/m³`, `viscosity 1.81e-05 Pa·s`
- Windgeschwindigkeiten: ±0.5 m/s über `streamVelocity`
- Reibung Rad–Boden: `coulombFriction 0.8` (`wheel` ↔ `ground`)
- Aerodynamik: `dragForceCoefficients 1 1 1`, `dragTorqueCoefficients 0.05 0.05 0.05`
