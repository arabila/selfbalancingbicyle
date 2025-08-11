## Fehlerberechnung (error) aus YOLO-Detektionen

Diese Notiz erklärt ausschließlich, wie der laterale Bildfehler `error` berechnet wird.

### Zielgröße

- `error` misst die horizontale Abweichung zwischen der Bildmitte und dem (gemittelten) Mittelpunkt der als „Straße“ erkannten Bounding-Boxen.
- Normierung: dimensionsloser Wert im Bereich ca. [-0.5, 0.5] (abhängig vom Sichtfeld).
- Vorzeichenkonvention: `error > 0` bedeutet, Straße liegt links der Bildmitte; `error < 0` rechts.

### Berechnungsschritte

1) Straßen-Detektionen filtern
   - Wir wählen alle YOLO-Detektionen mit Klassen-ID 2 (Straße):
   ```
street_indices = [i for i, cls in enumerate(r.boxes.cls) if cls == 2]
   ```

2) Box-Mittelpunkte bestimmen
   - Für jede dieser Boxen mit Koordinaten `xyxy = [x1, y1, x2, y2]`:
   ```
x_center = (x1 + x2) / 2.0
   ```
   - Wir sammeln alle `x_center` in einer Liste `x_centers`.

3) Durchschnittlicher Mittelpunkt (avg_x_center)
   - Arithmetisches Mittel der Mittelpunkte:
   ```
avg_x_center = sum(x_centers) / len(x_centers)
   ```
   - Dies entspricht dem horizontalen „Schwerpunkt“ aller erkannten Straßen-Boxen.

4) Bildmitte und Normierung
   - Bildmitte (in Pixeln): `frame_center = width / 2`.
   - Pixelabweichung: `pixel_error = frame_center - avg_x_center`.
   - Normierter Fehler:
   
   \[ error = \frac{\text{frame\_center} - \text{avg\_x\_center}}{\text{width}} \]

### Relevanter Codeauszug

```548:575:Regelstuerung/controllers/vision_control_py/vision_control_py.py
street_indices = [idx for idx, cls in enumerate(r.boxes.cls) if cls == 2]
...
x_center = float((xyxy[0] + xyxy[2]) / 2.0)
...
avg_x_center = sum(x_centers) / len(x_centers)
frame_center = frame.shape[1] / 2
error = (frame_center - avg_x_center) / frame.shape[1]
```

### Edge-Cases

- Keine Straßen-Detektion: Es wird auf die zuletzt gültigen Werte zurückgegriffen und diese ggf. mit einem Abklingfaktor (Decay) reduziert.
- Segmentierungsmaske optional: Für die Fehlerberechnung wird hier nur der Bounding-Box-Schwerpunkt genutzt; Masken dienen u. a. zur Coverage-Anzeige.


