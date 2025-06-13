# Self Balancing Bicycle Simulation

Dieses Repository enthält eine Webots-Simulation eines selbstbalancierenden Fahrrads. Der Controller befindet sich in `controllers/little_bicycle_P_V2/little_bicycle_P_V2.py`, während das World-File unter `worlds/Little Bicycle V2.wbt` liegt.

## Sensorik
- **Kamera**: Auf dem Fahrrad ist eine Kamera montiert (480 × 320 Pixel). Das Skript wertet das Kamerabild aus, um die Spur zu erkennen. Durch Farbsegmentierung und Konturerkennung wird die Abweichung zwischen der Bildmitte und der Spur ermittelt.
- **Tastatur**: Für manuelle Tests kann über die Tastatur direkt auf Geschwindigkeit und Lenkung zugegriffen werden.
- **Geschwindigkeit**: Die momentane Geschwindigkeit wird über `robotNode.getVelocity()` aus dem Simulator abgefragt.

## Aktorik
- **motor::crank**: Rotationsmotor für das Hinterrad, über den die Vortriebs-Geschwindigkeit vorgegeben wird.
- **handlebars motor**: Rotationsmotor für die Lenkung des Vorderrads.
- **Display**: Eingebauter Display-Device zur Darstellung von Hilfslinien und Status.

## Regelung
Das Fahrzeug wird über einen PID-Regler stabilisiert. Die Kamera liefert eine Querabweichung `P`, daraus werden die Größen `I` und `D` gebildet. Der resultierende Korrekturwert (`PID`) beeinflusst sowohl den Lenkwinkel als auch die Geschwindigkeit.

Der wesentliche Regelalgorithmus aus `little_bicycle_P_V2.py`:
```python
P = getError(P)
I = I * 2 / 3 + P * timestep / 1000
D = D * 0.5 + (P - oldP) / timestep * 1000
PID = Kp * P + Ki * I + Kd * D
oldP = P

hndB = hMax - abs(PID)
hndB = hndB + PID
if hndB > hMax: hndB = hMax
elif hndB < -hMax: hndB = -hMax

bcyS = maxS
bcyS = bcyS - abs(PID * 4)
if bcyS < minS: bcyS = minS

hndmotor.setPosition(hndB)
whemotor.setVelocity(bcyS)
```
Die Regelung verringert die Geschwindigkeit bei größeren Abweichungen und stellt den Lenkwinkel entsprechend ein.

## Systemüberblick
Eine vereinfachte Blockdarstellung der Regelstrecke:
```
[ Kamera ] -> (Bildverarbeitung) -> [ PID ] -> [ Lenk- & Antriebsmotoren ] -> Fahrradbewegung ->
      ^                                                                      |
      +----------------------------------------------------------------------+
```

Die Kamera ermittelt die Position relativ zur Spur. Daraus wird ein Fehler berechnet, der über den PID-Regler korrigiert wird. Der Lenkmotor erhält einen Winkelbefehl, der Antriebsmotor die angepasste Geschwindigkeit. Die resultierende Bewegung führt zu einem neuen Kamerabild, womit der Regelkreis geschlossen wird.

## Physikalische Einflüsse
Um ein möglichst realistisches Fahrverhalten zu erhalten, werden im World-File verschiedene physikalische Parameter gesetzt:

- **Gravitation**: Die Welt nutzt eine Erdanziehung von `-9.81 m/s²`.
- **Massenverteilung**: Gesamtmasse ca. 15 kg, die Räder sind mit jeweils 2 kg modelliert.
- **Reibung**: Die Räder verwenden das Kontaktmaterial `rubber`, der Boden `asphalt`. Über `ContactProperties` ist der Reibungskoeffizient auf etwa `0.9` festgelegt.
- **Trägheit**: Durch die Massenangaben wirkt das Fahrrad träger auf Lenk- und Beschleunigungsbefehle, was das Fahrverhalten realitätsnah beeinflusst.

Diese Parameter wirken direkt auf Beschleunigung, Kurvenfahrt und Stabilität und sorgen so für eine glaubwürdige Dynamik.

## Ausführen der Simulation
1. Webots starten und das World-File `Little Bicycle V2.wbt` öffnen.
2. Den Controller `little_bicycle_P_V2` starten. Die Fahrrad-Kamera wird automatisch initialisiert und der Regler übernimmt die Steuerung.


