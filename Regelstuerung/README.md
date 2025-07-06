# Projekt "Regelsteuerung" – Dateiübersicht

Diese Simulation implementiert die Regelung eines selbstbalancierenden Fahrrads in **Webots**. Der Ordner `Regelstuerung` enthält alle Dateien für die Zwei-Controller-Architektur: einen schnellen Balance-Controller in C und einen Vision-Controller in Python. Die nachfolgende Tabelle beschreibt die wichtigsten Dateien und Verzeichnisse.

| Datei/Ordner | Zweck in der Simulation |
|--------------|------------------------|
| `DETAILED_COMPARISON_README.md` | Ausführlicher Vergleich zwischen der ursprünglichen Hardware-Regelung (BeagleBone) und der Webots-Implementierung. Hilft bei der Nachvollziehbarkeit der Übertragung. |
| `README_OVERVIEW.md` | Kurze Zusammenfassung der Balance- und Vision-Regelkreise sowie deren Kommunikation über IPC. Enthält Codeauszüge zur Veranschaulichung. |
| `README_ZWEI_CONTROLLER_INTEGRATION.md` | Erläutert die Integration beider Controller und beschreibt die geänderte World-Datei, die IPC-Kanäle sowie neue Datenstrukturen. |
| `VERIFICATION_CHECKLIST.md` | Checkliste der behobenen Probleme und der korrekten Konfiguration (z.B. Kamera, IPC). Dient der Qualitätssicherung. |
| `run_balance_controller.sh` | Startet den Balance-Controller im Unterordner `controllers/balance_control_c`. Wird aus einem Terminal ausgeführt und kompiliert bzw. startet die C-Anwendung. |
| `run_vision_controller.sh` | Startet den Vision-Controller in `controllers/vision_control_py`. Ruft das Python-Skript auf. |
| `run_gui.sh` | Startet die Monitoring-GUI im Ordner `GUI`. Nutzbar für Live-Darstellung der Sensordaten und Anpassung der PID-Parameter. |
| `test_integration.py` | Python-Skript zum Test der IPC-Kommunikation zwischen Balance- und Vision-Controller. Benötigt laufende Webots-Simulation. |
| `controllers/` | Beinhaltet die beiden eigentlichen Controllerprogramme:
  - `balance_control_c/` – C-Quellcode, Makefile, Physik-Simulation und PID-Regelung für den schnellen Balancer.
  - `vision_control_py/` – Python-Code zur Wegerkennung (ggf. mit YOLO) und Übermittlung von Lenk- und Geschwindigkeitsbefehlen. |
| `GUI/` | Tkinter-basierte Desktop-Anwendung zur Parameteranpassung und zum Monitoring. Enthält ein eigenes `README.md`, ein Installationsskript und Konfigurationsdateien. |
| `Monitoring/` | Automatisch erzeugte CSV-Logs des Balance-Controllers. Jede Datei enthält Zeitstempel, Rollwinkel, PID-Terme usw. für die Analyse und das Tuning. |
| `protos/` | Eigene Webots-PROTO-Dateien (z.B. `Supervisor.proto`) zur Definition spezieller Nodes. |
| `worlds/` | Enthält die Simulationswelt `Little Bicycle V2.wbt`, 3D-Modelle (`obj/`) und Texturen (`textures/`). Außerdem gibt es eine Markdown-Datei zur Physik-Analyse. |

## Nutzung
1. **Webots-Projekt öffnen**: `worlds/Little Bicycle V2.wbt` laden und Simulation starten.
2. **Controller starten**: In separaten Terminals `./run_balance_controller.sh` und `./run_vision_controller.sh` ausführen (oder automatisch durch Webots).
3. **Monitoring**: Optional `./run_gui.sh` starten, um Reglerwerte live zu verfolgen.

Die vorhandenen Markdown-Dateien liefern weitere Details zur Implementierung, zu den physikalischen Annahmen und zur Inbetriebnahme. Somit bietet dieser Ordner eine komplette Umgebung, um die Zwei-Controller-Regelung des selbstbalancierenden Fahrrads nachzuvollziehen und anzupassen.
