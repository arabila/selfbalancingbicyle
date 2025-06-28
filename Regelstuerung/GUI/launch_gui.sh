#!/bin/bash

echo "🚴‍♂️ Starte Selbstbalancierendes Fahrrad GUI..."

# Prüfe ob virtuelle Umgebung existiert
if [ ! -d "venv" ]; then
    echo "❌ Virtuelle Umgebung nicht gefunden. Führe zuerst ./install_dependencies.sh aus"
    exit 1
fi

# Aktiviere virtuelle Umgebung und starte GUI
source venv/bin/activate
python balance_controller_gui.py 