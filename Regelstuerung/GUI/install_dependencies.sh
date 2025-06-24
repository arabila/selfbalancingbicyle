#!/bin/bash

echo "🚴‍♂️ Installiere Abhängigkeiten für Selbstbalancierendes Fahrrad GUI..."

# Prüfe ob brew installiert ist
if ! command -v brew &> /dev/null
then
    echo "❌ Homebrew ist nicht installiert. Bitte installiere Homebrew zuerst:"
    echo "   /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
    exit 1
fi

echo "✅ Homebrew gefunden"

# Python3 und tkinter installieren
echo "📦 Installiere Python3 und tkinter..."
brew install python-tk

# Prüfe ob Python3 verfügbar ist
if ! command -v python3 &> /dev/null
then
    echo "❌ Python3 konnte nicht gefunden werden"
    exit 1
fi

echo "✅ Python3 gefunden"

# Virtuelle Umgebung erstellen falls nicht vorhanden
if [ ! -d "venv" ]; then
    echo "🔧 Erstelle virtuelle Umgebung..."
    python3 -m venv venv
fi

# Virtuelle Umgebung aktivieren
echo "🔄 Aktiviere virtuelle Umgebung..."
source venv/bin/activate

# Requirements installieren
echo "📦 Installiere Python-Pakete..."
pip install --upgrade pip
pip install -r requirements.txt

echo "✅ Installation abgeschlossen!"
echo ""
echo "🚀 Starte die GUI mit:"
echo "   source venv/bin/activate"
echo "   python bicycle_controller_gui.py" 