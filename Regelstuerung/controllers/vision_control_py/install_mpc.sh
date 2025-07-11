#!/bin/bash
# Installation der MPC-Abhängigkeiten für Vision-Controller

set -e

echo "=== Vision Controller MPC-Installation ==="
echo

# Prüfe Python-Version
python_version=$(python3 --version 2>/dev/null || echo "Python not found")
echo "Python Version: $python_version"

if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 nicht gefunden. Bitte installieren Sie Python 3.8 oder höher."
    exit 1
fi

# Prüfe pip
if ! command -v pip3 &> /dev/null; then
    echo "❌ pip3 nicht gefunden. Bitte installieren Sie pip3."
    exit 1
fi

# Aktualisiere pip
echo "🔄 Aktualisiere pip..."
pip3 install --upgrade pip

# Installiere Basis-Abhängigkeiten
echo "📦 Installiere Basis-Abhängigkeiten..."
pip3 install numpy>=1.21.0

# Installiere OpenCV
echo "📦 Installiere OpenCV..."
pip3 install opencv-python>=4.8.0

# Installiere MPC-Abhängigkeiten
echo "📦 Installiere MPC-Abhängigkeiten (cvxpy)..."
pip3 install cvxpy>=1.3.0

# Installiere optionale Abhängigkeiten
echo "📦 Installiere optionale Abhängigkeiten..."
pip3 install scipy>=1.9.0

# Installiere YOLO-Abhängigkeiten (falls gewünscht)
echo "📦 Installiere YOLO-Abhängigkeiten..."
read -p "Möchten Sie YOLO-Abhängigkeiten installieren? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    pip3 install torch>=2.0.0 torchvision>=0.15.0 ultralytics>=8.0.0
    echo "✅ YOLO-Abhängigkeiten installiert."
else
    echo "⏭️ YOLO-Abhängigkeiten übersprungen."
fi

# Installiere alle Abhängigkeiten aus requirements.txt
echo "📦 Installiere alle Abhängigkeiten aus requirements.txt..."
pip3 install -r requirements.txt

# Teste die Installation
echo "🧪 Teste die Installation..."
python3 -c "
import numpy as np
import cv2
import cvxpy
print('✅ NumPy:', np.__version__)
print('✅ OpenCV:', cv2.__version__)
print('✅ CVXPY:', cvxpy.__version__)

try:
    import torch
    print('✅ PyTorch:', torch.__version__)
except ImportError:
    print('⚠️ PyTorch nicht installiert (optional)')

try:
    import ultralytics
    print('✅ Ultralytics verfügbar')
except ImportError:
    print('⚠️ Ultralytics nicht installiert (optional)')
"

# Teste MPC-Controller
echo "🧪 Teste MPC-Controller..."
if python3 test_mpc.py; then
    echo "✅ MPC-Controller-Test erfolgreich!"
else
    echo "⚠️ MPC-Controller-Test fehlgeschlagen. Überprüfen Sie die Installation."
fi

echo
echo "=== Installation abgeschlossen ==="
echo
echo "🚀 Sie können den Vision-Controller jetzt starten:"
echo "   python3 vision_control_py.py"
echo
echo "🧪 Oder die Tests ausführen:"
echo "   python3 test_mpc.py"
echo
echo "📖 Weitere Informationen finden Sie in README_MPC.md" 