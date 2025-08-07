#!/bin/bash
# Aktivierungsskript für die virtuelle Umgebung des Vision Controllers
# 
# Verwendung: source activate_venv.sh

echo "=== Vision Controller Virtuelle Umgebung ==="
echo "Aktiviere virtuelle Umgebung..."

# Aktiviere die virtuelle Umgebung
source venv/bin/activate

echo "✅ Virtuelle Umgebung aktiviert!"
echo "✅ PyTorch $(python -c 'import torch; print(torch.__version__)')"
echo "✅ torchvision $(python -c 'import torchvision; print(torchvision.__version__)')"
echo
echo "Jetzt können Sie den Vision Controller ohne Kompatibilitätswarnungen ausführen."
echo "Zum Deaktivieren der Umgebung verwenden Sie: deactivate"