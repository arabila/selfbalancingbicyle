#!/bin/bash
# Dieses Skript durchsucht rekursiv das aktuelle Verzeichnis
# und gibt für jede Datei den Pfad und den Inhalt aus.
# Anschließend wird die Ausgabe in die Zwischenablage kopiert.

# Zwischenspeicher für die gesamte Ausgabe
output=""

# Rekursive Durchsuchung aller Dateien
while IFS= read -r -d '' file; do
    output+="Dateipfad: ${file}\n"
    output+="--------------------\n"
    output+="$(cat "$file")\n\n"
done < <(find . -type f -print0)

# Kopieren in die Zwischenablage
if command -v pbcopy >/dev/null 2>&1; then
    # macOS
    echo -e "$output" | pbcopy
elif command -v xclip >/dev/null 2>&1; then
    # Linux (benötigt xclip)
    echo -e "$output" | xclip -selection clipboard
elif command -v wl-copy >/dev/null 2>&1; then
    # Linux (Wayland, benötigt wl-clipboard)
    echo -e "$output" | wl-copy
else
    echo "Kein unterstütztes Clipboard-Tool gefunden (pbcopy, xclip, wl-copy)."
    exit 1
fi

echo "Dateiinhalte wurden in die Zwischenablage kopiert."
