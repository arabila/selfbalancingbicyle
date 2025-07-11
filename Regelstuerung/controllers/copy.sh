#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# dump_sources_to_clipboard.sh
#
# Sammelt alle relevanten Quell-/Textdateien unterhalb des Projekt­wurzel-
# verzeichnisses, versieht jede Datei mit einer Kopfzeile
#    ===== relativer/pfad/datei =====
# und kopiert den gesamten Dump in die Zwischenablage.
#
# Benutzung:
#   chmod +x dump_sources_to_clipboard.sh
#   ./dump_sources_to_clipboard.sh          # danach ⌘V / Ctrl V zum Einfügen
# -----------------------------------------------------------------------------

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --------------------------------------------------------------------------- #
#  1) Alle gewünschten Dateien einsammeln                                     #
# --------------------------------------------------------------------------- #
TMP_FILE="$(mktemp)"
trap 'rm -f "$TMP_FILE"' EXIT

# Muster, die wir ignorieren
EXCLUDE_PATTERNS=( "*.pyc" "*.o" "*.so" "*.a" )

# find-Kommando zusammenbasteln
FIND_CMD=(find "$ROOT_DIR"
          # versteckte Ordner ignorieren
          -type d -name '.*' -prune -o
          # yolo_vision-Ordner ignorieren
          -path '*/balance_control_c/yolo_vision/*' -prune -o
)

# EXCLUDE_PATTERNS einhängen
for pat in "${EXCLUDE_PATTERNS[@]}"; do
  FIND_CMD+=( -name "$pat" -prune -o )
done

FIND_CMD+=( -type f -print )

# Dateien sortiert ausgeben und in TMP_FILE schreiben
"${FIND_CMD[@]}" | sort | while read -r FILE; do
  REL_PATH="${FILE#$ROOT_DIR/}"
  {
    echo "===== ${REL_PATH} ====="
    cat "$FILE"
    echo              # Leerzeile als Trenner
  } >> "$TMP_FILE"
done

# --------------------------------------------------------------------------- #
# 2) Dump in die Zwischenablage kopieren                                      #
# --------------------------------------------------------------------------- #
if command -v pbcopy >/dev/null 2>&1; then
  pbcopy < "$TMP_FILE"
  echo "✓ Dump in die macOS-Zwischenablage kopiert (pbcopy)."
elif command -v xclip >/dev/null 2>&1; then
  xclip -selection clipboard < "$TMP_FILE"
  echo "✓ Dump in die X11-Zwischenablage kopiert (xclip)."
elif command -v wl-copy >/dev/null 2>&1; then
  wl-copy < "$TMP_FILE"
  echo "✓ Dump in die Wayland-Zwischenablage kopiert (wl-copy)."
else
  echo "⚠ Kein Clipboard-Tool gefunden – Dump unter $TMP_FILE abgelegt."
fi