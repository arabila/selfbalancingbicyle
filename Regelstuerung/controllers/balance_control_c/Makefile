# Makefile für Balance Control C Controller
# Webots Controller Makefile

# Webots Umgebungsvariablen (macOS)
WEBOTS_HOME ?= /Applications/Webots.app/Contents
WEBOTS_HOME_PATH = $(WEBOTS_HOME)

# Compiler und Flags
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g
LDFLAGS = -lm

# Webots spezifische Flags (macOS)
WEBOTS_INCLUDE = -I$(WEBOTS_HOME_PATH)/include/controller/c
WEBOTS_LIB = -L$(WEBOTS_HOME_PATH)/lib/controller -lController

# Quell- und Zieldateien
TARGET = balance_control_c
SOURCES = balance_control_c.c balance_pid.c balance_config.c balance_logging.c bicycle_physics.c
OBJECTS = $(SOURCES:.c=.o)

# Hauptziel
all: $(TARGET)

# Linking
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(WEBOTS_LIB) $(LDFLAGS)
	@echo "Balance Control C Controller kompiliert"

# Kompilierung der Objektdateien
%.o: %.c
	$(CC) $(CFLAGS) $(WEBOTS_INCLUDE) -c $< -o $@

# Spezifische Abhängigkeiten
balance_control_c.o: balance_control_c.c balance_pid.h balance_config.h balance_logging.h bicycle_physics.h
balance_pid.o: balance_pid.c balance_pid.h
balance_config.o: balance_config.c balance_config.h
balance_logging.o: balance_logging.c balance_logging.h
bicycle_physics.o: bicycle_physics.c bicycle_physics.h

# Aufräumen
clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "Bereinigung abgeschlossen"

# Installation (kopiert ausführbare Datei an den richtigen Ort)
install: $(TARGET)
	@echo "Controller installiert in aktuelles Verzeichnis"

# Debug-Build
debug: CFLAGS += -DDEBUG -g3
debug: clean $(TARGET)

# Release-Build
release: CFLAGS += -O3 -DNDEBUG
release: clean $(TARGET)

# Hilfe
help:
	@echo "Verfügbare Targets:"
	@echo "  all     - Kompiliert den Controller (Standard)"
	@echo "  clean   - Löscht alle generierten Dateien"
	@echo "  debug   - Kompiliert mit Debug-Informationen"
	@echo "  release - Kompiliert mit Optimierungen"
	@echo "  help    - Zeigt diese Hilfe"
	@echo ""
	@echo "Umgebungsvariablen:"
	@echo "  WEBOTS_HOME - Pfad zur Webots-Installation (Standard: /usr/local/webots)"

# Phony Targets
.PHONY: all clean install debug release help 