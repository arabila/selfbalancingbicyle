#!/usr/bin/env python3
"""
Konfigurationslader für die Fahrradregelung
Ermöglicht das Laden der Parameter aus der GUI-Konfiguration
"""

import json
import os
import csv
from datetime import datetime

class ConfigLoader:
    def __init__(self, config_file="../../GUI/controller_config.json"):
        self.config_file = config_file
        self.config = self.load_config()
        
    def load_config(self):
        """Lade Konfiguration aus JSON-Datei"""
        try:
            with open(self.config_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            # Standard-Konfiguration
            default_config = {
                "Kp": 0.01,
                "Ki": 0.02,
                "Kd": 0.0001,
                "maxS": 5.0,
                "minS": 3.0,
                "hMax": 0.1920,
                "preview": 1
            }
            self.save_config(default_config)
            return default_config
    
    def save_config(self, config):
        """Speichere Konfiguration in JSON-Datei"""
        os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
        with open(self.config_file, 'w') as f:
            json.dump(config, f, indent=4)
    
    def get_parameter(self, param_name, default_value=None):
        """Hole einzelnen Parameter"""
        return self.config.get(param_name, default_value)
    
    def reload_config(self):
        """Lade Konfiguration neu"""
        self.config = self.load_config()
        return self.config

class MonitoringLogger:
    def __init__(self, monitoring_dir="../../Monitoring"):
        self.monitoring_dir = monitoring_dir
        os.makedirs(monitoring_dir, exist_ok=True)
        
        # Erstelle Dateiname mit Zeitstempel
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(monitoring_dir, f"monitoring_{timestamp}.csv")
        
        # CSV-Header
        self.headers = [
            "timestamp", "Kp", "Ki", "Kd", "P", "I", "D", "PID", 
            "speed", "handlebar_angle", "error", "roll_angle"
        ]
        
        # Erstelle CSV-Datei mit Headern
        self.write_header()
        
    def write_header(self):
        """Schreibe CSV-Header"""
        with open(self.csv_file, 'w') as f:
            f.write(','.join(self.headers) + '\n')
    
    def log_data(self, data_dict):
        """Schreibe Daten in CSV-Datei"""
        timestamp = datetime.now().isoformat()
        
        # Stelle sicher, dass alle benötigten Felder vorhanden sind
        row_data = [str(data_dict.get(header, '')) for header in self.headers]
        row_data[0] = timestamp  # Überschreibe mit aktuellem Zeitstempel
        
        with open(self.csv_file, 'a') as f:
            f.write(','.join(row_data) + '\n')
    
    def save_crash_data(self):
        """Speichere Daten bei Fahrrad-Sturz"""
        # Die Datei ist bereits gespeichert, da wir kontinuierlich schreiben
        # Hier könnten zusätzliche Metadaten hinzugefügt werden
        crash_info = {
            "crash_time": datetime.now().isoformat(),
            "file": self.csv_file
        }
        
        crash_file = os.path.join(self.monitoring_dir, "last_crash.json")
        with open(crash_file, 'w') as f:
            json.dump(crash_info, f, indent=4)
        
        return self.csv_file

# Convenience-Funktionen für einfache Nutzung
def load_controller_config():
    """Lade Controller-Konfiguration"""
    loader = ConfigLoader()
    return loader.load_config()

def create_monitoring_session():
    """Erstelle neue Monitoring-Session"""
    loader = ConfigLoader()
    return loader.start_monitoring_session() 