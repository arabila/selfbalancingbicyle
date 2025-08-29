#!/usr/bin/env python3
"""
Path Visualizer für Self-Balancing Bicycle
Implementiert die Pfad-Visualisierung basierend auf Pose-Daten aus CSV-Logs

Features:
- Weltkoordinaten zu Pixelkoordinaten Mapping
- Live Pfad-Visualisierung während der Simulation
- Automatische Pfad-Export als PNG nach Simulation
- Galerie-System für gespeicherte Fahrten
- Metadaten-Export (JSON/CSV)
"""

import os
import json
import csv
import math
import time
from datetime import datetime
from typing import List, Tuple, Dict, Optional
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import pandas as pd
from PIL import Image, ImageDraw, ImageFont
import tkinter as tk

class PathVisualizer:
    """Hauptklasse für die Pfad-Visualisierung"""
    
    def __init__(self, world_name: str = "S-Kurve"):
        """
        Initialisiert den Path Visualizer
        
        Args:
            world_name: Name der Webots-Welt (für Karten-Mapping)
        """
        self.world_name = world_name
        self.world_bounds = self._get_world_bounds(world_name)
        self.map_image_path = self._get_map_image_path(world_name)
        self.start_position = self._get_world_start_position(world_name)
        
        # Pfad-Daten
        self.path_points: List[Tuple[float, float]] = []
        self.path_timestamps: List[float] = []
        self.metadata: Dict = {}
        
        # Visualisierung
        self.figure = None
        self.canvas = None
        self.live_plot_ax = None
        
        # Galerie-System
        self.gallery_dir = "../../Monitoring/path_gallery"
        self._ensure_gallery_dir()
        
        print(f"✓ Path Visualizer initialisiert für Welt: {world_name}")
    
    def switch_map(self, new_world_name: str) -> bool:
        """
        Wechselt zu einer anderen Karte/Welt
        
        Args:
            new_world_name: Name der neuen Welt
            
        Returns:
            True bei Erfolg, False bei Fehler
        """
        try:
            # Neue Welt-Einstellungen laden
            self.world_name = new_world_name
            self.world_bounds = self._get_world_bounds(new_world_name)
            self.map_image_path = self._get_map_image_path(new_world_name)
            self.start_position = self._get_world_start_position(new_world_name)
            
            # Pfad-Daten zurücksetzen
            self.path_points = []
            self.path_timestamps = []
            
            # Live-Plot neu initialisieren falls vorhanden
            if self.live_plot_ax:
                self.live_plot_ax.clear()
                self._setup_plot_style()
                
                # Neue Karte laden und anzeigen
                if self.map_image_path and os.path.exists(self.map_image_path):
                    self.live_plot_ax.imshow(plt.imread(self.map_image_path), 
                                           extent=[self.world_bounds["min_x"], self.world_bounds["max_x"],
                                                   self.world_bounds["min_y"], self.world_bounds["max_y"]],
                                           alpha=0.7)
                
                # Startpunkt der neuen Welt anzeigen
                start_x, start_y = self.start_position
                self.live_plot_ax.plot(start_x, start_y, 'go', markersize=5, 
                                     markeredgecolor='darkgreen', markeredgewidth=1)
                
                # Achsen neu setzen
                self.live_plot_ax.set_xlim(self.world_bounds["min_x"], self.world_bounds["max_x"])
                self.live_plot_ax.set_ylim(self.world_bounds["min_y"], self.world_bounds["max_y"])
                self.live_plot_ax.set_aspect('equal')
                
                # Canvas aktualisieren
                if self.canvas:
                    self.canvas.draw()
            
            print(f"✓ Karte gewechselt zu: {new_world_name}")
            return True
            
        except Exception as e:
            print(f"❌ Fehler beim Karten-Wechsel: {e}")
            return False
    
    def get_available_maps(self) -> Dict[str, str]:
        """
        Gibt alle verfügbaren Karten zurück
        
        Returns:
            Dict mit Welt-Name -> Anzeige-Name Mapping
        """
        return {
            "S-Kurve": "S-Kurve (S-Kurve.png)",
            "Einfache_Kurve": "Einfache Kurve (Leichte_Kurven_Fahrt.jpeg)",
            "Balance_wind": "Balance Wind (track012.png)",
            "Balance_unebenheiten": "Balance Unebenheiten (track012.png)"
        }
    
    def _get_world_bounds(self, world_name: str) -> Dict[str, float]:
        """
        Bestimmt die Weltgrenzen basierend auf der Webots-Welt
        
        Returns:
            Dict mit min_x, max_x, min_y, max_y in Metern
        """
        # Standard-Bounds für verschiedene Welten
        bounds_map = {
            "S-Kurve": {"min_x": -32, "max_x": 32, "min_y": -50, "max_y": 50},
            "Einfache_Kurve": {"min_x": -32, "max_x": 32, "min_y": -32, "max_y": 32},
            "Balance_wind": {"min_x": -32, "max_x": 32, "min_y": -32, "max_y": 32},
            "Balance_unebenheiten": {"min_x": -32, "max_x": 32, "min_y": -32, "max_y": 32}
        }
        
        return bounds_map.get(world_name, bounds_map["S-Kurve"])
    
    def _get_world_start_position(self, world_name: str) -> Tuple[float, float]:
        """
        Bestimmt die tatsächliche Startposition des Fahrzeugs in der Webots-Welt
        
        Returns:
            Tuple mit (start_x, start_y) in Metern
        """
        # Startpositionen für verschiedene Welten (aus .wbt Dateien extrahiert)
        start_positions = {
            "S-Kurve": (2.348, -43.730),  # translation 2.34802 -43.7299 0.378952
            "Einfache_Kurve": (-17.493, -18.633),  # translation -17.4925 -18.6332 0.31986
            "Balance_wind": (-2.009, -31.171),  # translation -2.00891 -31.1712 0.290969
            "Balance_unebenheiten": (-2.008, -31.171)  # translation -2.00841 -31.1712 0.390969
        }
        
        return start_positions.get(world_name, start_positions["S-Kurve"])
    
    def _get_map_image_path(self, world_name: str) -> Optional[str]:
        """
        Bestimmt den Pfad zur Karten-Textur
        
        Returns:
            Pfad zur Kartentextur oder None
        """
        texture_map = {
            "S-Kurve": "../worlds/textures/S-Kurve.png",
            "Einfache_Kurve": "../worlds/textures/Leichte_Kurven_Fahrt.jpeg",
            "Balance_wind": "../worlds/textures/track012.png",
            "Balance_unebenheiten": "../worlds/textures/track012.png"
        }
        
        texture_path = texture_map.get(world_name)
        if texture_path and os.path.exists(texture_path):
            return texture_path
        return None
    
    def _ensure_gallery_dir(self):
        """Stellt sicher, dass das Galerie-Verzeichnis existiert"""
        os.makedirs(self.gallery_dir, exist_ok=True)
    
    def world_to_pixel(self, world_x: float, world_y: float, 
                      image_width: int = 800, image_height: int = 600) -> Tuple[int, int]:
        """
        Konvertiert Weltkoordinaten zu Pixelkoordinaten
        
        Args:
            world_x, world_y: Position in Weltkoordinaten (Meter)
            image_width, image_height: Zielbildgröße in Pixeln
            
        Returns:
            (pixel_x, pixel_y) Tuple
        """
        # Normalisierung auf [0,1]
        norm_x = (world_x - self.world_bounds["min_x"]) / (
            self.world_bounds["max_x"] - self.world_bounds["min_x"]
        )
        norm_y = (world_y - self.world_bounds["min_y"]) / (
            self.world_bounds["max_y"] - self.world_bounds["min_y"]
        )
        
        # Y-Achse umkehren (Bild-Koordinaten vs. Welt-Koordinaten)
        norm_y = 1.0 - norm_y
        
        # Zu Pixelkoordinaten konvertieren
        pixel_x = int(norm_x * image_width)
        pixel_y = int(norm_y * image_height)
        
        # Clipping
        pixel_x = max(0, min(image_width - 1, pixel_x))
        pixel_y = max(0, min(image_height - 1, pixel_y))
        
        return pixel_x, pixel_y
    
    def load_path_from_csv(self, csv_file_path: str) -> bool:
        """
        Lädt Pfad-Daten aus einer CSV-Log-Datei
        
        Args:
            csv_file_path: Pfad zur CSV-Datei
            
        Returns:
            True bei Erfolg, False bei Fehler
        """
        try:
            df = pd.read_csv(csv_file_path)
            
            # Prüfen ob Position-Spalten vorhanden sind
            required_cols = ['pos_x', 'pos_y', 'timestamp']
            if not all(col in df.columns for col in required_cols):
                print(f"⚠ CSV-Datei {csv_file_path} enthält nicht alle erforderlichen Spalten: {required_cols}")
                return False
            
            # Pfad-Punkte extrahieren
            self.path_points = list(zip(df['pos_x'].values, df['pos_y'].values))
            self.path_timestamps = df['timestamp'].values.tolist()
            
            # Metadaten extrahieren
            self.metadata = {
                "csv_file": os.path.basename(csv_file_path),
                "total_points": len(self.path_points),
                "duration_seconds": self.path_timestamps[-1] - self.path_timestamps[0] if self.path_timestamps else 0,
                "world_name": self.world_name,
                "start_position": self.path_points[0] if self.path_points else None,
                "end_position": self.path_points[-1] if self.path_points else None,
                "created_at": datetime.now().isoformat()
            }
            
            print(f"✓ Pfad geladen: {len(self.path_points)} Punkte über {self.metadata['duration_seconds']:.1f}s")
            return True
            
        except Exception as e:
            print(f"❌ Fehler beim Laden der CSV-Datei {csv_file_path}: {e}")
            return False
    
    def setup_live_plot(self, parent_frame) -> FigureCanvasTkAgg:
        """
        Erstellt die Live-Visualisierung für die GUI
        
        Args:
            parent_frame: Tkinter Frame für die Einbettung
            
        Returns:
            FigureCanvasTkAgg für die GUI-Integration
        """
        # Matplotlib Figure erstellen
        self.figure, self.live_plot_ax = plt.subplots(figsize=(8, 6))
        
        # Hintergrund-Karte laden falls vorhanden
        if self.map_image_path and os.path.exists(self.map_image_path):
            try:
                bg_image = plt.imread(self.map_image_path)
                self.live_plot_ax.imshow(bg_image, extent=[
                    self.world_bounds["min_x"], self.world_bounds["max_x"],
                    self.world_bounds["min_y"], self.world_bounds["max_y"]
                ], aspect='equal', alpha=0.7)
            except Exception as e:
                print(f"⚠ Konnte Hintergrund-Karte nicht laden: {e}")
        
        # Plot-Konfiguration
        self.live_plot_ax.set_xlim(self.world_bounds["min_x"], self.world_bounds["max_x"])
        self.live_plot_ax.set_ylim(self.world_bounds["min_y"], self.world_bounds["max_y"])
        self.live_plot_ax.set_xlabel("X Position [m]")
        self.live_plot_ax.set_ylabel("Y Position [m]")
        self.live_plot_ax.set_title(f"Live Pfad-Visualisierung - {self.world_name}")
        self.live_plot_ax.grid(True, alpha=0.3)
        
        # Startpunkt der Welt bereits anzeigen (50% kleiner, keine Legende)
        start_x, start_y = self.start_position
        self.live_plot_ax.plot(start_x, start_y, 'go', markersize=5, 
                              markeredgecolor='darkgreen', markeredgewidth=1)
        
        # Canvas für Tkinter erstellen
        self.canvas = FigureCanvasTkAgg(self.figure, parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        return self.canvas
    
    def update_live_plot(self, new_points: List[Tuple[float, float]]):
        """
        Aktualisiert die Live-Visualisierung mit neuen Punkten
        
        Args:
            new_points: Liste von (x, y) Koordinaten
        """
        if not self.live_plot_ax or not new_points:
            return
        
        # Neue Punkte zur Gesamtliste hinzufügen
        self.path_points.extend(new_points)
        
        # Plot aktualisieren
        if len(self.path_points) > 1:
            x_coords, y_coords = zip(*self.path_points)
            
            # Pfad zeichnen
            self.live_plot_ax.clear()
            
            # Hintergrund neu zeichnen falls vorhanden
            if self.map_image_path and os.path.exists(self.map_image_path):
                try:
                    bg_image = plt.imread(self.map_image_path)
                    self.live_plot_ax.imshow(bg_image, extent=[
                        self.world_bounds["min_x"], self.world_bounds["max_x"],
                        self.world_bounds["min_y"], self.world_bounds["max_y"]
                    ], aspect='equal', alpha=0.7)
                except:
                    pass
            
            # Pfad-Linie zeichnen (dünner)
            self.live_plot_ax.plot(x_coords, y_coords, 'b-', linewidth=1, alpha=0.8)
            
            # Tatsächlichen Startpunkt der Welt markieren (50% kleiner)
            start_x, start_y = self.start_position
            self.live_plot_ax.plot(start_x, start_y, 'go', markersize=5, markeredgecolor='darkgreen', markeredgewidth=1)
            
            # Ersten aufgezeichneten Punkt markieren (falls unterschiedlich, 50% kleiner)
            if len(x_coords) > 0:
                first_recorded_x, first_recorded_y = x_coords[0], y_coords[0]
                # Nur markieren wenn deutlich unterschiedlich vom Welt-Startpunkt
                distance = ((first_recorded_x - start_x)**2 + (first_recorded_y - start_y)**2)**0.5
                if distance > 1.0:  # Mehr als 1 Meter Unterschied
                    self.live_plot_ax.plot(first_recorded_x, first_recorded_y, 'yo', markersize=4, markeredgecolor='orange', markeredgewidth=1)
            
            # Aktuellen/Endpunkt markieren (50% kleiner)
            if len(x_coords) > 1:
                self.live_plot_ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=4, markeredgecolor='darkred', markeredgewidth=1)
            
            # Plot-Eigenschaften neu setzen
            self.live_plot_ax.set_xlim(self.world_bounds["min_x"], self.world_bounds["max_x"])
            self.live_plot_ax.set_ylim(self.world_bounds["min_y"], self.world_bounds["max_y"])
            self.live_plot_ax.set_xlabel("X Position [m]")
            self.live_plot_ax.set_ylabel("Y Position [m]")
            self.live_plot_ax.set_title(f"Live Pfad-Visualisierung - {self.world_name}")
            self.live_plot_ax.grid(True, alpha=0.3)
            
            # Canvas aktualisieren
            if self.canvas:
                self.canvas.draw()
    
    def export_path_image(self, output_path: str = None, 
                         image_width: int = 1200, image_height: int = 900) -> str:
        """
        Exportiert den gefahrenen Pfad als PNG-Bild
        
        Args:
            output_path: Ausgabepfad (None für automatischen Namen)
            image_width, image_height: Bildgröße in Pixeln
            
        Returns:
            Pfad zur exportierten Datei
        """
        if not self.path_points:
            raise ValueError("Keine Pfad-Daten zum Exportieren vorhanden")
        
        # Ausgabepfad generieren falls nicht angegeben
        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(self.gallery_dir, f"path_{timestamp}.png")
        
        # Bild erstellen
        image = Image.new('RGB', (image_width, image_height), 'white')
        draw = ImageDraw.Draw(image)
        
        # Hintergrund-Karte laden falls vorhanden
        if self.map_image_path and os.path.exists(self.map_image_path):
            try:
                bg_image = Image.open(self.map_image_path)
                bg_image = bg_image.resize((image_width, image_height))
                image.paste(bg_image, (0, 0))
            except Exception as e:
                print(f"⚠ Konnte Hintergrund-Karte nicht laden: {e}")
        
        # Pfad-Punkte zu Pixelkoordinaten konvertieren
        pixel_points = [
            self.world_to_pixel(x, y, image_width, image_height)
            for x, y in self.path_points
        ]
        
        # Pfad zeichnen (dünner)
        if len(pixel_points) > 1:
            draw.line(pixel_points, fill='blue', width=2)
        
        # Tatsächlichen Startpunkt der Welt markieren (50% kleiner)
        world_start_x, world_start_y = self.start_position
        start_pixel_x, start_pixel_y = self.world_to_pixel(world_start_x, world_start_y, image_width, image_height)
        draw.ellipse([start_pixel_x-5, start_pixel_y-5, start_pixel_x+5, start_pixel_y+5], 
                    fill='green', outline='darkgreen', width=2)
        
        # Ersten aufgezeichneten Punkt markieren (falls unterschiedlich, 50% kleiner)
        if pixel_points:
            first_recorded_x, first_recorded_y = pixel_points[0]
            # Nur markieren wenn deutlich unterschiedlich vom Welt-Startpunkt
            distance = ((first_recorded_x - start_pixel_x)**2 + (first_recorded_y - start_pixel_y)**2)**0.5
            if distance > 20:  # Mehr als 20 Pixel Unterschied
                draw.ellipse([first_recorded_x-4, first_recorded_y-4, first_recorded_x+4, first_recorded_y+4], 
                           fill='yellow', outline='orange', width=1)
            
            # Endpunkt markieren (50% kleiner)
            if len(pixel_points) > 1:
                end_x, end_y = pixel_points[-1]
                draw.ellipse([end_x-4, end_y-4, end_x+4, end_y+4], fill='red', outline='darkred', width=1)
        
        # Titel und Metadaten hinzufügen
        try:
            font = ImageFont.truetype("Arial.ttf", 24)
            small_font = ImageFont.truetype("Arial.ttf", 16)
        except:
            font = ImageFont.load_default()
            small_font = ImageFont.load_default()
        
        title = f"Fahrtstrecke - {self.world_name}"
        draw.text((20, 20), title, fill='black', font=font)
        
        if self.metadata:
            info_text = f"Punkte: {self.metadata.get('total_points', 'N/A')} | " \
                       f"Dauer: {self.metadata.get('duration_seconds', 0):.1f}s | " \
                       f"Erstellt: {datetime.now().strftime('%d.%m.%Y %H:%M')}"
            draw.text((20, image_height - 40), info_text, fill='black', font=small_font)
        
        # Bild speichern
        image.save(output_path, 'PNG', quality=95)
        print(f"✓ Pfad-Bild exportiert: {output_path}")
        
        return output_path
    
    def export_metadata(self, output_path: str = None) -> str:
        """
        Exportiert Metadaten als JSON-Datei
        
        Args:
            output_path: Ausgabepfad (None für automatischen Namen)
            
        Returns:
            Pfad zur exportierten JSON-Datei
        """
        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(self.gallery_dir, f"metadata_{timestamp}.json")
        
        # Erweiterte Metadaten
        extended_metadata = {
            **self.metadata,
            "path_points_count": len(self.path_points),
            "world_bounds": self.world_bounds,
            "export_timestamp": datetime.now().isoformat()
        }
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(extended_metadata, f, indent=2, ensure_ascii=False)
        
        print(f"✓ Metadaten exportiert: {output_path}")
        return output_path
    
    def get_gallery_runs(self) -> List[Dict]:
        """
        Gibt alle gespeicherten Läufe aus der Galerie zurück
        
        Returns:
            Liste von Dictionaries mit Run-Informationen
        """
        runs = []
        
        if not os.path.exists(self.gallery_dir):
            return runs
        
        # PNG-Dateien finden
        for filename in os.listdir(self.gallery_dir):
            if filename.endswith('.png') and filename.startswith('path_'):
                image_path = os.path.join(self.gallery_dir, filename)
                
                # Zugehörige JSON-Metadaten suchen
                base_name = filename.replace('path_', '').replace('.png', '')
                json_path = os.path.join(self.gallery_dir, f"metadata_{base_name}.json")
                
                metadata = {}
                if os.path.exists(json_path):
                    try:
                        with open(json_path, 'r', encoding='utf-8') as f:
                            metadata = json.load(f)
                    except:
                        pass
                
                # Run-Info zusammenstellen
                run_info = {
                    "image_path": image_path,
                    "metadata_path": json_path if os.path.exists(json_path) else None,
                    "filename": filename,
                    "created": datetime.fromtimestamp(os.path.getctime(image_path)),
                    **metadata
                }
                
                runs.append(run_info)
        
        # Nach Erstellungsdatum sortieren (neueste zuerst)
        runs.sort(key=lambda x: x["created"], reverse=True)
        
        return runs
    
    def clear_path(self):
        """Löscht die aktuellen Pfad-Daten"""
        self.path_points.clear()
        self.path_timestamps.clear()
        self.metadata.clear()
        
        if self.live_plot_ax:
            self.live_plot_ax.clear()
            if self.canvas:
                self.canvas.draw()


class PathMonitor:
    """
    Monitor-Klasse für kontinuierliche Pfad-Verfolgung aus CSV-Logs
    """
    
    def __init__(self, visualizer: PathVisualizer, monitoring_dir: str):
        self.visualizer = visualizer
        self.monitoring_dir = monitoring_dir
        self.current_csv_file = None
        self.last_processed_line = 0
        self.is_monitoring = False
    
    def start_monitoring(self, csv_file_path: str):
        """
        Startet die Überwachung einer CSV-Datei für Live-Updates
        
        Args:
            csv_file_path: Pfad zur zu überwachenden CSV-Datei
        """
        self.current_csv_file = csv_file_path
        self.last_processed_line = 0
        self.is_monitoring = True
        print(f"✓ Pfad-Monitoring gestartet: {csv_file_path}")
    
    def update_from_csv(self) -> int:
        """
        Aktualisiert die Visualisierung mit neuen Daten aus der CSV-Datei
        
        Returns:
            Anzahl der neu verarbeiteten Zeilen
        """
        if not self.is_monitoring or not self.current_csv_file:
            return 0
        
        if not os.path.exists(self.current_csv_file):
            return 0
        
        try:
            # Neue Zeilen seit der letzten Verarbeitung lesen
            with open(self.current_csv_file, 'r') as f:
                lines = f.readlines()
            
            new_lines = lines[self.last_processed_line + 1:]  # +1 um Header zu überspringen
            
            if not new_lines:
                return 0
            
            # Neue Pfad-Punkte extrahieren
            new_points = []
            for line in new_lines:
                try:
                    parts = line.strip().split(',')
                    if len(parts) >= 15:  # Mindestens bis pos_y (Index 14)
                        pos_x = float(parts[12])  # pos_x ist Spalte 13 (Index 12)
                        pos_y = float(parts[13])  # pos_y ist Spalte 14 (Index 13)
                        new_points.append((pos_x, pos_y))
                except (ValueError, IndexError):
                    continue
            
            if new_points:
                # Live-Plot aktualisieren
                self.visualizer.update_live_plot(new_points)
                self.last_processed_line = len(lines) - 1
                return len(new_points)
            
            return 0
            
        except Exception as e:
            print(f"⚠ Fehler beim CSV-Update: {e}")
            return 0
    
    def stop_monitoring(self):
        """Stoppt die Überwachung"""
        self.is_monitoring = False
        print("✓ Pfad-Monitoring gestoppt")
    
    def finalize_run(self) -> Tuple[str, str]:
        """
        Finalisiert den aktuellen Lauf und exportiert Bild + Metadaten
        
        Returns:
            Tuple von (image_path, metadata_path)
        """
        if not self.current_csv_file:
            return None, None
        
        # Vollständige Daten laden
        success = self.visualizer.load_path_from_csv(self.current_csv_file)
        if not success:
            return None, None
        
        # Export durchführen
        image_path = self.visualizer.export_path_image()
        metadata_path = self.visualizer.export_metadata()
        
        print(f"✓ Lauf finalisiert: {os.path.basename(image_path)}")
        
        return image_path, metadata_path


# Beispiel für die Verwendung
if __name__ == "__main__":
    # Test der Path Visualizer Funktionalität
    visualizer = PathVisualizer("S-Kurve")
    
    # Beispiel-CSV-Datei laden (falls vorhanden)
    csv_files = [f for f in os.listdir("../../Monitoring") if f.endswith('.csv')]
    if csv_files:
        latest_csv = os.path.join("../../Monitoring", sorted(csv_files)[-1])
        if visualizer.load_path_from_csv(latest_csv):
            image_path = visualizer.export_path_image()
            metadata_path = visualizer.export_metadata()
            print(f"Test-Export erstellt: {image_path}")
