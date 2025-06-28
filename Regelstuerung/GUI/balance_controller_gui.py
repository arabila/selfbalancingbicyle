#!/usr/bin/env python3
"""
Balance Controller GUI
Erweiterte GUI für die Konfiguration des Balance-Controllers

Basiert auf der bestehenden bicycle_controller_gui.py
Fügt spezifische Parameter für die Balance-Regelung hinzu
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import os
import threading
import time
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import pandas as pd

class BalanceControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Balance Controller GUI - Selbstbalancierendes Fahrrad")
        self.root.geometry("1200x800")
        
        # Konfigurationsdatei
        self.config_file = "balance_config.json"
        self.monitoring_dir = "../Monitoring"
        
        # Parameter-Definitionen (mit Limits und Beschreibungen)
        self.parameters = {
            # Angle PID Parameter
            "angle_Kp": {
                "name": "Angle PID - Kp",
                "value": 10.0,
                "min": 0.0,
                "max": 50.0,
                "description": "Proportional-Verstärkung für Roll-Winkel → Lenkwinkel",
                "unit": ""
            },
            "angle_Ki": {
                "name": "Angle PID - Ki", 
                "value": 0.0,
                "min": 0.0,
                "max": 10.0,
                "description": "Integral-Verstärkung (meist 0 für Stabilität)",
                "unit": ""
            },
            "angle_Kd": {
                "name": "Angle PID - Kd",
                "value": 2.2,
                "min": 0.0,
                "max": 10.0,
                "description": "Differential-Verstärkung für Dämpfung",
                "unit": ""
            },
            "angle_output_min": {
                "name": "Min. Lenkwinkel",
                "value": -0.3,
                "min": -1.57,
                "max": 0.0,
                "description": "Minimaler Lenkwinkel in Radiant",
                "unit": "rad"
            },
            "angle_output_max": {
                "name": "Max. Lenkwinkel",
                "value": 0.3,
                "min": 0.0,
                "max": 1.57,
                "description": "Maximaler Lenkwinkel in Radiant", 
                "unit": "rad"
            },
            
            # Speed Control Parameter
            "base_speed": {
                "name": "Basis-Geschwindigkeit",
                "value": 5.0,
                "min": 1.0,
                "max": 15.0,
                "description": "Standard-Fahrgeschwindigkeit",
                "unit": "rad/s"
            },
            "min_speed": {
                "name": "Min. Geschwindigkeit",
                "value": 3.0,
                "min": 0.5,
                "max": 10.0,
                "description": "Mindestgeschwindigkeit für Stabilität",
                "unit": "rad/s"
            },
            "max_speed": {
                "name": "Max. Geschwindigkeit", 
                "value": 8.0,
                "min": 5.0,
                "max": 20.0,
                "description": "Maximale Fahrgeschwindigkeit",
                "unit": "rad/s"
            },
            "stability_reduction": {
                "name": "Stabilitätsfaktor",
                "value": 0.5,
                "min": 0.0,
                "max": 1.0,
                "description": "Geschwindigkeitsreduktion bei Instabilität (0=keine, 1=maximal)",
                "unit": ""
            },
            
            # Mechanical Limits
            "max_handlebar_angle": {
                "name": "Max. Lenkwinkel",
                "value": 0.32,
                "min": 0.1,
                "max": 0.8,
                "description": "Maximaler physischer Lenkwinkel",
                "unit": "rad"
            },
            "max_roll_angle": {
                "name": "Max. Roll-Winkel",
                "value": 45.0,
                "min": 10.0,
                "max": 90.0,
                "description": "Maximaler Roll-Winkel für Plausibilitätsprüfung",
                "unit": "°"
            }
        }
        
        # Physik-Parameter für erweiterte Simulation
        self.physics_parameters = {
            # Physikalische Konstanten
            "bicycle_mass": {
                "name": "Fahrrad-Masse",
                "value": 3.5,
                "min": 1.0,
                "max": 10.0,
                "description": "Gesamtmasse des Fahrrads",
                "unit": "kg"
            },
            "wheel_radius": {
                "name": "Rad-Radius",
                "value": 0.055,
                "min": 0.02,
                "max": 0.15,
                "description": "Radius der Fahrradräder",
                "unit": "m"
            },
            "frontal_area": {
                "name": "Stirnfläche",
                "value": 0.3,
                "min": 0.1,
                "max": 0.8,
                "description": "Frontale Querschnittsfläche für Aerodynamik",
                "unit": "m²"
            },
            
            # Aerodynamische Parameter
            "air_density": {
                "name": "Luftdichte",
                "value": 1.225,
                "min": 0.8,
                "max": 1.5,
                "description": "Luftdichte (abhängig von Höhe/Temperatur)",
                "unit": "kg/m³"
            },
            "drag_coefficient": {
                "name": "Luftwiderstandsbeiwert",
                "value": 0.9,
                "min": 0.3,
                "max": 1.5,
                "description": "Cd-Wert für Aerodynamik",
                "unit": ""
            },
            "side_area": {
                "name": "Seitenfläche",
                "value": 0.4,
                "min": 0.2,
                "max": 0.8,
                "description": "Seitliche Fläche für Querwind",
                "unit": "m²"
            },
            
            # Reifen-Parameter
            "tire_stiffness": {
                "name": "Reifensteifigkeit",
                "value": 2000.0,
                "min": 500.0,
                "max": 5000.0,
                "description": "Seitenkraft-Steifigkeit der Reifen",
                "unit": "N/rad"
            },
            "rolling_resistance_coeff": {
                "name": "Rollwiderstandsbeiwert",
                "value": 0.005,
                "min": 0.001,
                "max": 0.02,
                "description": "Crr-Wert für Rollwiderstand",
                "unit": ""
            },
            "tire_friction_coeff": {
                "name": "Reibungskoeffizient",
                "value": 0.8,
                "min": 0.2,
                "max": 1.2,
                "description": "Maximaler Reibungskoeffizient μ",
                "unit": ""
            },
            
            # Sensorsimulation
            "imu_noise_sigma": {
                "name": "IMU-Rauschen",
                "value": 0.01,
                "min": 0.0,
                "max": 0.1,
                "description": "Standardabweichung des Sensorrauschens",
                "unit": "rad"
            },
            "imu_delay_samples": {
                "name": "IMU-Verzögerung",
                "value": 2,
                "min": 0,
                "max": 10,
                "description": "Verzögerung in Zeitschritten (5ms)",
                "unit": "samples"
            },
            
            # Umweltparameter
            "wind_speed": {
                "name": "Windgeschwindigkeit",
                "value": 0.0,
                "min": 0.0,
                "max": 20.0,
                "description": "Aktuelle Windgeschwindigkeit",
                "unit": "m/s"
            },
            "wind_direction": {
                "name": "Windrichtung",
                "value": 0.0,
                "min": -3.14159,
                "max": 3.14159,
                "description": "Windrichtung (0=Gegenwind, π/2=Seitenwind)",
                "unit": "rad"
            },
            "wind_turbulence": {
                "name": "Wind-Turbulenz",
                "value": 0.1,
                "min": 0.0,
                "max": 1.0,
                "description": "Turbulenzintensität (0=ruhig, 1=sehr böig)",
                "unit": ""
            },
            "road_slope": {
                "name": "Straßenneigung",
                "value": 0.0,
                "min": -0.2,
                "max": 0.2,
                "description": "Straßenneigung (positiv=bergab)",
                "unit": "rad"
            }
        }
        
        # Physik-Effekt-Aktivierung
        self.physics_effects = {
            "enable_lateral_forces": {
                "name": "Laterale Reifenkräfte",
                "value": True,
                "description": "Pacejka-Modell für Seitenkräfte aktivieren"
            },
            "enable_aerodynamics": {
                "name": "Aerodynamik",
                "value": True,
                "description": "Luftwiderstand und Querwind aktivieren"
            },
            "enable_rolling_resistance": {
                "name": "Rollwiderstand",
                "value": True,
                "description": "Rollwiderstandsmoment aktivieren"
            },
            "enable_gyroscopic": {
                "name": "Gyroskopmommente",
                "value": True,
                "description": "Gyroskopmommente der Räder aktivieren"
            },
            "enable_sensor_simulation": {
                "name": "Sensorsimulation",
                "value": True,
                "description": "IMU-Rauschen und Verzögerung aktivieren"
            },
            "enable_environment": {
                "name": "Umwelteinflüsse",
                "value": True,
                "description": "Wind und Straßenneigung aktivieren"
            }
        }
        
        # GUI-Komponenten
        self.param_widgets = {}
        self.physics_widgets = {}
        self.effects_widgets = {}
        self.status_vars = {}
        
        self.setup_gui()
        self.load_config()
        
    def setup_gui(self):
        """Erstelle die GUI-Komponenten"""
        
        # Hauptframe mit Notebook für Tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Tab 1: Parameter-Konfiguration
        param_frame = ttk.Frame(notebook)
        notebook.add(param_frame, text="Parameter")
        self.setup_parameter_tab(param_frame)
        
        # Tab 2: Monitoring
        monitor_frame = ttk.Frame(notebook)
        notebook.add(monitor_frame, text="Monitoring")
        self.setup_monitoring_tab(monitor_frame)
        
        # Tab 3: Erweiterte Physik
        physics_frame = ttk.Frame(notebook)
        notebook.add(physics_frame, text="Physik")
        self.setup_physics_tab(physics_frame)
        
        # Tab 4: Presets
        preset_frame = ttk.Frame(notebook)
        notebook.add(preset_frame, text="Presets")
        self.setup_preset_tab(preset_frame)
        
        # Status-Leiste
        self.setup_status_bar()
        
    def setup_parameter_tab(self, parent):
        """Erstelle die Parameter-Konfiguration"""
        
        # Scrollbarer Frame für viele Parameter
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Parameter-Gruppen
        groups = {
            "Angle PID Parameter": ["angle_Kp", "angle_Ki", "angle_Kd", "angle_output_min", "angle_output_max"],
            "Geschwindigkeitsregelung": ["base_speed", "min_speed", "max_speed", "stability_reduction"],
            "Mechanische Grenzen": ["max_handlebar_angle", "max_roll_angle"]
        }
        
        row = 0
        for group_name, param_list in groups.items():
            # Gruppe-Header
            group_label = ttk.Label(scrollable_frame, text=group_name, font=("Arial", 12, "bold"))
            group_label.grid(row=row, column=0, columnspan=4, sticky="w", pady=(20, 10))
            row += 1
            
            # Parameter in der Gruppe
            for param_key in param_list:
                param = self.parameters[param_key]
                
                # Label
                label = ttk.Label(scrollable_frame, text=param["name"])
                label.grid(row=row, column=0, sticky="w", padx=(20, 10), pady=2)
                
                # Scale/Slider
                var = tk.DoubleVar(value=param["value"])
                scale = ttk.Scale(
                    scrollable_frame,
                    from_=param["min"],
                    to=param["max"],
                    variable=var,
                    orient=tk.HORIZONTAL,
                    length=200,
                    command=lambda val, key=param_key: self.on_parameter_change(key, val)
                )
                scale.grid(row=row, column=1, padx=10, pady=2)
                
                # Wert-Anzeige
                value_label = ttk.Label(scrollable_frame, text=f"{param['value']:.3f} {param['unit']}")
                value_label.grid(row=row, column=2, padx=10, pady=2)
                
                # Beschreibung
                desc_label = ttk.Label(scrollable_frame, text=param["description"], foreground="gray")
                desc_label.grid(row=row, column=3, sticky="w", padx=10, pady=2)
                
                # Widgets speichern
                self.param_widgets[param_key] = {
                    "var": var,
                    "scale": scale,
                    "value_label": value_label
                }
                
                row += 1
        
        # Control-Buttons
        button_frame = ttk.Frame(scrollable_frame)
        button_frame.grid(row=row, column=0, columnspan=4, pady=20)
        
        ttk.Button(button_frame, text="Konfiguration laden", command=self.load_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Konfiguration speichern", command=self.save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Standardwerte", command=self.reset_defaults).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Anwenden", command=self.apply_config).pack(side=tk.LEFT, padx=5)
        
        # Scrollbaren Frame einrichten
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
    def setup_physics_tab(self, parent):
        """Erstelle das Erweiterte Physik-Tab"""
        
        # Hauptcontainer mit zwei Spalten
        main_frame = ttk.Frame(parent)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Linke Spalte: Physik-Parameter
        left_frame = ttk.LabelFrame(main_frame, text="Physik-Parameter")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Scrollbarer Frame für Parameter
        canvas_left = tk.Canvas(left_frame)
        scrollbar_left = ttk.Scrollbar(left_frame, orient="vertical", command=canvas_left.yview)
        scrollable_left = ttk.Frame(canvas_left)
        
        scrollable_left.bind(
            "<Configure>",
            lambda e: canvas_left.configure(scrollregion=canvas_left.bbox("all"))
        )
        
        canvas_left.create_window((0, 0), window=scrollable_left, anchor="nw")
        canvas_left.configure(yscrollcommand=scrollbar_left.set)
        
        # Physik-Parameter-Gruppen
        physics_groups = {
            "Fahrzeug-Eigenschaften": ["bicycle_mass", "wheel_radius", "frontal_area"],
            "Aerodynamik": ["air_density", "drag_coefficient", "side_area"],
            "Reifen-Parameter": ["tire_stiffness", "rolling_resistance_coeff", "tire_friction_coeff"],
            "Sensorsimulation": ["imu_noise_sigma", "imu_delay_samples"],
            "Umwelt-Parameter": ["wind_speed", "wind_direction", "wind_turbulence", "road_slope"]
        }
        
        row = 0
        for group_name, param_list in physics_groups.items():
            # Gruppe-Header
            group_label = ttk.Label(scrollable_left, text=group_name, font=("Arial", 11, "bold"))
            group_label.grid(row=row, column=0, columnspan=4, sticky="w", pady=(15, 5))
            row += 1
            
            # Parameter in der Gruppe
            for param_key in param_list:
                param = self.physics_parameters[param_key]
                
                # Label
                label = ttk.Label(scrollable_left, text=param["name"])
                label.grid(row=row, column=0, sticky="w", padx=(10, 10), pady=2)
                
                # Scale/Slider
                if param_key == "imu_delay_samples":
                    # Spezialbehandlung für Integer-Parameter
                    var = tk.IntVar(value=int(param["value"]))
                    scale = ttk.Scale(
                        scrollable_left,
                        from_=param["min"],
                        to=param["max"],
                        variable=var,
                        orient=tk.HORIZONTAL,
                        length=150,
                        command=lambda val, key=param_key: self.on_physics_parameter_change(key, val)
                    )
                else:
                    var = tk.DoubleVar(value=param["value"])
                    scale = ttk.Scale(
                        scrollable_left,
                        from_=param["min"],
                        to=param["max"],
                        variable=var,
                        orient=tk.HORIZONTAL,
                        length=150,
                        command=lambda val, key=param_key: self.on_physics_parameter_change(key, val)
                    )
                scale.grid(row=row, column=1, padx=5, pady=2)
                
                # Wert-Anzeige
                value_label = ttk.Label(scrollable_left, text=f"{param['value']:.3f} {param['unit']}")
                value_label.grid(row=row, column=2, padx=5, pady=2)
                
                # Beschreibung (kürzer)
                desc_text = param["description"][:40] + "..." if len(param["description"]) > 40 else param["description"]
                desc_label = ttk.Label(scrollable_left, text=desc_text, foreground="gray", font=("Arial", 8))
                desc_label.grid(row=row, column=3, sticky="w", padx=5, pady=2)
                
                # Widgets speichern
                self.physics_widgets[param_key] = {
                    "var": var,
                    "scale": scale,
                    "value_label": value_label
                }
                
                row += 1
        
        # Physics Parameter Controls
        control_frame_left = ttk.Frame(scrollable_left)
        control_frame_left.grid(row=row, column=0, columnspan=4, pady=15)
        
        ttk.Button(control_frame_left, text="Physik Reset", command=self.reset_physics_defaults).pack(side=tk.LEFT, padx=2)
        ttk.Button(control_frame_left, text="Live Update", command=self.apply_physics_live).pack(side=tk.LEFT, padx=2)
        
        # Scrollbar für linke Seite
        canvas_left.pack(side="left", fill="both", expand=True)
        scrollbar_left.pack(side="right", fill="y")
        
        # Rechte Spalte: Effekt-Aktivierung und Szenarien
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        
        # Effekt-Aktivierung
        effects_frame = ttk.LabelFrame(right_frame, text="Physik-Effekte aktivieren")
        effects_frame.pack(fill=tk.X, pady=(0, 10))
        
        row = 0
        for effect_key, effect_data in self.physics_effects.items():
            var = tk.BooleanVar(value=effect_data["value"])
            checkbox = ttk.Checkbutton(
                effects_frame,
                text=effect_data["name"],
                variable=var,
                command=lambda key=effect_key: self.on_physics_effect_change(key)
            )
            checkbox.grid(row=row, column=0, sticky="w", padx=10, pady=3)
            
            # Beschreibung
            desc_label = ttk.Label(effects_frame, text=effect_data["description"], 
                                 foreground="gray", font=("Arial", 8))
            desc_label.grid(row=row+1, column=0, sticky="w", padx=20, pady=(0, 5))
            
            self.effects_widgets[effect_key] = var
            row += 2
        
        # Schnelle Szenarien
        scenarios_frame = ttk.LabelFrame(right_frame, text="Schnelle Szenarien")
        scenarios_frame.pack(fill=tk.X, pady=10)
        
        scenarios = [
            ("Ruhige Bedingungen", self.scenario_calm),
            ("Leichter Wind", self.scenario_light_wind),
            ("Starker Seitenwind", self.scenario_strong_wind),
            ("Steile Abfahrt", self.scenario_downhill),
            ("Extrembedingungen", self.scenario_extreme),
            ("Sensorfehler Test", self.scenario_sensor_errors)
        ]
        
        for i, (name, command) in enumerate(scenarios):
            ttk.Button(scenarios_frame, text=name, command=command, width=18).pack(pady=2, padx=5)
        
        # Physik-Status-Anzeige
        status_frame = ttk.LabelFrame(right_frame, text="Physik-Status")
        status_frame.pack(fill=tk.X, pady=10)
        
        self.physics_status_text = tk.Text(status_frame, height=8, width=25, font=("Courier", 9))
        self.physics_status_text.pack(padx=5, pady=5)
        
        # Status regelmäßig aktualisieren
        self.update_physics_status()
        
    def setup_monitoring_tab(self, parent):
        """Erstelle das Monitoring-Tab"""
        
        # Monitoring-Controls
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Buttons links
        ttk.Button(control_frame, text="Log-Datei öffnen", command=self.open_log_file).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Live-Plot starten", command=self.start_live_plot).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Monitoring stoppen", command=self.stop_live_plot).pack(side=tk.LEFT, padx=5)
        
        # Separator
        separator = ttk.Separator(control_frame, orient="vertical")
        separator.pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        # Visibility checkboxes rechts
        visibility_frame = ttk.Frame(control_frame)
        visibility_frame.pack(side=tk.LEFT, padx=10)
        
        ttk.Label(visibility_frame, text="Anzeigen:").pack(side=tk.LEFT, padx=5)
        
        # Checkbox-Variablen initialisieren
        self.plot_visibility = {
            "roll_angle": tk.BooleanVar(value=True),
            "steering_output": tk.BooleanVar(value=True),
            "p_term": tk.BooleanVar(value=True),
            "i_term": tk.BooleanVar(value=True),
            "d_term": tk.BooleanVar(value=True)
        }
        
        # Checkboxes erstellen
        checkbox_config = [
            ("roll_angle", "Roll-Winkel"),
            ("steering_output", "Lenkwinkel"),
            ("p_term", "P-Term"),
            ("i_term", "I-Term"),
            ("d_term", "D-Term")
        ]
        
        for key, label in checkbox_config:
            checkbox = ttk.Checkbutton(
                visibility_frame,
                text=label,
                variable=self.plot_visibility[key],
                command=self.update_plot_visibility
            )
            checkbox.pack(side=tk.LEFT, padx=3)
        
        # Separator für Zoom-Controls
        zoom_separator = ttk.Separator(control_frame, orient="vertical")
        zoom_separator.pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        # X-Achse Zoom Controls
        zoom_frame = ttk.Frame(control_frame)
        zoom_frame.pack(side=tk.LEFT, padx=10)
        
        ttk.Label(zoom_frame, text="X-Achse Zoom:").pack(side=tk.LEFT, padx=5)
        
        # Zoom-Variablen initialisieren
        self.zoom_start = tk.DoubleVar(value=0.0)
        self.zoom_end = tk.DoubleVar(value=100.0)
        self.data_duration = 10.0  # Fallback-Wert
        
        # Start-Slider
        start_frame = ttk.Frame(zoom_frame)
        start_frame.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(start_frame, text="Von:", font=("Arial", 8)).pack()
        self.start_slider = ttk.Scale(
            start_frame,
            from_=0.0,
            to=100.0,
            variable=self.zoom_start,
            orient=tk.HORIZONTAL,
            length=80,
            command=self.on_zoom_start_change
        )
        self.start_slider.pack()
        self.start_label = ttk.Label(start_frame, text="0.0s", font=("Arial", 8))
        self.start_label.pack()
        
        # End-Slider
        end_frame = ttk.Frame(zoom_frame)
        end_frame.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(end_frame, text="Bis:", font=("Arial", 8)).pack()
        self.end_slider = ttk.Scale(
            end_frame,
            from_=0.0,
            to=100.0,
            variable=self.zoom_end,
            orient=tk.HORIZONTAL,
            length=80,
            command=self.on_zoom_end_change
        )
        self.end_slider.pack()
        self.end_label = ttk.Label(end_frame, text="10.0s", font=("Arial", 8))
        self.end_label.pack()
        
        # Plot-Bereich
        self.setup_plot_area(parent)
        
        # Status-Anzeige
        status_frame = ttk.LabelFrame(parent, text="System-Status")
        status_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.status_vars = {
            "controller_running": tk.StringVar(value="Unbekannt"),
            "log_file": tk.StringVar(value="Keine Datei geladen"),
            "last_update": tk.StringVar(value="Nie")
        }
        
        for i, (key, var) in enumerate(self.status_vars.items()):
            ttk.Label(status_frame, text=f"{key.replace('_', ' ').title()}:").grid(row=i, column=0, sticky="w", padx=5, pady=2)
            ttk.Label(status_frame, textvariable=var).grid(row=i, column=1, sticky="w", padx=20, pady=2)
        
    def setup_plot_area(self, parent):
        """Erstelle den Plot-Bereich für Live-Monitoring"""
        
        plot_frame = ttk.LabelFrame(parent, text="Live-Daten")
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Matplotlib Figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.tight_layout()
        
        # Plot-Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Plot-Daten initialisieren
        self.plot_data = {
            "time": [],
            "roll_angle": [],
            "steering_output": [],
            "speed": [],
            "p_term": [],
            "i_term": [],
            "d_term": []
        }
        
        self.live_plot_active = False
        self.current_data = None
        
    def setup_preset_tab(self, parent):
        """Erstelle das Preset-Tab"""
        
        preset_frame = ttk.LabelFrame(parent, text="Vorkonfigurierte Einstellungen")
        preset_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Preset-Definitionen
        presets = {
            "Autobike Original": {
                "description": "Original-Parameter aus der BachelorArbeit 2023",
                "params": {
                    "angle_Kp": 10.0, "angle_Ki": 0.0, "angle_Kd": 2.2,
                    "base_speed": 5.0, "min_speed": 3.0, "max_speed": 8.0
                }
            },
            "Konservativ": {
                "description": "Sichere, langsame Einstellungen für Tests",
                "params": {
                    "angle_Kp": 5.0, "angle_Ki": 0.0, "angle_Kd": 1.5,
                    "base_speed": 3.0, "min_speed": 2.0, "max_speed": 5.0
                }
            },
            "Aggressiv": {
                "description": "Schnelle, responsive Einstellungen",
                "params": {
                    "angle_Kp": 15.0, "angle_Ki": 0.5, "angle_Kd": 3.0,
                    "base_speed": 7.0, "min_speed": 4.0, "max_speed": 12.0
                }
            }
        }
        
        row = 0
        for preset_name, preset_data in presets.items():
            # Preset-Name
            name_label = ttk.Label(preset_frame, text=preset_name, font=("Arial", 11, "bold"))
            name_label.grid(row=row, column=0, sticky="w", padx=10, pady=(10, 5))
            
            # Beschreibung
            desc_label = ttk.Label(preset_frame, text=preset_data["description"], foreground="gray")
            desc_label.grid(row=row+1, column=0, sticky="w", padx=20, pady=(0, 5))
            
            # Laden-Button
            load_btn = ttk.Button(
                preset_frame, 
                text="Laden",
                command=lambda preset=preset_data["params"]: self.load_preset(preset)
            )
            load_btn.grid(row=row, column=1, padx=10, pady=5)
            
            row += 2
        
    def setup_status_bar(self):
        """Erstelle die Status-Leiste"""
        
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.status_text = tk.StringVar(value="Bereit")
        status_label = ttk.Label(status_frame, textvariable=self.status_text)
        status_label.pack(side=tk.LEFT, padx=10, pady=5)
        
        # Zeit-Anzeige
        self.time_text = tk.StringVar()
        time_label = ttk.Label(status_frame, textvariable=self.time_text)
        time_label.pack(side=tk.RIGHT, padx=10, pady=5)
        
        self.update_time()
        
    def update_time(self):
        """Aktualisiere die Zeit-Anzeige"""
        current_time = datetime.now().strftime("%H:%M:%S")
        self.time_text.set(current_time)
        self.root.after(1000, self.update_time)
        
    def on_parameter_change(self, param_key, value):
        """Handler für Parameter-Änderungen"""
        try:
            float_value = float(value)
            self.parameters[param_key]["value"] = float_value
            
            # Wert-Label aktualisieren
            unit = self.parameters[param_key]["unit"]
            self.param_widgets[param_key]["value_label"].config(text=f"{float_value:.3f} {unit}")
            
            self.status_text.set(f"Parameter {param_key} geändert: {float_value:.3f}")
            
        except ValueError:
            pass
            
    def on_physics_parameter_change(self, param_key, value):
        """Handler für Physik-Parameter-Änderungen"""
        try:
            if param_key == "imu_delay_samples":
                int_value = int(float(value))
                self.physics_parameters[param_key]["value"] = int_value
                unit = self.physics_parameters[param_key]["unit"]
                self.physics_widgets[param_key]["value_label"].config(text=f"{int_value} {unit}")
                self.status_text.set(f"Physik-Parameter {param_key} geändert: {int_value}")
            else:
                float_value = float(value)
                self.physics_parameters[param_key]["value"] = float_value
                unit = self.physics_parameters[param_key]["unit"]
                self.physics_widgets[param_key]["value_label"].config(text=f"{float_value:.3f} {unit}")
                self.status_text.set(f"Physik-Parameter {param_key} geändert: {float_value:.3f}")
                
        except ValueError:
            pass
            
    def on_physics_effect_change(self, effect_key):
        """Handler für Physik-Effekt-Aktivierung"""
        value = self.effects_widgets[effect_key].get()
        self.physics_effects[effect_key]["value"] = value
        status = "aktiviert" if value else "deaktiviert"
        self.status_text.set(f"Physik-Effekt {effect_key} {status}")
            
    def load_config(self):
        """Lade Konfiguration aus JSON-Datei"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config_data = json.load(f)
                
                # Parameter aus JSON laden
                balance_config = config_data.get("balance_control", {})
                
                # Alle Kategorien durchgehen
                for category in ["angle_pid", "speed_control", "mechanical_limits"]:
                    category_data = balance_config.get(category, {})
                    for key, value in category_data.items():
                        if key in self.parameters:
                            self.parameters[key]["value"] = value
                            if key in self.param_widgets:
                                self.param_widgets[key]["var"].set(value)
                                unit = self.parameters[key]["unit"]
                                self.param_widgets[key]["value_label"].config(text=f"{value:.3f} {unit}")
                
                # Physik-Parameter laden
                physics_config = config_data.get("physics", {})
                
                # Physik-Parameter
                physics_params = physics_config.get("parameters", {})
                for key, value in physics_params.items():
                    if key in self.physics_parameters:
                        self.physics_parameters[key]["value"] = value
                        if key in self.physics_widgets:
                            self.physics_widgets[key]["var"].set(value)
                            unit = self.physics_parameters[key]["unit"]
                            if key == "imu_delay_samples":
                                self.physics_widgets[key]["value_label"].config(text=f"{int(value)} {unit}")
                            else:
                                self.physics_widgets[key]["value_label"].config(text=f"{value:.3f} {unit}")
                
                # Physik-Effekte laden
                physics_effects = physics_config.get("effects", {})
                for key, value in physics_effects.items():
                    if key in self.physics_effects:
                        self.physics_effects[key]["value"] = value
                        if key in self.effects_widgets:
                            self.effects_widgets[key].set(value)
                
                self.status_text.set(f"Konfiguration aus {self.config_file} geladen")
            else:
                self.status_text.set(f"Konfigurationsdatei {self.config_file} nicht gefunden")
                
        except Exception as e:
            messagebox.showerror("Fehler", f"Fehler beim Laden der Konfiguration: {str(e)}")
            
    def save_config(self):
        """Speichere Konfiguration in JSON-Datei"""
        try:
            config_data = {
                "balance_control": {
                    "angle_pid": {
                        "angle_Kp": self.parameters["angle_Kp"]["value"],
                        "angle_Ki": self.parameters["angle_Ki"]["value"],
                        "angle_Kd": self.parameters["angle_Kd"]["value"],
                        "angle_output_min": self.parameters["angle_output_min"]["value"],
                        "angle_output_max": self.parameters["angle_output_max"]["value"],
                        "angle_integral_min": -60.0,
                        "angle_integral_max": 60.0
                    },
                    "speed_control": {
                        "base_speed": self.parameters["base_speed"]["value"],
                        "min_speed": self.parameters["min_speed"]["value"],
                        "max_speed": self.parameters["max_speed"]["value"],
                        "stability_reduction": self.parameters["stability_reduction"]["value"]
                    },
                    "mechanical_limits": {
                        "max_handlebar_angle": self.parameters["max_handlebar_angle"]["value"],
                        "max_roll_angle": self.parameters["max_roll_angle"]["value"]
                    },
                    "system": {
                        "enable_logging": 1,
                        "enable_preview": 1,
                        "config_reload_interval": 10,
                        "filter_size": 5
                    }
                },
                "physics": {
                    "parameters": {
                        key: param["value"] for key, param in self.physics_parameters.items()
                    },
                    "effects": {
                        key: effect["value"] for key, effect in self.physics_effects.items()
                    }
                }
            }
            
            with open(self.config_file, 'w') as f:
                json.dump(config_data, f, indent=4)
                
            self.status_text.set(f"Konfiguration in {self.config_file} gespeichert")
            
        except Exception as e:
            messagebox.showerror("Fehler", f"Fehler beim Speichern der Konfiguration: {str(e)}")
            
    def reset_defaults(self):
        """Setze alle Parameter auf Standardwerte zurück"""
        defaults = {
            "angle_Kp": 10.0, "angle_Ki": 0.0, "angle_Kd": 2.2,
            "angle_output_min": -0.3, "angle_output_max": 0.3,
            "base_speed": 5.0, "min_speed": 3.0, "max_speed": 8.0, "stability_reduction": 0.5,
            "max_handlebar_angle": 0.32, "max_roll_angle": 45.0
        }
        
        for key, value in defaults.items():
            if key in self.parameters:
                self.parameters[key]["value"] = value
                if key in self.param_widgets:
                    self.param_widgets[key]["var"].set(value)
                    unit = self.parameters[key]["unit"]
                    self.param_widgets[key]["value_label"].config(text=f"{value:.3f} {unit}")
        
        self.status_text.set("Standardwerte wiederhergestellt")
        
    def load_preset(self, preset_params):
        """Lade ein Preset"""
        for key, value in preset_params.items():
            if key in self.parameters:
                self.parameters[key]["value"] = value
                if key in self.param_widgets:
                    self.param_widgets[key]["var"].set(value)
                    unit = self.parameters[key]["unit"]
                    self.param_widgets[key]["value_label"].config(text=f"{value:.3f} {unit}")
        
        self.status_text.set("Preset geladen")
        
    def apply_config(self):
        """Wende Konfiguration sofort an (speichern + neu laden)"""
        self.save_config()
        self.status_text.set("Konfiguration angewendet - Controller lädt automatisch neu")
        
    def open_log_file(self):
        """Öffne eine Log-Datei zum Anzeigen"""
        filename = filedialog.askopenfilename(
            title="Log-Datei öffnen",
            initialdir=self.monitoring_dir,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                self.load_and_plot_data(filename)
                self.status_vars["log_file"].set(os.path.basename(filename))
            except Exception as e:
                messagebox.showerror("Fehler", f"Fehler beim Laden der Log-Datei: {str(e)}")
                
    def load_and_plot_data(self, filename):
        """Lade und plotte Daten aus Log-Datei"""
        df = pd.read_csv(filename)
        
        # Daten für Live-Update speichern
        self.current_data = df
        
        # Zoom zurücksetzen beim Laden neuer Daten
        if hasattr(self, 'zoom_start'):
            self.reset_zoom()
        
        # Plot aktualisieren
        self.refresh_plots()
        
    def refresh_plots(self):
        """Aktualisiere Plots basierend auf aktuellen Daten und Sichtbarkeit"""
        if not hasattr(self, 'current_data') or self.current_data is None:
            return
            
        # Fallback für Sichtbarkeitseinstellungen (alle anzeigen falls nicht initialisiert)
        if not hasattr(self, 'plot_visibility'):
            show_all = True
        else:
            show_all = False
            
        df = self.current_data.copy()
        
        # Zoom-Bereich aktualisieren
        self.update_zoom_range()
        
        # Zoom anwenden wenn nicht der komplette Bereich (0-100%) gewählt ist
        if hasattr(self, 'zoom_start') and (self.zoom_start.get() != 0.0 or self.zoom_end.get() != 100.0):
            min_time = df["timestamp"].min()
            max_time = df["timestamp"].max()
            time_range = max_time - min_time
            
            # Berechne Zoom-Grenzen
            start_percent = self.zoom_start.get() / 100.0
            end_percent = self.zoom_end.get() / 100.0
            
            zoom_start_time = min_time + (start_percent * time_range)
            zoom_end_time = min_time + (end_percent * time_range)
            
            # Filtere Daten im Zoom-Bereich
            mask = (df["timestamp"] >= zoom_start_time) & (df["timestamp"] <= zoom_end_time)
            df = df[mask]
            
            if len(df) == 0:
                # Keine Daten im Zoom-Bereich
                self.ax1.clear()
                self.ax2.clear()
                self.ax1.text(0.5, 0.5, "Keine Daten im gewählten Bereich", 
                             transform=self.ax1.transAxes, ha='center', va='center')
                self.ax2.text(0.5, 0.5, "Keine Daten im gewählten Bereich", 
                             transform=self.ax2.transAxes, ha='center', va='center')
                self.canvas.draw()
                return
        
        # Plot 1: Roll-Winkel und Lenkwinkel
        self.ax1.clear()
        
        if show_all or self.plot_visibility["roll_angle"].get():
            self.ax1.plot(df["timestamp"], df["roll_angle"] * 180/3.14159, label="Roll-Winkel", color="red", linewidth=2)
            
        if show_all or self.plot_visibility["steering_output"].get():
            self.ax1.plot(df["timestamp"], df["steering_output"] * 180/3.14159, label="Lenkwinkel", color="blue", linewidth=2)
        
        self.ax1.set_ylabel("Winkel [°]")
        self.ax1.legend()
        self.ax1.grid(True, alpha=0.3)
        
        # Plot 2: PID-Terme
        self.ax2.clear()
        
        if show_all or self.plot_visibility["p_term"].get():
            self.ax2.plot(df["timestamp"], df["p_term"], label="P-Term", color="green", linewidth=2)
            
        if show_all or self.plot_visibility["i_term"].get():
            self.ax2.plot(df["timestamp"], df["i_term"], label="I-Term", color="orange", linewidth=2)
            
        if show_all or self.plot_visibility["d_term"].get():
            self.ax2.plot(df["timestamp"], df["d_term"], label="D-Term", color="purple", linewidth=2)
        
        self.ax2.set_xlabel("Zeit [s]")
        self.ax2.set_ylabel("PID-Terme")
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.3)
        
        # Zoom-Titel hinzufügen wenn gezoomt
        if hasattr(self, 'zoom_start') and (self.zoom_start.get() != 0.0 or self.zoom_end.get() != 100.0):
            start_time = df["timestamp"].min() if len(df) > 0 else 0
            end_time = df["timestamp"].max() if len(df) > 0 else 0
            self.ax1.set_title(f"Gezoomter Bereich: {start_time:.2f}s - {end_time:.2f}s", fontsize=10)
        else:
            self.ax1.set_title("")
        
        self.canvas.draw()
        
    def update_plot_visibility(self):
        """Handler für Checkbox-Änderungen - aktualisiert die Plot-Sichtbarkeit"""
        self.refresh_plots()
        
    def on_zoom_start_change(self, value):
        """Callback für Start-Slider Änderung"""
        start_percent = float(value)
        
        # Stelle sicher, dass Start < End
        if start_percent >= self.zoom_end.get():
            self.zoom_start.set(self.zoom_end.get() - 1.0)
            start_percent = self.zoom_start.get()
        
        # Berechne tatsächliche Zeit
        if hasattr(self, 'data_duration'):
            start_time = (start_percent / 100.0) * self.data_duration
            self.start_label.config(text=f"{start_time:.1f}s")
        
        self.refresh_plots()
            
    def on_zoom_end_change(self, value):
        """Callback für End-Slider Änderung"""
        end_percent = float(value)
        
        # Stelle sicher, dass End > Start
        if end_percent <= self.zoom_start.get():
            self.zoom_end.set(self.zoom_start.get() + 1.0)
            end_percent = self.zoom_end.get()
        
        # Berechne tatsächliche Zeit
        if hasattr(self, 'data_duration'):
            end_time = (end_percent / 100.0) * self.data_duration
            self.end_label.config(text=f"{end_time:.1f}s")
        
        self.refresh_plots()
            
    def reset_zoom(self):
        """Setzt den Zoom zurück (zeigt alle Daten)"""
        self.zoom_start.set(0.0)
        self.zoom_end.set(100.0)
        
        # Labels aktualisieren
        self.start_label.config(text="0.0s")
        if hasattr(self, 'data_duration'):
            self.end_label.config(text=f"{self.data_duration:.1f}s")
        else:
            self.end_label.config(text="10.0s")
        
        self.status_text.set("Zoom zurückgesetzt")
        self.refresh_plots()
        
    def update_zoom_range(self):
        """Aktualisiert den Zoom-Bereich basierend auf aktuellen Daten"""
        if hasattr(self, 'current_data') and self.current_data is not None:
            df = self.current_data
            if len(df) > 0:
                min_time = df["timestamp"].min()
                max_time = df["timestamp"].max()
                self.data_duration = max_time - min_time
                
                # Labels aktualisieren wenn auf Vollbereich (0-100%)
                if self.zoom_start.get() == 0.0 and self.zoom_end.get() == 100.0:
                    self.start_label.config(text=f"{min_time:.1f}s")
                    self.end_label.config(text=f"{max_time:.1f}s")
        
    def start_live_plot(self):
        """Starte Live-Plot (sucht nach neuesten Log-Dateien)"""
        self.live_plot_active = True
        self.status_text.set("Live-Plot gestartet")
        threading.Thread(target=self.live_plot_worker, daemon=True).start()
        
    def stop_live_plot(self):
        """Stoppe Live-Plot"""
        self.live_plot_active = False
        self.status_text.set("Live-Plot gestoppt")
        
    def live_plot_worker(self):
        """Worker-Thread für Live-Plot"""
        while self.live_plot_active:
            try:
                # Finde neueste Log-Datei
                if os.path.exists(self.monitoring_dir):
                    log_files = [f for f in os.listdir(self.monitoring_dir) if f.endswith('.csv')]
                    if log_files:
                        latest_file = max(log_files, key=lambda f: os.path.getctime(os.path.join(self.monitoring_dir, f)))
                        full_path = os.path.join(self.monitoring_dir, latest_file)
                        
                        # Lade Daten direkt und aktualisiere
                        df = pd.read_csv(full_path)
                        self.current_data = df
                        
                        # Thread-sicheres Update über Tkinter
                        self.root.after(0, self.refresh_plots)
                        self.status_vars["last_update"].set(datetime.now().strftime("%H:%M:%S"))
                        
            except Exception as e:
                print(f"Live-Plot Fehler: {e}")
                
            time.sleep(2)  # Update alle 2 Sekunden
            
    # Neue Physik-Funktionen
    def reset_physics_defaults(self):
        """Setze Physik-Parameter auf Standardwerte zurück"""
        defaults = {
            "bicycle_mass": 3.5, "wheel_radius": 0.055, "frontal_area": 0.3,
            "air_density": 1.225, "drag_coefficient": 0.9, "side_area": 0.4,
            "tire_stiffness": 2000.0, "rolling_resistance_coeff": 0.005, "tire_friction_coeff": 0.8,
            "imu_noise_sigma": 0.01, "imu_delay_samples": 2,
            "wind_speed": 0.0, "wind_direction": 0.0, "wind_turbulence": 0.1, "road_slope": 0.0
        }
        
        for key, value in defaults.items():
            if key in self.physics_parameters:
                self.physics_parameters[key]["value"] = value
                if key in self.physics_widgets:
                    self.physics_widgets[key]["var"].set(value)
                    unit = self.physics_parameters[key]["unit"]
                    if key == "imu_delay_samples":
                        self.physics_widgets[key]["value_label"].config(text=f"{int(value)} {unit}")
                    else:
                        self.physics_widgets[key]["value_label"].config(text=f"{value:.3f} {unit}")
        
        # Alle Effekte aktivieren
        for key in self.physics_effects:
            self.physics_effects[key]["value"] = True
            if key in self.effects_widgets:
                self.effects_widgets[key].set(True)
        
        self.status_text.set("Physik-Standardwerte wiederhergestellt")
        
    def apply_physics_live(self):
        """Wende Physik-Konfiguration sofort an"""
        self.save_config()
        self.status_text.set("Physik-Konfiguration live angewendet")
        
    def update_physics_status(self):
        """Aktualisiere Physik-Status-Anzeige"""
        try:
            status_text = "=== PHYSIK-STATUS ===\n"
            
            # Effekte
            status_text += "Aktive Effekte:\n"
            for key, effect in self.physics_effects.items():
                if effect["value"]:
                    status_text += f"✓ {effect['name']}\n"
                else:
                    status_text += f"✗ {effect['name']}\n"
                    
            status_text += "\nUmwelt:\n"
            wind_speed = self.physics_parameters["wind_speed"]["value"]
            wind_dir = self.physics_parameters["wind_direction"]["value"] * 180 / 3.14159
            road_slope = self.physics_parameters["road_slope"]["value"] * 180 / 3.14159
            
            status_text += f"Wind: {wind_speed:.1f} m/s @ {wind_dir:.0f}°\n"
            status_text += f"Neigung: {road_slope:.1f}°\n"
            
            # Sensoren
            noise = self.physics_parameters["imu_noise_sigma"]["value"]
            delay = self.physics_parameters["imu_delay_samples"]["value"]
            status_text += f"IMU-Noise: {noise:.3f} rad\n"
            status_text += f"IMU-Delay: {delay} samples\n"
            
            self.physics_status_text.delete(1.0, tk.END)
            self.physics_status_text.insert(1.0, status_text)
            
        except Exception as e:
            print(f"Status-Update Fehler: {e}")
            
        # Alle 3 Sekunden aktualisieren
        self.root.after(3000, self.update_physics_status)
        
    # Szenarien-Funktionen
    def scenario_calm(self):
        """Ruhige Bedingungen"""
        self.physics_parameters["wind_speed"]["value"] = 0.0
        self.physics_parameters["wind_turbulence"]["value"] = 0.0
        self.physics_parameters["road_slope"]["value"] = 0.0
        self.physics_parameters["imu_noise_sigma"]["value"] = 0.005
        self.update_physics_widgets()
        self.status_text.set("Szenario: Ruhige Bedingungen")
        
    def scenario_light_wind(self):
        """Leichter Wind"""
        self.physics_parameters["wind_speed"]["value"] = 3.0
        self.physics_parameters["wind_direction"]["value"] = 1.57  # 90°
        self.physics_parameters["wind_turbulence"]["value"] = 0.2
        self.update_physics_widgets()
        self.status_text.set("Szenario: Leichter Seitenwind")
        
    def scenario_strong_wind(self):
        """Starker Seitenwind"""
        self.physics_parameters["wind_speed"]["value"] = 8.0
        self.physics_parameters["wind_direction"]["value"] = 1.57  # 90°
        self.physics_parameters["wind_turbulence"]["value"] = 0.6
        self.update_physics_widgets()
        self.status_text.set("Szenario: Starker Seitenwind")
        
    def scenario_downhill(self):
        """Steile Abfahrt"""
        self.physics_parameters["road_slope"]["value"] = 0.1  # 5.7°
        self.physics_parameters["wind_speed"]["value"] = 0.0
        self.physics_parameters["rolling_resistance_coeff"]["value"] = 0.008
        self.update_physics_widgets()
        self.status_text.set("Szenario: Steile Abfahrt")
        
    def scenario_extreme(self):
        """Extrembedingungen"""
        self.physics_parameters["wind_speed"]["value"] = 12.0
        self.physics_parameters["wind_turbulence"]["value"] = 0.8
        self.physics_parameters["road_slope"]["value"] = 0.08
        self.physics_parameters["imu_noise_sigma"]["value"] = 0.02
        self.update_physics_widgets()
        self.status_text.set("Szenario: Extrembedingungen")
        
    def scenario_sensor_errors(self):
        """Sensorfehler-Test"""
        self.physics_parameters["imu_noise_sigma"]["value"] = 0.05
        self.physics_parameters["imu_delay_samples"]["value"] = 5
        self.physics_parameters["wind_speed"]["value"] = 2.0
        self.update_physics_widgets()
        self.status_text.set("Szenario: Sensorfehler-Test")
        
    def update_physics_widgets(self):
        """Aktualisiere alle Physik-Widgets nach Szenario-Änderung"""
        for key, widget_data in self.physics_widgets.items():
            value = self.physics_parameters[key]["value"]
            widget_data["var"].set(value)
            unit = self.physics_parameters[key]["unit"]
            if key == "imu_delay_samples":
                widget_data["value_label"].config(text=f"{int(value)} {unit}")
            else:
                widget_data["value_label"].config(text=f"{value:.3f} {unit}")

def main():
    root = tk.Tk()
    app = BalanceControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main() 