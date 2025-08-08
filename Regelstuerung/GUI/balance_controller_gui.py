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
import locale

# Einfache i18n-Unterstützung über .arb-Dateien (DE/EN)
try:
    from i18n import t, set_locale
except Exception:
    # Fallbacks, falls i18n-Modul noch nicht vorhanden ist (wird im Projekt mitgeliefert)
    def t(key, **kwargs):
        # Zeige Key als Text, falls Übersetzung fehlt
        return key.format(**kwargs) if kwargs else key
    def set_locale(lang):
        pass

class CheckboxPopup:
    """Popup-Fenster für Checkbox-Gruppen"""
    def __init__(self, parent, title, checkboxes, plot_visibility, update_callback):
        self.parent = parent
        self.plot_visibility = plot_visibility
        self.update_callback = update_callback
        
        # Popup-Fenster erstellen
        self.popup = tk.Toplevel(parent)
        self.popup.title(title)
        self.popup.geometry("400x300")
        self.popup.resizable(False, False)
        
        # Popup zentrieren
        self.popup.transient(parent)
        self.popup.grab_set()
        
        # Hauptframe
        main_frame = ttk.Frame(self.popup)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Titel
        title_label = ttk.Label(main_frame, text=title, font=("Arial", 12, "bold"))
        title_label.pack(pady=(0, 10))
        
        # Scrollbarer Frame für Checkboxen
        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Checkboxen erstellen
        for key, label in checkboxes:
            checkbox = ttk.Checkbutton(
                scrollable_frame,
                text=label,
                variable=self.plot_visibility[key],
                command=self.update_callback
            )
            checkbox.pack(anchor="w", pady=2, padx=10)
        
        # Scrollbaren Frame einrichten
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Button-Frame
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Alle auswählen / Alle abwählen Buttons
        ttk.Button(button_frame, text="Alle auswählen", 
                  command=lambda: self.select_all(checkboxes, True)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Alle abwählen", 
                  command=lambda: self.select_all(checkboxes, False)).pack(side=tk.LEFT, padx=5)
        
        # Schließen-Button
        ttk.Button(button_frame, text="Schließen", 
                  command=self.popup.destroy).pack(side=tk.RIGHT, padx=5)
        
        # ESC zum Schließen
        self.popup.bind("<Escape>", lambda e: self.popup.destroy())
        
        # Popup zentrieren
        self.center_popup()
        
    def center_popup(self):
        """Zentriere das Popup-Fenster"""
        self.popup.update_idletasks()
        
        # Hauptfenster-Position und -Größe
        parent_x = self.parent.winfo_rootx()
        parent_y = self.parent.winfo_rooty()
        parent_width = self.parent.winfo_width()
        parent_height = self.parent.winfo_height()
        
        # Popup-Größe
        popup_width = self.popup.winfo_width()
        popup_height = self.popup.winfo_height()
        
        # Zentrierte Position berechnen
        x = parent_x + (parent_width // 2) - (popup_width // 2)
        y = parent_y + (parent_height // 2) - (popup_height // 2)
        
        self.popup.geometry(f"+{x}+{y}")
        
    def select_all(self, checkboxes, state):
        """Alle Checkboxen auswählen oder abwählen"""
        for key, _ in checkboxes:
            self.plot_visibility[key].set(state)
        self.update_callback()

class BalanceControllerGUI:
    def __init__(self, root):
        self.root = root
        # Locale erkennen (de als Standard)
        sys_locale = (locale.getdefaultlocale() or ("de_DE", None))[0] or "de_DE"
        lang = "de" if sys_locale.lower().startswith("de") else "en"
        set_locale(lang)

        self.root.title(t("app.title"))
        self.root.geometry("1200x800")
        
        # Konfigurationsdatei
        self.config_file = "balance_config.json"
        self.monitoring_dir = "../Monitoring"
        
        # Parameter-Definitionen (mit Limits); Texte werden über i18n geladen
        self.parameters = {
            # Angle PID Parameter
            "angle_Kp": {"value": 10.0, "min": 0.0, "max": 50.0, "unit": ""},
            "angle_Ki": {"value": 0.0, "min": 0.0, "max": 10.0, "unit": ""},
            "angle_Kd": {"value": 2.2, "min": 0.0, "max": 10.0, "unit": ""},
            "angle_output_min": {"value": -0.3, "min": -1.57, "max": 0.0, "unit": "rad"},
            "angle_output_max": {"value": 0.3, "min": 0.0, "max": 1.57, "unit": "rad"},

            # Speed Control Parameter
            "base_speed": {"value": 5.0, "min": 1.0, "max": 15.0, "unit": "rad/s"},
            "min_speed": {"value": 3.0, "min": 0.5, "max": 10.0, "unit": "rad/s"},
            "max_speed": {"value": 8.0, "min": 5.0, "max": 20.0, "unit": "rad/s"},
            "stability_reduction": {"value": 0.5, "min": 0.0, "max": 1.0, "unit": ""},

            # Mechanical Limits
            "max_handlebar_angle": {"value": 0.5, "min": 0.1, "max": 0.8, "unit": "rad"},
            "max_roll_angle": {"value": 45.0, "min": 10.0, "max": 90.0, "unit": "°"}
        }
        
        # Physik-Simulation wurde entfernt
        
        # GUI-Komponenten
        self.param_widgets = {}
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
        notebook.add(param_frame, text=t("tabs.parameters"))
        self.setup_parameter_tab(param_frame)
        
        # Tab 2: Monitoring
        monitor_frame = ttk.Frame(notebook)
        notebook.add(monitor_frame, text="Monitoring")
        self.setup_monitoring_tab(monitor_frame)
        
        # Tab 3: Presets (Physik-Tab wurde entfernt)
        preset_frame = ttk.Frame(notebook)
        notebook.add(preset_frame, text="Presets")
        self.setup_preset_tab(preset_frame)
        
        # Tab 4: Build & Compiler
        build_frame = ttk.Frame(notebook)
        notebook.add(build_frame, text="Build & Compiler")
        self.setup_build_tab(build_frame)
        
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
        
        # Parameter-Gruppen (Texte via i18n)
        groups = {
            t("groups.angle_pid"): ["angle_Kp", "angle_Ki", "angle_Kd", "angle_output_min", "angle_output_max"],
            t("groups.speed_control"): ["base_speed", "min_speed", "max_speed", "stability_reduction"],
            t("groups.mechanical_limits"): ["max_handlebar_angle", "max_roll_angle"]
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
                
                # Label (Name aus i18n)
                label = ttk.Label(scrollable_frame, text=t(f"params.{param_key}.name"))
                label.grid(row=row, column=0, sticky="w", padx=(20, 10), pady=2)
                
                # Textfeld statt Slider
                var = tk.StringVar(value=f"{param['value']}")
                entry = ttk.Entry(scrollable_frame, textvariable=var, width=16)
                entry.grid(row=row, column=1, padx=10, pady=2, sticky="w")

                # Live-Validierung
                var.trace_add('write', lambda *_args, key=param_key: self.on_parameter_text_change(key))

                # Wert-Anzeige
                value_label = ttk.Label(scrollable_frame, text=f"{param['value']:.3f} {param['unit']}")
                value_label.grid(row=row, column=2, padx=10, pady=2)
                
                # Beschreibung
                desc_label = ttk.Label(scrollable_frame, text=t(f"params.{param_key}.description"), foreground="gray")
                desc_label.grid(row=row, column=3, sticky="w", padx=10, pady=2)

                # Fehler-Label
                error_label = ttk.Label(scrollable_frame, text="", foreground="red")
                error_label.grid(row=row+1, column=1, columnspan=2, sticky="w", padx=10, pady=(0, 6))
                
                # Widgets speichern
                self.param_widgets[param_key] = {
                    "var": var,
                    "entry": entry,
                    "value_label": value_label,
                    "error_label": error_label
                }
                
                row += 2
        
        # Control-Buttons
        button_frame = ttk.Frame(scrollable_frame)
        button_frame.grid(row=row, column=0, columnspan=4, pady=20)
        
        ttk.Button(button_frame, text=t("buttons.load_config"), command=self.load_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text=t("buttons.save_config"), command=self.save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text=t("buttons.defaults"), command=self.reset_defaults).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text=t("buttons.apply"), command=self.apply_config).pack(side=tk.LEFT, padx=5)
        
        # Scrollbaren Frame einrichten
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
    # Physik-Tab wurde entfernt
        
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
        
        # Kompakte Popup-Buttons für Checkbox-Gruppen
        popup_frame = ttk.Frame(control_frame)
        popup_frame.pack(side=tk.LEFT, padx=10)
        
        ttk.Label(popup_frame, text="Anzeigen:", font=("Arial", 9, "bold")).pack(anchor="w", padx=5)
        
        # Checkbox-Variablen initialisieren (erweitert um Vision-Control-Daten)
        self.plot_visibility = {
            # Balance-Controller-Daten
            "roll_angle": tk.BooleanVar(value=True),
            "steering_output": tk.BooleanVar(value=True),
            "final_steer": tk.BooleanVar(value=True),
            "actual_handlebar_angle": tk.BooleanVar(value=False),
            "target_speed": tk.BooleanVar(value=False),
            "actual_speed_kmh": tk.BooleanVar(value=True),
            "p_term": tk.BooleanVar(value=True),
            "i_term": tk.BooleanVar(value=True),
            "d_term": tk.BooleanVar(value=True),
            # Vision-Controller-Daten
            "vision_error": tk.BooleanVar(value=False),
            "vision_steer_command": tk.BooleanVar(value=False),
            "vision_speed_command": tk.BooleanVar(value=False)
        }
        
        # Checkbox-Gruppen definieren
        self.balance_checkboxes = [
            ("roll_angle", "Roll-Winkel"),
            ("steering_output", "⚖️ Balance-Steer"),
            ("final_steer", "🎯 Final-Steer"),
            ("actual_handlebar_angle", "📏 Sensor-Winkel"),
            ("target_speed", "🎯 Ziel-Speed"),
            ("actual_speed_kmh", "📊 Ist-Speed (km/h)")
        ]
        
        self.pid_checkboxes = [
            ("p_term", "P-Term (Proportional)"),
            ("i_term", "I-Term (Integral)"),
            ("d_term", "D-Term (Differential)")
        ]
        
        self.vision_checkboxes = [
            ("vision_error", "🎯 Vision Error"),
            ("vision_steer_command", "🎮 Vision-Steer Command"),
            ("vision_speed_command", "⚡ Vision Speed Command")
        ]
        
        # Popup-Buttons in einer Zeile
        button_row = ttk.Frame(popup_frame)
        button_row.pack(fill=tk.X, pady=2)
        
        # Balance-Popup-Button
        self.balance_btn = ttk.Button(
            button_row,
            text="⚖️ Balance...",
            command=lambda: self.show_checkbox_popup("Balance Controller", self.balance_checkboxes),
            width=15
        )
        self.balance_btn.pack(side=tk.LEFT, padx=2)
        
        # Vision-Popup-Button
        self.vision_btn = ttk.Button(
            button_row,
            text="👁️ Vision...",
            command=lambda: self.show_checkbox_popup("Vision Controller", self.vision_checkboxes),
            width=15
        )
        self.vision_btn.pack(side=tk.LEFT, padx=2)
        
        # Sensor/Aktor-Popup-Button (PID-Terme)
        self.sensor_btn = ttk.Button(
            button_row,
            text="🔧 PID/Sensor...",
            command=lambda: self.show_checkbox_popup("PID-Terme & Sensoren", self.pid_checkboxes),
            width=17
        )
        self.sensor_btn.pack(side=tk.LEFT, padx=2)
        
        # Initialisiere Button-Texte mit Zählern
        self.update_button_counters()
        
        # Separator für Zoom-Controls
        zoom_separator = ttk.Separator(control_frame, orient="vertical")
        zoom_separator.pack(side=tk.LEFT, fill=tk.Y, padx=10)
        
        # X-Achse Zoom Controls
        zoom_frame = ttk.Frame(control_frame)
        zoom_frame.pack(side=tk.LEFT, padx=10)
        
        ttk.Label(zoom_frame, text="X-Achse:").pack(side=tk.LEFT, padx=5)
        
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
        
        # Plot-Daten initialisieren (erweitert um Vision-Control-Daten)
        self.plot_data = {
            # Balance-Controller-Daten
            "time": [],
            "roll_angle": [],
            "steering_output": [],
            "final_steer": [],
            "actual_handlebar_angle": [],
            "speed": [],
            "p_term": [],
            "i_term": [],
            "d_term": [],
            # Vision-Controller-Daten
            "vision_error": [],
            "vision_steer_command": [],
            "vision_speed_command": []
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
        
    def setup_build_tab(self, parent):
        """Erstelle das Build & Compiler-Tab"""
        
        # Hauptcontainer
        main_frame = ttk.Frame(parent)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Oberer Bereich: Build-Kontrollen
        control_frame = ttk.LabelFrame(main_frame, text="Build-Kontrollen")
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Build-Buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(button_frame, text="🔨 Build Balance Controller", 
                  command=self.build_balance_controller).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🔧 Build Vision Controller", 
                  command=self.build_vision_controller).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🧹 Clean Build", 
                  command=self.clean_build).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="📊 Build Status", 
                  command=self.check_build_status).pack(side=tk.LEFT, padx=5)
        
        # Build-Status-Anzeige
        status_frame = ttk.Frame(control_frame)
        status_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
        
        ttk.Label(status_frame, text="Build-Status:").pack(side=tk.LEFT)
        self.build_status_text = tk.StringVar(value="Unbekannt")
        self.build_status_label = ttk.Label(status_frame, textvariable=self.build_status_text, 
                                          font=("Arial", 10, "bold"))
        self.build_status_label.pack(side=tk.LEFT, padx=10)
        
        # Build-Log-Bereich
        log_frame = ttk.LabelFrame(main_frame, text="Build-Log")
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        # Scrollbares Textfeld für Build-Output
        text_frame = ttk.Frame(log_frame)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.build_log_text = tk.Text(text_frame, height=20, wrap=tk.WORD, 
                                     font=("Courier", 10))
        log_scrollbar = ttk.Scrollbar(text_frame, orient="vertical", 
                                     command=self.build_log_text.yview)
        self.build_log_text.configure(yscrollcommand=log_scrollbar.set)
        
        self.build_log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Log-Kontrollen
        log_control_frame = ttk.Frame(log_frame)
        log_control_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
        
        ttk.Button(log_control_frame, text="📋 Log löschen", 
                  command=self.clear_build_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(log_control_frame, text="💾 Log speichern", 
                  command=self.save_build_log).pack(side=tk.LEFT, padx=5)
        
        # Automatische Überwachung
        self.build_monitor_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(log_control_frame, text="🔄 Automatische Überwachung", 
                       variable=self.build_monitor_var,
                       command=self.toggle_build_monitoring).pack(side=tk.RIGHT, padx=5)
        
        # Initialen Build-Status prüfen
        self.check_build_status()
        
        # Automatische Überwachung starten
        self.start_build_monitoring()
        
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
        """Aktualisiere die Zeit-Anzeige mit Millisekunden-Präzision"""
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Millisekunden (3 Stellen)
        self.time_text.set(current_time)
        self.root.after(10, self.update_time)  # Update alle 10ms für Echtzeit-Gefühl
        
    def on_parameter_text_change(self, param_key):
        """Validiere Texteingabe und aktualisiere Parameter & Anzeige."""
        text = self.param_widgets[param_key]["var"].get().strip()
        error_label = self.param_widgets[param_key]["error_label"]
        unit = self.parameters[param_key]["unit"]

        if text == "" or text in {"-", ".", "-.", ","}:
            # Zwischenzustände beim Tippen erlauben
            error_label.config(text="")
            return

        try:
            # Ersetze Komma durch Punkt für DE-Eingaben
            value = float(text.replace(",", "."))
        except ValueError:
            error_label.config(text=t("errors.invalid_number"))
            return

        # Bereich wird nicht geprüft – nur Format
        error_label.config(text="")
        self.parameters[param_key]["value"] = value
        self.param_widgets[param_key]["value_label"].config(text=f"{value:.3f} {unit}")
        self.status_text.set(t("status.param_changed", key=param_key, value=f"{value:.3f}"))
            
    # Physik-Handler-Funktionen wurden entfernt
            
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
                                self.param_widgets[key]["var"].set(str(value))
                                unit = self.parameters[key]["unit"]
                                self.param_widgets[key]["value_label"].config(text=f"{value:.3f} {unit}")
                
                # Physik-Parameter-Laden wurde entfernt
                
                self.status_text.set(t("status.config_loaded", file=self.config_file))
            else:
                self.status_text.set(t("status.config_not_found", file=self.config_file))
                
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
                }
                # Physik-Konfiguration wurde entfernt
            }
            
            with open(self.config_file, 'w') as f:
                json.dump(config_data, f, indent=4)
                
            self.status_text.set(t("status.config_saved", file=self.config_file))
            
        except Exception as e:
            messagebox.showerror(t("errors.error"), t("errors.save_failed", err=str(e)))
            
    def reset_defaults(self):
        """Setze alle Parameter auf Standardwerte zurück"""
        defaults = {
            "angle_Kp": 10.0, "angle_Ki": 0.0, "angle_Kd": 2.2,
            "angle_output_min": -0.3, "angle_output_max": 0.3,
            "base_speed": 5.0, "min_speed": 3.0, "max_speed": 8.0, "stability_reduction": 0.5,
            "max_handlebar_angle": 0.5, "max_roll_angle": 45.0
        }
        
        for key, value in defaults.items():
            if key in self.parameters:
                self.parameters[key]["value"] = value
                if key in self.param_widgets:
                    self.param_widgets[key]["var"].set(str(value))
                    unit = self.parameters[key]["unit"]
                    self.param_widgets[key]["value_label"].config(text=f"{value:.3f} {unit}")
        
        self.status_text.set(t("status.defaults_restored"))
        
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
        # Vor dem Anwenden alle Felder validieren
        invalid = []
        for key in self.parameters.keys():
            self.on_parameter_text_change(key)
            if self.param_widgets[key]["error_label"].cget("text"):
                invalid.append(key)

        if invalid:
            messagebox.showerror(t("errors.error"), t("errors.validation_failed"))
            return

        self.save_config()
        self.status_text.set(t("status.config_applied"))
        
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
        
        # Plot 1: Roll-Winkel, Lenkwinkel und Vision-Daten
        self.ax1.clear()
        
        if show_all or self.plot_visibility["roll_angle"].get():
            self.ax1.plot(df["timestamp"], df["roll_angle"] * 180/3.14159, label="Roll-Winkel", color="red", linewidth=2)
            
        if show_all or self.plot_visibility["steering_output"].get():
            self.ax1.plot(df["timestamp"], df["steering_output"] * 180/3.14159, label="⚖️ Balance-Steer", color="blue", linewidth=2, linestyle='-')
        
        if "vision_steer_command" in df.columns and (show_all or self.plot_visibility["vision_steer_command"].get()):
            # Vision Steer Command in Grad umrechnen (vision_steer_command ist bereits in Rad * max_angle)
            self.ax1.plot(df["timestamp"], df["vision_steer_command"] * 180/3.14159, label="🎮 Vision-Steer", color="cyan", linewidth=2, linestyle='--')
        
        if "final_steer" in df.columns and (show_all or self.plot_visibility["final_steer"].get()):
            self.ax1.plot(df["timestamp"], df["final_steer"] * 180/3.14159, label="🎯 Final-Steer", color="darkgreen", linewidth=3, linestyle='-')
        
        if "actual_handlebar_angle" in df.columns and (show_all or self.plot_visibility["actual_handlebar_angle"].get()):
            self.ax1.plot(df["timestamp"], df["actual_handlebar_angle"] * 180/3.14159, label="📏 Sensor-Winkel", color="purple", linewidth=2, linestyle='-.')
        
        # Vision-Error zu Plot 1 hinzufügen (falls verfügbar)
        if "vision_error" in df.columns and (show_all or self.plot_visibility["vision_error"].get()):
            # Vision Error normalisiert auf ±30° für bessere Darstellung
            self.ax1.plot(df["timestamp"], df["vision_error"] * 30, label="🎯 Vision Error (×30)", color="orange", linewidth=2, linestyle=':')
        
        self.ax1.set_ylabel("Winkel [°] / Vision-Werte")
        self.ax1.legend()
        self.ax1.grid(True, alpha=0.3)
        
        # Plot 2: PID-Terme und Vision-Speed-Command
        self.ax2.clear()
        
        if show_all or self.plot_visibility["p_term"].get():
            self.ax2.plot(df["timestamp"], df["p_term"], label="P-Term", color="green", linewidth=2)
            
        if show_all or self.plot_visibility["i_term"].get():
            self.ax2.plot(df["timestamp"], df["i_term"], label="I-Term", color="orange", linewidth=2)
            
        if show_all or self.plot_visibility["d_term"].get():
            self.ax2.plot(df["timestamp"], df["d_term"], label="D-Term", color="purple", linewidth=2)
        
        # Geschwindigkeits-Daten hinzufügen
        if "target_speed" in df.columns and (show_all or self.plot_visibility["target_speed"].get()):
            self.ax2.plot(df["timestamp"], df["target_speed"], label="🎯 Ziel-Speed", color="blue", linewidth=2, linestyle='--')
        
        if "actual_speed_kmh" in df.columns and (show_all or self.plot_visibility["actual_speed_kmh"].get()):
            self.ax2.plot(df["timestamp"], df["actual_speed_kmh"], label="📊 Ist-Speed (km/h)", color="red", linewidth=3, linestyle='-')
        
        # Vision Speed Command hinzufügen (falls verfügbar)
        if "vision_speed_command" in df.columns and (show_all or self.plot_visibility["vision_speed_command"].get()):
            self.ax2.plot(df["timestamp"], df["vision_speed_command"], label="⚡ Vision Speed", color="magenta", linewidth=2, linestyle=':')
        
        self.ax2.set_xlabel("Zeit [s]")
        self.ax2.set_ylabel("PID-Terme / Geschwindigkeit / Vision-Speed")
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
        
    def show_checkbox_popup(self, title, checkboxes):
        """Zeige Popup für Checkbox-Gruppe"""
        CheckboxPopup(self.root, title, checkboxes, self.plot_visibility, self.update_plot_visibility)
        
    def update_button_counters(self):
        """Aktualisiere die Button-Texte mit der Anzahl aktivierter Checkboxen"""
        # Balance-Button
        balance_count = sum(1 for key, _ in self.balance_checkboxes if self.plot_visibility[key].get())
        balance_total = len(self.balance_checkboxes)
        self.balance_btn.config(text=f"⚖️ Balance ({balance_count}/{balance_total})")
        
        # Vision-Button  
        vision_count = sum(1 for key, _ in self.vision_checkboxes if self.plot_visibility[key].get())
        vision_total = len(self.vision_checkboxes)
        self.vision_btn.config(text=f"👁️ Vision ({vision_count}/{vision_total})")
        
        # Sensor-Button (PID)
        sensor_count = sum(1 for key, _ in self.pid_checkboxes if self.plot_visibility[key].get())
        sensor_total = len(self.pid_checkboxes)
        self.sensor_btn.config(text=f"🔧 PID/Sensor ({sensor_count}/{sensor_total})")
        
    def update_plot_visibility(self):
        """Handler für Checkbox-Änderungen - aktualisiert die Plot-Sichtbarkeit"""
        self.update_button_counters()  # Zähler aktualisieren
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
                        
                        # Lade alle verfügbaren Datenpunkte für vollständiges Monitoring
                        df = pd.read_csv(full_path)
                        self.current_data = df
                        
                        # Thread-sicheres Update über Tkinter
                        self.root.after(0, self.refresh_plots)
                        self.status_vars["last_update"].set(datetime.now().strftime("%H:%M:%S.%f")[:-3])  # Millisekunden-Präzision
                        
            except Exception as e:
                print(f"Live-Plot Fehler: {e}")
                
            time.sleep(0.2)  # Update alle 200ms für Echtzeit-Monitoring
            
    # Alle Physik-Funktionen wurden entfernt
    
    # Build-Monitoring-Methoden
    def build_balance_controller(self):
        """Kompiliere den Balance Controller"""
        self.add_build_log("🔨 Starte Balance Controller Build...")
        
        import subprocess
        import threading
        
        def build_thread():
            try:
                # Wechsel in das Controller-Verzeichnis
                controller_dir = "../controllers/balance_control_c"
                
                # Make-Befehl ausführen
                result = subprocess.run(
                    ["make"], 
                    cwd=controller_dir, 
                    capture_output=True, 
                    text=True,
                    timeout=60
                )
                
                if result.returncode == 0:
                    self.add_build_log("✅ Balance Controller erfolgreich kompiliert!")
                    self.build_status_text.set("✅ Erfolgreich")
                    self.build_status_label.config(foreground="green")
                else:
                    self.add_build_log(f"❌ Build-Fehler:\n{result.stderr}")
                    self.build_status_text.set("❌ Fehler")
                    self.build_status_label.config(foreground="red")
                    
                # Ausgabe hinzufügen
                if result.stdout:
                    self.add_build_log(f"Ausgabe:\n{result.stdout}")
                
            except subprocess.TimeoutExpired:
                self.add_build_log("⏱️ Build-Timeout nach 60 Sekunden")
                self.build_status_text.set("⏱️ Timeout")
                self.build_status_label.config(foreground="orange")
            except Exception as e:
                self.add_build_log(f"💥 Unerwarteter Fehler: {str(e)}")
                self.build_status_text.set("💥 Fehler")
                self.build_status_label.config(foreground="red")
        
        # Build in separatem Thread starten
        threading.Thread(target=build_thread, daemon=True).start()
    
    def build_vision_controller(self):
        """Kompiliere den Vision Controller"""
        self.add_build_log("🔧 Vision Controller wird überprüft...")
        
        import subprocess
        import threading
        
        def build_thread():
            try:
                # Prüfe ob Vision Controller (Python) korrekt ist
                vision_file = "../controllers/vision_control_py/vision_control_py.py"
                
                result = subprocess.run(
                    ["python3", "-m", "py_compile", vision_file],
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                
                if result.returncode == 0:
                    self.add_build_log("✅ Vision Controller Syntax OK!")
                    self.check_vision_requirements()
                else:
                    self.add_build_log(f"❌ Vision Controller Syntax-Fehler:\n{result.stderr}")
                    self.build_status_text.set("❌ Fehler")
                    self.build_status_label.config(foreground="red")
                    
            except Exception as e:
                self.add_build_log(f"💥 Vision Controller Fehler: {str(e)}")
        
        threading.Thread(target=build_thread, daemon=True).start()
    
    def check_vision_requirements(self):
        """Prüfe Vision Controller Requirements"""
        try:
            requirements_file = "../controllers/vision_control_py/requirements.txt"
            if os.path.exists(requirements_file):
                self.add_build_log("📋 Prüfe Vision Controller Requirements...")
                
                import subprocess
                result = subprocess.run(
                    ["pip", "check"], 
                    capture_output=True, 
                    text=True
                )
                
                if result.returncode == 0:
                    self.add_build_log("✅ Vision Controller Requirements OK!")
                else:
                    self.add_build_log(f"⚠️ Requirements-Probleme:\n{result.stdout}")
            else:
                self.add_build_log("⚠️ Keine requirements.txt für Vision Controller gefunden")
        except Exception as e:
            self.add_build_log(f"⚠️ Requirements-Prüfung fehlgeschlagen: {str(e)}")
    
    def clean_build(self):
        """Bereinige Build-Dateien"""
        self.add_build_log("🧹 Bereinige Build-Dateien...")
        
        import subprocess
        import threading
        
        def clean_thread():
            try:
                controller_dir = "../controllers/balance_control_c"
                
                result = subprocess.run(
                    ["make", "clean"], 
                    cwd=controller_dir, 
                    capture_output=True, 
                    text=True
                )
                
                if result.returncode == 0:
                    self.add_build_log("✅ Build-Dateien bereinigt!")
                    self.build_status_text.set("🧹 Bereinigt")
                    self.build_status_label.config(foreground="blue")
                else:
                    self.add_build_log(f"⚠️ Clean-Warnung:\n{result.stderr}")
                    
            except Exception as e:
                self.add_build_log(f"💥 Clean-Fehler: {str(e)}")
        
        threading.Thread(target=clean_thread, daemon=True).start()
    
    def check_build_status(self):
        """Prüfe aktuellen Build-Status"""
        self.add_build_log("📊 Prüfe Build-Status...")
        
        # Prüfe ob ausführbare Dateien existieren
        balance_exec = "../controllers/balance_control_c/balance_control_c"
        vision_exec = "../controllers/vision_control_py/vision_control_py.py"
        
        balance_ok = os.path.exists(balance_exec)
        vision_ok = os.path.exists(vision_exec)
        
        if balance_ok and vision_ok:
            self.add_build_log("✅ Alle Controller verfügbar")
            self.build_status_text.set("✅ Vollständig")
            self.build_status_label.config(foreground="green")
        elif balance_ok:
            self.add_build_log("⚠️ Nur Balance Controller verfügbar")
            self.build_status_text.set("⚠️ Teilweise")
            self.build_status_label.config(foreground="orange")
        elif vision_ok:
            self.add_build_log("⚠️ Nur Vision Controller verfügbar")
            self.build_status_text.set("⚠️ Teilweise")
            self.build_status_label.config(foreground="orange")
        else:
            self.add_build_log("❌ Keine Controller verfügbar")
            self.build_status_text.set("❌ Fehlt")
            self.build_status_label.config(foreground="red")
    
    def add_build_log(self, message):
        """Füge Nachricht zum Build-Log hinzu"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.build_log_text.insert(tk.END, log_entry)
        self.build_log_text.see(tk.END)  # Scrolle zur letzten Zeile
        
        # Aktualisiere GUI
        self.root.update_idletasks()
    
    def clear_build_log(self):
        """Lösche Build-Log"""
        self.build_log_text.delete(1.0, tk.END)
        self.add_build_log("📋 Build-Log geleert")
    
    def save_build_log(self):
        """Speichere Build-Log in Datei"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"build_log_{timestamp}.txt"
            
            with open(filename, 'w') as f:
                f.write(self.build_log_text.get(1.0, tk.END))
            
            self.add_build_log(f"💾 Build-Log gespeichert: {filename}")
            
        except Exception as e:
            self.add_build_log(f"💥 Speichern fehlgeschlagen: {str(e)}")
    
    def toggle_build_monitoring(self):
        """Schalte automatische Build-Überwachung um"""
        if self.build_monitor_var.get():
            self.add_build_log("🔄 Automatische Überwachung aktiviert")
            self.start_build_monitoring()
        else:
            self.add_build_log("⏸️ Automatische Überwachung deaktiviert")
            self.stop_build_monitoring()
    
    def start_build_monitoring(self):
        """Starte automatische Build-Überwachung"""
        if not hasattr(self, 'build_monitor_active'):
            self.build_monitor_active = True
            self.build_monitor_loop()
    
    def stop_build_monitoring(self):
        """Stoppe automatische Build-Überwachung"""
        self.build_monitor_active = False
    
    def build_monitor_loop(self):
        """Überwachungsschleife für Build-Änderungen"""
        if hasattr(self, 'build_monitor_active') and self.build_monitor_active and self.build_monitor_var.get():
            # Prüfe alle 5 Sekunden den Build-Status
            self.check_build_status()
            self.root.after(5000, self.build_monitor_loop)  # 5 Sekunden statt 30s

def main():
    root = tk.Tk()
    app = BalanceControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main() 