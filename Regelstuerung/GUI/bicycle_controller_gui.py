import tkinter as tk
from tkinter import ttk
import json
import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from datetime import datetime
import glob

class BicycleControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Selbstbalancierendes Fahrrad - Kontrollzentrum")
        self.root.geometry("1200x800")
        
        # Konfigurationsdatei Pfad
        self.config_file = "controller_config.json"
        self.monitoring_dir = "../Monitoring"
        
        # Stelle sicher, dass der Monitoring-Ordner existiert
        os.makedirs(self.monitoring_dir, exist_ok=True)
        
        # Lade Konfiguration
        self.load_config()
        
        # Erstelle Notebook für Tabs
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Style konfigurieren
        self.configure_styles()
        
        # Erstelle die Tabs
        self.create_controller_tab()
        self.create_monitoring_tab()
        self.create_reset_tab()
        
        # Automatisches Speichern alle 1 Sekunde
        self.auto_save()
        
    def configure_styles(self):
        """Konfiguriere moderne Styles"""
        style = ttk.Style()
        
        # Verfügbare Themes anzeigen und bestes auswählen
        available_themes = style.theme_names()
        if 'aqua' in available_themes:  # macOS
            style.theme_use('aqua')
        elif 'clam' in available_themes:
            style.theme_use('clam')
        else:
            style.theme_use('default')
            
        # Custom Styles
        style.configure('Title.TLabel', font=('Helvetica', 16, 'bold'))
        style.configure('Subtitle.TLabel', font=('Helvetica', 12, 'bold'))
        style.configure('Info.TLabel', font=('Helvetica', 10))
        
    def load_config(self):
        """Lade Konfiguration aus JSON-Datei"""
        try:
            with open(self.config_file, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            # Standard-Konfiguration
            self.config = {
                "Kp": 0.01,
                "Ki": 0.02,
                "Kd": 0.0001,
                "maxS": 5.0,
                "minS": 3.0,
                "hMax": 0.1920,
                "preview": 1
            }
            self.save_config()
    
    def save_config(self):
        """Speichere Konfiguration in JSON-Datei"""
        with open(self.config_file, 'w') as f:
            json.dump(self.config, f, indent=4)
            
    def create_controller_tab(self):
        """Erstelle Tab für Reglereinstellungen"""
        controller_frame = ttk.Frame(self.notebook)
        self.notebook.add(controller_frame, text="Reglereinstellungen")
        
        # Erstelle scrollbaren Bereich mit Text-Widget
        text_widget = tk.Text(controller_frame, wrap=tk.NONE, state=tk.DISABLED, height=1)
        scrollbar_v = ttk.Scrollbar(controller_frame, orient="vertical", command=text_widget.yview)
        text_widget.configure(yscrollcommand=scrollbar_v.set)
        
        # Verstecke Text-Widget (nur für Scrolling)
        text_widget.pack_forget()
        
        # Hauptcontainer
        main_frame = ttk.Frame(controller_frame)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Titel mit Icon
        title_frame = ttk.Frame(main_frame)
        title_frame.pack(fill=tk.X, pady=(0, 20))
        
        title_label = ttk.Label(title_frame, text="⚙️ Reglereinstellungen", style='Title.TLabel')
        title_label.pack(side=tk.LEFT)
        
        # Status-Anzeige
        self.status_label = ttk.Label(title_frame, text="🔄 Automatisches Speichern aktiv", 
                                     style='Info.TLabel', foreground='green')
        self.status_label.pack(side=tk.RIGHT)
        
        # Erstelle zwei Spalten für bessere Organisation
        columns_frame = ttk.Frame(main_frame)
        columns_frame.pack(fill=tk.BOTH, expand=True)
        
        # Linke Spalte
        left_column = ttk.Frame(columns_frame)
        left_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # PID-Regler Sektion
        pid_frame = ttk.LabelFrame(left_column, text="🎯 PID-Regler Parameter", padding=15)
        pid_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.create_parameter_control(pid_frame, "Kp", "Proportional-Verstärkung (Kp)", 0.0, 1.0, 0.001)
        self.create_parameter_control(pid_frame, "Ki", "Integral-Verstärkung (Ki)", 0.0, 1.0, 0.001)
        self.create_parameter_control(pid_frame, "Kd", "Differential-Verstärkung (Kd)", 0.0, 0.01, 0.0001)
        
        # Geschwindigkeits-Sektion
        speed_frame = ttk.LabelFrame(left_column, text="🚀 Geschwindigkeitsparameter", padding=15)
        speed_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.create_parameter_control(speed_frame, "maxS", "Maximale Geschwindigkeit (km/h)", 1.0, 50.0, 0.1)
        self.create_parameter_control(speed_frame, "minS", "Minimale Geschwindigkeit (km/h)", 0.5, 20.0, 0.1)
        
        # Rechte Spalte
        right_column = ttk.Frame(columns_frame)
        right_column.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # Lenker-Sektion
        steering_frame = ttk.LabelFrame(right_column, text="🎯 Lenkungsparameter", padding=15)
        steering_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.create_parameter_control(steering_frame, "hMax", "Maximaler Lenkwinkel (rad)", 0.0, 1.0, 0.001)
        
        # Anzeige-Sektion
        display_frame = ttk.LabelFrame(right_column, text="📱 Anzeigeoptionen", padding=15)
        display_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.create_boolean_control(display_frame, "preview", "🎥 Kamera-Vorschau aktivieren")
        
        # Konfigurationsstatus
        config_status_frame = ttk.LabelFrame(right_column, text="📊 Konfigurationsstatus", padding=15)
        config_status_frame.pack(fill=tk.X, pady=(0, 15))
        
        # Zeige aktuelle Werte aus JSON an
        self.create_config_display(config_status_frame)
        
        # Aktions-Buttons (unten, über beide Spalten)
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=20)
        
        # Separator
        separator = ttk.Separator(button_frame, orient='horizontal')
        separator.pack(fill=tk.X, pady=(0, 15))
        
        # Button-Grid
        save_button = ttk.Button(button_frame, text="💾 Einstellungen speichern", 
                                command=self.manual_save)
        save_button.pack(side=tk.LEFT, padx=5)
        
        load_button = ttk.Button(button_frame, text="🔄 Einstellungen neu laden", 
                                command=self.load_and_refresh)
        load_button.pack(side=tk.LEFT, padx=5)
        
        reset_button = ttk.Button(button_frame, text="🔧 Standard wiederherstellen", 
                                 command=self.reset_to_defaults)
        reset_button.pack(side=tk.LEFT, padx=5)
        
        # Info-Text
        info_text = "💡 Tipp: Änderungen werden automatisch alle 1 Sekunde gespeichert und an den Controller übertragen."
        info_label = ttk.Label(button_frame, text=info_text, style='Info.TLabel')
        info_label.pack(side=tk.RIGHT, padx=10)
        
    def create_parameter_control(self, parent, param_name, label_text, min_val, max_val, step):
        """Erstelle Slider und Textfeld für Parameter"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=10)
        
        # Label
        label = ttk.Label(frame, text=f"{label_text}:", style='Subtitle.TLabel')
        label.pack(anchor=tk.W)
        
        # Aktueller Wert
        current_val = self.config.get(param_name, min_val)
        
        # Wert-Label
        value_var = tk.DoubleVar(value=current_val)
        value_label = ttk.Label(frame, text=f"Aktueller Wert: {current_val:.4f}", style='Info.TLabel')
        value_label.pack(anchor=tk.W, pady=(0, 5))
        
        # Slider
        slider = ttk.Scale(frame, from_=min_val, to=max_val, variable=value_var, 
                          orient=tk.HORIZONTAL, length=400)
        slider.pack(fill=tk.X, pady=(0, 5))
        
        # Textfeld für genaue Eingabe
        entry_frame = ttk.Frame(frame)
        entry_frame.pack(fill=tk.X)
        
        ttk.Label(entry_frame, text="Genauer Wert:").pack(side=tk.LEFT)
        entry = ttk.Entry(entry_frame, width=10)
        entry.pack(side=tk.LEFT, padx=(10, 0))
        entry.insert(0, str(current_val))
        
        # Update-Funktionen
        def update_from_slider(event=None):
            val = value_var.get()
            self.config[param_name] = val
            value_label.config(text=f"Aktueller Wert: {val:.4f}")
            entry.delete(0, tk.END)
            entry.insert(0, f"{val:.4f}")
            
        def update_from_entry(event=None):
            try:
                val = float(entry.get())
                if min_val <= val <= max_val:
                    value_var.set(val)
                    self.config[param_name] = val
                    value_label.config(text=f"Aktueller Wert: {val:.4f}")
                else:
                    entry.delete(0, tk.END)
                    entry.insert(0, f"{value_var.get():.4f}")
            except ValueError:
                entry.delete(0, tk.END)
                entry.insert(0, f"{value_var.get():.4f}")
        
        slider.bind("<Motion>", update_from_slider)
        slider.bind("<ButtonRelease-1>", update_from_slider)
        entry.bind("<Return>", update_from_entry)
        entry.bind("<FocusOut>", update_from_entry)
        
        # Speichere Referenzen
        setattr(self, f"{param_name}_var", value_var)
        setattr(self, f"{param_name}_label", value_label)
        setattr(self, f"{param_name}_entry", entry)
        
    def create_boolean_control(self, parent, param_name, label_text):
        """Erstelle Checkbox für Boolean-Parameter"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=10)
        
        var = tk.BooleanVar(value=bool(self.config.get(param_name, False)))
        
        def update_config():
            self.config[param_name] = 1 if var.get() else 0
            
        checkbox = ttk.Checkbutton(frame, text=label_text, variable=var, command=update_config)
        checkbox.pack(anchor=tk.W)
        
        setattr(self, f"{param_name}_var", var)
        
    def create_monitoring_tab(self):
        """Erstelle Tab für Monitoring"""
        monitoring_frame = ttk.Frame(self.notebook)
        self.notebook.add(monitoring_frame, text="Monitoring")
        
        # Titel mit Icon
        title_frame = ttk.Frame(monitoring_frame)
        title_frame.pack(fill=tk.X, pady=20, padx=20)
        
        title_label = ttk.Label(title_frame, text="📊 Monitoring & Datenanalyse", style='Title.TLabel')
        title_label.pack(side=tk.LEFT)
        
        # Live-Status
        self.monitoring_status = ttk.Label(title_frame, text="📈 Bereit für Datenanalyse", 
                                          style='Info.TLabel', foreground='blue')
        self.monitoring_status.pack(side=tk.RIGHT)
        
        # Hauptcontainer
        main_container = ttk.Frame(monitoring_frame)
        main_container.pack(fill=tk.BOTH, expand=True, padx=20)
        
        # Linke Seite: Datei-Liste
        left_frame = ttk.Frame(main_container)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        ttk.Label(left_frame, text="CSV-Dateien:", style='Subtitle.TLabel').pack(anchor=tk.W)
        
        # Listbox für CSV-Dateien
        listbox_frame = ttk.Frame(left_frame)
        listbox_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.csv_listbox = tk.Listbox(listbox_frame, width=35, height=15)
        csv_scrollbar = ttk.Scrollbar(listbox_frame, orient="vertical", command=self.csv_listbox.yview)
        self.csv_listbox.configure(yscrollcommand=csv_scrollbar.set)
        
        # Automatisches Plotten beim Klick auf CSV-Datei
        self.csv_listbox.bind('<ButtonRelease-1>', lambda event: self.plot_selected_csv())
        self.csv_listbox.bind('<Return>', lambda event: self.plot_selected_csv())
        
        self.csv_listbox.pack(side="left", fill="both", expand=True)
        csv_scrollbar.pack(side="right", fill="y")
        
        # Checkboxen für Datenvisualisierung
        controls_frame = ttk.LabelFrame(left_frame, text="Diagramm-Optionen", padding=10)
        controls_frame.pack(fill=tk.X, pady=10)
        
        # Checkboxen für verschiedene Werte
        self.plot_vars = {}
        plot_options = [
            ('pid_components', 'PID-Komponenten (P, I, D, PID)', True),
            ('speed', 'Geschwindigkeit', True),
            ('handlebar_angle', 'Lenkwinkel', True),
            ('roll_angle', 'Roll-Winkel (Kippwinkel)', True),
            ('error', 'Regelfehler', True),
            ('parameters', 'Parameter (Kp, Ki, Kd)', False)
        ]
        
        for var_name, label, default in plot_options:
            var = tk.BooleanVar(value=default)
            self.plot_vars[var_name] = var
            checkbox = ttk.Checkbutton(controls_frame, text=label, variable=var, 
                                     command=self.update_plot)
            checkbox.pack(anchor=tk.W, pady=2)
        
        # Buttons
        button_frame = ttk.Frame(left_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        refresh_button = ttk.Button(button_frame, text="📊 Aktualisieren", command=self.refresh_csv_list)
        refresh_button.pack(fill=tk.X, pady=2)
        
        export_button = ttk.Button(button_frame, text="💾 Diagramm exportieren", command=self.export_plot)
        export_button.pack(fill=tk.X, pady=2)
        
        # Rechte Seite: Diagramm
        right_frame = ttk.Frame(main_container)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Matplotlib Figure
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Toolbar für Matplotlib
        from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
        toolbar = NavigationToolbar2Tk(self.canvas, right_frame)
        toolbar.update()
        
        # CSV-Liste initial laden
        self.refresh_csv_list()
        
    def create_reset_tab(self):
        """Erstelle Tab für Zurücksetzen"""
        reset_frame = ttk.Frame(self.notebook)
        self.notebook.add(reset_frame, text="Zurücksetzen")
        
        # Zentrierter Container
        center_frame = ttk.Frame(reset_frame)
        center_frame.pack(expand=True)
        
        # Titel
        title_label = ttk.Label(center_frame, text="🔧 System zurücksetzen", style='Title.TLabel')
        title_label.pack(pady=20)
        
        # Beschreibung
        desc_text = """
        Hier können Sie verschiedene Systemkomponenten zurücksetzen:
        
        • Standardwerte wiederherstellen
        • Monitoring-Daten löschen
        • Konfiguration zurücksetzen
        """
        desc_label = ttk.Label(center_frame, text=desc_text, style='Info.TLabel', justify=tk.CENTER)
        desc_label.pack(pady=20)
        
        # Buttons
        button_frame = ttk.Frame(center_frame)
        button_frame.pack(pady=20)
        
        default_button = ttk.Button(button_frame, text="Standardwerte wiederherstellen", 
                                   command=self.reset_to_defaults, width=25)
        default_button.pack(pady=10)
        
        clear_monitoring_button = ttk.Button(button_frame, text="Monitoring-Daten löschen", 
                                           command=self.clear_monitoring_data, width=25)
        clear_monitoring_button.pack(pady=10)
        
        # Warnung
        warning_text = "⚠️ Diese Aktionen können nicht rückgängig gemacht werden!"
        warning_label = ttk.Label(center_frame, text=warning_text, foreground='red', style='Info.TLabel')
        warning_label.pack(pady=20)
        
    def refresh_csv_list(self):
        """Aktualisiere die Liste der CSV-Dateien"""
        self.csv_listbox.delete(0, tk.END)
        
        if os.path.exists(self.monitoring_dir):
            csv_files = glob.glob(os.path.join(self.monitoring_dir, "*.csv"))
            csv_files.sort(reverse=True)  # Neueste zuerst
            
            for file_path in csv_files:
                filename = os.path.basename(file_path)
                self.csv_listbox.insert(tk.END, filename)
                
    def plot_selected_csv(self):
        """Zeige Diagramm für ausgewählte CSV-Datei"""
        selection = self.csv_listbox.curselection()
        if not selection:
            return
            
        filename = self.csv_listbox.get(selection[0])
        filepath = os.path.join(self.monitoring_dir, filename)
        
        try:
            # Lade CSV-Daten
            self.current_df = pd.read_csv(filepath)
            self.current_filename = filename
            self.update_plot()
            
        except Exception as e:
            self.show_error(f"Fehler beim Laden der Datei:\n{str(e)}")
    
    def update_plot(self):
        """Aktualisiere Diagramm basierend auf Checkbox-Auswahl"""
        if not hasattr(self, 'current_df') or self.current_df is None:
            return
            
        df = self.current_df
        
        # Lösche vorherige Plots
        self.fig.clear()
        
        # Bestimme Anzahl der Subplots basierend auf aktiven Checkboxen
        active_plots = []
        if self.plot_vars['pid_components'].get():
            active_plots.append('pid')
        if self.plot_vars['speed'].get() or self.plot_vars['handlebar_angle'].get():
            active_plots.append('dynamics')
        if self.plot_vars['roll_angle'].get():
            active_plots.append('roll_angle')
        if self.plot_vars['error'].get():
            active_plots.append('error')
        if self.plot_vars['parameters'].get():
            active_plots.append('parameters')
            
        if not active_plots:
            self.canvas.draw()
            return
            
        num_plots = len(active_plots)
        
        plot_idx = 1
        
        # Plot 1: PID-Komponenten
        if 'pid' in active_plots and all(col in df.columns for col in ['P', 'I', 'D', 'PID']):
            ax = self.fig.add_subplot(num_plots, 1, plot_idx)
            ax.plot(df.index, df['P'], label='P', alpha=0.8, linewidth=1.5)
            ax.plot(df.index, df['I'], label='I', alpha=0.8, linewidth=1.5)
            ax.plot(df.index, df['D'], label='D', alpha=0.8, linewidth=1.5)
            ax.plot(df.index, df['PID'], label='PID', linewidth=2.5, color='black')
            ax.set_title('🎯 PID-Reglerausgabe', fontsize=12, fontweight='bold')
            ax.set_ylabel('Wert')
            ax.legend(frameon=True, fancybox=True, shadow=True)
            ax.grid(True, alpha=0.3, linestyle='--')
            plot_idx += 1
        
        # Plot 2: Fahrdynamik
        if 'dynamics' in active_plots:
            ax = self.fig.add_subplot(num_plots, 1, plot_idx)
            
            if self.plot_vars['speed'].get() and 'speed' in df.columns:
                ax.plot(df.index, df['speed'], 'b-', label='Geschwindigkeit (km/h)', linewidth=2)
                ax.set_ylabel('Geschwindigkeit (km/h)', color='b')
                ax.tick_params(axis='y', labelcolor='b')
                
                if self.plot_vars['handlebar_angle'].get() and 'handlebar_angle' in df.columns:
                    ax2 = ax.twinx()
                    ax2.plot(df.index, df['handlebar_angle'], 'r-', label='Lenkwinkel (rad)', linewidth=2)
                    ax2.set_ylabel('Lenkwinkel (rad)', color='r')
                    ax2.tick_params(axis='y', labelcolor='r')
                    
            elif self.plot_vars['handlebar_angle'].get() and 'handlebar_angle' in df.columns:
                ax.plot(df.index, df['handlebar_angle'], 'r-', label='Lenkwinkel (rad)', linewidth=2)
                ax.set_ylabel('Lenkwinkel (rad)')
                
            ax.set_title('🚴‍♂️ Fahrdynamik', fontsize=12, fontweight='bold')
            ax.grid(True, alpha=0.3, linestyle='--')
            plot_idx += 1
        
        # Plot 3: Roll-Winkel (Kippwinkel)
        if 'roll_angle' in active_plots and 'roll_angle' in df.columns:
            ax = self.fig.add_subplot(num_plots, 1, plot_idx)
            ax.plot(df.index, df['roll_angle'], 'orange', label='Roll-Winkel (°)', linewidth=2)
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1)
            ax.set_title('🏍️ Roll-Winkel (Kippwinkel)', fontsize=12, fontweight='bold')
            ax.set_ylabel('Roll-Winkel (°)')
            ax.grid(True, alpha=0.3, linestyle='--')
            
            # Füge kritische Winkel-Linien hinzu
            ax.axhline(y=10, color='red', linestyle=':', alpha=0.7, linewidth=1, label='Kritisch (+10°)')
            ax.axhline(y=-10, color='red', linestyle=':', alpha=0.7, linewidth=1, label='Kritisch (-10°)')
            ax.legend(frameon=True, fancybox=True, shadow=True)
            plot_idx += 1
        
        # Plot 4: Regelfehler
        if 'error' in active_plots and 'error' in df.columns:
            ax = self.fig.add_subplot(num_plots, 1, plot_idx)
            ax.plot(df.index, df['error'], 'g-', label='Regelfehler', linewidth=2)
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.5, linewidth=1)
            ax.set_title('⚠️ Regelfehler', fontsize=12, fontweight='bold')
            ax.set_ylabel('Fehler')
            ax.grid(True, alpha=0.3, linestyle='--')
            plot_idx += 1
            
        # Plot 5: Parameter-Verlauf
        if 'parameters' in active_plots and all(col in df.columns for col in ['Kp', 'Ki', 'Kd']):
            ax = self.fig.add_subplot(num_plots, 1, plot_idx)
            ax.plot(df.index, df['Kp'], label='Kp', alpha=0.8, linewidth=1.5)
            ax.plot(df.index, df['Ki'], label='Ki', alpha=0.8, linewidth=1.5)
            ax.plot(df.index, df['Kd'], label='Kd', alpha=0.8, linewidth=1.5)
            ax.set_title('⚙️ Parameter-Verlauf', fontsize=12, fontweight='bold')
            ax.set_ylabel('Parameter-Wert')
            ax.legend(frameon=True, fancybox=True, shadow=True)
            ax.grid(True, alpha=0.3, linestyle='--')
        
        # Gemeinsame X-Achse für letzten Plot
        if active_plots:
            ax.set_xlabel('Messungen')
            
        # Layout anpassen
        self.fig.tight_layout()
        
        # Titel mit Dateiname hinzufügen
        if hasattr(self, 'current_filename'):
            self.fig.suptitle(f'📊 Monitoring: {self.current_filename}', fontsize=14, fontweight='bold', y=0.98)
            self.fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        self.canvas.draw()
    
    def export_plot(self):
        """Exportiere aktuelles Diagramm"""
        if not hasattr(self, 'current_df') or self.current_df is None:
            self.show_error("Kein Diagramm zum Exportieren verfügbar!")
            return
            
        from tkinter import filedialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG-Dateien", "*.png"), ("PDF-Dateien", "*.pdf"), ("SVG-Dateien", "*.svg")],
            title="Diagramm speichern unter..."
        )
        
        if filename:
            try:
                self.fig.savefig(filename, dpi=300, bbox_inches='tight')
                tk.messagebox.showinfo("Erfolg", f"Diagramm gespeichert als:\n{filename}")
            except Exception as e:
                self.show_error(f"Fehler beim Speichern:\n{str(e)}")
    
    def show_error(self, message):
        """Zeige Fehler-Dialog"""
        error_window = tk.Toplevel(self.root)
        error_window.title("Fehler")
        error_window.geometry("400x200")
        error_window.grab_set()  # Modal dialog
        
        ttk.Label(error_window, text=message, wraplength=350, justify=tk.CENTER).pack(expand=True)
        ttk.Button(error_window, text="OK", command=error_window.destroy).pack(pady=10)
            
    def reset_to_defaults(self):
        """Setze Konfiguration auf Standardwerte zurück"""
        result = tk.messagebox.askyesno("Bestätigung", 
                                       "Möchten Sie wirklich alle Einstellungen auf die Standardwerte zurücksetzen?")
        if result:
            self.config = {
                "Kp": 0.01,
                "Ki": 0.02,
                "Kd": 0.0001,
                "maxS": 5.0,
                "minS": 3.0,
                "hMax": 0.1920,
                "preview": 1
            }
            self.save_config()
            self.load_and_refresh()
            
    def clear_monitoring_data(self):
        """Lösche alle Monitoring-Daten"""
        result = tk.messagebox.askyesno("Bestätigung", 
                                       "Möchten Sie wirklich alle Monitoring-Daten löschen?\nDiese Aktion kann nicht rückgängig gemacht werden!")
        if result:
            if os.path.exists(self.monitoring_dir):
                csv_files = glob.glob(os.path.join(self.monitoring_dir, "*.csv"))
                for file_path in csv_files:
                    try:
                        os.remove(file_path)
                    except Exception as e:
                        print(f"Fehler beim Löschen von {file_path}: {e}")
                        
                self.refresh_csv_list()
                tk.messagebox.showinfo("Erfolg", "Monitoring-Daten wurden gelöscht.")
            
    def load_and_refresh(self):
        """Lade Konfiguration und aktualisiere GUI"""
        self.load_config()
        
        # Aktualisiere alle Parameter-Controls
        for param_name in self.config:
            if hasattr(self, f"{param_name}_var"):
                var = getattr(self, f"{param_name}_var")
                if param_name == "preview":
                    var.set(bool(self.config[param_name]))
                else:
                    var.set(self.config[param_name])
                    
                    # Aktualisiere auch Labels und Entry-Felder
                    if hasattr(self, f"{param_name}_label"):
                        label = getattr(self, f"{param_name}_label")
                        label.config(text=f"Aktueller Wert: {self.config[param_name]:.4f}")
                    
                    if hasattr(self, f"{param_name}_entry"):
                        entry = getattr(self, f"{param_name}_entry")
                        entry.delete(0, tk.END)
                        entry.insert(0, f"{self.config[param_name]:.4f}")
                        
    def create_config_display(self, parent):
        """Erstelle Anzeige der aktuellen Konfigurationswerte"""
        info_text = "Aktuelle Werte aus controller_config.json:"
        ttk.Label(parent, text=info_text, style='Info.TLabel').pack(anchor=tk.W, pady=(0, 5))
        
        # Erstelle Textfeld für JSON-Anzeige mit verbessertem Layout
        text_frame = ttk.Frame(parent)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        # Textfeld mit fester Höhe und besserer Konfiguration
        self.config_text = tk.Text(text_frame, 
                                  height=12,  # Erhöht für mehr Inhalt
                                  width=35,   # Erweitert für bessere Lesbarkeit
                                  wrap=tk.NONE,  # Kein Zeilenumbruch für JSON
                                  font=('Monaco', 9) if tk.sys.platform == 'darwin' else ('Consolas', 9),  # macOS-optimierte Schrift
                                  state=tk.DISABLED,
                                  relief=tk.SUNKEN,
                                  borderwidth=1)
        
        # Vertikale Scrollbar
        v_scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=self.config_text.yview)
        self.config_text.configure(yscrollcommand=v_scrollbar.set)
        
        # Horizontale Scrollbar für lange JSON-Zeilen
        h_scrollbar = ttk.Scrollbar(text_frame, orient="horizontal", command=self.config_text.xview)
        self.config_text.configure(xscrollcommand=h_scrollbar.set)
        
        # Grid-Layout für bessere Kontrolle
        self.config_text.grid(row=0, column=0, sticky="nsew")
        v_scrollbar.grid(row=0, column=1, sticky="ns")
        h_scrollbar.grid(row=1, column=0, sticky="ew")
        
        # Grid weights für responsives Verhalten
        text_frame.grid_rowconfigure(0, weight=1)
        text_frame.grid_columnconfigure(0, weight=1)
        
        # Zeige aktuelle Konfiguration an
        self.update_config_display()
    
    def update_config_display(self):
        """Aktualisiere die Konfigurationsanzeige"""
        if hasattr(self, 'config_text'):
            self.config_text.config(state=tk.NORMAL)
            self.config_text.delete(1.0, tk.END)
            
            # Formatiere JSON schön mit besserer Darstellung
            import json
            formatted_json = json.dumps(self.config, indent=3, ensure_ascii=False, sort_keys=True)
            
            # Füge Header hinzu
            display_text = f"📄 controller_config.json\n{'='*35}\n\n{formatted_json}\n\n📝 Letzte Aktualisierung: {datetime.now().strftime('%H:%M:%S')}"
            
            self.config_text.insert(1.0, display_text)
            
            # Scroll zum Anfang
            self.config_text.see(1.0)
            
            self.config_text.config(state=tk.DISABLED)
    
    def manual_save(self):
        """Manuelles Speichern mit Feedback"""
        self.save_config()
        self.update_config_display()
        
        # Zeige Bestätigung
        self.status_label.config(text="✅ Gespeichert!", foreground='green')
        self.root.after(2000, lambda: self.status_label.config(
            text="🔄 Automatisches Speichern aktiv", foreground='green'))
        
        # Kleine Erfolgsmeldung
        tk.messagebox.showinfo("Erfolg", "Konfiguration wurde gespeichert!")
    
    def auto_save(self):
        """Automatisches Speichern alle 1 Sekunde"""
        self.save_config()
        self.update_config_display()
        self.root.after(1000, self.auto_save)

def main():
    # Importiere messagebox hier um Probleme zu vermeiden
    import tkinter.messagebox
    tk.messagebox = tkinter.messagebox
    
    root = tk.Tk()
    app = BicycleControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()