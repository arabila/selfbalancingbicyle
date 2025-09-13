"""
Vision Control Python Controller mit MPC-Regelung

Implementiert die langsamere Vision-basierte Pfadplanung mit YOLO-Segmentierung.
Sendet Steer/Speed-Commands an den ultraschnellen Balance-Controller in C.
Nutzt Model Predictive Control (MPC) für optimale Trajektorienplanung.

Architektur:
- Läuft mit 10-20 Hz (jeder 25. Simulation-Step bei 2ms → 50ms)
- Empfängt Balance-Status vom C-Controller
- Sendet Vision-Commands an C-Controller
- Nutzt YOLO für Straßenerkennung und Pfadplanung
- MPC für prädiktive Fahrzeugkontrolle
"""

import os
import sys
import cv2
import numpy as np
import struct
import time
import math
import json
import platform
import psutil
from datetime import datetime
from controller import Supervisor

# Setze MPS Fallback für YOLO
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'

# YOLO-Imports (optional - falls verfügbar)
try:
    import torch
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    print("✓ YOLO verfügbar - Vollständige Vision-Pipeline aktiv")
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠ YOLO nicht verfügbar - Fallback-Vision-Modus aktiv")

# MPC-Imports
try:
    import cvxpy # Für MPC-Regelung -> Optimierungsproblem
    MPC_AVAILABLE = True
    print("✓ CVXPY verfügbar - MPC-Regelung aktiv")
except ImportError:
    MPC_AVAILABLE = False
    print("⚠ CVXPY nicht verfügbar - Fallback auf PID-Regelung")

class PerformanceMonitor:
    """Überwacht und misst die Rechenzeit verschiedener Komponenten"""
    
    def __init__(self):
        self.timings = {
            'yolo_computation': [],
            'mpc_computation': [],
            'fallback_vision': [],
            'physics_step': [],
            'total_loop': [],
            'camera_processing': [],
            'display_update': []
        }
        
        # Hardware-Informationen sammeln
        self.hardware_info = self._collect_hardware_info()
        
        # Simulation-Timing
        self.simulation_start_time = None
        self.real_start_time = None
        self.step_count = 0
        
        # Performance-Report-Parameter
        self.report_interval = 100  # Alle 100 Steps einen Report
        self.last_report_step = 0
        
        print("✓ Performance Monitor initialisiert")
        print(f"Hardware: {self.hardware_info['cpu_model']}")
        print(f"CPU Kerne: {self.hardware_info['cpu_cores']} ({self.hardware_info['cpu_threads']} Threads)")
        print(f"CPU Frequenz: {self.hardware_info['cpu_freq']:.1f} GHz")
        if self.hardware_info['gpu_info']:
            print(f"GPU: {self.hardware_info['gpu_info']}")
        print(f"RAM: {self.hardware_info['memory_total']:.1f} GB")
    
    def _collect_hardware_info(self):
        """Sammelt Hardware-Informationen des Systems"""
        info = {}
        
        # CPU-Informationen
        info['cpu_model'] = platform.processor() or "Unbekannt"
        info['cpu_cores'] = psutil.cpu_count(logical=False)
        info['cpu_threads'] = psutil.cpu_count(logical=True)
        
        # CPU-Frequenz
        try:
            cpu_freq = psutil.cpu_freq()
            info['cpu_freq'] = cpu_freq.max / 1000.0 if cpu_freq else 0.0  # GHz
        except:
            info['cpu_freq'] = 0.0
        
        # Speicher
        memory = psutil.virtual_memory()
        info['memory_total'] = memory.total / (1024**3)  # GB
        
        # GPU-Informationen (falls verfügbar)
        info['gpu_info'] = None
        try:
            if YOLO_AVAILABLE:
                import torch
                if torch.cuda.is_available():
                    info['gpu_info'] = torch.cuda.get_device_name(0)
                elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
                    info['gpu_info'] = "Apple Metal Performance Shaders"
        except:
            pass
        
        # Betriebssystem
        info['os'] = f"{platform.system()} {platform.release()}"
        
        return info
    
    def start_timing(self, component):
        """Startet Zeitmessung für eine Komponente"""
        return time.perf_counter()
    
    def end_timing(self, component, start_time):
        """Beendet Zeitmessung und speichert Ergebnis"""
        duration = time.perf_counter() - start_time
        duration_ms = duration * 1000  # ms
        
        # Plausibilitätsprüfung: Extrem hohe Werte (>10s) sind wahrscheinlich Messfehler
        if duration_ms > 10000:  # 10 Sekunden
            print(f"⚠ WARNUNG: Unplausible Messzeit für {component}: {duration_ms:.1f}ms - ignoriert")
            return
        
        if component in self.timings:
            self.timings[component].append(duration_ms)
            
            # Debug-Ausgabe für ungewöhnlich hohe Zeiten
            if duration_ms > 100 and component != 'yolo_computation':
                print(f"DEBUG: Hohe {component}-Zeit: {duration_ms:.1f}ms")
            
            # Begrenzt die Liste auf die letzten 1000 Messungen
            if len(self.timings[component]) > 1000:
                self.timings[component] = self.timings[component][-1000:]
    
    def set_simulation_start(self, sim_time):
        """Setzt den Simulationsstart-Zeitpunkt"""
        if self.simulation_start_time is None:
            self.simulation_start_time = sim_time
            self.real_start_time = time.time()
    
    def update_step_count(self):
        """Aktualisiert die Schrittzählung"""
        self.step_count += 1
    
    def get_realtime_factor(self, current_sim_time):
        """Berechnet den Echtzeitfaktor"""
        if self.simulation_start_time is None or self.real_start_time is None:
            return 0.0
        
        sim_duration = current_sim_time - self.simulation_start_time
        real_duration = time.time() - self.real_start_time
        
        # Debug-Ausgabe für Echtzeitfaktor-Berechnung
        if self.step_count % 100 == 0:  # Nur alle 100 Steps
            print(f"DEBUG Echtzeitfaktor: sim_duration={sim_duration:.3f}s, real_duration={real_duration:.3f}s")
        
        if real_duration > 0:
            return sim_duration / real_duration
        return 0.0
    
    def get_statistics(self, component):
        """Berechnet Statistiken für eine Komponente"""
        if component not in self.timings or not self.timings[component]:
            return {'count': 0, 'avg': 0, 'min': 0, 'max': 0, 'std': 0}
        
        data = np.array(self.timings[component])
        
        # Für YOLO: Filtere Cold-Start-Zeiten (> 500ms) für realistischere Statistiken
        if component == 'yolo_computation' and len(data) > 1:
            # Zeige sowohl gefilterte als auch ungefilterte Werte
            cold_start_mask = data > 500  # Cold-Start-Zeiten
            if np.any(cold_start_mask):
                cold_starts = data[cold_start_mask]
                warm_data = data[~cold_start_mask]
                
                if len(warm_data) > 0:
                    return {
                        'count': len(data),
                        'count_warm': len(warm_data),
                        'count_cold': len(cold_starts),
                        'avg': np.mean(warm_data),  # Durchschnitt ohne Cold-Start
                        'avg_total': np.mean(data),  # Durchschnitt mit Cold-Start
                        'min': np.min(data),
                        'max': np.max(data),
                        'std': np.std(warm_data),
                        'cold_start_times': cold_starts.tolist()
                    }
        
        return {
            'count': len(data),
            'avg': np.mean(data),
            'min': np.min(data),
            'max': np.max(data),
            'std': np.std(data)
        }
    
    def print_performance_report(self, current_sim_time):
        """Druckt einen detaillierten Performance-Report"""
        if self.step_count - self.last_report_step < self.report_interval:
            return
        
        print("\n" + "="*80)
        print("PERFORMANCE MONITOR - RECHENZEIT & ECHTZEITFÄHIGKEIT")
        print("="*80)
        
        # Hardware-Informationen
        print(f"Hardware: {self.hardware_info['cpu_model']}")
        print(f"CPU: {self.hardware_info['cpu_cores']} Kerne, {self.hardware_info['cpu_threads']} Threads, {self.hardware_info['cpu_freq']:.1f} GHz")
        if self.hardware_info['gpu_info']:
            print(f"GPU: {self.hardware_info['gpu_info']}")
        print(f"RAM: {self.hardware_info['memory_total']:.1f} GB")
        print(f"OS: {self.hardware_info['os']}")
        
        # Echtzeitfähigkeit
        rt_factor = self.get_realtime_factor(current_sim_time)
        print(f"\nECHTZEITFÄHIGKEIT:")
        print(f"Simulation Zeit: {current_sim_time:.1f}s")
        print(f"Reale Zeit: {time.time() - self.real_start_time:.1f}s" if self.real_start_time else "N/A")
        print(f"Echtzeitfaktor: {rt_factor:.2f}x {'✓' if rt_factor >= 0.95 else '⚠' if rt_factor >= 0.8 else '✗'}")
        
        # Komponenten-Timing
        print(f"\nKOMPONENTEN-TIMING (Mittelwerte der letzten {self.report_interval} Messungen):")
        print("-"*80)
        print(f"{'Komponente':<20} {'Anzahl':<8} {'Mittel':<10} {'Min':<10} {'Max':<10} {'Std':<10}")
        print("-"*80)
        
        for component in ['yolo_computation', 'mpc_computation', 'fallback_vision', 
                         'camera_processing', 'display_update', 'physics_step', 'total_loop']:
            stats = self.get_statistics(component)
            if stats['count'] > 0:
                # Spezielle Behandlung für YOLO mit Cold-Start-Filterung
                if component == 'yolo_computation' and 'count_warm' in stats:
                    print(f"{component:<20} {stats['count_warm']:<8} {stats['avg']:<10.2f} "
                          f"{stats['min']:<10.2f} {stats['max']:<10.2f} {stats['std']:<10.2f}")
                    if stats['count_cold'] > 0:
                        print(f"  └─ Cold-Starts     {stats['count_cold']:<8} {np.mean(stats['cold_start_times']):<10.1f} "
                              f"{'--':<10} {max(stats['cold_start_times']):<10.1f} {'--':<10}")
                else:
                    print(f"{component:<20} {stats['count']:<8} {stats['avg']:<10.2f} "
                          f"{stats['min']:<10.2f} {stats['max']:<10.2f} {stats['std']:<10.2f}")
        
        # CPU-Auslastung
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_percent = psutil.virtual_memory().percent
            print(f"\nSYSTEM-AUSLASTUNG:")
            print(f"CPU: {cpu_percent:.1f}%")
            print(f"RAM: {memory_percent:.1f}%")
        except:
            pass
        
        # Performance-Analyse und Erklärungen
        print(f"\nPERFORMANCE-ANALYSE:")
        
        # YOLO Cold-Start Erklärung
        yolo_stats = self.get_statistics('yolo_computation')
        if yolo_stats['count'] > 0 and 'cold_start_times' in yolo_stats:
            if yolo_stats['count_cold'] > 0:
                print(f"• YOLO Cold-Start erkannt: {yolo_stats['count_cold']} Aufrufe mit >500ms")
                print(f"  (Normal bei Neural Networks - erste Inferenz lädt Modell in GPU/RAM)")
        
        # MPC Ausreißer-Analyse
        mpc_stats = self.get_statistics('mpc_computation')
        if mpc_stats['count'] > 0 and mpc_stats['max'] > 30:
            print(f"• MPC Ausreißer: {mpc_stats['max']:.1f}ms (möglicherweise Solver-Problem)")
        
        # Echtzeitfaktor-Erklärung
        if rt_factor < 0.1:
            print(f"• Niedriger Echtzeitfaktor durch Cold-Starts - wird sich nach Warmup verbessern")
        elif rt_factor < 0.8:
            print(f"• Simulation läuft {1/rt_factor:.1f}x langsamer als Echtzeit")
        
        print("="*80)
        
        self.last_report_step = self.step_count
    
    def save_performance_report(self, filename=None):
        """Speichert einen detaillierten Performance-Report als JSON"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"performance_report_{timestamp}.json"
        
        report = {
            'timestamp': datetime.now().isoformat(),
            'hardware_info': self.hardware_info,
            'simulation_info': {
                'total_steps': self.step_count,
                'simulation_duration': time.time() - self.real_start_time if self.real_start_time else 0,
                'realtime_factor': self.get_realtime_factor(time.time()) if self.simulation_start_time else 0
            },
            'timing_statistics': {}
        }
        
        # Statistiken für alle Komponenten
        for component in self.timings:
            report['timing_statistics'][component] = self.get_statistics(component)
        
        try:
            with open(filename, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"✓ Performance-Report gespeichert: {filename}")
        except Exception as e:
            print(f"✗ Fehler beim Speichern des Reports: {e}")
        
        return filename

class VisionMPCController: #Unser MPC Controller
    """Model Predictive Controller für Vision-basierte Fahrzeugkontrolle"""
    
    def __init__(self):
        # MPC-Parameter
        self.NX = 4  # Zustandsvektor: [x, y, v, yaw]
        self.NU = 2  # Eingänge: [accel, steer]
        self.T = 3   # Prädiktionshorizont (reduziert für Echtzeitfähigkeit)
        
        # Kostenmatrizen
        self.R = np.diag([0.01, 0.01])      # Eingangskostenmatrix
        self.Rd = np.diag([0.01, 1.0])      # Eingangsdifferenzkostenmatrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])  # Zustandskostenmatrix
        self.Qf = self.Q                    # Terminale Kostenmatrix
        
        # Fahrzeugparameter (angepasst für Fahrrad)
        self.WB = 1.2  # Radstand [m] (Fahrrad)
        self.DT = 0.1  # Zeitschritt [s] (schneller als Vision-Frequenz)
        
        # Begrenzungen (angepasst für Fahrrad)
        self.MAX_STEER = math.radians(30.0)  # Maximaler Lenkwinkel [rad]
        self.MAX_DSTEER = math.radians(20.0) # Maximale Lenkwinkelrate [rad/s]
        self.MAX_SPEED = 8.0                 # Maximale Geschwindigkeit [m/s]
        self.MIN_SPEED = 0.5                 # Minimale Geschwindigkeit [m/s]
        self.MAX_ACCEL = 2.0                 # Maximale Beschleunigung [m/s²]
        
        # Iterative Parameter
        self.MAX_ITER = 3  # Reduziert für Echtzeitfähigkeit
        self.DU_TH = 0.1   # Konvergenz-Schwellwert
        
        # Zustandsspeicher
        # x, y → die Position des Fahrzeugs in der Ebene
        #v → die Geschwindigkeit des Fahrzeugs in der Ebene
        #yaw → der Gierwinkel, also die Orientierung des Fahrzeugs in der Ebene 
        self.state = {'x': 0.0, 'y': 0.0, 'v': 2.0, 'yaw': 0.0} 
        
        self.last_control = {'accel': 0.0, 'steer': 0.0}
        self.reference_path = []
        
        # MPC-Debug-Terme für IPC
        self.mpc_p_term = 0.0
        self.mpc_i_term = 0.0
        self.mpc_d_term = 0.0
        
        # Speicher für vorherigen Referenzpfad (xbar)
        self.xbar_prev = None
        
        print("✓ Vision MPC Controller initialisiert")
    
    def pi_2_pi(self, angle):
        """Normalisiert Winkel auf [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def get_linear_model_matrix(self, v, phi, delta):
        """Berechnet linearisierte Systemmatrizen"""
        A = np.zeros((self.NX, self.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.DT * math.cos(phi)
        A[0, 3] = -self.DT * v * math.sin(phi)
        A[1, 2] = self.DT * math.sin(phi)
        A[1, 3] = self.DT * v * math.cos(phi)
        A[3, 2] = self.DT * math.tan(delta) / self.WB
        
        B = np.zeros((self.NX, self.NU))
        B[2, 0] = self.DT
        B[3, 1] = self.DT * v / (self.WB * math.cos(delta) ** 2)
        
        C = np.zeros(self.NX)
        C[0] = self.DT * v * math.sin(phi) * phi
        C[1] = -self.DT * v * math.cos(phi) * phi
        C[3] = v * delta / (self.WB * math.cos(delta) ** 2)
        
        return A, B, C
    
    def update_state(self, state, accel, steer, bicycle_node=None):
        """Aktualisiert Fahrzeugzustand - x, y, yaw werden aus Webots abgefragt"""
        # Begrenzungen anwenden
        if steer > self.MAX_STEER:
            steer = self.MAX_STEER
        elif steer < -self.MAX_STEER:
            steer = -self.MAX_STEER
        
        new_state = state.copy()
        
        # x, y, yaw aus Webots-Simulation abrufen
        if bicycle_node:
            try:
                # Position (x, y) aus translation field
                bike_pos = bicycle_node.getPosition()
                new_state['x'] = bike_pos[0]  # x-Koordinate
                new_state['y'] = bike_pos[1]  # y-Koordinate (negiert für korrekte Orientierung)
                
                # Yaw aus rotation field (4. Komponente ist der Winkel)
                bike_rotation = bicycle_node.getField('rotation').getSFRotation()
                new_state['yaw'] = bike_rotation[3]  # Rotationswinkel
                
                # Debug-Output (alle 100 Steps)
                if hasattr(self, '_debug_counter'):
                    self._debug_counter += 1
                else:
                    self._debug_counter = 0
                
                if self._debug_counter % 100 == 0:
                    print(f"DEBUG: Webots-Position - x={new_state['x']:.3f}, y={new_state['y']:.3f}, yaw={new_state['yaw']:.3f}")
                
            except Exception as e:
                print(f"Fehler beim Abrufen der Fahrradposition: {e}")
                # Fallback auf berechnete Werte
                new_state['x'] += new_state['v'] * math.cos(new_state['yaw']) * self.DT
                new_state['y'] += new_state['v'] * math.sin(new_state['yaw']) * self.DT
                new_state['yaw'] += new_state['v'] / self.WB * math.tan(steer) * self.DT
        else:
            # Fallback auf berechnete Werte wenn keine Fahrrad-Node verfügbar
            new_state['x'] += new_state['v'] * math.cos(new_state['yaw']) * self.DT
            new_state['y'] += new_state['v'] * math.sin(new_state['yaw']) * self.DT
            new_state['yaw'] += new_state['v'] / self.WB * math.tan(steer) * self.DT
        
        # Geschwindigkeit weiterhin berechnen (wie gewünscht)
        new_state['v'] += accel * self.DT
        
        # Geschwindigkeitsbegrenzungen
        if new_state['v'] > self.MAX_SPEED:
            new_state['v'] = self.MAX_SPEED
        elif new_state['v'] < self.MIN_SPEED:
            new_state['v'] = self.MIN_SPEED
        
        return new_state
    
    def generate_reference_path(self, vision_error, current_state):
        """Generiert Referenzpfad aus Vision-Fehler"""
        # Einfache Referenztrajectorie: Geradeaus mit Korrektur
        ref_path = []
        print("ref_path: ", ref_path)
        
        #Vorheriger Zustand als Startpunkt
        x, y, v, yaw = current_state['x'], current_state['y'], current_state['v'], current_state['yaw']
        print("x, y, v, yaw: ", x, y, v, yaw)
        
        # Ziel-Orientierung basierend auf Vision-Fehler
        target_yaw = yaw - vision_error * 0.5  # Proportionale Korrektur
        
        for i in range(self.T + 1):
            # Interpolation zwischen aktueller und Ziel-Orientierung
            alpha = i / self.T
            ref_yaw = yaw + alpha * (target_yaw - yaw)
            
            # Pfad vorwärts projizieren
            ref_x = x + v * math.cos(ref_yaw) * self.DT * i
            ref_y = y + v * math.sin(ref_yaw) * self.DT * i
            ref_v = v  # Konstante Geschwindigkeit
            
            ref_path.append([ref_x, ref_y, ref_v, ref_yaw])
            print("ref_path: ", ref_path)
            print("np.array(ref_path).T: ", np.array(ref_path).T)
        
        return np.array(ref_path).T
    
    def linear_mpc_control(self, xref, xbar, x0):
        """Löst MPC-Optimierungsproblem"""
        try:
            x = cvxpy.Variable((self.NX, self.T + 1))
            u = cvxpy.Variable((self.NU, self.T))

            print("x: ", x.shape)
            print("u: ", u.shape)
            
            cost = 0.0
            constraints = []
            
            for t in range(self.T):
                cost += cvxpy.quad_form(u[:, t], self.R)
                
                if t != 0:
                    cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)
                
                A, B, C = self.get_linear_model_matrix(
                    xbar[2, t], xbar[3, t], 0.0)  # Linearisierung um Referenz
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]
                
                if t < (self.T - 1):
                    cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                    constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= self.MAX_DSTEER * self.DT]
            
            cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)
            
            # Randbedingungen
            constraints += [x[:, 0] == x0]
            constraints += [x[2, :] <= self.MAX_SPEED]
            constraints += [x[2, :] >= self.MIN_SPEED]
            constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
            constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]
            
            # Problem lösen
            prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
            prob.solve(verbose=False)
            
            if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
                u_opt = u.value
                if u_opt is not None:
                    return u_opt[0, 0], u_opt[1, 0]  # Erste Kontrollaktion
            
            print("⚠ MPC-Lösung nicht optimal - Fallback")
            return self._simple_control_fallback(xref, x0)
            
        except Exception as e:
            print(f"MPC-Fehler: {e}")
            return self._simple_control_fallback(xref, x0)
    
    def control(self, vision_error, dt, bicycle_node=None):
        """Hauptfunktion des MPC-Controllers"""
        # Referenzpfad generieren
        xref = self.generate_reference_path(vision_error, self.state)
        
        # xbar aus vorherigem Zeitschritt verwenden (50ms Verzögerung)
        if self.xbar_prev is not None and 1 == 1: 
            xbar = self.xbar_prev
        else:
            # Beim ersten Aufruf: xbar = xref
            xbar = xref
        
        # Aktueller Zustand
        x0 = np.array([self.state['x'], self.state['y'], self.state['v'], self.state['yaw']])
       
        print("x0: ", x0)
        print("xref: ", xref)
        print("xbar (previous): ", xbar.shape if hasattr(xbar, 'shape') else type(xbar))
    
        # MPC-Regelung
        accel, steer = self.linear_mpc_control(xref, xbar, x0)
        
        # Aktuelles xref für nächsten Zeitschritt speichern
        self.xbar_prev = xref.copy()
        
        # Zustand aktualisieren mit Webots-Daten
        self.state = self.update_state(self.state, accel, steer, bicycle_node)
        
        # Kontrolle speichern
        self.last_control = {'accel': accel, 'steer': steer}
        
        # Debug-Terme für IPC (vereinfacht)
        self.mpc_p_term = vision_error * 2.0  # Proportionalterm
        self.mpc_i_term = 0.0  # Nicht verwendet in MPC
        self.mpc_d_term = steer * 5.0  # Differential-Äquivalent
        
        return steer, accel
    


class VisionController: #Unser Vision Controller -> MPC Controller wird hier verwendet
    def __init__(self):
        # Webots initialisieren
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        print(f"Vision Controller mit MPC - Timestep: {self.timestep} ms")
        
        # Devices initialisieren
        self._init_devices()

        # Referenz auf Fahrrad für Supervisor-Funktionen
        self.bicycle = self.robot.getFromDef('BICYCLE')
        
        # Kamera-Offset relativ zum Fahrrad
        self.camera_offset = [0, 0.1, 0.35]  # x, y, z Offset
        
        # Vision Controller Kamera-Transform-Node-Referenz
        self.camera_transform_node = self.robot.getFromDef('VISION_CAMERA_TRANSFORM')
        
        # YOLO-Modell laden (falls verfügbar)
        self._init_yolo()
        
        # MPC-Controller initialisieren
        self.mpc_controller = VisionMPCController()
        
        # Performance Monitor initialisieren
        self.performance_monitor = PerformanceMonitor()
        
        # Initialen Zustand aus Webots-Simulation setzen
        self._initialize_mpc_state_from_webots()
        
        # Steuerungsparameter
        self.max_steer = 0.4      # Maximaler Lenkbefehl (-1.0 bis +1.0)
        self.base_speed = 0.5     # Basis-Geschwindigkeit (0.0 bis 1.0)
        self.min_speed = 0.3      # Minimale Geschwindigkeit
        self.max_speed = 0.6      # Maximale Geschwindigkeit
        
        # MPC-Zustand für IPC
        self.vision_error = 0.0
        self.vision_mask_coverage = 0.0
        
        # Speicherung der letzten gültigen Werte
        self.last_valid_error = 0.0
        self.last_valid_mask = None
        self.last_valid_mask_coverage = 0.0
        self.last_valid_timestamp = 0.0
        self.no_detection_counter = 0
        self.max_no_detection_steps = 10
        self.error_decay_factor = 0.95
        
        # Status
        self.step_counter = 0
        self.last_balance_status = None
        self.vision_enabled = True
        
        # Konfiguration (GUI) laden – Vision-Methode (yolo|fallback)
        base_dir = os.path.dirname(__file__)
        self.config_file_path = os.path.normpath(os.path.join(base_dir, "..", "..", "GUI", "balance_config.json"))
        self.config_reload_interval = 10.0  # Sekunden; kann aus JSON überschrieben werden
        self.vision_method = "fallback"  # Default; wird gleich überschrieben
        self._last_config_load_ts = 0.0
        self._load_vision_method_from_config(force=True)
        
        # Debug-Visualisierung: Box-Mittelpunkte und Zentren
        self.debug_x_centers = []
        self.debug_avg_x_center = None
        self.debug_frame_center = None

        # Debug: Fallback-Maskenanzeige
        self.debug_show_fallback_masks = True
        self.last_fallback_hsv_mask = None
        self.last_fallback_contour_mask = None

        # Quantisierung und Halte-Logik für Steer-Command
        # 10 Bins im Bereich [-0.06, +0.06], Wechsel erst nach 0.5 s erlaubt
        self.steer_quant_min = -0.06
        self.steer_quant_max = 0.06
        self.steer_quant_bins = 10
        self.steer_bin_hold_s = 0.5
        self._current_steer_bin_idx = None
        self._last_steer_bin_change_ts = 0.0

        # Fallback-ROI-Parameter (anheben/absenken des Erkennungsbalkens)
        # 0.0 = ganz oben, 1.0 = ganz unten
        self.fallback_roi_top_frac = 0.4      # oberer Rand der ROI (höher erkennen → kleinerer Wert)
        self.fallback_roi_height_frac = 0.02   # Höhe der ROI (vertikale Dicke des Balkens)
        self.fallback_y_set_frac = 0.70        # Anzeige-/Messzeile innerhalb der ROI
        
        print("=== Vision Controller mit MPC gestartet ===")
        print(f"YOLO verfügbar: {YOLO_AVAILABLE}")
        print(f"MPC verfügbar: {MPC_AVAILABLE}")
        print(f"Vision Controller Kamera: {'✓' if self.camera else '✗'}")
        print(f"Kamera-Transform: {'✓' if self.camera_transform_node else '✗'}")
        print(f"Display: {'✓' if self.display else '✗'}")
        print(f"Fahrrad-Tracking: {'✓' if self.bicycle else '✗'}")
        print(f"Speed-Range: {self.min_speed:.1f} - {self.max_speed:.1f}")
        print("====================================\n")
    
    def _load_vision_method_from_config(self, force: bool = False):
        """Liest vision_control.method aus der GUI-Konfigurationsdatei ein und aktualisiert die Laufzeitwerte."""
        now = time.time()
        if not force and (now - getattr(self, "_last_config_load_ts", 0.0) < self.config_reload_interval):
            return
        try:
            if os.path.exists(self.config_file_path):
                with open(self.config_file_path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                bc = cfg.get("balance_control", {})
                system_cfg = bc.get("system", {})
                if isinstance(system_cfg, dict):
                    self.config_reload_interval = float(system_cfg.get("config_reload_interval", self.config_reload_interval))
                vision_cfg = bc.get("vision_control", {})
                method = vision_cfg.get("method", self.vision_method)
                if method in {"yolo", "fallback"}:
                    self.vision_method = method
                # Zusätzliche Parameter laden
                self.yolo_conf = float(vision_cfg.get("yolo_conf", 0.5))
                self.yolo_show = bool(int(vision_cfg.get("yolo_show", 0)))
                self.fallback_roi_top_frac = float(vision_cfg.get("fallback_roi_top_frac", getattr(self, 'fallback_roi_top_frac', 0.68)))
                self.fallback_roi_height_frac = float(vision_cfg.get("fallback_roi_height_frac", getattr(self, 'fallback_roi_height_frac', 0.06)))
                self.steer_max_delta = float(vision_cfg.get("steer_max_delta", 0.0025))
                self.steer_max_cmd = float(vision_cfg.get("steer_max_cmd", 0.06))
            self._last_config_load_ts = now
        except Exception as e:
            print(f"Config-Ladefehler: {e}")
            self._last_config_load_ts = now

    def _initialize_mpc_state_from_webots(self):
        """Initialisiert den MPC-Zustand mit aktuellen Werten aus der Webots-Simulation"""
        if self.bicycle:
            try:
                # Position (x, y) aus translation field
                bike_pos = self.bicycle.getPosition()
                self.mpc_controller.state['x'] = bike_pos[0]  # x-Koordinate
                self.mpc_controller.state['y'] = -bike_pos[1]  # y-Koordinate (negiert)
                
                # Yaw aus rotation field (4. Komponente ist der Winkel)
                bike_rotation = self.bicycle.getField('rotation').getSFRotation()
                self.mpc_controller.state['yaw'] = bike_rotation[3]  # Rotationswinkel
                
                print(f"✓ MPC-Zustand initialisiert: x={self.mpc_controller.state['x']:.3f}, y={self.mpc_controller.state['y']:.3f}, yaw={self.mpc_controller.state['yaw']:.3f}")
                
            except Exception as e:
                print(f"⚠ Fehler beim Initialisieren des MPC-Zustands: {e}")
        else:
            print("⚠ Fahrrad-Node nicht verfügbar für MPC-Initialisierung")

    def _init_devices(self):
        """Initialisiert alle Webots-Geräte"""
        # Vision Controller Kamera (folgt dem Fahrrad)
        self.camera = self.robot.getDevice('camera')
        if self.camera:
            self.camera.enable(self.timestep * 4)  # Reduzierte Kamera-Frequenz
            print("✓ Vision Controller Kamera initialisiert")
        else:
            print("✗ Vision Controller Kamera nicht gefunden!")
            
        # Display für Overlay
        self.display = self.robot.getDevice('display')
        if self.display:
            print("✓ Display initialisiert")
        else:
            print("⚠ Display nicht gefunden - nur OpenCV-Anzeige verfügbar")
            
        # IPC: Emitter für Commands
        self.command_emitter = self.robot.getDevice('command_tx')
        if not self.command_emitter:
            print("✗ FEHLER: Command Emitter nicht gefunden!")
            sys.exit(1)
        print("✓ Command Emitter initialisiert")
        
        # IPC: Receiver für Status
        self.status_receiver = self.robot.getDevice('status_rx')
        if not self.status_receiver:
            print("✗ FEHLER: Status Receiver nicht gefunden!")
            sys.exit(1)
        self.status_receiver.enable(self.timestep)
        print("✓ Status Receiver initialisiert")
        
        # Keyboard-Eingaben werden nicht mehr verwendet

    def _update_camera_position(self):
        """Aktualisiert die Vision Controller Kamera-Position relativ zum Fahrrad"""
        if not (self.bicycle and self.camera_transform_node):
            return
            
        try:
            # Fahrrad-Position und -Rotation abrufen
            bike_pos = self.bicycle.getPosition()
            bike_rotation = self.bicycle.getField('rotation').getSFRotation()
            
            # Vision Controller Kamera-Transform exakt an Fahrradposition setzen
            translation_field = self.camera_transform_node.getField('translation')
            translation_field.setSFVec3f(bike_pos)
            
            # Fahrrad-Rotation auf Vision Controller Transform übertragen
            rotation_field = self.camera_transform_node.getField('rotation')
            rotation_field.setSFRotation(bike_rotation)
            
        except Exception as e:
            print(f"Vision Controller Kamera-Positionierung-Fehler: {e}")
        
    def _init_yolo(self):
        """Initialisiert YOLO-Modell (falls verfügbar)"""
        global YOLO_AVAILABLE
        self.yolo_model = None
        
        if YOLO_AVAILABLE:
            # Suche nach YOLO-Modell in verschiedenen Pfaden
            possible_paths = [
                "../balance_control_c/yolo_vision/runs/segment/train/weights/best.pt",
                "../balance_control_c/yolo_vision/runs/segment/train/weights/best.pt",
                "yolo_weights/best.pt",
                "best.pt"
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    try:
                        device = "mps" if torch.backends.mps.is_available() else ("cuda" if torch.cuda.is_available() else "cpu")
                        self.yolo_model = YOLO(path).to(device)
                        print(f"✓ YOLO-Modell geladen: {path} (Device: {device})")
                        break
                    except Exception as e:
                        print(f"⚠ Fehler beim Laden von {path}: {e}")
                        continue
            
            if not self.yolo_model:
                print("⚠ Kein YOLO-Modell gefunden - Fallback-Vision aktiviert")
                YOLO_AVAILABLE = False
    
    def _store_valid_values(self, error, mask, mask_coverage):
        """Speichert die aktuellen gültigen Werte für zukünftige Verwendung"""
        self.last_valid_error = error
        self.last_valid_mask = mask.copy() if mask is not None else None
        self.last_valid_mask_coverage = mask_coverage
        self.last_valid_timestamp = self.robot.getTime()
        self.no_detection_counter = 0  # Reset des Counters bei erfolgreicher Erkennung
        
        if self.step_counter % 100 == 0:
            print(f"VISION: Gültige Werte gespeichert - Error: {error:.3f}, Mask-Coverage: {mask_coverage:.1f}%")

    def _use_last_valid_values(self):
        """Verwendet die letzten gültigen Werte mit langsamem Ausblenden"""
        self.no_detection_counter += 1
        current_time = self.robot.getTime()
        
        # Wenn wir zu lange keine Erkennung hatten, blende langsam aus
        if self.no_detection_counter > self.max_no_detection_steps:
            # Langsames Ausblenden der letzten Werte
            decay_factor = self.error_decay_factor ** (self.no_detection_counter - self.max_no_detection_steps)
            error = self.last_valid_error * decay_factor
            
            # Nach einer bestimmten Zeit komplett auf 0 setzen
            if self.no_detection_counter > self.max_no_detection_steps * 3:
                error = 0.0
                self.vision_mask_coverage = 0.0
                mask = np.zeros((480, 640), dtype=np.uint8)  # Standard-Größe
                
                if self.step_counter % 100 == 0:
                    print(f"VISION: Lange keine Erkennung - Error auf 0 gesetzt")
            else:
                self.vision_mask_coverage = self.last_valid_mask_coverage * decay_factor
                mask = self.last_valid_mask if self.last_valid_mask is not None else np.zeros((480, 640), dtype=np.uint8)
                
                if self.step_counter % 100 == 0:
                    print(f"VISION: Verwende ausgeblendete Werte - Error: {error:.3f} (Decay: {decay_factor:.3f})")
        else:
            # Verwende die letzten gültigen Werte unverändert
            error = self.last_valid_error
            self.vision_mask_coverage = self.last_valid_mask_coverage
            mask = self.last_valid_mask if self.last_valid_mask is not None else np.zeros((480, 640), dtype=np.uint8)
            
            if self.step_counter % 100 == 0:
                print(f"VISION: Verwende letzte gültige Werte - Error: {error:.3f}, seit {self.no_detection_counter} Steps")
        
        return error, mask
    
    def receive_balance_status(self):
        """Empfängt Status vom Balance-Controller"""
        if self.status_receiver.getQueueLength() > 0:
            try:
                data = self.status_receiver.getBytes()
                # Struct-Format: 4 floats (roll_angle, steering_output, current_speed, stability_factor)
                status = struct.unpack('ffff', data[:16])
                
                self.last_balance_status = {
                    'roll_angle': status[0],
                    'steering_output': status[1], 
                    'current_speed': status[2],
                    'stability_factor': status[3],
                    'timestamp': self.robot.getTime()
                }
                
                self.status_receiver.nextPacket()
                return True
            except Exception as e:
                print(f"Fehler beim Empfangen des Balance-Status: {e}")
                self.status_receiver.nextPacket()
        
        return False
    
    def send_vision_command(self, steer_cmd, speed_cmd):
        """Sendet erweiterte Vision-Command an Balance-Controller"""
        try:
            # Begrenzungen anwenden
            steer_cmd = max(-1.0, min(1.0, steer_cmd))
            speed_cmd = max(0.0, min(1.0, speed_cmd))
            
            # Erweiterte Struct-Format: 8 Felder (2 floats + 1 int + 5 floats)
            # (steer_command, speed_command, valid, vision_error, vision_p_term, vision_i_term, vision_d_term, mask_coverage)
            command_data = struct.pack('ffifffff', 
                                     steer_cmd, speed_cmd, 1,
                                     self.vision_error, self.mpc_controller.mpc_p_term, 
                                     self.mpc_controller.mpc_i_term, self.mpc_controller.mpc_d_term, 
                                     self.vision_mask_coverage)
            
            # Debug: Informationen über das gesendete Command
            print(f"DEBUG: Vision-Command senden - Größe: {len(command_data)} Bytes")
            print(f"DEBUG: steer={steer_cmd:.3f}, speed={speed_cmd:.3f}, valid=1")
            print(f"DEBUG: v_error={self.vision_error:.3f}, v_p={self.mpc_controller.mpc_p_term:.3f}, v_i={self.mpc_controller.mpc_i_term:.3f}, v_d={self.mpc_controller.mpc_d_term:.3f}, mask={self.vision_mask_coverage:.2f}")
            
            self.command_emitter.send(command_data)
            
            return True
        except Exception as e:
            print(f"Fehler beim Senden des Vision-Commands: {e}")
            return False
    
    def get_vision_error_yolo(self, frame):
        """YOLO-basierte Straßenerkennung und Fehlerberechnung mit Speicherung der letzten gültigen Werte"""
        if not self.yolo_model:
            return self._use_last_valid_values()
        
        try:
            # YOLO-Vorhersage mit Timing
            yolo_start = self.performance_monitor.start_timing('yolo_computation') if hasattr(self, 'performance_monitor') else time.perf_counter()
            
            # Konfiguration ggf. aktualisieren (yolo_conf, yolo_show)
            self._load_vision_method_from_config()
            conf_thr = float(getattr(self, 'yolo_conf', 0.5))
            show_flag = bool(getattr(self, 'yolo_show', False))
            results = self.yolo_model.predict(
                source=frame,   # Bild, das verarbeitet werden soll
                conf=conf_thr,  # Konfidenz-Schwellenwert
                max_det=3,      # Maximale Anzahl von Erkennungen
                classes=[2],    # Nur Fahrbahn/Spur
                show=show_flag, # Bildausgabe
                verbose=False   # Ausgabe
            )
            
            # YOLO-Timing beenden
            if hasattr(self, 'performance_monitor'):
                self.performance_monitor.end_timing('yolo_computation', yolo_start)
            print(f"Yolo results: {results} End")
            
            if not results:
                print("YOLO: Keine Ergebnisse")
                return self._use_last_valid_values()
            

            r = results[0]
            mask = np.zeros(frame.shape[:2], dtype=np.uint8) #Maske erstellen 

            print(f"r.boxes.cls: {r.boxes.cls}")
            
            # Debug: Zeige erkannte Klassen
            if r.boxes is not None and len(r.boxes.cls) > 0:
                detected_classes = [int(cls) for cls in r.boxes.cls]
                if self.step_counter % 100 == 0:  # Nur alle 100 Steps
                    print(f"YOLO: Erkannte Klassen: {detected_classes}")
            
            # Segmentierungsmasken verarbeiten (nur Klasse 2 = Straße)
            if r.masks is not None and r.masks.data is not None and r.boxes is not None:
                for seg, cls in zip(r.masks.data, r.boxes.cls):
                    if int(cls) != 2:
                        continue
                    seg_np = seg.cpu().numpy()
                    seg_resized = cv2.resize(seg_np, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                    seg_bin = (seg_resized > 0.5).astype(np.uint8)
                    mask = np.maximum(mask, seg_bin)
                if self.step_counter % 100 == 0:
                    mask_pixels = int(np.sum(mask > 0))
                    print(f"YOLO: Straßen-Maske (Klasse 2) mit {mask_pixels} Pixeln erstellt")
            else:
                if self.step_counter % 100 == 0:
                    print("YOLO: Keine Segmentierungsmasken gefunden")
            
            #Berechnung des Fehlers aus der Straßen-Bounding-Box
            if r.boxes is not None and len(r.boxes.cls) > 0:
                street_indices = [idx for idx, cls in enumerate(r.boxes.cls) if cls == 2] #Suche nach Straßen-Klasse (ID 2 = 'street_main')
                
                if street_indices:
                    # Berechne Mittelpunkt der Straßen-Bounding-Boxes
                    x_centers = []
                    for idx in street_indices:
                        xyxy = r.boxes.xyxy[idx]
                        x_center = float((xyxy[0] + xyxy[2]) / 2.0)
                        x_centers.append(x_center)
                    
                    avg_x_center = sum(x_centers) / len(x_centers)
                    frame_center = frame.shape[1] / 2
                    error = (frame_center - avg_x_center) / frame.shape[1]  # Normiert
                    
                    # Debug-Visualisierungspunkte speichern
                    self.debug_x_centers = x_centers.copy()
                    self.debug_avg_x_center = float(avg_x_center)
                    self.debug_frame_center = float(frame_center)
                    
                    if self.step_counter % 100 == 0:
                        print(f"YOLO: Straße erkannt - Mittelpunkt: {avg_x_center:.1f}, Error: {error:.3f}")
                    
                    # Mask-Coverage berechnen
                    total_pixels = mask.shape[0] * mask.shape[1]
                    mask_pixels = np.sum(mask > 0)
                    self.vision_mask_coverage = (mask_pixels / total_pixels) * 100.0 if total_pixels > 0 else 0.0
                    
                    # Gültige Werte speichern
                    self._store_valid_values(error, mask, self.vision_mask_coverage)
                    
                    return error, mask
                else:
                    if self.step_counter % 100 == 0:
                        print("YOLO: Keine Straßen-Klasse (ID=2) erkannt - verwende letzte gültige Werte")
            
            # Keine Straße erkannt - verwende letzte gültige Werte
            return self._use_last_valid_values()
            
        except Exception as e:
            print(f"YOLO-Fehler: {e}")
            return self._use_last_valid_values()
    
    def get_vision_error_fallback(self, frame):
        """Fallback-Vision ohne YOLO basierend auf simple_mask.py (ROI + HSV + Morphologie + Konturen)."""
        try:
            # Fallback-Vision Timing starten
            fallback_start = self.performance_monitor.start_timing('fallback_vision') if hasattr(self, 'performance_monitor') else time.perf_counter()
            # Bildabmessungen
            height, width = frame.shape[:2]

            # Zielpunkt: exakt die Bildmitte (x)
            x_set = int(width / 2)

            # Parametrisierte, schmale Trapez-ROI höher/tiefer setzbar
            top_frac = float(np.clip(getattr(self, 'fallback_roi_top_frac', 0.68), 0.0, 0.98)) #vertikale Position des oberen ROI-Rands
            band_frac = float(np.clip(getattr(self, 'fallback_roi_height_frac', 0.06), 0.01, 0.3))
            top_y = int(top_frac * height)
            bot_y = min(height - 1, top_y + max(2, int(band_frac * height)))

            # y-Setzpunkt innerhalb des Bildes, nahe der ROI
            y_set_frac = float(np.clip(getattr(self, 'fallback_y_set_frac', 0.70), 0.0, 1.0)) #y-Position der Messlinie innerhalb des Bildes;
            y_set = int(np.clip(y_set_frac * height, top_y, bot_y - 1))

            # Trapezbreiten (oben schmaler als unten)
            left_top = int(0.18 * width)
            right_top = int(0.82 * width)
            left_bot = int(0.14 * width)
            right_bot = int(0.86 * width)

            pts = np.array([[
                [left_top, top_y],
                [right_top, top_y],
                [right_bot, bot_y],
                [left_bot, bot_y]
            ]], dtype=np.int32)

            roi_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(roi_mask, pts, 255)
            zone = cv2.bitwise_and(frame, frame, mask=roi_mask)

            # HSV-Threshold für Straße (dunkelgrau) + weiße Markierungen
            # - Graue Straße: niedrige Sättigung, mittlere Helligkeit
            # - Weiße Markierungen: sehr niedrige Sättigung, hohe Helligkeit
            hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)
            kernel = np.ones((5, 5), np.uint8)

            road_lower = np.array([0, 0, 30])    # H egal, S niedrig, V mittel
            road_upper = np.array([179, 60, 200])
            white_lower = np.array([0, 0, 200])  # H egal, S sehr niedrig, V hoch
            white_upper = np.array([179, 40, 255])

            mask_road = cv2.inRange(hsv, road_lower, road_upper)
            mask_white = cv2.inRange(hsv, white_lower, white_upper)
            mask0 = cv2.bitwise_or(mask_road, mask_white)

            mask0 = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, kernel)
            mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, kernel)

            # Speichere HSV-Maske für Debug-Anzeige
            self.last_fallback_hsv_mask = mask0.copy()

            # Konturen finden in der gefilterten ROI
            contours, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)

                # Mittelpunkt der Bounding-Box
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = int(x + w / 2)

                # Fehler normiert an Bildbreite (simple_mask gibt Pixel aus → hier normieren)
                error = (x_set - center_x) / float(max(1, width))

                # Binäre Maske der größten Kontur für Overlay und Coverage
                mask = np.zeros((height, width), dtype=np.uint8)
                cv2.drawContours(mask, [largest_contour], -1, 255, -1)

                # Speichere Konturmaske für Debug-Anzeige
                self.last_fallback_contour_mask = mask.copy()

                # Maskenabdeckung berechnen
                total_pixels = mask.size
                mask_pixels = int(np.sum(mask > 0))
                self.vision_mask_coverage = (mask_pixels / total_pixels) * 100.0 if total_pixels > 0 else 0.0

                # Werte speichern für Ausfallsicherheit
                self._store_valid_values(error, mask, self.vision_mask_coverage)

                # Debug-Werte für OpenCV-Overlay
                self.debug_x_centers = [float(center_x)]
                self.debug_avg_x_center = float(center_x)
                self.debug_frame_center = float(width / 2)

                # Fallback-Vision Timing beenden
                if hasattr(self, 'performance_monitor'):
                    self.performance_monitor.end_timing('fallback_vision', fallback_start)

                return error, mask

            # Keine gültige Kontur → letzte gültige Werte verwenden (mit Decay-Logik)
            # Bei Misserfolg: Letzte Masken für Debug zurücksetzen
            self.last_fallback_contour_mask = None
            
            # Fallback-Vision Timing beenden (auch bei Misserfolg)
            if hasattr(self, 'performance_monitor'):
                self.performance_monitor.end_timing('fallback_vision', fallback_start)
            
            return self._use_last_valid_values()

        except Exception as e:
            print(f"Fallback-Vision-Fehler: {e}")
            self.last_fallback_contour_mask = None
            
            # Fallback-Vision Timing beenden (auch bei Fehler)
            if hasattr(self, 'performance_monitor'):
                self.performance_monitor.end_timing('fallback_vision', fallback_start)
            
            return self._use_last_valid_values()
    
    def vision_mpc_control(self, error, dt):
        """MPC-Controller für Vision-basierte Lenkung"""
        # MPC-Timing starten
        mpc_start = self.performance_monitor.start_timing('mpc_computation') if hasattr(self, 'performance_monitor') else time.perf_counter()
        
        # MPC-Regelung durchführen mit Fahrrad-Node für Position/Orientierung
        steer_rad, accel = self.mpc_controller.control(error, dt, self.bicycle)
        
        # MPC-Timing beenden
        if hasattr(self, 'performance_monitor'):
            self.performance_monitor.end_timing('mpc_computation', mpc_start)
        
        # Lenkwinkel normalisieren auf [-1, 1] für IPC
        steer_cmd = np.clip(steer_rad / self.mpc_controller.MAX_STEER, -1.0, 1.0)
        
        # Geschwindigkeitsbefehl aus Beschleunigung ableiten
        current_speed = self.mpc_controller.state['v']
        target_speed = current_speed + accel * dt
        target_speed = np.clip(target_speed, self.mpc_controller.MIN_SPEED, self.mpc_controller.MAX_SPEED)
        
        # Geschwindigkeit auf [0, 1] normalisieren
        speed_cmd = (target_speed - self.min_speed * 10) / (self.max_speed * 10 - self.min_speed * 10)
        speed_cmd = np.clip(speed_cmd, 0.0, 1.0)
        
        # MPC-Terme für IPC-Übertragung speichern
        self.vision_error = error
        
        return steer_cmd, speed_cmd
    
    def update_display(self, frame, mask, error, steer_cmd, speed_cmd):
        """Aktualisiert das Display mit Vision-Overlay"""
        if not self.display:
            # Nur OpenCV-Anzeige verfügbar
            pass
            
        try:
            # Maske als Overlay - gleiche Methode wie in yolo_vision_mps.py
            if mask is not None and mask.size > 0:
                # Maske in BGR umwandeln (wie in yolo_vision_mps.py)
                mask_bgr = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
                overlay = cv2.addWeighted(frame, 0.6, mask_bgr, 0.4, 0)
            else:
                overlay = frame.copy()
            
            # Nur Error und Steer anzeigen
            cv2.putText(overlay, f"Error: {error:.3f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, f"Steer: {steer_cmd:.3f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Box-Mittelpunkte, Durchschnitt und Bildmitte darstellen
            h, w = overlay.shape[:2]
            if self.debug_frame_center is not None:
                x_fc = int(max(0, min(w - 1, round(self.debug_frame_center))))
                cv2.line(overlay, (x_fc, 0), (x_fc, h - 1), (255, 255, 0), 1)
                cv2.putText(overlay, "frame_center", (max(0, x_fc - 60), 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

            if self.debug_avg_x_center is not None:
                x_avg = int(max(0, min(w - 1, round(self.debug_avg_x_center))))
                cv2.line(overlay, (x_avg, 0), (x_avg, h - 1), (255, 0, 255), 1)
                cv2.putText(overlay, "avg_center", (max(0, x_avg - 45), 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

            if self.debug_x_centers:
                for xc in self.debug_x_centers:
                    xi = int(max(0, min(w - 1, round(float(xc)))))
                    cv2.line(overlay, (xi, 0), (xi, h - 1), (0, 255, 255), 1)
                cv2.putText(overlay, f"{len(self.debug_x_centers)} boxes", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if self.debug_frame_center is not None and self.debug_avg_x_center is not None:
                x_fc = int(max(0, min(w - 1, round(self.debug_frame_center))))
                x_avg = int(max(0, min(w - 1, round(self.debug_avg_x_center))))
                y_arrow = max(40, h - 25)
                color = (0, 0, 255)
                cv2.arrowedLine(overlay, (x_avg, y_arrow), (x_fc, y_arrow), color, 2, tipLength=0.15)
                cv2.putText(overlay, f"err_px={x_fc - x_avg:+d}", (min(w - 120, max(0, (x_fc + x_avg)//2 - 60)), y_arrow - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # OpenCV-Anzeige (immer verfügbar)
            cv2.imshow("Vision Control - Fahrrad Kamera", overlay)

            # Zusätzliche Debug-Fenster für Fallback-Masken
            if self.debug_show_fallback_masks:
                if self.last_fallback_hsv_mask is not None:
                    cv2.imshow("Fallback HSV Mask", self.last_fallback_hsv_mask)
                if self.last_fallback_contour_mask is not None:
                    cv2.imshow("Fallback Contour Mask", self.last_fallback_contour_mask)
            cv2.waitKey(1)
            
        except Exception as e:
            print(f"Display-Update-Fehler: {e}")
    
    def print_status(self, error, steer_cmd, speed_cmd, p_term, i_term, d_term):
        """Gibt Status-Informationen aus"""
        print(f"VISION: Error={error:6.3f} | Steer={steer_cmd:6.3f} | Speed={speed_cmd:5.2f} | "
              f"PID=[P:{p_term:5.2f} I:{i_term:5.2f} D:{d_term:5.2f}]", end="")
        
        if self.last_balance_status:
            roll = self.last_balance_status['roll_angle']
            stability = self.last_balance_status['stability_factor']
            print(f" | Balance: Roll={roll:5.1f}° Stab={stability:4.2f}")
        else:
            print(" | Balance: N/A")

    def optimize_steer_cmd(self, steer_cmd, last_steer_cmd):
        """Quantisiert und begrenzt den Lenkbefehl in 10 Bins innerhalb [-0.06, +0.06]
        und hält den aktuellen Bin mindestens 0.5 s, bevor ein Wechsel erlaubt ist.

        Hinweis: last_steer_cmd wird nicht mehr für Rate-Limiting verwendet, da
        hier feste Bin-Mitten ausgegeben werden sollen.
        """

        # Konfiguration ggf. laden; Quantisierungsgrenzen sind fest auf [-0.06, +0.06]
        self._load_vision_method_from_config()
        max_cmd = float(self.steer_quant_max)
        min_cmd = float(self.steer_quant_min)

        # 1) Hart begrenzen auf [-max_cmd, +max_cmd]
        raw = float(np.clip(steer_cmd, min_cmd, max_cmd))

        # 2) In 10 gleich große Bins quantisieren (5 negativ, 5 positiv)
        bins = int(self.steer_quant_bins)
        step = (max_cmd - min_cmd) / float(bins)
        # Numerische Robustheit
        if step <= 0:
            return raw

        # Ziel-Index berechnen (0..bins-1), inkl. rechter Rand für max_cmd
        rel = (raw - min_cmd) / step
        target_idx = int(np.floor(rel))
        if target_idx < 0:
            target_idx = 0
        if target_idx >= bins:
            target_idx = bins - 1

        # 3) Halte- und Schritt-Logik: mind. 0.5 s halten, pro Takt max. ein Nachbar-Schritt
        now = self.robot.getTime()
        if self._current_steer_bin_idx is None:
            # Startbedingung: immer bei 0.0 beginnen und mindestens 0.5 s halten
            if self._last_steer_bin_change_ts == 0.0:
                self._last_steer_bin_change_ts = now
                return 0.0
            if (now - self._last_steer_bin_change_ts) >= float(self.steer_bin_hold_s):
                # Erster erlaubter Schritt: genau ein Nachbar-Bin Richtung Ziel
                if raw > 0.0:
                    self._current_steer_bin_idx = 5  # erstes positives Bin (nahe 0)
                    self._last_steer_bin_change_ts = now
                elif raw < 0.0:
                    self._current_steer_bin_idx = 4  # erstes negatives Bin (nahe 0)
                    self._last_steer_bin_change_ts = now
                else:
                    return 0.0
            else:
                return 0.0
        elif target_idx != self._current_steer_bin_idx:
            if (now - self._last_steer_bin_change_ts) >= float(self.steer_bin_hold_s):
                direction = 1 if target_idx > self._current_steer_bin_idx else -1
                self._current_steer_bin_idx += direction
                # Sicherheit innerhalb [0, bins-1]
                if self._current_steer_bin_idx < 0:
                    self._current_steer_bin_idx = 0
                if self._current_steer_bin_idx >= bins:
                    self._current_steer_bin_idx = bins - 1
                self._last_steer_bin_change_ts = now
            # sonst: halten ohne Änderung

        # 4) Feste Ausgabe: Bin-Mitte des aktuellen Bins
        bin_center = min_cmd + (self._current_steer_bin_idx + 0.5) * step

        # 5) Sicherheits-Clamp (numerisch)
        bin_center = float(np.clip(bin_center, min_cmd, max_cmd))

        return bin_center
    


    
    def update_screen_labels(self, error, steer_cmd):
        """Zeigt Speed, Error, Steer und Roll-Winkel als On-Screen-Label an (wie in printStatus())."""
        try:
            # Geschwindigkeit aus der Fahrrad-Node berechnen (m/s → km/h)
            speed_kmh = 0.0
            if self.bicycle:
                velo = self.bicycle.getVelocity()
                if velo and len(velo) >= 3:
                    vx, vy, vz = float(velo[0]), float(velo[1]), float(velo[2])
                    speed_kmh = (vx * vx + vy * vy + vz * vz) ** 0.5 * 3.6

            # Roll-Winkel aus Balance-Status (in Grad)
            roll_deg = 0.0
            if self.last_balance_status and 'roll_angle' in self.last_balance_status:
                roll_deg = float(self.last_balance_status['roll_angle'])

            # Label-Text und Position
            vpos = 0.93
            text = (
                f"Speed: {speed_kmh:5.2f} km/h   "
                f"Vision_Error: {error:6.3f}   "
                f"Vision_Steer: {steer_cmd:6.3f}   "
                f"Roll: {roll_deg:6.3f}°"
            )

            # On-Screen-Label zeichnen (ID 1, links oben)
            self.robot.setLabel(1, text, 0, vpos, 0.06, 0x000000, 0, 'Lucida Console')
        except Exception as e:
            print(f"Label-Update-Fehler: {e}")

    def run(self):
        """Hauptschleife des Vision-Controllers"""
        last_vision_time = 0.0
        vision_interval = 0.05  # 50ms → 20 Hz
        last_steer_cmd = 0.0
        
        print("Vision Controller läuft...\n")
        
        while self.robot.step(self.timestep) != -1:
            # Timing für gesamte Schleife starten
            loop_start = self.performance_monitor.start_timing('total_loop')
            
            current_time = self.robot.getTime() #Aktuelle Zeit in der Simulation
            self.step_counter += 1
            
            # Performance Monitor initialisieren bei erstem Durchlauf
            self.performance_monitor.set_simulation_start(current_time)
            self.performance_monitor.update_step_count()

            #---------------#Position der Kamera dem Fahrrad anpassen---------
            self._update_camera_position() 
            
            #---------------#Balance-Status empfangen-----------------
            self.receive_balance_status()
            
            #---------------#Vision-Processing nur alle 50ms (20 Hz)-----
            if current_time - last_vision_time >= vision_interval:
                if self.vision_enabled and self.camera:
                    try:
                        # Kamera-Processing Timing starten
                        camera_start = self.performance_monitor.start_timing('camera_processing')
                        
                        img_bytes = self.camera.getImage() #Bild von der Kamera empfangen: https://cyberbotics.com/doc/reference/camera?tab-language=python
                        if img_bytes:
                            # Bildgröße auslesen: Eingestellt in der Webots-Weltdatei
                            width = self.camera.getWidth() 
                            height = self.camera.getHeight()

                            # Bild in ein numpy-Array umwandeln
                            frame = np.frombuffer(img_bytes, dtype=np.uint8).reshape((height, width, 4))
                            print(f"frame: {frame}")
                
                            # Bild in BGR umwandeln (RGB -> BGR)
                            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                            print(f"frame_bgr: {frame_bgr}")
                            
                            # Kamera-Processing Timing beenden
                            self.performance_monitor.end_timing('camera_processing', camera_start)
                            
                            # Vision-Fehler berechnen – Methode aus GUI-Konfiguration
                            self._load_vision_method_from_config()
                            if self.vision_method == "yolo" and YOLO_AVAILABLE and self.yolo_model:
                                error, mask = self.get_vision_error_yolo(frame_bgr)
                            else:
                                error, mask = self.get_vision_error_fallback(frame_bgr)
                                
                             
                            print("error: ", error)
                            # MPC-Regelung
                            dt = current_time - last_vision_time
                            steer_cmd, speed_cmd = self.vision_mpc_control(error, dt)

                            steer_cmd = self.optimize_steer_cmd(steer_cmd, last_steer_cmd)

                        

                            last_steer_cmd = steer_cmd
                           
                            
                            # Command senden
                            self.send_vision_command(steer_cmd, speed_cmd)
                            
                            # Display-Update Timing starten
                            display_start = self.performance_monitor.start_timing('display_update')
                            
                            # Display aktualisieren
                            self.update_display(frame_bgr, mask, error, steer_cmd, speed_cmd)
                            # On-Screen-Label (wie printStatus) aktualisieren
                            self.update_screen_labels(error, steer_cmd)
                            
                            # Display-Update Timing beenden
                            self.performance_monitor.end_timing('display_update', display_start)
                            
                            # Status ausgeben (alle 2 Sekunden)
                            if self.step_counter % (int(2.0 / vision_interval)) == 0:
                                self.print_status(error, steer_cmd, speed_cmd, self.mpc_controller.mpc_p_term, self.mpc_controller.mpc_i_term, self.mpc_controller.mpc_d_term)
                    
                    except Exception as e:
                        print(f"Vision-Processing-Fehler: {e}")
                
                last_vision_time = current_time
            
            # Timing für gesamte Schleife beenden
            self.performance_monitor.end_timing('total_loop', loop_start)
            
            # Performance-Report ausgeben (alle 100 Steps)
            if self.step_counter % 100 == 0:
                self.performance_monitor.print_performance_report(current_time)
        
        # Cleanup
        cv2.destroyAllWindows()
        
        # Finalen Performance-Report speichern
        self.performance_monitor.save_performance_report()
        print("Vision Controller beendet")

def main():
    """Hauptfunktion"""
    try:
        controller = VisionController()
        controller.run()
    except KeyboardInterrupt:
        print("\nVision Controller durch Benutzerunterbrechung beendet")
    except Exception as e:
        print(f"FEHLER: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 

