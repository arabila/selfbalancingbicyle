"""
Vision Control Python Controller

Implementiert die langsamere Vision-basierte Pfadplanung mit YOLO-Segmentierung.
Sendet Steer/Speed-Commands an den ultraschnellen Balance-Controller in C.

Architektur:
- Läuft mit 10-20 Hz (jeder 25. Simulation-Step bei 2ms → 50ms)
- Empfängt Balance-Status vom C-Controller
- Sendet Vision-Commands an C-Controller
- Nutzt YOLO für Straßenerkennung und Pfadplanung
"""

import os
import sys
import cv2
import numpy as np
import struct
import time
import math
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

class VisionController:
    def __init__(self):
        # Webots initialisieren
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        print(f"Vision Controller - Timestep: {self.timestep} ms")
        
        # Devices initialisieren
        self._init_devices()

        # Referenz auf Fahrrad für Supervisor-Funktionen
        self.bicycle = self.robot.getFromDef('BICYCLE')
        
        # Kamera-Offset relativ zum Fahrrad (nur Translation, Rotation erbt vom Transform)
        self.camera_offset = [0, 0.1, 0.35]  # x, y, z Offset
        
        # Vision Controller Kamera-Transform-Node-Referenz für dynamische Positionierung
        self.camera_transform_node = self.robot.getFromDef('VISION_CAMERA_TRANSFORM')
        
        # YOLO-Modell laden (falls verfügbar)
        self._init_yolo()
        
        # Steuerungsparameter
        self.max_steer = 1.0      # Maximaler Lenkbefehl (-1.0 bis +1.0)
        self.base_speed = 0.6     # Basis-Geschwindigkeit (0.0 bis 1.0)
        self.min_speed = 0.3      # Minimale Geschwindigkeit
        self.max_speed = 0.9      # Maximale Geschwindigkeit
        
        # PID-Parameter für Vision-basierte Lenkung
        self.vision_kp = 5   # Reduziert für sanftere Lenkung
        self.vision_ki = 0
        self.vision_kd = 0
        
        # PID-Zustand
        self.vision_integral = 0.0
        self.vision_last_error = 0.0
        self.vision_p_term = 0.0
        self.vision_i_term = 0.0
        self.vision_d_term = 0.0
        self.vision_error = 0.0
        self.vision_mask_coverage = 0.0
        
        # Status
        self.step_counter = 0
        self.last_balance_status = None
        self.vision_enabled = True
        
        print("=== Vision Controller gestartet ===")
        print(f"YOLO verfügbar: {YOLO_AVAILABLE}")
        print(f"Vision Controller Kamera: {'✓' if self.camera else '✗'}")
        print(f"Fahrrad Kamera: ✓ (am Fahrrad montiert)")
        print(f"Kamera-Transform: {'✓' if self.camera_transform_node else '✗'}")
        print(f"Display: {'✓' if self.display else '✗'}")
        print(f"Fahrrad-Tracking: {'✓' if self.bicycle else '✗'}")
        print(f"Vision-PID: Kp={self.vision_kp}, Ki={self.vision_ki}, Kd={self.vision_kd}")
        print(f"Speed-Range: {self.min_speed:.1f} - {self.max_speed:.1f}")
        print(f"Kamera-Architektur: Dual-Kamera (Vision Controller + Fahrrad)")
        print("====================================\n")
    
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
        
        # Keyboard für Debug-Eingaben
        self.robot.keyboard.enable(self.timestep)



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
                "../balance_control_c/yolo_vision/runs/segment/train/weights/last.pt",
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
                                     self.vision_error, self.vision_p_term, 
                                     self.vision_i_term, self.vision_d_term, 
                                     self.vision_mask_coverage)
            
            # Debug: Informationen über das gesendete Command
            print(f"DEBUG: Vision-Command senden - Größe: {len(command_data)} Bytes")
            print(f"DEBUG: steer={steer_cmd:.3f}, speed={speed_cmd:.3f}, valid=1")
            print(f"DEBUG: v_error={self.vision_error:.3f}, v_p={self.vision_p_term:.3f}, v_i={self.vision_i_term:.3f}, v_d={self.vision_d_term:.3f}, mask={self.vision_mask_coverage:.2f}")
            
            self.command_emitter.send(command_data)
            
            return True
        except Exception as e:
            print(f"Fehler beim Senden des Vision-Commands: {e}")
            return False
    
    def get_vision_error_yolo(self, frame):
        """YOLO-basierte Straßenerkennung und Fehlerberechnung"""
        if not self.yolo_model:
            return 0.0, np.zeros(frame.shape[:2], dtype=np.uint8)
        
        try:
            # YOLO-Vorhersage
            results = self.yolo_model.predict(
                source=frame,
                conf=0.5,
                max_det=5,
                show=False,
                verbose=False
            )
            
            if not results:
                print("YOLO: Keine Ergebnisse")
                return 0.0, np.zeros(frame.shape[:2], dtype=np.uint8)
            
            r = results[0]
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            
            # Debug: Zeige erkannte Klassen
            if r.boxes is not None and len(r.boxes.cls) > 0:
                detected_classes = [int(cls) for cls in r.boxes.cls]
                if self.step_counter % 100 == 0:  # Nur alle 100 Steps
                    print(f"YOLO: Erkannte Klassen: {detected_classes}")
            
            # Segmentierungsmasken verarbeiten
            if r.masks is not None and r.masks.data is not None:
                for seg in r.masks.data:
                    seg = seg.cpu().numpy()
                    seg_resized = cv2.resize(seg, (frame.shape[1], frame.shape[0]), 
                                           interpolation=cv2.INTER_NEAREST)
                    mask = np.maximum(mask, seg_resized.astype(np.uint8))
                
                if self.step_counter % 100 == 0:  # Debug-Info
                    mask_pixels = np.sum(mask > 0)
                    print(f"YOLO: Segmentierungsmaske mit {mask_pixels} Pixeln erstellt")
            else:
                if self.step_counter % 100 == 0:
                    print("YOLO: Keine Segmentierungsmasken gefunden")
            
            # Suche nach Straßen-Klasse (ID 2 = 'street_main')
            if r.boxes is not None and len(r.boxes.cls) > 0:
                street_indices = [idx for idx, cls in enumerate(r.boxes.cls) if cls == 2]
                
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
                    
                    if self.step_counter % 100 == 0:
                        print(f"YOLO: Straße erkannt - Mittelpunkt: {avg_x_center:.1f}, Error: {error:.3f}")
                    
                    # Mask-Coverage berechnen
                    total_pixels = mask.shape[0] * mask.shape[1]
                    mask_pixels = np.sum(mask > 0)
                    self.vision_mask_coverage = (mask_pixels / total_pixels) * 100.0 if total_pixels > 0 else 0.0
                    
                    return error, mask
                else:
                    if self.step_counter % 100 == 0:
                        print("YOLO: Keine Straßen-Klasse (ID=2) erkannt")
            
            # Mask-Coverage berechnen auch wenn keine Straße erkannt
            total_pixels = mask.shape[0] * mask.shape[1]
            mask_pixels = np.sum(mask > 0)
            self.vision_mask_coverage = (mask_pixels / total_pixels) * 100.0 if total_pixels > 0 else 0.0
            
            return 0.0, mask
            
        except Exception as e:
            print(f"YOLO-Fehler: {e}")
            return 0.0, np.zeros(frame.shape[:2], dtype=np.uint8)
    
    def get_vision_error_fallback(self, frame):
        """Fallback-Vision ohne YOLO (einfache Kantenerkennung)"""
        try:
            # Graustufenkonversion
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Gaussscher Weichzeichner
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Canny-Kantenerkennung
            edges = cv2.Canny(blurred, 50, 150)
            
            # Region of Interest (untere Hälfte des Bildes)
            height, width = edges.shape
            roi_mask = np.zeros_like(edges)
            roi_mask[height//2:, :] = 255
            edges = cv2.bitwise_and(edges, roi_mask)
            
            # Finde Konturen
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Größte Kontur als "Straße" interpretieren
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Berechne Bounding-Box-Mittelpunkt
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                
                # Fehler relativ zur Bildmitte
                frame_center = width / 2
                error = (frame_center - center_x) / width
                
                # Einfache Maske erstellen
                mask = np.zeros((height, width), dtype=np.uint8)
                cv2.drawContours(mask, [largest_contour], -1, 255, -1)
                
                # Mask-Coverage berechnen
                total_pixels = mask.shape[0] * mask.shape[1]
                mask_pixels = np.sum(mask > 0)
                self.vision_mask_coverage = (mask_pixels / total_pixels) * 100.0 if total_pixels > 0 else 0.0
                
                return error, mask
            
            # Keine Konturen gefunden
            self.vision_mask_coverage = 0.0
            return 0.0, np.zeros((height, width), dtype=np.uint8)
            
        except Exception as e:
            print(f"Fallback-Vision-Fehler: {e}")
            return 0.0, np.zeros(frame.shape[:2], dtype=np.uint8)
    
    def vision_pid_control(self, error, dt):
        """PID-Controller für Vision-basierte Lenkung"""
        # P-Term
        p_term = self.vision_kp * error
        
        # I-Term mit Anti-Windup
        self.vision_integral += error * dt
        # Begrenze Integral-Term
        integral_limit = 0.5
        self.vision_integral = max(-integral_limit, min(integral_limit, self.vision_integral))
        i_term = self.vision_ki * self.vision_integral
        
        # D-Term
        d_error = (error - self.vision_last_error) / dt if dt > 0 else 0.0
        d_term = self.vision_kd * d_error
        
        # PID-Terme für IPC-Übertragung speichern
        self.vision_p_term = p_term
        self.vision_i_term = i_term
        self.vision_d_term = d_term
        self.vision_error = error
        
        # Gesamtausgang
        output = p_term + i_term + d_term
        
        # Ausgang begrenzen
        output = max(-self.max_steer, min(self.max_steer, output))
        
        # Für nächsten Zyklus
        self.vision_last_error = error
        
        return output, p_term, i_term, d_term
    
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
            
            # Status-Text hinzufügen
            cv2.putText(overlay, f"Error: {error:.3f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, f"Steer: {steer_cmd:.3f}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, f"Speed: {speed_cmd:.3f}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Balance-Status anzeigen (falls verfügbar)
            if self.last_balance_status:
                roll = self.last_balance_status['roll_angle']
                stability = self.last_balance_status['stability_factor']
                cv2.putText(overlay, f"Roll: {roll:.2f}°", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(overlay, f"Stability: {stability:.2f}", (10, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # YOLO-Status anzeigen
            yolo_status = "YOLO: ✓" if YOLO_AVAILABLE and self.yolo_model else "YOLO: Fallback"
            cv2.putText(overlay, yolo_status, (10, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Masken-Info anzeigen
            mask_info = f"Mask: {np.sum(mask > 0)} pixels" if mask is not None and mask.size > 0 else "Mask: None"
            cv2.putText(overlay, mask_info, (10, 210), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            # OpenCV-Anzeige (immer verfügbar)
            cv2.imshow("Vision Control - Fahrrad Kamera", overlay)
            cv2.waitKey(1)
            
        except Exception as e:
            print(f"Display-Update-Fehler: {e}")
    
    def handle_keyboard_input(self):
        """Verarbeitet Tastatureingaben für Debug/Kontrolle"""
        key = self.robot.keyboard.getKey()
        
        if key == ord('V') or key == ord('v'):
            self.vision_enabled = not self.vision_enabled
            print(f"Vision {'aktiviert' if self.vision_enabled else 'deaktiviert'}")
            
        elif key == ord('R') or key == ord('r'):
            # Reset PID
            self.vision_integral = 0.0
            self.vision_last_error = 0.0
            print("Vision-PID zurückgesetzt")
            
        elif key == 27:  # ESC
            print("ESC gedrückt - beende Vision Controller")
            return False
            
        return True
    
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
    
    def run(self):
        """Hauptschleife des Vision-Controllers"""
        last_vision_time = 0.0
        vision_interval = 0.05  # 50ms → 20 Hz
        
        print("Vision Controller läuft...\n")
        
        while self.robot.step(self.timestep) != -1:
            current_time = self.robot.getTime()
            self.step_counter += 1

            # Kamera-Position an Fahrrad anpassen
            self._update_camera_position()
            
            # Tastatureingaben verarbeiten
            if not self.handle_keyboard_input():
                break
            
            # Balance-Status empfangen
            self.receive_balance_status()
            
            # Vision-Processing nur alle 50ms (20 Hz)
            if current_time - last_vision_time >= vision_interval:
                if self.vision_enabled and self.camera:
                    try:
                        # Kamerabild holen
                        img_bytes = self.camera.getImage()
                        if img_bytes:
                            width = self.camera.getWidth()
                            height = self.camera.getHeight()
                            frame = np.frombuffer(img_bytes, dtype=np.uint8).reshape((height, width, 4))
                            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                            
                            # Vision-Fehler berechnen
                            if YOLO_AVAILABLE and self.yolo_model:
                                error, mask = self.get_vision_error_yolo(frame_bgr)
                            else:
                                error, mask = self.get_vision_error_fallback(frame_bgr)
                            
                            # PID-Controller
                            dt = current_time - last_vision_time
                            steer_cmd, p_term, i_term, d_term = self.vision_pid_control(error, dt)
                            
                            # Geschwindigkeitsanpassung basierend auf Fehler
                            speed_reduction = min(abs(error) * 2.0, 0.4)  # Max 40% Reduktion
                            speed_cmd = self.base_speed - speed_reduction
                            speed_cmd = max(self.min_speed, min(self.max_speed, speed_cmd))
                            
                            # Command senden
                            self.send_vision_command(steer_cmd, speed_cmd)
                            
                            # Display aktualisieren
                            self.update_display(frame_bgr, mask, error, steer_cmd, speed_cmd)
                            
                            # Status ausgeben (alle 2 Sekunden)
                            if self.step_counter % (int(2.0 / vision_interval)) == 0:
                                self.print_status(error, steer_cmd, speed_cmd, p_term, i_term, d_term)
                    
                    except Exception as e:
                        print(f"Vision-Processing-Fehler: {e}")
                
                last_vision_time = current_time
        
        # Cleanup
        cv2.destroyAllWindows()
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
