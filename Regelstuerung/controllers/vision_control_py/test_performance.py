#!/usr/bin/env python3
"""
Test-Script für Performance-Monitoring

Testet die Performance-Monitor-Klasse ohne Webots-Abhängigkeiten.
"""

import time
import numpy as np
from vision_control_py import PerformanceMonitor

def simulate_yolo_computation():
    """Simuliert YOLO-Berechnung"""
    time.sleep(0.05)  # 50ms

def simulate_mpc_computation():
    """Simuliert MPC-Berechnung"""
    time.sleep(0.02)  # 20ms

def simulate_fallback_vision():
    """Simuliert Fallback-Vision"""
    time.sleep(0.01)  # 10ms

def test_performance_monitor():
    """Testet den Performance Monitor"""
    print("=== Performance Monitor Test ===")
    
    monitor = PerformanceMonitor()
    
    # Simuliere mehrere Berechnungszyklen
    for i in range(50):
        # YOLO-Timing
        start = monitor.start_timing('yolo_computation')
        simulate_yolo_computation()
        monitor.end_timing('yolo_computation', start)
        
        # MPC-Timing
        start = monitor.start_timing('mpc_computation')
        simulate_mpc_computation()
        monitor.end_timing('mpc_computation', start)
        
        # Fallback-Vision-Timing
        start = monitor.start_timing('fallback_vision')
        simulate_fallback_vision()
        monitor.end_timing('fallback_vision', start)
        
        monitor.update_step_count()
        
        if i == 0:
            monitor.set_simulation_start(0.0)
    
    # Statistiken anzeigen
    print("\nStatistiken:")
    for component in ['yolo_computation', 'mpc_computation', 'fallback_vision']:
        stats = monitor.get_statistics(component)
        print(f"{component}: {stats['avg']:.2f}ms (min: {stats['min']:.2f}, max: {stats['max']:.2f})")
    
    # Performance-Report speichern
    filename = monitor.save_performance_report("test_performance_report.json")
    print(f"\nTest-Report gespeichert: {filename}")

if __name__ == "__main__":
    test_performance_monitor()
