#!/usr/bin/env python3
"""
Test-Skript für MPC-Integration im Vision-Controller

Testet die MPC-Funktionalität ohne Webots-Abhängigkeiten.
"""

import numpy as np
import sys
import os

# Füge den aktuellen Pfad hinzu
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_mpc_controller():
    """Testet den MPC-Controller standalone"""
    print("=== MPC-Controller Test ===")
    
    # Importiere die MPC-Klasse
    try:
        from vision_control_py import VisionMPCController
        print("✓ MPC-Controller erfolgreich importiert")
    except ImportError as e:
        print(f"✗ Import-Fehler: {e}")
        return False
    
    # Initialisiere MPC-Controller
    try:
        mpc = VisionMPCController()
        print("✓ MPC-Controller initialisiert")
    except Exception as e:
        print(f"✗ Initialisierungsfehler: {e}")
        return False
    
    # Teste verschiedene Vision-Fehler
    test_errors = [0.0, 0.1, -0.2, 0.5, -0.3]
    dt = 0.05  # 20 Hz
    
    print("\nTeste MPC-Kontrolle:")
    print("Error\tSteer\tAccel\tSpeed")
    print("-" * 40)
    
    for error in test_errors:
        try:
            steer, accel = mpc.control(error, dt)
            speed = mpc.state['v']
            print(f"{error:5.1f}\t{steer:5.3f}\t{accel:5.3f}\t{speed:5.2f}")
        except Exception as e:
            print(f"✗ Kontrollfehler bei Error={error}: {e}")
            return False
    
    print("\n✓ MPC-Controller-Test erfolgreich abgeschlossen")
    return True

def test_fallback_mode():
    """Testet den Fallback-Modus ohne cvxpy"""
    print("\n=== Fallback-Mode Test ===")
    
    # Simuliere fehlende cvxpy-Abhängigkeit
    import sys
    old_modules = sys.modules.copy()
    if 'cvxpy' in sys.modules:
        del sys.modules['cvxpy']
    
    try:
        # Reimportiere mit simuliertem cvxpy-Fehler
        import importlib
        if 'vision_control_py' in sys.modules:
            importlib.reload(sys.modules['vision_control_py'])
        
        from vision_control_py import VisionMPCController
        mpc = VisionMPCController()
        
        # Teste Fallback-Kontrolle
        steer, accel = mpc.control(0.1, 0.05)
        print(f"✓ Fallback-Kontrolle: Steer={steer:.3f}, Accel={accel:.3f}")
        
    except Exception as e:
        print(f"✗ Fallback-Test-Fehler: {e}")
        return False
    finally:
        # Wiederherstellen der ursprünglichen Module
        sys.modules.clear()
        sys.modules.update(old_modules)
    
    return True

def main():
    """Hauptfunktion für Tests"""
    print("Vision-Controller MPC-Integration Test\n")
    
    success = True
    
    # Test 1: MPC-Controller
    if not test_mpc_controller():
        success = False
    
    # Test 2: Fallback-Modus
    if not test_fallback_mode():
        success = False
    
    # Ergebnis
    print("\n" + "="*50)
    if success:
        print("✓ Alle Tests erfolgreich!")
        return 0
    else:
        print("✗ Einige Tests fehlgeschlagen!")
        return 1

if __name__ == "__main__":
    exit(main()) 