#!/usr/bin/env python3
"""
Test-Script für die Zwei-Controller-Integration

Testet die IPC-Kommunikation zwischen Balance-Controller (C) und Vision-Controller (Python)
"""

import sys
import time
import struct
from controller import Supervisor

def test_ipc_communication():
    """Testet die IPC-Kommunikation zwischen den Controllern"""
    print("🔧 Test: IPC-Kommunikation")
    print("=" * 50)
    
    # Supervisor für Test initialisieren  
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    
    print(f"Timestep: {timestep} ms")
    
    # Test-Devices holen
    try:
        command_emitter = robot.getDevice('command_tx')
        status_receiver = robot.getDevice('status_rx')
        
        if not command_emitter:
            print("❌ FEHLER: Command Emitter nicht gefunden!")
            return False
            
        if not status_receiver:
            print("❌ FEHLER: Status Receiver nicht gefunden!")
            return False
            
        status_receiver.enable(timestep)
        print("✅ IPC-Devices erfolgreich initialisiert")
        
    except Exception as e:
        print(f"❌ FEHLER bei Device-Initialisierung: {e}")
        return False
    
    # Test-Commands senden
    test_commands = [
        (0.0, 0.5),    # Geradeaus, mittlere Geschwindigkeit
        (0.2, 0.7),    # Leicht rechts, höhere Geschwindigkeit  
        (-0.1, 0.4),   # Leicht links, niedrigere Geschwindigkeit
        (0.0, 0.6),    # Wieder geradeaus
    ]
    
    print("\n📤 Sende Test-Commands...")
    
    for i, (steer, speed) in enumerate(test_commands):
        # Command senden
        command_data = struct.pack('ffi', steer, speed, 1)
        command_emitter.send(command_data)
        
        print(f"  Command {i+1}: Steer={steer:+.1f}, Speed={speed:.1f}")
        
        # Kurz warten
        for _ in range(5):
            robot.step(timestep)
            time.sleep(0.01)
    
    print("✅ Test-Commands gesendet")
    
    # Status-Empfang testen
    print("\n📥 Teste Status-Empfang...")
    status_received = False
    
    for _ in range(50):  # 50 Steps = 100ms bei 2ms Timestep
        robot.step(timestep)
        
        if status_receiver.getQueueLength() > 0:
            try:
                data = status_receiver.getData()
                status = struct.unpack('ffff', data[:16])
                
                print(f"  📊 Balance-Status empfangen:")
                print(f"     Roll-Winkel: {status[0]:+.2f} rad ({status[0]*180/3.14159:.1f}°)")
                print(f"     Lenkwinkel:  {status[1]:+.3f} rad ({status[1]*180/3.14159:.1f}°)")
                print(f"     Geschwindigkeit: {status[2]:.2f} rad/s")
                print(f"     Stabilität:  {status[3]:.2f}")
                
                status_receiver.nextPacket()
                status_received = True
                break
                
            except Exception as e:
                print(f"❌ Fehler beim Status-Parsing: {e}")
                status_receiver.nextPacket()
    
    if status_received:
        print("✅ Status-Empfang erfolgreich")
        return True
    else:
        print("⚠️  Kein Status empfangen (möglicherweise Balance-Controller nicht aktiv)")
        return False

def main():
    """Hauptfunktion für Integrations-Test"""
    print("🧪 Zwei-Controller-Integration Test")
    print("=" * 60)
    print()
    
    print("Dieser Test prüft die IPC-Kommunikation zwischen:")
    print("  • Balance-Controller (C) - empfängt Commands, sendet Status")
    print("  • Vision-Controller (Python) - sendet Commands, empfängt Status")
    print()
    
    print("⚠️  WICHTIG: Beide Controller müssen in Webots laufen!")
    print("   1. Öffne: Regelstuerung/worlds/Little Bicycle V2.wbt")
    print("   2. Starte Simulation")
    print("   3. Führe diesen Test aus")
    print()
    
    input("📋 Drücke ENTER wenn die Simulation läuft...")
    print()
    
    try:
        # Test ausführen
        success = test_ipc_communication()
        
        print("\n" + "=" * 60)
        if success:
            print("🎉 INTEGRATION TEST BESTANDEN!")
            print("   ✅ IPC-Kommunikation funktioniert")
            print("   ✅ Commands werden gesendet")
            print("   ✅ Status wird empfangen")
        else:
            print("❌ INTEGRATION TEST FEHLGESCHLAGEN!")
            print("   🔧 Prüfe Webots-Simulation und Controller")
            
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n🛑 Test durch Benutzer abgebrochen")
        
    except Exception as e:
        print(f"\n❌ UNERWARTETER FEHLER: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 
