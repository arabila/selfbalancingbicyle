#!/usr/bin/env python3
"""
Test-Script für IPC-Struktur-Größen

Testet die Kompatibilität zwischen Python struct.pack und C-Strukturen
"""

import struct
import sys

def test_struct_sizes():
    """Testet die Größen der IPC-Strukturen"""
    
    print("=== IPC-Struktur-Größen Test ===")
    
    # Original Vision-Command (alte Version)
    old_format = 'ffi'  # 2 floats + 1 int
    old_data = struct.pack(old_format, 0.5, 0.7, 1)
    print(f"Alte Vision-Command: Format='{old_format}', Größe={len(old_data)} Bytes")
    
    # Erweiterte Vision-Command (neue Version)
    new_format = 'ffifffff'  # 2 floats + 1 int + 5 floats
    new_data = struct.pack(new_format, 0.5, 0.7, 1, 0.1, 0.2, 0.3, 0.4, 25.5)
    print(f"Neue Vision-Command: Format='{new_format}', Größe={len(new_data)} Bytes")
    
    # Balance-Status
    balance_format = 'ffff'  # 4 floats
    balance_data = struct.pack(balance_format, 0.1, 0.2, 5.0, 0.3)
    print(f"Balance-Status: Format='{balance_format}', Größe={len(balance_data)} Bytes")
    
    print("\n=== Erwartete C-Struktur-Größen ===")
    print("vision_command_t: 8 Felder × 4 Bytes = 32 Bytes (ohne Padding)")
    print("balance_status_t: 4 Felder × 4 Bytes = 16 Bytes")
    
    # Teste das Entpacken der neuen Struktur
    print("\n=== Entpack-Test ===")
    try:
        unpacked = struct.unpack(new_format, new_data)
        print(f"Entpackte Daten: {unpacked}")
        print(f"steer_command: {unpacked[0]}")
        print(f"speed_command: {unpacked[1]}")
        print(f"valid: {unpacked[2]}")
        print(f"vision_error: {unpacked[3]}")
        print(f"vision_p_term: {unpacked[4]}")
        print(f"vision_i_term: {unpacked[5]}")
        print(f"vision_d_term: {unpacked[6]}")
        print(f"mask_coverage: {unpacked[7]}")
    except Exception as e:
        print(f"Fehler beim Entpacken: {e}")
    
    print("\n=== Byte-Analyse ===")
    print(f"Neue Struktur Bytes: {[hex(b) for b in new_data]}")
    
    # Teste auch mit C-natürlichem Alignment
    print("\n=== C-kompatibles Alignment ===")
    # '@' verwendet natives Alignment (wie in C)
    c_format = '@ffifffff'
    c_data = struct.pack(c_format, 0.5, 0.7, 1, 0.1, 0.2, 0.3, 0.4, 25.5)
    print(f"C-kompatibel: Format='{c_format}', Größe={len(c_data)} Bytes")

if __name__ == "__main__":
    test_struct_sizes() 