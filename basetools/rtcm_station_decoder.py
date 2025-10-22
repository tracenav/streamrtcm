#!/usr/bin/env python3
"""
rtcm_station_decoder.py

Decode non-MSM RTCM messages:
- 1005: Stationary RTK Reference Station ARP
- 1032: Physical Reference Station Position
- 1033: Receiver and Antenna Descriptors
- 1230: GLONASS L1 and L2 Code-Phase Biases

Based on RTCM 3.3 specification.
"""

from __future__ import annotations

import sys
from typing import Dict, Optional

# Fallback BitStream implementation (same as msmbands.py)
try:
    from bitstring import BitStream
except ImportError:
    class BitStream:
        def __init__(self, data):
            self.data = data
            self.pos = 0
        
        def read(self, fmt):
            if fmt.startswith('uint:') or fmt.startswith('int:'):
                signed = fmt.startswith('int:')
                bits = int(fmt.split(':')[1])
                result = 0
                for _ in range(bits):
                    byte_idx = self.pos // 8
                    bit_idx = 7 - (self.pos % 8)
                    if byte_idx < len(self.data):
                        bit = (self.data[byte_idx] >> bit_idx) & 1
                        result = (result << 1) | bit
                    self.pos += 1
                
                if signed and bits > 0:
                    sign_bit = result >> (bits - 1)
                    if sign_bit:
                        result = result - (1 << bits)
                
                return result
            raise ValueError(f"Unsupported format: {fmt}")


def decode_1005(payload: bytes) -> Optional[Dict]:
    """
    Decode RTCM 1005: Stationary RTK Reference Station ARP
    Contains: station ID, ITRF realization year, GPS/GLO/GAL indicators, 
              ECEF coordinates, antenna height
    """
    bs = BitStream(payload)
    
    msg_type = bs.read('uint:12')
    if msg_type != 1005:
        return None
    
    station_id = bs.read('uint:12')
    itrf_year = bs.read('uint:6')  # ITRF realization year (0=unknown)
    gps_ind = bs.read('uint:1')    # GPS indicator
    glo_ind = bs.read('uint:1')    # GLONASS indicator
    reserved = bs.read('uint:1')
    gal_ind = bs.read('uint:1')    # Galileo indicator
    ref_station_ind = bs.read('uint:1')  # Reference station indicator
    
    # ECEF coordinates (1mm resolution, shifted by -2^38 meters)
    ecef_x_raw = bs.read('int:38')
    ecef_y_raw = bs.read('int:38')
    ecef_z_raw = bs.read('int:38')
    
    # Convert to meters
    ecef_x = ecef_x_raw * 0.0001
    ecef_y = ecef_y_raw * 0.0001
    ecef_z = ecef_z_raw * 0.0001
    
    return {
        "msg_type": 1005,
        "station_id": station_id,
        "itrf_year": itrf_year if itrf_year > 0 else "Unknown",
        "gps": bool(gps_ind),
        "glonass": bool(glo_ind),
        "galileo": bool(gal_ind),
        "reference_station": bool(ref_station_ind),
        "ecef_x_m": ecef_x,
        "ecef_y_m": ecef_y,
        "ecef_z_m": ecef_z,
    }


def decode_1032(payload: bytes) -> Optional[Dict]:
    """
    Decode RTCM 1032: Physical Reference Station Position
    Contains: non-physical station ID, physical station ID, ITRF year,
              physical reference point ECEF coordinates (NO antenna height)
    """
    bs = BitStream(payload)
    
    msg_type = bs.read('uint:12')
    if msg_type != 1032:
        return None
    
    non_physical_station_id = bs.read('uint:12')
    physical_station_id = bs.read('uint:12')
    itrf_year = bs.read('uint:6')
    
    # Physical reference point ECEF (0.1mm resolution)
    ecef_x_raw = bs.read('int:38')
    ecef_y_raw = bs.read('int:38')
    ecef_z_raw = bs.read('int:38')
    
    ecef_x = ecef_x_raw * 0.0001
    ecef_y = ecef_y_raw * 0.0001
    ecef_z = ecef_z_raw * 0.0001
    
    return {
        "msg_type": 1032,
        "non_physical_station_id": non_physical_station_id,
        "physical_station_id": physical_station_id,
        "itrf_year": itrf_year if itrf_year > 0 else "Unknown",
        "physical_ecef_x_m": ecef_x,
        "physical_ecef_y_m": ecef_y,
        "physical_ecef_z_m": ecef_z,
    }


def decode_1033(payload: bytes) -> Optional[Dict]:
    """
    Decode RTCM 1033: Receiver and Antenna Descriptors
    Contains: station ID, antenna descriptor, antenna serial, 
              receiver descriptor, receiver serial, receiver firmware
    """
    bs = BitStream(payload)
    
    msg_type = bs.read('uint:12')
    if msg_type != 1033:
        return None
    
    station_id = bs.read('uint:12')
    
    # Antenna descriptor string (N chars)
    ant_desc_count = bs.read('uint:8')
    ant_desc_bytes = []
    for _ in range(ant_desc_count):
        ant_desc_bytes.append(bs.read('uint:8'))
    antenna_descriptor = bytes(ant_desc_bytes).decode('ascii', errors='ignore')
    
    # Antenna setup ID
    antenna_setup_id = bs.read('uint:8')
    
    # Antenna serial number (N chars)
    ant_serial_count = bs.read('uint:8')
    ant_serial_bytes = []
    for _ in range(ant_serial_count):
        ant_serial_bytes.append(bs.read('uint:8'))
    antenna_serial = bytes(ant_serial_bytes).decode('ascii', errors='ignore')
    
    # Receiver descriptor (N chars)
    rcv_desc_count = bs.read('uint:8')
    rcv_desc_bytes = []
    for _ in range(rcv_desc_count):
        rcv_desc_bytes.append(bs.read('uint:8'))
    receiver_descriptor = bytes(rcv_desc_bytes).decode('ascii', errors='ignore')
    
    # Receiver firmware version (N chars)
    rcv_fw_count = bs.read('uint:8')
    rcv_fw_bytes = []
    for _ in range(rcv_fw_count):
        rcv_fw_bytes.append(bs.read('uint:8'))
    receiver_firmware = bytes(rcv_fw_bytes).decode('ascii', errors='ignore')
    
    # Receiver serial number (N chars)
    rcv_serial_count = bs.read('uint:8')
    rcv_serial_bytes = []
    for _ in range(rcv_serial_count):
        rcv_serial_bytes.append(bs.read('uint:8'))
    receiver_serial = bytes(rcv_serial_bytes).decode('ascii', errors='ignore')
    
    return {
        "msg_type": 1033,
        "station_id": station_id,
        "antenna_descriptor": antenna_descriptor.strip(),
        "antenna_setup_id": antenna_setup_id,
        "antenna_serial": antenna_serial.strip(),
        "receiver_descriptor": receiver_descriptor.strip(),
        "receiver_firmware": receiver_firmware.strip(),
        "receiver_serial": receiver_serial.strip(),
    }


def decode_1230(payload: bytes) -> Optional[Dict]:
    """
    Decode RTCM 1230: GLONASS L1 and L2 Code-Phase Biases
    Contains: station ID, code-phase bias indicator, signal mask, and biases
    """
    bs = BitStream(payload)
    
    msg_type = bs.read('uint:12')
    if msg_type != 1230:
        return None
    
    station_id = bs.read('uint:12')
    code_phase_bias_ind = bs.read('uint:1')  # 0=no bias, 1=bias present
    reserved = bs.read('uint:3')
    fdma_signal_mask = bs.read('uint:4')  # bit mask for L1 C/A, L1 P, L2 C/A, L2 P
    
    biases = {}
    
    # L1 C/A bias (if bit 0 set)
    if fdma_signal_mask & 0b1000:
        l1_ca_bias_raw = bs.read('int:16')
        biases['L1_CA'] = l1_ca_bias_raw * 0.02  # 0.02m resolution
    
    # L1 P bias (if bit 1 set)
    if fdma_signal_mask & 0b0100:
        l1_p_bias_raw = bs.read('int:16')
        biases['L1_P'] = l1_p_bias_raw * 0.02
    
    # L2 C/A bias (if bit 2 set)
    if fdma_signal_mask & 0b0010:
        l2_ca_bias_raw = bs.read('int:16')
        biases['L2_CA'] = l2_ca_bias_raw * 0.02
    
    # L2 P bias (if bit 3 set)
    if fdma_signal_mask & 0b0001:
        l2_p_bias_raw = bs.read('int:16')
        biases['L2_P'] = l2_p_bias_raw * 0.02
    
    return {
        "msg_type": 1230,
        "station_id": station_id,
        "bias_present": bool(code_phase_bias_ind),
        "signals": {
            "L1_CA": bool(fdma_signal_mask & 0b1000),
            "L1_P": bool(fdma_signal_mask & 0b0100),
            "L2_CA": bool(fdma_signal_mask & 0b0010),
            "L2_P": bool(fdma_signal_mask & 0b0001),
        },
        "biases_m": biases,
    }


def parse_log_file(log_path: str):
    """Parse log file and extract RTCM messages."""
    messages = []
    
    with open(log_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or not 'RTCM' in line:
                continue
            
            parts = line.split()
            if len(parts) >= 5:
                # Format: timestamp direction RTCM msg_type length hex_data
                msg_type = parts[3]
                hex_data = parts[5] if len(parts) > 5 else ""
                
                # Remove any prefix garbage from hex data
                if hex_data.startswith('ca'):
                    hex_data = hex_data[2:]  # Remove 'ca' prefix
                
                messages.append((msg_type, hex_data))
    
    return messages


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Decode RTCM station metadata messages")
    parser.add_argument("log_file", nargs='?', default="basetools/testfull.log",
                       help="Log file containing RTCM messages (default: basetools/testfull.log)")
    args = parser.parse_args()
    
    print("RTCM Station and Reference Information Decoder")
    print("=" * 80)
    print(f"Reading from: {args.log_file}\n")
    
    try:
        test_messages = parse_log_file(args.log_file)
    except FileNotFoundError:
        print(f"Error: File not found: {args.log_file}")
        return
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    for label, hex_data in test_messages:
        frame = bytes.fromhex(hex_data)
        payload = frame[3:-3]  # Remove 3-byte header and 3-byte CRC
        
        msg_type_check = int.from_bytes(payload[0:2], 'big') >> 4
        
        print(f"\n{'='*80}")
        print(f"Message Type: {msg_type_check}")
        print("=" * 80)
        
        if msg_type_check == 1005:
            result = decode_1005(payload)
            if result:
                print(f"Station ID: {result['station_id']}")
                print(f"ITRF Year: {result['itrf_year']}")
                print(f"Systems: GPS={result['gps']}, GLONASS={result['glonass']}, Galileo={result['galileo']}")
                print(f"Reference Station: {result['reference_station']}")
                print(f"\nAntenna Reference Point (ARP) - ECEF Coordinates:")
                print(f"  X: {result['ecef_x_m']:15.4f} meters")
                print(f"  Y: {result['ecef_y_m']:15.4f} meters")
                print(f"  Z: {result['ecef_z_m']:15.4f} meters")
        
        elif msg_type_check == 1032:
            result = decode_1032(payload)
            if result:
                print(f"Non-Physical Station ID: {result['non_physical_station_id']}")
                print(f"Physical Station ID: {result['physical_station_id']}")
                print(f"ITRF Year: {result['itrf_year']}")
                print(f"\nPhysical Reference Point - ECEF Coordinates:")
                print(f"  X: {result['physical_ecef_x_m']:15.4f} meters")
                print(f"  Y: {result['physical_ecef_y_m']:15.4f} meters")
                print(f"  Z: {result['physical_ecef_z_m']:15.4f} meters")
                print(f"\nNote: Message 1032 links non-physical to physical reference station")
                print(f"      (no antenna height included, see 1005/1006 for ARP)")
        
        elif msg_type_check == 1033:
            result = decode_1033(payload)
            if result:
                print(f"Station ID: {result['station_id']}")
                print(f"\nAntenna Information:")
                print(f"  Descriptor: {result['antenna_descriptor']}")
                print(f"  Setup ID: {result['antenna_setup_id']}")
                print(f"  Serial: {result['antenna_serial']}")
                print(f"\nReceiver Information:")
                print(f"  Descriptor: {result['receiver_descriptor']}")
                print(f"  Firmware: {result['receiver_firmware']}")
                print(f"  Serial: {result['receiver_serial']}")
        
        elif msg_type_check == 1230:
            result = decode_1230(payload)
            if result:
                print(f"Station ID: {result['station_id']}")
                print(f"Bias Present: {result['bias_present']}")
                print(f"\nSignal Mask:")
                for sig, present in result['signals'].items():
                    print(f"  {sig}: {present}")
                if result['biases_m']:
                    print(f"\nCode-Phase Biases:")
                    for sig, bias in result['biases_m'].items():
                        print(f"  {sig}: {bias:8.4f} meters")
                else:
                    print("\nNo biases reported (all zeros)")


if __name__ == "__main__":
    main()

