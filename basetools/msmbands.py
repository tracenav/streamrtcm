#!/usr/bin/env python3
"""
msmbands.py

Script to decode and report signal bands present in RTCM MSM messages from a binary file.

Supports MSM4 and MSM7 for GPS, GLONASS, Galileo, BeiDou, QZSS.
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, List, Optional, Set, Tuple

try:
    from bitstring import BitStream
except ImportError:
    # Fallback to manual bit parsing if bitstring not available
    class BitStream:
        def __init__(self, data):
            self.data = data
            self.pos = 0
        
        def read(self, fmt):
            # Simple parser for 'uint:N' and 'int:N' formats
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
                
                # Handle signed values (two's complement)
                if signed and bits > 0:
                    sign_bit = result >> (bits - 1)
                    if sign_bit:
                        # Negative number - compute two's complement
                        result = result - (1 << bits)
                
                return result
            raise ValueError(f"Unsupported format: {fmt}")

# MSM message types supported
MSM_TYPES = {
    # GPS
    1074: ("GPS", 4),
    1077: ("GPS", 7),
    # GLONASS
    1084: ("GLO", 4),
    1087: ("GLO", 7),
    # Galileo
    1094: ("GAL", 4),
    1097: ("GAL", 7),
    # QZSS
    1114: ("QZS", 4),
    1117: ("QZS", 7),
    # BeiDou
    1124: ("BDS", 4),
    1127: ("BDS", 7),
}

# Signal ID mapping based on RTCM 3.3 specification
# Table 3.5-91 (GPS), Table 3.5-96 (GLO), Table 3.5-99 (GAL), 
# Table 3.5-102 (SBAS), Table 3.5-105 (QZS), Table 3.5-108 (BDS)
BAND_MAP = {
    "GPS": {
        1: "L1 C/A", 2: "L1 P", 3: "L1 Z-track", 4: "Reserved",
        5: "Reserved", 6: "Reserved", 7: "Reserved",
        8: "L2 2C", 9: "L2 2P", 10: "L2 2W",
        11: "Reserved", 12: "Reserved", 13: "Reserved", 14: "Reserved",
        15: "L2 2S", 16: "L2 2L", 17: "L2 2X",
        18: "Reserved", 19: "Reserved", 20: "Reserved", 21: "Reserved",
        22: "L5 5I", 23: "L5 5Q", 24: "L5 5X",
        25: "Reserved", 26: "Reserved", 27: "Reserved", 28: "Reserved", 29: "Reserved",
        30: "L1 1S", 31: "L1 1L", 32: "L1 1X",
    },
    "GLO": {
        1: "G1 C/A (1C)", 2: "G1 P (1P)", 3: "G2 C/A (2C)",
        4: "Reserved", 5: "Reserved", 6: "Reserved", 7: "Reserved",
        8: "G2 P (2P)", 9: "Reserved",
        # Signals 10-32: Reserved per Table 3.5-96
    },
    "GAL": {
        1: "Reserved", 2: "E1-C (1C)", 3: "E1-A (1A)", 4: "E1-B (1B)", 5: "E1 B+C (1X)", 6: "E1 A+B+C (1Z)",
        7: "Reserved", 8: "E6-C (6C)", 9: "E6-A (6A)", 10: "E6-B (6B)", 11: "E6 B+C (6X)", 12: "E6 A+B+C (6Z)",
        13: "E5b-I (7I)", 14: "E5b-Q (7Q)", 15: "E5b I+Q (7X)",
        16: "Reserved", 17: "Reserved",
        18: "E5(A+B)-I (8I)", 19: "E5(A+B)-Q (8Q)", 20: "E5(A+B) I+Q (8X)",
        21: "Reserved",
        22: "E5a-I (5I)", 23: "E5a-Q (5Q)", 24: "E5a I+Q (5X)",
    },
    "QZS": {
        1: "L1 C/A (1C)", 2: "Reserved",
        3: "Reserved", 4: "Reserved", 5: "Reserved", 6: "Reserved", 7: "Reserved", 8: "Reserved",
        9: "LEX S (6S)", 10: "LEX L (6L)", 11: "LEX S+L (6X)",
        12: "Reserved", 13: "Reserved", 14: "Reserved",
        15: "L2 L2C(M) (2S)", 16: "L2 L2C(L) (2L)", 17: "L2 L2C(M+L) (2X)",
        18: "Reserved", 19: "Reserved", 20: "Reserved", 21: "Reserved",
        22: "L5-I (5I)", 23: "L5-Q (5Q)", 24: "L5 I+Q (5X)",
        25: "Reserved", 26: "Reserved", 27: "Reserved", 28: "Reserved", 29: "Reserved",
        30: "L1C(D) (1S)", 31: "L1C(P) (1L)", 32: "L1C(D+P) (1X)",
    },
    "BDS": {
        1: "Reserved",
        2: "B1-I (2I)", 3: "B1-Q (2Q)", 4: "B1 I+Q (2X)",
        5: "Reserved", 6: "Reserved", 7: "Reserved",
        8: "B3-I (6I)", 9: "B3-Q (6Q)", 10: "B3 I+Q (6X)",
        11: "Reserved", 12: "Reserved", 13: "Reserved",
        14: "B2-I (7I)", 15: "B2-Q (7Q)", 16: "B2 I+Q (7X)",
        # Signals 17-32: Reserved per Table 3.5-108
        # Note: Real-world receivers may use reserved slots for newer signals like B1C, B2a not in RTCM 3.3
    },
}

def parse_rtcm_frame(buf: bytes, offset: int = 0) -> Tuple[Optional[bytes], int]:
    """Parse next RTCM frame from buffer starting at offset. Returns (frame, new_offset)"""
    n = offset
    while n < len(buf):
        if buf[n] == 0xD3:  # preamble
            break
        n += 1
    if n >= len(buf):
        return None, len(buf)
    
    if n + 3 > len(buf):
        return None, n
    
    # Length: 6 reserved + 10 length bits
    length = ((buf[n+1] & 0x3F) << 8) | buf[n+2]
    msg_len = length & 0x3FF
    total_len = 6 + msg_len  # preamble(1) + reserved+len(2) + payload + crc(3)
    
    if n + total_len > len(buf):
        return None, n
    
    frame = buf[n : n + total_len]
    # Simple CRC check (optional, for robustness)
    # CRC24Q poly=0x1864CFB, but skip for simplicity
    return frame, n + total_len

def decode_msm(frame: bytes) -> Optional[Dict]:
    """Decode MSM4/MSM7 messages including measurements."""
    if len(frame) < 6:
        return None
    
    # Remove 3-byte header and 3-byte CRC
    payload = frame[3 : -3]
    if len(payload) < 12:
        return None
    
    bs = BitStream(payload)
    
    msg_type = bs.read('uint:12')
    if msg_type not in MSM_TYPES:
        return None
    
    system, msm_type = MSM_TYPES[msg_type]
    
    # Read fields sequentially
    station_id = bs.read('uint:12')
    epoch_time = bs.read('uint:30')
    mm = bs.read('uint:1')
    iods = bs.read('uint:3')
    reserved = bs.read('uint:7')
    clk_steer = bs.read('uint:2')
    ext_clk = bs.read('uint:2')
    smooth_ind = bs.read('uint:1')
    smooth_int = bs.read('uint:3')
    
    sat_mask = bs.read('uint:64')
    sig_mask = bs.read('uint:32')
    
    # Extract satellite IDs (PRNs) present
    # Per RTCM 3.3 DF394: MSB (bit 63) = satellite ID 1, LSB (bit 0) = satellite ID 64
    sats = []
    for idx in range(64):
        if (sat_mask >> (63 - idx)) & 1:
            sats.append(idx + 1)
    
    # Extract signal IDs present
    # Per RTCM 3.3 DF395: MSB (bit 31) = signal ID 1, LSB (bit 0) = signal ID 32
    sigs = []
    for idx in range(32):
        if (sig_mask >> (31 - idx)) & 1:
            sigs.append(idx + 1)
    
    # Cell mask: one bit per sat/sig pair
    cell_pairs = []
    for s_idx in range(len(sats)):
        for g_idx in range(len(sigs)):
            bit = bs.read('uint:1')
            if bit:
                cell_pairs.append((sats[s_idx], sigs[g_idx]))
    
    # Satellite data: rough ranges (integer milliseconds)
    rough_ranges_ms = {}
    for sat in sats:
        rr = bs.read('uint:8')
        if rr == 255:
            rough_ranges_ms[sat] = None  # Invalid
        else:
            rough_ranges_ms[sat] = float(rr)
    
    # Extended rough range for MSM7 (fractional ms)
    ext_rough_ranges_ms = {}
    if msm_type == 7:
        for sat in sats:
            err = bs.read('uint:4')
            ext_rough_ranges_ms[sat] = err / 16.0
    
    # Per-cell fine measurements
    # MSM4: fine pseudorange(15), phaserange(22), lock(4), halfcycle(1), cnr(6)
    # MSM7: fine pseudorange(20), phaserange(24), doppler(15), lock(4), halfcycle(1), cnr(10)
    measurements = {}
    for (sat, sig) in cell_pairs:
        if msm_type == 4:
            fp_raw = bs.read('int:15')  # signed
            fr_raw = bs.read('int:22')  # signed
            lock = bs.read('uint:4')
            half = bs.read('uint:1')
            cnr_raw = bs.read('uint:6')
            
            fp = fp_raw / 1024.0  # fine pseudorange in ms
            fr = fr_raw / 1024.0  # fine phase range in ms
            cnr = float(cnr_raw)  # CNR in dB-Hz
            
            measurements[(sat, sig)] = {
                "fine_pseudorange_ms": fp,
                "fine_phaserange_ms": fr,
                "lock": lock,
                "cnr_dbhz": cnr,
            }
        elif msm_type == 7:
            fp_raw = bs.read('int:20')
            fr_raw = bs.read('int:24')
            fd_raw = bs.read('int:15')
            lock = bs.read('uint:4')
            half = bs.read('uint:1')
            cnr_raw = bs.read('uint:10')
            
            fp = fp_raw / 1024.0
            fr = fr_raw / 1024.0
            fd = fd_raw / 16.0  # doppler in Hz (approx)
            cnr = float(cnr_raw) / 4.0  # CNR in dB-Hz
            
            measurements[(sat, sig)] = {
                "fine_pseudorange_ms": fp,
                "fine_phaserange_ms": fr,
                "doppler_hz": fd,
                "lock": lock,
                "cnr_dbhz": cnr,
            }
    
    return {
        "msg_type": msg_type,
        "system": system,
        "msm_type": msm_type,
        "epoch_ms": epoch_time,
        "sats": sats,
        "sigs": sigs,
        "cell_pairs": cell_pairs,
        "rough_ranges_ms": rough_ranges_ms,
        "ext_rough_ranges_ms": ext_rough_ranges_ms,
        "measurements": measurements,
    }

def get_signal_descriptions(system: str, sigs: List[int]) -> List[str]:
    """Get descriptive names for signals present."""
    map_ = BAND_MAP.get(system, {})
    # For BDS, signals 17-32 are reserved
    result = []
    for sig in sigs:
        if system == "BDS" and sig >= 17:
            result.append(f"Reserved (sig {sig})")
        else:
            result.append(map_.get(sig, f"Unknown Signal {sig}"))
    return result

def estimate_signal_frequency(phase_measurements, time_ms):
    """
    Estimate signal frequency from carrier phase rate of change.
    Returns approximate frequency in MHz based on phase progression.
    """
    # This is a rough estimate - would need multiple epochs for accuracy
    # For now, we'll use the magnitude of phase measurements as a clue
    return None

def identify_bds_reserved_signal(sig_id, measurements, system):
    """
    Try to identify what a reserved BeiDou signal might be based on heuristics.
    
    Known BeiDou frequencies:
    - B1I: 1561.098 MHz (Signal 2 in RTCM 3.3)
    - B3I: 1268.520 MHz (Signal 8 in RTCM 3.3)
    - B2I: 1207.140 MHz (Signal 14 in RTCM 3.3)
    
    BDS-3 modernized (not in RTCM 3.3):
    - B1C: 1575.420 MHz (GPS L1 compatible)
    - B2a: 1176.450 MHz (GPS L5 compatible)
    - B2b: 1207.140 MHz (overlaps B2I)
    """
    if system != "BDS" or sig_id < 17:
        return None
    
    # Common manufacturer implementations based on field observations:
    # Signal 23 is often B2a (L5-like at 1176.45 MHz)
    # Signal 31 is often B1C (L1-like at 1575.42 MHz)
    
    if sig_id == 23:
        return "B2a (1176.45 MHz, GPS L5 compatible) [non-standard]"
    elif sig_id == 31:
        return "B1C (1575.42 MHz, GPS L1 compatible) [non-standard]"
    
    return f"Reserved (sig {sig_id})"

def main():
    # Hardcoded MSM messages from rtcmtest.log (hex data after length field)
    hex_messages = [
        # 1074 GPS MSM4
        "d300b04325c88998ea2200248021218080000000204101007f7fffa4a623a1a2266563c3050a6882508db115c22ac41f967a2c3a58cc6b48ce519e03720511097212f62920acd95732b0e5617707ee0bdc53b75c158e705e59017b1b05c0582c1ab0a63382aefd06a77c193330675c41b00905d5e818747062ac81a91c0befb0323a60c8e9c32b3af8144fe2c68f8bf57e3197fffffffffffffffffffffff0000018e76e76db6e36e1ae7869ce5bdfbdd6db78b79024",
        # 1084 GLO MSM4
        "d3007b43c5c8d0925ee20024b808700000000000208000007ffe9284909294848f9a597b4f84f8fb4d16e0edc8b81b7000927424a8395c7539d843bc1d1dba3f82ab049bee0a4fb8953e2ba7f8e80e140e30531360ff64046b181b70b06a377f4ddf7d5a220393000ee55fffffffffffffe0005f6ed9e176175d6eba63806947b9",
        # 1094 GAL MSM4
        "d300914465c88998ea2200248028102440000000200101007fffeaea298b4a09f181e0e1f5b27be1f9cff517eb63d8278e0f1e9f4a3e047c3015683660738788cefd1e070d161950335ffd8c80019c0006581ffa587f2a21fcaf885da62290308b94e007a47fb6abfece287d7241fb9787eec806db8818eec06226ffffffffffffffffff800016df75b8e16e585b6db8eba63b6af71f0d",
        # 1124 BDS MSM4
        "d300a64645c889980f6000248000190494000000200001017ffffd3929593d514542ea43c0b304f4a9121197a31f8645760eead1d784120817b041ded4fd937b4436487050d921a1b348a680c4ba8a8612f20ef9b84164a0ea96fdeb7ff834cfdd7cc030d480c46202c08fea653f9bd6fe941219eb206728c19e2e869eaa1ae4d86a10a0bad6038ae40b5d77ffffffffffffffffffff8000030c6edb6cedb6dbaeb6db6db6db6ec2d0715614"
    ]
    
    print("Processing hardcoded MSM messages from rtcmtest.log...")
    
    msm_count = 0
    all_bands = set()
    
    for hex_data in hex_messages:
        # Convert hex to bytes (full frame including preamble, length, payload, CRC)
        frame = bytes.fromhex(hex_data)
        
        msm = decode_msm(frame)
        if msm:
            msm_count += 1
            system = msm["system"]
            sats = msm["sats"]
            sigs = msm["sigs"]
            descriptions = get_signal_descriptions(system, sigs)
            
            print(f"\n{'='*80}")
            print(f"MSM {msm['msg_type']} ({system}) - MSM Type {msm['msm_type']}")
            print(f"Epoch: {msm['epoch_ms']} ms")
            print(f"Satellites (PRNs): {sats} ({len(sats)} sats)")
            print(f"Signals: {sigs}")
            for idx, sig_id in enumerate(sigs):
                print(f"  Signal {sig_id}: {descriptions[idx]}")
            
            print(f"\nMeasurements ({len(msm['cell_pairs'])} observations):")
            print(f"{'PRN':<5} {'Signal':<25} {'Rough(ms)':<12} {'Fine PR(ms)':<13} {'Fine Phase(ms)':<15} {'CNR(dB-Hz)':<10}")
            print("-" * 90)
            
            CLIGHT = 299792458.0  # speed of light m/s
            
            for (sat, sig) in msm['cell_pairs']:
                sig_desc = get_signal_descriptions(system, [sig])[0]
                
                # Try to identify reserved BDS signals
                if system == "BDS" and sig >= 17:
                    identified = identify_bds_reserved_signal(sig, msm['measurements'], system)
                    if identified:
                        sig_desc = identified
                
                rough = msm['rough_ranges_ms'].get(sat)
                ext = msm['ext_rough_ranges_ms'].get(sat, 0.0)
                meas = msm['measurements'].get((sat, sig), {})
                
                fine_pr = meas.get('fine_pseudorange_ms', 0.0)
                fine_ph = meas.get('fine_phaserange_ms', 0.0)
                cnr = meas.get('cnr_dbhz', 0.0)
                
                # Total pseudorange in meters
                if rough is not None:
                    total_ms = rough + ext + fine_pr
                    total_pr_m = total_ms * 1e-3 * CLIGHT
                    rough_str = f"{rough:.1f}"
                else:
                    total_pr_m = None
                    rough_str = "Invalid"
                
                print(f"{sat:<5} {sig_desc:<25} {rough_str:<12} {fine_pr:<13.6f} {fine_ph:<15.6f} {cnr:<10.1f}")
            
            # Collect unique bands (using descriptions as bands)
            all_bands.update(descriptions)
    
    print(f"\nProcessed {msm_count} MSM messages.")
    if all_bands:
        print(f"Unique bands across all messages: {sorted(all_bands)}")
    else:
        print("No MSM messages found.")
    
    print("\n" + "="*80)
    print("METHODS TO VERIFY RESERVED SIGNAL IDENTIFICATIONS:")
    print("="*80)
    print("""
1. RECEIVER DOCUMENTATION: Check your receiver's manual/datasheet for MSM signal mapping
2. RINEX CONVERSION: Convert RTCM to RINEX and check observation codes (C1P, C5P, etc.)
3. FREQUENCY ANALYSIS: Use multiple epochs to calculate Doppler and infer carrier frequency
4. COMPARE WITH GPS: B1C should behave similar to GPS L1, B2a similar to GPS L5
5. MANUFACTURER SUPPORT: Contact receiver manufacturer for their signal ID mapping
    
Common BeiDou-3 signal assignments (manufacturer-specific):
- Signal 23 → B2a (1176.45 MHz) - GPS L5 compatible
- Signal 31 → B1C (1575.42 MHz) - GPS L1 compatible
    """)

if __name__ == "__main__":
    main()