#!/usr/bin/env python3
"""
Decode SPARTN frames embedded in UBX-RXM-PMP files.

Input: UBX-RXM-PMP binary files (e.g., in ./5minset/ or ./spartnLogs/)
Output: Prints one JSON line per decoded SPARTN frame with key header fields.
Message type basics (columns are type and subtype as separate fields):

Type 0, Subtype 0: OCB (GPS)
Type 0, Subtype 1: OCB (GLONASS)
Type 0, Subtype 2: OCB (Galileo)
Type 0, Subtype 3: OCB (BeiDou)
Type 1, Subtype 0: HPAC 1-0
Type 1, Subtype 1: HPAC 1-1
Type 1, Subtype 2: HPAC 1-2
Type 1, Subtype 3: HPAC 1-3
Type 2, Subtype 0: GAD
References:
- SPARTN ICD 2.0.2 (220221_SPARTN_v2.0.2.txt), Section 7 Transport Layer
"""
import os
import sys
import json
import glob
import argparse
import subprocess
import re
import time
from pathlib import Path
from typing import Optional, Tuple, List, Union


class BitReader:
    def __init__(self, data: bytes, bit_offset: int = 0):
        self.data = data
        self.bit_offset = bit_offset  # offset in bits from start of data

    def remaining_bits(self) -> int:
        return len(self.data) * 8 - self.bit_offset

    def align_to_next_byte(self):
        mod = self.bit_offset % 8
        if mod:
            self.bit_offset += (8 - mod)

    def read_bits(self, nbits: int) -> int:
        if nbits == 0:
            return 0
        if nbits < 0 or self.remaining_bits() < nbits:
            raise ValueError("Not enough bits to read")
        result = 0
        for _ in range(nbits):
            byte_index = self.bit_offset // 8
            bit_index = 7 - (self.bit_offset % 8)
            bit = (self.data[byte_index] >> bit_index) & 1
            result = (result << 1) | bit
            self.bit_offset += 1
        return result

    def read_bytes(self, nbytes: int) -> bytes:
        self.align_to_next_byte()
        start_byte = self.bit_offset // 8
        end_byte = start_byte + nbytes
        if end_byte > len(self.data):
            raise ValueError("Not enough bytes to read")
        self.bit_offset += nbytes * 8
        return self.data[start_byte:end_byte]


def ubx_parse_rxm_pmp(buf: bytes) -> Optional[bytes]:
    """
    Parse a single UBX-RXM-PMP message and return the userData bytes which contain SPARTN stream.

    UBX header: 0xB5 0x62 class=0x02 id=0x72 len(LSB,MSB) payload ... CK_A CK_B
    Within payload: numBytesUserData at payload[2..3] (LE), userData starts at payload[24]
    """
    if len(buf) < 8:
        return None
    if not (buf[0] == 0xB5 and buf[1] == 0x62 and buf[2] == 0x02 and buf[3] == 0x72):
        return None
    length = buf[4] | (buf[5] << 8)
    if len(buf) < 6 + length + 2:
        return None
    payload = buf[6:6 + length]
    # Simple checksum check (CK_A, CK_B)
    ck_a = 0
    ck_b = 0
    for b in buf[2:6 + length]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    if ck_a != buf[6 + length] or ck_b != buf[6 + length + 1]:
        # Still proceed; some captures may truncate checksums
        pass
    if len(payload) < 24:
        return None
    num_user = payload[2] | (payload[3] << 8)
    if num_user <= 0:
        return b""
    start = 24
    end = start + num_user
    if end > len(payload):
        # Fallback to available bytes
        end = len(payload)
    return payload[start:end]


CRC_TYPE_TO_BYTES = {0: 1, 1: 2, 2: 3, 3: 4}
AUTH_LEN_TO_BYTES = {0: 8, 1: 12, 2: 16, 3: 32, 4: 64}


def parse_spartn_frame(data: bytes, start_index: int) -> Optional[Tuple[dict, int, bytes, Union[bytes, None]]]:
    """
    Attempt to parse a SPARTN frame starting at data[start_index].
    Returns (record, next_index) on success; None if cannot parse a valid frame here.
    """
    if start_index >= len(data):
        return None
    if data[start_index] != 0x73:  # TF001 preamble
        return None
    reader = BitReader(data, bit_offset=(start_index + 1) * 8)
    try:
        msg_type = reader.read_bits(7)            # TF002
        payload_length = reader.read_bits(10)     # TF003 (bytes)
        eaf = reader.read_bits(1)                 # TF004
        crc_type = reader.read_bits(2)            # TF005
        frame_crc4 = reader.read_bits(4)          # TF006

        # Payload Description Block (TF007 - TF015)
        subtype = reader.read_bits(4)             # TF007
        time_tag_type = reader.read_bits(1)       # TF008
        if time_tag_type == 0:
            time_tag = reader.read_bits(16)       # half-day seconds (ambiguous)
        else:
            time_tag = reader.read_bits(32)       # seconds since 2010-01-01
        solution_id = reader.read_bits(7)         # TF010
        solution_proc_id = reader.read_bits(4)    # TF011

        encryption_id = None
        enc_seq_num = None
        auth_indicator = None
        embedded_auth_len_sel = None
        if eaf == 1:
            encryption_id = reader.read_bits(4)   # TF012
            enc_seq_num = reader.read_bits(6)     # TF013
            auth_indicator = reader.read_bits(3)  # TF014
            if auth_indicator > 1:
                embedded_auth_len_sel = reader.read_bits(3)  # TF015

        # Payload (TF016)
        payload_bytes = reader.read_bytes(payload_length) if payload_length > 0 else b""

        # Embedded Authentication Data (TF017)
        embedded_auth = b""
        if eaf == 1 and auth_indicator is not None and auth_indicator > 1:
            auth_bytes = AUTH_LEN_TO_BYTES.get(embedded_auth_len_sel, 0)
            if auth_bytes > 0:
                embedded_auth = reader.read_bytes(auth_bytes)

        # Message CRC (TF018)
        crc_nbytes = CRC_TYPE_TO_BYTES.get(crc_type, 0)
        msg_crc = reader.read_bytes(crc_nbytes) if crc_nbytes > 0 else b""

        next_index = (reader.bit_offset + 7) // 8
        record = {
            "preamble": 0x73,
            "type": msg_type,
            "subtype": subtype,
            "payload_len": payload_length,
            "eaf": eaf,
            "crc_type": crc_type,
            "frame_crc4": frame_crc4,
            "time_tag_type": time_tag_type,
            "time_tag": time_tag,
            "solution_id": solution_id,
            "solution_proc_id": solution_proc_id,
        }
        if eaf == 1:
            record.update({
                "encryption_id": encryption_id,
                "enc_seq_num": enc_seq_num,
                "auth_indicator": auth_indicator,
            })
            if auth_indicator and auth_indicator > 1:
                record.update({
                    "embedded_auth_len_sel": embedded_auth_len_sel,
                    "embedded_auth_len_bytes": AUTH_LEN_TO_BYTES.get(embedded_auth_len_sel, 0),
                })

        # Include a short preview of payload
        if payload_bytes:
            preview = payload_bytes[:8].hex()
            record["payload_preview_hex"] = preview + ("..." if len(payload_bytes) > 8 else "")

        return record, next_index, payload_bytes, embedded_auth
    except Exception:
        return None


def extract_spartn_frames(userdata: bytes) -> List[Tuple[dict, bytes, Union[bytes, None]]]:
    frames: List[Tuple[dict, bytes, Union[bytes, None]]] = []
    i = 0
    n = len(userdata)
    while i < n:
        # Find preamble 0x73
        j = userdata.find(b"\x73", i)
        if j < 0:
            break
        parsed = parse_spartn_frame(userdata, j)
        if parsed is None:
            # Not a valid SPARTN frame here; move past this byte
            i = j + 1
            continue
        rec, next_i, payload_bytes, embedded_auth = parsed
        frames.append((rec, payload_bytes, embedded_auth))
        if next_i <= j:
            # Safety to avoid infinite loop
            i = j + 1
        else:
            i = next_i
    return frames


def decode_file(path: str) -> List[dict]:
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except Exception as e:
        sys.stderr.write(f"Failed to read {path}: {e}\n")
        return []

    # Try as UBX-RXM-PMP first
    user = ubx_parse_rxm_pmp(data)
    if user is not None and len(user) > 0:
        frames = extract_spartn_frames(user)
        return [rec for rec, _p, _e in frames]

    # Fallback: scan entire file as raw SPARTN stream
    frames = extract_spartn_frames(data)
    if frames:
        return [rec for rec, _p, _e in frames]

    sys.stderr.write(f"{path}: no SPARTN frames found (not UBX-RXM-PMP or raw)\n")
    return []


def read_satellite_mask(reader: BitReader, subtype: int) -> List[int]:
    """Read constellation-specific satellite mask and return list of PRN/IDs present (1-based)."""
    size_bits = reader.read_bits(2)
    # Map size selection to mask length for each constellation
    if subtype == 0:  # GPS SF011
        mapping = [32, 44, 56, 64]
    elif subtype == 1:  # GLONASS SF012
        mapping = [24, 36, 48, 63]
    elif subtype == 2:  # Galileo SF093
        mapping = [36, 45, 54, 64]
    elif subtype == 3:  # BeiDou SF094
        mapping = [37, 46, 55, 64]
    elif subtype == 4:  # QZSS SF095
        mapping = [10, 40, 48, 64]
    else:
        # Default safe fallback
        mapping = [32, 44, 56, 64]
    mask_len = mapping[size_bits]
    ids: List[int] = []
    for pos in range(mask_len):
        bit = reader.read_bits(1)
        if bit == 1:
            ids.append(pos + 1)
    return ids


def parse_hpac_payload(payload: bytes, subtype: int) -> dict:
    r = BitReader(payload)
    hpac: dict = {}
    # Header
    siou = r.read_bits(9)
    aiou = r.read_bits(4)
    _reserved = r.read_bits(1)
    area_count = r.read_bits(5)
    hpac.update({"siou": siou, "aiou": aiou, "area_count": area_count})
    areas: List[dict] = []
    for _ in range(area_count):
        area: dict = {}
        area_id = r.read_bits(8)
        ngrid_present = r.read_bits(7)
        tropo_indicator = r.read_bits(2)
        iono_indicator = r.read_bits(2)
        area.update({
            "area_id": area_id,
            "ngrid_present": ngrid_present,
            "tropo_indicator": tropo_indicator,
            "iono_indicator": iono_indicator,
        })

        # Troposphere
        if tropo_indicator in (1, 2):
            t_eq = r.read_bits(3)
            t_quality = r.read_bits(3)
            t_avg_hydro = r.read_bits(8)
            t_coeff_size = r.read_bits(1)
            t_coeffs = {}
            if t_coeff_size == 0:
                # small coefficients
                if t_eq >= 0:
                    t_coeffs["T00"] = r.read_bits(7)
                if t_eq >= 1:
                    t_coeffs["T01"] = r.read_bits(7)
                    t_coeffs["T10"] = r.read_bits(7)
                if t_eq >= 2:
                    t_coeffs["T11"] = r.read_bits(9)
            else:
                # large coefficients
                if t_eq >= 0:
                    t_coeffs["T00"] = r.read_bits(9)
                if t_eq >= 1:
                    t_coeffs["T01"] = r.read_bits(9)
                    t_coeffs["T10"] = r.read_bits(9)
                if t_eq >= 2:
                    t_coeffs["T11"] = r.read_bits(11)

            tropo: dict = {
                "eq_type": t_eq,
                "quality": t_quality,
                "avg_hydro": t_avg_hydro,
                "coeff_size": t_coeff_size,
                "coeffs": t_coeffs,
            }

            # Grid residuals
            if tropo_indicator == 2:
                resid_size = r.read_bits(1)  # SF051
                bits_per = 6 if resid_size == 0 else 8
                residuals = [r.read_bits(bits_per) for _ in range(ngrid_present)]
                tropo.update({
                    "residual_field_size": resid_size,
                    "residuals": residuals,
                })
            area["tropo"] = tropo

        # Ionosphere
        if iono_indicator in (1, 2):
            i_eq = r.read_bits(3)
            sat_ids = read_satellite_mask(r, subtype)
            sats: List[dict] = []
            for _s in sat_ids:
                srec: dict = {"sat_id": _s}
                i_quality = r.read_bits(4)
                i_coeff_size = r.read_bits(1)
                coeffs = {}
                if i_coeff_size == 0:
                    # small coefficients
                    if i_eq >= 0:
                        coeffs["C00"] = r.read_bits(12)
                    if i_eq >= 1:
                        coeffs["C01"] = r.read_bits(12)
                        coeffs["C10"] = r.read_bits(12)
                    if i_eq >= 2:
                        coeffs["C11"] = r.read_bits(13)
                else:
                    # large coefficients
                    if i_eq >= 0:
                        coeffs["C00"] = r.read_bits(14)
                    if i_eq >= 1:
                        coeffs["C01"] = r.read_bits(14)
                        coeffs["C10"] = r.read_bits(14)
                    if i_eq >= 2:
                        coeffs["C11"] = r.read_bits(15)
                srec.update({
                    "quality": i_quality,
                    "coeff_size": i_coeff_size,
                    "coeffs": coeffs,
                })

                # Per-satellite grid residuals if iono_indicator == 2
                if iono_indicator == 2:
                    resid_sel = r.read_bits(2)  # SF063
                    bits_map = {0: 4, 1: 7, 2: 10, 3: 14}
                    bits_per = bits_map.get(resid_sel, 4)
                    residuals = [r.read_bits(bits_per) for _ in range(ngrid_present)]
                    srec.update({
                        "residual_field_size": resid_sel,
                        "residuals": residuals,
                    })
                sats.append(srec)
            iono: dict = {"eq_type": i_eq, "sat_ids": sat_ids, "sat_data": sats}
            area["iono"] = iono

        areas.append(area)
    hpac["areas"] = areas
    return hpac


def read_var_size_mask(reader: BitReader, short_len: int, long_len: int) -> List[int]:
    """Read a bias mask with first bit selecting short/long. Return list of indices (0-based) set to 1."""
    sel = reader.read_bits(1)
    nbits = long_len if sel == 1 else short_len
    indices: List[int] = []
    for i in range(nbits):
        if reader.read_bits(1) == 1:
            indices.append(i)
    return indices


def decode_sf020_to_meters(val: int) -> float:
    return (val * 0.002) - 16.382


def decode_sf029_to_meters(val: int) -> float:
    return (val * 0.02) - 20.46


def parse_ocb_payload(payload: bytes, subtype: int) -> dict:
    r = BitReader(payload)
    ocb: dict = {}
    # Header fields
    siou = r.read_bits(9)
    eos = r.read_bits(1)
    _res = r.read_bits(1)
    yaw_present = r.read_bits(1)
    ref_datum = r.read_bits(1)
    if subtype == 0:
        eph_type = r.read_bits(2)
    elif subtype == 1:
        eph_type = r.read_bits(2)
    elif subtype == 2:
        eph_type = r.read_bits(3)
    elif subtype == 3:
        eph_type = r.read_bits(4)
    elif subtype == 4:
        eph_type = r.read_bits(3)
    else:
        eph_type = 0
    sats = read_satellite_mask(r, subtype)
    ocb.update({
        "siou": siou,
        "eos": eos,
        "yaw_present": yaw_present,
        "ref_datum": ref_datum,
        "eph_type": eph_type,
        "sat_ids": sats,
    })

    # Lenient satellite parsing: if payload is compact/segmented and does not carry full
    # per-satellite blocks, return header-only without raising.
    iode_bits = {0: 8, 1: 7, 2: 10, 3: 8, 4: 8}.get(subtype, 8)
    satrecs: List[dict] = []
    for _sid in sats:
        # Ensure at least 1+3+3 bits remain for dnu/present/continuity
        if r.remaining_bits() < 7:
            break
        try:
            dnu = r.read_bits(1)
            present_flags = r.read_bits(3)
            cont_ind = r.read_bits(3)
        except Exception:
            break
        srec: dict = {"dnu": dnu, "present_flags": present_flags, "continuity": cont_ind}
        if dnu == 0:
            # Orbit block
            if (present_flags & 0b001) != 0:
                if r.remaining_bits() < iode_bits + 14 + 14 + 14 + (6 if yaw_present == 1 else 0):
                    # Insufficient bits for full orbit; stop parsing further sats
                    satrecs.append(srec)
                    break
                iode = r.read_bits(iode_bits)
                rad = decode_sf020_to_meters(r.read_bits(14))
                along = decode_sf020_to_meters(r.read_bits(14))
                cross = decode_sf020_to_meters(r.read_bits(14))
                yaw = None
                if yaw_present == 1:
                    yaw = r.read_bits(6)
                srec["orbit"] = {"iode": iode, "radial_m": rad, "along_m": along, "cross_m": cross, "yaw_deg_step6": yaw}
            # Clock block
            if (present_flags & 0b010) != 0:
                if r.remaining_bits() < 3 + 14 + 3:
                    satrecs.append(srec)
                    break
                iode_cont = r.read_bits(3)
                clk = decode_sf020_to_meters(r.read_bits(14))
                ure = r.read_bits(3)
                srec["clock"] = {"iode_cont": iode_cont, "clock_m": clk, "ure_index": ure}
            # Bias block
            if (present_flags & 0b100) != 0:
                try:
                    if subtype == 0:
                        phase_mask = read_var_size_mask(r, 6, 11)
                        phase_corrs = []
                        for _ in phase_mask:
                            fix = r.read_bits(1); cont = r.read_bits(3); corr = decode_sf020_to_meters(r.read_bits(14))
                            phase_corrs.append({"fix": fix, "continuity": cont, "bias_m": corr})
                        code_mask = read_var_size_mask(r, 6, 11)
                        code_corrs = [decode_sf029_to_meters(r.read_bits(11)) for _ in code_mask]
                    elif subtype == 1:
                        phase_mask = read_var_size_mask(r, 5, 9)
                        phase_corrs = []
                        for _ in phase_mask:
                            fix = r.read_bits(1); cont = r.read_bits(3); corr = decode_sf020_to_meters(r.read_bits(14))
                            phase_corrs.append({"fix": fix, "continuity": cont, "bias_m": corr})
                        code_mask = read_var_size_mask(r, 5, 9)
                        code_corrs = [decode_sf029_to_meters(r.read_bits(11)) for _ in code_mask]
                    elif subtype == 2:
                        phase_mask = read_var_size_mask(r, 8, 15)
                        phase_corrs = []
                        for _ in phase_mask:
                            fix = r.read_bits(1); cont = r.read_bits(3); corr = decode_sf020_to_meters(r.read_bits(14))
                            phase_corrs.append({"fix": fix, "continuity": cont, "bias_m": corr})
                        code_mask = read_var_size_mask(r, 8, 15)
                        code_corrs = [decode_sf029_to_meters(r.read_bits(11)) for _ in code_mask]
                    elif subtype == 3:
                        phase_mask = read_var_size_mask(r, 8, 15)
                        phase_corrs = []
                        for _ in phase_mask:
                            fix = r.read_bits(1); cont = r.read_bits(3); corr = decode_sf020_to_meters(r.read_bits(14))
                            phase_corrs.append({"fix": fix, "continuity": cont, "bias_m": corr})
                        code_mask = read_var_size_mask(r, 8, 15)
                        code_corrs = [decode_sf029_to_meters(r.read_bits(11)) for _ in code_mask]
                    else:
                        phase_mask = read_var_size_mask(r, 6, 11)
                        phase_corrs = []
                        for _ in phase_mask:
                            fix = r.read_bits(1); cont = r.read_bits(3); corr = decode_sf020_to_meters(r.read_bits(14))
                            phase_corrs.append({"fix": fix, "continuity": cont, "bias_m": corr})
                        code_mask = read_var_size_mask(r, 6, 11)
                        code_corrs = [decode_sf029_to_meters(r.read_bits(11)) for _ in code_mask]
                    srec["bias"] = {"phase_mask_indices": phase_mask, "phase_biases": phase_corrs, "code_mask_indices": code_mask, "code_biases_m": code_corrs}
                except Exception:
                    # Not enough bits for bias block; stop parsing further sats
                    satrecs.append(srec)
                    break
        satrecs.append(srec)
    ocb["satellites"] = satrecs
    # If no satellites could be parsed, annotate header-only to help callers
    if not satrecs:
        ocb["header_only"] = True
    return ocb


def parse_bpac_payload(payload: bytes) -> dict:
    r = BitReader(payload)
    header = {
        "siou": r.read_bits(9),
        "reserved": r.read_bits(1),
        "iono_shell_height_sel": r.read_bits(2),
        "area_count": r.read_bits(2),
    }
    areas: List[dict] = []
    for _ in range(header["area_count"]):
        area_id = r.read_bits(2)
        ref_lat = r.read_bits(8)
        ref_lon = r.read_bits(9)
        lat_nodes = r.read_bits(4)
        lon_nodes = r.read_bits(4)
        lat_spacing = r.read_bits(2)
        lon_spacing = r.read_bits(2)
        avg_vtec = r.read_bits(12)
        num_grid = lat_nodes * lon_nodes
        present = []
        for i in range(num_grid):
            if r.read_bits(1) == 1:
                present.append(i)
        vtec_points: List[dict] = []
        for _idx in present:
            vq = r.read_bits(4)
            vsize = r.read_bits(1)
            if vsize == 0:
                vres = r.read_bits(7)
            else:
                vres = r.read_bits(11)
            vtec_points.append({"grid_index": _idx, "quality": vq, "size": vsize, "residual": vres})
        areas.append({
            "area_id": area_id,
            "ref_lat": ref_lat,
            "ref_lon": ref_lon,
            "lat_nodes": lat_nodes,
            "lon_nodes": lon_nodes,
            "lat_spacing_sel": lat_spacing,
            "lon_spacing_sel": lon_spacing,
            "avg_vtec": avg_vtec,
            "grid_present_indices": present,
            "vtec_points": vtec_points,
        })
    return {"header": header, "areas": areas}


def parse_gad_payload(payload: bytes) -> dict:
    r = BitReader(payload)
    header = {
        "siou": r.read_bits(9),
        "aiou": r.read_bits(4),
        "reserved": r.read_bits(1),
        "area_count": r.read_bits(5),
    }
    areas: List[dict] = []
    for _ in range(header["area_count"]):
        area_id = r.read_bits(8)
        # SF032: -90..+90, res 0.1
        enc_lat = r.read_bits(11)
        ref_lat = -90.0 + enc_lat * 0.1
        # SF033: -180..+180, res 0.1
        enc_lon = r.read_bits(12)
        ref_lon = -180.0 + enc_lon * 0.1
        lat_nodes = r.read_bits(3)
        lon_nodes = r.read_bits(3)
        # SF036/037: 0.1..3.2, step 0.1
        lat_spacing = 0.1 + r.read_bits(5) * 0.1
        lon_spacing = 0.1 + r.read_bits(5) * 0.1
        # Compute bounding box: ref_lat is northern-most; ref_lon is western-most
        north = ref_lat
        south = ref_lat - (lat_nodes - 1) * lat_spacing if lat_nodes > 0 else ref_lat
        west = ref_lon
        east = ref_lon + (lon_nodes - 1) * lon_spacing if lon_nodes > 0 else ref_lon
        areas.append({
            "area_id": area_id,
            "ref_lat": ref_lat,
            "ref_lon": ref_lon,
            "lat_nodes": lat_nodes,
            "lon_nodes": lon_nodes,
            "lat_spacing_deg": lat_spacing,
            "lon_spacing_deg": lon_spacing,
            "north_deg": north,
            "south_deg": south,
            "west_deg": west,
            "east_deg": east,
        })
    return {"header": header, "areas": areas}

def build_iv_ae_ctr(rec: dict) -> bytes:
    """Construct 16-byte AES-CTR IV per SPARTN ICD 8.15.1 (RFC-3686 style)."""
    # Pack fields into 96-bit nonce: TF002(7), TF003(10), TF007(4), TF009(32), TF010(7), TF011(4), TF012(4), TF013(6)
    fields = [
        (rec.get("type", 0), 7),
        (rec.get("payload_len", 0), 10),
        (rec.get("subtype", 0), 4),
        (rec.get("time_tag", 0), 32),
        (rec.get("solution_id", 0), 7),
        (rec.get("solution_proc_id", 0), 4),
        (rec.get("encryption_id", 0), 4),
        (rec.get("enc_seq_num", 0), 6),
    ]
    acc = 0
    total_bits = 0
    for value, width in fields:
        acc = (acc << width) | (int(value) & ((1 << width) - 1))
        total_bits += width
    if total_bits > 96:
        raise ValueError("IV packing exceeded 96 bits")
    acc <<= (96 - total_bits)
    nonce96 = acc.to_bytes(12, "big")
    counter = (1).to_bytes(4, "big")
    return nonce96 + counter


def aes_ctr_decrypt_openssl(key_hex: str, iv: bytes, ciphertext: bytes) -> bytes:
    """Use local OpenSSL to perform AES-128-CTR decryption."""
    iv_hex = iv.hex()
    try:
        p = subprocess.Popen([
            "openssl", "enc", "-aes-128-ctr", "-d", "-K", key_hex, "-iv", iv_hex, "-nopad"
        ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise RuntimeError("OpenSSL not found on PATH")
    out, err = p.communicate(ciphertext)
    if p.returncode != 0:
        raise RuntimeError(f"OpenSSL decrypt failed: {err.decode(errors='ignore')}")
    return out


def _reflect_byte(b: int) -> int:
    v = 0
    for i in range(8):
        if b & (1 << i):
            v |= (1 << (7 - i))
    return v


def crc4_ccitt(bytes24: bytes) -> int:
    # Polynomial 0x09, init 0x0, input reflected, output reflected, no xor
    reg = 0x0
    poly = 0x09
    for b in bytes24:
        rb = _reflect_byte(b)
        for i in range(8):
            bit = (rb >> (7 - i)) & 1
            top = (reg >> 3) & 1
            reg = ((reg << 1) & 0xF) | bit
            if top:
                reg ^= poly
    # reflect 4-bit output
    out = 0
    for i in range(4):
        if reg & (1 << i):
            out |= (1 << (3 - i))
    return out & 0xF


def crc_generic(data: bytes, poly: int, width: int, init: int = 0, xor_out: int = 0) -> int:
    reg = init
    topbit = 1 << (width - 1)
    mask = (1 << width) - 1
    for b in data:
        reg ^= (b << (width - 8)) & mask
        for _ in range(8):
            if reg & topbit:
                reg = ((reg << 1) ^ poly) & mask
            else:
                reg = (reg << 1) & mask
    return (reg ^ xor_out) & mask


def compute_message_crc(tf005: int, frame_without_crc: bytes) -> bytes:
    # tf005 selects CRC size
    if tf005 == 0:
        val = crc_generic(frame_without_crc, 0x07, 8, 0x00, 0x00)
        return val.to_bytes(1, 'big')
    elif tf005 == 1:
        val = crc_generic(frame_without_crc, 0x1021, 16, 0x0000, 0x0000)
        return val.to_bytes(2, 'big')
    elif tf005 == 2:
        val = crc_generic(frame_without_crc, 0x864CFB, 24, 0x000000, 0x000000)
        return val.to_bytes(3, 'big')
    else:
        val = crc_generic(frame_without_crc, 0x04C11DB7, 32, 0xFFFFFFFF, 0xFFFFFFFF)
        return val.to_bytes(4, 'big')


class BitWriter:
    def __init__(self):
        self.bits: List[int] = []

    def write_bits(self, value: int, nbits: int):
        for i in range(nbits - 1, -1, -1):
            self.bits.append((value >> i) & 1)

    def to_bytes(self) -> bytes:
        out = bytearray()
        for i in range(0, len(self.bits), 8):
            byte = 0
            for j in range(8):
                if i + j < len(self.bits):
                    byte = (byte << 1) | self.bits[i + j]
                else:
                    byte <<= 1
            out.append(byte)
        return bytes(out)


def build_plain_spartn_frame(rec: dict, payload_plain: bytes) -> bytes:
    # Build TF002..TF005
    hdr = BitWriter()
    hdr.write_bits(rec["type"], 7)
    hdr.write_bits(len(payload_plain), 10)
    hdr.write_bits(0, 1)  # EAF=0
    hdr.write_bits(rec["crc_type"], 2)
    header_bytes = hdr.to_bytes()
    # Compute TF006 CRC4 over TF002..TF005 plus 4 zero filler → 24 bits
    # header_bytes length should be 3 bytes because 7+10+1+2 = 20 bits -> 3 bytes
    if len(header_bytes) < 3:
        header_bytes = (header_bytes + b"\x00\x00\x00")[:3]
    hb = bytearray(header_bytes[:3])
    hb[2] &= 0xF0  # zero out low 4 bits (filler)
    crc4 = crc4_ccitt(bytes(hb))

    # Build PDB (TF007..TF011)
    pdb = BitWriter()
    pdb.write_bits(rec["subtype"], 4)
    pdb.write_bits(rec["time_tag_type"], 1)
    if rec["time_tag_type"] == 0:
        pdb.write_bits(rec["time_tag"], 16)
    else:
        pdb.write_bits(rec["time_tag"], 32)
    pdb.write_bits(rec["solution_id"], 7)
    pdb.write_bits(rec["solution_proc_id"], 4)
    pdb_bytes = pdb.to_bytes()

    # Assemble frame without message CRC: TF001 + TF002..TF006 + TF007.. + TF016
    frame_wo_crc = bytearray()
    frame_wo_crc.append(0x73)
    # TF002..TF005 (20 bits) and TF006 nibble
    frame_wo_crc.extend(bytes([header_bytes[0], header_bytes[1], (header_bytes[2] & 0xF0) | (crc4 & 0x0F)]))
    # PDB
    frame_wo_crc.extend(pdb_bytes)
    # Payload TF016 (byte aligned per transport)
    frame_wo_crc.extend(payload_plain)

    # Compute message CRC over TF002..TF017 (here TF017 absent)
    # Exclude preamble (TF001)
    tf_bytes = bytes(frame_wo_crc[1:])
    crc_bytes = compute_message_crc(rec["crc_type"], tf_bytes)
    frame = bytes(frame_wo_crc) + crc_bytes
    return frame


class TimeResolver:
    def __init__(self):
        self.last_full_t32: Optional[int] = None

    def note_full(self, t32: int):
        self.last_full_t32 = t32

    def resolve_halfday(self, t16: int) -> Optional[int]:
        # Need a reference full time tag to resolve
        if self.last_full_t32 is None:
            return None
        # last day base
        day = (self.last_full_t32 // 86400)
        candidates = []
        for d_offset in (-1, 0, 1):
            day_base = (day + d_offset) * 86400
            candidates.append(day_base + t16)
            candidates.append(day_base + 43200 + t16)
        # choose candidate closest to last_full_t32
        best = min(candidates, key=lambda c: abs(c - self.last_full_t32))
        return best


def _stitch_dir_to_line_log(dir_path: str, out_log_path: str, decrypt_key: Optional[str]) -> None:
    # Gather PMP files
    p = Path(dir_path)
    files = sorted([f for f in p.glob('PMP_*.bin') if f.is_file()])
    # Open output text log
    outp = open(out_log_path, 'w')
    now_iso = time.strftime('%Y-%m-%dT%H:%M:%S UTC', time.gmtime())
    outp.write('# SPARTN Message Log\n')
    outp.write(f'# Generated: {now_iso}\n')
    outp.write('# Mountpoint: PMP\n')
    outp.write('# GPS Week: 0\n')
    outp.write('# GPS TOW: 0ms\n')
    outp.write('# Format: [timestamp_ms] [type] [subtype] [length] [hex_data]\n')
    outp.write('# ----------------------------------------\n')
    outp.flush()

    # Build combined carry scanner over PMP userData
    carry = b''
    frame_ms = 0
    resolver = TimeResolver()

    # Determine base epoch from filenames
    def _parse_epoch_ms(name: str) -> Optional[int]:
        m = re.match(r'^PMP_(\d{8})_(\d{6})_(\d{3})\.bin$', name, re.IGNORECASE)
        if not m:
            return None
        ymd, hms, ms = m.group(1), m.group(2), int(m.group(3))
        try:
            dt = time.strptime(ymd + hms, '%Y%m%d%H%M%S')
            base = int(time.mktime(dt)) * 1000
            return base + ms
        except Exception:
            return None

    epochs = [e for e in (_parse_epoch_ms(f.name) for f in files) if e is not None]
    base_epoch_ms = min(epochs) if epochs else None

    for f in files:
        data = f.read_bytes()
        user = ubx_parse_rxm_pmp(data)
        stream = user if (user is not None and len(user) > 0) else data
        buf = carry + stream
        i = 0
        while i < len(buf):
            j = buf.find(b"\x73", i)
            if j < 0:
                break
            parsed = parse_spartn_frame(buf, j)
            if parsed is None:
                i = j + 1
                continue
            rec, next_i, payload_bytes, _embedded = parsed
            # CRC gate
            crc_n = CRC_TYPE_TO_BYTES.get(int(rec.get('crc_type', 0)), 0)
            if next_i <= j or crc_n <= 0 or next_i - crc_n <= j:
                i = j + 1
                continue
            body = buf[j+1: next_i - crc_n]
            exp = buf[next_i - crc_n: next_i]
            calc = compute_message_crc(int(rec.get('crc_type', 0)), body)
            if calc != exp:
                i = j + 1
                continue

            # Maintain resolver state
            if rec.get('time_tag_type', 0) == 1:
                try:
                    resolver.note_full(int(rec.get('time_tag', 0)))
                except Exception:
                    pass

            # Timestamp
            ts_ms = frame_ms if base_epoch_ms is None else 0
            if base_epoch_ms is not None:
                # Approximate relative timestamp with filename epoch if available
                fe = _parse_epoch_ms(f.name)
                if fe is not None:
                    ts_ms = int(fe - base_epoch_ms)
            else:
                frame_ms += 1

            # Optional decrypt and rebuild plaintext
            frame_out = buf[j:next_i]
            if rec.get('eaf', 0) == 1 and payload_bytes and decrypt_key:
                try:
                    rec_for_iv = dict(rec)
                    if rec_for_iv.get('time_tag_type', 0) == 0 and resolver.last_full_t32 is not None:
                        rec_for_iv['time_tag_type'] = 1
                        rec_for_iv['time_tag'] = resolver.last_full_t32
                    iv = build_iv_ae_ctr(rec_for_iv)
                    plain = aes_ctr_decrypt_openssl(decrypt_key, iv, payload_bytes)
                    rec_plain_hdr = dict(rec)
                    rec_plain_hdr['eaf'] = 0
                    frame_out = build_plain_spartn_frame(rec_plain_hdr, plain)
                except Exception:
                    frame_out = buf[j:next_i]

            outp.write(f"{ts_ms} {int(rec.get('type',0))} {int(rec.get('subtype',0))} {len(frame_out)} {frame_out.hex()}\n")
            outp.flush()
            i = next_i
        # Update carry window to last 2400 bytes
        carry = buf[-2400:] if len(buf) > 2400 else buf
    outp.close()


def main():
    parser = argparse.ArgumentParser(description="Decode SPARTN frames; optional decrypt and parse HPAC")
    parser.add_argument("paths", nargs="*", help="Files/dirs (default: 5minset, spartnLogs, spartn_*.log)")
    parser.add_argument("--decrypt-key", dest="decrypt_key", help="Hex AES-128 key for AES-CTR (32 hex)")
    parser.add_argument("--decrypt-one", action="store_true", help="Stop after decrypting the first eligible frame in each file")
    parser.add_argument("--only-hpac", action="store_true", help="Only output parsed HPAC (type 1) frames")
    parser.add_argument("--write-log", dest="write_log", help="Write decrypted/plain SPARTN frames to this binary log (like NTRIP)")
    # New: stitched directory scan to a single line-oriented log
    parser.add_argument("--stitch-dir", dest="stitch_dir", help="Directory of PMP_*.bin to scan with a sliding buffer")
    parser.add_argument("--line-log", dest="line_log", help="Write a single line log (text) with stitched frames")
    args = parser.parse_args()

    # Fast-path: stitched line log from a directory of PMP files
    if args.stitch_dir and args.line_log:
        _stitch_dir_to_line_log(args.stitch_dir, args.line_log, args.decrypt_key)
        return

    inputs: List[str] = []
    if args.paths:
        for arg in args.paths:
            if os.path.isdir(arg):
                inputs.extend(sorted(glob.glob(os.path.join(arg, "*.bin"))))
                inputs.extend(sorted(glob.glob(os.path.join(arg, "*.log"))))
            else:
                inputs.append(arg)
    else:
        inputs.extend(sorted(glob.glob("./5minset/PMP_*.bin")))
        inputs.extend(sorted(glob.glob("./spartnLogs/PMP_*.bin")))
        inputs.extend(sorted(glob.glob("./spartn_*.log")))

    if not inputs:
        sys.stderr.write("No input files found. Provide a directory or file paths.\n")
        sys.exit(1)

    # Maintain resolver and optional combined output across all inputs
    resolver = TimeResolver()
    out_log_global = None
    if args.write_log:
        out_log_global = open(args.write_log, 'ab')
    # Heuristic counters to detect invalid decrypt key for HPAC
    hpac_enc_attempts = 0  # encrypted HPAC frames with resolvable full time-tag that we attempted to decrypt
    hpac_enc_success = 0   # those that parsed successfully as HPAC

    for path in inputs:
        # Read raw
        try:
            with open(path, 'rb') as f:
                raw = f.read()
        except Exception as e:
            sys.stderr.write(f"Failed to read {path}: {e}\n")
            continue

        # Prefer UBX userData; else raw
        user = ubx_parse_rxm_pmp(raw)
        scan = user if (user is not None and len(user) > 0) else raw

        frames = extract_spartn_frames(scan)
        if not frames:
            sys.stderr.write(f"{path}: no SPARTN frames found\n")
            continue

        out_log = None
        if args.write_log:
            out_log = open(args.write_log, 'ab')

        for rec, payload_bytes, _embedded in frames:
            # Optionally filter to HPAC only
            if args.only_hpac and rec.get("type") != 1:
                continue

            out = {"file": os.path.basename(path), **rec}

            # Update/resolve time tags
            resolved_t32: Optional[int] = None
            if rec.get("time_tag_type", 1) == 1:
                resolver.note_full(rec.get("time_tag", 0))
            else:
                rt = resolver.resolve_halfday(rec.get("time_tag", 0))
                if rt is not None:
                    resolved_t32 = rt

            # Decide which payload to parse: decrypted or plain
            parsed_hpac = None
            payload_for_parse = None

            if rec.get("type") == 1:
                if rec.get("eaf", 0) == 1:
                    if not args.decrypt_key:
                        out["parse_error"] = "encrypted HPAC; provide --decrypt-key"
                    elif rec.get("time_tag_type", 1) == 0 and resolved_t32 is None:
                        out["parse_error"] = "HPAC has half-day time tag; cannot build IV"
                    else:
                        # Count an HPAC decrypt attempt when we have a key and a resolvable full time-tag (or already full)
                        if args.decrypt_key and (rec.get("time_tag_type", 1) == 1 or resolved_t32 is not None):
                            hpac_enc_attempts += 1
                        try:
                            rec_for_iv = dict(rec)
                            if resolved_t32 is not None:
                                rec_for_iv["time_tag_type"] = 1
                                rec_for_iv["time_tag"] = resolved_t32
                            iv = build_iv_ae_ctr(rec_for_iv)
                            payload_for_parse = aes_ctr_decrypt_openssl(args.decrypt_key, iv, payload_bytes)
                        except Exception as e:
                            out["parse_error"] = f"decrypt failed: {str(e)}"
                else:
                    payload_for_parse = payload_bytes

                if payload_for_parse is not None:
                    try:
                        parsed_hpac = parse_hpac_payload(payload_for_parse, rec.get("subtype", 0))
                        out["hpac"] = parsed_hpac
                        # Successful HPAC parse after decrypt attempt counts as a success
                        if rec.get("eaf", 0) == 1 and args.decrypt_key:
                            hpac_enc_success += 1
                    except Exception as e:
                        out["parse_error"] = f"hpac parse error: {str(e)}"

            # OCB (type 0) parsing
            if rec.get("type") == 0:
                payload_for_parse = None
                if rec.get("eaf", 0) == 1:
                    if (rec.get("time_tag_type", 1) == 1 or resolved_t32 is not None) and args.decrypt_key:
                        try:
                            rec_for_iv = dict(rec)
                            if resolved_t32 is not None:
                                rec_for_iv["time_tag_type"] = 1
                                rec_for_iv["time_tag"] = resolved_t32
                            iv = build_iv_ae_ctr(rec_for_iv)
                            payload_for_parse = aes_ctr_decrypt_openssl(args.decrypt_key, iv, payload_bytes)
                        except Exception as e:
                            out["parse_error"] = f"decrypt failed: {str(e)}"
                    else:
                        out["parse_error"] = out.get("parse_error") or "encrypted OCB; need --decrypt-key and full time-tag"
                else:
                    payload_for_parse = payload_bytes

                if payload_for_parse is not None:
                    try:
                        out["ocb"] = parse_ocb_payload(payload_for_parse, rec.get("subtype", 0))
                    except Exception as e:
                        out["parse_error"] = f"ocb parse error: {str(e)}"

            # Optionally emit decrypted/plain frames to log
            # Choose output handle: per-file or global
            out_handle = out_log if out_log is not None else out_log_global
            if out_handle is not None:
                payload_for_emit = None
                if rec.get("eaf", 0) == 1:
                    # Only emit if we can decrypt (need 32-bit time tag or resolved)
                    if args.decrypt_key and (rec.get("time_tag_type", 1) == 1 or resolved_t32 is not None):
                        try:
                            rec_for_iv = dict(rec)
                            if resolved_t32 is not None:
                                rec_for_iv["time_tag_type"] = 1
                                rec_for_iv["time_tag"] = resolved_t32
                            iv = build_iv_ae_ctr(rec_for_iv)
                            payload_for_emit = aes_ctr_decrypt_openssl(args.decrypt_key, iv, payload_bytes)
                        except Exception:
                            payload_for_emit = None
                else:
                    payload_for_emit = payload_bytes

                if payload_for_emit is not None:
                    try:
                        frame = build_plain_spartn_frame(rec, payload_for_emit)
                        out_handle.write(frame)
                    except Exception:
                        pass

            try:
                print(json.dumps(out))
            except BrokenPipeError:
                # Allow early pipe closures (e.g., when piping to head) without stack traces
                os._exit(0)

            if args.decrypt_key and args.decrypt_one and rec.get("eaf", 0) == 1:
                break

        if out_log is not None:
            out_log.close()

    if out_log_global is not None:
        out_log_global.close()

    # Final heuristic: if a key was provided and we attempted HPAC decrypts with full time-tag
    # but none parsed successfully, the key is likely invalid (or wrong week/region)
    if args.decrypt_key:
        threshold = 1 if args.decrypt_one else 3
        if hpac_enc_attempts >= threshold and hpac_enc_success == 0:
            sys.stderr.write(
                f"ERROR: Provided --decrypt-key may be invalid (HPAC decrypt attempts={hpac_enc_attempts}, successes={hpac_enc_success}).\n"
            )
            sys.stderr.write("Hint: ensure frames have a full 32-bit time tag, and the key matches the applicable week/region.\n")
            sys.exit(2)

if __name__ == "__main__":
    main()


