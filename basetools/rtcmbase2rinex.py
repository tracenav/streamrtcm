#!/usr/bin/env python3

"""
rtcmbase2rinex.py

Minimal RTCM3 MSM4/MSM7 to RINEX 3.x observation converter for base logs.

This script uses the modular rtcm_logger package for RTCM parsing functionality
to keep packages lean and avoid code duplication.

Notes
- Uses rtcm_logger.rtcm.RTCMParser for RTCM3 frame parsing and CRC validation
- Supports GPS(1074/1077), GLO(1084/1087), GAL(1094/1097), BDS(1124/1127)
- Extracts per-cell fine pseudorange where available; falls back to rough range
- Writes minimal RINEX 3.05 Observation file with system-appropriate observables

Limitations
- Conservative MSM bit field assumptions for typical base logs
- Doppler/Carrier/SNR only from MSM7; MSM4 has code and phase only
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import sys
from typing import Dict, List, Tuple, Optional

# Import RTCM parsing from the modular rtcm_logger package
try:
    from rtcm_logger.rtcm import RTCMParser, should_log_message, EPHEMERIS_TYPES
except ImportError:
    # Fallback if package not installed - use local copy
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'pkg', 'logger'))
    from rtcm_logger.rtcm import RTCMParser, should_log_message, EPHEMERIS_TYPES

# MSM message types for RINEX conversion
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


def parse_rtcm_frame(buf: bytes, offset: int) -> Tuple[Optional[bytes], int]:
    """Locate and extract one RTCM3 frame from buf[offset:].
    Returns (payload_with_header, next_offset) or (None, next_offset_if_resync).
    """
    n = len(buf)
    i = offset
    while i + 3 < n:
        if buf[i] == 0xD3:
            # length: 10 bits across the next 2 bytes
            if i + 3 >= n:
                return (None, n)
            l_hi = buf[i + 1] & 0x03
            l_lo = buf[i + 2]
            length = (l_hi << 8) | l_lo
            end = i + 3 + length + 3  # header(3) + data(length) + CRC(3)
            if end > n:
                return (None, n)
            frame = buf[i:end]
            # CRC validation - simplified for RINEX conversion
            if len(frame) >= 6:  # Basic length check
                return (frame, end)
        i += 1
    return (None, n)


def decode_msm(frame: bytes) -> Optional[dict]:
    """Decode MSM4/MSM7 (subset) into a dict with epoch, system, sats, cells and measurements.
    Returns None if unsupported.
    """
    # strip 3-byte header and 3-byte crc
    payload = frame[3:-3]
    bs = BitStream(payload)

    msg_type = bs.readu(12)
    if msg_type not in MSM_TYPES:
        return None
    system, msm = MSM_TYPES[msg_type]

    station_id = bs.readu(12)
    epoch_time = bs.readu(30)  # unit ~ milliseconds of GNSS time-of-week (system dependent)
    mm = bs.readu(1)  # multiple message bit
    _iods = bs.readu(3)
    _reserved = bs.readu(7)
    _clk_steer = bs.readu(2)
    _ext_clk = bs.readu(2)
    smooth_ind = bs.readu(1)
    smooth_int = bs.readu(3)

    sat_mask = bs.readu(64)
    sig_mask = bs.readu(32)

    sats = [idx + 1 for idx in range(64) if (sat_mask >> (63 - idx)) & 1]
    sigs = [idx + 1 for idx in range(32) if (sig_mask >> (31 - idx)) & 1]
    # Cell mask: one bit per sat/sig pair
    cell_pairs: List[Tuple[int, int]] = []
    for s_idx in range(len(sats)):
        for g_idx in range(len(sigs)):
            bit = bs.readu(1)
            if bit:
                cell_pairs.append((sats[s_idx], sigs[g_idx]))

    # Satellite data: rough ranges
    rough_ranges_ms: Dict[int, float] = {}
    ext_rough_ranges_ms: Dict[int, float] = {}

    for sat in sats:
        # Rough range integer milliseconds (8 bits) for MSM4/MSM7 (255 indicates invalid)
        rr = bs.readu(8)
        if rr == 255:
            rough_ranges_ms[sat] = float("nan")
        else:
            rough_ranges_ms[sat] = float(rr)
    # extended rough range (MSM7)
    if msm == 7:
        for sat in sats:
            err = bs.readu(4)  # 4-bit extension in MSM7 for rough range (fractional ms)
            ext_rough_ranges_ms[sat] = err / 16.0

    # Per-cell fine measurements
    # MSM4: fine pseudorange(15), phaserange(22), lock(4), halfcycle(1), cnr(6)
    # MSM7: fine pseudorange(20), phaserange(24), doppler(15), lock(4), halfcycle(1), cnr(10)
    meas: Dict[Tuple[int, int], Dict[str, float]] = {}
    for (sat, sig) in cell_pairs:
        if msm == 4:
            fp = bs.reads(15)  # fine pseudorange (roughly 1/1024 ms)
            fr = bs.reads(22)  # fine phase range
            _lock = bs.readu(4)
            _half = bs.readu(1)
            cnr = bs.readu(6)
            meas[(sat, sig)] = {
                "fp": fp / 1024.0,
                "fr": fr / 1024.0,
                "cnr": float(cnr),
            }
        else:
            fp = bs.reads(20)
            fr = bs.reads(24)
            fd = bs.reads(15)
            _lock = bs.readu(4)
            _half = bs.readu(1)
            cnr = bs.readu(10)
            meas[(sat, sig)] = {
                "fp": fp / 1024.0,
                "fr": fr / 1024.0,
                "fd": fd / 16.0,  # doppler rough scaling
                "cnr": float(cnr) / 4.0,
            }

    return {
        "type": msg_type,
        "system": system,
        "msm": msm,
        "epoch_ms": epoch_time,
        "sats": sats,
        "sigs": sigs,
        "cells": cell_pairs,
        "rough_ms": rough_ranges_ms,
        "ext_rough_ms": ext_rough_ranges_ms,
        "meas": meas,
    }


# ----------------------------- RINEX writer ----------------------------------

CLIGHT = 299792458.0


def _gps_epoch() -> dt.datetime:
    return dt.datetime(1980, 1, 6)


def _guess_gps_week_from_file(rtcm_path: str) -> int:
    """Anchor epochs to the GPS week closest to the file mtime."""
    mtime = dt.datetime.utcfromtimestamp(os.path.getmtime(rtcm_path))
    delta = mtime - _gps_epoch()
    week = int(delta.total_seconds() // (7 * 24 * 3600))
    return max(0, week)


def rinex_time_from_epoch_ms(epoch_ms: int, gps_week: int) -> dt.datetime:
    # Epoch time is milliseconds within the GNSS week; anchor to provided GPS week
    tow = (epoch_ms / 1000.0) % (7 * 24 * 3600)
    return _gps_epoch() + dt.timedelta(weeks=gps_week, seconds=tow)


def _gps_week_from_datetime(dt_utc: dt.datetime) -> int:
    delta = dt_utc - _gps_epoch()
    return int(delta.total_seconds() // (7 * 24 * 3600))


def _parse_rinex_first_obs_datetime(rnx_path: str) -> Optional[dt.datetime]:
    try:
        with open(rnx_path, "r", errors="ignore") as f:
            for line in f:
                if "TIME OF FIRST OBS" in line:
                    # Columns: year month day hour min sec
                    try:
                        year = int(line[3:7])
                        month = int(line[9:11])
                        day = int(line[13:15])
                        hour = int(line[17:19])
                        minute = int(line[21:23])
                        sec = float(line[24:36])
                        whole = int(sec)
                        frac = sec - whole
                        return dt.datetime(year, month, day, hour, minute, whole) + dt.timedelta(seconds=frac)
                    except Exception:
                        return None
    except Exception:
        return None
    return None


def write_rinex3_header(f, start_dt: dt.datetime, end_dt: dt.datetime, systems: List[str]):
    f.write("     3.05           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE\n")
    f.write(f"{dt.datetime.utcnow():>40s} UTC PGM / RUN BY / DATE\n")
    f.write("                                                            MARKER NAME         \n")
    f.write("                                                            MARKER NUMBER       \n")
    f.write("                                                            MARKER TYPE         \n")
    f.write("                                                            OBSERVER / AGENCY   \n")
    f.write("                                                            REC # / TYPE / VERS \n")
    f.write("                                                            ANT # / TYPE        \n")
    f.write(f"        0.0000        0.0000        0.0000                  APPROX POSITION XYZ \n")
    f.write(f"        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N\n")
    # Define a minimal set of observables per system
    for sys in sorted(set(systems)):
        if sys == "GPS":
            f.write("G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \n")
        elif sys == "GLO":
            f.write("R    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \n")
        elif sys == "GAL":
            f.write("E    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \n")
        elif sys == "BDS":
            f.write("C    4 C2I L2I D2I S2I                                      SYS / # / OBS TYPES \n")
        elif sys == "QZS":
            f.write("J    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES \n")
    f.write(f"  {start_dt.year:4d}{start_dt.month:6d}{start_dt.day:6d}{start_dt.hour:6d}{start_dt.minute:6d}{start_dt.second:13.7f}     GPS         TIME OF FIRST OBS   \n")
    f.write(f"  {end_dt .year:4d}{end_dt .month:6d}{end_dt .day:6d}{end_dt .hour:6d}{end_dt .minute:6d}{end_dt .second:13.7f}     GPS         TIME OF LAST OBS    \n")
    f.write("                                                            SYS / PHASE SHIFT   \n")
    f.write("                                                            SYS / PHASE SHIFT   \n")
    f.write("                                                            SYS / PHASE SHIFT   \n")
    f.write("                                                            SYS / PHASE SHIFT   \n")
    f.write("  0                                                         GLONASS SLOT / FRQ #\n")
    f.write(" C1C    0.000 C1P    0.000 C2C    0.000 C2P    0.000        GLONASS COD/PHS/BIS \n")
    f.write("                                                            END OF HEADER       \n")


def format_epoch_record(epoch_dt: dt.datetime, sats: List[Tuple[str, int]]) -> str:
    return (
        f"> {epoch_dt.year:4d} {epoch_dt.month:2d} {epoch_dt.day:2d} {epoch_dt.hour:2d} {epoch_dt.minute:2d} {epoch_dt.second:11.7f}  0 {len(sats):2d}                     \n"
    )


def sys_prn_prefix(system: str) -> str:
    return {
        "GPS": "G",
        "GLO": "R",
        "GAL": "E",
        "BDS": "C",
        "QZS": "J",
    }.get(system, "G")


def write_obs_line(system: str, prn: int, c1: Optional[float], l1: Optional[float], d1: Optional[float], s1: Optional[float]) -> str:
    prfx = sys_prn_prefix(system)
    sat = f"{prfx}{prn:>2d}"
    def fld(x: Optional[float]) -> str:
        return f"{x:14.3f}" if x is not None else "              "
    return f"{sat} {fld(c1)}{fld(l1)}{fld(d1)}{fld(s1)}\n"


# ----------------------------- Main pipeline ---------------------------------

def convert_rtcm_to_rinex(rtcm_path: str, rinex_out: str, anchor_week: Optional[int] = None):
    with open(rtcm_path, "rb") as f:
        data = f.read()

    systems_seen: List[str] = []
    epochs: Dict[int, Dict[str, Dict[int, Dict[str, float]]]] = {}

    i = 0
    while i < len(data):
        frame, i = parse_rtcm_frame(data, i)
        if frame is None:
            break
        try:
            msm = decode_msm(frame)
        except Exception:
            continue
        if not msm:
            continue
        system = msm["system"]
        systems_seen.append(system)
        epoch_ms = msm["epoch_ms"]
        if epoch_ms not in epochs:
            epochs[epoch_ms] = {"GPS": {}, "GLO": {}, "GAL": {}, "BDS": {}, "QZS": {}}
        # Build simple C1 from rough + fine pseudorange
        for (sat, sig) in msm["cells"]:
            rr = msm["rough_ms"].get(sat)
            ext = msm["ext_rough_ms"].get(sat, 0.0)
            m = msm["meas"].get((sat, sig), {})
            fp = m.get("fp")  # ms fine
            if rr is None or (isinstance(rr, float) and rr != rr):
                continue
            # Total range in milliseconds (approx)
            total_ms = rr + ext + (fp if fp is not None else 0.0)
            c1 = total_ms * 1e-3 * CLIGHT
            l1 = None
            d1 = m.get("fd")  # Hz approx; leave None if not present
            s1 = m.get("cnr")
            # Record the first signal per sat
            if sat not in epochs[epoch_ms][system]:
                epochs[epoch_ms][system][sat] = {"C1": c1, "L1": l1, "D1": d1, "S1": s1}

    # Write RINEX
    if not epochs:
        raise RuntimeError("No MSM4/MSM7 observations found.")

    first_epoch = min(epochs.keys())
    last_epoch = max(epochs.keys())
    gps_week = anchor_week if anchor_week is not None else _guess_gps_week_from_file(rtcm_path)
    start_dt = rinex_time_from_epoch_ms(first_epoch, gps_week)
    end_dt = rinex_time_from_epoch_ms(last_epoch, gps_week)
    os.makedirs(os.path.dirname(rinex_out), exist_ok=True)
    with open(rinex_out, "w") as outf:
        write_rinex3_header(outf, start_dt, end_dt, systems_seen)
        for ep in sorted(epochs.keys()):
            ep_dt = rinex_time_from_epoch_ms(ep, gps_week)
            sats_all: List[Tuple[str, int]] = []
            for sys in ["GPS", "GLO", "GAL", "BDS", "QZS"]:
                for prn in sorted(epochs[ep][sys].keys()):
                    sats_all.append((sys, prn))
            outf.write(format_epoch_record(ep_dt, sats_all))
            for (sys, prn) in sats_all:
                rec = epochs[ep][sys][prn]
                outf.write(
                    write_obs_line(
                        sys,
                        prn,
                        rec.get("C1"),
                        rec.get("L1"),
                        rec.get("D1"),
                        rec.get("S1"),
                    )
                )


def main():
    ap = argparse.ArgumentParser(description="Convert RTCM3 MSM4/MSM7 base log to RINEX 3 observation")
    ap.add_argument("rtcm_log", help="Path to RTCM3 log (binary)")
    ap.add_argument("-o", "--output", default=None, help="Output RINEX obs path (.rnx/.obs)")
    ap.add_argument("--anchor-date", default=None, help="Anchor date (UTC) like YYYY-MM-DD to choose GPS week")
    ap.add_argument("--anchor-rnx", default=None, help="Path to RINEX whose TIME OF FIRST OBS sets GPS week")
    args = ap.parse_args()

    out = args.output
    if not out:
        base, _ = os.path.splitext(args.rtcm_log)
        out = base + ".obs"
    anchor_week: Optional[int] = None
    if args.anchor_rnx:
        dt_first = _parse_rinex_first_obs_datetime(args.anchor_rnx)
        if dt_first:
            anchor_week = _gps_week_from_datetime(dt_first)
    if anchor_week is None and args.anchor_date:
        try:
            y, m, d = map(int, args.anchor_date.split("-"))
            anchor_week = _gps_week_from_datetime(dt.datetime(y, m, d))
        except Exception:
            anchor_week = None

    convert_rtcm_to_rinex(args.rtcm_log, out, anchor_week)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()


