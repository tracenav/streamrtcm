"""
ephemeris_filter.py
--------------------

Purpose
    Filter an RTCM ephemeris log to only the satellites visible from a given
    receiver location at a requested UTC time.

What it does (pipeline)
    1) Calls parsemulti.py to decode the input RTCM log (1019/1020/1042/1046)
       into a structured ephemeris JSON file.
    2) Calls gps_ecef_from_json.py (ALL-GNSS mode) to compute satellite ECEF
       positions and broadcast clocks at the target UTC time.
    3) Calls skyplot_from_ecef.py to convert those ECEF coordinates into
       azimuth/elevation for the specified receiver (lat/lon/height).
    4) Applies a mask angle (elevation cutoff) to determine which PRNs are
       visible and writes a new RTCM log containing only ephemeris messages
       for those visible satellites.

CLI example (run from repo root)
    # Filter a log to satellites visible at 16:03 UTC from a given receiver
    python3 orbits/ephemeris_filter.py \
        orbits/data/rtcm_20250819_160327.log \
        --hhmm 1603 \
        --lat 38.36876 --lon -78.92694 --h 100 \
        --mask 0 \
        --out orbits/data/rtcm_20250819_160327_filtered.log \
        --debug

Inputs
    - input_log: path to RTCM ephemeris log to filter
    - time: UTC time as ISO string (e.g., 2025-08-15T18:00:00Z) or
            hhmm (e.g., 1800) combined with the log's header date
    - lat, lon, h: receiver geodetic coordinates (WGS-84)
    - mask: elevation mask angle in degrees

Outputs
    - A filtered .log containing only the ephemeris records for visible PRNs.

Notes
    - Intermediates are written to orbits/temp/ (portable, module-relative):
        - orbits/temp/eph/eph_<timestamp>.json (decoded ephemeris)
        - orbits/temp/ALL_GNSS_FOR_FILTER_<input>.json (ECEF + clocks)
        - orbits/temp/AZEL_FOR_FILTER_<input>.json (az/el per PRN)
    - Use --time ISO (e.g., 2025-08-15T18:00:00Z) or --hhmm HHMM to set the epoch.
    - Add --debug to print stage timings and whether caches were reused.
    - The script calls internal functions in-process for speed and will fall back
      to CLI tools if imports fail.
"""

import argparse
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from typing import Dict, List, Set, Tuple
import time


MODULE_DIR = os.path.abspath(os.path.dirname(__file__))


ALLOWED_TYPES = {1019, 1020, 1042, 1046}


def read_rtcm_header(input_path: str) -> Tuple[List[str], List[str]]:
    """Split an RTCM log into header lines and data lines.

    Header lines start with '#'. Data lines are message frames with
    the format: '[timestamp_ms] [msg_type] [sat_prn] [length] [hex_data]'.
    """
    header_lines: List[str] = []
    data_lines: List[str] = []
    with open(input_path, "r", encoding="utf-8") as f:
        for raw in f:
            if not raw.strip():
                continue
            if raw.lstrip().startswith("#"):
                header_lines.append(raw.rstrip("\n"))
            else:
                data_lines.append(raw.rstrip("\n"))
    return header_lines, data_lines


def parse_generated_date(header_lines: List[str]) -> Tuple[int, int, int]:
    """Extract (year, month, day) UTC from a '# Generated: ...' header.

    Example header
        '# Generated: 2025-08-15T17:33:36 UTC'
    Fallback to current UTC date if not found.
    """
    for line in header_lines:
        if line.startswith("# Generated:"):
            s = line.split("Generated:", 1)[1].strip()
            if s.endswith(" UTC"):
                s = s[:-4]
            try:
                dt = datetime.fromisoformat(s)
                return dt.year, dt.month, dt.day
            except Exception:
                pass
    now = datetime.now(timezone.utc)
    return now.year, now.month, now.day


def _guess_existing_eph_json(input_path: str) -> str | None:
    """Try to infer an existing eph_*.json path from the input log filename."""
    base = os.path.basename(input_path)
    m = re.search(r"(\d{8}_\d{6})", base)
    if not m:
        return None
    stamp = m.group(1)
    candidate = os.path.join(os.path.dirname(input_path), f"eph_{stamp}.json")
    if os.path.exists(candidate) and os.path.getsize(candidate) > 0:
        return candidate
    return None


def ensure_json_from_parsemulti(input_path: str, debug: bool = False) -> Tuple[str, bool]:
    """Run parsemulti.py on the input RTCM log; return ephemeris JSON path.

    Reuses an existing eph_*.json inferred from the input filename if present.
    """
    guess = _guess_existing_eph_json(input_path)
    if guess:
        if debug:
            print(f"[debug] reuse ephemeris JSON: {guess}")
        return os.path.abspath(guess), True
    cmd = [sys.executable, os.path.join(MODULE_DIR, "parsemulti.py"), input_path]
    if debug:
        print(f"[debug] run parsemulti: {' '.join(cmd)}")
    res = subprocess.run(cmd, check=True, capture_output=True, text=True)
    out_lines = [ln.strip() for ln in res.stdout.splitlines() if ln.strip()]
    if not out_lines:
        raise RuntimeError("parsemulti produced no output")
    json_path = out_lines[-1]
    if not os.path.isabs(json_path):
        json_path = os.path.abspath(json_path)
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Ephemeris JSON not found: {json_path}")
    if debug:
        print(f"[debug] produced ephemeris JSON: {json_path}")
    return json_path, False


def run_all_gnss_ecef(ephemeris_json: str, iso_time_utc: str, out_path: str, debug: bool = False) -> bool:
    """Run gps_ecef_from_json.py in ALL-GNSS mode for the given UTC epoch.

    Writes the resulting ECEF+clock structure to 'out_path'. Reuses an existing
    file if it already contains data and matches the requested time string.
    """
    # Reuse if present and non-trivial and includes the time string
    if os.path.exists(out_path) and os.path.getsize(out_path) > 256:
        try:
            with open(out_path, "r", encoding="utf-8") as rf:
                head = rf.read(4096)
                if iso_time_utc in head:
                    if debug:
                        print(f"[debug] reuse ECEF cache: {out_path}")
                    return True
        except Exception:
            pass
    cmd = [
        sys.executable,
        os.path.join(MODULE_DIR, "gps_ecef_from_json.py"),
        ephemeris_json,
        "ALL-GNSS",
        iso_time_utc,
        "UTC",
    ]
    if debug:
        print(f"[debug] run gps_ecef_from_json: {' '.join(cmd)} -> {out_path}")
    # Stream stdout directly to file to avoid buffering huge JSON in memory
    with open(out_path, "w", encoding="utf-8") as f:
        subprocess.run(cmd, check=True, stdout=f, stderr=subprocess.PIPE, text=True, timeout=180)
    return False


def run_skyplot_az_el(ecef_json: str, lat: float, lon: float, h_m: float, out_path: str, debug: bool = False) -> bool:
    """Run skyplot_from_ecef.py to produce az/el JSON for a receiver.

    Reuses an existing file if it already contains the same receiver params.
    """
    # Reuse if present and receiver matches
    if os.path.exists(out_path) and os.path.getsize(out_path) > 128:
        try:
            with open(out_path, "r", encoding="utf-8") as rf:
                j = json.load(rf)
            rcv = j.get("receiver", {})
            if (
                abs(float(rcv.get("lat_deg", 1e9)) - lat) < 1e-9
                and abs(float(rcv.get("lon_deg", 1e9)) - lon) < 1e-9
                and abs(float(rcv.get("h_m", 1e9)) - h_m) < 1e-6
            ):
                if debug:
                    print(f"[debug] reuse AZ/EL cache: {out_path}")
                return True
        except Exception:
            pass
    cmd = [
        sys.executable,
        os.path.join(MODULE_DIR, "skyplot_from_ecef.py"),
        ecef_json,
        "--lat",
        str(lat),
        "--lon",
        str(lon),
        "--h",
        str(h_m),
        "--out",
        out_path,
    ]
    if debug:
        print(f"[debug] run skyplot_from_ecef: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    return False


def prn_to_msg_type_and_id(prn: str) -> Tuple[int, int]:
    """Map a PRN string to (RTCM message type, numeric satellite id).

    Examples
        'G01' -> (1019, 1)
        'R12' -> (1020, 12)
        'C07' -> (1042, 7)
        'E02' -> (1046, 2)
    """
    if not prn or len(prn) < 2:
        raise ValueError(f"Invalid PRN: {prn}")
    const = prn[0].upper()
    num = int(prn[1:])
    if const == "G":
        return 1019, num
    if const == "R":
        return 1020, num
    if const == "C":
        return 1042, num
    if const == "E":
        return 1046, num
    raise ValueError(f"Unsupported PRN: {prn}")


def filter_visible_prns(az_el_json: str, mask_deg: float) -> Set[str]:
    """Return PRNs whose elevation is >= mask_deg from an az/el JSON file."""
    with open(az_el_json, "r", encoding="utf-8") as f:
        data = json.load(f)
    sats = data.get("satellites", [])
    visible: Set[str] = set()
    for s in sats:
        try:
            el = float(s.get("el_deg", -90.0))
            prn = str(s.get("prn", "")).strip()
        except Exception:
            continue
        if prn and el >= mask_deg:
            visible.add(prn)
    return visible


def filter_rtcm_log_by_prns(input_path: str, visible: Set[str], out_path: str, *,
                            iso_time: str = "", lat: float | None = None,
                            lon: float | None = None, h_m: float | None = None,
                            mask_deg: float | None = None) -> None:
    """Write a filtered RTCM log including only ephemeris for 'visible' PRNs.

    Adds header annotations documenting the filter epoch, receiver, and mask.
    """
    # Build mapping from PRN to (msg_type, sat_id)
    prn_map: Dict[Tuple[int, int], str] = {}
    for p in visible:
        t, sid = prn_to_msg_type_and_id(p)
        prn_map[(t, sid)] = p

    header_lines, data_lines = read_rtcm_header(input_path)

    with open(out_path, "w", encoding="utf-8") as out:
        # Write header, preserve original Generated line
        wrote_generated = False
        for hl in header_lines:
            if hl.startswith("# Generated:"):
                out.write(hl + "\n")
                wrote_generated = True
                # Append filter metadata immediately after Generated
                out.write("# Filtered: true\n")
                if iso_time:
                    out.write(f"# Filter UTC: {iso_time}\n")
                if (lat is not None) and (lon is not None) and (h_m is not None):
                    out.write(f"# Filter receiver: lat={lat}, lon={lon}, h_m={h_m}\n")
                if mask_deg is not None:
                    out.write(f"# Filter mask_deg: {mask_deg}\n")
            else:
                out.write(hl + "\n")
        if not wrote_generated:
            now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S UTC")
            out.write(f"# Generated: {now}\n")
            out.write("# Filtered: true\n")
            if iso_time:
                out.write(f"# Filter UTC: {iso_time}\n")
            if (lat is not None) and (lon is not None) and (h_m is not None):
                out.write(f"# Filter receiver: lat={lat}, lon={lon}, h_m={h_m}\n")
            if mask_deg is not None:
                out.write(f"# Filter mask_deg: {mask_deg}\n")

        # Copy only ephemeris lines for visible PRNs
        for ln in data_lines:
            parts = ln.strip().split()
            if len(parts) < 5:
                continue
            try:
                msg_type = int(parts[1])
                sat_prn = int(parts[2])
            except Exception:
                continue
            if msg_type not in ALLOWED_TYPES:
                continue
            if (msg_type, sat_prn) in prn_map:
                out.write(ln + "\n")


def main() -> None:
    """Entry-point: parse args, run decode/propagate/az-el, and filter log."""
    ap = argparse.ArgumentParser(description="Filter RTCM ephemeris log to only visible satellites at a given time.")
    ap.add_argument("input_log", help="Path to RTCM ephemeris log (e.g., orbits/rtcm_20250815.log)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--time", dest="iso_time", help="UTC time in ISO format, e.g., 2025-08-15T18:00:00Z")
    g.add_argument("--hhmm", dest="hhmm", help="UTC time as HHMM (e.g., 1800); date is taken from log header")
    ap.add_argument("--lat", type=float, default=0.0, help="Receiver latitude in degrees (default: 0.0)")
    ap.add_argument("--lon", type=float, default=0.0, help="Receiver longitude in degrees (default: 0.0)")
    ap.add_argument("--h", type=float, default=100.0, help="Receiver height in meters (default: 100)")
    ap.add_argument("--mask", type=float, default=0.0, help="Mask angle in degrees (default: 0)")
    ap.add_argument("--out", dest="out_log", help="Path to output filtered log; defaults to <input> with '_filtered.log'")
    ap.add_argument("--debug", action="store_true", help="Print timing and cache debug info")
    args = ap.parse_args()

    in_path = os.path.abspath(args.input_log)
    if not os.path.exists(in_path):
        raise FileNotFoundError(in_path)

    # Determine ISO UTC time
    if args.iso_time:
        iso_time = args.iso_time.strip()
        # Normalize trailing Z
        if iso_time.endswith("Z"):
            iso_time = iso_time
        elif iso_time.endswith(" UTC"):
            iso_time = iso_time[:-4] + "Z"
        else:
            # Assume ISO without timezone is UTC
            if re.match(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}(:\d{2})?$", iso_time):
                if len(iso_time) == 16:
                    iso_time += ":00Z"
                else:
                    iso_time += "Z"
    else:
        # Build from header + hhmm
        header, _ = read_rtcm_header(in_path)
        y, m, d = parse_generated_date(header)
        hhmm = args.hhmm or "1800"
        if not re.match(r"^\d{4}$", hhmm):
            raise ValueError("--hhmm must be 4 digits, e.g., 1800")
        hh = int(hhmm[:2]); mm = int(hhmm[2:])
        iso_time = f"{y:04d}-{m:02d}-{d:02d}T{hh:02d}:{mm:02d}:00Z"

    # Stage paths
    work_dir = os.path.dirname(in_path)
    temp_dir = os.path.join(MODULE_DIR, "temp")
    os.makedirs(temp_dir, exist_ok=True)
    t0 = time.monotonic()
    eph_json_guess = _guess_existing_eph_json(in_path)
    if eph_json_guess:
        eph_json, eph_reused = eph_json_guess, True
        if args.debug:
            print(f"[debug] reuse ephemeris JSON: {eph_json}")
    else:
        # Prefer in-process decode for portability
        try:
            from .parsemulti import parse_log_to_json  # type: ignore
            eph_outdir = os.path.join(temp_dir, "eph")
            os.makedirs(eph_outdir, exist_ok=True)
            # Name output deterministically from input timestamp
            m = re.search(r"(\\d{8}_\\d{6})", os.path.basename(in_path))
            if m:
                eph_json = os.path.join(eph_outdir, f"eph_{m.group(1)}.json")
            else:
                name_default = os.path.splitext(os.path.basename(in_path))[0]
                eph_json = os.path.join(eph_outdir, f"eph_{name_default}.json")
            if args.debug:
                print(f"[debug] parsemulti (in-proc) -> {eph_json}")
            parse_log_to_json(in_path, eph_json)
            eph_reused = False
        except Exception:
            # Fallback to subprocess with explicit outdir; print stderr on failure
            eph_outdir = os.path.join(temp_dir, "eph")
            os.makedirs(eph_outdir, exist_ok=True)
            cmd = [sys.executable, os.path.join(MODULE_DIR, "parsemulti.py"), in_path, "--outdir", eph_outdir]
            if args.debug:
                print(f"[debug] run parsemulti: {' '.join(cmd)}")
            try:
                res = subprocess.run(cmd, check=True, capture_output=True, text=True)
            except subprocess.CalledProcessError as cpe:
                sys.stderr.write("[error] parsemulti failed\n")
                if cpe.stderr:
                    sys.stderr.write(cpe.stderr + "\n")
                raise
            out_lines = [ln.strip() for ln in res.stdout.splitlines() if ln.strip()]
            if not out_lines:
                raise RuntimeError("parsemulti produced no output")
            eph_json = out_lines[-1]
            if not os.path.isabs(eph_json):
                eph_json = os.path.abspath(eph_json)
            eph_reused = False
    t1 = time.monotonic()
    # Place intermediates in portable temp folder under this module
    base = os.path.splitext(os.path.basename(in_path))[0]
    ecef_json = os.path.join(temp_dir, f"ALL_GNSS_FOR_FILTER_{base}.json")
    azel_json = os.path.join(temp_dir, f"AZEL_FOR_FILTER_{base}.json")
    # In-process path: avoid subprocess by importing helpers when available
    ecef_reused = False
    azel_reused = False
    t2s = time.monotonic()
    try:
        from . import gps_ecef_from_json as ecef_mod  # type: ignore
        with open(eph_json, "r", encoding="utf-8") as f:
            eph_obj = json.load(f)
        ecef_obj = ecef_mod.all_gnss_ecef_from_ephemeris(eph_obj, iso_time, "UTC")
        with open(ecef_json, "w", encoding="utf-8") as f:
            json.dump(ecef_obj, f, indent=2)
    except Exception:
        ecef_reused = run_all_gnss_ecef(eph_json, iso_time, ecef_json, debug=args.debug)
    t2e = time.monotonic()
    t3s = time.monotonic()
    try:
        from . import skyplot_from_ecef as sky_mod  # type: ignore
        entries = sky_mod.load_gps_ecef_entries(ecef_json)
        azel_obj = sky_mod.compute_az_el(entries, args.lat, args.lon, args.h)
        with open(azel_json, "w", encoding="utf-8") as f:
            json.dump(azel_obj, f, indent=2)
    except Exception:
        azel_reused = run_skyplot_az_el(ecef_json, args.lat, args.lon, args.h, azel_json, debug=args.debug)
    t3e = time.monotonic()
    t4s = time.monotonic()
    visible = filter_visible_prns(azel_json, args.mask)
    t4e = time.monotonic()

    # Determine output log path
    if args.out_log:
        out_log = os.path.abspath(args.out_log)
    else:
        base = os.path.basename(in_path)
        name, ext = os.path.splitext(base)
        out_log = os.path.join(work_dir, f"{name}_filtered.log")

    t5s = time.monotonic()
    filter_rtcm_log_by_prns(
        in_path,
        visible,
        out_log,
        iso_time=iso_time,
        lat=args.lat,
        lon=args.lon,
        h_m=args.h,
        mask_deg=args.mask,
    )
    t5e = time.monotonic()
    if args.debug:
        print(f"[debug] parsemulti: {'cache' if eph_reused else 'run'} {(t1 - t0):.3f}s")
        print(f"[debug] ecef: {'cache' if ecef_reused else 'run'} {(t2e - t2s):.3f}s")
        print(f"[debug] azel: {'cache' if azel_reused else 'run'} {(t3e - t3s):.3f}s")
        print(f"[debug] visible selection: {(t4e - t4s):.3f}s (N={len(visible)})")
        print(f"[debug] write output: {(t5e - t5s):.3f}s -> {out_log}")
        print(f"[debug] total: {(t5e - t0):.3f}s")
    print(out_log)


if __name__ == "__main__":
    main()


