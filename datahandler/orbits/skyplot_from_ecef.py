import json
import math
import os
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple


# WGS84 ellipsoid
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass
class Receiver:
    lat_deg: float
    lon_deg: float
    h_m: float = 100.0

    @property
    def lat_rad(self) -> float:
        return math.radians(self.lat_deg)

    @property
    def lon_rad(self) -> float:
        return math.radians(self.lon_deg)

    def ecef_m(self) -> Tuple[float, float, float]:
        phi = self.lat_rad
        lam = self.lon_rad
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
        x = (N + self.h_m) * cos_phi * math.cos(lam)
        y = (N + self.h_m) * cos_phi * math.sin(lam)
        z = ((1.0 - WGS84_E2) * N + self.h_m) * sin_phi
        return x, y, z


def enu_rotation_matrix(lat_rad: float, lon_rad: float) -> List[List[float]]:
    sphi = math.sin(lat_rad)
    cphi = math.cos(lat_rad)
    slam = math.sin(lon_rad)
    clam = math.cos(lon_rad)
    # Rows correspond to E, N, U axes
    return [
        [-slam,            clam,            0.0],
        [-sphi * clam,    -sphi * slam,     cphi],
        [ cphi * clam,     cphi * slam,     sphi],
    ]


def dot3(a: List[float], b: Tuple[float, float, float]) -> float:
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def load_gps_ecef_entries(json_path: str) -> List[Dict[str, Any]]:
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    results = data.get("results", [])
    # Support ALL-GNSS format where results is a dict of lists
    if isinstance(results, dict):
        merged: List[Dict[str, Any]] = []
        for key in ("GPS", "GAL", "BDS", "GLO"):
            lst = results.get(key, [])
            if isinstance(lst, list):
                merged.extend(lst)
        results = merged
    # Each entry expected: {"prn": "Gxx/Eyy/Czz/Rrr", "ecef_km": {x,y,z}, ...}
    return results


def compute_az_el_for_all(entries: List[Dict[str, Any]], rx: Receiver) -> Dict[str, Any]:
    x_r, y_r, z_r = rx.ecef_m()
    R = enu_rotation_matrix(rx.lat_rad, rx.lon_rad)

    out: List[Dict[str, Any]] = []
    for entry in entries:
        prn = entry.get("prn", "")
        ecef_km = entry.get("ecef_km", {})
        try:
            xs = float(ecef_km["x"]) * 1000.0
            ys = float(ecef_km["y"]) * 1000.0
            zs = float(ecef_km["z"]) * 1000.0
        except Exception:
            continue

        d = (xs - x_r, ys - y_r, zs - z_r)
        e = dot3(R[0], d)
        n = dot3(R[1], d)
        u = dot3(R[2], d)
        rho = math.sqrt(e*e + n*n + u*u)
        # Handle numerical issues
        el_rad = math.asin(max(-1.0, min(1.0, u / rho)))
        az_rad = math.atan2(e, n)
        az_deg = math.degrees(az_rad)
        if az_deg < 0.0:
            az_deg += 360.0
        el_deg = math.degrees(el_rad)
        out.append({
            "prn": prn,
            "az_deg": az_deg,
            "el_deg": el_deg,
            "range_km": rho / 1000.0,
        })
    return {"receiver": {"lat_deg": rx.lat_deg, "lon_deg": rx.lon_deg, "h_m": rx.h_m}, "satellites": out}


def compute_az_el(entries: List[Dict[str, Any]], lat_deg: float, lon_deg: float, h_m: float) -> Dict[str, Any]:
	"""Importable helper returning the same structure as compute_az_el_for_all."""
	rx = Receiver(lat_deg=lat_deg, lon_deg=lon_deg, h_m=h_m)
	return compute_az_el_for_all(entries, rx)


def main(argv: List[str]) -> None:
    if len(argv) < 2:
        print("Usage: python skyplot_from_ecef.py <gps_all_ecef.json> [--lat <deg>] [--lon <deg>] [--h <m>] [--out <file.json>]")
        print("Example: python skyplot_from_ecef.py orbits/GPS_ALL_20250815T1800_gps.json --lat 37.7749 --lon -122.4194 --h 100 --out orbits/az_el.json")
        sys.exit(1)

    json_path = argv[1]
    lat = 0.0
    lon = 0.0
    h = 100.0
    out_path = None
    i = 2
    while i < len(argv):
        if argv[i] == "--lat" and i+1 < len(argv):
            lat = float(argv[i+1]); i += 2
        elif argv[i] == "--lon" and i+1 < len(argv):
            lon = float(argv[i+1]); i += 2
        elif argv[i] == "--h" and i+1 < len(argv):
            h = float(argv[i+1]); i += 2
        elif argv[i] == "--out" and i+1 < len(argv):
            out_path = argv[i+1]; i += 2
        else:
            i += 1

    rx = Receiver(lat_deg=lat, lon_deg=lon, h_m=h)
    entries = load_gps_ecef_entries(json_path)
    result = compute_az_el_for_all(entries, rx)

    text = json.dumps(result, indent=2)
    if out_path:
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(text)
        print(out_path)
    else:
        print(text)


if __name__ == "__main__":
    main(sys.argv)


