#!/usr/bin/env python3
"""
GAD + HPAC map with OCB satellite Orbit/Clock table overlay.

Inputs:
  - SPARTN binary (decrypted combined stream recommended)
Outputs:
  - map.geojson (FeatureCollection for GAD polygons and HPAC points)
  - map.html (interactive Folium map with OCB table overlay)

Legend:
Usage:
  python3 make_gad_hpac_ocb_map.py /path/to/spartn.bin
"""
# Field/Unit reference (per SPARTN ICD 2.0.2)
# - eph_type: Ephemeris type used for the OCB (constellation-specific). Indicates which
#   broadcast ephemeris the OCB was generated against.
# - IODE: Issue Of Data Ephemeris (or constellation equivalent). Identifies the broadcast
#   ephemeris to combine with the OCB for that satellite.
# - Orbit radial/along/cross (m): Satellite-orbit corrections in meters in the satellite
#   RTN frame (radial, along-track, cross-track). Apply to broadcast position.
# - Clock (ns): Satellite clock correction converted to nanoseconds (meters / c * 1e9),
#   where c = 299,792,458 m/s. Positive increases observed pseudorange; negative reduces it.
# - URE idx: User Range Error index for the combined OCB solution quality.
#   Mapping (approx 1σ at zenith): 0=unknown, 1≤0.01m, 2≤0.02m, 3≤0.05m, 4≤0.10m,
#   5≤0.30m, 6≤1.00m, 7=>1.00m.
# - Phase bias (m by track): Carrier-phase bias per selected tracking channel index for the
#   constellation (via SF025/SF026/SF102/SF103/SF104 masks). Units: meters; scale per SF020
#   (±16.382 m range, 0.002 m/LSB). An asterisk (*) indicates fixed ambiguity per SF023.
# - Code bias (m by track): Pseudorange code bias per selected tracking channel index (via
#   SF027/SF028/SF105/SF106/SF107 masks). Units: meters; scale per SF029 (±20.46 m range,
#   0.02 m/LSB).
# - TECU: Total Electron Content Unit. 1 TECU = 1e16 electrons/m^2 (column density).
# - Iono: HPAC ionosphere model value (in TECU). On the map we show a per-grid-node value
#   derived from per-satellite polynomials and/or residual grids, averaged when multiple
#   satellites contribute.
# - GAD: Geographic Area Definition. Defines rectangular grids (node counts and spacing)
#   and their geographic bounds (north/south/west/east) over which HPAC values are valid.

from __future__ import annotations

import json
import sys
from typing import Dict, List, Tuple, Optional
import os
from pathlib import Path
import sys

# ensure repo root on sys.path for imports
try:
    sys.path.append(str(Path(__file__).resolve().parents[1]))
except Exception:
    pass

from decode_spartn_from_pmp import (
    extract_spartn_frames,
    parse_gad_payload,
    parse_hpac_payload,
    parse_ocb_payload,
)


def collect_gad(frames, allowed_area_ids: set | None = None) -> Dict[int, dict]:
    index: Dict[int, dict] = {}
    for rec, payload, _ in frames:
        if rec.get("type") == 2 and rec.get("subtype") == 0:
            try:
                gad = parse_gad_payload(payload)
            except Exception:
                continue
            for a in gad.get("areas", []):
                if allowed_area_ids and a["area_id"] not in allowed_area_ids:
                    continue
                index[a["area_id"]] = a
    return index


def features_from_gad(gad_index: Dict[int, dict]) -> List[dict]:
    feats: List[dict] = []
    for a in gad_index.values():
        north = a["north_deg"]
        south = a["south_deg"]
        west = a["west_deg"]
        east = a["east_deg"]
        coords = [[west, south], [west, north], [east, north], [east, south], [west, south]]
        feats.append({
            "type": "Feature",
            "properties": {
                "layer": "gad",
                "area_id": a["area_id"],
                "north": north, "south": south, "west": west, "east": east,
                "lat_nodes": a["lat_nodes"], "lon_nodes": a["lon_nodes"],
                "lat_spacing_deg": a["lat_spacing_deg"], "lon_spacing_deg": a["lon_spacing_deg"],
            },
            "geometry": {"type": "Polygon", "coordinates": [coords]},
        })
    return feats


def _decode_tropo_coeffs(tropo: dict) -> Tuple[float, float, float, float, int]:
    eq = tropo.get("eq_type", 0)
    size = tropo.get("coeff_size", 0)
    c = tropo.get("coeffs", {})
    T00e = c.get("T00"); T01e = c.get("T01"); T10e = c.get("T10"); T11e = c.get("T11", 0)
    if size == 0:
        T00 = (T00e or 0) * 0.004 - 0.252
        T01 = (T01e or 0) * 0.001 - 0.063
        T10 = (T10e or 0) * 0.001 - 0.063
        T11 = (T11e or 0) * 0.0002 - 0.0510
    else:
        T00 = (T00e or 0) * 0.004 - 1.020
        T01 = (T01e or 0) * 0.001 - 0.255
        T10 = (T10e or 0) * 0.001 - 0.255
        T11 = (T11e or 0) * 0.0002 - 0.2046
    return T00, T01, T10, T11, eq


def _decode_iono_coeffs(coeffs: dict, coeff_size: int) -> Tuple[float, float, float, float]:
    """Decode ionosphere polynomial coefficients to TECU units per ICD."""
    if coeff_size == 0:
        # small
        C00 = (coeffs.get("C00", 0) * 0.04) - 81.88
        C01 = (coeffs.get("C01", 0) * 0.008) - 16.376
        C10 = (coeffs.get("C10", 0) * 0.008) - 16.376
        C11 = (coeffs.get("C11", 0) * 0.002) - 8.190
    else:
        # large
        C00 = (coeffs.get("C00", 0) * 0.04) - 327.64
        C01 = (coeffs.get("C01", 0) * 0.008) - 65.528
        C10 = (coeffs.get("C10", 0) * 0.008) - 65.528
        C11 = (coeffs.get("C11", 0) * 0.002) - 32.766
    return float(C00), float(C01), float(C10), float(C11)


def _decode_iono_residual(enc: int, sel: int) -> float:
    """Decode ionosphere residual value (TECU) for selection 0..3."""
    if sel == 0:
        return enc * 0.04 - 0.28
    if sel == 1:
        return enc * 0.04 - 2.52
    if sel == 2:
        return enc * 0.04 - 20.44
    # sel == 3
    return enc * 0.04 - 327.64


def features_from_hpac(frames, gad_index: Dict[int, dict]) -> List[dict]:
    feats: List[dict] = []
    for rec, payload, _ in frames:
        if rec.get("type") != 1:
            continue
        try:
            hpac = parse_hpac_payload(payload, rec.get("subtype", 0))
        except Exception:
            continue
        for area in hpac.get("areas", []):
            area_id = area.get("area_id")
            gad = gad_index.get(area_id)
            if not gad:
                continue
            tropo = area.get("tropo")
            iono = area.get("iono")
            lat_nodes = gad["lat_nodes"]
            lon_nodes = gad["lon_nodes"]
            lat_spacing = gad["lat_spacing_deg"]
            lon_spacing = gad["lon_spacing_deg"]
            ref_lat = gad["north_deg"]
            ref_lon = gad["west_deg"]
            # Build combined node values so each dot has both tropo and iono where available
            nodes: Dict[Tuple[int, int], dict] = {}
            # Tropo
            if tropo and "residuals" in tropo and tropo["residuals"]:
                residuals = tropo["residuals"]
                total = min(len(residuals), lat_nodes * lon_nodes)
                idx = 0
                for i in range(lat_nodes):
                    for j in range(lon_nodes):
                        if idx >= total:
                            break
                        val_m = residuals[idx] * 0.004
                        idx += 1
                        n = nodes.setdefault((i, j), {})
                        n["tropo_m"] = float(val_m)
            elif tropo:
                T00, T01, T10, T11, eq = _decode_tropo_coeffs(tropo)
                south = gad["south_deg"]; north = gad["north_deg"]
                west = gad["west_deg"]; east = gad["east_deg"]
                c_lat = (north + south) / 2.0
                c_lon = (west + east) / 2.0
                for i in range(lat_nodes):
                    lat = ref_lat - i * lat_spacing
                    dphi = lat - c_lat
                    for j in range(lon_nodes):
                        lon = ref_lon + j * lon_spacing
                        dlmb = lon - c_lon
                        if eq == 0:
                            val_m = T00
                        elif eq == 1:
                            val_m = T00 + T10 * dphi + T01 * dlmb
                        else:
                            val_m = T00 + T10 * dphi + T01 * dlmb + T11 * dphi * dlmb
                        n = nodes.setdefault((i, j), {})
                        n["tropo_m"] = float(val_m)
            # Iono
            if iono:
                try:
                    acc = [[{"sum": 0.0, "cnt": 0} for _ in range(lon_nodes)] for _ in range(lat_nodes)]
                    for srec in iono.get("sat_data", []):
                        residuals = srec.get("residuals")
                        sel = srec.get("residual_field_size")
                        if residuals and sel is not None:
                            total = min(len(residuals), lat_nodes * lon_nodes)
                            idx = 0
                            for i in range(lat_nodes):
                                for j in range(lon_nodes):
                                    if idx >= total:
                                        break
                                    val_tecu = _decode_iono_residual(int(residuals[idx]), int(sel))
                                    idx += 1
                                    acc[i][j]["sum"] += float(val_tecu)
                                    acc[i][j]["cnt"] += 1
                        coeffs = srec.get("coeffs")
                        if coeffs:
                            C00, C01, C10, C11 = _decode_iono_coeffs(coeffs, int(srec.get("coeff_size", 0)))
                            south = gad["south_deg"]; north = gad["north_deg"]
                            west = gad["west_deg"]; east = gad["east_deg"]
                            c_lat = (north + south) / 2.0
                            c_lon = (west + east) / 2.0
                            eq = int(iono.get("eq_type", 0))
                            for i in range(lat_nodes):
                                lat = ref_lat - i * lat_spacing
                                dphi = lat - c_lat
                                for j in range(lon_nodes):
                                    lon = ref_lon + j * lon_spacing
                                    dlmb = lon - c_lon
                                    if eq == 0:
                                        v = C00
                                    elif eq == 1:
                                        v = C00 + C10 * dphi + C01 * dlmb
                                    else:
                                        v = C00 + C10 * dphi + C01 * dlmb + C11 * dphi * dlmb
                                    acc[i][j]["sum"] += float(v)
                                    acc[i][j]["cnt"] += 1
                    for i in range(lat_nodes):
                        for j in range(lon_nodes):
                            c = acc[i][j]["cnt"]
                            if c > 0:
                                avg = acc[i][j]["sum"] / c
                                n = nodes.setdefault((i, j), {})
                                n["iono_tecu"] = float(avg)
                except Exception:
                    pass
            # Emit combined features
            for i in range(lat_nodes):
                lat = ref_lat - i * lat_spacing
                for j in range(lon_nodes):
                    lon = ref_lon + j * lon_spacing
                    n = nodes.get((i, j))
                    if not n:
                        continue
                    feats.append({
                        "type": "Feature",
                        "properties": {
                            "layer": "hpac_node",
                            "area_id": area_id,
                            "lat": lat,
                            "lon": lon,
                            "value_m": n.get("tropo_m"),
                            "value_tecu": n.get("iono_tecu"),
                        },
                        "geometry": {"type": "Point", "coordinates": [lon, lat]},
                    })
    return feats


def color_for_value(val: float) -> str:
    v = max(min(val / 0.2, 1.0), -1.0)  # ±20 cm range mapped to colors
    if v >= 0:
        r, g, b = 255, int(255 * (1.0 - v)), 0
    else:
        v = -v
        r, g, b = 0, int(255 * v), 255
    return f"#{r:02x}{g:02x}{b:02x}"


def color_for_tecu(val_tecu: float) -> str:
    # Map ±10 TECU to the same blue↔red scheme
    v = max(min(val_tecu / 10.0, 1.0), -1.0)
    if v >= 0:
        r, g, b = 255, int(255 * (1.0 - v)), 0
    else:
        v = -v
        r, g, b = 0, int(255 * v), 255
    return f"#{r:02x}{g:02x}{b:02x}"


def collect_latest_ocb(frames) -> Dict[int, dict]:
    """Return latest orbit/clock per satellite across all OCB frames."""
    latest: Dict[int, dict] = {}
    for rec, payload, _ in frames:
        if rec.get("type") != 0:  # OCB family
            continue
        try:
            ocb = parse_ocb_payload(payload, rec.get("subtype", 0))
        except Exception:
            # Skip malformed/partial OCB frames
            continue
        eph_type = ocb.get("eph_type")
        subtype = rec.get("subtype", 0)
        # Track time tag if full 32-bit available for this frame
        t32: int | None = None
        if rec.get("time_tag_type", 0) == 1:
            try:
                t32 = int(rec.get("time_tag", 0))
            except Exception:
                t32 = None
        for sid, srec in zip(ocb.get("sat_ids", []), ocb.get("satellites", [])):
            entry = latest.get(sid, {"sat_id": sid, "subtype": subtype})
            entry["eph_type"] = eph_type
            if "orbit" in srec:
                entry["orbit"] = srec["orbit"]
            if "clock" in srec:
                entry["clock"] = srec["clock"]
            if "bias" in srec:
                entry["bias"] = srec["bias"]
            # Update per-satellite last full time tag if provided
            if t32 is not None:
                prev = entry.get("t32")
                entry["t32"] = t32 if prev is None or t32 >= prev else prev
            latest[sid] = entry
    return latest


def _prn_label(subtype: int, sat_id: int) -> str:
    prefix = {0: "G", 1: "R", 2: "E", 3: "C", 4: "Q"}.get(subtype, "S")
    return f"{prefix}{sat_id:02d}"


def build_ocb_table_html(ocb_by_sat: Dict[int, dict], last_ocb_t32: int | None = None) -> str:
    rows: List[str] = []
    header = (
        "<tr><th>PRN</th><th>UTC</th><th>eph_type</th><th>IODE</th>"
        "<th>Orbit radial (m)</th><th>along (m)</th><th>cross (m)</th>"
        "<th>Clock (ns)</th><th>URE idx</th><th>Phase bias (m by track)</th><th>Code bias (m by track)</th></tr>"
    )
    # Constellation summary
    const_counts = {"G": 0, "R": 0, "E": 0, "C": 0, "Q": 0}
    for sid in sorted(ocb_by_sat.keys()):
        e = ocb_by_sat[sid]
        subtype = int(e.get("subtype", 0))
        prn = _prn_label(subtype, sid)
        const_counts[prn[0]] = const_counts.get(prn[0], 0) + 1
        eph = e.get("eph_type", "")
        orb = e.get("orbit", {})
        clk = e.get("clock", {})
        bias = e.get("bias", {})
        iode = orb.get("iode", "") if orb else ""
        rad = float(orb.get("radial_m")) if orb and isinstance(orb.get("radial_m"), (int, float)) else float(0)
        along = float(orb.get("along_m")) if orb and isinstance(orb.get("along_m"), (int, float)) else float(0)
        cross = float(orb.get("cross_m")) if orb and isinstance(orb.get("cross_m"), (int, float)) else float(0)
        clk_m = float(clk.get("clock_m")) if clk and isinstance(clk.get("clock_m"), (int, float)) else float(0)
        # Convert meters to nanoseconds using speed of light
        c = 299792458.0
        clk_ns = (clk_m / c) * 1e9 if c else 0.0
        ure = clk.get("ure_index", "") if clk else ""
        # Compose bias detail strings
        phase_detail = ""
        code_detail = ""
        if bias:
            pidx = bias.get("phase_mask_indices", []) or []
            pvals = bias.get("phase_biases", []) or []
            items = []
            for idx, vb in zip(pidx, pvals):
                try:
                    b = float(vb.get("bias_m", 0.0))
                except Exception:
                    b = 0.0
                fix = vb.get("fix", 0)
                cont = vb.get("continuity", "")
                star = "*" if int(fix) == 1 else ""
                items.append(f"{idx}:{b:.3f}{star}")
            phase_detail = ", ".join(items)
            cidx = bias.get("code_mask_indices", []) or []
            cvals = bias.get("code_biases_m", []) or []
            items = []
            for idx, b in zip(cidx, cvals):
                try:
                    bf = float(b)
                except Exception:
                    bf = 0.0
                items.append(f"{idx}:{bf:.3f}")
            code_detail = ", ".join(items)
        # Per-SV UTC time (HH:MM:SS) derived from last full t32 for this SV
        utc_str = ""
        t32_sv = e.get("t32")
        if isinstance(t32_sv, int):
            import datetime as _dt
            base = _dt.datetime(2010, 1, 1, tzinfo=_dt.timezone.utc)
            utc_str = (base + _dt.timedelta(seconds=t32_sv)).strftime("%H:%M:%S")
        rows.append(
            f"<tr><td>{prn}</td><td>{utc_str}</td><td>{eph}</td><td>{iode}</td>"
            f"<td>{rad:.3f}</td><td>{along:.3f}</td><td>{cross:.3f}</td>"
            f"<td>{clk_ns:.1f}</td><td>{ure}</td><td style='white-space:nowrap'>{phase_detail}</td><td style='white-space:nowrap'>{code_detail}</td></tr>"
        )
    # Optional last time tag info
    time_html = ""
    if last_ocb_t32 is not None:
        # Convert seconds-since-2010-01-01 to ISO UTC
        import datetime as _dt
        base = _dt.datetime(2010, 1, 1, tzinfo=_dt.timezone.utc)
        iso = (base + _dt.timedelta(seconds=int(last_ocb_t32))).strftime("%Y-%m-%d %H:%M:%S %Z")
        time_html = f"<div style='margin:4px 0'>Latest OCB time tag: {last_ocb_t32}s since 2010-01-01 (≈ {iso})</div>"
    summary = (
        f"<div style='margin:0 0 6px 0;font-weight:600'>"
        f"GPS: {const_counts.get('G',0)} &nbsp; "
        f"GLO: {const_counts.get('R',0)} &nbsp; "
        f"GAL: {const_counts.get('E',0)} &nbsp; "
        f"BDS: {const_counts.get('C',0)}"
        f"</div>"
    )
    table = (
        "<div style='font: 13px/1.2 -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif'>"
        "<h4 style='margin:0 0 8px 0'>OCB Orbit/Clock</h4>" + summary + time_html +
        "<div style='max-height: 240px; overflow:auto; border:1px solid #ccc'>"
        "<table style='border-collapse:collapse; width:100%'>" + header + "".join(rows) + "</table>"
        "</div></div>"
    )
    return table


def render_map(fc: dict, ocb_html: str, out_html: str = "map.html") -> None:
    import folium
    from branca.element import MacroElement, Template

    # Center
    pts = [f for f in fc["features"] if f["geometry"]["type"] == "Point"]
    if pts:
        lat_c = sum(p["properties"]["lat"] for p in pts) / len(pts)
        lon_c = sum(p["properties"]["lon"] for p in pts) / len(pts)
    else:
        lat_c, lon_c = 39.0, -96.0
    m = folium.Map(location=[lat_c, lon_c], zoom_start=5, tiles="OpenStreetMap")

    # GAD polygons
    for f in fc["features"]:
        if f["properties"].get("layer") != "gad":
            continue
        coords = f["geometry"]["coordinates"][0]
        latlon = [[c[1], c[0]] for c in coords]
        p = f["properties"]
        folium.Polygon(
            locations=latlon, color="blue", weight=2, fill=True, fill_opacity=0.06,
            tooltip=f"Area {p['area_id']} [{p['south']:.2f},{p['west']:.2f}]→[{p['north']:.2f},{p['east']:.2f}]",
        ).add_to(m)

    # HPAC node markers (combined tropo+iono, concentric)
    for f in fc["features"]:
        if f["properties"].get("layer") != "hpac_node":
            continue
        p = f["properties"]
        tval = p.get("value_m")
        ival = p.get("value_tecu")
        col_t = color_for_value(float(tval)) if tval is not None else "#666666"
        col_i = color_for_tecu(float(ival)) if ival is not None else "#666666"
        tip_lines = [f"Area {p['area_id']}"]
        if tval is not None:
            tip_lines.append(f"Tropo: {tval:.3f} m")
        if ival is not None:
            tip_lines.append(f"Iono: {ival:.2f} TECU")
        tip = "\\n".join(tip_lines)
        # Outer: tropo
        folium.CircleMarker(location=[p["lat"], p["lon"]], radius=6, color=col_t, weight=0,
                            fill=True, fill_color=col_t, fill_opacity=0.85, tooltip=tip).add_to(m)
        # Inner: iono
        folium.CircleMarker(location=[p["lat"], p["lon"]], radius=3, color=col_i, weight=0,
                            fill=True, fill_color=col_i, fill_opacity=0.9).add_to(m)

    # OCB overlay panel
    template = Template(
        """
        {% macro html(this, kwargs) %}
        <div id="ocb-panel" style="position: fixed; bottom: 10px; left: 10px; width: 460px; z-index: 999999;">
          <div style="background:#fff; padding:8px; border:1px solid #999; box-shadow: 0 2px 6px rgba(0,0,0,0.3);">
            {{ this.ocb | safe }}
          </div>
        </div>
        {% endmacro %}
        """
    )
    macro = MacroElement()
    macro._template = template
    # attach data
    macro.ocb = ocb_html
    m.get_root().add_child(macro)

    # Color legend overlay for Tropo (m) and Iono (TECU)
    legend_tpl = Template(
        """
        {% macro html(this, kwargs) %}
        <div style="position: fixed; bottom: 10px; right: 10px; z-index: 999999;">
          <div style="background:#fff; padding:8px; border:1px solid #999; font: 12px/1.2 -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif; box-shadow: 0 2px 6px rgba(0,0,0,0.3);">
            <div style="font-weight:600; margin-bottom:4px;">Color scales</div>
            <div style="margin-bottom:6px;">Tropo residual (m): -0.20 <span style="display:inline-block; width:140px; height:10px; background: linear-gradient(90deg, #00ffff, #ff0000);"></span> +0.20</div>
            <div>Iono (TECU): -10 <span style="display:inline-block; width:140px; height:10px; background: linear-gradient(90deg, #00ffff, #ff0000);"></span> +10</div>
          </div>
        </div>
        {% endmacro %}
        """
    )
    legend = MacroElement()
    legend._template = legend_tpl
    m.get_root().add_child(legend)

    # Save beside this script
    script_dir = Path(__file__).resolve().parent
    m.save(str(script_dir / out_html))


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description="GAD+HPAC+OCB map generator")
    ap.add_argument("input", help="SPARTN .bin or line .log")
    ap.add_argument("--center-lat", type=float, default=None, help="Center latitude for NEAR (no GAD) HPAC plotting")
    ap.add_argument("--center-lon", type=float, default=None, help="Center longitude for NEAR (no GAD) HPAC plotting")
    ap.add_argument("--grid-size-deg", type=float, default=4.0, help="Total grid size in degrees when synthesizing GAD (default 4.0)")
    ap.add_argument("--grid-steps", type=int, default=17, help="Grid steps per axis when synthesizing GAD (default 17)")
    args = ap.parse_args()
    path = args.input
    # Support both binary .bin and line logs with format:
    # [timestamp_ms] [type] [subtype] [length] [hex_data]
    data: bytes
    try:
        with open(path, "rb") as f:
            blob = f.read()
    except Exception as e:
        print(f"Failed to read {path}: {e}", file=sys.stderr)
        sys.exit(1)
    # Heuristic: if blob looks like ASCII and contains lines with tokens and hex, parse as log
    try:
        text = blob.decode("ascii", errors="ignore")
        lines = [ln.strip() for ln in text.splitlines() if ln.strip() and not ln.startswith("#")]
        is_log = False
        if lines:
            parts = lines[0].split()
            if len(parts) >= 5:
                hexfld = parts[4]
                # Accept both timestamp styles: with colon (e.g. 12:34:56.789) or numeric ms (e.g. 12345)
                if all(c in "0123456789abcdefABCDEF" for c in hexfld):
                    is_log = True
        if is_log:
            chunks: list[bytes] = []
            for ln in lines:
                ps = ln.split()
                if len(ps) < 5:
                    continue
                hexfld = ps[4]
                try:
                    chunks.append(bytes.fromhex(hexfld))
                except Exception:
                    continue
            data = b"".join(chunks)
        else:
            data = blob
    except Exception:
        data = blob

    frames = extract_spartn_frames(data)
    # Heuristic: keep only GAD areas within plausible N. America/US PP footprint if map is too noisy
    gad_index = collect_gad(frames)
    if len(gad_index) > 200:
        # Rough bounding-box filter using area centers
        allowed: set[int] = set()
        for rec, payload, _ in frames:
            if rec.get("type") == 2 and rec.get("subtype") == 0:
                try:
                    gad = parse_gad_payload(payload)
                except Exception:
                    continue
                for a in gad.get("areas", []):
                    c_lat = (a.get("north_deg", 0.0) + a.get("south_deg", 0.0)) / 2.0
                    c_lon = (a.get("west_deg", 0.0) + a.get("east_deg", 0.0)) / 2.0
                    if 8.0 <= c_lat <= 72.0 and -170.0 <= c_lon <= -50.0:
                        allowed.add(a.get("area_id"))
        if allowed:
            gad_index = collect_gad(frames, allowed)
    # If no GAD: allow optional synthetic grid around provided center so HPAC can be visualized
    if not gad_index:
        if args.center_lat is not None and args.center_lon is not None and args.grid_steps >= 2:
            # Build synthetic GAD per HPAC area found
            synth: Dict[int, dict] = {}
            half = args.grid_size_deg / 2.0
            north = args.center_lat + half
            south = args.center_lat - half
            west = args.center_lon - half
            east = args.center_lon + half
            lat_nodes = args.grid_steps
            lon_nodes = args.grid_steps
            lat_spacing = (north - south) / (lat_nodes - 1)
            lon_spacing = (east - west) / (lon_nodes - 1)
            # Collect HPAC area_ids present
            area_ids: set[int] = set()
            for rec, payload, _ in frames:
                if rec.get("type") != 1:
                    continue
                try:
                    hp = parse_hpac_payload(payload, rec.get("subtype", 0))
                except Exception:
                    continue
                for a in hp.get("areas", []):
                    area_ids.add(int(a.get("area_id", 0)))
            for aid in area_ids:
                synth[aid] = {
                    "area_id": aid,
                    "north_deg": north,
                    "south_deg": south,
                    "west_deg": west,
                    "east_deg": east,
                    "lat_nodes": lat_nodes,
                    "lon_nodes": lon_nodes,
                    "lat_spacing_deg": lat_spacing,
                    "lon_spacing_deg": lon_spacing,
                }
            gad_index = synth
            print("No GAD found – synthesized local grid for HPAC plotting", file=sys.stderr)
        else:
            print("No GAD found (rendering OCB-only map)", file=sys.stderr)

    feats: List[dict] = []
    if gad_index:
        # Only add GAD polygons if these are real, not synthesized
        # Synthetic grids are for HPAC point plotting only
        real_gad = collect_gad(frames)
        if real_gad:
            feats.extend(features_from_gad(real_gad))
        feats.extend(features_from_hpac(frames, gad_index))
    fc = {"type": "FeatureCollection", "features": feats}
    # Save beside this script
    out_dir = Path(__file__).resolve().parent
    with open(out_dir / "map.geojson", "w") as f:
        json.dump(fc, f)
    print(f"Wrote {out_dir / 'map.geojson'}")

    ocb_latest = collect_latest_ocb(frames)
    # Determine latest 32-bit time tag among OCB for display
    last_t32: int | None = None
    for rec, _payload, _ in frames:
        if rec.get("type") == 0:
            if rec.get("time_tag_type", 0) == 1:
                t = int(rec.get("time_tag", 0))
                last_t32 = t if last_t32 is None or t > last_t32 else last_t32
    ocb_html = build_ocb_table_html(ocb_latest, last_t32)
    try:
        render_map(fc, ocb_html, "map.html")
        print(f"Wrote {Path(__file__).resolve().parent / 'map.html'}")
    except Exception as e:
        print(f"Failed to render map: {e}")


if __name__ == "__main__":
    main()


