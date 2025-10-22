import json
import math
import os
import sys
from typing import Any, Dict, List, Tuple


# Use a non-interactive backend
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_az_el(json_path: str) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    rx = data.get("receiver", {})
    sats = data.get("satellites", [])
    return rx, sats


def skyplot(ax: plt.Axes, sats: List[Dict[str, Any]], title: str = "") -> None:
    # Convert to polar: theta = azimuth (radians), r = 90 - elevation
    # Group by constellation using PRN prefix
    groups = {
        "G": {"theta": [], "r": [], "label": [], "color": "tab:blue", "name": "GPS"},
        "E": {"theta": [], "r": [], "label": [], "color": "tab:green", "name": "Galileo"},
        "C": {"theta": [], "r": [], "label": [], "color": "tab:red", "name": "BeiDou"},
        "R": {"theta": [], "r": [], "label": [], "color": "tab:orange", "name": "GLONASS"},
    }

    for s in sats:
        az = float(s.get("az_deg", 0.0))
        el = float(s.get("el_deg", 0.0))
        prn = s.get("prn", "")
        const = prn[:1].upper() if prn else "G"
        if const not in groups:
            const = "G"
        el = max(-90.0, min(90.0, el))
        theta = math.radians(az)
        r = 90.0 - el
        groups[const]["theta"].append(theta)
        groups[const]["r"].append(r)
        groups[const]["label"].append(prn)

    # Configure polar plot: 0 deg at North, clockwise
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_rlim(0, 90)
    ax.set_rticks([0, 15, 30, 45, 60, 75, 90])
    ax.set_rlabel_position(225)  # move radial labels away from North
    ax.grid(True, linestyle=":", alpha=0.6)

    # Scatter per constellation
    handles = []
    for key in ("G", "E", "C", "R"):
        g = groups[key]
        if not g["theta"]:
            continue
        sc = ax.scatter(
            g["theta"], g["r"], c=g["color"], s=70, alpha=0.85,
            edgecolors="k", linewidths=0.4, label=g["name"]
        )
        handles.append(sc)
        # Annotate
        for theta, r, label in zip(g["theta"], g["r"], g["label"]):
            ax.text(theta, r, label, fontsize=8, ha="center", va="center", color="black")

    if handles:
        ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1.05), frameon=False, fontsize=9)

    ax.set_title(title, fontsize=14)


def main(argv: List[str]) -> None:
    if len(argv) < 3:
        print("Usage: python skyplot_plot.py <az_el.json> <out.png> [--title 'Skyplot']")
        sys.exit(1)

    in_json = argv[1]
    out_png = argv[2]
    title = ""
    if "--title" in argv:
        idx = argv.index("--title")
        if idx + 1 < len(argv):
            title = argv[idx + 1]

    rx, sats = load_az_el(in_json)
    rx_str = f"Lat {rx.get('lat_deg', 0):.4f}°, Lon {rx.get('lon_deg', 0):.4f}°, h {rx.get('h_m', 0)} m"
    title_full = title or f"GPS Skyplot ({rx_str})"

    fig = plt.figure(figsize=(8.0, 8.0), dpi=220)
    ax = fig.add_subplot(111, projection="polar")
    skyplot(ax, sats, title_full)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    print(out_png)


if __name__ == "__main__":
    main(sys.argv)