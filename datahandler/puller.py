"""
puller.py
---------

Fetch RTCM and PMP logs over HTTP for a specified UTC time range and
organize them into an output directory structure for downstream processing.

Current scope (phase 1):
  - Download RTCM logs from the RTCM data endpoint into out/rtcm
  - Download PMP logs (organized by day folders) into out/pmp

Planned (phase 2 – not implemented here):
  - Create out/rtcm_filtered by calling ephemeris_filter.py per file epoch
  - Create out/spartn by converting PMP to SPARTN logs in 5-minute intervals

Endpoints
  - PMP listing:    http://143.198.0.80:8000/data/               (day subfolders)
  - RTCM listing:   http://143.198.0.80:8082/data/          (flat files)

Examples
  python3 datahandler/puller.py \
    --start 2025-08-19T12:00:00Z \
    --end   2025-08-19T13:00:00Z \
    --lat 38.36876 --lon -78.92694 --h 100 \
    --out /tmp/logpull

Notes
  - This script assumes the servers provide simple directory listings
    with anchor hrefs to files/subfolders.
  - Timestamps are parsed from filenames:
      * RTCM: rtcm_YYYYMMDD_HHMMSS.(log|bin) or rtcm_YYYYMMDD.(log|bin)
      * PMP:  day directory YYYYMMDD/ containing PMP_YYYYMMDD_HHMMSS_mmm.bin
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import sys
import time
from dataclasses import dataclass
from html import unescape
from typing import Iterable, List, Optional, Sequence, Tuple
import io
import subprocess
import zipfile

try:
    # Prefer stdlib over adding a new dependency
    from urllib.parse import urljoin
    from urllib.request import urlopen
except Exception:
    raise


DEFAULT_PMP_BASE = "http://143.198.0.80:8000/data/"
DEFAULT_RTCM_BASE = "http://143.198.0.80:8082/data/"


_HREF_RE = re.compile(r"href=\"([^\"]+)\"", re.IGNORECASE)
_PMP_FILE_RE = re.compile(r"^PMP_(\d{8})_(\d{6})_(\d{3})\.bin$", re.IGNORECASE)
_DAY_DIR_RE = re.compile(r"^(\d{8})/?$")
_RTCM_FILE_RE = re.compile(
    r"^rtcm_(\d{8})(?:_(\d{6}))?\.(?:log|bin)$",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class TimeRange:
    start: dt.datetime  # inclusive, UTC
    end: dt.datetime    # inclusive, UTC

    def contains(self, when: dt.datetime) -> bool:
        return self.start <= when <= self.end


def parse_utc(s: str) -> dt.datetime:
    """Parse a UTC timestamp in flexible formats into an aware datetime.

    Accepted formats:
      - YYYY-MM-DD
      - YYYY-MM-DDTHH:MM[:SS][Z]
      - YYYYMMDD
      - YYYYMMDD_HHMMSS
    """
    s = s.strip()
    def _mk(y: int, m: int, d: int, hh: int = 0, mm: int = 0, ss: int = 0) -> dt.datetime:
        return dt.datetime(y, m, d, hh, mm, ss, tzinfo=dt.timezone.utc)

    # ISO date
    m = re.fullmatch(r"(\d{4})-(\d{2})-(\d{2})", s)
    if m:
        return _mk(int(m.group(1)), int(m.group(2)), int(m.group(3)))

    # ISO datetime (with or without seconds), optional trailing Z
    m = re.fullmatch(r"(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2})(?::(\d{2}))?(Z)?", s)
    if m:
        y, mo, d = int(m.group(1)), int(m.group(2)), int(m.group(3))
        hh, mm, ss = int(m.group(4)), int(m.group(5)), int(m.group(6) or 0)
        return _mk(y, mo, d, hh, mm, ss)

    # Compact date
    m = re.fullmatch(r"(\d{8})", s)
    if m:
        y = int(s[0:4]); mo = int(s[4:6]); d = int(s[6:8])
        return _mk(y, mo, d)

    # Compact datetime with underscore
    m = re.fullmatch(r"(\d{8})_(\d{6})", s)
    if m:
        ymd, hms = m.group(1), m.group(2)
        y = int(ymd[0:4]); mo = int(ymd[4:6]); d = int(ymd[6:8])
        hh = int(hms[0:2]); mm_ = int(hms[2:4]); ss = int(hms[4:6])
        return _mk(y, mo, d, hh, mm_, ss)

    raise ValueError(f"Unrecognized UTC time format: {s}")


def http_list(url: str) -> List[str]:
    """Return a list of href targets from a simple directory listing page."""
    with urlopen(url) as resp:
        data = resp.read()
    # Try to decode as UTF-8; fall back to latin-1 if needed
    try:
        text = data.decode("utf-8", errors="replace")
    except Exception:
        text = data.decode("latin-1", errors="replace")
    hrefs = [unescape(h) for h in _HREF_RE.findall(text)]
    # Filter out anchors and parent links
    cleaned: List[str] = []
    for h in hrefs:
        if not h:
            continue
        if h.startswith("#"):
            continue
        if h.startswith("?"):
            continue
        if h in ("..", "../"):
            continue
        cleaned.append(h)
    return cleaned


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def download_file(url: str, dest_path: str) -> None:
    tmp_path = dest_path + ".part"
    with urlopen(url) as resp, open(tmp_path, "wb") as out:
        while True:
            chunk = resp.read(1024 * 64)
            if not chunk:
                break
            out.write(chunk)
    os.replace(tmp_path, dest_path)


def iter_day_strings(tr: TimeRange) -> Iterable[str]:
    cur = tr.start.date()
    end = tr.end.date()
    while cur <= end:
        yield cur.strftime("%Y%m%d")
        cur = cur + dt.timedelta(days=1)


def iter_hour_keys(tr: TimeRange) -> Iterable[Tuple[str, str, dt.datetime]]:
    """Yield (YYYYMMDD, HH, hour_start_dt) for each hour intersecting the range."""
    # Align to hour start
    cur = tr.start.replace(minute=0, second=0, microsecond=0)
    if cur < tr.start:
        cur = cur + dt.timedelta(hours=1)
    end = tr.end
    # Include the hour that contains tr.end if end is exactly at hour start? We include if cur <= end
    while cur <= end:
        yield cur.strftime("%Y%m%d"), cur.strftime("%H"), cur
        cur = cur + dt.timedelta(hours=1)


def download_and_extract_zip(url: str, extract_dir: str, allow_exts: Optional[Tuple[str, ...]] = None) -> int:
    """Download a ZIP archive into memory and extract files to extract_dir.

    Returns the number of files extracted (that either didn't exist or were overwritten).
    If allow_exts is provided, only files whose lowercased names end with one of those
    extensions are extracted.
    """
    ensure_dir(extract_dir)
    with urlopen(url) as resp:
        data = resp.read()
    zf = zipfile.ZipFile(io.BytesIO(data))
    count = 0
    for zi in zf.infolist():
        if zi.is_dir():
            continue
        name = zi.filename.split('/')[-1]
        if not name:
            continue
        if allow_exts is not None and not any(name.lower().endswith(ext) for ext in allow_exts):
            continue
        dest_path = os.path.join(extract_dir, name)
        # Extract to temp and move to avoid partials
        tmp_path = dest_path + '.part'
        with zf.open(zi, 'r') as src, open(tmp_path, 'wb') as out:
            while True:
                chunk = src.read(1024 * 64)
                if not chunk:
                    break
                out.write(chunk)
        os.replace(tmp_path, dest_path)
        count += 1
    return count


def parse_pmp_epoch_ms_from_name(name: str) -> Optional[int]:
    m = _PMP_FILE_RE.match(name)
    if not m:
        return None
    ymd, hms, ms = m.group(1), m.group(2), int(m.group(3))
    try:
        base = dt.datetime.strptime(ymd + hms, "%Y%m%d%H%M%S").replace(tzinfo=dt.timezone.utc)
        return int(base.timestamp() * 1000) + ms
    except Exception:
        return None


def parse_rtcm_epoch_from_name(name: str) -> Optional[dt.datetime]:
    m = _RTCM_FILE_RE.match(name)
    if not m:
        return None
    ymd = m.group(1)
    hms = m.group(2)
    try:
        if hms:
            base = dt.datetime.strptime(ymd + hms, "%Y%m%d%H%M%S").replace(tzinfo=dt.timezone.utc)
        else:
            base = dt.datetime.strptime(ymd, "%Y%m%d").replace(tzinfo=dt.timezone.utc)
        return base
    except Exception:
        return None


def fetch_rtcm(tr: TimeRange, base_url: str, out_dir: str, dry_run: bool = False) -> int:
    ensure_dir(out_dir)
    links = http_list(base_url)
    num = 0
    for href in links:
        # Convert to absolute URL if needed
        abs_url = urljoin(base_url, href)
        fname = href.rstrip("/").split("/")[-1]
        when = parse_rtcm_epoch_from_name(fname)
        if when is None:
            continue
        # If filename has only date (no time), include if the whole day intersects the range
        if re.match(r"^rtcm_\d{8}\.", fname, re.IGNORECASE):
            day_start = when
            day_end = when + dt.timedelta(days=1) - dt.timedelta(seconds=1)
            intersects = not (day_end < tr.start or day_start > tr.end)
            if not intersects:
                continue
        else:
            if not tr.contains(when):
                continue
        dest = os.path.join(out_dir, fname)
        if os.path.exists(dest):
            continue
        print(f"[RTCM] Downloading {fname}")
        if not dry_run:
            download_file(abs_url, dest)
        num += 1
    return num


def fetch_pmp(tr: TimeRange, base_url: str, out_dir: str, dry_run: bool = False) -> int:
    ensure_dir(out_dir)
    num = 0
    # Iterate day subfolders matching the range
    base_links: Optional[List[str]] = None
    for day in iter_day_strings(tr):
        day_url = urljoin(base_url, f"{day}/")
        try:
            links = http_list(day_url)
        except Exception:
            # Try without trailing slash or skip if missing
            try:
                day_url_alt = urljoin(base_url, day)
                links = http_list(day_url_alt)
                day_url = day_url_alt
            except Exception:
                # Fallback: base index page may list full paths like /data/YYYYMMDD/PMP_...
                if base_links is None:
                    try:
                        base_links = http_list(base_url)
                    except Exception:
                        base_links = []
                filtered: List[str] = []
                for href in base_links:
                    fname = href.rstrip("/").split("/")[-1]
                    if _PMP_FILE_RE.match(fname) is None:
                        continue
                    # Ensure the href corresponds to this day (path contains day or filename ymd == day)
                    if day not in href:
                        m2 = _PMP_FILE_RE.match(fname)
                        if not m2 or m2.group(1) != day:
                            continue
                    filtered.append(href)
                links = filtered
                day_url = base_url
        for href in links:
            fname = href.rstrip("/").split("/")[-1]
            if _PMP_FILE_RE.match(fname) is None:
                continue
            epoch_ms = parse_pmp_epoch_ms_from_name(fname)
            if epoch_ms is None:
                continue
            when = dt.datetime.fromtimestamp(epoch_ms / 1000.0, tz=dt.timezone.utc)
            if not tr.contains(when):
                continue
            abs_url = urljoin(day_url, href)
            dest = os.path.join(out_dir, fname)
            if os.path.exists(dest):
                continue
            print(f"[PMP ] Downloading {fname}")
            if not dry_run:
                download_file(abs_url, dest)
            num += 1
    return num


def fetch_rtcm_zips(tr: TimeRange, base_url: str, out_dir: str, dry_run: bool = False) -> int:
    """Fetch per-hour ZIPs from base_url/YYYYMMDD/HH/YYYYMMDD_HH.zip and extract .log/.bin."""
    ensure_dir(out_dir)
    total = 0
    for ymd, hh, _hour_dt in iter_hour_keys(tr):
        zip_url = urljoin(base_url, f"{ymd}/{hh}/{ymd}_{hh}.zip")
        marker = os.path.join(out_dir, f".hour_{ymd}_{hh}.done")
        if os.path.exists(marker):
            continue
        print(f"[RTCM] Hour archive {ymd}_{hh}.zip")
        if not dry_run:
            try:
                n = download_and_extract_zip(zip_url, out_dir, allow_exts=(".log", ".bin"))
            except Exception as e:
                print(f"[RTCM] ZIP fetch failed for {ymd}_{hh}: {e}", file=sys.stderr)
                continue
            with open(marker, 'w') as f:
                f.write(str(n))
            total += n
    return total


def fetch_pmp_zips(tr: TimeRange, base_url: str, out_dir: str, dry_run: bool = False) -> int:
    """Fetch per-hour ZIPs from base_url/YYYYMMDD/HH/YYYYMMDD_HH.zip and extract .bin."""
    ensure_dir(out_dir)
    total = 0
    for ymd, hh, _hour_dt in iter_hour_keys(tr):
        zip_url = urljoin(base_url, f"{ymd}/{hh}/{ymd}_{hh}.zip")
        marker = os.path.join(out_dir, f".hour_{ymd}_{hh}.done")
        if os.path.exists(marker):
            continue
        print(f"[PMP ] Hour archive {ymd}_{hh}.zip")
        if not dry_run:
            try:
                n = download_and_extract_zip(zip_url, out_dir, allow_exts=(".bin",))
            except Exception as e:
                print(f"[PMP ] ZIP fetch failed for {ymd}_{hh}: {e}", file=sys.stderr)
                continue
            with open(marker, 'w') as f:
                f.write(str(n))
            total += n
    return total


def filter_rtcm_logs(
    tr: TimeRange,
    in_dir: str,
    out_dir: str,
    lat: float,
    lon: float,
    h_m: float,
    mask_deg: float = 0.0,
) -> int:
    """Run ephemeris_filter.py for each RTCM log in time range, writing to out_dir."""
    ensure_dir(out_dir)
    # Discover input files
    all_files = [f for f in os.listdir(in_dir) if f.lower().startswith("rtcm_") and (f.lower().endswith('.log') or f.lower().endswith('.bin'))]
    count = 0
    eph_filter = os.path.abspath(os.path.join(os.path.dirname(__file__), 'orbits', 'ephemeris_filter.py'))
    for fname in sorted(all_files):
        when = parse_rtcm_epoch_from_name(fname)
        if when is None or not tr.contains(when):
            continue
        in_path = os.path.join(in_dir, fname)
        base, _ext = os.path.splitext(fname)
        out_path = os.path.join(out_dir, f"{base}_filtered.log")
        if os.path.exists(out_path):
            continue
        iso = when.strftime('%Y-%m-%dT%H:%M:%SZ')
        print(f"[FILT] {fname} @ {iso}")
        try:
            subprocess.run(
                [sys.executable, eph_filter, in_path, '--time', iso, '--lat', str(lat), '--lon', str(lon), '--h', str(h_m), '--mask', str(mask_deg), '--out', out_path],
                check=True,
                capture_output=True,
                text=True,
            )
            count += 1
        except subprocess.CalledProcessError as e:
            sys.stderr.write(f"[FILT] Failed {fname}: {e.stderr or e.stdout}\n")
            continue
    return count


def build_spartn_from_pmp(
    tr: TimeRange,
    pmp_dir: str,
    out_dir: str,
    key_hex: Optional[str] = None,
    rotate_seconds: int = 300,
    ms_per_frame: int = 1,
) -> int:
    """Invoke pmp_to_spartn_logger.py on PMP files within time range, writing .log into out_dir.

    Returns number of PMP input files passed to the converter.
    """
    ensure_dir(out_dir)
    # Collect hour-range PMP files
    pmp_files = []
    for fname in sorted(os.listdir(pmp_dir)):
        if not fname.lower().endswith('.bin'):
            continue
        epoch_ms = parse_pmp_epoch_ms_from_name(fname)
        if epoch_ms is None:
            continue
        when = dt.datetime.fromtimestamp(epoch_ms / 1000.0, tz=dt.timezone.utc)
        if not tr.contains(when):
            continue
        pmp_files.append(os.path.join(pmp_dir, fname))
    if not pmp_files:
        print("[SPRT] No PMP files in range")
        return 0
    converter = os.path.abspath(os.path.join(os.path.dirname(__file__), 'spartnPress', 'pmp_to_spartn_logger.py'))
    env = dict(os.environ)
    if key_hex:
        env['PP_AES_KEY'] = key_hex
    cmd = [sys.executable, converter, '--outdir', out_dir, '--rotate-seconds', str(rotate_seconds), '--ms-per-frame', str(ms_per_frame)] + pmp_files
    print(f"[SPRT] Converting {len(pmp_files)} PMP file(s) -> {out_dir}")
    try:
        subprocess.run(cmd, check=True, env=env)
    except subprocess.CalledProcessError as e:
        sys.stderr.write(f"[SPRT] Conversion failed: {e}\n")
        return 0
    return len(pmp_files)


def main() -> None:
    ap = argparse.ArgumentParser(description="Fetch RTCM and PMP logs for a UTC time range")
    ap.add_argument("--start", required=True, help="UTC start time (e.g., 2025-08-19T12:00:00Z or 20250819_120000)")
    ap.add_argument("--end", required=True, help="UTC end time (inclusive)")
    ap.add_argument("--lat", type=float, default=0.0, help="Receiver latitude (deg) [reserved for phase 2]")
    ap.add_argument("--lon", type=float, default=0.0, help="Receiver longitude (deg) [reserved for phase 2]")
    ap.add_argument("--h", type=float, default=100.0, help="Receiver height (m) [reserved for phase 2]")
    ap.add_argument("--out", default="pull_out", help="Output root directory")
    ap.add_argument("--pmp-base", default=DEFAULT_PMP_BASE, help="Base URL for PMP day directories")
    ap.add_argument("--rtcm-base", default=DEFAULT_RTCM_BASE, help="Base URL for RTCM files")
    ap.add_argument("--skip-pmp", action="store_true", help="Skip PMP downloads")
    ap.add_argument("--skip-rtcm", action="store_true", help="Skip RTCM downloads")
    ap.add_argument("--use-hour-zips", action="store_true", help="Fetch per-hour ZIP archives instead of listing pages")
    ap.add_argument("--filter-rtcm", action="store_true", help="Generate rtcm_filtered/ using ephemeris_filter.py for files in range")
    ap.add_argument("--spartn-from-pmp", action="store_true", help="Generate spartn/ logs from pmp/ files in range")
    ap.add_argument("--pp-key-hex", default=os.getenv('PP_AES_KEY', ''), help="AES-128 key hex for SPARTN decryption (optional; env PP_AES_KEY also used)")
    ap.add_argument("--dry-run", action="store_true", help="List what would be downloaded without writing files")
    args = ap.parse_args()

    start = parse_utc(args.start)
    end = parse_utc(args.end)
    if end < start:
        raise ValueError("--end must be >= --start")
    tr = TimeRange(start=start, end=end)

    # Prepare directory layout
    root = os.path.abspath(args.out)
    dir_rtcm = os.path.join(root, "rtcm")
    dir_rtcm_filtered = os.path.join(root, "rtcm_filtered")
    dir_pmp = os.path.join(root, "pmp")
    dir_spartn = os.path.join(root, "spartn")
    for d in (dir_rtcm, dir_rtcm_filtered, dir_pmp, dir_spartn):
        ensure_dir(d)

    print(f"Pulling logs UTC [{start.isoformat()} .. {end.isoformat()}] -> {root}")
    total = 0
    if not args.skip_rtcm:
        try:
            if args.use_hour_zips:
                n = fetch_rtcm_zips(tr, args.rtcm_base, dir_rtcm, dry_run=args.dry_run)
            else:
                n = fetch_rtcm(tr, args.rtcm_base, dir_rtcm, dry_run=args.dry_run)
            print(f"RTCM: {n} file(s) fetched")
            total += n
        except Exception as e:
            print(f"RTCM fetch failed: {e}", file=sys.stderr)
    if not args.skip_pmp:
        try:
            if args.use_hour_zips:
                n = fetch_pmp_zips(tr, args.pmp_base, dir_pmp, dry_run=args.dry_run)
            else:
                n = fetch_pmp(tr, args.pmp_base, dir_pmp, dry_run=args.dry_run)
            print(f"PMP:  {n} file(s) fetched")
            total += n
        except Exception as e:
            print(f"PMP fetch failed: {e}", file=sys.stderr)

    if args.dry_run:
        print("Dry run complete.")
    else:
        print(f"Done. {total} file(s) downloaded.")

    # Phase 2 (optional): filtering and SPARTN conversion
    if not args.dry_run and args.filter_rtcm:
        n = filter_rtcm_logs(tr, dir_rtcm, dir_rtcm_filtered, lat=args.lat, lon=args.lon, h_m=args.h)
        print(f"Filtered: {n} rtcm file(s)")
    if not args.dry_run and args.spartn_from_pmp:
        n = build_spartn_from_pmp(tr, dir_pmp, dir_spartn, key_hex=(args.pp_key_hex or None))
        print(f"SPARTN: built from {n} pmp file(s)")


if __name__ == "__main__":
    main()
