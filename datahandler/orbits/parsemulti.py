import os
import sys
import json
import datetime as dt
import importlib.util
from types import ModuleType
from typing import Any, Dict, List, Tuple


CURRENT_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, os.pardir))
RTCM_DEMO_DIR = os.path.join(REPO_ROOT, "RTCM-SSR-Python-Demonstrator")


def _try_import_rtcm_decoder_from(dir_path: str) -> ModuleType | None:
    """Try to import rtcm_decoder from a specific directory by path.

    Ensures the directory is on sys.path so sibling imports like
    'import coord_and_time_transformations' resolve.
    """
    if not os.path.isdir(dir_path):
        return None
    decoder_path = os.path.join(dir_path, "rtcm_decoder.py")
    if not os.path.exists(decoder_path):
        return None
    if dir_path not in sys.path:
        sys.path.insert(0, dir_path)
    # Clean any prior failed import
    if "rtcm_decoder" in sys.modules:
        del sys.modules["rtcm_decoder"]
    try:
        spec = importlib.util.spec_from_file_location("rtcm_decoder", decoder_path)
        if spec is None or spec.loader is None:
            return None
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        sys.modules["rtcm_decoder"] = mod
        return mod
    except Exception:
        return None

# Numpy compatibility for external decoder on NumPy >= 2.0
try:
    import numpy as np  # type: ignore
    if not hasattr(np, "int"):
        np.int = int  # type: ignore[attr-defined]
    if not hasattr(np, "float"):
        np.float = float  # type: ignore[attr-defined]
    if not hasattr(np, "bool"):
        np.bool = bool  # type: ignore[attr-defined]
    if not hasattr(np, "str"):
        np.str = str  # type: ignore[attr-defined]
except Exception:
    pass

# Prefer decoders in this order:
# 1) orbits/rtcm_decoder.py
# 2) orbits/RTCM-SSR-Python-Demonstrator/rtcm_decoder.py
# 3) <repo_root>/RTCM-SSR-Python-Demonstrator on sys.path
rtcm_decoder = _try_import_rtcm_decoder_from(CURRENT_DIR)
if rtcm_decoder is None:
    rtcm_decoder = _try_import_rtcm_decoder_from(os.path.join(CURRENT_DIR, "RTCM-SSR-Python-Demonstrator"))
if rtcm_decoder is None:
    if RTCM_DEMO_DIR not in sys.path:
        sys.path.append(RTCM_DEMO_DIR)
    try:
        import rtcm_decoder  # type: ignore  # noqa: E402
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            f"Failed to import rtcm_decoder from local (orbits/) or {RTCM_DEMO_DIR}: {exc}"
        )


TARGET_TYPES = {1019, 1020, 1042, 1046}


def parse_generated_timestamp(header_lines: List[str]) -> Tuple[int, int]:
    """
    Extract (year, day_of_year) from a header line like:
    '# Generated: 2025-08-15T13:33:36.268116'
    """
    year = dt.datetime.utcnow().year
    doy = int(dt.datetime.utcnow().strftime("%j"))
    for line in header_lines:
        if line.startswith("# Generated:"):
            try:
                iso_str = line.split("Generated:", 1)[1].strip()
                # handle trailing ' UTC' suffix if present
                if iso_str.endswith(" UTC"):
                    iso_str = iso_str[:-4]
                ts = dt.datetime.fromisoformat(iso_str)
                year = ts.year
                doy = int(ts.strftime("%j"))
                break
            except Exception:
                # Fallback to current UTC if parsing fails
                pass
    return year, doy


def to_serializable(value: Any) -> Any:
    """Convert decoder values (including numpy types) into JSON-safe types."""
    # Lazy import to avoid hard dependency if not needed
    try:
        import numpy as np  # type: ignore
        np_types = (np.generic,)
        np_array = np.ndarray
    except Exception:  # pragma: no cover
        np_types = tuple()
        np_array = tuple()  # type: ignore

    if isinstance(value, dict):
        return {k: to_serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_serializable(v) for v in value]
    if np_types and isinstance(value, np_types):
        return value.item()
    if np_array and isinstance(value, np_array):
        return value.tolist()
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    if hasattr(value, "__dict__"):
        return {k: to_serializable(v) for k, v in vars(value).items() if not k.startswith("_")}
    # Basic JSON types pass-through
    return value


def decode_rtcm_payload(payload: bytes, payload_len: int, year: int, doy: int) -> Dict[str, Any]:
    decoder = rtcm_decoder.rtcm_decoder(payload, payload_len, year, doy)
    msg = decoder.dec_msg
    if msg is None:
        return {}
    data: Dict[str, Any] = to_serializable(msg)
    data["_msg_type"] = decoder.msg_type
    data["_data_length_bytes"] = payload_len
    return data


def parse_log_line(line: str) -> Tuple[str, int, int, int, bytes]:
    """
    Parse one data line from the log.
    Format: [timestamp_ms] [msg_type] [sat_prn] [length] [hex_data]
    Returns: (timestamp_token, msg_type, sat_prn, payload_len, payload_bytes)
    """
    parts = line.strip().split()
    if len(parts) < 5:
        raise ValueError("Malformed data line: not enough fields")

    timestamp_token = parts[0]
    msg_type = int(parts[1])
    sat_prn = int(parts[2])
    total_len = int(parts[3])
    hex_data = parts[4]

    frame = bytes.fromhex(hex_data)
    # Frame layout: 1 byte preamble + 2 bytes length + N payload bytes + 3 bytes CRC
    payload_len = total_len - 6
    if payload_len < 0 or len(frame) < 3 + payload_len:
        raise ValueError("Invalid frame or length")
    payload = frame[3 : 3 + payload_len]
    return timestamp_token, msg_type, sat_prn, payload_len, payload


def default_output_for(input_path: str) -> str:
    base = os.path.basename(input_path)
    name, _ = os.path.splitext(base)
    # Try to transform leading 'rtcm_' to 'eph_'
    if name.startswith("rtcm_"):
        name = "eph_" + name[len("rtcm_"):]
    else:
        name = f"eph_{name}"
    # Save alongside the input (e.g., in the orbits folder)
    out_dir = os.path.dirname(input_path)
    return os.path.join(out_dir, f"{name}.json")


def resolve_output_path(input_path: str, argv: List[str]) -> str:
    """Resolve output JSON path from CLI args if provided.

    Supported overrides:
      --out <file.json>    explicit output file path
      --outdir <dir>       directory to place default-named output
    """
    out_path = None
    out_dir = None
    i = 2
    while i < len(argv):
        if argv[i] == "--out" and i + 1 < len(argv):
            out_path = argv[i + 1]
            i += 2
        elif argv[i] == "--outdir" and i + 1 < len(argv):
            out_dir = argv[i + 1]
            i += 2
        else:
            i += 1
    if out_path:
        # Make absolute relative to CWD
        if not os.path.isabs(out_path):
            out_path = os.path.abspath(out_path)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        return out_path
    if out_dir:
        # Use default name within provided directory
        name_default = os.path.basename(default_output_for(input_path))
        if not os.path.isabs(out_dir):
            out_dir = os.path.abspath(out_dir)
        os.makedirs(out_dir, exist_ok=True)
        return os.path.join(out_dir, name_default)
    return default_output_for(input_path)


def main(argv: List[str]) -> None:
    if len(argv) > 1:
        in_path = argv[1]
    else:
        in_path = os.path.join(CURRENT_DIR, "rtcm_20250815_133336.log")

    if not os.path.isabs(in_path):
        in_path = os.path.abspath(in_path)

    if not os.path.exists(in_path):
        raise FileNotFoundError(f"Input file not found: {in_path}")

    out_path = resolve_output_path(in_path, argv)
    parse_log_to_json(in_path, out_path)
    print(out_path)


def parse_log_to_json(in_path: str, out_path: str) -> None:
    """Decode an RTCM ephemeris log to JSON at out_path.

    This is the importable function version of the CLI entrypoint.
    """
    if not os.path.isabs(in_path):
        in_path = os.path.abspath(in_path)
    if not os.path.exists(in_path):
        raise FileNotFoundError(f"Input file not found: {in_path}")

    # Collect header lines to determine year/day-of-year
    header_lines: List[str] = []
    data_lines: List[str] = []
    with open(in_path, "r", encoding="utf-8") as f:
        for raw in f:
            if not raw.strip():
                continue
            if raw.lstrip().startswith("#"):
                header_lines.append(raw.strip())
                continue
            data_lines.append(raw.strip())

    year, doy = parse_generated_timestamp(header_lines)

    result: Dict[str, List[Dict[str, Any]]] = {str(t): [] for t in sorted(TARGET_TYPES)}
    errors: List[Dict[str, Any]] = []

    for line in data_lines:
        try:
            timestamp_token, msg_type, sat_prn, payload_len, payload = parse_log_line(line)
            if msg_type not in TARGET_TYPES:
                continue
            decoded = decode_rtcm_payload(payload, payload_len, year, doy)
            if not decoded:
                continue
            entry: Dict[str, Any] = {
                "timestamp": timestamp_token,
                "msg_type": msg_type,
                "sat_prn": sat_prn,
                "decoded": decoded,
            }
            result[str(msg_type)].append(entry)
        except Exception as exc:
            errors.append({"line": line[:256], "error": str(exc)})

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    payload: Dict[str, Any] = {
        "source_file": in_path,
        "generated_utc": dt.datetime.utcnow().isoformat(timespec="seconds"),
        "year": year,
        "day_of_year": doy,
        "messages": result,
    }
    if errors:
        payload["errors"] = errors

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=False)


if __name__ == "__main__":  # pragma: no cover
    main(sys.argv)

