#!/usr/bin/env python3

import argparse
import os
from pathlib import Path


def convert_log_to_rtcm3(input_log_path: str, output_rtcm3_path: str) -> dict:
    messages_written = 0
    bytes_written = 0
    skipped_lines = 0

    input_path = Path(input_log_path)
    output_path = Path(output_rtcm3_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with input_path.open('r') as fin, output_path.open('wb') as fout:
        for line in fin:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                parts = line.split(maxsplit=4)
                if len(parts) < 5:
                    skipped_lines += 1
                    continue
                # timestamp_ms, msg_type, sat_prn, length, hex_data
                length_str = parts[3]
                hex_data = parts[4]

                msg_bytes = bytes.fromhex(hex_data)
                # Optional: Validate declared length vs actual
                try:
                    declared_len = int(length_str)
                    if declared_len != len(msg_bytes):
                        # Tolerate mismatch, but count as skipped if clearly corrupted
                        # Heuristic: RTCM messages must start with 0xD3 and be >= 6 bytes
                        if not (len(msg_bytes) >= 6 and msg_bytes[0] == 0xD3):
                            skipped_lines += 1
                            continue
                except Exception:
                    pass

                fout.write(msg_bytes)
                messages_written += 1
                bytes_written += len(msg_bytes)
            except Exception:
                skipped_lines += 1
                continue

    return {
        'messages_written': messages_written,
        'bytes_written': bytes_written,
        'skipped_lines': skipped_lines,
        'output_path': str(output_path),
    }


def main():
    parser = argparse.ArgumentParser(description='Convert RTCM .log (hex lines) to binary .rtcm3 file')
    parser.add_argument('--input', required=True, help='Path to input .log file')
    parser.add_argument('--output', help='Path to output .rtcm3 file (default: same name with .rtcm3)')
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        raise SystemExit(f"Input file does not exist: {input_path}")

    output_path = Path(args.output) if args.output else input_path.with_suffix('.rtcm3')

    stats = convert_log_to_rtcm3(str(input_path), str(output_path))
    print(f"Wrote {stats['messages_written']} messages, {stats['bytes_written']} bytes → {stats['output_path']}")
    if stats['skipped_lines']:
        print(f"Skipped {stats['skipped_lines']} non-message lines")


if __name__ == '__main__':
    main()


