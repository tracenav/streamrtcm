#!/usr/bin/env python3
"""
PMP → SPARTN line logger.

Reads UBX-RXM-PMP binary files or a directory of PMP chunks, OR existing
SPARTN line-oriented logs, extracts/decrypts SPARTN frames using
decode_spartn_from_pmp, and writes a rotating text log in the same format
used by goodlogger.py:

  # Format: [timestamp_ms] [type] [subtype] [length] [hex_data]

Timestamp origin resets per output file. Rotation interval is configurable
via --rotate-seconds (default 300s = 5 minutes).

If frames are encrypted and a key is provided, decryption will be attempted
using the decoder's IV construction and time resolver. Encrypted frames that
cannot be decrypted are still logged as-is (hex), with the same header fields.
"""

import argparse
import os
import sys
import time
import glob
import re
from pathlib import Path
from datetime import datetime, timezone

# Ensure repo root on path for decode_spartn_from_pmp import
try:
    sys.path.append(str(Path(__file__).resolve().parents[1]))
except Exception:
    pass


from decode_spartn_from_pmp import (
	ubx_parse_rxm_pmp,
	extract_spartn_frames,
	parse_spartn_frame,
	build_iv_ae_ctr,
	aes_ctr_decrypt_openssl,
    build_plain_spartn_frame,
    TimeResolver,
    compute_message_crc,
)
try:
    # Optional import of CRC length mapping
    from decode_spartn_from_pmp import CRC_TYPE_TO_BYTES  # type: ignore
except Exception:
    CRC_TYPE_TO_BYTES = {0: 1, 1: 2, 2: 3, 3: 4}


def iter_input_bytes(paths):
	for p in paths:
		fp = Path(p)
		if fp.is_dir():
			for f in sorted(fp.glob('*.bin')):
				with open(f, 'rb') as fh:
					yield fh.read(), f
		else:
			with open(fp, 'rb') as fh:
				yield fh.read(), fp


def _iter_log_frames(log_path: Path):
	with open(log_path, 'r', encoding='utf-8', errors='ignore') as fh:
		for line in fh:
			line = line.strip()
			if not line or line.startswith('#'):
				continue
			parts = line.split()
			if len(parts) < 5:
				continue
			# accept both "123" and "123:0"
			ts_ok = parts[0].isdigit() or re.match(r'^\d+:\d+$', parts[0]) is not None
			if not ts_ok:
				continue
			hex_str = parts[4]
			if re.fullmatch(r'[0-9a-fA-F]+', hex_str) is None:
				continue
			try:
				frame = bytes.fromhex(hex_str)
				yield frame
			except Exception:
				continue


_PMP_TS_RE = re.compile(r"PMP_(\d{8})_(\d{6})_(\d{3})\.bin$", re.IGNORECASE)


def _parse_pmp_timestamp_ms(path: Path) -> int | None:
	"""Parse PMP_YYYYMMDD_HHMMSS_mmm.bin -> epoch ms (UTC)."""
	m = _PMP_TS_RE.search(path.name)
	if not m:
		return None
	ymd = m.group(1)
	hms = m.group(2)
	ms = int(m.group(3))
	try:
		dt = datetime.strptime(ymd + hms, "%Y%m%d%H%M%S").replace(tzinfo=timezone.utc)
		return int(dt.timestamp() * 1000) + ms
	except Exception:
		return None


def main():
	ap = argparse.ArgumentParser(description='PMP → SPARTN line logger (rotating files)')
	ap.add_argument('inputs', nargs='+', help='Input(s): PMP .bin, directory of .bin, or SPARTN .log')
	ap.add_argument('--outdir', default='spartn_pmp_logs', help='Output directory for text logs')
	ap.add_argument('--rotate-seconds', type=int, default=300, help='Seconds per output file (default 300)')
	ap.add_argument('--ms-per-frame', type=int, default=1, help='If >0, use a synthetic monotonic timestamp that increments by this many ms per frame (default 1). If 0, use wall-clock processing time.')
	ap.add_argument('--key-hex', default=os.getenv('PP_AES_KEY', 'd611b057b1f922f8726627d597ddd171'), help='AES-128 key hex for SPARTN decryption (env PP_AES_KEY overrides)')
	ap.add_argument('--debug', action='store_true', help='Emit debug reasons when encrypted frames are not decrypted')
	ap.add_argument('--single-log', default=None, help='Write all output into this single .log file (no rotation)')
	args = ap.parse_args()

	outdir = Path(args.outdir)
	outdir.mkdir(parents=True, exist_ok=True)

	def open_new_log(start_epoch_ms: int | None = None):
		# If single-log specified, always use that path and do not rotate/rename
		if args.single_log:
			path = Path(args.single_log)
			now_iso = time.strftime('%Y-%m-%dT%H:%M:%S UTC', time.gmtime()) if start_epoch_ms is None else time.strftime('%Y-%m-%dT%H:%M:%S UTC', time.gmtime(start_epoch_ms/1000))
			f = open(path, 'w')
		else:
			if start_epoch_ms is not None:
				dt = time.gmtime(start_epoch_ms / 1000)
				ts = time.strftime('%Y%m%d_%H%M%S', dt)
				now_iso = time.strftime('%Y-%m-%dT%H:%M:%S UTC', dt)
			else:
				ts = time.strftime('%Y%m%d_%H%M%S')
				now_iso = time.strftime('%Y-%m-%dT%H:%M:%S UTC', time.gmtime())
			path = outdir / f'spartn_{ts}.log'
			f = open(path, 'w')
		f.write('# SPARTN Message Log\n')
		f.write(f'# Generated: {now_iso}\n')
		f.write('# Mountpoint: PMP\n')
		f.write('# GPS Week: 0\n')
		f.write('# GPS TOW: 0ms\n')
		f.write('# Format: [timestamp_ms] [type] [subtype] [length] [hex_data]\n')
		f.write('# ----------------------------------------\n')
		f.flush()
		return f

	logf = open_new_log()
	# Track if anything was written to the current file so we can delete an
	# initially created placeholder if we later re-open with a timestamped name.
	wrote_any = False
	start_time = time.time()
	last_rotate = start_time
	frame_ms = 0

	# Iterate each input individually to support mixed types
	# Resolver for 16-bit half-day time tags across the whole run
	resolver = TimeResolver()

	for inp in args.inputs:
		fp = Path(inp)
		# Choose frame source
		if fp.suffix.lower() == '.log':
			frames_iter = ((frm, None) for frm in _iter_log_frames(fp))
		else:
			# Build binary streams from PMP or raw SPARTN
			streams: list[tuple[bytes, int | None]] = []
			file_epochs: list[int] = []
			# Support single file or directory
			if fp.is_dir():
				for bf in sorted(fp.glob('*.bin')):
					epoch = _parse_pmp_timestamp_ms(bf)
					with open(bf, 'rb') as fh:
						data = fh.read()
					user = ubx_parse_rxm_pmp(data)
					stream = user if user is not None and len(user) > 0 else data
					streams.append((stream, epoch))
					if epoch is not None:
						file_epochs.append(epoch)
			else:
				for data, src in iter_input_bytes([inp]):
					epoch = _parse_pmp_timestamp_ms(src)
					user = ubx_parse_rxm_pmp(data)
					stream = user if user is not None and len(user) > 0 else data
					streams.append((stream, epoch))
					if epoch is not None:
						file_epochs.append(epoch)
			base_epoch_ms = min(file_epochs) if file_epochs else None
			# If we have an absolute base, rename file start to match (unless single-log)
			if base_epoch_ms is not None and not args.single_log:
				# If the current file is just the initial placeholder and nothing
				# was written yet, remove it to avoid an empty extra file.
				old_name = logf.name
				logf.close()
				if not wrote_any:
					try:
						os.remove(old_name)
					except Exception:
						pass
				logf = open_new_log(base_epoch_ms)
				wrote_any = False
				current_file_start_ms = 0
			# Cross-stream scanner with rolling carry buffer so frames that span
			# adjacent PMP userData chunks are not lost.
			def _scan_streams_carry():
				# Conservative maximum SPARTN frame size calculation:
				# payload_len up to 1023 + embedded_auth up to 64 + CRC up to 4 + header ~ 12
				MAX_FRAME_BYTES = 1200
				# Keep a rolling window at least 2x the maximum frame size as requested
				CARRY_WINDOW = MAX_FRAME_BYTES * 2  # 2400 bytes
				carry_bytes = b""
				# Per-byte epoch map aligned with carry_bytes
				carry_epochs: list[int | None] = []
				# Buffer to accumulate undecoded non-padding runs that are still within carry window
				pending_runs: list[tuple[int, bytes]] = []  # list of (epoch_ms_or_neg1, bytes)
				# Buffer to accumulate CRC-failed candidate frames for deferred emission
				pending_crc: list[tuple[int, bytes]] = []
				pending_crc_seen: set[str] = set()
				for stream, epoch in streams:
					if not stream:
						continue
					# Build combined buffer and epoch map
					buf = carry_bytes + stream
					buf_epochs = carry_epochs + [epoch] * len(stream)
					prev_len = len(carry_bytes)
					# Start scanning a little before the boundary to catch frames that began in carry
					i = max(0, prev_len - MAX_FRAME_BYTES)
					# Track frame ranges found in this combined buffer for coverage analysis
					found_ranges: list[tuple[int, int]] = []
					while i < len(buf):
						j = buf.find(b'\x73', i)
						if j < 0:
							break
						parsed = parse_spartn_frame(buf, j)
						if parsed is None:
							i = j + 1
							continue
						rec, next_i, _payload_bytes, _auth = parsed
						if next_i <= j:
							i = j + 1
							continue
						# Validate CRC before accepting this candidate; if CRC fails, do not consume length
						crc_ok = True
						try:
							crc_type = int(rec.get('crc_type', 0))
							crc_n = CRC_TYPE_TO_BYTES.get(crc_type, 0)
							if crc_n > 0 and (j + 1 + crc_n) <= next_i:
								body = buf[j + 1: next_i - crc_n]
								expected = buf[next_i - crc_n: next_i]
								calc = compute_message_crc(crc_type, body)
								crc_ok = (calc == expected)
						except Exception:
							crc_ok = False
						if not crc_ok:
							# Save CRC-failed candidate for later debug emission (-2) once out of window
							if args.debug:
								cand = buf[j:next_i]
								run_epoch = buf_epochs[j] if 0 <= j < len(buf_epochs) else None
								cand_ts = int(run_epoch - base_epoch_ms) if (base_epoch_ms is not None and run_epoch is not None) else -1
								hex_id = cand[:16].hex() + f"_{len(cand)}_{cand_ts}"
								if hex_id not in pending_crc_seen:
									pending_crc.append((cand_ts, cand))
									pending_crc_seen.add(hex_id)
							# Advance by one to try the next possible preamble; do not skip over data
							i = j + 1
							continue
						# Avoid emitting frames that were entirely contained in previous carry
						if next_i <= prev_len:
							i = next_i
							continue
						# Record frame coverage
						found_ranges.append((j, next_i))
						# Attribute frame timestamp to the epoch corresponding to the preamble byte
						preamble_epoch = None
						if 0 <= j < len(buf_epochs):
							preamble_epoch = buf_epochs[j]
						yield buf[j:next_i], preamble_epoch
						i = next_i
					# After scanning this buffer, analyze undecoded bytes in the new segment
					if args.debug and len(buf) > prev_len:
						nonlocal frame_ms, wrote_any, current_file_start_ms, logf
						seg_len = len(buf) - prev_len
						seg_cov = bytearray(seg_len)
						for a, b in found_ranges:
							# overlap with new segment
							lo = max(a, prev_len)
							hi = min(b, len(buf))
							if hi > lo:
								for k in range(lo, hi):
									seg_cov[k - prev_len] = 1
						# Known filler bytes (observed 0x13 in captures)
						FILLER_BYTES = {0x13}
						# Collect contiguous runs of undecoded non-padding bytes into pending_runs.
						idx = 0
						while idx < seg_len:
							if seg_cov[idx] == 0 and buf[prev_len + idx] not in FILLER_BYTES:
								start = idx
								while idx < seg_len and seg_cov[idx] == 0 and buf[prev_len + idx] not in FILLER_BYTES:
									idx += 1
								run_bytes = buf[prev_len + start: prev_len + idx]
								# Store with epoch for later emission once it falls out of carry window
								run_epoch = buf_epochs[prev_len + start] if (prev_len + start) < len(buf_epochs) else None
								pend_ts = int(run_epoch - base_epoch_ms) if (base_epoch_ms is not None and run_epoch is not None) else -1
								pending_runs.append((pend_ts, run_bytes))
							else:
								idx += 1
						# Also print a brief stderr summary for visibility
						# Count total non-padding undecoded bytes for this segment
						cnt = sum(1 for i2 in range(seg_len) if seg_cov[i2] == 0 and buf[prev_len + i2] not in FILLER_BYTES)
						if cnt:
							# sample from the start of segment
							sample = bytes(buf[prev_len + i2] for i2 in range(seg_len) if seg_cov[i2] == 0 and buf[prev_len + i2] not in FILLER_BYTES)[:64].hex()
							sys.stderr.write(f"Undecoded non-padding bytes: count={cnt} epoch={epoch} sample_hex={sample}\n")
					# Before updating carry, emit any pending runs that have fully fallen out of the new carry window
					if args.debug and (pending_runs or pending_crc):
						# Everything strictly before len(buf) - CARRY_WINDOW is outside window now
						cutoff_index = max(0, len(buf) - CARRY_WINDOW)
						# We approximate by emitting all pending runs when prev_len < cutoff_index,
						# i.e., runs from the older segment no longer in carry
						if prev_len <= cutoff_index:
							for ts_ms, run_bytes in pending_runs:
								# If no absolute ts, use synthetic ts
								if ts_ms < 0:
									ts_ms = frame_ms
									frame_ms += args.ms_per_frame
								# Handle rotation similar to valid frames
								if base_epoch_ms is not None and (ts_ms - current_file_start_ms) >= args.rotate_seconds * 1000:
									logf.close()
									logf = open_new_log(base_epoch_ms + ts_ms)
									wrote_any = False
									current_file_start_ms = ts_ms
								line = f"{ts_ms} -1 -1 {len(run_bytes)} {run_bytes.hex()}\n"
								logf.write(line)
								wrote_any = True
							# Emit pending CRC-failed candidates as -2 lines
							for ts_ms, cand in pending_crc:
								if ts_ms < 0:
									ts_ms = frame_ms
									frame_ms += args.ms_per_frame
								if base_epoch_ms is not None and (ts_ms - current_file_start_ms) >= args.rotate_seconds * 1000:
									logf.close()
									logf = open_new_log(base_epoch_ms + ts_ms)
									wrote_any = False
									current_file_start_ms = ts_ms
								line = f"{ts_ms} -2 -2 {len(cand)} {cand.hex()}\n"
								logf.write(line)
								wrote_any = True
							# Clear pending after emission
							pending_runs.clear()
							pending_crc.clear()

					# Update carry window to last CARRY_WINDOW bytes
					if len(buf) > CARRY_WINDOW:
						carry_bytes = buf[-CARRY_WINDOW:]
						carry_epochs = buf_epochs[-CARRY_WINDOW:]
					else:
						carry_bytes = buf
						carry_epochs = buf_epochs
			frames_iter = _scan_streams_carry()

		# Consume frames
		for frame, pmp_epoch in frames_iter:
			parsed = parse_spartn_frame(frame, 0)
			if parsed is None:
				continue
			rec, _next_i, payload_bytes, _auth = parsed
			# CRC validation: compute over TF002.. with selected width and compare to trailing CRC bytes
			crc_ok = True
			try:
				crc_type = int(rec.get('crc_type', 0))
				crc_n = CRC_TYPE_TO_BYTES.get(crc_type, 0)
				if crc_n > 0 and len(frame) > 1 + crc_n:
					body = frame[1:-crc_n]
					expected = frame[-crc_n:]
					calc = compute_message_crc(crc_type, body)
					crc_ok = (calc == expected)
			except Exception:
				crc_ok = False

			# Maintain time resolver state
			if rec.get('time_tag_type', 0) == 1:
				try:
					resolver.note_full(int(rec.get('time_tag', 0)))
				except Exception:
					pass

			# Timestamp ms relative to earliest PMP epoch if available
			if base_epoch_ms is not None and pmp_epoch is not None:
				ts_ms = int(pmp_epoch - base_epoch_ms)
			else:
				ts_ms = frame_ms
				frame_ms += args.ms_per_frame

			# Rotate based on absolute ms (disabled for single-log)
			if (not args.single_log) and base_epoch_ms is not None and (ts_ms - current_file_start_ms) >= args.rotate_seconds * 1000:
				logf.close()
				logf = open_new_log(base_epoch_ms + ts_ms)
				wrote_any = False
				current_file_start_ms = ts_ms
			type_field = int(rec.get('type', 0))
			subtype = int(rec.get('subtype', 0))
			# Attempt payload decryption and rebuild plaintext frame
			frame_out = frame
			if rec.get('eaf', 0) == 1 and payload_bytes:
				try:
					rec_for_iv = dict(rec)
					# If only half-day time tag, try to resolve using last full
					if rec_for_iv.get('time_tag_type', 0) == 0:
						resolved = resolver.resolve_halfday(int(rec_for_iv.get('time_tag', 0)))
						if resolved is not None:
							rec_for_iv['time_tag_type'] = 1
							rec_for_iv['time_tag'] = int(resolved)
					iv = build_iv_ae_ctr(rec_for_iv)
					plain = aes_ctr_decrypt_openssl(args.key_hex, iv, payload_bytes)
					# Rebuild plaintext frame using ORIGINAL header fields to preserve
					# on-wire time tag width, but with encryption flag cleared.
					rec_plain_hdr = dict(rec)
					rec_plain_hdr['eaf'] = 0
					frame_out = build_plain_spartn_frame(rec_plain_hdr, plain)
				except Exception as e:
					if args.debug:
						sys.stderr.write(f"Decrypt failed (type {type_field}-{subtype}) time_tag_type={rec.get('time_tag_type')} time_tag={rec.get('time_tag')} reason={str(e)}\n")
					frame_out = frame
			# Log plain ms and the frame (decrypted if possible); only emit valid CRC frames
			if not crc_ok and args.debug:
				# In debug, still emit as invalid marker to see it, rather than as a normal frame
				line = f"{ts_ms} -2 -2 {len(frame)} {frame.hex()}\n"
			else:
				line = f"{ts_ms} {type_field} {subtype} {len(frame_out)} {frame_out.hex()}\n"
			logf.write(line)
			wrote_any = True

	logf.close()


if __name__ == '__main__':
	main()
