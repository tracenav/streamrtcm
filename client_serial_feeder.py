#!/usr/bin/env python3

import argparse
import datetime
import serial  # pyserial
import sys
import time
import re
import os

def build_gga(lat_deg: float, lon_deg: float, alt_m: float, sats: int = 12, hdop: float = 1.0, dt: datetime.datetime = None) -> str:
  now = dt if dt else datetime.datetime.now(datetime.UTC)
  ts = now.strftime("%H%M%S.%f")[:-4]  # HHMMSS.SS

  def to_nmea_lat(v: float):
    hemi = 'N' if v >= 0 else 'S'
    av = abs(v)
    d = int(av)
    m = (av - d) * 60.0
    return f"{d:02d}{m:08.5f}", hemi

  def to_nmea_lon(v: float):
    hemi = 'E' if v >= 0 else 'W'
    av = abs(v)
    d = int(av)
    m = (av - d) * 60.0
    return f"{d:03d}{m:08.5f}", hemi

  lat_nmea, lat_h = to_nmea_lat(lat_deg)
  lon_nmea, lon_h = to_nmea_lon(lon_deg)

  base = (
    f"GPGGA,{ts},{lat_nmea},{lat_h},{lon_nmea},{lon_h},1,{sats},{hdop:.1f},{alt_m:.1f},M,0.0,M,,"
  )
  chk = 0
  for c in base:
    chk ^= ord(c)
  return f"${base}*{chk:02X}\r\n"


def build_zda(dt: datetime.datetime = None) -> str:
  now = dt if dt else datetime.datetime.now(datetime.UTC)
  ts = now.strftime("%H%M%S.%f")[:-4]
  base = f"GPZDA,{ts},{now.day:02d},{now.month:02d},{now.year:04d},00,00"
  chk = 0
  for c in base:
    chk ^= ord(c)
  return f"${base}*{chk:02X}\r\n"


def main():
  ap = argparse.ArgumentParser(description="Serial feeder for ESP32-S3 PPL minimal sketch")
  ap.add_argument('--serial', required=True, help='Serial port (e.g. /dev/tty.usbmodemXXXX)')
  ap.add_argument('--baud', type=int, default=115200)

  # Ephemeris from file (optional). If provided, overrides NTRIP ephemeris.
  ap.add_argument('--eph-file', default=None, help='Path to filtered ephemeris log file to replay')
  ap.add_argument('--eph-rate', type=float, default=0.2, help='Seconds between ephemeris frames when replaying file')
  ap.add_argument('--eph-loop', type=int, default=0, help='Seconds between repeating the full ephemeris file (0=one-shot)')
  # PMP/SPARTN from file (optional). If provided, overrides NTRIP SPARTN.
  ap.add_argument('--pmp-file', default=None, help='Path to PMP (UBX RXM-PMP or SPARTN hex) log to replay')
  ap.add_argument('--pmp-dir', default=None, help='Directory of UBX PMP .bin files to replay (timed by filename, e.g. PMP_YYYYMMDD_HHMMSS_mmm.bin)')
  ap.add_argument('--pmp-loop', action='store_true', help='Loop PMP directory replay when reaching the end')
  # SPARTN from latest.log (timestamped hex) overrides NTRIP/PMP when provided
  ap.add_argument('--spartn-file', default=None, help='Path to SPARTN latest.log to replay (timestamped hex)')
  # Log PPL RTCM output to a file in the same format as filtered ephemeris logs
  ap.add_argument('--rtcm-out-log', default=None, help='Path to write RTCM output log (append)')
  # RTCM filtered folder: choose file by virtual time and resend every cycle
  ap.add_argument('--rtcm-dir', default=None, help='Directory of filtered RTCM logs (e.g., rtcm_YYYYMMDD_HHMMSS_filtered.log)')
  ap.add_argument('--rtcm-cycle', type=int, default=60, help='Seconds between re-sending the current RTCM set (default 60)')
  # Dynamic key (optional) to push to ESP via !K frame
  ap.add_argument('--key-file', default=None, help='Path to file containing PPL dynamic key (first non-empty line used)')
  ap.add_argument('--key', default=None, help='Dynamic key string (UTF-8) to send via !K')

  # GGA position (Fredericksburg, VA)
  ap.add_argument('--gga-lat', type=float, default=38.3032)
  ap.add_argument('--gga-lon', type=float, default=-77.4605)
  ap.add_argument('--gga-alt', type=float, default=30.0)
  ap.add_argument('--no-zda', action='store_true', help='Disable ZDA time messages (only send GGA)')
  # Speed multiplier for replay timing
  ap.add_argument('--speed', type=float, default=1.0, help='Speed multiplier for replay (e.g., 2.0 for 2x)')

  args = ap.parse_args()

  # Support both device paths and pyserial URL handlers (e.g., socket://localhost:5555)
  try:
    ser = serial.serial_for_url(args.serial, args.baud, timeout=0)
  except Exception:
    ser = serial.Serial(args.serial, args.baud, timeout=0)
  print(f"Opened serial {args.serial} @ {args.baud}")

  # Prepare RTCM output log if requested
  rtcm_out_fh = None
  rtcm_out_header_written = False
  rtcm_out_base_ms = None  # absolute UTC ms at first RTCM out
  if args.rtcm_out_log:
    try:
      rtcm_out_fh = open(args.rtcm_out_log, 'a', buffering=1)
    except Exception as e:
      print(f"Failed to open --rtcm-out-log {args.rtcm_out_log}: {e}")
      rtcm_out_fh = None

  # Load dynamic key if provided
  key_bytes = None
  key_sent = False
  # --key takes precedence if provided
  if args.key:
    try:
      s = args.key.strip()
      key_bytes = s.encode('utf-8')
      print(f"Loaded dynamic key from --key ({len(key_bytes)} bytes)")
    except Exception as e:
      print(f"Failed to use --key: {e}")
  if args.key_file:
    try:
      with open(args.key_file, 'rb') as kf:
        content = kf.read()
      # Use first non-empty line if multiple
      try:
        first_line = content.splitlines()
        for ln in first_line:
          ln = ln.strip()
          if ln:
            # Try interpret as ASCII hex first, else raw bytes
            try:
              s = ln.decode('ascii').strip()
              if s.startswith('0x'):
                s = s[2:]
              if re.fullmatch(r'[0-9A-Fa-f]+', s) and (len(s) % 2 == 0):
                key_bytes = bytes.fromhex(s)
              else:
                key_bytes = ln
            except Exception:
              key_bytes = ln
            break
      except Exception:
        key_bytes = content.strip() or content
      if key_bytes:
        print(f"Loaded dynamic key from {args.key_file} ({len(key_bytes)} bytes)")
      else:
        print(f"Key file {args.key_file} contained no data; skipping key send")
    except Exception as e:
      print(f"Failed to read key file {args.key_file}: {e}")

  # Preload ephemeris frames from file if requested
  use_eph_file = False
  eph_file_frames = []
  if args.eph_file:
    try:
      with open(args.eph_file, 'r') as f:
        for line in f:
          line = line.strip()
          if not line or line.startswith('#'):
            continue
          # Expected: <ts>:0 <msg_type> <sat_prn> <length> <hex_data>
          try:
            parts = line.split()
            if len(parts) < 5:
              continue
            hex_data = parts[-1]
            frame_bytes = bytes.fromhex(hex_data)
            # Basic sanity: first byte should be 0xD3 and length matches
            if len(frame_bytes) >= 6 and frame_bytes[0] == 0xD3:
              eph_file_frames.append(frame_bytes)
          except Exception:
            continue
      if eph_file_frames:
        use_eph_file = True
        print(f"Loaded {len(eph_file_frames)} ephemeris frames from file: {args.eph_file}")
      else:
        print(f"No valid ephemeris frames found in {args.eph_file}; falling back to NTRIP")
    except Exception as e:
      print(f"Failed to read ephemeris file {args.eph_file}: {e}")

  # Preload PMP/SPARTN frames from file if requested
  use_pmp_file = False
  pmp_file_frames = []  # list of tuples (t_ms or None, bytes)
  use_pmp_dir = False
  pmp_dir_frames = []   # list of tuples (offset_ms, bytes, filename)
  # Track absolute base and date extracted from filenames for display purposes
  pmp_absolute_base_ms = None  # epoch ms if full date parsed
  pmp_base_tod_ms = None       # ms since midnight if no date
  pmp_date_str = None          # e.g., '0821' (MMDD)
  use_spartn_file = False
  spartn_file_frames = []  # list of tuples (t_ms, bytes)
  # RTCM filtered directory
  use_rtcm_dir = False
  rtcm_sets = []  # list of tuples (t_ms_epoch, [frames], filename)
  if args.pmp_file:
    try:
      with open(args.pmp_file, 'r') as f:
        for line in f:
          s = line.strip()
          if not s or s.startswith('#'):
            continue
          # Try format 1: "RX <len>B: <hex>..." (SPARTN direct, no timestamp)
          if s.startswith('RX') and ':' in s:
            try:
              hex_str = s.split(':', 1)[1].strip()
              hex_str = hex_str.replace(' ', '')
              data = bytes.fromhex(hex_str)
              pmp_file_frames.append((None, data))
              continue
            except Exception:
              pass
          # Try format 2: "<ts>:0 ... ... <len> <hex>" (we'll treat hex as UBX or SPARTN)
          try:
            parts = s.split()
            # ts may be like "12345:0" or "12345:"; extract digits before ':'
            ts_part = parts[0]
            t_ms = int(ts_part.split(':')[0])
            hex_str = parts[-1]
            # If hex starts with b562 (UBX), extract SPARTN user data from RXM-PMP
            raw = bytes.fromhex(hex_str)
            if len(raw) >= 8 and raw[0:2] == b'\xb5\x62':
              # UBX header: 0:B5 1:62 2:class 3:id 4-5:len 6.. payload
              # Expect class 0x02 (RXM), id 0x72 (PMP)
              # payload length little-endian at 4..5
              plen = raw[4] | (raw[5] << 8)
              if 6 + plen + 2 <= len(raw):
                payload = raw[6:6+plen]
                # In RXM-PMP, user data length at payload[2..3] (little-endian), user data starts at 24
                if len(payload) >= 26:
                  ulen = payload[2] | (payload[3] << 8)
                  start = 24
                  end = start + ulen
                  if end <= len(payload):
                    spartn = payload[start:end]
                    pmp_file_frames.append((t_ms, spartn))
                  else:
                    # Fallback: treat whole payload as SPARTN fragment
                    pmp_file_frames.append((t_ms, payload))
              else:
                # Fallback: treat as SPARTN
                pmp_file_frames.append((t_ms, raw))
            else:
              # treat as SPARTN chunk
              pmp_file_frames.append((t_ms, raw))
            continue
          except Exception:
            pass
      if pmp_file_frames:
        use_pmp_file = True
        print(f"Loaded {len(pmp_file_frames)} PMP/SPARTN frames from file: {args.pmp_file}")
      else:
        print(f"No valid PMP/SPARTN frames found in {args.pmp_file}; falling back to NTRIP SPARTN")
    except Exception as e:
      print(f"Failed to read PMP/SPARTN file {args.pmp_file}: {e}")

  # Load PMP frames from a directory of .bin files
  if args.pmp_dir:
    try:
      entries = sorted([os.path.join(args.pmp_dir, f) for f in os.listdir(args.pmp_dir) if f.lower().endswith('.bin')])
      ts_list = []  # (t_ms, path, ymd_or_None)
      for path in entries:
        name = os.path.basename(path)
        # Expect like PMP_YYYYMMDD_HHMMSS_mmm.bin or PMP_HHMMSS_mmm.bin (fallback)
        m = re.search(r'(\d{8})_(\d{6})_(\d{1,3})', name, re.IGNORECASE)
        if m:
          ymd, hms, ms = m.group(1), m.group(2), m.group(3)
          # Prefer absolute UTC time from date+time in filename
          try:
            yy = int(ymd[0:4]); mo = int(ymd[4:6]); dd = int(ymd[6:8])
            hh = int(hms[0:2]); mi = int(hms[2:4]); ss = int(hms[4:6]); mss = int(ms)
            dt_utc = datetime.datetime(yy, mo, dd, hh, mi, ss, mss * 1000, tzinfo=datetime.UTC)
            t_ms = int(dt_utc.timestamp() * 1000)
            ts_list.append((t_ms, path, ymd))
            continue
          except Exception:
            # Fallback: use time-of-day only
            hh = int(hms[0:2]); mi = int(hms[2:4]); ss = int(hms[4:6]); mss = int(ms)
            t_ms = ((hh*3600 + mi*60 + ss) * 1000) + mss
            ts_list.append((t_ms, path, None))
            continue
        else:
          m2 = re.search(r'(\d{6})_(\d{1,3})', name)
          if m2:
            hms, ms = m2.group(1), m2.group(2)
            hh = int(hms[0:2]); mi = int(hms[2:4]); ss = int(hms[4:6]); mss = int(ms)
            t_ms = ((hh*3600 + mi*60 + ss) * 1000) + mss
            ts_list.append((t_ms, path, None))
          else:
            # No time; skip
            continue
      if ts_list:
        ts_list.sort(key=lambda x: x[0])
        base, base_path, base_ymd = ts_list[0]
        # Track base time/date for display
        if base_ymd is not None:
          pmp_absolute_base_ms = base
          pmp_date_str = base_ymd[4:8]
        else:
          pmp_base_tod_ms = base
          pmp_date_str = None
        for t_ms, path, _ymd in ts_list:
          try:
            with open(path, 'rb') as f:
              raw = f.read()
            # If UBX, extract user data; else send as-is
            payload = raw
            if len(raw) >= 8 and raw[0:2] == b'\xb5\x62':
              plen = raw[4] | (raw[5] << 8)
              if 6 + plen + 2 <= len(raw):
                ubx_payload = raw[6:6+plen]
                if len(ubx_payload) >= 26:
                  ulen = ubx_payload[2] | (ubx_payload[3] << 8)
                  start = 24
                  end = start + ulen
                  if end <= len(ubx_payload):
                    payload = ubx_payload[start:end]
                  else:
                    payload = ubx_payload
            base_name = os.path.basename(path)
            pmp_dir_frames.append((t_ms - base, payload, base_name))
          except Exception:
            continue
        if pmp_dir_frames:
          use_pmp_dir = True
          print(f"Loaded {len(pmp_dir_frames)} PMP frames from dir: {args.pmp_dir}")
      else:
        print(f"No .bin PMP frames found in {args.pmp_dir}")
    except Exception as e:
      print(f"Failed to read PMP dir {args.pmp_dir}: {e}")

  # Load RTCM frames from a directory of filtered logs, keyed by filename timestamp
  if args.rtcm_dir:
    try:
      entries = sorted([os.path.join(args.rtcm_dir, f) for f in os.listdir(args.rtcm_dir) if f.lower().endswith('.log')])
      for path in entries:
        name = os.path.basename(path)
        # Expect rtcm_YYYYMMDD_HHMMSS_filtered.log (parse first YYYYMMDD_HHMMSS)
        m = re.search(r'(\d{8})_(\d{6})', name)
        if not m:
          continue
        ymd, hms = m.group(1), m.group(2)
        try:
          yy = int(ymd[0:4]); mo = int(ymd[4:6]); dd = int(ymd[6:8])
          hh = int(hms[0:2]); mi = int(hms[2:4]); ss = int(hms[4:6])
          dt_utc = datetime.datetime(yy, mo, dd, hh, mi, ss, tzinfo=datetime.UTC)
          t_ms_epoch = int(dt_utc.timestamp() * 1000)
        except Exception:
          continue
        # Load frames from file
        frames = []
        try:
          with open(path, 'r') as f:
            for line in f:
              s = line.strip()
              if not s or s.startswith('#'):
                continue
              parts = s.split()
              if len(parts) < 5:
                continue
              try:
                hex_data = parts[-1]
                frame_bytes = bytes.fromhex(hex_data)
                if len(frame_bytes) >= 6 and frame_bytes[0] == 0xD3:
                  frames.append(frame_bytes)
              except Exception:
                continue
        except Exception:
          continue
        if frames:
          rtcm_sets.append((t_ms_epoch, frames, name))
      if rtcm_sets:
        rtcm_sets.sort(key=lambda x: x[0])
        use_rtcm_dir = True
        print(f"Loaded {len(rtcm_sets)} RTCM sets from dir: {args.rtcm_dir}")
      else:
        print(f"No valid RTCM logs found in {args.rtcm_dir}")
    except Exception as e:
      print(f"Failed to read RTCM dir {args.rtcm_dir}: {e}")

  # Load SPARTN frames from latest.log style file
  if args.spartn_file:
    try:
      with open(args.spartn_file, 'r') as f:
        for line in f:
          s = line.strip()
          if not s or s.startswith('#'):
            continue
          parts = s.split()
          # Expect: [timestamp_ms] [type] [subtype] [length] [hex_data]
          if len(parts) < 5:
            continue
          try:
            t_ms = int(parts[0])
            length = int(parts[3])
            hex_data = parts[4]
            data = bytes.fromhex(hex_data)
            if length != len(data):
              # length may not include framing; still accept
              pass
            spartn_file_frames.append((t_ms, data))
          except Exception:
            continue
      if spartn_file_frames:
        use_spartn_file = True
        print(f"Loaded {len(spartn_file_frames)} SPARTN frames from file: {args.spartn_file}")
      else:
        print(f"No valid SPARTN frames found in {args.spartn_file}")
    except Exception as e:
      print(f"Failed to read SPARTN file {args.spartn_file}: {e}")

  # Ensure at least one input is provided
  if not (use_eph_file or use_pmp_file or use_pmp_dir or use_spartn_file):
    print("No inputs provided. Specify --eph-file and/or --spartn-file (or --pmp-*)")
    return

  # Sequential loops to keep script simple and minimal
  while True:
    try:
      # Ephemeris source: file only
      eph_index = 0
      last_eph_file_send = 0.0
      if use_eph_file:
        print(f"Using ephemeris from file: {args.eph_file}")

      last_gga = time.time()
      speed = args.speed if args.speed and args.speed > 0 else 1.0
      def gga_fn():
        return build_gga(args.gga_lat, args.gga_lon, args.gga_alt)

      last_nmea_virtual_sec = None
      # Virtual clock anchor (based on PMP base time, scaled by --speed)
      virt_anchor_wall = None          # wall-clock time when virtual clock was anchored
      virt_anchor_base_ms = None       # base epoch ms or ms since midnight
      virt_is_absolute = False         # True if base is absolute epoch ms
      # Gate GGA/SPARTN until first full ephemeris batch (if eph-file provided)
      eph_initial_batch_done = (not use_eph_file) or (len(eph_file_frames) == 0)
      pmp_start_time = None
      pmp_index = 0
      spartn_base_ts = None
      # Dynamic key multi-push control
      key_push_attempts = 0
      key_last_push = 0.0
      # RTCM-dir resend scheduler
      rtcm_current_idx = None  # index into rtcm_sets
      rtcm_send_index = 0
      rtcm_last_send = 0.0
      rtcm_last_cycle_id = None

      while True:
        # Send dynamic key up to 3 times at startup (spaced)
        if key_bytes and key_push_attempts < 3:
          if (time.time() - key_last_push) >= 0.5:
            try:
              ser.write(f"!K {len(key_bytes)}\n".encode('ascii'))
              ser.write(key_bytes)
              ser.write(b"\n")
              key_push_attempts += 1
              key_last_push = time.time()
              print(f"[ESP32] pushed K {len(key_bytes)} bytes (attempt {key_push_attempts}/3)")
              sys.stdout.flush()
            except Exception as e:
              print(f"Serial key send failed: {e}")
        # (no NTRIP GGA needed; only serial NMEA)

        # periodic NMEA (GGA + ZDA) to ESP32 for PPL — at most once per virtual UTC second
        try:
          # Initialize virtual clock anchor once, based on PMP base or SPARTN file
          if virt_anchor_wall is None and eph_initial_batch_done:
            if use_pmp_dir and (pmp_absolute_base_ms is not None or pmp_base_tod_ms is not None):
              virt_anchor_wall = time.time()
              if pmp_absolute_base_ms is not None:
                virt_anchor_base_ms = pmp_absolute_base_ms
                virt_is_absolute = True
              else:
                virt_anchor_base_ms = pmp_base_tod_ms or 0
                virt_is_absolute = False
            elif use_spartn_file and spartn_file_frames:
              # Initialize from SPARTN file timestamps
              virt_anchor_wall = time.time()
              virt_anchor_base_ms = spartn_file_frames[0][0]  # First SPARTN timestamp (unix_ms)
              virt_is_absolute = True

          disp_mmdd = None
          disp_hms = None
          virt_sec_id = None
          if virt_anchor_wall is not None and virt_anchor_base_ms is not None:
            elapsed_ms = int((time.time() - virt_anchor_wall) * 1000.0 * (speed if speed > 0 else 1.0))
            if virt_is_absolute:
              t_abs_ms = virt_anchor_base_ms + elapsed_ms
              virt_sec_id = t_abs_ms // 1000
              dt = datetime.datetime.fromtimestamp(t_abs_ms / 1000.0, tz=datetime.UTC)
              disp_mmdd = dt.strftime('%m%d')
              disp_hms = dt.strftime('%H:%M:%S')
            else:
              combined = virt_anchor_base_ms + elapsed_ms
              sec = (combined // 1000) % (24*3600)
              virt_sec_id = sec  # day-relative seconds
              hh = sec // 3600
              mi = (sec // 60) % 60
              ss = sec % 60
              disp_hms = f"{hh:02d}:{mi:02d}:{ss:02d}"
              disp_mmdd = pmp_date_str or datetime.datetime.now(datetime.UTC).strftime('%m%d')
          else:
            # Fallback: real time
            now_dt = datetime.datetime.now(datetime.UTC)
            virt_sec_id = int(now_dt.timestamp())
            disp_mmdd = now_dt.strftime('%m%d')
            disp_hms = now_dt.strftime('%H:%M:%S')

          # Gate NMEA if ephemeris initial batch not yet completed
          if not eph_initial_batch_done:
            pass
          elif virt_sec_id != last_nmea_virtual_sec:
            # Convert virt_sec_id to datetime for NMEA generation
            virt_dt = datetime.datetime.fromtimestamp(virt_sec_id, tz=datetime.UTC)
            gga_line = build_gga(args.gga_lat, args.gga_lon, args.gga_alt, dt=virt_dt)
            ser.write(gga_line.encode('ascii'))
            if not args.no_zda:
              zda_line = build_zda(dt=virt_dt)
              ser.write(zda_line.encode('ascii'))
            msg_type = "GGA+ZDA" if not args.no_zda else "GGA"
            print(f"[ESP32] NMEA sent: {msg_type} ({disp_mmdd}, {disp_hms} UTC, {args.gga_lat:.5f}, {args.gga_lon:.5f})")
            sys.stdout.flush()
            last_nmea_virtual_sec = virt_sec_id
        except Exception as e:
          print(f"Serial NMEA send failed: {e}")

        # Read any output coming back from the ESP32 and print it
        try:
          # Maintain an input buffer for framed data from ESP32
          if 'esp_in_buf' not in locals():
            esp_in_buf = bytearray()
            start_time = time.time()

          # Helpers for RTCM parsing
          def extract_msg_type(payload: bytes) -> int:
            if len(payload) >= 2:
              return ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF
            return 0

          def extract_sat_prn(msg_type: int, payload: bytes) -> int:
            # For requested PPL types, PRN is not relevant; return 0
            return 0

          def log_rtcm_messages(rtcm_blob: bytes):
            # Walk blob: find 0xD3 frames and log in rtcm_logger style
            idx = 0
            def is_allowed(mtype: int) -> bool:
              if mtype in (1005, 1033, 1230):
                return True
              # Any MSMx for GPS/GLO/GAL/BDS (x = 1..7)
              if 1070 <= mtype <= 1077:  # GPS
                return True
              if 1080 <= mtype <= 1087:  # GLONASS
                return True
              if 1090 <= mtype <= 1097:  # Galileo
                return True
              if 1120 <= mtype <= 1127:  # BeiDou
                return True
              return False
            while idx + 6 <= len(rtcm_blob):
              if rtcm_blob[idx] != 0xD3:
                idx += 1
                continue
              if idx + 3 > len(rtcm_blob):
                break
              hdr = (rtcm_blob[idx+1] << 8) | rtcm_blob[idx+2]
              msg_len = hdr & 0x03FF
              total = 3 + msg_len + 3
              if idx + total > len(rtcm_blob):
                break
              frame = rtcm_blob[idx:idx+total]
              payload = frame[3:3+msg_len]
              mtype = extract_msg_type(payload)
              if is_allowed(mtype):
                # Compute virtual absolute UTC ms now (aligns to ZDA/gga virtual clock)
                if virt_anchor_wall is not None and virt_anchor_base_ms is not None:
                  elapsed_ms_v = int((time.time() - virt_anchor_wall) * 1000.0 * (speed if speed > 0 else 1.0))
                  if virt_is_absolute:
                    virt_abs_ms_now = virt_anchor_base_ms + elapsed_ms_v
                  else:
                    # Day-relative base: assume same day as anchor base
                    day0 = datetime.datetime.now(datetime.UTC).replace(hour=0, minute=0, second=0, microsecond=0)
                    day0_ms = int(day0.timestamp() * 1000)
                    virt_abs_ms_now = day0_ms + (virt_anchor_base_ms + elapsed_ms_v)
                else:
                  virt_abs_ms_now = int(time.time() * 1000)

                # Write header just-in-time on first log line using virtual time
                nonlocal rtcm_out_header_written, rtcm_out_base_ms
                if (rtcm_out_fh is not None) and (not rtcm_out_header_written):
                  try:
                    rtcm_out_base_ms = virt_abs_ms_now
                    gen_dt = datetime.datetime.fromtimestamp(rtcm_out_base_ms / 1000.0, tz=datetime.UTC)
                    gen = gen_dt.isoformat()
                    rtcm_out_fh.write("# RTCM Message Log (ephemeris only)\n")
                    rtcm_out_fh.write(f"# Generated: {gen}\n")
                    rtcm_out_fh.write(f"# mountpoint: PPLOUT\n")
                    rtcm_out_fh.write(f"# Allowed types: [1005, 1033, 1230, 107x, 109x, 112x]\n")
                    rtcm_out_fh.write(f"# GPS Week: 0\n")
                    rtcm_out_fh.write(f"# GPS TOW: 0ms\n")
                    rtcm_out_fh.write(f"# Format: [timestamp_ms] [msg_type] [sat_prn] [length] [hex_data]\n")
                    rtcm_out_fh.write(f"# ----------------------------------------\n")
                  except Exception:
                    pass
                  rtcm_out_header_written = True
                sat = extract_sat_prn(mtype, payload)
                hex_data = frame.hex()
                # Millisecond offset from first RTCM out in this session
                ts_ms_rel = 0
                if rtcm_out_base_ms is not None:
                  ts_ms_rel = max(0, virt_abs_ms_now - rtcm_out_base_ms)
                line_str = f"{ts_ms_rel} {mtype} {sat} {len(frame)} {hex_data}"
                print(line_str)
                if rtcm_out_fh is not None:
                  try:
                    rtcm_out_fh.write(line_str)
                    rtcm_out_fh.write("\n")
                  except Exception:
                    pass
              idx += total

          # Read and append
          esp_chunk = ser.read(8192)
          if esp_chunk:
            esp_in_buf.extend(esp_chunk)

            # Process lines and frames
            while True:
              # Look for a header line
              nl = esp_in_buf.find(b"\n")
              if nl == -1:
                break
              line = esp_in_buf[:nl].decode('utf-8', errors='ignore').strip()
              rest = esp_in_buf[nl+1:]
              if line.startswith('!O ') or line.startswith('!RTK '):
                parts = line.split()
                if len(parts) >= 2:
                  try:
                    flen = int(parts[1])
                  except ValueError:
                    # discard malformed header
                    esp_in_buf = rest
                    continue
                  # Ensure we have payload + trailing newline
                  if len(rest) < flen + 1:
                    # Wait for more bytes
                    # Restore buffer and break
                    esp_in_buf = esp_in_buf
                    break
                  payload = rest[:flen]
                  trailer = rest[flen:flen+1]
                  # Move buffer forward
                  esp_in_buf = rest[flen+1:]
                  # Log parsed RTCM frames
                  log_rtcm_messages(payload)
                else:
                  esp_in_buf = rest
              else:
                # Not a framed block; print the line
                if line:
                  print(f"[ESP32<-] {line}")
                esp_in_buf = rest
            sys.stdout.flush()
        except Exception:
          pass

        # pump SPARTN: from file or PMP only
        if use_spartn_file or use_pmp_file or use_pmp_dir:
          # Align to first timestamp if available
          # Align to first timestamp if available
          if use_spartn_file and spartn_file_frames:
            if spartn_base_ts is None:
              spartn_base_ts = spartn_file_frames[0][0]
              spartn_index = 0
              # Defer start_time until ephemeris gate releases
              spartn_start_time = None
            if eph_initial_batch_done and spartn_start_time is None:
              spartn_start_time = virt_anchor_wall or time.time()
            # send due spartn frames (only when started)
            if spartn_start_time is not None:
              while spartn_index < len(spartn_file_frames) and (time.time() - spartn_start_time) * 1000.0 * speed >= (spartn_file_frames[spartn_index][0] - spartn_base_ts):
                _, sdat = spartn_file_frames[spartn_index]
                ser.write(f"!S {len(sdat)}\n".encode('ascii'))
                ser.write(sdat)
                ser.write(b"\n")
                print(f"[ESP32] pushed S {len(sdat)} bytes (spartn-file)")
                sys.stdout.flush()
                spartn_index += 1
          elif use_pmp_file and pmp_file_frames:
            # Determine base timestamp
            first_ts = None
            for t_ms, _ in pmp_file_frames:
              if t_ms is not None:
                first_ts = t_ms
                break
            if first_ts is not None:
              # schedule by timestamps; defer start until ephemeris gate releases
              if eph_initial_batch_done and pmp_start_time is None:
                pmp_start_time = virt_anchor_wall or time.time()
              if pmp_start_time is not None:
                # Pop frames in order when due
                while pmp_file_frames and pmp_file_frames[0][0] is not None and (time.time() - pmp_start_time) * 1000.0 * speed >= (pmp_file_frames[0][0] - first_ts):
                  _, spartn = pmp_file_frames.pop(0)
                  ser.write(f"!S {len(spartn)}\n".encode('ascii'))
                  ser.write(spartn)
                  ser.write(b"\n")
                  print(f"[ESP32] pushed S {len(spartn)} bytes (pmp-file ts)")
                  sys.stdout.flush()
            else:
              # No timestamps, send at a steady cadence (10 Hz)
              if pmp_file_frames and eph_initial_batch_done:
                if 'last_pmp_send' not in locals():
                  last_pmp_send = 0.0
                if (time.time() - last_pmp_send) >= (0.1 / speed):
                  t_ms, spartn = pmp_file_frames.pop(0)
                  ser.write(f"!S {len(spartn)}\n".encode('ascii'))
                  ser.write(spartn)
                  ser.write(b"\n")
                  print(f"[ESP32] pushed S {len(spartn)} bytes (pmp-file)\n")
                  sys.stdout.flush()
                  last_pmp_send = time.time()
          elif use_pmp_dir and pmp_dir_frames:
            # schedule by offsets
            if eph_initial_batch_done and pmp_start_time is None:
              pmp_start_time = virt_anchor_wall or time.time()
              pmp_index = 0
            # send due frames
            if pmp_start_time is not None:
              while pmp_index < len(pmp_dir_frames) and (time.time() - pmp_start_time) * 1000.0 * speed >= pmp_dir_frames[pmp_index][0]:
                _, spartn, fname = pmp_dir_frames[pmp_index]
                ser.write(f"!S {len(spartn)}\n".encode('ascii'))
                ser.write(spartn)
                ser.write(b"\n")
                print(f"[ESP32] pushed S {len(spartn)} bytes (pmp-dir- {fname})")
                sys.stdout.flush()
                pmp_index += 1
            # loop if requested
            if args.pmp_loop and pmp_index >= len(pmp_dir_frames):
              pmp_start_time = time.time()
              pmp_index = 0

        # pump EPH/RTCM
        # a) RTCM from directory: resend selected set every rtcm-cycle seconds based on virtual time
        if use_rtcm_dir:
          # Determine virtual absolute ms
          virt_abs_ms = None
          if virt_anchor_wall is not None and virt_anchor_base_ms is not None:
            elapsed_ms = int((time.time() - virt_anchor_wall) * 1000.0 * (speed if speed > 0 else 1.0))
            base_ms = virt_anchor_base_ms if virt_is_absolute else (virt_anchor_base_ms + (datetime.datetime.now(datetime.UTC).replace(hour=0, minute=0, second=0, microsecond=0).timestamp()*1000))
            try:
              base_ms = int(base_ms)
            except Exception:
              base_ms = virt_anchor_base_ms
            virt_abs_ms = (virt_anchor_base_ms + elapsed_ms) if virt_is_absolute else (base_ms + elapsed_ms)
          else:
            virt_abs_ms = int(time.time() * 1000)

          # Compute cycle id (every rtcm-cycle seconds)
          virt_sec = virt_abs_ms // 1000
          cycle_id = virt_sec // max(1, args.rtcm_cycle)
          # On new cycle, pick appropriate set and reset sender
          if rtcm_last_cycle_id is None or cycle_id != rtcm_last_cycle_id:
            rtcm_last_cycle_id = cycle_id
            # Choose set: last with time <= virt_abs_ms, else earliest
            sel = 0
            for i, (tms, _, _) in enumerate(rtcm_sets):
              if tms <= virt_abs_ms:
                sel = i
              else:
                break
            if rtcm_current_idx != sel:
              if 0 <= sel < len(rtcm_sets):
                print(f"[HOST] RTCM select: {rtcm_sets[sel][2]}")
            rtcm_current_idx = sel
            rtcm_send_index = 0
            rtcm_last_send = 0.0

          # Send current set frames paced by eph_rate (scaled by speed like eph-file)
          if 0 <= (rtcm_current_idx or 0) < len(rtcm_sets):
            frames = rtcm_sets[rtcm_current_idx][1]
            now_time = time.time()
            eff_rate = args.eph_rate / speed if speed > 0 else args.eph_rate
            if rtcm_send_index < len(frames) and (now_time - rtcm_last_send) >= eff_rate:
              frame = frames[rtcm_send_index]
              ser.write(f"!R {len(frame)}\n".encode('ascii'))
              ser.write(frame)
              ser.write(b"\n")
              print(f"[ESP32] pushed R {len(frame)} bytes ({rtcm_sets[rtcm_current_idx][2]})")
              sys.stdout.flush()
              rtcm_send_index += 1
              rtcm_last_send = now_time

        # b) Ephemeris from single file (legacy path)
        if use_eph_file:
          now_time = time.time()
          eff_eph_rate = args.eph_rate / speed if speed > 0 else args.eph_rate
          if eph_index < len(eph_file_frames) and (now_time - last_eph_file_send) >= eff_eph_rate:
            frame = eph_file_frames[eph_index]
            ser.write(f"!R {len(frame)}\n".encode('ascii'))
            ser.write(frame)
            ser.write(b"\n")
            print(f"[ESP32] pushed R {len(frame)} bytes (file)")
            sys.stdout.flush()
            eph_index += 1
            last_eph_file_send = now_time
          # If we've sent all frames and eph-loop is enabled, wait until next loop interval
          # Detect initial batch completion to release gating
          if eph_index >= len(eph_file_frames) and not eph_initial_batch_done:
            eph_initial_batch_done = True
            print(f"[ESP32] EPH initial batch complete; releasing GGA/SPARTN")
          if eph_index >= len(eph_file_frames) and args.eph_loop > 0:
            # Initialize a loop timer
            if 'eph_loop_start' not in locals():
              eph_loop_start = time.time()
            eff_loop = args.eph_loop / speed if speed > 0 else args.eph_loop
            if (time.time() - eph_loop_start) >= eff_loop:
              eph_index = 0
              eph_loop_start = time.time()

    except KeyboardInterrupt:
      print("Interrupted")
      break
    except Exception as e:
      print(f"Reconnecting in 3s due to: {e}")
      time.sleep(3)

  # Close RTCM output log if open
  try:
    if rtcm_out_fh is not None:
      rtcm_out_fh.close()
  except Exception:
    pass

if __name__ == '__main__':
  main()


