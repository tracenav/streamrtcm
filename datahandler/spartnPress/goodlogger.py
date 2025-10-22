#!/usr/bin/env python
"""
SPARTN PointPerfect NTRIP logger (text .log by default).

What it does:
- Connects to PointPerfect NTRIP (e.g., NEAR-SPARTN) and sends periodic GGA.
- Parses the SPARTN transport frames (preamble 0x73) from the TCP stream.
- Writes a line-oriented log compatible with typical Tracenav log style:
  "# Format: [timestamp_ms] [type] [subtype] [length] [hex_data]"
  where timestamp_ms is relative to the start time of each log file.

Message type basics (columns are type and subtype as separate fields):
Type 0, Subtype 0: OCB (GPS)
Type 0, Subtype 1: OCB (GLONASS)
Type 0, Subtype 2: OCB (Galileo)
Type 0, Subtype 3: OCB (BeiDou)
Type 1, Subtype 0: HPAC 1-0
Type 1, Subtype 1: HPAC 1-1
Type 1, Subtype 2: HPAC 1-2
Type 1, Subtype 3: HPAC 1-3
Type 2, Subtype 0: GAD

Notes:
- Binary capture (.bin) is disabled by default; pass --binary to enable.
- The script rotates to a new file every --log-duration seconds (default 60).

About subtype:
- For Type 0 (OCB), subtype selects constellation: 0=GPS, 1=GLONASS, 2=Galileo, 3=BeiDou, 4=QZSS.
- For Type 1 (HPAC), subtype (0–4) denotes the HPAC payload variant as defined in the ICD (1-0..1-4).
- For Type 2 (GAD) and Type 3 (BPAC), subtype is 0.
- For higher types (EAS/admin), subtype enumerates the specific support message per ICD tables.
"""

import socket
import base64
import sys
try:
    import serial  # optional
except Exception:  # pragma: no cover
    serial = None
import argparse
import time
import datetime
import os
from pathlib import Path

# Access repo decoder to parse SPARTN frames
try:
    sys.path.append(str(Path(__file__).resolve().parents[1]))  # repo root
except Exception:
    pass
from decode_spartn_from_pmp import parse_spartn_frame  # type: ignore

# Hardcoded GGA parameters - easily configurable
LATITUDE = 38.36876          # Latitude in decimal degrees (NYC example)
LONGITUDE = -78.92694        # Longitude in decimal degrees (NYC example)
ALTITUDE = 10.0             # Altitude above sea level in meters
GEOID_HEIGHT = -32.0        # Height of geoid above WGS84 ellipsoid in meters
SATELLITES = 15             # Number of satellites in use
HDOP = 0.9                  # Horizontal dilution of precision
GPS_QUALITY = 1             # GPS quality (0=invalid, 1=GPS fix, 2=DGPS fix)
DGPS_AGE = 0.0              # Time since last DGPS update in seconds
DGPS_ID = 0                 # DGPS station ID
GGA_INTERVAL = 10           # GGA sending interval in seconds

# PointPerfect NTRIP server settings
DEFAULT_SERVER = 'ppntrip.services.u-blox.com'
DEFAULT_PORT = '2101'
DEFAULT_MOUNTPOINT = 'NEAR-SPARTN'
DEFAULT_USERNAME = 'j2Nu9ebxjdmm'
DEFAULT_PASSWORD = 'UgeJADKBpmR4'

# Logging settings
LOG_DIRECTORY = 'spartn_logs'
LOG_FILE_DURATION = 60  # seconds per file
RECONNECT_DELAY = 2       # Seconds to wait before reconnecting

# Command line arguments
parser = argparse.ArgumentParser(description='NTRIP Client with SPARTN Logging and Auto-Reconnect')
parser.add_argument('--tty', default=None, help='Serial port device (optional)')
parser.add_argument('--server', default=DEFAULT_SERVER, help='NTRIP server')
parser.add_argument('--port', default=DEFAULT_PORT, help='NTRIP port')
parser.add_argument('--mountpoint', default=DEFAULT_MOUNTPOINT, help='Mountpoint')
parser.add_argument('--username', default=DEFAULT_USERNAME, help='Username')
parser.add_argument('--password', default=DEFAULT_PASSWORD, help='Password')
parser.add_argument('--logdir', default=LOG_DIRECTORY, help='Directory for SPARTN log files')
parser.add_argument('--log-duration', default=LOG_FILE_DURATION, type=int, help='Log file duration in seconds')
parser.add_argument('--binary', action='store_true', help='Also write raw binary stream (.bin). Disabled by default')
parser.add_argument('--single-log', default=None, help='Write all text output to this single .log file (no rotation)')

args = parser.parse_args()

def decimal_to_nmea_lat(decimal_degrees):
    """Convert decimal latitude to NMEA format (ddmm.mmmmm)"""
    abs_lat = abs(decimal_degrees)
    degrees = int(abs_lat)
    minutes = (abs_lat - degrees) * 60
    hemisphere = 'N' if decimal_degrees >= 0 else 'S'
    return f"{degrees:02d}{minutes:08.5f}", hemisphere

def decimal_to_nmea_lon(decimal_degrees):
    """Convert decimal longitude to NMEA format (dddmm.mmmmm)"""
    abs_lon = abs(decimal_degrees)
    degrees = int(abs_lon)
    minutes = (abs_lon - degrees) * 60
    hemisphere = 'E' if decimal_degrees >= 0 else 'W'
    return f"{degrees:03d}{minutes:08.5f}", hemisphere

def calculate_nmea_checksum(sentence):
    """Calculate NMEA checksum for a sentence (without $ and *)"""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def generate_gga_sentence(lat, lon, altitude=50.0, geoid_height=-32.0, satellites=12, 
                         hdop=1.2, gps_quality=1, dgps_age=0.0, dgps_id=0):
    """Generate a complete NMEA GGA sentence"""
    
    # Get current UTC time
    now = datetime.datetime.now(datetime.timezone.utc)
    time_str = now.strftime("%H%M%S.%f")[:-4]  # HHMMSS.SS format
    
    # Convert coordinates to NMEA format
    lat_nmea, lat_hemisphere = decimal_to_nmea_lat(lat)
    lon_nmea, lon_hemisphere = decimal_to_nmea_lon(lon)
    
    # Build GGA sentence (without $ and checksum)
    gga_data = (
        f"GPGGA,"
        f"{time_str},"
        f"{lat_nmea},{lat_hemisphere},"
        f"{lon_nmea},{lon_hemisphere},"
        f"{gps_quality},"
        f"{satellites:02d},"
        f"{hdop:.1f},"
        f"{altitude:.1f},M,"
        f"{geoid_height:.1f},M,"
        f"{dgps_age:.1f},"
        f"{dgps_id:04d}"
    )
    
    # Calculate checksum
    checksum = calculate_nmea_checksum(gga_data)
    
    # Complete sentence
    complete_sentence = f"${gga_data}*{checksum}\r\n"
    
    return complete_sentence

def getHTTPBasicAuthString(username, password):
    inputstring = username + ':' + password
    pwd_bytes = base64.encodebytes(inputstring.encode("utf-8"))
    pwd = pwd_bytes.decode("utf-8").replace('\n','')
    return pwd

def create_log_directory(log_dir):
    """Create log directory if it doesn't exist"""
    Path(log_dir).mkdir(parents=True, exist_ok=True)

def get_log_filenames(log_dir):
    """Generate pair of filenames (text .log and binary .bin)"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(log_dir, f"spartn_{timestamp}")
    return base + ".log", base + ".bin"

def connect_to_ntrip(server, port, mountpoint, username, password):
    """Establish connection to NTRIP server"""
    print(f"Connecting to NTRIP server: {server}:{port}")
    print(f"Mountpoint: {mountpoint}")
    print(f"Username: {username}")
    
    pwd = getHTTPBasicAuthString(username, password)
    
    header = (
        f"GET /{mountpoint} HTTP/1.0\r\n"
        f"User-Agent: NTRIP u-blox\r\n"
        f"Accept: */*\r\n"
        f"Authorization: Basic {pwd}\r\n"
        f"Connection: keep-alive\r\n\r\n"
    )
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((server, int(port)))
    s.sendto(header.encode('utf-8'), (server, int(port)))
    resp = s.recv(1024)
    
    print("Server response:")
    print(resp.decode('utf-8', errors='ignore'))
    
    if resp.startswith(b"STREAMTABLE"):
        raise Exception("Invalid or No Mountpoint")
    elif resp.startswith(b"HTTP/1.1 200 OK") or resp.startswith(b"ICY 200 OK"):
        print("Connection successful - receiving data...")
        return s
    else:
        raise Exception("Unexpected response from server")

def main():
    # Initialize serial connection if tty is provided
    ser = None
    if args.tty and serial is not None:
        try:
            ser = serial.Serial(args.tty, 19200, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False)
            ser.flushInput()
            ser.flushOutput()
            print(f"Serial port {args.tty} opened successfully")
        except Exception as e:
            print(f"Error opening serial port {args.tty}: {e}")
            sys.exit(1)
    else:
        print("No serial port specified - logging to files only")

    # Create log directory
    create_log_directory(args.logdir)
    
    print(f"Position: {LATITUDE:.6f}, {LONGITUDE:.6f}")
    print(f"Altitude: {ALTITUDE}m, Satellites: {SATELLITES}")
    print(f"Logging to directory: {args.logdir}")
    print(f"Log file duration: {args.log_duration} seconds")

    # SPARTN frame parser state
    buffer = bytearray()
    start_time = None  # reset per file

    while True:  # Main reconnection loop
        s = None
        log_file_txt = None
        log_file_bin = None
        
        try:
            # Connect to NTRIP server
            s = connect_to_ntrip(args.server, args.port, args.mountpoint, args.username, args.password)
            
            # Generate and send initial GGA sentence
            gga_sentence = generate_gga_sentence(
                LATITUDE, LONGITUDE, ALTITUDE, GEOID_HEIGHT,
                SATELLITES, HDOP, GPS_QUALITY, 
                DGPS_AGE, DGPS_ID
            )
            
            print(f"Sending GGA sentence: {gga_sentence.strip()}")
            s.send(gga_sentence.encode('utf-8'))
            
            last_gga_time = time.time()
            log_start_time = time.time()
            data_count = 0
            
            # Open initial log files
            if args.single_log:
                # Ensure parent directory exists
                Path(os.path.dirname(args.single_log) or '.').mkdir(parents=True, exist_ok=True)
                # Append if file exists to keep prior content across reconnects
                mode = 'a' if os.path.exists(args.single_log) else 'w'
                log_file_txt = open(args.single_log, mode)
                log_txt = args.single_log
                log_bin = None
            else:
                log_txt, log_bin = get_log_filenames(args.logdir)
                log_file_txt = open(log_txt, 'w')
            log_file_bin = open(log_bin, 'wb') if args.binary else None
            print(f"Started logging to: {log_txt}{' and ' + log_bin if args.binary else ''}")

            # Write header for text log (RTCM-like format)
            now_iso = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S UTC')
            log_file_txt.write("# SPARTN Message Log\n")
            log_file_txt.write(f"# Generated: {now_iso}\n")
            log_file_txt.write(f"# Mountpoint: {args.mountpoint}\n")
            log_file_txt.write("# GPS Week: 0\n")
            log_file_txt.write("# GPS TOW: 0ms\n")
            log_file_txt.write("# Format: [timestamp_ms] [type] [subtype] [length] [hex_data]\n")
            log_file_txt.write("# ----------------------------------------\n")
            log_file_txt.flush()
            # Reset relative timestamp for this file
            start_time = time.time()

            while True:  # Data reception loop
                # Check if we need to rotate log files (disabled when single-log is set)
                current_time = time.time()
                if (not args.single_log) and (current_time - log_start_time >= args.log_duration):
                    if log_file_txt:
                        log_file_txt.close()
                    if log_file_bin:
                        log_file_bin.close()
                    print("Closed current log files")

                    log_txt, log_bin = get_log_filenames(args.logdir)
                    log_file_txt = open(log_txt, 'w')
                    log_file_bin = open(log_bin, 'wb') if args.binary else None
                    log_start_time = current_time
                    print(f"Started new log file: {log_txt}{' and ' + log_bin if args.binary else ''}")

                    # Rewrite header for new text log
                    now_iso = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S UTC')
                    log_file_txt.write("# SPARTN Message Log\n")
                    log_file_txt.write(f"# Generated: {now_iso}\n")
                    log_file_txt.write(f"# Mountpoint: {args.mountpoint}\n")
                    log_file_txt.write("# GPS Week: 0\n")
                    log_file_txt.write("# GPS TOW: 0ms\n")
                    log_file_txt.write("# Format: [timestamp_ms] [type] [subtype] [length] [hex_data]\n")
                    log_file_txt.write("# ----------------------------------------\n")
                    log_file_txt.flush()
                    # Reset relative timestamp for new file
                    start_time = time.time()
                
                # Send GGA sentence periodically
                if current_time - last_gga_time >= GGA_INTERVAL:
                    gga_sentence = generate_gga_sentence(
                        LATITUDE, LONGITUDE, ALTITUDE, GEOID_HEIGHT,
                        SATELLITES, HDOP, GPS_QUALITY,
                        DGPS_AGE, DGPS_ID
                    )
                    try:
                        s.send(gga_sentence.encode('utf-8'))
                        print(f"Sent periodic GGA: {gga_sentence.strip()}")
                        last_gga_time = current_time
                    except:
                        print("Could not send GGA - connection may be closed")
                        break
                
                # Receive data with timeout
                s.settimeout(1.0)  # 1 second timeout
                try:
                    data = s.recv(1024)
                except socket.timeout:
                    continue  # Continue loop to check for GGA sending and log rotation
                
                if not data:
                    print("No more data received - connection closed")
                    break
                
                data_count += len(data)
                
                # Write raw data to binary file
                if args.binary and log_file_bin:
                    log_file_bin.write(data)
                    log_file_bin.flush()

                # Parse SPARTN frames and write line entries
                buffer.extend(data)
                while True:
                    if len(buffer) < 4:
                        break
                    pre_idx = buffer.find(0x73)
                    if pre_idx == -1:
                        # keep last few bytes to handle split preamble
                        buffer = buffer[-3:]
                        break
                    if pre_idx > 0:
                        buffer = buffer[pre_idx:]
                    parsed = parse_spartn_frame(bytes(buffer), 0)
                    if parsed is None:
                        # drop one byte and retry
                        buffer = buffer[1:]
                        continue
                    rec, next_index, _payload, _auth = parsed
                    if next_index <= 0 or next_index > len(buffer):
                        break
                    frame = bytes(buffer[:next_index])
                    buffer = buffer[next_index:]
                    # If start_time not yet initialized (edge case), initialize now
                    if start_time is None:
                        start_time = time.time()
                    ts_ms = int((time.time() - start_time) * 1000)
                    type_field = int(rec.get('type', 0))
                    subtype = int(rec.get('subtype', 0))
                    # Drop the ":0" placeholder; use plain ms
                    line = f"{ts_ms} {type_field} {subtype} {len(frame)} {frame.hex()}\n"
                    if log_file_txt:
                        log_file_txt.write(line)
                        log_file_txt.flush()
                
                # Write to serial port if available
                if ser:
                    ret = ser.write(data)
                    if data_count % 10240 == 0:  # Print every 10KB
                        print(f"Received {data_count} bytes, wrote {ret} bytes to serial port")
                else:
                    if data_count % 10240 == 0:  # Print every 10KB
                        print(f"Received {data_count} bytes, logged to file")
                
                sys.stdout.flush()
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
            break
        except Exception as e:
            print(f"Connection error: {e}")
            print(f"Will reconnect in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
        finally:
            if s:
                s.close()
            if log_file_txt:
                log_file_txt.close()
            if log_file_bin:
                log_file_bin.close()
            print("Closed log files")
    
    if ser:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()