#!/usr/bin/env python

import socket
import base64
import sys
import serial
import argparse
import time
import datetime
import os
from pathlib import Path

# Hardcoded GGA parameters - easily configurable
LATITUDE = 40.7128          # Latitude in decimal degrees (NYC example)
LONGITUDE = -74.0060        # Longitude in decimal degrees (NYC example)
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
LOG_FILE_DURATION = 60  # 1 hour per file in seconds
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

def get_log_filename(log_dir):
    """Generate log filename based on current timestamp"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(log_dir, f"spartn_{timestamp}.log")

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
        print("Connection successful - receiving RTCM data...")
        return s
    else:
        raise Exception("Unexpected response from server")

def main():
    # Initialize serial connection if tty is provided
    ser = None
    if args.tty:
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

    while True:  # Main reconnection loop
        s = None
        log_file = None
        
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
            
            # Open initial log file
            log_filename = get_log_filename(args.logdir)
            log_file = open(log_filename, 'wb')
            print(f"Started logging to: {log_filename}")

            while True:  # Data reception loop
                # Check if we need to rotate log file
                current_time = time.time()
                if current_time - log_start_time >= args.log_duration:
                    if log_file:
                        log_file.close()
                        print(f"Closed log file: {log_filename}")
                    
                    log_filename = get_log_filename(args.logdir)
                    log_file = open(log_filename, 'wb')
                    log_start_time = current_time
                    print(f"Started new log file: {log_filename}")
                
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
                
                # Log data to file
                if log_file:
                    log_file.write(data)
                    log_file.flush()  # Ensure data is written immediately
                
                # Print a small hex snippet of received data
                try:
                    head_len = 24 if len(data) >= 24 else len(data)
                    snippet = data[:head_len].hex()
                    print(f"RX {len(data)}B: {snippet}{'...' if len(data) > head_len else ''}")
                except Exception as _:
                    pass
                
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
            if log_file:
                log_file.close()
                print(f"Closed log file: {log_filename}")
    
    if ser:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main() 