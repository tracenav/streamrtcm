#!/usr/bin/env python3

import socket
import base64
import sys
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
GGA_INTERVAL = 99999999           # GGA sending interval in seconds

# Default server settings
#GLOBAL EPHMERIS BCEP00BKG0
#SSR SSRA00CNE0

DEFAULT_SERVER = 'ntrip.data.gnss.ga.gov.au'
DEFAULT_PORT = '2101'
DEFAULT_MOUNTPOINT = 'BCEP00BKG0'
DEFAULT_USERNAME = 'jacob222'
DEFAULT_PASSWORD = 'Gilmer2284!'

# Logging settings
LOG_DIRECTORY = 'rtcm_logs'
RECONNECT_DELAY = 2       # Seconds to wait before reconnecting

# Command line arguments
parser = argparse.ArgumentParser(description='NTRIP Client with RTCM Message Parsing and Logging')
parser.add_argument('--tty', default=None, help='Serial port device (optional)')
parser.add_argument('--server', default=DEFAULT_SERVER, help='NTRIP server')
parser.add_argument('--port', default=DEFAULT_PORT, help='NTRIP port')
parser.add_argument('--mountpoint', default=DEFAULT_MOUNTPOINT, help='Mountpoint')
parser.add_argument('--username', default=DEFAULT_USERNAME, help='Username')
parser.add_argument('--password', default=DEFAULT_PASSWORD, help='Password')
parser.add_argument('--logdir', default=LOG_DIRECTORY, help='Directory for RTCM log files')
parser.add_argument('--session-type', choices=['ephemeris', 'ssr'], default='ssr', 
                   help='Session type: ephemeris (for broadcast ephemeris) or ssr (for corrections)')
parser.add_argument('--binary-only', action='store_true', help='Only output binary .bin file, skip text .log file')

args = parser.parse_args()

class RTCMParser:
    def __init__(self):
        self.buffer = bytearray()
        self.start_time = time.time()
        
    def parse_rtcm_messages(self, data):
        """Parse RTCM 3.x messages from binary data stream"""
        self.buffer.extend(data)
        messages = []
        
        while len(self.buffer) >= 3:
            # Look for RTCM 3.x preamble (0xD3)
            preamble_idx = self.buffer.find(0xD3)
            if preamble_idx == -1:
                # No preamble found, clear buffer
                self.buffer.clear()
                break
                
            if preamble_idx > 0:
                # Remove data before preamble
                self.buffer = self.buffer[preamble_idx:]
                
            if len(self.buffer) < 3:
                break
                
            # Parse message length from bytes 1-2
            # Format: 6 reserved bits + 10 length bits
            length_bytes = (self.buffer[1] << 8) | self.buffer[2]
            msg_length = length_bytes & 0x3FF  # Extract lower 10 bits
            
            total_length = 3 + msg_length + 3  # preamble + length + payload + CRC
            
            if len(self.buffer) < total_length:
                # Not enough data for complete message
                break
                
            # Extract complete message
            message_data = bytes(self.buffer[:total_length])
            
            # Parse message type from payload (first 12 bits after length)
            if msg_length >= 2:
                msg_type = ((self.buffer[3] << 4) | (self.buffer[4] >> 4)) & 0xFFF
                
                # Extract satellite PRN if applicable
                sat_prn = self._extract_satellite_prn(msg_type, self.buffer[3:3+msg_length])
                
                # Calculate timestamp (milliseconds since start)
                timestamp_ms = int((time.time() - self.start_time) * 1000)
                
                # Create message record
                message_record = {
                    'timestamp_ms': timestamp_ms,
                    'msg_type': msg_type,
                    'sat_prn': sat_prn,
                    'length': total_length,
                    'hex_data': message_data.hex()
                }
                messages.append(message_record)
            
            # Remove processed message from buffer
            self.buffer = self.buffer[total_length:]
            
        return messages
    
    def _extract_satellite_prn(self, msg_type, payload):
        """Extract satellite PRN from specific RTCM message types"""
        if len(payload) < 2:
            return 0
            
        # Message type specific PRN extraction
        if msg_type in [1019, 1020, 1044, 1045, 1046]:  # GPS, GLONASS, Galileo ephemeris
            # PRN is typically in bits 12-17 (6 bits) after message type
            if len(payload) >= 3:
                prn_bits = ((payload[1] & 0x0F) << 2) | ((payload[2] & 0xC0) >> 6)
                return prn_bits if prn_bits > 0 else 0
                
        elif msg_type in [1042, 1043]:  # BeiDou ephemeris
            if len(payload) >= 3:
                prn_bits = ((payload[1] & 0x0F) << 2) | ((payload[2] & 0xC0) >> 6)
                return prn_bits if prn_bits > 0 else 0
                
        elif msg_type in [1001, 1002, 1003, 1004]:  # GPS observations
            return 0  # Multiple satellites, return 0
            
        elif msg_type in [1009, 1010, 1011, 1012]:  # GLONASS observations
            return 0  # Multiple satellites, return 0
            
        elif msg_type == 1005:  # Station coordinates
            return 0
            
        return 0

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

def create_gga_sentence():
    """Create NMEA GGA sentence for position reporting"""
    now = datetime.datetime.utcnow()
    timestamp = now.strftime("%H%M%S.%f")[:-4]  # HHMMSS.SS format
    
    lat_nmea, lat_hem = decimal_to_nmea_lat(LATITUDE)
    lon_nmea, lon_hem = decimal_to_nmea_lon(LONGITUDE)
    
    # Create GGA sentence without checksum
    gga_no_checksum = (f"GPGGA,{timestamp},{lat_nmea},{lat_hem},{lon_nmea},{lon_hem},"
                      f"{GPS_QUALITY},{SATELLITES},{HDOP},{ALTITUDE},M,"
                      f"{GEOID_HEIGHT},M,{DGPS_AGE},{DGPS_ID:04d}")
    
    # Calculate checksum
    checksum = 0
    for char in gga_no_checksum:
        checksum ^= ord(char)
    
    return f"${gga_no_checksum}*{checksum:02X}"

def getHTTPBasicAuthString(username, password):
    inputstring = username + ':' + password
    pwd_bytes = base64.encodebytes(inputstring.encode("utf-8"))
    pwd = pwd_bytes.decode("utf-8").replace('\n','')
    return pwd

# Initialize RTCM parser
rtcm_parser = RTCMParser()

# Create log directory
log_dir = Path(args.logdir)
log_dir.mkdir(exist_ok=True)

# NTRIP server configuration
server = args.server
port = args.port
mountpoint = args.mountpoint
username = args.username
password = args.password

print(f"RTCM Parser Logger - Session Type: {args.session_type.upper()}")
print(f"Connecting to NTRIP server: {server}:{port}")
print(f"Mountpoint: {mountpoint}")
print(f"Username: {username}")
print(f"Position: {LATITUDE:.6f}, {LONGITUDE:.6f}")
print(f"Logging to directory: {args.logdir}")
print(f"Output format: {'Binary only (.bin)' if args.binary_only else 'Text (.log) + Binary (.bin)'}")

# Create log files
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
session_suffix = f"_{args.session_type}" if args.session_type != 'ssr' else ""
log_file_path = log_dir / f"rtcm_{timestamp}{session_suffix}.log"
bin_file_path = log_dir / f"rtcm_{timestamp}{session_suffix}.bin"

def connect_and_log():
    """Connect to NTRIP server and log RTCM messages"""
    global rtcm_parser
    
    try:
        # Create socket connection
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(30)
        s.connect((server, int(port)))
        
        # Create HTTP request
        pwd = getHTTPBasicAuthString(username, password)
        header = (f"GET /{mountpoint} HTTP/1.0\r\n"
                 f"User-Agent: NTRIP RTCMLogger\r\n"
                 f"Accept: */*\r\n"
                 f"Authorization: Basic {pwd}\r\n"
                 f"Connection: close\r\n\r\n")
        
        s.sendto(header.encode('utf-8'), (server, int(port)))
        resp = s.recv(1024)
        
        print("Server response:")
        print(resp.decode('utf-8', errors='ignore'))
        
        if resp.startswith(b"STREAMTABLE"):
            print("Invalid or No Mountpoint")
            return False
        elif not (resp.startswith(b"HTTP/1.1 200 OK") or resp.startswith(b"ICY 200 OK")):
            print("Connection error: Unexpected response from server")
            return False
        
        print("Connection successful - receiving RTCM data...")
        
        # Open log files
        log_file = None
        if not args.binary_only:
            log_file = open(log_file_path, 'w')
            # Write header
            log_file.write("# RTCM Message Log\n")
            log_file.write(f"# Generated: {datetime.datetime.now().isoformat()}\n")
            log_file.write(f"# Session Type: {args.session_type}\n")
            log_file.write(f"# Mountpoint: {mountpoint}\n")
            log_file.write("# GPS Week: 0\n")
            log_file.write("# GPS TOW: 0ms\n")
            log_file.write("# Format: [timestamp_ms] [msg_type] [sat_prn] [length] [hex_data]\n")
            log_file.write("# ----------------------------------------\n")
            log_file.flush()
            
        bin_file = open(bin_file_path, 'wb')
        
        if log_file:
            print(f"Started logging to: {log_file_path}")
        print(f"Started binary logging to: {bin_file_path}")
        
        last_gga_time = time.time()
        
        try:
            while True:
                try:
                    # Send periodic GGA sentences
                    current_time = time.time()
                    if current_time - last_gga_time >= GGA_INTERVAL:
                        gga_sentence = create_gga_sentence()
                        s.send((gga_sentence + '\r\n').encode())
                        print(f"Sent periodic GGA: {gga_sentence}")
                        last_gga_time = current_time
                    
                    # Receive RTCM data
                    data = s.recv(4096)
                    if not data:
                        print("Connection closed by server")
                        break
                    
                    # Write raw binary data to .bin file
                    bin_file.write(data)
                    bin_file.flush()
                    
                    # Parse RTCM messages and log to text file (if enabled)
                    if log_file:
                        messages = rtcm_parser.parse_rtcm_messages(data)
                        
                        # Log messages
                        for msg in messages:
                            log_line = f"{msg['timestamp_ms']}:0 {msg['msg_type']} {msg['sat_prn']} {msg['length']} {msg['hex_data']}\n"
                            log_file.write(log_line)
                            log_file.flush()
                            print(f"Logged: MSG {msg['msg_type']} PRN {msg['sat_prn']} ({msg['length']} bytes)")
                    else:
                        # Just show data received without parsing
                        print(f"Received {len(data)} bytes of binary RTCM data")
                        
                except socket.timeout:
                    print("Socket timeout - continuing...")
                    continue
                except KeyboardInterrupt:
                    print("\nInterrupted by user")
                    break
                except Exception as e:
                    print(f"Error receiving data: {e}")
                    break
        finally:
            # Close files
            if log_file:
                log_file.close()
                print(f"Closed log file: {log_file_path}")
            bin_file.close()
            print(f"Closed binary file: {bin_file_path}")
        
        s.close()
        return True
        
    except Exception as e:
        print(f"Connection failed: {e}")
        return False

# Main loop with reconnection
while True:
    try:
        if connect_and_log():
            break  # Successful completion
        else:
            print(f"Will reconnect in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        break
    except Exception as e:
        print(f"Unexpected error: {e}")
        print(f"Will reconnect in {RECONNECT_DELAY} seconds...")
        time.sleep(RECONNECT_DELAY)