#!/usr/bin/env python3

import socket
import base64
import argparse
import time
import datetime
import os
import json
import logging
import threading
import urllib.parse
import mimetypes
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path

#ephem credentials BCEP00BKG0, ssr-SSRA00CNE0
# Default server settings
DEFAULT_SERVER = 'ntrip.data.gnss.ga.gov.au'
DEFAULT_PORT = '2101'
DEFAULT_MOUNTPOINT = 'BCEP00BKG0'
DEFAULT_USERNAME = 'jacob222'
DEFAULT_PASSWORD = 'Gilmer2284!'

# Logging settings
LOG_DIRECTORY = 'rtcm_logs'

# Base station position for NMEA GGA (Fredericksburg, VA)
BASE_LAT_DEG = 38.3032   # North positive
BASE_LON_DEG = -77.4605  # West negative
DEFAULT_GGA_INTERVAL_SECONDS = 5
DEFAULT_GGA_DELAY_SECONDS = 3

def is_msm_message(msg_type: int) -> bool:
    # MSM message families: 107x GPS, 108x GLO, 109x GAL, 110x SBAS, 111x QZSS, 112x BDS, 113x IRNSS
    return (
        1071 <= msg_type <= 1077 or
        1081 <= msg_type <= 1087 or
        1091 <= msg_type <= 1097 or
        1101 <= msg_type <= 1107 or
        1111 <= msg_type <= 1117 or
        1121 <= msg_type <= 1127 or
        1131 <= msg_type <= 1137
    )

# (no expected PRN set; continuous logger)

# Command line arguments
parser = argparse.ArgumentParser(description='Base RTCM logger (all RTCM message types) with optional HTTP server')
parser.add_argument('--server', default=DEFAULT_SERVER, help='NTRIP server')
parser.add_argument('--port', default=DEFAULT_PORT, help='NTRIP port')
parser.add_argument('--mountpoint', default=DEFAULT_MOUNTPOINT, help='Mountpoint')
parser.add_argument('--username', default=DEFAULT_USERNAME, help='Username')
parser.add_argument('--password', default=DEFAULT_PASSWORD, help='Password')
parser.add_argument('--logdir', default=LOG_DIRECTORY, help='Directory for RTCM log files')
parser.add_argument('--serve', action='store_true', help='Serve log files over HTTP while logging')
parser.add_argument('--host', default='127.0.0.1', help='HTTP server bind host (default: 0.0.0.0)')
parser.add_argument('--http-port', type=int, default=8081, help='HTTP server port (default: 8081)')
parser.add_argument('--gga-interval', type=int, default=DEFAULT_GGA_INTERVAL_SECONDS, help=f'GGA message interval in seconds (default: {DEFAULT_GGA_INTERVAL_SECONDS})')
parser.add_argument('--gga-delay', type=int, default=DEFAULT_GGA_DELAY_SECONDS, help=f'Delay in seconds before sending first GGA after connecting (default: {DEFAULT_GGA_DELAY_SECONDS})')

args = parser.parse_args()

class RTCMParser:
    def __init__(self):
        self.buffer = bytearray()
        
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
                
                # Capture Unix timestamp in milliseconds
                unix_ms = int(time.time() * 1000)
                
                # Create message record
                message_record = {
                    'unix_ms': unix_ms,
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
        
        # MSM messages include multiple satellites; do not extract PRN
        try:
            if is_msm_message(msg_type):
                return 0
        except Exception:
            pass
        
        return 0

def getHTTPBasicAuthString(username, password):
    inputstring = username + ':' + password
    pwd_bytes = base64.encodebytes(inputstring.encode("utf-8"))
    pwd = pwd_bytes.decode("utf-8").replace('\n','')
    return pwd


# Initialize RTCM parser
rtcm_parser = RTCMParser()

# Prepare log directory
log_dir = Path(args.logdir)
log_dir.mkdir(exist_ok=True)

# NTRIP server configuration
server = args.server
port = args.port
mountpoint = args.mountpoint
username = args.username
password = args.password

print("Base RTCM Logger (all message types)")
print(f"NTRIP server: {server}:{port}")
print(f"Mountpoint: {mountpoint}")
print(f"Logging directory: {args.logdir}")
print("Logging filter: none (logging every RTCM message type)")
gga_status = f"every {args.gga_interval}s" if args.gga_interval > 0 else "disabled"
print(f"GGA base position: {BASE_LAT_DEG}, {BASE_LON_DEG} ({gga_status}, delay: {args.gga_delay}s)")


class LogServerHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):
        logging.info(f"{self.address_string()} - {format % args}")

    def do_GET(self):
        if self.path == '/status':
            self.handle_status()
        elif self.path == '/data' or self.path == '/data/':
            self.list_log_files()
        elif self.path.startswith('/data/'):
            self.serve_log_file()
        else:
            self.serve_index()

    def handle_status(self):
        try:
            file_count = 0
            total_size = 0
            if os.path.exists(log_dir):
                for root, _, files in os.walk(log_dir):
                    for filename in files:
                        if filename.endswith('.log'):
                            filepath = os.path.join(root, filename)
                            file_count += 1
                            total_size += os.path.getsize(filepath)

            status = {
                'status': 'running',
                'server_time': datetime.datetime.utcnow().isoformat() + 'Z',
                'log_dir': str(log_dir),
                'log_files': file_count,
                'total_size_bytes': total_size,
                'total_size_mb': round(total_size / (1024 * 1024), 2),
            }
            body = json.dumps(status, indent=2).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            logging.error(f"Error in /status: {e}")
            self.send_error(500, f"Server error: {str(e)}")

    def serve_index(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Base RTCM Logger</title>
            <meta http-equiv="refresh" content="30">
        </head>
        <body>
            <h1>Base RTCM Logger</h1>
            <p>Server running at {datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')} UTC</p>
            <p><a href="/status">JSON Status</a></p>
            <p><a href="/data/">Browse Log Files</a></p>
        </body>
        </html>
        """
        self.wfile.write(html.encode())

    def list_log_files(self):
        try:
            files = []
            if os.path.exists(log_dir):
                for root, _, filenames in os.walk(log_dir):
                    for filename in filenames:
                        if filename.endswith('.log'):
                            filepath = os.path.join(root, filename)
                            rel_path = os.path.relpath(filepath, log_dir)
                            file_size = os.path.getsize(filepath)
                            file_time = datetime.datetime.fromtimestamp(os.path.getmtime(filepath))
                            files.append({
                                'name': rel_path,
                                'size': file_size,
                                'modified': file_time.strftime('%Y-%m-%d %H:%M:%S UTC'),
                                'download_url': f"/data/{urllib.parse.quote(rel_path)}"
                            })

            files.sort(key=lambda x: x['modified'], reverse=True)

            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()

            html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <title>RTCM Log Files</title>
                <style>
                    table {{ border-collapse: collapse; width: 100%; }}
                    th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
                    th {{ background-color: #f2f2f2; }}
                    .size {{ text-align: right; }}
                </style>
            </head>
            <body>
                <h1>RTCM Log Files</h1>
                <p><a href="/">← Back to Status</a> | <a href="/status">JSON Status</a></p>
                <p>Total files: {len(files)}</p>
                <table>
                    <tr>
                        <th>Filename</th>
                        <th>Size</th>
                        <th>Modified</th>
                        <th>Download</th>
                    </tr>
            """

            for file_info in files:
                html += f"""
                    <tr>
                        <td>{file_info['name']}</td>
                        <td class=\"size\">{file_info['size']:,} bytes</td>
                        <td>{file_info['modified']}</td>
                        <td><a href=\"{file_info['download_url']}\">Download</a></td>
                    </tr>
                """

            html += """
                </table>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
        except Exception as e:
            logging.error(f"Error listing log files: {e}")
            self.send_error(500, f"Server error: {str(e)}")

    def serve_log_file(self):
        try:
            filename = self.path[6:]  # remove '/data/'
            filename = urllib.parse.unquote(filename)

            if '..' in filename or filename.startswith('/'):
                self.send_error(403, "Access denied")
                return

            filepath = os.path.join(log_dir, filename)
            if not os.path.exists(filepath) or not os.path.isfile(filepath):
                self.send_error(404, "File not found")
                return

            file_size = os.path.getsize(filepath)
            content_type = mimetypes.guess_type(filepath)[0] or 'text/plain'

            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(file_size))
            self.send_header('Content-Disposition', f'attachment; filename="{os.path.basename(filename)}"')
            self.end_headers()
            with open(filepath, 'rb') as f:
                self.wfile.write(f.read())
        except Exception as e:
            logging.error(f"Error serving log file: {e}")
            self.send_error(500, f"Server error: {str(e)}")


class LogHTTPServer:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.server = None
        self.thread = None

        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('rtcm_log_server.log'),
                logging.StreamHandler()
            ]
        )

    def start(self):
        self.server = ThreadingHTTPServer((self.host, self.port), LogServerHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        logging.info(f"RTCM Log HTTP server started on {self.host}:{self.port}")
        logging.info(f"Endpoints: /status, /data/, /data/<file>")
        logging.info(f"Serving directory: {log_dir}")

    def stop(self):
        if self.server:
            logging.info("Stopping RTCM Log HTTP server...")
            self.server.shutdown()
            self.server.server_close()
            logging.info("RTCM Log HTTP server stopped")


def _nmea_checksum(sentence_no_dollar: str) -> str:
    csum = 0
    for ch in sentence_no_dollar:
        csum ^= ord(ch)
    return f"{csum:02X}"

def _deg_to_nmea_dm(value_deg: float, is_lat: bool) -> tuple[str, str]:
    hemi = 'N' if (is_lat and value_deg >= 0) else 'S' if is_lat else ('E' if value_deg >= 0 else 'W')
    abs_val = abs(value_deg)
    deg = int(abs_val)
    minutes = (abs_val - deg) * 60.0
    if is_lat:
        dm = f"{deg:02d}{minutes:07.4f}"
    else:
        dm = f"{deg:03d}{minutes:07.4f}"
    return dm, hemi

def build_gga_sentence(lat_deg: float, lon_deg: float, alt_m: float = 0.0, fix_quality: int = 1, num_sats: int = 12, hdop: float = 0.8) -> str:
    now = datetime.datetime.utcnow()
    timestr = now.strftime('%H%M%S')
    lat_dm, lat_hemi = _deg_to_nmea_dm(lat_deg, True)
    lon_dm, lon_hemi = _deg_to_nmea_dm(lon_deg, False)
    base = (
        f"GPGGA,{timestr},{lat_dm},{lat_hemi},{lon_dm},{lon_hemi},{fix_quality},{num_sats:02d},"
        f"{hdop:.1f},{alt_m:.1f},M,0.0,M,,"
    )
    checksum = _nmea_checksum(base)
    return f"${base}*{checksum}\r\n"


def run_continuous_logging():
    """Continuously connect to NTRIP caster, send GGA every 5s, and log all RTCM messages with daily rotation."""
    current_day_str = None
    log_file = None
    log_file_path = None

    def open_new_log_file():
        nonlocal current_day_str, log_file_path, log_file
        now = datetime.datetime.utcnow()
        current_day_str = now.strftime("%Y%m%d")
        day_folder = log_dir / current_day_str
        day_folder.mkdir(parents=True, exist_ok=True)
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        log_file_path = day_folder / f"rtcmbase_{timestamp}.log"
        log_file = open(log_file_path, 'w')
        log_file.write("# RTCM Message Log (all message types)\n")
        log_file.write(f"# Generated: {now.isoformat()} UTC\n")
        log_file.write(f"# Mountpoint: {mountpoint}\n")
        log_file.write(f"# GGA Position: {BASE_LAT_DEG}, {BASE_LON_DEG}\n")
        log_file.write("# Format: [unix_ms] [msg_type] [sat_prn] [length] [hex_data]\n")
        log_file.write("# ----------------------------------------\n")
        log_file.flush()

    while True:
        try:
            if log_file is None:
                open_new_log_file()

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((server, int(port)))

            try:
                pwd = getHTTPBasicAuthString(username, password)
                header = (
                    f"GET /{mountpoint} HTTP/1.0\r\n"
                    f"User-Agent: NTRIP basentriplogger/1.0\r\n"
                    f"Accept: */*\r\n"
                    f"Authorization: Basic {pwd}\r\n"
                    f"Ntrip-Version: Ntrip/2.0\r\n"
                    f"Connection: keep-alive\r\n\r\n"
                )
                s.sendall(header.encode('utf-8'))

                resp = s.recv(1024)
                print("Server response:")
                print(resp.decode('utf-8', errors='ignore'))

                if resp.startswith(b"STREAMTABLE"):
                    print("Invalid or No Mountpoint")
                    s.close()
                    time.sleep(5)
                    continue
                if not (resp.startswith(b"HTTP/1.1 200 OK") or resp.startswith(b"ICY 200 OK") or resp.startswith(b"HTTP/1.0 200 OK")):
                    print("Connection error: Unexpected response from server")
                    s.close()
                    time.sleep(5)
                    continue

                stop_event = threading.Event()

                def gga_sender():
                    # Wait for initial delay before sending first GGA
                    if args.gga_delay > 0:
                        stop_event.wait(args.gga_delay)

                    while not stop_event.is_set():
                        try:
                            gga = build_gga_sentence(BASE_LAT_DEG, BASE_LON_DEG)
                            s.sendall(gga.encode('ascii'))
                        except Exception:
                            pass
                        if args.gga_interval <= 0:
                            break
                        stop_event.wait(args.gga_interval)

                gga_thread = None
                if args.gga_interval > 0:
                    gga_thread = threading.Thread(target=gga_sender, daemon=True)
                    gga_thread.start()
                else:
                    print("GGA sending disabled (interval <= 0).")

                print("Connected; streaming and logging messages...")
                s.settimeout(1.0)
                while True:
                    try:
                        data = s.recv(4096)
                        if not data:
                            print("Connection closed by server")
                            break

                        messages = rtcm_parser.parse_rtcm_messages(data)
                        for msg in messages:
                            hex_data = msg['hex_data']
                            log_line = (
                                f"{msg['unix_ms']} {msg['msg_type']} "
                                f"{msg['sat_prn']} {msg['length']} {hex_data}\n"
                            )
                            try:
                                log_file.write(log_line)
                            except Exception:
                                pass

                        # Daily rotation
                        new_day_str = datetime.datetime.utcnow().strftime("%Y%m%d")
                        if new_day_str != current_day_str:
                            try:
                                log_file.flush()
                                log_file.close()
                            except Exception:
                                pass
                            open_new_log_file()

                    except socket.timeout:
                        continue

            finally:
                try:
                    stop_event.set()
                except Exception:
                    pass
                try:
                    s.close()
                except Exception:
                    pass
                try:
                    if log_file:
                        log_file.flush()
                except Exception:
                    pass
                print(f"Disconnected. Current log file: {log_file_path}")

            time.sleep(3)

        except KeyboardInterrupt:
            print("\nInterrupted by user; stopping logger")
            try:
                if log_file:
                    log_file.flush()
                    log_file.close()
            except Exception:
                pass
            break
        except Exception as e:
            print(f"Error in continuous logger: {e}")
            time.sleep(5)


# Start optional HTTP server, then run continuous logging
server_instance = None
if args.serve:
    try:
        server_instance = LogHTTPServer(args.host, args.http_port)
        server_instance.start()
    except Exception as e:
        print(f"Failed to start HTTP server: {e}")

try:
    run_continuous_logging()
finally:
    if server_instance:
        server_instance.stop()
