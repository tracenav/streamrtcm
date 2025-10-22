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

# Default server settings
# script is for ephemeris logging
# logging 1046 GAL, 1019 GPS, 1042 BDS, 1020 GLO
DEFAULT_SERVER = 'ntrip.data.gnss.ga.gov.au'
DEFAULT_PORT = '2101'
DEFAULT_MOUNTPOINT = 'BCEP00BKG0'
DEFAULT_USERNAME = 'jacob222'
DEFAULT_PASSWORD = 'Gilmer2284!'

# Logging settings
LOG_DIRECTORY = 'rtcm_logs'

# Session settings
SESSION_INTERVAL_SECONDS = 300  # Connect every 5 minutes
DATA_COLLECTION_WINDOW_MS = 10000  # Collect for 5 minutes per session, then write ordered log

# Allowed ephemeris message types and preferred output ordering
ORDERED_MSG_TYPES = [1019, 1020, 1042, 1046]
ALLOWED_MSG_TYPES = set(ORDERED_MSG_TYPES)
ORDER_INDEX = {mt: idx for idx, mt in enumerate(ORDERED_MSG_TYPES)}

# Default expected PRN ranges to ensure completeness before writing
# You can override with --expected "1019:1-32;1020:1-27;1042:1-63;1046:1-36" (or a subset)
DEFAULT_EXPECTED_SPEC = '1019:1-32;1020:1-27;1042:1-63;1046:1-36'

# Command line arguments
parser = argparse.ArgumentParser(description='NTRIP ephemeris logger (.log only) + optional HTTP server')
parser.add_argument('--server', default=DEFAULT_SERVER, help='NTRIP server')
parser.add_argument('--port', default=DEFAULT_PORT, help='NTRIP port')
parser.add_argument('--mountpoint', default=DEFAULT_MOUNTPOINT, help='Mountpoint')
parser.add_argument('--username', default=DEFAULT_USERNAME, help='Username')
parser.add_argument('--password', default=DEFAULT_PASSWORD, help='Password')
parser.add_argument('--logdir', default=LOG_DIRECTORY, help='Directory for RTCM log files')
parser.add_argument('--serve', action='store_true', help='Serve log files over HTTP while logging')
parser.add_argument('--host', default='127.0.0.1', help='HTTP server bind host (default: 0.0.0.0)')
parser.add_argument('--http-port', type=int, default=8081, help='HTTP server port (default: 8081)')
parser.add_argument('--expected', default=DEFAULT_EXPECTED_SPEC, help='Expected PRNs per message type for early drop. Format: 1019:1-32;1020:1-27;1042:1-63;1046:1-36')
parser.add_argument('--expected-timeout-ms', type=int, default=None, help='Max time to wait before flushing (ms). Default: capture window duration')

args = parser.parse_args()

class RTCMParser:
    def __init__(self):
        self.buffer = bytearray()
        # Epoch for per-line timestamps (ms since session 'Generated' time)
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

print("RTCM Ephemeris Logger (.log only)")
print(f"NTRIP server: {server}:{port}")
print(f"Mountpoint: {mountpoint}")
print(f"Logging directory: {args.logdir}")
print(f"Allowed message types: {sorted(ALLOWED_MSG_TYPES)}")
print(f"Session cadence: every {SESSION_INTERVAL_SECONDS} seconds; collect {DATA_COLLECTION_WINDOW_MS} ms after data starts")


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
            <title>RTCM Ephemeris Logger</title>
            <meta http-equiv="refresh" content="30">
        </head>
        <body>
            <h1>RTCM Ephemeris Logger</h1>
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


def parse_expected_spec(spec: str) -> dict:
    """Parse expected PRN spec string into a dict: {msg_type: set(prns)}
    Format: '1019:1-32;1046:1-36;1042:1-63;1020:1-27'
    Ranges can be comma-separated lists and ranges, e.g., '1,3,5-7'.
    """
    result: dict[int, set[int]] = {}
    if not spec:
        return result
    groups = [g.strip() for g in spec.split(';') if g.strip()]
    for group in groups:
        try:
            msg_str, prn_str = group.split(':', 1)
            msg_type = int(msg_str.strip())
            prn_set: set[int] = set()
            for token in [t.strip() for t in prn_str.split(',') if t.strip()]:
                if '-' in token:
                    start_s, end_s = token.split('-', 1)
                    start, end = int(start_s), int(end_s)
                    if start <= end:
                        prn_set.update(range(start, end + 1))
                    else:
                        prn_set.update(range(end, start + 1))
                else:
                    prn_set.add(int(token))
            if prn_set:
                result[msg_type] = prn_set
        except Exception:
            # Ignore malformed groups
            continue
    return result


def connect_and_log_one_session():
    """Connect to NTRIP server, log allowed ephemeris messages to a .log file.
    Connect until data starts flowing, then collect for DATA_COLLECTION_WINDOW_MS and drop.
    """
    # Create a new log file per session inside a daily folder (YYYYMMDD)
    # Use UTC for folder/file naming
    now = datetime.datetime.utcnow()
    current_day_str = now.strftime("%Y%m%d")
    day_folder = log_dir / current_day_str
    day_folder.mkdir(parents=True, exist_ok=True)
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    log_file_path = day_folder / f"rtcm_{timestamp}.log"
    log_file = open(log_file_path, 'w')

    try:
        # Helpers to write header and flush buffers deterministically
        def write_log_header(f):
            f.write("# RTCM Message Log (ephemeris only)\n")
            # Generated time in UTC
            f.write(f"# Generated: {datetime.datetime.utcnow().isoformat()} UTC\n")
            f.write(f"# Mountpoint: {mountpoint}\n")
            f.write(f"# Allowed types: {sorted(ALLOWED_MSG_TYPES)}\n")
            f.write("# GPS Week: 0\n")
            f.write("# GPS TOW: 0ms\n")
            f.write("# Format: [timestamp_ms] [msg_type] [sat_prn] [length] [hex_data]\n")
            f.write("# ----------------------------------------\n")
            f.flush()

        def flush_buffer_to_file(f, buffers):
            for mt in ORDERED_MSG_TYPES:
                entries = buffers.get(mt, [])
                entries.sort(key=lambda t: t[0])
                for _, line in entries:
                    f.write(line)
            f.flush()

        # Write initial header and reset parser epoch so timestamps are ms since 'Generated'
        write_log_header(log_file)
        try:
            rtcm_parser.start_time = time.time()
        except Exception:
            pass

        # Create socket connection
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(None)  # Block until data arrives
        s.connect((server, int(port)))

        try:
                # Create HTTP request
                pwd = getHTTPBasicAuthString(username, password)
                header = (f"GET /{mountpoint} HTTP/1.0\r\n"
                          f"User-Agent: NTRIP RTCMLogger\r\n"
                          f"Accept: */*\r\n"
                          f"Authorization: Basic {pwd}\r\n"
                          f"Connection: close\r\n\r\n")

                s.sendto(header.encode('utf-8'), (server, int(port)))

                # Read and validate server response
                resp = s.recv(1024)
                print("Server response:")
                print(resp.decode('utf-8', errors='ignore'))

                if resp.startswith(b"STREAMTABLE"):
                    print("Invalid or No Mountpoint")
                    return False
                elif not (resp.startswith(b"HTTP/1.1 200 OK") or resp.startswith(b"ICY 200 OK") or resp.startswith(b"HTTP/1.0 200 OK")):
                    print("Connection error: Unexpected response from server")
                    return False

                print("Connection successful - waiting for data...")

                data_started_at = None
                expected_spec = parse_expected_spec(args.expected) if args.expected else {}
                # Early-drop condition: if expected set is provided and fully seen, drop before full window
                # Deadline defaults to the capture window if not provided
                expected_deadline_ms = None
                if expected_spec:
                    expected_deadline_ms = (
                        args.expected_timeout_ms
                        if (args.expected_timeout_ms is not None and args.expected_timeout_ms > 0)
                        else DATA_COLLECTION_WINDOW_MS
                    )
                # Track duplicates and completeness
                seen_hex_messages: set[str] = set()
                seen_by_type: dict[int, set[int]] = {mt: set() for mt in expected_spec.keys()}
                # Buffer by type for ordered output; store (timestamp_ms, line)
                buffered_by_type: dict[int, list[tuple[int, str]]] = {mt: [] for mt in ORDERED_MSG_TYPES}

                while True:
                    try:
                        data = s.recv(4096)
                        if not data:
                            print("Connection closed by server before data start")
                            break

                        # Mark the moment data starts
                        if data_started_at is None:
                            data_started_at = time.time()
                            # After data starts, switch to short timeouts so we can exit on time
                            s.settimeout(0.2)
                            mode_note = (
                                f"waiting for expected set ({args.expected}) or {int((expected_deadline_ms or DATA_COLLECTION_WINDOW_MS)/1000)} s max"
                                if expected_spec else
                                f"{int(DATA_COLLECTION_WINDOW_MS/1000)} s window"
                            )
                            print("Data started; collecting:", mode_note)

                        # Parse RTCM messages and log only allowed types
                        messages = rtcm_parser.parse_rtcm_messages(data)
                        for msg in messages:
                            if msg['msg_type'] in ALLOWED_MSG_TYPES:
                                # Deduplicate by full message hex
                                hex_data = msg['hex_data']
                                if hex_data in seen_hex_messages:
                                    continue
                                seen_hex_messages.add(hex_data)

                                # Timestamp in ms since session 'Generated' time (hard value; no suffix)
                                log_line = (
                                    f"{msg['timestamp_ms']} {msg['msg_type']} "
                                    f"{msg['sat_prn']} {msg['length']} {hex_data}\n"
                                )
                                # Always buffer; flush later in deterministic order
                                buffered_by_type[msg['msg_type']].append((msg['timestamp_ms'], log_line))
                                # Track PRNs per expected type
                                if expected_spec and msg['msg_type'] in expected_spec and msg['sat_prn']:
                                    seen_by_type.setdefault(msg['msg_type'], set()).add(msg['sat_prn'])

                        # Handle day rollover (rotate file at midnight)
                        # Rotate at UTC midnight
                        new_day_str = datetime.datetime.utcnow().strftime("%Y%m%d")
                        if data_started_at is not None and new_day_str != current_day_str:
                            # Flush current buffers to the old day's file, then rotate
                            flush_buffer_to_file(log_file, buffered_by_type)
                            for mt in ORDERED_MSG_TYPES:
                                buffered_by_type[mt] = []
                            try:
                                log_file.close()
                            except Exception:
                                pass
                            current_day_str = new_day_str
                            new_day_folder = log_dir / current_day_str
                            new_day_folder.mkdir(parents=True, exist_ok=True)
                            timestamp = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
                            log_file_path = new_day_folder / f"rtcm_{timestamp}.log"
                            log_file = open(log_file_path, 'w')
                            write_log_header(log_file)
                            print(f"Date changed; rotated log to {log_file_path}")

                        # Determine completion condition
                        if expected_spec:
                            # Check if all expected PRNs have been seen for each listed msg type
                            complete = all(
                                expected_prns.issubset(seen_by_type.get(mt, set()))
                                for mt, expected_prns in expected_spec.items()
                            )
                            elapsed_ms = int((time.time() - data_started_at) * 1000) if data_started_at else 0
                            if complete:
                                # Flush buffered lines in deterministic order by message type and timestamp
                                flush_buffer_to_file(log_file, buffered_by_type)
                                print("All expected PRNs received; closing session")
                                break
                            if expected_deadline_ms is not None and data_started_at is not None and elapsed_ms >= expected_deadline_ms:
                                # Flush whatever we have in deterministic order and end session on timeout
                                flush_buffer_to_file(log_file, buffered_by_type)
                                print("Expected PRN timeout reached; closing session")
                                break
                        else:
                            # No expected list: use time window, but ensure ordered flush
                            if data_started_at is not None:
                                elapsed_ms = int((time.time() - data_started_at) * 1000)
                                if elapsed_ms >= DATA_COLLECTION_WINDOW_MS:
                                    # Flush buffered in deterministic order and exit
                                    flush_buffer_to_file(log_file, buffered_by_type)
                                    print("Data collection window reached; closing session")
                                    break

                    except socket.timeout:
                        # After data start, check completion/timeouts
                        if data_started_at is not None:
                            if expected_spec:
                                complete = all(
                                    expected_prns.issubset(seen_by_type.get(mt, set()))
                                    for mt, expected_prns in expected_spec.items()
                                )
                                elapsed_ms = int((time.time() - data_started_at) * 1000)
                                if complete or (expected_deadline_ms is not None and elapsed_ms >= expected_deadline_ms):
                                    # Flush buffered in deterministic order and exit
                                    flush_buffer_to_file(log_file, buffered_by_type)
                                    print("Ending session (expected set satisfied or timeout)")
                                    break
                                continue
                            else:
                                elapsed_ms = int((time.time() - data_started_at) * 1000)
                                if elapsed_ms >= DATA_COLLECTION_WINDOW_MS:
                                    # Flush buffered in deterministic order and exit
                                    flush_buffer_to_file(log_file, buffered_by_type)
                                    print("Data collection window reached during idle; closing session")
                                    break
                                continue
                        # Before data start we block with no timeout; shouldn't happen
                        continue
        finally:
            s.close()
            try:
                log_file.close()
            except Exception:
                pass
            print(f"Closed connection. Log saved: {log_file_path}")

        return True

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return False
    except Exception as e:
        print(f"Connection/session failed: {e}")
        return False


# Periodic session loop (every 5 minutes)
# Optionally start HTTP server
server_instance = None
if args.serve:
    try:
        server_instance = LogHTTPServer(args.host, args.http_port)
        server_instance.start()
    except Exception as e:
        print(f"Failed to start HTTP server: {e}")

while True:
    session_start = time.time()
    try:
        connect_and_log_one_session()
    except KeyboardInterrupt:
        print("\nInterrupted by user; exiting")
        if server_instance:
            server_instance.stop()
        break
    except Exception as e:
        print(f"Unexpected error: {e}")

    # Sleep until next 5-minute cycle
    elapsed = time.time() - session_start
    sleep_seconds = max(0, SESSION_INTERVAL_SECONDS - elapsed)
    if sleep_seconds > 0:
        print(f"Next session in {int(sleep_seconds)} seconds...")
        try:
            time.sleep(sleep_seconds)
        except KeyboardInterrupt:
            print("\nInterrupted during wait; exiting")
            if server_instance:
                server_instance.stop()
            break