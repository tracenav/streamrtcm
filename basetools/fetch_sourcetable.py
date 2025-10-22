#!/usr/bin/env python3

import argparse
import socket
from typing import Tuple


def build_request() -> bytes:
    # Minimal NTRIP sourcetable request. Many casters accept HTTP/1.0.
    # Include Ntrip-Version header for wider compatibility.
    request_lines = [
        "GET / HTTP/1.0",
        "User-Agent: NTRIP PythonClient/1.0",
        "Accept: */*",
        "Ntrip-Version: Ntrip/2.0",
        "Connection: close",
        "",
        "",
    ]
    return ("\r\n".join(request_lines)).encode("ascii")


def fetch_sourcetable(host: str, port: int = 2101, timeout_seconds: int = 10) -> Tuple[str, bytes]:
    """
    Connects to an NTRIP caster and retrieves the sourcetable bytes.

    Returns a tuple of (server_banner, sourcetable_bytes).
    Raises exceptions on connection or protocol errors.
    """
    with socket.create_connection((host, port), timeout=timeout_seconds) as sock:
        sock.sendall(build_request())
        chunks = []
        while True:
            data = sock.recv(4096)
            if not data:
                break
            chunks.append(data)

    raw = b"".join(chunks)

    # Split headers and body
    header_sep = b"\r\n\r\n"
    if header_sep in raw:
        header_bytes, body = raw.split(header_sep, 1)
    else:
        # Some casters send just plain text without HTTP headers (rare)
        header_bytes, body = b"", raw

    header_text = header_bytes.decode("latin1", errors="replace")

    # Validate that this looks like a sourcetable
    # Common starts: "SOURCETABLE 200 OK" or standard HTTP status + body containing SOURCETABLE entries
    if not body:
        raise RuntimeError("Empty response body from caster; no sourcetable received.")

    return header_text, body


def main():
    parser = argparse.ArgumentParser(description="Fetch NTRIP SOURCETABLE and save to a text file.")
    parser.add_argument("--host", default="caster.godigifarm.com", help="NTRIP caster hostname or IP")
    parser.add_argument("--port", type=int, default=2101, help="NTRIP caster port (default: 2101)")
    parser.add_argument("--out", default="sourcetable.txt", help="Output file path")
    parser.add_argument("--timeout", type=int, default=10, help="Socket timeout seconds")
    args = parser.parse_args()

    header_text, body = fetch_sourcetable(args.host, args.port, args.timeout)

    # Write raw body to file exactly as provided by caster
    with open(args.out, "wb") as f:
        f.write(body)

    print(f"Saved sourcetable to {args.out}")
    if header_text:
        # Print a short banner so the user can see status at a glance
        first_header_line = header_text.splitlines()[0] if header_text.splitlines() else ""
        if first_header_line:
            print(f"Server response: {first_header_line}")


if __name__ == "__main__":
    main()


