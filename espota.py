#!/usr/bin/env python3
"""
espota.py  —  WiFi OTA tool for esp32-aprs-modem
Implements the espota protocol (UDP handshake + TCP binary transfer, port 3232).

Usage:
    python espota.py -i <ESP32_IP> [-f <firmware.bin>]

If -f is omitted, defaults to build/esp32-aprs-modem.bin.

Examples:
    python espota.py -i 192.168.1.50
    python espota.py -i 192.168.1.50 -f build/esp32-aprs-modem.bin
"""

import argparse
import hashlib
import os
import socket
import sys
import time

ESPOTA_PORT   = 3232
CHUNK         = 1024          # bytes per TCP write (matches device-side ack granularity)
UDP_TIMEOUT   = 10            # seconds waiting for UDP handshake response
TCP_TIMEOUT   = 120           # seconds for entire TCP transfer
FINAL_TIMEOUT = 30            # seconds waiting for final "OK\n" after transfer


def ota(ip, port, firmware):
    # ------------------------------------------------------------------
    # Read firmware and compute MD5
    # ------------------------------------------------------------------
    if not os.path.isfile(firmware):
        print(f"ERROR: file not found: {firmware}")
        return False

    with open(firmware, "rb") as fh:
        data = fh.read()

    size = len(data)
    md5  = hashlib.md5(data).hexdigest()
    print(f"Target  : {ip}:{port}")
    print(f"Firmware: {firmware}  ({size} bytes)")
    print(f"MD5     : {md5}")

    # ------------------------------------------------------------------
    # Phase 1 — UDP handshake
    # ------------------------------------------------------------------
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.settimeout(UDP_TIMEOUT)
    msg = f"0 {size} {md5}".encode()

    print("\nSending OTA request...", end=" ", flush=True)
    udp.sendto(msg, (ip, port))

    try:
        resp, _ = udp.recvfrom(128)
    except socket.timeout:
        print("TIMEOUT")
        print("ERROR: no response from device.")
        print("  • Check that the ESP32 is running and connected to WiFi.")
        print("  • Check the IP address.")
        print(f"  • Verify UDP port {port} is not blocked by a firewall.")
        udp.close()
        return False
    finally:
        udp.close()

    if b"OK" not in resp.upper():
        print(f"FAILED ({resp!r})")
        return False

    print("OK")

    # ------------------------------------------------------------------
    # Phase 2 — TCP binary transfer
    # ------------------------------------------------------------------
    print(f"Connecting TCP to {ip}:{port}...", end=" ", flush=True)
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp.settimeout(TCP_TIMEOUT)
    try:
        tcp.connect((ip, port))
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f"FAILED ({e})")
        return False
    print("OK")
    print("Uploading...")

    t0   = time.monotonic()
    sent = 0

    try:
        while sent < size:
            chunk = data[sent : sent + CHUNK]
            tcp.sendall(chunk)
            sent += len(chunk)

            pct = sent * 100 // size
            bar = "=" * (pct // 2) + "." * (50 - pct // 2)
            elapsed = time.monotonic() - t0
            kbps    = sent / elapsed / 1024 if elapsed > 0 else 0
            print(f"\r  [{bar}] {pct:3d}%  {sent//1024}/{size//1024} KB  {kbps:.0f} KB/s",
                  end="", flush=True)
    except (socket.timeout, BrokenPipeError, OSError) as e:
        print(f"\nERROR during transfer: {e}")
        tcp.close()
        return False

    print()
    elapsed = time.monotonic() - t0
    kbps    = size / elapsed / 1024 if elapsed > 0 else 0
    print(f"Sent {size} bytes in {elapsed:.1f}s ({kbps:.0f} KB/s)")

    # ------------------------------------------------------------------
    # Phase 3 — read final result  (drain 'O' progress bytes, find "OK")
    # ------------------------------------------------------------------
    print("Waiting for device confirmation...", end=" ", flush=True)
    tcp.settimeout(FINAL_TIMEOUT)
    result = b""
    try:
        while True:
            chunk = tcp.recv(256)
            if not chunk:
                break
            result += chunk
            if b"OK" in result:
                break
    except socket.timeout:
        pass
    finally:
        tcp.close()

    if b"OK" in result:
        print("OK")
        print("OTA successful! Device is rebooting...")
        return True
    else:
        print(f"FAILED (response: {result!r})")
        print("The firmware may not have passed MD5 verification.")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="WiFi OTA for esp32-aprs-modem (espota protocol, port 3232)"
    )
    parser.add_argument("-i", "--ip",   required=True,
                        help="ESP32 IP address")
    parser.add_argument("-p", "--port", type=int, default=ESPOTA_PORT,
                        help=f"espota port (default {ESPOTA_PORT})")
    parser.add_argument("-f", "--file", default="build/esp32-aprs-modem.bin",
                        help="firmware .bin  (default: build/esp32-aprs-modem.bin)")
    args = parser.parse_args()

    ok = ota(args.ip, args.port, args.file)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
