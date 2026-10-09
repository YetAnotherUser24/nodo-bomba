#!/usr/bin/env python3
"""
Resilient serial capture/command helper for the AquaControl virtual nodes.

Handles the USB-CDC quirk where closing/opening the port (or a chip reset)
momentarily drops and re-enumerates the device: it retries open, and keeps
reading across reconnects.

Usage:
  serial_probe.py capture --port /dev/ttyACM0 --seconds 20 [--filter k1,k2]
  serial_probe.py send --port /dev/ttyACM0 --cmd "STATUS" --seconds 8
  serial_probe.py watch --port /dev/ttyACM0 --seconds 30 --filter VIRTUAL,HTTP
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial not available; run with /root/.platformio/penv/bin/python")
    sys.exit(2)


def open_port(port, retries=20, delay=0.5):
    last = None
    for _ in range(retries):
        try:
            s = serial.Serial(port, 115200, timeout=0.3)
            # DTR/RTS toggle is flaky on USB-CDC; ignore errors.
            try:
                s.setDTR(False)
                s.setRTS(True)
                time.sleep(0.1)
                s.setRTS(False)
            except Exception:
                pass
            return s
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(delay)
    print(f"could not open {port}: {last}")
    return None


def pump(ser, seconds, filters, sink):
    end = time.time() + seconds
    buf = b""
    while time.time() < end:
        try:
            chunk = ser.read(4096)
        except Exception:
            # Device dropped (reset/re-enumerate): try to reopen.
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(0.5)
            ser = open_port(ser.portstr, retries=40)
            if ser is None:
                return None
            continue
        if chunk:
            buf += chunk
            text = buf.decode("utf-8", "replace")
            for line in text.splitlines():
                line = line.strip()
                if not line:
                    continue
                if filters and not any(f in line for f in filters):
                    continue
                sink(line)
            buf = b""
    return ser


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["capture", "send", "watch"])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--cmd", default="")
    ap.add_argument("--filter", default="")
    args = ap.parse_args()

    filters = [f for f in args.filter.split(",") if f]
    seen = []
    ser = open_port(args.port)
    if ser is None:
        sys.exit(1)

    if args.mode == "send" and args.cmd:
        ser.write((args.cmd + "\r\n").encode())
        time.sleep(0.2)

    def sink(line):
        seen.append(line)
        print(line)

    ser = pump(ser, args.seconds, filters, sink)
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass
    print(f"\n[{len(seen)} matching lines]")


if __name__ == "__main__":
    main()
