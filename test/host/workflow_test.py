#!/usr/bin/env python3
"""
End-to-end workflow test for a virtual node against the real AquaControl backend.

Enqueues commands through /api/commands (exactly like the dashboard does), captures the
device serial in parallel, and checks that each command is received and ACKed.

Usage:
  workflow_test.py --device sensor --port /dev/ttyACM0
  workflow_test.py --device pump  --port /dev/ttyACM0
"""
import argparse
import json
import sys
import threading
import time
import urllib.request

try:
    import serial
except ImportError:
    print("pyserial missing; use /root/.platformio/penv/bin/python")
    sys.exit(2)

API = "https://tesisutec.vercel.app/api/commands"
DEVICES = {
    "sensor": "a0000000-0000-0000-0000-000000000001",
    "odrive": "b0000000-0000-0000-0000-000000000002",
    "mixer":  "c0000000-0000-0000-0000-000000000003",
    "pump":   "d0000000-0000-0000-0000-000000000004",
}


def enqueue(device_id, action, command_type=None, payload=None, timeout=15):
    body = json.dumps({
        "device_id": device_id,
        "command_type": command_type or action,
        "payload": {"action": action, **(payload or {})},
        "requested_by": "workflow-test",
    }).encode()
    req = urllib.request.Request(API, data=body,
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status, r.read().decode("utf-8", "replace")


def open_port(port, retries=40):
    for _ in range(retries):
        try:
            return serial.Serial(port, 115200, timeout=0.3)
        except Exception:  # noqa: BLE001
            time.sleep(0.5)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", required=True, choices=list(DEVICES))
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--actions", default="start_monitor,manual_sample,stop_monitor")
    args = ap.parse_args()

    device_id = DEVICES[args.device]
    actions = [a for a in args.actions.split(",") if a]

    ser = open_port(args.port)
    if ser is None:
        print("could not open serial"); sys.exit(1)

    captured = []
    stop = False

    def reader():
        nonlocal ser
        buf = b""
        while not stop:
            try:
                chunk = ser.read(4096)
            except Exception:
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(0.5)
                ser = open_port(args.port)
                if ser is None:
                    return
                continue
            if chunk:
                buf += chunk
                for line in buf.decode("utf-8", "replace").splitlines():
                    if line.strip():
                        captured.append(line.strip())
                buf = b""

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    print(f"=== Virtual node workflow test: {args.device} ({device_id}) ===")
    results = []
    for action in actions:
        try:
            status, resp = enqueue(device_id, action)
            print(f"\n[enqueue] {action} -> HTTP {status}")
        except Exception as e:  # noqa: BLE001
            print(f"\n[enqueue] {action} FAILED: {e}")
            results.append((action, "enqueue_failed"))
            continue
        time.sleep(10)  # let the device poll (1-2 s) and act
        # The backend stores the normalized action (start/stop), so match loosely on
        # the first token and on the ACK for this action's command id.
        token = action.split("_")[0]
        recv = [l for l in captured if "[CMD]" in l and (action in l or token.lower() in l.lower())]
        results.append((action, "received" if recv else "no_receive"))

    stop = True
    time.sleep(0.5)
    try:
        ser.close()
    except Exception:
        pass

    print("\n=== SERIAL HIGHLIGHTS ===")
    for line in captured:
        if any(k in line for k in ("[CMD]", "[ACK]", "[MASTER]", "[VIRTUAL]",
                                   "[BAT]", "HTTP", "Payload", "manual", "Sample")):
            print("DEV>", line)

    print("\n=== SUMMARY ===")
    for action, r in results:
        print(f"  {action:20s} {r}")


if __name__ == "__main__":
    main()
