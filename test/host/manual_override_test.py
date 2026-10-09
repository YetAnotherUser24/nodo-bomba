#!/usr/bin/env python3
"""
Manual-override workflow test for the virtual nodes against the real backend.

Reproduces exactly what the dashboard does when the operator plans to run an
experiment mostly by hand:

  1. Set the orchestrator to MANUAL_OVERRIDE  -> broadcastState() enqueues a
     `set_state` to every node that must abort recipes and force actuators OFF.
  2. Confirm recipe actions are rejected (HTTP 409) in this state.
  3. Send direct operator commands (allowed in MANUAL_OVERRIDE):
        sensor : manual_sample
        odrive : set_speed / emergency_stop / clear_estop
        mixer  : start_mixer (mixer_rpm) / stop_mixer
        pump   : dose (target_ml)
  4. Restore IDLE.

It captures device serial in parallel and reports received/ACK per command.

Usage:
  manual_override_test.py --port /dev/ttyACM0
  manual_override_test.py --port /dev/ttyACM0 --only odrive
"""
import argparse
import json
import sys
import threading
import time
import urllib.error
import urllib.request

try:
    import serial
except ImportError:
    print("pyserial missing; use /root/.platformio/penv/bin/python")
    sys.exit(2)

BASE = "https://tesisutec.vercel.app"
API = BASE + "/api/commands"
STATE = BASE + "/api/system/state"

DEVICES = {
    "sensor": "a0000000-0000-0000-0000-000000000001",
    "odrive": "b0000000-0000-0000-0000-000000000002",
    "mixer": "c0000000-0000-0000-0000-000000000003",
    "pump": "d0000000-0000-0000-0000-000000000004",
}


def _post(url, payload, timeout=15):
    body = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=body,
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode("utf-8", "replace")


def set_state(state, updated_by="manual-override-test"):
    return _post(STATE, {"state": state, "updated_by": updated_by})


def get_state():
    with urllib.request.urlopen(STATE, timeout=15) as r:
        return json.loads(r.read().decode())


def enqueue(device_id, action, command_type, payload=None):
    return _post(API, {
        "device_id": device_id,
        "command_type": command_type,
        "payload": {"action": action, **(payload or {})},
        "requested_by": "manual-override-test",
    })


def open_port(port, retries=40):
    for _ in range(retries):
        try:
            return serial.Serial(port, 115200, timeout=0.3)
        except Exception:  # noqa: BLE001
            time.sleep(0.5)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--only", default="", help="comma list of roles to exercise")
    args = ap.parse_args()

    roles = [r for r in (args.only.split(",") if args.only else DEVICES) if r in DEVICES]

    ser = open_port(args.port)
    if ser is None:
        print("could not open serial")
        sys.exit(1)

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

    results = []

    def check(action, device_role, tag):
        hits = [l for l in captured if tag in l or (action in l and "[CMD]" in l)]
        results.append((device_role, action, "received" if hits else "no_receive"))

    print("=== Manual-override workflow test ===")

    # Step 1: enter MANUAL_OVERRIDE (broadcasts set_state to all nodes).
    st, body = set_state("MANUAL_OVERRIDE")
    print(f"[1] set MANUAL_OVERRIDE -> HTTP {st}")
    time.sleep(12)  # let every node poll its set_state and force actuators OFF
    check("set_state", "all", "MANUAL_OVERRIDE")

    # Step 2: a recipe action must be rejected with 409.
    st, body = enqueue(DEVICES["sensor"], "start_monitor", "start")
    print(f"[2] recipe start_monitor in MANUAL_OVERRIDE -> HTTP {st} "
          f"{'(rejected as expected)' if st == 409 else '(NOT rejected!)'}")

    # Step 3: direct operator commands are allowed.
    plan = {
        "sensor": [("manual_sample", "start", None)],
        "odrive": [("start", "set_speed", {"speed_percent": 40}),
                   ("emergency_stop", "emergency_stop", None),
                   ("clear_estop", "set_speed", None)],
        "mixer": [("start_mixer", "start", {"mixer": "on", "mixer_rpm": 900}),
                  ("stop_mixer", "stop", {"mixer": "off"})],
        "pump": [("dose", "set_speed", {"target_ml": 3.0})],
    }
    for role in roles:
        for action, ctype, payload in plan.get(role, []):
            st, body = enqueue(DEVICES[role], action, ctype, payload)
            print(f"[3] {role:6s} {action:16s} -> HTTP {st}")
            time.sleep(9)
            check(action, role, "[ACK]")

    # Step 4: restore IDLE.
    st, body = set_state("IDLE")
    print(f"[4] restore IDLE -> HTTP {st}")
    time.sleep(8)

    stop = True
    time.sleep(0.5)
    try:
        ser.close()
    except Exception:
        pass

    print("\n=== SERIAL HIGHLIGHTS ===")
    for line in captured:
        if any(k in line for k in ("[CMD]", "[ACK]", "[MASTER]", "MANUAL",
                                   "EMERGENCY", "Target", "Cloud", "Mixer", "Dosing")):
            print("DEV>", line)

    print("\n=== SUMMARY ===")
    for role, action, r in results:
        print(f"  {role:8s} {action:16s} {r}")


if __name__ == "__main__":
    main()
