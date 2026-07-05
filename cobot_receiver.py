#!/usr/bin/env python3
"""
cobot_receiver.py — Pi-side command receiver for the guided sweep

Runs ON THE PI. Opens a TCP socket, waits for target poses (sent by the Mac's
camera/planning code, or by the local test sender), validates each pose is inside
a sane workspace envelope, then drives the arm there with send_coords. Sends an
ack back so the sender knows the move finished.

The Mac does all the heavy vision (4 markers, transforms) and sends only the
distilled result: one target pose. This receiver stays thin: receive -> validate
-> move -> ack.

Protocol (newline-delimited JSON, one message per line):
    IN : {"coords": [x,y,z,rx,ry,rz], "speed": 40}
    OUT: {"ok": true,  "coords_now": [...]}            # moved OK
    OUT: {"ok": false, "error": "reason"}              # rejected / failed

Test it with NO Mac: run this, then in a second Pi terminal run cobot_test_sender.py.

    python3 cobot_receiver.py                 # listen on 0.0.0.0:5005
    python3 cobot_receiver.py --port 5005 --dry-run   # validate + print, DON'T move

Ctrl-C powers servos off cleanly.
"""

import argparse
import json
import socket
import sys
import time

from pymycobot import MyCobot320

PORT_SERIAL = "/dev/ttyAMA0"
BAUD = 115200

# --- SAFETY: workspace envelope (mm / deg). Reject any target outside this.
# A garbage pose from a network glitch must NOT be sent to the arm. Tune to your
# real reachable volume — these are deliberately conservative starting bounds.
X_MIN, X_MAX = -300.0, 300.0
Y_MIN, Y_MAX = -300.0, 300.0
Z_MIN, Z_MAX =    50.0, 500.0      # keep off the table (z>50) and within reach
RX_MIN, RX_MAX = -180.0, 180.0
RY_MIN, RY_MAX = -180.0, 180.0
RZ_MIN, RZ_MAX = -180.0, 180.0
SPEED_MIN, SPEED_MAX = 1, 60       # cap speed; never let a message command a wild rate
MOVE_TIMEOUT_S = 20.0


def valid_pose(coords, speed):
    """Return (ok, reason). Rejects malformed or out-of-envelope targets."""
    if not isinstance(coords, (list, tuple)) or len(coords) != 6:
        return False, "coords must be 6 numbers [x,y,z,rx,ry,rz]"
    try:
        x, y, z, rx, ry, rz = (float(v) for v in coords)
    except (TypeError, ValueError):
        return False, "coords must all be numbers"
    bounds = [
        ("x", x, X_MIN, X_MAX), ("y", y, Y_MIN, Y_MAX), ("z", z, Z_MIN, Z_MAX),
        ("rx", rx, RX_MIN, RX_MAX), ("ry", ry, RY_MIN, RY_MAX), ("rz", rz, RZ_MIN, RZ_MAX),
    ]
    for name, val, lo, hi in bounds:
        if not (lo <= val <= hi):
            return False, f"{name}={val:.1f} outside [{lo},{hi}]"
    if not (SPEED_MIN <= speed <= SPEED_MAX):
        return False, f"speed={speed} outside [{SPEED_MIN},{SPEED_MAX}]"
    return True, "ok"


def wait_until_stopped(mc, timeout_s=MOVE_TIMEOUT_S):
    t0 = time.time()
    time.sleep(0.4)
    while time.time() - t0 < timeout_s:
        try:
            if not mc.is_moving():
                return
        except Exception:
            return
        time.sleep(0.1)


def handle_message(mc, msg, dry_run):
    """Parse one JSON message, validate, move. Returns a response dict."""
    try:
        req = json.loads(msg)
    except json.JSONDecodeError:
        return {"ok": False, "error": "bad JSON"}
    coords = req.get("coords")
    speed = int(req.get("speed", 30))

    ok, reason = valid_pose(coords, speed)
    if not ok:
        print(f"  REJECT: {reason}  ({coords})")
        return {"ok": False, "error": reason}

    coords = [float(v) for v in coords]
    print(f"  target OK -> move to {[round(v,1) for v in coords]} @ speed {speed}"
          f"{'  [DRY-RUN, not moving]' if dry_run else ''}")
    if dry_run:
        return {"ok": True, "dry_run": True}

    try:
        mc.send_coords(coords, speed, 1)      # mode 1 = linear (confirmed working on this arm)
        wait_until_stopped(mc)
        now = mc.get_coords()
    except Exception as e:
        return {"ok": False, "error": f"move failed: {e}"}
    return {"ok": True, "coords_now": now}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0", help="bind address (default all interfaces)")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--dry-run", action="store_true", help="validate + print, do NOT move the arm")
    args = ap.parse_args()

    if args.dry_run:
        mc = None
        print("DRY-RUN: not connecting to the arm.")
    else:
        mc = MyCobot320(PORT_SERIAL, BAUD)
        time.sleep(1)
        mc.power_on()                 # engage servos so send_coords actually moves the arm
        time.sleep(1)
        print(f"connected to arm on {PORT_SERIAL} (servos on)")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"listening on {args.host}:{args.port}  (Ctrl-C to quit)")

    try:
        while True:
            conn, addr = srv.accept()
            print(f"\nconnected: {addr[0]}:{addr[1]}")
            buf = ""
            with conn:
                f = conn.makefile("rwb")
                while True:
                    line = f.readline()
                    if not line:
                        print("  client disconnected")
                        break
                    msg = line.decode("utf-8").strip()
                    if not msg:
                        continue
                    resp = handle_message(mc, msg, args.dry_run)
                    f.write((json.dumps(resp) + "\n").encode("utf-8"))
                    f.flush()
    except KeyboardInterrupt:
        print("\nshutting down...")
    finally:
        srv.close()
        if mc is not None:
            try:
                mc.power_off()      # servos off — arm goes limp, safe resting state
                print("servos powered off.")
            except Exception:
                pass


if __name__ == "__main__":
    main()