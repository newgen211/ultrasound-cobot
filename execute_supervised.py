#!/usr/bin/env python3
"""execute_supervised.py — play a joint path; apply supervisor z-offsets
from lift_table.json. THIS MOVES THE ARM. Air test first.
Listens on TCP :5006 for {"dz": <mm>} (absolute target offset, mm).
--supervise off: listen + LOG but never apply (E5 baseline arm).
Clamp rule (Blaze): walk requested dz toward 0 in 0.5 steps until the
table has a reachable entry at this waypoint; apply that; log both.
Usage: python3 execute_supervised.py pose_logs/shifted_clamp15_0813.jsonl \
         --speed 25 --rate 2 --delay 45 --supervise on
"""
import argparse, json, socket, threading, time, sys
from datetime import datetime
from pathlib import Path
from pymycobot import MyCobot320

PORT_SERIAL, BAUD = "/dev/ttyAMA0", 115200

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--speed", type=int, default=25)
ap.add_argument("--rate", type=float, default=2.0)
ap.add_argument("--delay", type=float, default=0.0)
ap.add_argument("--supervise", choices=["on", "off"], default="off")
ap.add_argument("--listen-port", type=int, default=5006)
ap.add_argument("--table", default="lift_table.json")
args = ap.parse_args()

path = [json.loads(l) for l in open(args.log) if l.strip()]
table = json.load(open(args.table))
STEPS = [round(-3.0 + 0.5*i, 1) for i in range(13)]

dz_target = 0.0
_lock = threading.Lock()

def listener():
    global dz_target
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.listen_port)); srv.listen(1)
    while True:
        conn, _ = srv.accept()
        buf = b""
        with conn:
            while True:
                d = conn.recv(1024)
                if not d: break
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        msg = json.loads(line)
                        with _lock:
                            dz_target = max(-1.5, min(3.0,
                                            float(msg.get("dz", 0.0))))
                    except Exception:
                        pass

threading.Thread(target=listener, daemon=True).start()

def clamp_to_table(k, dz):
    """Nearest reachable step toward 0 at waypoint k. Returns (dz_app, ang)."""
    entry = table.get(str(k))
    if entry is None: return 0.0, path[k]["angles"]
    z = entry["z"]
    d = round(round(dz / 0.5) * 0.5, 1)
    d = max(-3.0, min(3.0, d))
    while True:
        a = z.get(str(d))
        if a is not None: return d, a
        if d == 0.0: return 0.0, path[k]["angles"]
        d = round(d - 0.5, 1) if d > 0 else round(d + 0.5, 1)

mc = MyCobot320(PORT_SERIAL, BAUD)
mc.power_on(); time.sleep(0.5)
runlog = open(f"pose_logs/supervised_{datetime.now():%Y%m%d_%H%M%S}.jsonl", "w")
print(f"supervise={args.supervise}  waypoints={len(path)}  "
      f"listening :{args.listen_port}")
if args.delay: print(f"starting in {args.delay:.0f}s"); time.sleep(args.delay)

try:
    t_prev = None
    for k, wp in enumerate(path):
        with _lock: dz_req = dz_target
        dz_app, ang = (clamp_to_table(k, dz_req) if args.supervise == "on"
                       else (0.0, wp["angles"]))
        mc.send_angles([float(a) for a in ang], args.speed)
        runlog.write(json.dumps(dict(k=k, t_ns=wp.get("t_ns"),
                     dz_req=round(dz_req, 2), dz_app=dz_app,
                     applied=args.supervise == "on")) + "\n")
        if k % 20 == 0:
            print(f"wp {k}/{len(path)}  dz_req={dz_req:+.2f} "
                  f"dz_app={dz_app:+.1f}", flush=True)
        t = wp.get("t_ns")
        if t_prev is not None and t is not None:
            dt = (t - t_prev) / 1e9 / args.rate
            time.sleep(max(min(dt, 1.0), 0.02))
        else:
            time.sleep(0.1)
        t_prev = t
finally:
    runlog.close()
    print("done — sweep finished, servos left on; home separately.")
