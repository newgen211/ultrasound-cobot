#!/usr/bin/env python3
"""apply_offsets.py — execute_sweep's proven flow + supervisor z-offsets.
Reuses execute_sweep's PoseLogger and wait_until_stopped; same hover ->
settle -> countdown -> timestamp-paced play (rate MULTIPLIES duration).
Listens on :5006 for {"dz": mm}; --supervise on swaps waypoint angles from
lift_table.json (clamped toward 0 to nearest reachable). off = log only.
Usage: python3 apply_offsets.py pose_logs/shifted_clamp15_0813.jsonl \
         --speed 25 --rate 2 --delay 45 --supervise off
"""
import argparse, json, socket, threading, time, sys
from datetime import datetime
from pathlib import Path
import execute_sweep as es

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--speed", type=int, default=25)
ap.add_argument("--start-speed", type=int, default=20)
ap.add_argument("--rate", type=float, default=2.0)
ap.add_argument("--delay", type=float, default=None)
ap.add_argument("--supervise", choices=["on", "off"], default="off")
ap.add_argument("--listen-port", type=int, default=5006)
ap.add_argument("--table", default="lift_table.json")
ap.add_argument("--no-home", action="store_true")
args = ap.parse_args()

# load with ORIGINAL indices (lift_table is keyed on raw line number)
rows = []
for k, line in enumerate(open(args.log)):
    line = line.strip()
    if not line: continue
    d = json.loads(line)
    if d.get("angles") and len(d["angles"]) == 6:
        rows.append((k, d["t_ns"], d["angles"]))
if len(rows) < 2: sys.exit("not enough poses")
# dedupe consecutive near-identical, keeping indices
kept = [rows[0]]
for r in rows[1:]:
    if max(abs(a-b) for a, b in zip(r[2], kept[-1][2])) > 0.05:
        kept.append(r)
if kept[-1] is not rows[-1]: kept.append(rows[-1])
table = json.load(open(args.table))

dz_target = 0.0
_dz_lock = threading.Lock()
def listener():
    global dz_target
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.listen_port)); srv.listen(1)
    while True:
        conn, _ = srv.accept(); buf = b""
        with conn:
            while True:
                d = conn.recv(1024)
                if not d: break
                buf += d
                while b"\n" in buf:
                    ln, buf = buf.split(b"\n", 1)
                    try:
                        with _dz_lock:
                            dz_target = max(-1.5, min(3.0,
                                float(json.loads(ln).get("dz", 0.0))))
                    except Exception: pass
threading.Thread(target=listener, daemon=True).start()

def clamp_to_table(k, dz):
    entry = table.get(str(k))
    base = next(a for kk, t, a in kept if kk == k)
    if entry is None: return 0.0, base
    z = entry["z"]
    d = max(-3.0, min(3.0, round(round(dz/0.5)*0.5, 1)))
    while True:
        a = z.get(str(d))
        if a is not None: return d, a
        if d == 0.0: return 0.0, base
        d = round(d-0.5, 1) if d > 0 else round(d+0.5, 1)

dur = (kept[-1][1] - kept[0][1]) / 1e9
print(f"{len(kept)} poses, recorded {dur:.0f}s -> playback "
      f"~{dur*args.rate:.0f}s at rate {args.rate}x  supervise={args.supervise}")
input("Arm will MOVE. Clear workspace, press Enter... ")

mc = es.MyCobot320(es.PORT, es.BAUD)
lock = threading.Lock()
with lock: mc.power_on()
time.sleep(0.5)
plog = es.PoseLogger(mc, lock,
        f"pose_logs/replay_{datetime.now():%Y%m%d_%H%M%S}.jsonl")
plog.start()
runlog = open(f"pose_logs/supervised_{datetime.now():%Y%m%d_%H%M%S}.jsonl", "w")

print("-> hover (start pose, slow)...")
with lock: mc.send_angles([float(a) for a in kept[0][2]], args.start_speed)
es.wait_until_stopped(mc, lock)
if args.delay is not None:
    print(f"playing in {args.delay:.0f}s"); time.sleep(args.delay)
else:
    input("Press Enter to play... ")

try:
    t_prev = None
    for k, t, ang in kept:
        with _dz_lock: dz_req = dz_target
        if args.supervise == "on":
            dz_app, target = clamp_to_table(k, dz_req)
        else:
            dz_app, target = 0.0, ang
        with lock: mc.send_angles([float(a) for a in target], args.speed)
        runlog.write(json.dumps(dict(k=k, t_ns=t, dz_req=round(dz_req, 2),
                     dz_app=dz_app, applied=args.supervise == "on")) + "\n")
        if t_prev is not None:
            time.sleep(max(min((t - t_prev)/1e9*args.rate, 2.0), 0.02))
        t_prev = t
finally:
    runlog.close()
    if not args.no_home:
        print("-> homing...")
        with lock: mc.send_angles([0,0,0,0,0,0], 20)
        es.wait_until_stopped(mc, lock)
    plog.stop()
    print(f"done. pose samples logged: {plog.count}")
