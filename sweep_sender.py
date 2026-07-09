#!/usr/bin/env python3
"""
sweep_sender.py — stream a guided sweep with proper approach choreography.

Motion plan (instead of raw target list):
  1. HOVER: joint move (mode 0) to a point APPROACH_Z above waypoint 0 —
     free-space repositioning, no straight-line constraint, no phantom bumps.
  2. DESCEND: short vertical linear hops (mode 1) down onto waypoint 0.
  3. SWEEP: the waypoints as small linear hops (mode 1).
  4. RETRACT: straight up APPROACH_Z, then done.

Requires the receiver to honor an optional "mode" field
(send_coords(coords, speed, msg.get("mode", 1))).

    python sweep_sender.py waypoints.jsonl --host 192.168.196.134 --step
"""

import argparse, json, socket, sys, time

ap = argparse.ArgumentParser()
ap.add_argument("waypoints", help="file of JSON lines from the reader")
ap.add_argument("--host", default="192.168.196.134")
ap.add_argument("--port", type=int, default=5005)
ap.add_argument("--speed", type=int, default=15)
ap.add_argument("--approach-z", type=float, default=40.0,
                help="hover height above waypoint 0 (mm)")
ap.add_argument("--descend-step", type=float, default=8.0,
                help="vertical descent step size (mm)")
ap.add_argument("--dwell", type=float, default=1.0,
                help="hold at each sweep waypoint after ack (s)")
ap.add_argument("--step", action="store_true",
                help="require Enter before each move (first-run mode)")
args = ap.parse_args()

wps = [json.loads(l) for l in open(args.waypoints) if l.strip().startswith("{")]
if not wps:
    sys.exit("no waypoint lines found")

def send(coords, mode, label):
    if args.step:
        input(f"  Enter to send {label}: {[round(c,1) for c in coords[:3]]} "
              f"(mode {mode}) ...")
    msg = json.dumps({"coords": coords, "speed": args.speed, "mode": mode}) + "\n"
    with socket.create_connection((args.host, args.port), timeout=60) as s:
        s.sendall(msg.encode())
        ack = s.makefile("r").readline().strip()
    warn = ""
    try:
        now = json.loads(ack).get("coords_now")
        if now:
            dev = sum((a - b) ** 2 for a, b in zip(now[:3], coords[:3])) ** 0.5
            if dev > 6.0:
                warn = f"  !! {dev:.1f} mm from target — arm may not have moved (mode-1 no-op?)"
    except Exception:
        pass
    print(f"  {label}: {ack}{warn}")
    if "REJECT" in ack.upper():
        sys.exit("receiver rejected — stopping.")

start = wps[0]["coords"]
hover = [start[0], start[1], start[2] + args.approach_z] + start[3:]

print("1/4 HOVER (joint move, free path)")
send(hover, 0, "hover")
time.sleep(1.5)                       # let the joint move fully settle

print("2/4 DESCEND (vertical linear hops)")
z = start[2] + args.approach_z
while z - args.descend_step > start[2]:
    z -= args.descend_step
    send([start[0], start[1], z] + start[3:], 1, f"descend z={z:.1f}")
send(start, 1, "touchdown")
time.sleep(1.0)

print(f"3/4 SWEEP ({len(wps)-1} stations)")
HOP_Z = 15.0
prev_coords = start
for k, wp in enumerate(wps[1:], start=2):
    tgt = wp["coords"]
    if wp.get("hop"):
        # backlash crossing here — do it in FREE AIR, not against the phantom:
        # lift straight up, translate+rotate airborne, descend onto target.
        print(f"  [hop] wp {k}: crossing J5 dead zone unloaded")
        up_here = [prev_coords[0], prev_coords[1], prev_coords[2] + HOP_Z] + list(prev_coords[3:])
        send(up_here, 1, f"wp {k} hop-lift")
        over_tgt = [tgt[0], tgt[1], tgt[2] + HOP_Z] + list(tgt[3:])
        send(over_tgt, 1, f"wp {k} hop-move")
        time.sleep(0.8)                    # let the reversal settle in air
        send(tgt, 1, f"wp {k} hop-land")
    else:
        send(tgt, 1, f"wp {k}/{len(wps)}")
    prev_coords = tgt
    time.sleep(args.dwell)

print("4/4 RETRACT (straight up)")
last = wps[-1]["coords"]
send([last[0], last[1], last[2] + args.approach_z] + last[3:], 1, "retract")

print("sweep complete.")