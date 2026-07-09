#!/usr/bin/env python3
"""
make_waypoints_from_log.py — taught-path replay with a vision anchor.

Takes the tracked manual sweep (pose_logger jsonl) and the vision-derived
start x/y (from guided_sweep_reader's waypoint 0), and emits waypoints that
ARE the taught path — z profile, per-station orientation, everything —
rigidly translated in x/y so the path lands on the container wherever the
camera says it currently sits.

Why: every pose in the taught path is reachable by construction (the arm
was physically there), the orientation bends where the joint limits demand
it (J4 clamps at -82 mid-path — constant orientation is infeasible there),
and contact stays light because the teaching hand kept it light.

    python make_waypoints_from_log.py sweep_teach.jsonl \
        --start-x 175.0 --start-y 210.2 --spacing 5 > waypoints.jsonl
"""

import argparse, json, sys
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("log", help="pose_logger jsonl of the taught sweep")
ap.add_argument("--anchor", default=None,
                help="vision_anchor.json from the reader (default: "
                "src/calibration/vision_anchor.json, then ./vision_anchor.json)")
ap.add_argument("--start-x", type=float, default=None,
                help="override: vision waypoint-0 x")
ap.add_argument("--start-y", type=float, default=None,
                help="override: vision waypoint-0 y")
ap.add_argument("--spacing", type=float, default=5.0,
                help="waypoint spacing along the path (mm)")
ap.add_argument("--max-ang", type=float, default=2.0,
                help="max orientation change per station (deg) — densifies "
                "stations where the wrist works hard (J5 backlash crossing "
                "at x~155 needs tiny steps, not smooth ones)")
ap.add_argument("--z-lift", type=float, default=0.0,
                help="add to every z: compensates the arm's gravity droop "
                "(free-space run measured ~7 mm settle-below-command)")
ap.add_argument("--smooth", type=int, default=15,
                help="moving-average window (samples, ~90ms each) over all "
                "6 DOF — removes hand jitter/corrections from the teach "
                "(same fix as pose stair-stepping); 0 disables")
ap.add_argument("--speed", type=int, default=15)
args = ap.parse_args()

# resolve the anchor: explicit --start-x/y wins, else read the reader's file
if args.start_x is None or args.start_y is None:
    import os
    candidates = ([args.anchor] if args.anchor else
                  ["src/calibration/vision_anchor.json", "vision_anchor.json",
                   "../src/calibration/vision_anchor.json"])
    for cpath in candidates:
        if cpath and os.path.exists(cpath):
            a = json.load(open(cpath))
            args.start_x, args.start_y = a["start_x"], a["start_y"]
            print(f"# anchor from {cpath} (written {a.get('written','?')}): "
                  f"start {args.start_x}, {args.start_y}", file=sys.stderr)
            break
    else:
        sys.exit("no vision anchor found — run guided_sweep_reader first "
                 "(it writes vision_anchor.json), or pass --start-x/--start-y.")

recs = [json.loads(l) for l in open(args.log) if l.strip()]
P = np.array([r["coords"] for r in recs], dtype=np.float64)
J5 = np.array([r["angles"][4] for r in recs], dtype=np.float64) \
     if "angles" in recs[0] else None
if len(P) < 10:
    sys.exit("log too short")

if args.smooth > 1:
    from scipy.ndimage import uniform_filter1d
    P = uniform_filter1d(P, size=args.smooth, axis=0, mode="nearest")

# subsample by travelled distance OR orientation change — a new station
# whenever we've gone --spacing mm in xy, or the wrist has rotated --max-ang
# degrees (whichever first). Dense steps through the J5-reversal region.
xy = P[:, :2]
d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
picks = [0]
for i in range(1, len(P)):
    d_xy = d[i] - d[picks[-1]]
    d_ang = np.max(np.abs(P[i, 3:] - P[picks[-1], 3:]))
    if d_xy >= args.spacing or (d_ang >= args.max_ang and d_xy >= 0.5):
        picks.append(i)
if picks[-1] != len(P) - 1:
    picks.append(len(P) - 1)

dx = args.start_x - P[0, 0]
dy = args.start_y - P[0, 1]
print(f"# taught path: {len(P)} samples, {d[-1]:.0f} mm travel; "
      f"{len(picks)} waypoints; vision shift dx={dx:+.1f} dy={dy:+.1f}",
      file=sys.stderr)

n_hops = 0
prev = picks[0]
for i in picks:
    c = P[i].copy()
    c[0] += dx
    c[1] += dy
    c[2] += args.z_lift
    coords = [round(float(v), 2) for v in c]
    wp = {"coords": coords, "speed": args.speed}
    # J5 sign change since the last station => backlash dead-zone crossing.
    # Flag it so the sender HOPS: crosses the reversal in free air, not
    # while pressed into the phantom.
    if J5 is not None and i != prev and (J5[prev] < 0) != (J5[i] < 0):
        wp["hop"] = True
        n_hops += 1
    prev = i
    print(json.dumps(wp))
if n_hops:
    print(f"# {n_hops} hop station(s) flagged (J5 zero-crossing)", file=sys.stderr)