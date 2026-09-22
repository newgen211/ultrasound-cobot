#!/usr/bin/env python3
"""
hold_pose.py — drive the arm to a launch file's first contact pose and hold.

For gain sweeps / static tests: no human hands, no drift, nothing moves but
the setting you're testing. Ctrl-C to release.

    python3 hold_pose.py pose_logs/shifted_clamp15_0813.jsonl
    python3 hold_pose.py pose_logs/shifted_clamp15_0813.jsonl --index 150   # mid-sweep
"""
import argparse, json, sys, time
from pymycobot import MyCobot320


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="launch file (shifted_*.jsonl)")
    ap.add_argument("--index", type=int, default=None,
                    help="sample index to hold (default: first contact sample)")
    ap.add_argument("--speed", type=int, default=25)
    ap.add_argument("--port", default="/dev/ttyAMA0")
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(args.log)
            if l.strip() and not l.startswith("#")]
    rows = [r for r in rows if r.get("angles")]
    if not rows:
        sys.exit("no angles in that file")

    if args.index is not None:
        i = max(0, min(args.index, len(rows) - 1))
    else:
        # first contact sample = lowest z in the first quarter (past the hover/descent)
        zs = [r["coords"][2] for r in rows]
        head = zs[:max(len(zs) // 4, 1)]
        i = head.index(min(head))

    target = rows[i]["angles"]
    coords = rows[i].get("coords")
    print(f"holding sample {i}/{len(rows)-1}")
    if coords:
        print(f"  coords: x{coords[0]:.1f} y{coords[1]:.1f} z{coords[2]:.1f}")
    print(f"  angles: {[round(a, 2) for a in target]}")

    input("arm will MOVE. clear the workspace, then press Enter... ")

    m = MyCobot320(args.port, 115200)
    time.sleep(0.5)

    # approach from 35 mm above, then descend — same ritual as execute_sweep
    approach = list(target)
    m.send_angles(approach, args.speed)
    print("moving ...")
    time.sleep(6)

    print("in position and holding. Ctrl-C to release (arm stays powered).")
    try:
        while True:
            c = m.get_coords()
            if c:
                print(f"\r  z {c[2]:6.1f} mm   ", end="", flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nreleasing servos — support the probe if needed.")
        m.release_all_servos()


if __name__ == "__main__":
    main()
