#!/usr/bin/env python3
"""
level_teach.py — clamp a taught sweep's depth to the touched phantom surface.

The hand presses in while establishing contact; the press is worst a few cm
into the drag, where shift_joint_path's start-lift taper has already faded.
This snaps each sample's z to (interpolated surface line - max_indent), but
only where the teach went DEEPER than that. Shallow samples are untouched.

Writes edited coords + untouched coords_raw, so shift_joint_path.py aligns on
the true angle<->coord pairs (it reads coords_raw) and only the targets move.

    python3 level_teach.py pose_logs/sweep_teach_up2.jsonl \
        --max-indent 1.5 --out pose_logs/sweep_teach_clamp15.jsonl
"""
import argparse, json
import numpy as np

# touched phantom surface, TCP frame: start -> end of sweep
SURF_START = (201.9, 186.5, 233.3)
SURF_END   = (80.7, 194.7, 237.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--max-indent", type=float, default=1.5,
                    help="mm the probe may sink below the surface line")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    p1 = np.array(SURF_START, float)
    p2 = np.array(SURF_END, float)
    axis = (p2 - p1)[:2]
    span = float(np.linalg.norm(axis))
    axis = axis / span

    rows = [json.loads(l) for l in open(args.log) if l.strip()]
    edits, deepest = [], 0.0
    for r in rows:
        c = r.get("coords")
        if not c:
            continue
        r.setdefault("coords_raw", list(c))
        t = float(np.dot(np.array(c[:2]) - p1[:2], axis)) / span
        t = min(max(t, 0.0), 1.0)
        floor = p1[2] + t * (p2[2] - p1[2]) - args.max_indent
        deepest = max(deepest, floor + args.max_indent - c[2])
        if c[2] < floor:
            edits.append(floor - c[2])
            c[2] = round(floor, 1)

    with open(args.out, "w") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")

    n = len(edits)
    print(f"{len(rows)} samples, {n} clamped ({n/max(len(rows),1):.0%})")
    if n:
        print(f"lift applied: median {np.median(edits):.1f} mm, max {max(edits):.1f} mm")
    print(f"deepest taught indent was {deepest:.1f} mm -> now capped at {args.max_indent} mm")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
