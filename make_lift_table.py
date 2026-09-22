#!/usr/bin/env python3
"""make_lift_table.py — precomputed joint solutions for supervisor corrections.
For each waypoint: solve IK at z+dz and x+dx offsets (±3.0 mm, 0.5 mm steps),
seeded and anchored at the taught joints (same convention as shift_joint_path's
own pipeline). Each solve FK-verified (<0.5 mm pos, <1.0 deg ori) else null.
Output: lift_table.json  {k: {"z": {"-3.0": deg6|null, ...}, "x": {...}}}
Usage: python3 make_lift_table.py pose_logs/shifted_clamp15_0813.jsonl
"""
import sys, json
import numpy as np
LOG = sys.argv[1]
import sjp_defs as sjp

path = [json.loads(l) for l in open(LOG) if l.strip()]
STEPS = [round(s, 1) for s in np.arange(-3.0, 3.01, 0.5)]
table, bad, tot = {}, 0, 0
for k, wp in enumerate(path):
    ang = wp["angles"]
    q_taught = np.zeros(sjp.NQ); q_taught[1:7] = np.radians(ang)
    T0 = sjp.fk_T(ang)
    entry = {"z": {}, "x": {}}
    for axis, idx in (("z", 2), ("x", 0)):
        for d in STEPS:
            tot += 1
            if d == 0.0:
                entry[axis]["0.0"] = [round(float(a), 2) for a in ang]
                continue
            T = T0.copy()
            T[idx, 3] += d / 1000.0
            q = sjp.solve_dls(T, q_taught.copy(), q_taught)
            T_chk = sjp.chain.forward_kinematics(q)
            pos_err = np.linalg.norm(T_chk[:3, 3] - T[:3, 3]) * 1000.0
            ori_err = np.degrees(np.arccos(np.clip(
                (np.trace(T_chk[:3, :3].T @ T[:3, :3]) - 1) / 2, -1, 1)))
            if pos_err < 0.5 and ori_err < 1.0:
                entry[axis][str(d)] = [round(float(v), 2)
                                       for v in np.degrees(q[1:7])]
            else:
                entry[axis][str(d)] = None
                bad += 1
    table[k] = entry
    if k % 25 == 0:
        print(f"waypoint {k}/{len(path)}  (unreachable so far: {bad})",
              flush=True)
json.dump(table, open("lift_table.json", "w"))
print(f"done: {len(path)} waypoints, {tot} solves, {bad} unreachable "
      f"({100*bad/tot:.1f}%) -> lift_table.json")
