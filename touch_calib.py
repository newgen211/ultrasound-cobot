#!/usr/bin/env python3
"""
touch_calib.py — repeated TCP-touch of desk markers id1/id2, with checks.

Run on the Pi. Drag-touch the probe tip onto each marker's pencil-cross
center, N rounds. Keeps the wrist orientation honest (warns if it drifts
between the id1 and id2 touch of a round), prints per-round baselines,
and finishes with averaged positions as a paste-ready JSON block.
"""

import time
import numpy as np
from pymycobot import MyCobot320

N_ROUNDS = 4
ORI_WARN_DEG = 8.0     # wrist drift within a round that triggers a warning
BASE_WARN_MM = 2.0     # baseline spread across rounds that triggers a warning

def euler_xyz_to_R(rx, ry, rz):
    a, b, c = np.radians([rx, ry, rz])
    Rx = np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
    Ry = np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    Rz = np.array([[np.cos(c),-np.sin(c),0],[np.sin(c),np.cos(c),0],[0,0,1]])
    return Rx @ Ry @ Rz

def rot_between_deg(c1, c2):
    """True relative rotation angle — immune to gimbal lock, unlike
    comparing rx/ry/rz per component (degenerate when ry ~ +-90)."""
    R = euler_xyz_to_R(*c1[3:]).T @ euler_xyz_to_R(*c2[3:])
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))

m = MyCobot320('/dev/ttyAMA0', 115200)
m.power_on(); time.sleep(1)
m.release_all_servos()
print("servos released — drag freely. Same wrist orientation all session:\n"
      "park on id1, then TRANSLATE to id2 without twisting.\n")

p1s, p2s, baselines = [], [], []

for r in range(1, N_ROUNDS + 1):
    input(f"[round {r}/{N_ROUNDS}] tip on id1 center, Enter...")
    c1 = m.get_coords()
    print(f"  id1: {c1}")
    input(f"[round {r}/{N_ROUNDS}] tip on id2 center, Enter...")
    c2 = m.get_coords()
    print(f"  id2: {c2}")

    ori_drift = rot_between_deg(c1, c2)
    if ori_drift > ORI_WARN_DEG:
        print(f"  !! wrist rotated {ori_drift:.1f} deg between touches — "
              f"redo this round (translate only).")
        continue

    p1, p2 = np.array(c1[:3]), np.array(c2[:3])
    b = float(np.linalg.norm(p2 - p1))
    print(f"  baseline: {b:.1f} mm  (dz {p2[2]-p1[2]:+.1f} mm)")
    p1s.append(p1); p2s.append(p2); baselines.append(b)

if len(baselines) < 2:
    raise SystemExit("not enough clean rounds — rerun.")

b = np.array(baselines)
print(f"\nbaselines: {[f'{x:.1f}' for x in baselines]}")
print(f"mean {b.mean():.2f} mm, spread {b.max()-b.min():.2f} mm"
      + ("  !! spread > %.1f mm — touches inconsistent, consider rerunning"
         % BASE_WARN_MM if b.max()-b.min() > BASE_WARN_MM else "  — clean"))

a1, a2 = np.mean(p1s, axis=0), np.mean(p2s, axis=0)
print("\npaste into guided_sweep_calib.json:")
print(f'''  "desk_markers_base_mm": {{
    "1": [{a1[0]:.1f}, {a1[1]:.1f}, {a1[2]:.1f}],
    "2": [{a2[0]:.1f}, {a2[1]:.1f}, {a2[2]:.1f}]
  }}''')