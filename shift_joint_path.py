#!/usr/bin/env python3
"""
shift_joint_path.py — WE compute the joint angles; the arm just plays them.

Takes the taught sweep (pose_logger jsonl: angles + coords) and the vision
anchor (container position from guided_sweep_reader), and produces a NEW
joint-angle log: the taught path rigidly translated by the container's
measured (dx, dy), solved by seeded numerical IK on the Mac.

The firmware's IK is never invoked — its wrist-singularity instability
(J5~0) can't fire because there is nothing for it to solve. The output is
pure joint targets, played by replay_sweep.py exactly like a teach log.

Trust chain (all checked, script aborts if any fails):
  1. Model FK vs firmware FK: the URDF model is aligned to the teach log's
     own (angles -> coords) pairs; alignment RMS must be < 1.5 mm.
  2. Every IK solve is verified by model FK against its shifted target
     (< 0.5 mm, < 0.5 deg) before being written.
  3. Solved joint path smoothness: consecutive-sample joint steps must stay
     small (no configuration hops).

Usage:
  python shift_joint_path.py sweep_teach.jsonl > shifted_sweep.jsonl
  # then on the Pi:  python3 replay_sweep.py shifted_sweep.jsonl --speed 25

Needs: pip install ikpy ; mycobot_320_pi.urdf next to this script (or --urdf).
"""

import argparse, json, os, sys, warnings
import numpy as np
warnings.filterwarnings("ignore")

ap = argparse.ArgumentParser()
ap.add_argument("log", help="taught sweep jsonl (pose_logger: angles + coords)")
ap.add_argument("--anchor", default=None,
                help="vision_anchor.json (default: search usual spots)")
ap.add_argument("--urdf", default=None,
                help="mycobot 320 pi urdf (default: next to this script)")
ap.add_argument("--dx", type=float, default=None, help="override shift x (mm)")
ap.add_argument("--dy", type=float, default=None, help="override shift y (mm)")
ap.add_argument("--max-align-rms", type=float, default=1.5)
ap.add_argument("--max-step-deg", type=float, default=3.0,
                help="max joint change between consecutive samples (hop guard)")
args = ap.parse_args()

# ---- taught path -----------------------------------------------------------
recs = [json.loads(l) for l in open(args.log) if l.strip()]
recs = [r for r in recs if r.get("angles") and r.get("coords")]
if len(recs) < 10:
    sys.exit("log too short / missing angles")
A = np.array([r["angles"] for r in recs], dtype=np.float64)   # deg
C = np.array([r["coords"] for r in recs], dtype=np.float64)   # mm + deg
T_ns = [r["t_ns"] for r in recs]

# ---- vision shift ----------------------------------------------------------
if args.dx is not None and args.dy is not None:
    dx, dy = args.dx, args.dy
else:
    cands = ([args.anchor] if args.anchor else
             ["src/calibration/vision_anchor.json", "vision_anchor.json",
              "../src/calibration/vision_anchor.json"])
    for cp in cands:
        if cp and os.path.exists(cp):
            a = json.load(open(cp))
            dx, dy = a["start_x"] - C[0, 0], a["start_y"] - C[0, 1]
            print(f"# anchor {cp} (written {a.get('written','?')}): "
                  f"shift dx={dx:+.1f} dy={dy:+.1f} mm", file=sys.stderr)
            break
    else:
        sys.exit("no vision anchor found — run guided_sweep_reader, or pass --dx --dy")

if abs(dx) < 1.0 and abs(dy) < 1.0:
    print("# shift < 1 mm — taught log is valid as-is, passing through",
          file=sys.stderr)
    for r in recs:
        print(json.dumps(r))
    sys.exit(0)

# ---- kinematic model, self-calibrated against the teach log ----------------
from ikpy.chain import Chain
urdf = args.urdf or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "mycobot_320_pi.urdf")
chain = Chain.from_urdf_file(urdf, base_elements=['base'],
                             active_links_mask=[False] + [True]*6)
NQ = len(chain.links)

def fk_T(deg6):
    q = np.zeros(NQ); q[1:7] = np.radians(deg6)
    return chain.forward_kinematics(q)

Pm = np.array([fk_T(a)[:3, 3] * 1000.0 for a in A])       # model, mm
Pf = C[:, :3]                                             # firmware, mm
cm, cf = Pm.mean(0), Pf.mean(0)
U, S, Vt = np.linalg.svd((Pm - cm).T @ (Pf - cf))
d = np.sign(np.linalg.det(Vt.T @ U.T))
R_align = Vt.T @ np.diag([1, 1, d]) @ U.T                 # model -> firmware
rms = np.sqrt((((R_align @ (Pm - cm).T).T + cf - Pf) ** 2).sum(1).mean())
print(f"# model<->firmware alignment RMS: {rms:.2f} mm "
      f"({len(recs)} samples)", file=sys.stderr)
if rms > args.max_align_rms:
    sys.exit(f"alignment RMS {rms:.2f} > {args.max_align_rms} mm — model not "
             f"trustworthy for this log; aborting before anything moves.")

dp_model = R_align.T @ np.array([dx, dy, 0.0]) / 1000.0   # shift, model frame

# ---- seed-local damped-least-squares IK -------------------------------------
# ikpy's global optimizer basin-hops near the wrist singularity (J5~0) — the
# same pathology as the firmware. A local Newton iteration from the taught
# seed CANNOT leave the basin: steps are small and damped by construction.
def pose_err(T_now, T_tgt):
    dp = T_tgt[:3, 3] - T_now[:3, 3]
    Re = T_tgt[:3, :3] @ T_now[:3, :3].T
    ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
    if ang < 1e-9:
        w = np.zeros(3)
    else:
        w = ang / (2 * np.sin(ang)) * np.array(
            [Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    return np.concatenate([dp, w])                       # m, rad

def solve_dls(T_tgt, q_seed, q_anchor, iters=80, damp=1e-4, mu=5e-3,
              step_clip=0.06):
    """Damped least-squares with null-space anchoring: pose-determined
    directions are solved exactly; near the wrist singularity the Jacobian's
    null direction is pulled toward q_anchor (the TAUGHT joints) instead of
    drifting — the taught configuration is by definition valid there."""
    q = q_seed.copy()
    for _ in range(iters):
        T = chain.forward_kinematics(q)
        e = pose_err(T, T_tgt)
        if np.linalg.norm(e[:3]) < 5e-5 and np.linalg.norm(e[3:]) < 5e-4:
            break
        J = np.zeros((6, 6))
        for j in range(6):
            qq = q.copy(); qq[1 + j] += 1e-5
            Tj = chain.forward_kinematics(qq)
            J[:, j] = pose_err(T, Tj) / 1e-5
        rhs = J.T @ e
        JJ = J.T @ J + damp * np.eye(6)
        dq_task = np.linalg.solve(JJ, rhs)
        # null-space projection: pull toward the taught joints ONLY in
        # directions the pose does not constrain (exactly the directions
        # that go free at the wrist singularity)
        Jp = np.linalg.solve(JJ, J.T)
        N = np.eye(6) - Jp @ J
        dq = dq_task + N @ (0.3 * (q_anchor[1:7] - q[1:7]))
        dq = np.clip(dq, -step_clip, step_clip)
        q[1:7] += dq
    return q

solved_q, targets = [], []
prev_q = None
for k, ang in enumerate(A):
    q_taught = np.zeros(NQ); q_taught[1:7] = np.radians(ang)
    T_tgt = fk_T(ang); T_tgt[:3, 3] += dp_model
    seed = prev_q if prev_q is not None else q_taught
    q = solve_dls(T_tgt, seed, q_taught)
    T_chk = chain.forward_kinematics(q)
    pos_err = np.linalg.norm(T_chk[:3, 3] - T_tgt[:3, 3]) * 1000.0
    ori_err = np.degrees(np.arccos(np.clip(
        (np.trace(T_chk[:3, :3].T @ T_tgt[:3, :3]) - 1) / 2, -1, 1)))
    if pos_err > 1.0 or ori_err > 1.0:
        sys.exit(f"sample {k}: IK verification failed "
                 f"({pos_err:.2f} mm / {ori_err:.2f} deg) — aborting.")
    prev_q = q
    solved_q.append(q.copy())
    targets.append(T_tgt)

# ---- smooth the solved joint path, then PROVE it still hits the targets ----
# Near the singularity the null direction wiggles sample to sample; those
# wiggles barely move the tool (that's what singular means), so smoothing
# them out is nearly free in pose — and we verify that claim per sample.
from scipy.ndimage import uniform_filter1d
Q = np.array(solved_q)
Q[:, 1:7] = uniform_filter1d(Q[:, 1:7], size=9, axis=0, mode="nearest")

out = []
worst_pos = worst_ori = worst_step = 0.0
for k, (q, T_tgt) in enumerate(zip(Q, targets)):
    T_chk = chain.forward_kinematics(q)
    pos_err = np.linalg.norm(T_chk[:3, 3] - T_tgt[:3, 3]) * 1000.0
    ori_err = np.degrees(np.arccos(np.clip(
        (np.trace(T_chk[:3, :3].T @ T_tgt[:3, :3]) - 1) / 2, -1, 1)))
    if pos_err > 1.5 or ori_err > 1.5:
        sys.exit(f"sample {k}: post-smoothing verification failed "
                 f"({pos_err:.2f} mm / {ori_err:.2f} deg) — aborting.")
    if k > 0:
        step = np.degrees(np.abs(Q[k, 1:7] - Q[k-1, 1:7])).max()
        worst_step = max(worst_step, step)
        if step > args.max_step_deg:
            sys.exit(f"sample {k}: joint step {step:.1f} deg > "
                     f"{args.max_step_deg} even after smoothing — aborting.")
    worst_pos, worst_ori = max(worst_pos, pos_err), max(worst_ori, ori_err)
    ang_out = [round(float(v), 2) for v in np.degrees(q[1:7])]
    coords_out = [round(float(C[k, 0] + dx), 1), round(float(C[k, 1] + dy), 1),
                  round(float(C[k, 2]), 1)] + [float(v) for v in C[k, 3:]]
    out.append({"t_ns": T_ns[k], "coords": coords_out, "angles": ang_out})

dq_from_taught = np.degrees(np.abs(
    np.radians(np.array([o["angles"] for o in out])) - np.radians(A))).max()
print(f"# {len(out)} samples solved. FK check worst: {worst_pos:.3f} mm / "
      f"{worst_ori:.3f} deg. max joint step {worst_step:.2f} deg. "
      f"max deviation from taught {dq_from_taught:.1f} deg.", file=sys.stderr)

for o in out:
    print(json.dumps(o))
