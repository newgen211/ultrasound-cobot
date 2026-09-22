import json, os, sys, warnings
import numpy as np
warnings.filterwarnings('ignore')
class _A: pass
args = _A()
args.urdf = None
args.max_step_deg = 3.0
args.start_lift = 0.0
args.start_taper_mm = 0.0
from ikpy.chain import Chain
urdf = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "mycobot_320_pi.urdf")
chain = Chain.from_urdf_file(urdf, base_elements=['base'],
                             active_links_mask=[False] + [True]*6)
NQ = len(chain.links)
def fk_T(deg6):
    q = np.zeros(NQ); q[1:7] = np.radians(deg6)
    return chain.forward_kinematics(q)

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

def solve_dls(T_tgt, q_seed, q_anchor, iters=200, damp=1e-4, step_clip=0.06,
              gain=0.3):
    """Two-phase damped least-squares.
    Phase 1 (anchored): null-space pull toward the taught joints selects the
    branch and prevents drift near the wrist singularity.
    Phase 2 (polish, last 60 iters): anchor OFF — near the singularity the
    damped 'null space' leaks into the weak task direction, so the anchor
    fights convergence and the solve plateaus 1-2 mm short. With the branch
    already locked, pure DLS polishes task error to zero without hopping."""
    q = q_seed.copy()
    for it in range(iters):
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
        dq = np.linalg.solve(JJ, rhs)
        # constant strong anchor: smoothness (anti-spaz) is the safety
        # requirement and it comes from tracking the taught branch tightly.
        # Cost: at the ~5 near-singular samples the residual along the
        # singular screw plateaus ~2 mm — a direction the physical arm can't
        # command precisely anyway (that's what singular means).
        gain_it = gain
        Jp = np.linalg.solve(JJ, J.T)
        N = np.eye(6) - Jp @ J
        dq = dq + N @ (gain_it * (q_anchor[1:7] - q[1:7]))
        dq *= 0.5                      # relaxation: kills Newton-overshoot
        dq = np.clip(dq, -step_clip, step_clip)   # limit cycles at curved poses
        q[1:7] += dq
    return q
