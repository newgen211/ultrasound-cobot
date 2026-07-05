#!/usr/bin/env python3
"""
replay_sweep.py — replay a recorded sweep on the myCobot 320

Reads a pose_log.jsonl (the file pose_logger.py writes), moves the arm to the
START of the recording, then plays the whole path back under servo control,
paced by the recorded timestamps.

Runs on the Pi (same place as pose_logger.py).

⚠️  THIS MOVES THE ARM BY ITSELF. It is NOT the limp drag-teach mode.
    - Run it in the AIR first, with nothing under the probe.
    - Keep the workspace clear and a hand near the power / e-stop.
    - Use --dry-run to see the plan without moving anything.

Playback uses the logged JOINT ANGLES, not coords: joint playback reproduces
the exact taught configuration with no inverse-kinematics guessing and no
gimbal-lock surprises. Falls back to coords only if a log has no angles.

Usage:
    python3 replay_sweep.py                      # plays the latest log in pose_logs/
    python3 replay_sweep.py my_sweep.jsonl       # by name (looked up in . and pose_logs/)
    python3 replay_sweep.py --dry-run            # print the plan, don't move
    python3 replay_sweep.py --speed 25 --rate 2  # slower joints, half-speed timeline
    python3 replay_sweep.py --stride 2           # play every 2nd logged pose
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

from pymycobot import MyCobot320

PORT = "/dev/ttyAMA0"
BAUD = 115200
LOG_DIR = "pose_logs"   # where pose_logger.py writes its logs


def resolve_log(arg):
    """Find the log to replay: the given path, the same name inside pose_logs/,
    or — with no argument — the most recent log in pose_logs/ (then cwd)."""
    folder = Path(LOG_DIR)
    if arg:
        for cand in (Path(arg), folder / arg):
            if cand.exists():
                return cand
        sys.exit(f"❌ Log not found: {arg} (looked in . and {LOG_DIR}/)")
    candidates = sorted(folder.glob("*.jsonl"), key=lambda p: p.stat().st_mtime) if folder.exists() else []
    if not candidates:
        candidates = sorted(Path(".").glob("pose_log*.jsonl"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        sys.exit(f"❌ No .jsonl logs found in {LOG_DIR}/ or current folder.")
    return candidates[-1]


def load_poses(path):
    """Return (list of {t, angles?, coords?}, has_angles_for_all)."""
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            e = {"t": d["t_ns"]}
            if d.get("angles") and len(d["angles"]) == 6:
                e["angles"] = d["angles"]
            if d.get("coords") and len(d["coords"]) == 6:
                e["coords"] = d["coords"]
            if "angles" in e or "coords" in e:
                poses.append(e)
    has_angles = bool(poses) and all("angles" in p for p in poses)
    return poses, has_angles


def wait_until_stopped(mc, timeout_s=15):
    """Bounded wait for the arm to stop moving (is_moving can be flaky)."""
    t0 = time.time()
    time.sleep(0.4)
    while time.time() - t0 < timeout_s:
        try:
            if not mc.is_moving():
                return
        except Exception:
            return
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pose_log", nargs="?", default=None,
                    help="log to replay (default: most recent in pose_logs/)")
    ap.add_argument("--speed", type=int, default=30,
                    help="joint/coord speed 1-100 for playback (default 30, keep it slow)")
    ap.add_argument("--start-speed", type=int, default=20,
                    help="speed for the initial move to the start pose (default 20)")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="timeline scale: 1.0 = real time, 2.0 = half speed (default 1.0)")
    ap.add_argument("--stride", type=int, default=1,
                    help="play every Nth logged pose (default 1)")
    ap.add_argument("--mode", choices=["auto", "angles", "coords"], default="auto",
                    help="playback source (default auto: angles if available)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and exit without moving the arm")
    ap.add_argument("--home", default="0,0,0,0,0,0",
                    help="JOINT angles to return to at the end (default 0,0,0,0,0,0 = home)")
    ap.add_argument("--home-speed", type=int, default=20,
                    help="speed for the return-home move (default 20, kept slow)")
    ap.add_argument("--no-home", action="store_true",
                    help="skip the return-to-home move at the end")
    ap.add_argument("--no-log", action="store_true",
                    help="don't log live poses during playback (on by default; "
                         "the log is what you merge with the new capture)")
    args = ap.parse_args()

    home_angles = [float(x) for x in args.home.split(",")]
    if len(home_angles) != 6:
        sys.exit("❌ --home needs 6 comma-separated joint angles, e.g. 0,0,0,0,0,0")

    log_path = resolve_log(args.pose_log)
    poses, has_angles = load_poses(log_path)
    if len(poses) < 2:
        sys.exit(f"❌ Need at least 2 usable poses in {log_path}")
    poses = poses[::args.stride]

    use_angles = (args.mode == "angles") or (args.mode == "auto" and has_angles)
    if use_angles and not has_angles:
        sys.exit("❌ --mode angles requested but the log has no joint angles.")
    key = "angles" if use_angles else "coords"
    if any(key not in p for p in poses):
        sys.exit(f"❌ Some poses are missing '{key}'. Try --mode "
                 f"{'coords' if use_angles else 'angles'}.")

    t0 = poses[0]["t"]
    duration_s = (poses[-1]["t"] - t0) / 1e9
    print(f"📂 {log_path}: {len(poses)} poses (stride {args.stride})")
    print(f"   playback source: {key}")
    print(f"   recorded length ≈ {duration_s:.1f} s  →  playback ≈ {duration_s*args.rate:.1f} s "
          f"at rate {args.rate}x")
    print(f"   start {key}: {poses[0][key]}")

    if args.dry_run:
        print("   --dry-run: not connecting, not moving. ✔")
        return

    print("\n⚠️  The arm will MOVE on its own. Clear the workspace, run it in the")
    print("    air with nothing under the probe, keep a hand near the power.")
    input("    Press Enter to connect and begin... ")

    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
    time.sleep(0.5)

    def send(target):
        if use_angles:
            mc.send_angles(target, args.speed)
        else:
            mc.send_coords(target, args.speed, 0)  # mode 0 = non-linear (robust near singularities)

    # 1) go to the start pose slowly and wait for arrival
    print("→ moving to start pose...")
    if use_angles:
        mc.send_angles(poses[0]["angles"], args.start_speed)
    else:
        mc.send_coords(poses[0]["coords"], args.start_speed, 0)
    wait_until_stopped(mc)
    print("   at start.")
    input("Start your capture on the Mac, then press Enter to play back the sweep... ")

    # open a live pose log: the arm's ACTUAL pose during THIS run, timestamped
    # now so it lines up with the frames you're capturing now. (The old teach
    # log's timestamps are from a different session and won't merge.)
    log_f = None
    if not args.no_log:
        Path(LOG_DIR).mkdir(exist_ok=True)
        out_log = Path(LOG_DIR) / f"replay_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
        log_f = open(out_log, "w")
        print(f"   logging live poses → {out_log}  (merge THIS file, not the teach log)")

    # 2) stream the path, pacing each command by the recorded timestamps,
    #    sampling the arm's real pose as we go
    print("▶ playing back...")
    wall0 = time.time()
    logged = 0
    for p in poses:
        target_dt = ((p["t"] - t0) / 1e9) * args.rate  # when this pose should fire
        sleep_s = target_dt - (time.time() - wall0)
        if sleep_s > 0:
            time.sleep(sleep_s)
        send(p[key])
        if log_f is not None:
            try:
                c = mc.get_coords()
                a = mc.get_angles()
            except Exception:
                c = a = None
            ns = time.time_ns()
            if c and len(c) == 6 and a and len(a) == 6:
                log_f.write(json.dumps({"t_ns": ns, "coords": c, "angles": a}) + "\n")
                log_f.flush()
                logged += 1

    if log_f is not None:
        log_f.close()
        print(f"   logged {logged} live poses → {out_log}")

    wait_until_stopped(mc)
    print("   sweep playback complete.")

    # 3) return to home (joint zero by default) — a big, slow move
    if not args.no_home:
        print(f"\n↩  Return to home {home_angles} — this is a large move, done slowly.")
        input("    Clear of the arm? Press Enter to home... ")
        mc.send_angles(home_angles, args.home_speed)
        wait_until_stopped(mc)
        print("   at home.")

    try:
        print("✅ done. Final angles:", mc.get_angles())
    except Exception:
        print("✅ done.")
    print("   (arm left powered and holding position — release manually if you want it limp.)")


if __name__ == "__main__":
    main()