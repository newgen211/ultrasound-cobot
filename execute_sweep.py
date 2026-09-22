#!/usr/bin/env python3
"""
execute_sweep.py — execute a joint-path sweep on the myCobot 320

The EXECUTE stage of the pipeline:  perceive (guided_sweep_reader) ->
plan (shift_joint_path) -> execute (this). Plays any joint log — a taught
recording or a vision-shifted solved path — under servo control, paced by
the log's timestamps, while logging the arm's real pose for the merge.

⚠️  THIS MOVES THE ARM BY ITSELF. Air test first. Hand near the e-stop.

Capture optimizations vs the previous version:
  - Live pose logging runs in a BACKGROUND THREAD (serial access guarded by a
    lock so reads never collide with sends): the command loop keeps exact
    pacing, and the replay log gets ~2-3x more pose samples for the merge.
  - Consecutive duplicate poses in the source log (pauses during the teach)
    are skipped — less serial traffic, same path.

Usage:
    python3 execute_sweep.py shifted_sweep.jsonl --speed 25
    python3 execute_sweep.py --dry-run
"""

import argparse
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from pymycobot import MyCobot320

PORT = "/dev/ttyAMA0"
BAUD = 115200
LOG_DIR = "pose_logs"


def resolve_log(arg):
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


def dedupe(poses, key, tol=0.05):
    """Drop consecutive near-identical targets (teach-time pauses)."""
    kept = [poses[0]]
    for p in poses[1:]:
        if max(abs(a - b) for a, b in zip(p[key], kept[-1][key])) > tol:
            kept.append(p)
    if kept[-1] is not poses[-1]:
        kept.append(poses[-1])
    return kept


def wait_until_stopped(mc, lock, timeout_s=15):
    t0 = time.time()
    time.sleep(0.4)
    while time.time() - t0 < timeout_s:
        try:
            with lock:
                moving = mc.is_moving()
            if not moving:
                return
        except Exception:
            return
        time.sleep(0.1)


class PoseLogger(threading.Thread):
    """Reads the arm's real pose as fast as the (locked) serial line allows,
    independent of the command loop. Daemon: dies with the main thread."""

    def __init__(self, mc, lock, out_path):
        super().__init__(daemon=True)
        self.mc, self.lock = mc, lock
        self.f = open(out_path, "w")
        self.stop_evt = threading.Event()
        self.count = 0

    def run(self):
        while not self.stop_evt.is_set():
            try:
                with self.lock:
                    c = self.mc.get_coords()
                    a = self.mc.get_angles()
                ns = time.time_ns()
                if c and len(c) == 6 and a and len(a) == 6:
                    self.f.write(json.dumps(
                        {"t_ns": ns, "coords": c, "angles": a}) + "\n")
                    self.count += 1
            except Exception:
                pass
            time.sleep(0.02)          # ~breather; serial latency sets true rate

    def stop(self):
        self.stop_evt.set()
        self.join(timeout=2)
        self.f.flush()
        self.f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pose_log", nargs="?", default=None)
    ap.add_argument("--speed", type=int, default=30)
    ap.add_argument("--start-speed", type=int, default=20)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--mode", choices=["auto", "angles", "coords"], default="auto")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--home", default="0,0,0,0,0,0")
    ap.add_argument("--home-speed", type=int, default=20)
    ap.add_argument("--no-home", action="store_true")
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--delay", type=float, default=None,
                    help="instead of waiting for Enter at the hover, count down "
                    "N seconds then play — for when starting the capture "
                    "requires switching the Mac onto the Clarius WiFi (which "
                    "kills the SSH session). 45 is comfortable.")
    ap.add_argument("--no-prompt", action="store_true",
                    help="never wait for Enter: connect at once and home without asking "
                    "(for flight.sh, which runs this without a terminal)")
    args = ap.parse_args()

    home_angles = [float(x) for x in args.home.split(",")]
    if len(home_angles) != 6:
        sys.exit("❌ --home needs 6 comma-separated joint angles")

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
        sys.exit(f"❌ Some poses are missing '{key}'.")

    n_raw = len(poses)
    poses = dedupe(poses, key)
    t0 = poses[0]["t"]
    duration_s = (poses[-1]["t"] - t0) / 1e9
    print(f"📂 {log_path}: {len(poses)} poses ({n_raw - len(poses)} stationary "
          f"duplicates skipped, stride {args.stride})")
    print(f"   playback source: {key}")
    print(f"   recorded ≈ {duration_s:.1f} s → playback ≈ {duration_s*args.rate:.1f} s "
          f"at rate {args.rate}x")
    print(f"   start {key}: {poses[0][key]}")

    if args.dry_run:
        print("   --dry-run: not connecting, not moving. ✔")
        return

    print("\n⚠️  The arm will MOVE on its own. Clear the workspace.")
    if not args.no_prompt:
        input("    Press Enter to connect and begin... ")

    mc = MyCobot320(PORT, BAUD)
    lock = threading.Lock()
    with lock:
        mc.power_on()
    time.sleep(0.5)

    def send(target):
        with lock:
            if use_angles:
                mc.send_angles(target, args.speed)
            else:
                mc.send_coords(target, args.speed, 0)

    print("→ moving to start pose (the hover, if the log has an approach)...")
    with lock:
        if use_angles:
            mc.send_angles(poses[0]["angles"], args.start_speed)
        else:
            mc.send_coords(poses[0]["coords"], args.start_speed, 0)
    wait_until_stopped(mc, lock)
    print("   at start.")
    if args.delay is not None:
        print(f"⏱  {args.delay:.0f} s to switch the Mac to Clarius WiFi and "
              f"start the capture — sweep starts on its own. SSH may drop; "
              f"the run continues regardless.")
        t_go = time.time() + args.delay
        while time.time() < t_go:
            remaining = t_go - time.time()
            if remaining > 5:
                time.sleep(min(5, remaining - 5))
                print(f"   ... {t_go - time.time():.0f} s")
            else:
                time.sleep(remaining)
    else:
        input("Start your capture on the Mac, then press Enter to play the sweep... ")

    logger = None
    meta = {"source_log": str(log_path), "speed": args.speed, "rate": args.rate,
            "stride": args.stride, "mode": key, "poses_played": len(poses),
            "started": datetime.now().isoformat(timespec="seconds"),
            "events": {}}
    def mark(name):
        meta["events"][name] = time.time_ns()
    mark("at_hover")
    if not args.no_log:
        Path(LOG_DIR).mkdir(exist_ok=True)
        stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
        out_log = Path(LOG_DIR) / f"exec_{stamp}.jsonl"
        meta_path = Path(LOG_DIR) / f"exec_{stamp}_meta.json"
        logger = PoseLogger(mc, lock, out_log)
        logger.start()
        print(f"   logging live poses (threaded) → {out_log}  (merge THIS file)")

    print("▶ playing back...")
    mark("playback_start")
    wall0 = time.time()
    for p in poses:
        target_dt = ((p["t"] - t0) / 1e9) * args.rate
        sleep_s = target_dt - (time.time() - wall0)
        if sleep_s > 0:
            time.sleep(sleep_s)
        send(p[key])

    wait_until_stopped(mc, lock)
    mark("playback_end")
    if logger is not None:
        logger.stop()
        meta["poses_logged"] = logger.count
        with open(meta_path, "w") as mf:
            json.dump(meta, mf, indent=2)
        print(f"   logged {logger.count} live poses; run manifest → {meta_path}")
    print("   sweep playback complete.")

    if not args.no_home:
        print(f"\n↩  Return to home {home_angles} — large move, slow.")
        go = True
        if not args.no_prompt:
            try:
                input("    Clear of the arm? Press Enter to home... ")
            except EOFError:
                print("    (stdin gone — SSH dropped. Skipping home; arm holds at "
                      "retract. Reattach tmux and home manually when back online.)")
                go = False
        if go:
            with lock:
                mc.send_angles(home_angles, args.home_speed)
            wait_until_stopped(mc, lock)
            print("   at home.")

    try:
        with lock:
            a = mc.get_angles()
        print("✅ done. Final angles:", a)
    except Exception:
        print("✅ done.")
    print("   (arm left powered and holding — release manually for limp mode.)")


if __name__ == "__main__":
    main()