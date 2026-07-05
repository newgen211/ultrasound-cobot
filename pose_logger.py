#!/usr/bin/env python3
"""
pose_logger.py — drag-teach pose recorder for the myCobot 320

Releases the servos so you can move the arm by hand, records its pose
continuously while you sweep, stops on Enter, then re-engages the servos.

Each line of the output .jsonl is one sample:
    {"t_ns": <host ns>, "coords": [x,y,z,rx,ry,rz], "angles": [j1..j6]}

coords feed the reconstruction (position + orientation); angles feed replay.
A sample is only written if BOTH reads came back valid, so the log stays clean
for both consumers.

Runs on the Pi. Logs are written into the pose_logs/ folder. After logging,
the arm powers its servos back on and returns to its joint-zero home pose.

Usage:
    python3 pose_logger.py                 # writes pose_logs/pose_log_<timestamp>.jsonl
    python3 pose_logger.py my_sweep.jsonl  # custom path
    python3 pose_logger.py --no-home       # don't return home at the end
"""

import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path

from pymycobot import MyCobot320

PORT = "/dev/ttyAMA0"
BAUD = 115200
PERIOD_S = 0.05  # target sampling period (actual rate is lower due to serial latency)
LOG_DIR = "pose_logs"            # all logs live here
HOME_ANGLES = [0, 0, 0, 0, 0, 0]  # joint-zero rest pose
HOME_SPEED = 20                   # slow, since homing can be a big swing


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
    ap.add_argument("outfile", nargs="?", default=None,
                    help=f"output path (default: {LOG_DIR}/pose_log_<timestamp>.jsonl)")
    ap.add_argument("--no-home", action="store_true",
                    help="skip the return-to-home move at the end")
    args = ap.parse_args()

    outfile = Path(args.outfile) if args.outfile else \
        Path(LOG_DIR) / f"pose_log_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    outfile.parent.mkdir(parents=True, exist_ok=True)

    mc = MyCobot320(PORT, BAUD)

    input("Hold the arm, then press Enter to release the servos... ")
    mc.release_all_servos()
    print("Released — arm is limp.")

    input("Move the arm to your start position, then press Enter to begin logging... ")
    print(f"Logging to {outfile} — move the arm through the sweep. Press Enter to stop.")

    stop = threading.Event()
    threading.Thread(target=lambda: (input(), stop.set()), daemon=True).start()

    count = skipped = 0
    t_start = time.time()
    with open(outfile, "w") as f:
        while not stop.is_set():
            try:
                c = mc.get_coords()
                a = mc.get_angles()
            except Exception:
                skipped += 1                      # transient serial glitch — don't crash
                time.sleep(PERIOD_S)
                continue
            ns = time.time_ns()

            if c and len(c) == 6 and a and len(a) == 6:
                f.write(json.dumps({"t_ns": ns, "coords": c, "angles": a}) + "\n")
                f.flush()                          # durable: keep data if it crashes mid-sweep
                count += 1
                if count % 20 == 0:
                    print(f"\r   logged {count} samples...", end="", flush=True)
            else:
                skipped += 1
            time.sleep(PERIOD_S)

    dur = time.time() - t_start
    rate = count / dur if dur > 0 else 0.0
    print(f"\nStopped. {count} samples in {dur:.1f}s (~{rate:.1f} Hz), {skipped} skipped.")
    if count == 0:
        print("⚠️  No samples logged — check the arm connection (get_coords returned nothing).")

    input("Hold the arm, then press Enter to power the servos back on... ")
    mc.power_on()
    time.sleep(0.5)
    print(f"Powered on. Log saved to {outfile}")

    if not args.no_home:
        print(f"\n↩  Return to home {HOME_ANGLES} — large, slow move.")
        input("    Clear of the arm? Press Enter to home... ")
        mc.send_angles(HOME_ANGLES, HOME_SPEED)
        wait_until_stopped(mc)
        print("   at home.")


if __name__ == "__main__":
    main()