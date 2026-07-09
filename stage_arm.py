#!/usr/bin/env python3
"""
stage_arm.py — put the arm in the canonical scanning configuration.

The configuration comes from the tracked manual sweep (first log entry):
a known-good, singularity-free posture from which the whole sweep is
reachable in one smooth joint trajectory. Staging in JOINT space pins the
elbow/wrist configuration — the IK never gets to choose a weird one.

Run on the Pi BEFORE starting the receiver. Keep the area clear: the arm
moves on Enter.
"""

import time
from pymycobot import MyCobot320

# first entry of the tracked manual sweep (pose_logger, 2026-07-07)
SCAN_CONFIG = [83.32, -35.68, -56.86, -64.68, -6.59, 63.8]
SPEED = 25

m = MyCobot320('/dev/ttyAMA0', 115200)
m.power_on(); time.sleep(1)
input("arm will move to the scanning configuration — area clear? Enter...")
m.send_angles(SCAN_CONFIG, SPEED)
time.sleep(4)
print("staged. pose:", m.get_coords())
print("angles:", m.get_angles())
print("ready — start the receiver and send the sweep.")