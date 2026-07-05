#!/usr/bin/env python3
# reset_arm.py — send the myCobot 320 back to its zero pose. Clear the workspace first.
import time
from pymycobot.mycobot320 import MyCobot320

mc = MyCobot320('/dev/ttyAMA0', 115200)   # if it won't connect, try 1000000

mc.power_on()                              # re-engage servos (in case they were released)
time.sleep(0.5)

print("moving to zero…")
mc.send_angles([0, 0, 0, 0, 0, 0], 30)     # all joints to 0°, speed 30/100

time.sleep(7)                              # let it arrive
print("angles:", mc.get_angles())
print("coords:", mc.get_coords())