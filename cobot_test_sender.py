#!/usr/bin/env python3
"""
cobot_test_sender.py — send a fake target to cobot_receiver (NO Mac needed)

Run this in a SECOND terminal on the Pi (or later, from the Mac). Sends one target
pose to the receiver and prints the ack. Lets you validate receive->validate->move
->ack entirely on the Pi before the camera is involved.

    python3 cobot_test_sender.py --coords 100 -50 300 0 0 0 --speed 30
    python3 cobot_test_sender.py --host 127.0.0.1 --port 5005
    python3 cobot_test_sender.py --coords 9999 0 0 0 0 0    # test the REJECT path

Default target is a mild, near-center pose. ALWAYS eyeball that the arm is clear
before sending a real (non-dry-run) move.
"""

import argparse
import json
import socket


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--coords", type=float, nargs=6,
                    metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                    default=[100, -50, 300, 0, 0, 0],
                    help="target pose x y z rx ry rz (mm/deg)")
    ap.add_argument("--speed", type=int, default=30)
    args = ap.parse_args()

    msg = json.dumps({"coords": args.coords, "speed": args.speed}) + "\n"
    with socket.create_connection((args.host, args.port), timeout=30) as s:
        s.sendall(msg.encode("utf-8"))
        print(f"sent: {msg.strip()}")
        resp = s.makefile("r").readline().strip()
        print(f"ack : {resp}")


if __name__ == "__main__":
    main()