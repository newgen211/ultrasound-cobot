#!/usr/bin/env python3
"""cast_headless.py --section N --ip IP --port P [--root DIR] [--interval-ms 50] [--seconds 0]

Headless capture into <root>/section_N (default root ./clarius_sessions), the
same files the GUI writes. Runs until --seconds elapses (0 = forever) or
SIGINT/SIGTERM, and unfreezes the probe first if it comes up frozen.

Run from the directory that holds the SDK's TLS key pair: the SDK is
initialised with the process cwd, exactly like the GUI, so on the Mac that
means the repo root:

    python src/capture/cast_headless.py --section 7 --ip 192.168.1.1 \
        --port 5828 --root data/clarius_sessions
"""
import os
import signal
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from cast_capture import Session, make_caster  # noqa: E402

a = dict(zip(sys.argv[1::2], sys.argv[2::2]))
try:
    sec, ip, port = int(a["--section"]), a["--ip"], int(a["--port"])
except KeyError:
    sys.exit(__doc__)
root = a.get("--root", "clarius_sessions")
interval = int(a.get("--interval-ms", 50))
seconds = float(a.get("--seconds", 0))

stop = False


def _stop(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)

frozen = {"is": None}


def _on_freeze(f):
    frozen["is"] = bool(f)


cast = make_caster(on_freeze=_on_freeze)
if not cast.init(str(Path.cwd()), 640, 480):
    cast.destroy()
    sys.exit("SDK init failed")
if not cast.connect(ip, port, "research"):
    cast.destroy()
    sys.exit("connect failed %s:%d" % (ip, port))
# The probe reports its freeze state right after connecting and comes up frozen,
# in which case nothing streams. Toggle it into run before opening the session.
for _ in range(20):
    if frozen["is"] is not None:
        break
    time.sleep(0.25)
if frozen["is"]:
    cast.userFunction(1, 0)          # CMD_FREEZE toggles run/freeze
    print("probe was frozen, started imaging", flush=True)
    time.sleep(1.0)

s = Session(root=root, sec=sec)
s.write_connection(ip, port)
s.start(interval)
print("capturing into %s" % s.dir, flush=True)
t0 = time.time()
while not stop and (seconds <= 0 or time.time() - t0 < seconds):
    time.sleep(interval / 1000)
    if s.save_frame() and s.frames % 100 == 0:
        print("%d frames" % s.frames, flush=True)
print(s.stop(), s.dir, flush=True)
cast.disconnect()
cast.destroy()
