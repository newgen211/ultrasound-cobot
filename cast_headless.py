#!/usr/bin/env python3
"""cast_headless.py --section N --ip IP --port P [--interval-ms 50] [--seconds 0]

Headless capture into ./clarius_sessions/section_N, same files as the GUI.
Runs until --seconds elapses (0 = forever) or SIGINT/SIGTERM.
Run from the directory that holds the Cast libs and the SDK's TLS key pair:
the SDK is initialised with the process cwd, exactly like the GUI.
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
interval = int(a.get("--interval-ms", 50))
seconds = float(a.get("--seconds", 0))

stop = False


def _stop(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)

cast = make_caster()
if not cast.init(str(Path.cwd()), 640, 480):
    sys.exit("SDK init failed")
if not cast.connect(ip, port, "research"):
    sys.exit("connect failed %s:%d" % (ip, port))
s = Session(sec=sec)
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
