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
from PySide6 import QtCore  # noqa: E402

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

# The SDK hands frames over through Qt's event loop, so one has to be running or
# they arrive at a fraction of the probe's rate (the GUI gets ~15/s at 50 ms; a
# plain sleep loop got ~5). Built before init, in the order the GUI does it.
app = QtCore.QCoreApplication(sys.argv)

if not cast.init(str(Path.cwd()), 640, 480):
    cast.destroy()
    sys.exit("SDK init failed")
if not cast.connect(ip, port, "research"):
    cast.destroy()
    sys.exit("connect failed %s:%d" % (ip, port))


# The probe reports its freeze state right after connecting and comes up frozen,
# in which case nothing streams. That callback also arrives through the event
# loop, so pump it while waiting rather than just sleeping.
def _pump(seconds_to_pump):
    end = time.time() + seconds_to_pump
    while time.time() < end:
        app.processEvents()
        time.sleep(0.05)


for _ in range(20):
    _pump(0.25)
    if frozen["is"] is not None:
        break
if frozen["is"]:
    cast.userFunction(1, 0)          # CMD_FREEZE toggles run/freeze
    print("probe was frozen, started imaging", flush=True)
    _pump(1.0)

s = Session(root=root, sec=sec)
s.write_connection(ip, port)
s.start(interval)
print("capturing into %s" % s.dir, flush=True)
t0 = time.time()



def tick():
    """Save whatever the SDK has delivered since the last tick."""
    if stop or (seconds > 0 and time.time() - t0 >= seconds):
        app.quit()
        return
    if s.save_frame() and s.frames % 100 == 0:
        print("%d frames" % s.frames, flush=True)


timer = QtCore.QTimer()
timer.timeout.connect(tick)
timer.start(interval)
# The timer re-enters Python every interval, which is also what lets the signal
# handlers above run while Qt owns the loop.
app.exec()

timer.stop()
summary = s.stop()
print(summary, s.dir, flush=True)
print("%.1f raw frames/s over %.1f s" % (summary["frames"] / max(summary["duration_s"], 1e-6),
                                         summary["duration_s"]), flush=True)
cast.disconnect()
cast.destroy()
