"""cast_capture.py: shared Clarius Cast capture core, one save path for GUI and headless.

Owns library loading, the SDK callbacks, the thread-safe frame store, and the
on-disk format (raw_<ts>.bin + raw_<ts>.json + manifest.json + connection.json).
The format is unchanged from pysidecaster, so nothing downstream moves.

Must sit beside libcast.{dylib,so} and pyclariuscast.so from the same Cast
release (12.2.0 here; the Clarius app must match).
"""
import ctypes
import json
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _load_libs():
    if sys.platform.startswith("linux"):
        name = "libcast.so"
    elif sys.platform.startswith("darwin"):
        name = "libcast.dylib"
    elif sys.platform.startswith("win"):
        name = "cast.dll"
    else:
        raise RuntimeError("unsupported platform: %s" % sys.platform)
    libcast, pycast = HERE / name, HERE / "pyclariuscast.so"
    if not libcast.exists():
        raise FileNotFoundError(
            "need %s beside %s (Cast release matching the Clarius app, 12.2.x; "
            "https://github.com/clariusdev/cast/releases)" % (name, __file__))
    if not pycast.exists() and not sys.platform.startswith("win"):
        raise FileNotFoundError(
            "need pyclariuscast.so beside %s, from the same Cast release as %s"
            % (__file__, name))
    # RTLD_GLOBAL so pyclariuscast can resolve libcast's symbols.
    handle = ctypes.CDLL(str(libcast), ctypes.RTLD_GLOBAL)._handle
    if not sys.platform.startswith("win"):
        ctypes.cdll.LoadLibrary(str(pycast))
    return handle


libcast_handle = _load_libs()
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import pyclariuscast  # noqa: E402

IMU_KEYS = "tm gx gy gz ax ay az mx my mz qw qx qy qz".split()


def host_now():
    """Host wall clock as (ISO 8601 UTC string, ns int), the pair every sidecar carries."""
    return datetime.now(timezone.utc).isoformat(), time.time_ns()


def imu_dict(s):
    return {k: getattr(s, k, None) for k in IMU_KEYS}


class Store:
    """Last raw frame, last processed timestamp, and the IMU bundled with it.

    The SDK calls back on its own threads; readers take a snapshot under the lock.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.raw = None
        self.proc_ts = None
        self.imu = []

    def snapshot(self):
        with self.lock:
            return self.raw, self.proc_ts, list(self.imu)


store = Store()


def make_caster(on_image=None, on_freeze=None, on_button=None):
    """Build the pyclariuscast.Caster with the shared callbacks wired in.

    on_image gets the processed frame without the IMU list (the store keeps it);
    on_freeze and on_button are the SDK's freeze and probe-button callbacks.
    """

    def processed(image, w, h, sz, upp, ts, angle, imu):
        with store.lock:
            store.proc_ts = ts
            store.imu = [imu_dict(s) for s in imu] if imu else []
        if on_image:
            on_image(image, w, h, sz, upp, ts, angle)

    def raw(image, lines, samples, bps, axial, lateral, ts, jpg, rf, angle):
        with store.lock:
            store.raw = dict(image=bytes(image[:]), lines=lines, samples=samples, bps=bps,
                             axial=axial, lateral=lateral, timestamp=ts, jpg=jpg, rf=rf,
                             angle=angle)

    def imu_only(imu):
        # The GUI always took IMU from the processed frame it was bundled with,
        # never from this callback; keep that so the sidecars stay identical.
        return

    def spectrum(*_):
        return

    return pyclariuscast.Caster(processed, raw, spectrum, imu_only,
                                on_freeze or (lambda *_: None),
                                on_button or (lambda *_: None))


class Session:
    """One section_<N> directory and everything written into it."""

    def __init__(self, root="clarius_sessions", sec=None, base=None):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        # manifest.section_dir is written relative to base (the repo root on the
        # Mac GUI, the process cwd on the Pi), matching what the GUI always wrote.
        self.base = Path(base) if base is not None else Path.cwd()
        if sec is None:
            nums = [int(d.name[8:]) for d in self.root.iterdir()
                    if d.is_dir() and d.name.startswith("section_") and d.name[8:].isdigit()]
            sec = max(nums, default=0) + 1
        self.dir = self.root / ("section_%d" % sec)
        self.dir.mkdir(exist_ok=True)
        self.frames = 0
        self.t0 = None
        self.last_ts = None

    def _rel(self):
        try:
            return str(self.dir.resolve().relative_to(self.base.resolve()))
        except ValueError:
            return str(self.dir)

    def write_connection(self, ip, port):
        (self.dir / "connection.json").write_text(json.dumps(
            dict(ip=ip, port=port, platform=sys.platform, host_time=host_now()[0]), indent=2))

    def manifest(self, **extra):
        p = self.dir / "manifest.json"
        data = dict(section_dir=self._rel(), host_time=host_now()[0])
        data.update(extra)
        if p.exists():
            try:
                old = json.loads(p.read_text())
                old.update(data)
                data = old
            except json.JSONDecodeError:
                pass
        with open(p, "w") as f:
            json.dump(data, f, indent=2)

    def start(self, interval_ms):
        self.t0 = time.time_ns()
        self.frames = 0
        self.manifest(state="started", interval_ms=interval_ms)

    def save_frame(self, force=False):
        """Write the newest raw frame. Skips a frame already saved unless force
        (the GUI saves on every timer tick, overwriting; headless saves each new one).
        Returns the probe timestamp written, or None."""
        raw, proc_ts, imu = store.snapshot()
        if raw is None or (raw["timestamp"] == self.last_ts and not force):
            return None
        ts = raw["timestamp"]
        self.last_ts = ts
        with open(self.dir / ("raw_%s.bin" % ts), "wb") as f:
            f.write(raw["image"])
        iso, ns = host_now()
        meta = {
            "probe_timestamp_ns": ts,
            "host_timestamp_iso": iso,
            "host_timestamp_ns": ns,
            "frame": {
                "lines": raw["lines"],
                "samples": raw["samples"],
                "bps": raw["bps"],
                "axial_um_per_sample": raw["axial"],
                "lateral_um_per_line": raw["lateral"],
                "angle": raw["angle"],
                "jpg_size": raw["jpg"],
                "is_rf": bool(raw["rf"]),
            },
            "last_processed_probe_ts_ns": proc_ts,
            "imu_samples": imu or [],
            "imu_sample_count": len(imu) if imu else 0,
        }
        with open(self.dir / ("raw_%s.json" % ts), "w") as f:
            json.dump(meta, f, indent=2)
        self.frames += 1
        return ts

    def stop(self):
        dur = (time.time_ns() - (self.t0 or time.time_ns())) / 1e9
        s = dict(frames=self.frames, duration_s=round(dur, 2))
        self.manifest(state="stopped", **s)
        return s
