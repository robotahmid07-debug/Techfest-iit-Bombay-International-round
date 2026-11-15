#!/usr/bin/env python3
"""
main_vision.py  --  AGV autonomy for Raspberry Pi 5 + Arduino Nano (Ackermann, single sonar)

THREADS
    camera     : grabs frames as fast as the camera delivers, keeps ONLY the newest, wakes the autopilot
    autopilot  : lane + traffic-light vision -> PD -> command; never touches the GUI (stable FPS)
    serial-tx  : fixed-rate command stream (stale command => speed 0)
    serial-rx  : telemetry parser
    main       : OpenCV tuning GUI (sliders, live plot).  Only with --gui.
    csv-log    : optional per-frame CSV (--log-dir)

PIPELINE (320x240 processing, lower 50 % ROI)
    HSV -> V-channel top-hat (lighting-robust thin-line isolation) AND white/yellow colour gate
        -> 3x3 open -> [optional bird's-eye warp]
        -> histogram base (warm-started from previous frame) -> sliding windows with
           drift prediction (follows curves) -> 2nd-order fit per lane
        -> centre path (single-lane fallback with perspective-aware lane width)
        -> lookahead error + heading + curvature
    steer = Kp*error + Kd*d(error)/dt + Kh*heading        (+ rate limit)
    speed = base * (1 - curve_slowdown*max(|steer|,|curvature|)) * confidence

WIRE PROTOCOL (ASCII, XOR checksum of the bytes between '$' and '*')
    Pi  -> Nano : $CMD,<seq16>,<speed -100..100>,<steer -100..100>*HH\\n      steer +100 = full right
    Nano -> Pi  : $TEL,<ms>,<dist_mm 9999=none>,<flags>,<bad>,<dropped>*HH\\n
                  flags: bit0 ESTOP, bit1 CMD_TIMEOUT, bit2 SONAR_STALE/not primed

TUNING UI (pick one or both):
    --web  : browser dashboard at http://<pi>:8080  (headless friendly: sliders, live view, plot, buttons)
    --gui  : OpenCV window; keys SPACE run/hold  s save tuning.json  r reset PD  q/ESC quit
             (needs a desktop session, e.g. inside Raspberry Pi Connect / VNC)
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import logging
import math
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Deque, Dict, List, Optional, Tuple, Union
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

try:
    import serial  # pyserial
except ImportError:
    serial = None

log = logging.getLogger("agv")
STOP = threading.Event()


# =============================================================================
# Static configuration (changing these needs a restart)
# =============================================================================
@dataclass
class Config:
    # ---- camera ----
    cam_source: Union[int, str] = "/dev/video0"
    cap_width: int = 320              # ask the camera for this directly (cheapest)
    cap_height: int = 240
    cap_fps: int = 30                 # Logitech delivers 30 fps
    # Applied with v4l2-ctl at startup (ignored if v4l2-ctl is missing). exposure_dynamic_framerate=0
    # stops the camera silently dropping to 15 fps in dim light. Add e.g. 'auto_exposure=1',
    # 'exposure_time_absolute=150' for fixed exposure (more stable thresholds).
    v4l2_controls: Tuple[str, ...] = ("exposure_dynamic_framerate=0",)
    width: int = 320                  # processing size
    height: int = 240
    hfov_deg: float = 60.0            # Logitech ~60 deg; calibrate
    cam_stale_s: float = 0.30

    # ---- serial (Arduino Nano over USB) ----
    port: str = "auto"                # auto = first /dev/ttyUSB* or /dev/ttyACM*
    baud: int = 115200
    boot_wait_s: float = 2.2          # Nano resets when the port opens
    cmd_hz: float = 40.0
    cmd_max_age_s: float = 0.20
    max_tx_backlog_bytes: int = 64

    # ---- lane vision ----
    roi_top_frac: float = 0.50
    yellow_lo: Tuple[int, int, int] = (15, 80, 100)
    yellow_hi: Tuple[int, int, int] = (35, 255, 255)
    use_canny: bool = False           # extra Canny gating (off: top-hat is more lighting robust)
    canny_lo: int = 50
    canny_hi: int = 150
    tophat_k: int = 15                # must exceed the painted line width in pixels
    n_windows: int = 10
    win_margin_px: int = 30
    win_minpix: int = 12
    hist_min: float = 8.0
    lane_width_px: float = 190.0      # at the bottom of the ROI; self-calibrates when both lanes visible
    cam_center_offset_px: float = 0.0
    lane_lost_timeout_s: float = 0.50
    use_bev: bool = False             # bird's-eye warp (needs a sensible trapezoid for YOUR camera)
    bev_src: Tuple[Tuple[float, float], ...] = ((0.25, 0.0), (0.75, 0.0), (1.0, 1.0), (0.0, 1.0))

    # ---- steering ----
    d_alpha: float = 0.45
    steer_limit: float = 1.0

    # ---- traffic light ----
    light_roi_bottom_frac: float = 0.60
    light_s_min: int = 90
    light_v_min: int = 120
    light_min_area: int = 10
    light_min_circ: float = 0.55
    light_diameter_m: float = 0.08    # measure the real lamp!
    light_stop_dist_m: float = 0.45
    slow_ratio: float = 0.60
    green_ratio: float = 0.50
    slow_scale: float = 0.55
    red_frames: int = 3               # at 30 fps = 100 ms debounce
    green_frames: int = 3
    light_every_n: int = 1

    # ---- GUI ----
    gui_every_n: int = 2
    hist_len: int = 240


# =============================================================================
# Live-tunable parameters (GUI sliders write these; autopilot reads every frame)
# =============================================================================
@dataclass
class Params:
    kp: float = 0.90
    kd: float = 0.10
    kh: float = 0.35                  # heading gain (feed-forward for curves)
    steer_rate: float = 4.0           # max steer change per second (full lock = 1.0)
    base_speed: float = 45.0
    min_speed: float = 25.0
    curve_slowdown: float = 0.50
    lookahead: float = 0.55           # 0 = top of ROI, 1 = bottom
    v_min: int = 170
    s_max: int = 70
    tophat_thr: int = 25
    run: bool = True

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)
        log.info("saved tuning to %s", path)

    def load(self, path: str) -> None:
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError) as exc:
            log.warning("could not load %s (%s) - using defaults", path, exc)
            return
        names = {f.name: f for f in fields(self)}
        for k, v in data.items():
            if k in names and k != "run":
                setattr(self, k, type(getattr(self, k))(v))
        log.info("loaded tuning from %s", path)


# =============================================================================
# Camera: newest-frame-only capture with condition-variable wake-up and reconnect
# =============================================================================
class Camera:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._cond = threading.Condition()
        self._frame: Optional[np.ndarray] = None
        self._ts = 0.0
        self._seq = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        src = cfg.cam_source
        self._is_file = isinstance(src, str) and not src.startswith("/dev/video")

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=2.0)

    def wait_for_frame(self, last_seq: int, timeout: float) -> Tuple[Optional[np.ndarray], float, int]:
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq or self._stop.is_set(), timeout)
            return self._frame, self._ts, self._seq

    def _apply_v4l2(self) -> None:
        src = self.cfg.cam_source
        dev = src if isinstance(src, str) else f"/dev/video{src}"
        for ctl in self.cfg.v4l2_controls:
            try:
                r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", ctl], capture_output=True, timeout=2)
                if r.returncode != 0:
                    log.warning("v4l2-ctl %s failed: %s", ctl, r.stderr.decode(errors="ignore").strip())
            except (OSError, subprocess.TimeoutExpired):
                log.info("v4l2-ctl not available; skipped %s (sudo apt install v4l-utils)", ctl)
                break

    def _open(self) -> Optional[cv2.VideoCapture]:
        c = self.cfg
        cap = cv2.VideoCapture(c.cam_source) if self._is_file else cv2.VideoCapture(c.cam_source, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        if not self._is_file:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.cap_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.cap_height)
            cap.set(cv2.CAP_PROP_FPS, c.cap_fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._apply_v4l2()
        log.info("camera: %dx%d @ %.0f fps (requested %dx%d @ %d)",
                 cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                 cap.get(cv2.CAP_PROP_FPS), c.cap_width, c.cap_height, c.cap_fps)
        return cap

    def _run(self) -> None:
        cap: Optional[cv2.VideoCapture] = None
        fails, backoff = 0, 0.25
        while not self._stop.is_set():
            if cap is None:
                cap = self._open()
                if cap is None:
                    log.warning("camera: open failed, retry in %.1fs", backoff)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 3.0)
                    continue
                backoff, fails = 0.25, 0
            try:
                ok, frame = cap.read()
            except cv2.error as exc:
                log.error("camera: cv2 error %s", exc)
                ok, frame = False, None
            if ok and frame is not None:
                fails = 0
                with self._cond:
                    self._frame, self._ts, self._seq = frame, time.monotonic(), self._seq + 1
                    self._cond.notify_all()
                if self._is_file:
                    time.sleep(1.0 / 30.0)
            else:
                if self._is_file:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                fails += 1
                if fails >= 10:
                    log.error("camera: lost, reopening")
                    cap.release()
                    cap = None
                    with self._cond:
                        self._frame = None
                else:
                    time.sleep(0.01)
        if cap is not None:
            cap.release()


# =============================================================================
# Serial link (Arduino Nano over USB)
# =============================================================================
def find_port(preferred: str) -> Optional[str]:
    if preferred and preferred != "auto":
        return preferred
    cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return cands[0] if cands else None


@dataclass
class Telemetry:
    ms: int
    dist_mm: int
    flags: int
    bad_frames: int
    dropped: int
    rx_time: float

    @property
    def estop(self) -> bool:
        return bool(self.flags & 1)

    @property
    def cmd_timeout(self) -> bool:
        return bool(self.flags & 2)

    @property
    def sonar_stale(self) -> bool:
        return bool(self.flags & 4)


class SerialLink:
    def __init__(self, cfg: Config, enabled: bool = True):
        self.cfg = cfg
        self.enabled = enabled and serial is not None
        self._ser: Optional["serial.Serial"] = None
        self._ser_lock = threading.Lock()
        self._next_try = 0.0
        self._cmd = (0, 0, 0.0)
        self._cmd_lock = threading.Lock()
        self._tel: Optional[Telemetry] = None
        self._tel_lock = threading.Lock()
        self._seq = 0
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._tx_loop, name="serial-tx", daemon=True),
                         threading.Thread(target=self._rx_loop, name="serial-rx", daemon=True)]
        self.tx_skipped = 0
        self.rx_bad = 0
        self._last_warn = 0.0

    @property
    def connected(self) -> bool:
        return self._ser is not None

    def start(self) -> None:
        if self.enabled:
            for t in self._threads:
                t.start()

    def set_command(self, speed: int, steer: int) -> None:
        with self._cmd_lock:
            self._cmd = (max(-100, min(100, int(speed))), max(-100, min(100, int(steer))), time.monotonic())

    def telemetry(self) -> Optional[Telemetry]:
        with self._tel_lock:
            t = self._tel
        if t is not None and time.monotonic() - t.rx_time > 0.5:
            return None
        return t

    def stop(self) -> None:
        self.set_command(0, 0)
        if not self.enabled:
            return
        time.sleep(0.15)  # let TX thread flush zero frames
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._drop(self._ser)

    @staticmethod
    def _xor(s: str) -> int:
        c = 0
        for ch in s.encode("ascii"):
            c ^= ch
        return c

    def _warn(self, msg: str, *args) -> None:
        now = time.monotonic()
        if now - self._last_warn > 2.0:
            self._last_warn = now
            log.warning(msg, *args)

    def _ensure_open(self) -> Optional["serial.Serial"]:
        with self._ser_lock:
            if self._ser is not None:
                return self._ser
            if time.monotonic() < self._next_try:
                return None
        port = find_port(self.cfg.port)
        if port is None:
            self._next_try = time.monotonic() + 1.0
            self._warn("serial: no /dev/ttyUSB* or /dev/ttyACM* found")
            return None
        try:
            ser = serial.Serial(port, self.cfg.baud, timeout=0.02, write_timeout=0.02)
        except (serial.SerialException, OSError) as exc:
            self._next_try = time.monotonic() + 1.0
            self._warn("serial: open %s failed (%s)", port, exc)
            return None
        time.sleep(self.cfg.boot_wait_s)        # Nano auto-resets on open; wait for the sketch
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except (serial.SerialException, OSError):
            pass
        with self._ser_lock:
            self._ser = ser
        log.info("serial: connected %s @ %d", port, self.cfg.baud)
        return ser

    def _drop(self, ser) -> None:
        with self._ser_lock:
            if ser is not None and self._ser is ser:
                try:
                    ser.close()
                except Exception:
                    pass
                self._ser = None
                self._next_try = time.monotonic() + 0.5

    def _tx_loop(self) -> None:
        period = 1.0 / self.cfg.cmd_hz
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += period
            ser = self._ensure_open()
            if ser is not None:
                with self._cmd_lock:
                    speed, steer, ts = self._cmd
                if time.monotonic() - ts > self.cfg.cmd_max_age_s:
                    speed = 0                    # stale vision -> stop (keep last steer angle)
                self._seq = (self._seq + 1) & 0xFFFF
                payload = f"CMD,{self._seq},{speed},{steer}"
                frame = f"${payload}*{self._xor(payload):02X}\n".encode("ascii")
                try:
                    if ser.out_waiting > self.cfg.max_tx_backlog_bytes:
                        self.tx_skipped += 1
                    else:
                        ser.write(frame)
                except (serial.SerialException, OSError) as exc:
                    self._warn("serial: write failed (%s)", exc)
                    self._drop(ser)
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            with self._ser_lock:
                ser = self._ser
            if ser is None:
                time.sleep(0.05)
                continue
            try:
                data = ser.read(max(1, ser.in_waiting))
            except (serial.SerialException, OSError) as exc:
                self._warn("serial: read failed (%s)", exc)
                self._drop(ser)
                buf.clear()
                continue
            if not data:
                continue
            buf.extend(data)
            if len(buf) > 512:
                buf.clear()
                continue
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break
                line = bytes(buf[:i])
                del buf[: i + 1]
                self._parse_line(line)

    def _parse_line(self, line: bytes) -> None:
        try:
            s = line.decode("ascii").strip()
            if not s.startswith("$") or "*" not in s:
                raise ValueError("framing")
            body, cs = s[1:].rsplit("*", 1)
            if self._xor(body) != int(cs, 16):
                raise ValueError("checksum")
            f = body.split(",")
            if f[0] != "TEL" or len(f) != 6:
                raise ValueError("fields")
            v = [int(x) for x in f[1:]]
            tel = Telemetry(v[0], v[1], v[2], v[3], v[4], time.monotonic())
            with self._tel_lock:
                self._tel = tel
        except (ValueError, UnicodeDecodeError, IndexError):
            self.rx_bad += 1


# =============================================================================
# Lane detection (curve aware)
# =============================================================================
@dataclass
class LaneResult:
    error: Optional[float]            # +ve = path centre is right of the vehicle (normalised)
    heading: float                    # +ve = path veers right ahead (normalised)
    curve: float                      # normalised curvature index (sign not used)
    conf: float                       # fraction of windows with lane pixels (0..1)
    n_left: int
    n_right: int
    debug: Optional[np.ndarray] = None


class LaneDetector:
    def __init__(self, cfg: Config, params: Params):
        self.cfg, self.p = cfg, params
        self._k_th = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.tophat_k, cfg.tophat_k))
        self._k3 = np.ones((3, 3), np.uint8)
        self._wprof = np.array([0.0, cfg.lane_width_px])   # lane width(y) = slope*y + intercept
        self._base_l: Optional[int] = None
        self._base_r: Optional[int] = None
        self._M: Optional[np.ndarray] = None
        self._M_shape: Tuple[int, int] = (0, 0)

    # -- preprocessing -------------------------------------------------------
    def _binary(self, roi: np.ndarray) -> np.ndarray:
        c, p = self.cfg, self.p
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        v = cv2.GaussianBlur(hsv[:, :, 2], (3, 3), 0)
        th = cv2.morphologyEx(v, cv2.MORPH_TOPHAT, self._k_th)       # thin bright structures only
        _, bright = cv2.threshold(th, p.tophat_thr, 255, cv2.THRESH_BINARY)
        white = cv2.inRange(hsv, (0, 0, p.v_min), (180, p.s_max, 255))
        yellow = cv2.inRange(hsv, np.array(c.yellow_lo), np.array(c.yellow_hi))
        binary = cv2.bitwise_and(bright, cv2.bitwise_or(white, yellow))
        if c.use_canny:
            edges = cv2.Canny(v, c.canny_lo, c.canny_hi)
            binary = cv2.bitwise_and(binary, cv2.dilate(edges, self._k3, iterations=2))
        return cv2.morphologyEx(binary, cv2.MORPH_OPEN, self._k3)

    def _bev(self, shape: Tuple[int, int]) -> np.ndarray:
        if self._M is None or self._M_shape != shape:
            H, W = shape
            src = np.float32([[x * W, y * H] for x, y in self.cfg.bev_src])
            dst = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
            self._M = cv2.getPerspectiveTransform(src, dst)
            self._M_shape = shape
        return self._M

    # -- tracking ------------------------------------------------------------
    def _pick_base(self, hist: np.ndarray, lo: int, hi: int, prev: Optional[int]) -> Optional[int]:
        c = self.cfg
        if prev is not None:                                       # warm start from last frame
            a, b = max(lo, prev - c.win_margin_px), min(hi, prev + c.win_margin_px)
            if b > a and hist[a:b].max() >= c.hist_min:
                return a + int(np.argmax(hist[a:b]))
        seg = hist[lo:hi]
        if seg.size and seg.max() >= c.hist_min:
            return lo + int(np.argmax(seg))
        return None

    def _track(self, b01: np.ndarray, base: int) -> List[Tuple[float, float]]:
        c = self.cfg
        H, W = b01.shape
        win_h = max(1, H // c.n_windows)
        x, dx, last = float(base), 0.0, None
        pts: List[Tuple[float, float]] = []
        for i in range(c.n_windows):
            y_hi, y_lo = H - (i + 1) * win_h, H - i * win_h
            if y_hi < 0:
                break
            xc = int(np.clip(x, 0, W - 1))
            lo, hi = max(0, xc - c.win_margin_px), min(W, xc + c.win_margin_px)
            ys, xs = np.nonzero(b01[y_hi:y_lo, lo:hi])
            if xs.size >= c.win_minpix:
                nx = float(xs.mean()) + lo
                if last is not None:
                    dx = 0.6 * dx + 0.4 * (nx - last)              # drift prediction follows curves
                last = nx
                x = nx + dx
                pts.append((nx, (y_hi + y_lo) / 2.0))
            else:
                x += dx
        return pts

    @staticmethod
    def _fit(pts: List[Tuple[float, float]]) -> Optional[np.ndarray]:
        n = len(pts)
        if n < 3:
            return None
        ys = np.array([q[1] for q in pts])
        xs = np.array([q[0] for q in pts])
        if n >= 5:
            return np.polyfit(ys, xs, 2)
        k = np.polyfit(ys, xs, 1)
        return np.array([0.0, k[0], k[1]])

    # -- main ----------------------------------------------------------------
    def process(self, frame: np.ndarray, draw: bool = False) -> LaneResult:
        c, p = self.cfg, self.p
        h = frame.shape[0]
        roi = frame[int(h * c.roi_top_frac):, :]
        binary = self._binary(roi)
        if c.use_bev:
            binary = cv2.warpPerspective(binary, self._bev(binary.shape), (binary.shape[1], binary.shape[0]),
                                         flags=cv2.INTER_NEAREST)
        H, W = binary.shape
        b01 = binary > 0

        col = b01[2 * H // 3:, :].sum(axis=0).astype(np.float32)
        hist = np.convolve(col, np.ones(7, np.float32), mode="same")
        mid = W // 2
        bl = self._pick_base(hist, 0, mid, self._base_l)
        br = self._pick_base(hist, mid, W, self._base_r)
        self._base_l, self._base_r = bl, br

        pts_l = self._track(b01, bl) if bl is not None else []
        pts_r = self._track(b01, br) if br is not None else []
        cl, cr = self._fit(pts_l), self._fit(pts_r)
        rows = np.linspace(0.10 * H, H - 1, 8)

        xl = np.clip(np.polyval(cl, rows), -W, 2 * W) if cl is not None else None
        xr = np.clip(np.polyval(cr, rows), -W, 2 * W) if cr is not None else None
        wexp = self._wprof[0] * rows + self._wprof[1]

        if xl is not None and xr is not None:
            gap = xr - xl
            ratio = float(np.mean(gap) / max(1.0, np.mean(wexp)))
            if 0.55 <= ratio <= 1.6:
                k = np.polyfit(rows, gap, 1)                      # learn perspective width profile
                self._wprof = 0.95 * self._wprof + 0.05 * k
            elif len(pts_l) >= len(pts_r):
                xr = None
            else:
                xl = None

        if xl is not None and xr is not None:
            cx = 0.5 * (xl + xr)
        elif xl is not None:
            cx = xl + 0.5 * wexp
        elif xr is not None:
            cx = xr - 0.5 * wexp
        else:
            cx = None

        conf = (len(pts_l) + len(pts_r)) / float(2 * c.n_windows)
        error, heading, curve, cc, y_look = None, 0.0, 0.0, None, H * p.lookahead
        if cx is not None:
            cc = np.polyfit(rows, cx, 2)
            vehicle_x = W / 2.0 + c.cam_center_offset_px
            error = float(np.clip((np.polyval(cc, y_look) - vehicle_x) / (W / 2.0), -1.5, 1.5))
            slope = float(np.polyval(np.polyder(cc), y_look))     # dx/dy; forward = decreasing y
            heading = float(np.clip(-slope, -1.0, 1.0))
            curve = float(np.clip(cc[0] * H * H / W, -1.5, 1.5))

        dbg = None
        if draw:
            dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            for x, y in pts_l:
                cv2.circle(dbg, (int(x), int(y)), 3, (255, 0, 0), -1)
            for x, y in pts_r:
                cv2.circle(dbg, (int(x), int(y)), 3, (0, 0, 255), -1)
            if cc is not None:
                ys = np.linspace(0, H - 1, 20)
                path = np.stack([np.polyval(cc, ys), ys], 1).astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(dbg, [path], False, (0, 255, 0), 2)
                cv2.circle(dbg, (int(np.polyval(cc, y_look)), int(y_look)), 5, (0, 255, 255), -1)
            vx = int(W / 2 + c.cam_center_offset_px)
            cv2.line(dbg, (vx, H - 1), (vx, H - 15), (255, 255, 0), 2)
        return LaneResult(error, heading, curve, conf, len(pts_l), len(pts_r), dbg)


# =============================================================================
# PD controller (+ heading feed-forward for curves)
# =============================================================================
class PDController:
    def __init__(self, cfg: Config, params: Params):
        self.cfg, self.p = cfg, params
        self.reset()

    def reset(self) -> None:
        self._t: Optional[float] = None
        self._prev = 0.0
        self._d = 0.0
        self.out = 0.0

    def update(self, err: float, heading: float, now: float) -> float:
        c, p = self.cfg, self.p
        if self._t is None or now - self._t > 0.5:
            dt, d_raw = None, 0.0
        else:
            dt = max(1e-3, now - self._t)
            d_raw = (err - self._prev) / dt
        self._d = c.d_alpha * d_raw + (1.0 - c.d_alpha) * self._d
        u = p.kp * err + p.kd * self._d + p.kh * heading
        u = float(np.clip(u, -c.steer_limit, c.steer_limit))
        if dt is not None and p.steer_rate > 0:
            step = p.steer_rate * dt
            u = self.out + float(np.clip(u - self.out, -step, step))
        self._t, self._prev, self.out = now, err, u
        return u


# =============================================================================
# Traffic light: HSV colour AND apparent size
# =============================================================================
class LightColor(Enum):
    NONE = 0
    RED = 1
    GREEN = 2


@dataclass
class LightDetection:
    color: LightColor
    bbox: Optional[Tuple[int, int, int, int]]
    height_px: int
    dist_m: Optional[float]


class TrafficLightDetector:
    """h_px = f*D/Z.  A lamp counts as 'at the line' only when h_px >= f*D/Z_stop."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.f_px = (cfg.width / 2.0) / math.tan(math.radians(cfg.hfov_deg / 2.0))
        self.h_stop_px = self.f_px * cfg.light_diameter_m / cfg.light_stop_dist_m
        self._k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        log.info("traffic light: f=%.0fpx, stop-gate height=%.1fpx", self.f_px, self.h_stop_px)

    def _best(self, mask: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        c = self.cfg
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < c.light_min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if not 0.6 <= w / float(h) <= 1.6:
                continue
            per = cv2.arcLength(cnt, True)
            if per <= 0 or 4.0 * math.pi * area / (per * per) < c.light_min_circ:
                continue
            if area / float(w * h) < 0.5:
                continue
            if best is None or h > best[3]:
                best = (x, y, w, h)
        return best

    def detect(self, frame: np.ndarray) -> LightDetection:
        c = self.cfg
        roi = frame[: int(frame.shape[0] * c.light_roi_bottom_frac), :]
        hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (3, 3), 0), cv2.COLOR_BGR2HSV)
        s, v = c.light_s_min, c.light_v_min
        red = cv2.inRange(hsv, (0, s, v), (10, 255, 255)) | cv2.inRange(hsv, (170, s, v), (180, 255, 255))
        green = cv2.inRange(hsv, (40, s, v), (90, 255, 255))
        r, g = self._best(red), self._best(green)
        if r is None and g is None:
            return LightDetection(LightColor.NONE, None, 0, None)
        if r is not None and (g is None or r[3] >= g[3]):       # red wins ties (fail-safe)
            color, box = LightColor.RED, r
        else:
            color, box = LightColor.GREEN, g
        return LightDetection(color, box, box[3], self.f_px * c.light_diameter_m / box[3])


class StopState(Enum):
    GO = 0
    SLOW = 1
    STOP = 2


class StopLogic:
    def __init__(self, cfg: Config, det: TrafficLightDetector):
        self.cfg, self.det = cfg, det
        self.state = StopState.GO
        self._red_n = 0
        self._green_n = 0

    def update(self, d: LightDetection) -> Tuple[StopState, float]:
        c, hs = self.cfg, self.det.h_stop_px
        at_line_red = d.color == LightColor.RED and d.height_px >= hs
        near_red = d.color == LightColor.RED and d.height_px >= hs * c.slow_ratio
        near_green = d.color == LightColor.GREEN and d.height_px >= hs * c.green_ratio
        if self.state == StopState.STOP:
            self._green_n = self._green_n + 1 if near_green else 0
            if self._green_n >= c.green_frames:
                self.state, self._red_n, self._green_n = StopState.GO, 0, 0
        else:
            self._red_n = self._red_n + 1 if at_line_red else 0
            if self._red_n >= c.red_frames:
                self.state, self._green_n = StopState.STOP, 0
            else:
                self.state = StopState.SLOW if near_red else StopState.GO
        scale = {StopState.GO: 1.0, StopState.SLOW: c.slow_scale, StopState.STOP: 0.0}[self.state]
        return self.state, scale


# =============================================================================
# CSV data logger (non-blocking for the autopilot)
# =============================================================================
class CsvLogger:
    COLS = ["t", "seq", "fps", "proc_ms", "err", "heading", "curve", "conf", "steer", "speed", "state",
            "light", "light_h", "dist_mm", "flags", "kp", "kd", "kh"]

    def __init__(self, directory: str):
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, time.strftime("run_%Y%m%d_%H%M%S.csv"))
        self._q: "queue.Queue[Dict]" = queue.Queue(maxsize=4000)
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, name="csv-log", daemon=True)
        self._t.start()
        log.info("logging to %s", self.path)

    def log(self, row: Dict) -> None:
        try:
            self._q.put_nowait(row)
        except queue.Full:
            pass  # never stall the control loop

    def _run(self) -> None:
        with open(self.path, "w", newline="") as f:
            w = csv.DictWriter(f, self.COLS, extrasaction="ignore")
            w.writeheader()
            last = time.monotonic()
            while not (self._stop.is_set() and self._q.empty()):
                try:
                    w.writerow(self._q.get(timeout=0.2))
                except queue.Empty:
                    pass
                if time.monotonic() - last > 1.0:
                    f.flush()
                    last = time.monotonic()

    def close(self) -> None:
        self._stop.set()
        self._t.join(timeout=2.0)


# =============================================================================
# Autopilot thread
# =============================================================================
@dataclass
class Snapshot:
    vis: Optional[np.ndarray]
    lane_dbg: Optional[np.ndarray]
    hist: List[Tuple[float, float, float]]
    m: Dict[str, object]


class Autopilot(threading.Thread):
    def __init__(self, cfg: Config, params: Params, cam: Camera, link: SerialLink,
                 logger: Optional[CsvLogger], want_debug: bool):
        super().__init__(name="autopilot", daemon=True)
        self.cfg, self.p, self.cam, self.link, self.logger = cfg, params, cam, link, logger
        self.want_debug = want_debug
        self._snap: Optional[Snapshot] = None
        self._snap_lock = threading.Lock()
        self._reset_pd = False
        self.hist: Deque[Tuple[float, float, float]] = deque(maxlen=cfg.hist_len)

    def request_pd_reset(self) -> None:
        self._reset_pd = True

    def get_snapshot(self) -> Optional[Snapshot]:
        with self._snap_lock:
            return self._snap

    def run(self) -> None:
        cfg, p = self.cfg, self.p
        lane = LaneDetector(cfg, p)
        pd = PDController(cfg, p)
        light = TrafficLightDetector(cfg)
        stop_logic = StopLogic(cfg, light)

        last_seq, n, last_good_t, last_steer = -1, 0, 0.0, 0.0
        ldet = LightDetection(LightColor.NONE, None, 0, None)
        state, scale = StopState.GO, 1.0
        fps, proc_ms, fps_n, fps_proc, fps_t = 0.0, 0.0, 0, 0.0, time.monotonic()
        err_count, last_err_log = 0, 0.0
        speed, steer = 0.0, 0.0

        while not STOP.is_set():
            frame, ts, seq = self.cam.wait_for_frame(last_seq, 0.1)
            now = time.monotonic()
            if frame is None or now - ts > cfg.cam_stale_s:
                self.link.set_command(0, int(round(last_steer * 100)))
                pd.reset()
                self._publish(None, None, {"status": "NO CAMERA", "fps": 0.0, "proc_ms": 0.0})
                continue
            if seq == last_seq:
                continue
            last_seq = seq
            t0 = time.perf_counter()

            try:
                if frame.shape[1] != cfg.width or frame.shape[0] != cfg.height:
                    frame = cv2.resize(frame, (cfg.width, cfg.height), interpolation=cv2.INTER_AREA)
                if self._reset_pd:
                    pd.reset()
                    self._reset_pd = False

                draw = self.want_debug and (n % cfg.gui_every_n == 0)
                res = lane.process(frame, draw=draw)

                if n % cfg.light_every_n == 0:
                    ldet = light.detect(frame)
                    state, scale = stop_logic.update(ldet)

                # ---- steering ----
                if res.error is not None:
                    last_good_t = now
                    steer = pd.update(res.error, res.heading, ts)
                    last_steer = steer
                    lane_ok = True
                elif now - last_good_t < cfg.lane_lost_timeout_s:
                    steer = last_steer * 0.9          # coast on last angle briefly
                    last_steer = steer
                    lane_ok = True
                else:
                    steer, last_steer, lane_ok = 0.0, 0.0, False
                    pd.reset()

                # ---- speed ----
                if lane_ok and scale > 0.0 and p.run:
                    curve_term = min(1.0, max(abs(steer), abs(res.curve) if res.error is not None else 0.0))
                    speed = p.base_speed * scale * (1.0 - p.curve_slowdown * curve_term) * (0.7 + 0.3 * res.conf)
                    speed = min(max(speed, p.min_speed * scale), p.base_speed * scale)
                else:
                    speed = 0.0                       # hold wheel angle while stopped (Ackermann)

                tel = self.link.telemetry()
                if tel is not None and tel.estop:
                    speed = 0.0

                self.link.set_command(int(round(speed)), int(round(steer * 100)))
                self.hist.append((res.error if res.error is not None else 0.0, steer, speed / 100.0))
            except Exception:                         # one bad frame must never kill the run
                err_count += 1
                self.link.set_command(0, int(round(last_steer * 100)))
                if now - last_err_log > 2.0:
                    last_err_log = now
                    log.exception("frame processing error (#%d), vehicle commanded to stop", err_count)
                continue

            dt = time.perf_counter() - t0
            fps_n += 1
            fps_proc += dt
            if now - fps_t >= 1.0:
                fps, proc_ms = fps_n / (now - fps_t), 1000.0 * fps_proc / max(1, fps_n)
                fps_n, fps_proc, fps_t = 0, 0.0, now
                if fps < 0.8 * cfg.cap_fps:
                    log.warning("only %.1f fps (camera should give %d): dim light / USB bandwidth / "
                                "auto-exposure? try v4l2_controls", fps, cfg.cap_fps)
                if not self.want_debug:
                    log.info("fps=%.1f proc=%.1fms state=%s err=%s steer=%.2f speed=%.0f dist=%s",
                             fps, proc_ms, state.name,
                             None if res.error is None else round(res.error, 3), steer, speed,
                             None if tel is None else tel.dist_mm)

            if self.logger is not None:
                self.logger.log({"t": f"{ts:.4f}", "seq": seq, "fps": f"{fps:.1f}", "proc_ms": f"{proc_ms:.2f}",
                                 "err": "" if res.error is None else f"{res.error:.4f}",
                                 "heading": f"{res.heading:.4f}", "curve": f"{res.curve:.4f}",
                                 "conf": f"{res.conf:.2f}", "steer": f"{steer:.4f}", "speed": f"{speed:.1f}",
                                 "state": state.name, "light": ldet.color.name, "light_h": ldet.height_px,
                                 "dist_mm": "" if tel is None else tel.dist_mm,
                                 "flags": "" if tel is None else tel.flags,
                                 "kp": p.kp, "kd": p.kd, "kh": p.kh})

            if draw:
                vis = frame.copy()
                y0 = int(cfg.height * cfg.roi_top_frac)
                cv2.line(vis, (0, y0), (cfg.width, y0), (90, 90, 90), 1)
                if ldet.bbox:
                    x, y, w, h = ldet.bbox
                    col = (0, 0, 255) if ldet.color == LightColor.RED else (0, 255, 0)
                    cv2.rectangle(vis, (x, y), (x + w, y + h), col, 2)
                self._publish(vis, res.debug, {
                    "status": "RUN" if p.run else "HOLD", "fps": fps, "proc_ms": proc_ms, "state": state.name,
                    "err": res.error, "heading": res.heading, "curve": res.curve, "conf": res.conf,
                    "steer": steer, "speed": speed, "tel": tel, "link": self.link.connected,
                    "tx_skip": self.link.tx_skipped, "rx_bad": self.link.rx_bad})
            n += 1

    def _publish(self, vis, dbg, m) -> None:
        if not self.want_debug:
            return
        with self._snap_lock:
            self._snap = Snapshot(vis, dbg, list(self.hist), m)


# =============================================================================
# On-screen tuning GUI (main thread)
# =============================================================================
class TuningGUI:
    WIN = "AGV tuning"
    # (label, attribute, scale, min, max)
    SLIDERS = [
        ("Kp x100", "kp", 100.0, 0, 300),
        ("Kd x100", "kd", 100.0, 0, 100),
        ("Kheading x100", "kh", 100.0, 0, 200),
        ("Steer rate /s", "steer_rate", 1.0, 1, 10),
        ("Base speed", "base_speed", 1.0, 0, 100),
        ("Min speed", "min_speed", 1.0, 0, 100),
        ("Curve slow x100", "curve_slowdown", 100.0, 0, 100),
        ("Lookahead x100", "lookahead", 100.0, 10, 95),
        ("V min", "v_min", 1.0, 0, 255),
        ("S max", "s_max", 1.0, 0, 255),
        ("TopHat thr", "tophat_thr", 1.0, 0, 100),
    ]

    def __init__(self, cfg: Config, params: Params, ap: Autopilot, params_path: str, window: bool = True):
        self.cfg, self.p, self.ap, self.path = cfg, params, ap, params_path
        if not window:            # renderer only (used by the web dashboard)
            return
        cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WIN, 960, 720)
        for label, attr, scale, vmin, vmax in self.SLIDERS:
            init = int(round(getattr(params, attr) * scale))
            cv2.createTrackbar(label, self.WIN, int(np.clip(init, vmin, vmax)), vmax,
                               lambda v, a=attr, s=scale, lo=vmin: self._set(a, s, lo, v))

    def _set(self, attr: str, scale: float, vmin: int, v: int) -> None:
        val = max(v, vmin) / scale
        cur = getattr(self.p, attr)
        setattr(self.p, attr, int(round(val)) if isinstance(cur, int) and not isinstance(cur, bool) else float(val))

    @staticmethod
    def _plot(hist, w: int, h: int) -> np.ndarray:
        img = np.full((h, w, 3), 24, np.uint8)
        cv2.line(img, (0, h // 2), (w, h // 2), (80, 80, 80), 1)
        if len(hist) >= 2:
            arr = np.array(hist, np.float32)
            xs = np.linspace(0, w - 1, len(arr))
            for col, color in ((0, (255, 255, 0)), (1, (255, 0, 255)), (2, (0, 255, 0))):
                ys = h / 2 - np.clip(arr[:, col], -1.2, 1.2) / 1.2 * (h / 2 - 4)
                pts = np.stack([xs, ys], 1).astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(img, [pts], False, color, 1, cv2.LINE_AA)
        cv2.putText(img, "error", (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        cv2.putText(img, "steer", (60, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        cv2.putText(img, "speed/100", (114, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        return img

    def _render(self, snap: Optional[Snapshot]) -> np.ndarray:
        W, H = self.cfg.width, self.cfg.height
        Hroi = H - int(H * self.cfg.roi_top_frac)
        canvas = np.zeros((H + 120, 2 * W, 3), np.uint8)
        if snap is None or snap.vis is None:
            cv2.putText(canvas, "waiting for frames...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            return canvas
        canvas[0:H, 0:W] = snap.vis
        if snap.lane_dbg is not None:
            canvas[0:Hroi, W:2 * W] = cv2.resize(snap.lane_dbg, (W, Hroi))
        m = snap.m
        tel = m.get("tel")
        err = m.get("err")
        lines = [
            f"{m.get('status')}  fps {m.get('fps', 0):.0f}  proc {m.get('proc_ms', 0):.1f}ms  {m.get('state')}",
            f"err {err:+.2f}" if err is not None else "err  ---",
            f"head {m.get('heading', 0):+.2f}  curve {m.get('curve', 0):+.2f}  conf {m.get('conf', 0):.2f}",
            f"steer {m.get('steer', 0):+.2f}  speed {m.get('speed', 0):.0f}",
            f"link {'OK' if m.get('link') else 'DOWN'}  dist {tel.dist_mm if tel else '---'} mm"
            + ("  ESTOP" if tel and tel.estop else ""),
            "SPACE run/hold  s save  r reset  q quit",
        ]
        y = Hroi + 16
        for ln in lines:
            if y > H - 2:
                break
            cv2.putText(canvas, ln, (W + 6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1, cv2.LINE_AA)
            y += 17
        canvas[H:H + 120, :] = self._plot(snap.hist, 2 * W, 120)
        return canvas

    def run(self) -> None:
        while not STOP.is_set() and self.ap.is_alive():
            cv2.imshow(self.WIN, self._render(self.ap.get_snapshot()))
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                STOP.set()
            elif k == ord(" "):
                self.p.run = not self.p.run
            elif k == ord("s"):
                self.p.save(self.path)
            elif k == ord("r"):
                self.ap.request_pd_reset()
        cv2.destroyAllWindows()



# =============================================================================
# Browser dashboard (works on a headless Pi, no desktop / VNC needed)
#   http://<pi-ip>:8080   -> live annotated view + plot (MJPEG) + the same sliders
# =============================================================================
_PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AGV tuning</title><style>
body{background:#14161a;color:#e6e6e6;font:14px system-ui,sans-serif;margin:0;padding:10px}
.wrap{display:flex;flex-wrap:wrap;gap:14px}img{max-width:100%;width:960px;border:1px solid #333;image-rendering:pixelated}
.panel{min-width:300px;flex:1}.row{margin:7px 0}.row label{display:flex;justify-content:space-between}
input[type=range]{width:100%}button{background:#2b5fd9;color:#fff;border:0;border-radius:6px;padding:9px 14px;margin:3px 3px 3px 0;font-size:14px}
button.warn{background:#c0392b}small{color:#9aa}</style></head><body>
<h3 style="margin:0 0 8px">AGV tuning <small id="msg"></small></h3><div class="wrap">
<img src="/stream" alt="stream"><div class="panel">
<button onclick="act('run')">Run / Hold</button><button onclick="act('save')">Save tuning.json</button>
<button class="warn" onclick="act('reset')">Reset PD</button>__SLIDERS__</div></div>
<script>
function say(t){var m=document.getElementById('msg');m.textContent=t;setTimeout(function(){m.textContent=''},1500)}
function setp(n,v,l){document.getElementById('v_'+n).textContent=(v/l).toFixed(l>1?2:0);
 fetch('/set?name='+n+'&value='+v).catch(function(){say('link lost')})}
function act(a){fetch('/action?a='+a).then(function(r){return r.text()}).then(say)}
</script></body></html>"""


class WebUI:
    def __init__(self, cfg: Config, params: Params, ap: Autopilot, params_path: str, host: str, port: int):
        self.cfg, self.p, self.ap, self.path = cfg, params, ap, params_path
        self.r = TuningGUI(cfg, params, ap, params_path, window=False)
        ui = self
        sliders = {s[1]: s for s in TuningGUI.SLIDERS}

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # silence per-request logging
                pass

            def _send(self, body: bytes, ctype: str = "text/plain", code: int = 200) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                try:
                    if u.path in ("/", "/index.html"):
                        self._send(ui.page().encode(), "text/html; charset=utf-8")
                    elif u.path == "/stream":
                        self._stream()
                    elif u.path == "/set":
                        name, val = q["name"][0], int(q["value"][0])
                        if name not in sliders:
                            return self._send(b"unknown", code=400)
                        _, attr, scale, vmin, vmax = sliders[name]
                        ui.r._set(attr, scale, vmin, max(vmin, min(vmax, val)))     # server-side clamp
                        self._send(b"ok")
                    elif u.path == "/action":
                        a = q["a"][0]
                        if a == "run":
                            ui.p.run = not ui.p.run
                            self._send(b"RUN" if ui.p.run else b"HOLD")
                        elif a == "save":
                            ui.p.save(ui.path)
                            self._send(b"saved")
                        elif a == "reset":
                            ui.ap.request_pd_reset()
                            self._send(b"PD reset")
                        else:
                            self._send(b"unknown", code=400)
                    else:
                        self._send(b"not found", code=404)
                except (KeyError, ValueError, IndexError):
                    self._send(b"bad request", code=400)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while not STOP.is_set():
                        img = ui.r._render(ui.ap.get_snapshot())
                        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(buf)).encode() + b"\r\n\r\n" + buf.tobytes() + b"\r\n")
                        time.sleep(0.066)             # ~15 fps view; control loop is unaffected
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.httpd.daemon_threads = True
        self._t = threading.Thread(target=self.httpd.serve_forever, name="web", daemon=True)
        self.host, self.port = host, port

    def page(self) -> str:
        rows = []
        for label, attr, scale, vmin, vmax in TuningGUI.SLIDERS:
            val = int(round(getattr(self.p, attr) * scale))
            val = max(vmin, min(vmax, val))
            shown = f"{val / scale:.2f}" if scale > 1 else str(val)
            rows.append(f'<div class="row"><label><span>{label}</span><b id="v_{attr}">{shown}</b></label>'
                        f'<input type="range" min="{vmin}" max="{vmax}" value="{val}" '
                        f'oninput="setp(\'{attr}\',this.value,{int(scale)})"></div>')
        return _PAGE.replace("__SLIDERS__", "\n".join(rows))

    def start(self) -> None:
        self._t.start()
        log.info("web dashboard: http://%s.local:%d  (or http://<pi-ip>:%d)", socket.gethostname(), self.port, self.port)

    def stop(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()


# =============================================================================
# Entry point
# =============================================================================
def _on_signal(signum, _frame):
    STOP.set()


def main() -> int:
    ap = argparse.ArgumentParser(description="AGV vision/control (Pi 5 + Arduino Nano)")
    ap.add_argument("--port", default="auto", help="serial port or 'auto'")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--video", help="video file for bench testing")
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--gui", action="store_true", help="OpenCV tuning window (needs a desktop session, e.g. Pi Connect/VNC)")
    ap.add_argument("--web", action="store_true", help="browser dashboard: sliders + live view, works fully headless")
    ap.add_argument("--web-port", type=int, default=8080)
    ap.add_argument("--web-host", default="0.0.0.0", help="use 127.0.0.1 + SSH tunnel to keep it private")
    ap.add_argument("--params", default="tuning.json", help="tuning file to load/save")
    ap.add_argument("--start-hold", action="store_true", help="boot in HOLD (wheels stopped) until Run is pressed")
    ap.add_argument("--log-dir", help="write per-frame CSV logs here")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)s %(message)s")
    if serial is None and not args.no_serial:
        log.error("pyserial missing: sudo apt install python3-serial (or use --no-serial)")
        return 2
    if args.gui and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        log.warning("no desktop session for --gui (launched over plain SSH?). Run it from a terminal INSIDE the "
                    "Pi Connect / VNC desktop, or use --web instead.")
        args.gui = False
    if args.gui and os.environ.get("WAYLAND_DISPLAY"):
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")   # OpenCV's Qt build talks X11 (via XWayland)

    src: Union[int, str] = args.video or args.camera
    if isinstance(src, str) and src.isdigit():
        src = int(src)
    cfg = Config(cam_source=src, port=args.port, baud=args.baud)
    params = Params()
    if os.path.exists(args.params):
        params.load(args.params)
    if args.start_hold:
        params.run = False
        log.info("starting in HOLD: press Run in the dashboard (or SPACE in the GUI) to drive")

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    cam = Camera(cfg)
    link = SerialLink(cfg, enabled=not args.no_serial)
    logger = CsvLogger(args.log_dir) if args.log_dir else None
    pilot = Autopilot(cfg, params, cam, link, logger, want_debug=args.gui or args.web)
    web = WebUI(cfg, params, pilot, args.params, args.web_host, args.web_port) if args.web else None

    cam.start()
    link.start()
    pilot.start()
    if web:
        web.start()
    try:
        if args.gui:
            TuningGUI(cfg, params, pilot, args.params).run()
        else:
            while not STOP.is_set() and pilot.is_alive():
                time.sleep(0.2)
    finally:
        log.info("shutting down: commanding stop")
        STOP.set()
        pilot.join(timeout=2.0)
        if web:
            web.stop()
        link.stop()
        cam.stop()
        if logger:
            logger.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
