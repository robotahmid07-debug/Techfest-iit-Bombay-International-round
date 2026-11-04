#!/usr/bin/env python3
"""
main_vision.py  --  High-level autonomy for the AGV (Raspberry Pi 5)

Pipeline
    USB camera -> [Camera thread: latest-frame only] -> lane detector
                                                     -> traffic-light detector
               -> PD steering + speed scheduling + stop-line state machine
               -> [Serial TX thread @ 30 Hz fixed rate] -> ESP32-S3

Design rules (see architecture notes):
  * The Pi is ADVISORY.  The ESP32 owns safety (15 cm hard stop, watchdog).
  * Vision never blocks the serial link: a dedicated TX thread sends the newest
    command at a fixed rate and forces speed=0 if the command is stale.
  * Frames are never queued: the camera thread overwrites a single slot, so
    latency cannot grow when processing is slower than capture.
  * Serial writes are skipped (not queued) if the OS buffer has a backlog.

Wire protocol (ASCII, XOR checksum over the bytes between '$' and '*'):
    Pi  -> ESP32 : $CMD,<seq16>,<speed -100..100>,<steer -100..100>*HH\\n
    ESP32 -> Pi  : $TEL,<ms>,<L>,<FL>,<FC>,<FR>,<R>,<flags>,<bad>,<dropped>*HH\\n
                   distances in mm (9999 = no echo), flags bit0=ESTOP,
                   bit1=CMD_TIMEOUT, bit2=FRONT_SENSOR_STALE
    steer: +100 = full right.

Run:
    python3 main_vision.py --port /dev/ttyAMA0
    python3 main_vision.py --no-serial --show --video test.mp4      # bench test
"""
from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np

try:
    import serial  # pyserial
except ImportError:  # allows --no-serial bench runs without pyserial
    serial = None

log = logging.getLogger("agv")


# =============================================================================
# Configuration  (every tunable lives here -- calibrate on the real track)
# =============================================================================
@dataclass
class Config:
    # ---- camera ----
    cam_source: Union[int, str] = "/dev/video0"
    width: int = 640
    height: int = 480
    fps: int = 30
    hfov_deg: float = 60.0            # Logitech C270 ~60 deg; calibrate!
    cam_stale_s: float = 0.40         # no new frame for this long -> stop

    # ---- serial ----
    port: str = "/dev/ttyAMA0"        # Pi 5 40-pin header UART (GPIO14/15)
    baud: int = 115200
    cmd_hz: float = 30.0
    cmd_max_age_s: float = 0.25       # older than this -> transmit speed 0
    max_tx_backlog_bytes: int = 64    # skip write if more than this is pending

    # ---- lane detection ----
    roi_top_frac: float = 0.50        # keep lower 50 % of the frame
    white_lo: Tuple[int, int, int] = (0, 0, 175)
    white_hi: Tuple[int, int, int] = (180, 70, 255)
    yellow_lo: Tuple[int, int, int] = (15, 80, 100)
    yellow_hi: Tuple[int, int, int] = (35, 255, 255)
    canny_lo: int = 50
    canny_hi: int = 150
    n_windows: int = 8
    win_margin_px: int = 60
    win_minpix: int = 40
    min_fit_pts: int = 3
    hist_min: int = 15                # min histogram peak to accept a lane base
    lane_width_px_default: float = 380.0
    lookahead_row_frac: float = 0.55  # 0 = top of ROI, 1 = bottom of ROI
    cam_center_offset_px: float = 0.0 # camera mounting offset from vehicle axis
    lane_lost_timeout_s: float = 0.60

    # ---- PD steering ----
    kp: float = 0.90
    kd: float = 0.12
    d_alpha: float = 0.45             # derivative low-pass (1 = no filtering)
    steer_limit: float = 1.0
    steer_rate_limit: float = 5.0     # max change of steer (full-scale units) / s

    # ---- speed scheduling (units: ESP32 percent) ----
    base_speed: float = 45.0
    min_speed: float = 22.0
    curve_slowdown: float = 0.50
    slow_scale: float = 0.55          # speed factor in the SLOW (approach) zone

    # ---- traffic light ----
    light_roi_bottom_frac: float = 0.60   # search only the upper 60 % of frame
    light_s_min: int = 90
    light_v_min: int = 120
    light_min_area: int = 30
    light_min_circ: float = 0.55
    light_diameter_m: float = 0.08    # REAL lamp diameter -- measure it!
    light_stop_dist_m: float = 0.45   # camera-to-lamp range at which to stop
    slow_ratio: float = 0.60          # slow when h >= 0.6 * h_stop
    green_ratio: float = 0.50         # green must be >= 0.5 * h_stop to release
    red_frames: int = 3               # debounce
    green_frames: int = 3


# =============================================================================
# Camera: single-slot, latest-frame capture with automatic reconnect
# =============================================================================
class Camera:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._lock = threading.Lock()
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
        self._thread.join(timeout=2.0)

    def latest(self) -> Tuple[Optional[np.ndarray], float, int]:
        with self._lock:
            return self._frame, self._ts, self._seq

    def _open(self) -> Optional[cv2.VideoCapture]:
        src = self.cfg.cam_source
        cap = cv2.VideoCapture(src) if self._is_file else cv2.VideoCapture(src, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        if not self._is_file:
            # MJPG keeps USB bandwidth/latency low at 30 fps.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
            cap.set(cv2.CAP_PROP_FPS, self.cfg.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _run(self) -> None:
        cap: Optional[cv2.VideoCapture] = None
        fails = 0
        backoff = 0.25
        while not self._stop.is_set():
            if cap is None:
                cap = self._open()
                if cap is None:
                    log.warning("camera: open failed, retry in %.1fs", backoff)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 3.0)
                    continue
                log.info("camera: opened %s", self.cfg.cam_source)
                backoff, fails = 0.25, 0
            try:
                ok, frame = cap.read()
            except cv2.error as exc:
                log.error("camera: cv2 error %s", exc)
                ok, frame = False, None
            if ok and frame is not None:
                fails = 0
                with self._lock:
                    self._frame, self._ts, self._seq = frame, time.monotonic(), self._seq + 1
                if self._is_file:
                    time.sleep(1.0 / self.cfg.fps)
            else:
                if self._is_file:  # loop test videos
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                fails += 1
                if fails >= 10:  # ~ camera unplugged / USB reset
                    log.error("camera: lost, reopening")
                    cap.release()
                    cap = None
                    with self._lock:
                        self._frame = None  # consumers see staleness -> stop
                else:
                    time.sleep(0.01)
        if cap is not None:
            cap.release()


# =============================================================================
# Serial link: fixed-rate TX, tolerant RX, automatic reconnect
# =============================================================================
@dataclass
class Telemetry:
    ms: int
    dist_mm: Tuple[int, int, int, int, int]   # L, FL, FC, FR, R
    flags: int
    bad_frames: int
    dropped: int
    rx_time: float

    @property
    def estop(self) -> bool:
        return bool(self.flags & 1)


class SerialLink:
    def __init__(self, cfg: Config, enabled: bool = True):
        self.cfg = cfg
        self.enabled = enabled and serial is not None
        self._ser: Optional["serial.Serial"] = None
        self._ser_lock = threading.Lock()
        self._next_try = 0.0
        self._cmd = (0, 0, 0.0)  # speed, steer, monotonic timestamp
        self._cmd_lock = threading.Lock()
        self._tel: Optional[Telemetry] = None
        self._tel_lock = threading.Lock()
        self._seq = 0
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._tx_loop, name="serial-tx", daemon=True),
            threading.Thread(target=self._rx_loop, name="serial-rx", daemon=True),
        ]
        self.tx_skipped = 0
        self.rx_bad = 0
        self._last_warn = 0.0

    # ---- public API ----
    def start(self) -> None:
        if self.enabled:
            for t in self._threads:
                t.start()

    def set_command(self, speed: int, steer: int) -> None:
        speed = max(-100, min(100, int(speed)))
        steer = max(-100, min(100, int(steer)))
        with self._cmd_lock:
            self._cmd = (speed, steer, time.monotonic())

    def telemetry(self) -> Optional[Telemetry]:
        with self._tel_lock:
            t = self._tel
        if t is not None and time.monotonic() - t.rx_time > 0.5:
            return None  # stale telemetry is no telemetry
        return t

    def stop(self) -> None:
        self.set_command(0, 0)
        if not self.enabled:
            return
        time.sleep(0.12)  # let TX thread flush a few zero frames
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._drop(self._ser)

    # ---- helpers ----
    @staticmethod
    def _xor(s: str) -> int:
        c = 0
        for ch in s.encode("ascii"):
            c ^= ch
        return c

    def _warn(self, msg: str, *args) -> None:
        now = time.monotonic()
        if now - self._last_warn > 2.0:  # rate-limit log spam
            self._last_warn = now
            log.warning(msg, *args)

    def _ensure_open(self) -> Optional["serial.Serial"]:
        with self._ser_lock:
            if self._ser is not None:
                return self._ser
            now = time.monotonic()
            if now < self._next_try:
                return None
            try:
                ser = serial.Serial(self.cfg.port, self.cfg.baud, timeout=0.02,
                                    write_timeout=0.02)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                self._ser = ser
                log.info("serial: opened %s @ %d", self.cfg.port, self.cfg.baud)
            except (serial.SerialException, OSError) as exc:
                self._next_try = now + 1.0
                self._warn("serial: open failed (%s)", exc)
            return self._ser

    def _drop(self, ser) -> None:
        with self._ser_lock:
            if ser is not None and self._ser is ser:
                try:
                    ser.close()
                except Exception:
                    pass
                self._ser = None
                self._next_try = time.monotonic() + 0.5

    # ---- threads ----
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
                    speed = steer = 0  # vision stalled -> fail safe
                self._seq = (self._seq + 1) & 0xFFFF
                payload = f"CMD,{self._seq},{speed},{steer}"
                frame = f"${payload}*{self._xor(payload):02X}\n".encode("ascii")
                try:
                    if ser.out_waiting > self.cfg.max_tx_backlog_bytes:
                        self.tx_skipped += 1  # never build a queue
                    else:
                        ser.write(frame)
                except (serial.SerialException, OSError) as exc:
                    self._warn("serial: write failed (%s)", exc)
                    self._drop(ser)
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()  # overrun: resync, don't burst

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
            if len(buf) > 512:       # garbage / no newline: resync
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
            if f[0] != "TEL" or len(f) != 10:
                raise ValueError("fields")
            v = [int(x) for x in f[1:]]
            tel = Telemetry(ms=v[0], dist_mm=(v[1], v[2], v[3], v[4], v[5]),
                            flags=v[6], bad_frames=v[7], dropped=v[8],
                            rx_time=time.monotonic())
            with self._tel_lock:
                self._tel = tel
        except (ValueError, UnicodeDecodeError, IndexError):
            self.rx_bad += 1


# =============================================================================
# Lane detection: HSV threshold + Canny gating + sliding windows
# =============================================================================
@dataclass
class LaneResult:
    error: Optional[float]          # normalised lateral error, +ve = lane centre is right of vehicle
    n_left: int
    n_right: int
    debug: Optional[np.ndarray] = None


class LaneDetector:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.lane_w = cfg.lane_width_px_default
        self._k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    def _binary(self, roi: np.ndarray) -> np.ndarray:
        c = self.cfg
        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        color = cv2.inRange(hsv, np.array(c.white_lo), np.array(c.white_hi)) | \
                cv2.inRange(hsv, np.array(c.yellow_lo), np.array(c.yellow_hi))
        edges = cv2.Canny(gray, c.canny_lo, c.canny_hi)
        # Keep colour pixels that sit next to a strong edge: rejects large flat
        # bright regions (glare, white walls) but keeps painted line borders.
        near_edge = cv2.dilate(edges, self._k5, iterations=3)
        binary = cv2.bitwise_and(color, near_edge)
        return cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._k5)

    def _sliding(self, b01: np.ndarray, base: int) -> List[Tuple[int, int]]:
        c = self.cfg
        H, W = b01.shape
        win_h = H // c.n_windows
        x_cur = base
        pts: List[Tuple[int, int]] = []
        for i in range(c.n_windows):
            y_hi, y_lo = H - (i + 1) * win_h, H - i * win_h
            x_lo, x_hi = max(0, x_cur - c.win_margin_px), min(W, x_cur + c.win_margin_px)
            ys, xs = np.nonzero(b01[y_hi:y_lo, x_lo:x_hi])
            if len(xs) >= c.win_minpix:
                x_cur = int(xs.mean()) + x_lo
                pts.append((x_cur, (y_hi + y_lo) // 2))
        return pts

    def _fit_x(self, pts: List[Tuple[int, int]], y_look: float) -> Optional[float]:
        if len(pts) < self.cfg.min_fit_pts:
            return None
        xs = np.array([p[0] for p in pts], dtype=float)
        ys = np.array([p[1] for p in pts], dtype=float)
        coef = np.polyfit(ys, xs, 1)
        return float(np.polyval(coef, y_look))

    def process(self, frame: np.ndarray, draw: bool = False) -> LaneResult:
        c = self.cfg
        h, w = frame.shape[:2]
        roi = frame[int(h * c.roi_top_frac):, :]
        binary = self._binary(roi)
        b01 = (binary > 0).astype(np.uint8)
        H, W = b01.shape
        hist = b01[H // 2:, :].sum(axis=0)
        mid = W // 2
        left_pts: List[Tuple[int, int]] = []
        right_pts: List[Tuple[int, int]] = []
        if hist[:mid].max() >= c.hist_min:
            left_pts = self._sliding(b01, int(np.argmax(hist[:mid])))
        if hist[mid:].max() >= c.hist_min:
            right_pts = self._sliding(b01, mid + int(np.argmax(hist[mid:])))

        y_look = H * c.lookahead_row_frac
        xl, xr = self._fit_x(left_pts, y_look), self._fit_x(right_pts, y_look)

        # Plausibility: if both found but implausible spacing, keep the stronger.
        if xl is not None and xr is not None:
            gap = xr - xl
            if gap < 0.4 * c.lane_width_px_default or gap > 1.8 * c.lane_width_px_default:
                if len(left_pts) >= len(right_pts):
                    xr = None
                else:
                    xl = None
            else:
                self.lane_w = 0.9 * self.lane_w + 0.1 * gap  # EMA of lane width

        if xl is not None and xr is not None:
            centre = 0.5 * (xl + xr)
        elif xl is not None:
            centre = xl + 0.5 * self.lane_w
        elif xr is not None:
            centre = xr - 0.5 * self.lane_w
        else:
            centre = None

        error = None
        if centre is not None:
            vehicle_x = W / 2.0 + c.cam_center_offset_px
            error = float(np.clip((centre - vehicle_x) / (W / 2.0), -1.5, 1.5))

        dbg = None
        if draw:
            dbg = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            for x, y in left_pts:
                cv2.circle(dbg, (x, y), 5, (255, 0, 0), -1)
            for x, y in right_pts:
                cv2.circle(dbg, (x, y), 5, (0, 0, 255), -1)
            if centre is not None:
                cv2.line(dbg, (int(centre), 0), (int(centre), H), (0, 255, 0), 2)
            cv2.line(dbg, (int(W / 2 + c.cam_center_offset_px), 0),
                     (int(W / 2 + c.cam_center_offset_px), H), (255, 255, 0), 1)
        return LaneResult(error, len(left_pts), len(right_pts), dbg)


# =============================================================================
# PD controller
# =============================================================================
class PDController:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self._t: Optional[float] = None
        self._prev_err = 0.0
        self._d = 0.0
        self.out = 0.0

    def update(self, err: float, now: float) -> float:
        c = self.cfg
        if self._t is None or now - self._t > 0.5:  # first sample / long gap
            dt, d_raw = None, 0.0
        else:
            dt = max(1e-3, now - self._t)
            d_raw = (err - self._prev_err) / dt
        self._d = c.d_alpha * d_raw + (1.0 - c.d_alpha) * self._d
        u = float(np.clip(c.kp * err + c.kd * self._d, -c.steer_limit, c.steer_limit))
        if dt is not None and c.steer_rate_limit > 0:  # smooth steering
            step = c.steer_rate_limit * dt
            u = self.out + float(np.clip(u - self.out, -step, step))
        self._t, self._prev_err, self.out = now, err, u
        return u


# =============================================================================
# Traffic light: HSV colour AND apparent-size (distance) gating
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
    """
    Pinhole model:  h_px = f_px * D / Z   =>   Z = f_px * D / h_px
    A lamp only counts as 'at the stop line' when h_px >= h_stop_px, which is
    the same as Z <= light_stop_dist_m.  Far-away ambient lights are small, so
    they are rejected by the size gate even if their colour matches.
    """

    def __init__(self, cfg: Config, frame_w: int):
        self.cfg = cfg
        self.f_px = (frame_w / 2.0) / math.tan(math.radians(cfg.hfov_deg / 2.0))
        self.h_stop_px = self.f_px * cfg.light_diameter_m / cfg.light_stop_dist_m
        self._k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
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
            if not 0.6 <= w / float(h) <= 1.6:           # roughly round
                continue
            per = cv2.arcLength(cnt, True)
            if per <= 0 or 4.0 * math.pi * area / (per * per) < c.light_min_circ:
                continue
            if area / float(w * h) < 0.5:                # circle fills ~0.785 of its box
                continue
            if best is None or h > best[3]:
                best = (x, y, w, h)
        return best

    def detect(self, frame: np.ndarray) -> LightDetection:
        c = self.cfg
        h = frame.shape[0]
        roi = frame[: int(h * c.light_roi_bottom_frac), :]
        hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
        s, v = c.light_s_min, c.light_v_min
        red = cv2.inRange(hsv, (0, s, v), (10, 255, 255)) | \
              cv2.inRange(hsv, (170, s, v), (180, 255, 255))
        green = cv2.inRange(hsv, (40, s, v), (90, 255, 255))
        r, g = self._best(red), self._best(green)
        if r is None and g is None:
            return LightDetection(LightColor.NONE, None, 0, None)
        # Fail-safe: red wins ties.
        if r is not None and (g is None or r[3] >= g[3]):
            color, box = LightColor.RED, r
        else:
            color, box = LightColor.GREEN, g
        return LightDetection(color, box, box[3], self.f_px * c.light_diameter_m / box[3])


class StopState(Enum):
    GO = 0
    SLOW = 1
    STOP = 2


class StopLogic:
    """Debounced GO / SLOW / STOP machine.  STOP latches until a *near* green."""

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
# Main control loop
# =============================================================================
_running = True


def _on_signal(signum, _frame):
    global _running
    _running = False


def run(cfg: Config, show: bool, serial_enabled: bool) -> int:
    cam = Camera(cfg)
    link = SerialLink(cfg, enabled=serial_enabled)
    lane = LaneDetector(cfg)
    pd = PDController(cfg)
    light = TrafficLightDetector(cfg, cfg.width)
    stop_logic = StopLogic(cfg, light)

    cam.start()
    link.start()

    last_seq = -1
    last_good_t = 0.0
    last_steer = 0.0
    t_report = time.monotonic()
    n_frames, proc_acc = 0, 0.0

    try:
        while _running:
            frame, ts, seq = cam.latest()
            now = time.monotonic()

            if frame is None or now - ts > cfg.cam_stale_s:
                link.set_command(0, 0)  # camera dead -> stop (ESP32 also has a watchdog)
                pd.reset()
                time.sleep(0.01)
                continue
            if seq == last_seq:
                time.sleep(0.002)
                continue
            last_seq = seq
            t0 = time.monotonic()

            if frame.shape[1] != cfg.width or frame.shape[0] != cfg.height:
                frame = cv2.resize(frame, (cfg.width, cfg.height))

            lres = lane.process(frame, draw=show)
            ldet = light.detect(frame)
            state, scale = stop_logic.update(ldet)

            # ---- steering ----
            if lres.error is not None:
                last_good_t = now
                steer = pd.update(lres.error, ts)
                last_steer = steer
                lane_ok = True
            elif now - last_good_t < cfg.lane_lost_timeout_s:
                steer = last_steer * 0.9       # coast briefly on last steering
                last_steer = steer
                lane_ok = True
            else:
                steer = 0.0                    # lane lost too long -> stop
                pd.reset()
                lane_ok = False

            # ---- speed ----
            if lane_ok and scale > 0.0:
                speed = cfg.base_speed * scale * (1.0 - cfg.curve_slowdown * abs(steer))
                speed = max(speed, cfg.min_speed * scale)
            else:
                speed, steer = 0.0, 0.0        # no pivoting while stopped

            tel = link.telemetry()
            if tel is not None and tel.estop:  # ESP32 is overriding anyway
                speed, steer = 0.0, 0.0

            link.set_command(int(round(speed)), int(round(steer * 100)))

            # ---- diagnostics ----
            proc_acc += time.monotonic() - t0
            n_frames += 1
            if now - t_report > 5.0:
                log.info("fps=%.1f proc=%.1fms state=%s err=%s steer=%.2f speed=%.0f "
                         "tel=%s tx_skip=%d rx_bad=%d",
                         n_frames / (now - t_report), 1000 * proc_acc / max(1, n_frames),
                         state.name, None if lres.error is None else round(lres.error, 3),
                         steer, speed,
                         None if tel is None else tel.dist_mm, link.tx_skipped, link.rx_bad)
                t_report, n_frames, proc_acc = now, 0, 0.0

            if show:
                vis = frame.copy()
                if ldet.bbox:
                    x, y, w, h = ldet.bbox
                    col = (0, 0, 255) if ldet.color == LightColor.RED else (0, 255, 0)
                    cv2.rectangle(vis, (x, y), (x + w, y + h), col, 2)
                cv2.putText(vis, f"{state.name} spd={speed:.0f} str={steer:+.2f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("frame", vis)
                if lres.debug is not None:
                    cv2.imshow("lane", lres.debug)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        log.info("shutting down: commanding stop")
        link.stop()
        cam.stop()
        if show:
            cv2.destroyAllWindows()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="AGV vision/control (Raspberry Pi 5)")
    ap.add_argument("--port", default=Config.port)
    ap.add_argument("--baud", type=int, default=Config.baud)
    ap.add_argument("--camera", default=Config.cam_source,
                    help="device path or index (default /dev/video0)")
    ap.add_argument("--video", help="video file for bench testing (overrides --camera)")
    ap.add_argument("--no-serial", action="store_true", help="vision only, no UART")
    ap.add_argument("--show", action="store_true", help="debug windows (needs a display)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)s %(message)s")
    if serial is None and not args.no_serial:
        log.error("pyserial missing: pip install pyserial (or use --no-serial)")
        return 2

    cam_src: Union[int, str] = args.video or args.camera
    if isinstance(cam_src, str) and cam_src.isdigit():
        cam_src = int(cam_src)
    cfg = Config(cam_source=cam_src, port=args.port, baud=args.baud)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    return run(cfg, show=args.show, serial_enabled=not args.no_serial)


if __name__ == "__main__":
    sys.exit(main())
