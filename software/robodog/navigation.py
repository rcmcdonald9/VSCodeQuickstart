"""Behaviours: walk/turn by distance, explore, find / go to / follow an object.

Exactly one behaviour runs at a time in its own thread. Starting a new one
(or calling stop()) cancels the previous. Each run returns a short result
string, which the brain reports back to the AI model / the user.
"""
from __future__ import annotations

import logging
import math
import random
import threading
import time

log = logging.getLogger(__name__)


class Cancelled(Exception):
    pass


class Navigator:
    def __init__(self, motion, vision, tof):
        self.motion = motion
        self.vision = vision
        self.tof = tof
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None
        self._result: str | None = None
        self.current: str | None = None
        self._lock = threading.Lock()

    # -------------------------------------------------------- lifecycle
    def run(self, name: str, fn, *args, wait: bool = True, timeout: float = 60.0) -> str:
        """Cancel whatever is running, start fn(*args). If wait, block for its result."""
        self.stop()
        self._cancel.clear()
        done = threading.Event()

        def target():
            try:
                self._result = fn(*args)
            except Cancelled:
                self._result = f"{name} was cancelled"
            except Exception as e:  # report, don't crash
                log.exception("behaviour %s failed", name)
                self._result = f"{name} failed: {e}"
            finally:
                self.motion.stop()
                self.current = None
                done.set()

        with self._lock:
            self.current = name
            self._thread = threading.Thread(target=target, name=f"nav-{name}", daemon=True)
            self._thread.start()
        if not wait:
            return f"started {name}"
        if not done.wait(timeout):
            self.stop()
            return f"{name} timed out after {timeout:.0f}s"
        return self._result or ""

    def stop(self):
        self._cancel.set()
        t = self._thread
        if t and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=2)
        self.motion.stop()

    def _sleep(self, s: float):
        if self._cancel.wait(s):
            raise Cancelled()

    def _distance(self) -> float | None:
        return self.tof.distance_mm()

    # ------------------------------------------------------ primitives
    def walk(self, distance_cm: float, direction: str = "forward") -> str:
        speed = 120.0
        dist = abs(distance_cm) * 10
        if direction in ("left", "right"):      # no sideways stepping on v2: turn, walk, turn back
            self.turn(90 if direction == "left" else -90)
            r = self.walk(distance_cm, "forward")
            self.turn(-90 if direction == "left" else 90)
            return r + f" (went {direction} by turning)"
        vx, vy = {"forward": (speed, 0), "backward": (-speed, 0)}[direction]
        start = list(self.motion.odometry)
        self.motion.set_velocity(vx, vy, 0)
        t_end = time.monotonic() + dist / max(1, math.hypot(vx, vy)) * 1.6 + 2
        while time.monotonic() < t_end:
            moved = math.hypot(self.motion.odometry[0] - start[0], self.motion.odometry[1] - start[1])
            if moved >= dist:
                return f"walked about {moved / 10:.0f} cm {direction}"
            if self.motion.blocked:
                return f"stopped after about {moved / 10:.0f} cm: obstacle ahead"
            self._sleep(0.05)
        moved = math.hypot(self.motion.odometry[0] - start[0], self.motion.odometry[1] - start[1])
        return f"walked about {moved / 10:.0f} cm {direction} (ran out of time)"

    def turn(self, degrees: float) -> str:
        """Positive = turn left (counter-clockwise)."""
        rate = math.radians(40)
        start = self.motion.odometry[2]
        target = math.radians(degrees)
        self.motion.set_velocity(0, 0, rate if degrees > 0 else -rate)
        t_end = time.monotonic() + abs(target) / rate * 1.8 + 2
        while time.monotonic() < t_end:
            if abs(self.motion.odometry[2] - start) >= abs(target):
                break
            self._sleep(0.03)
        return f"turned {'left' if degrees > 0 else 'right'} about {abs(degrees):.0f} degrees"

    # ----------------------------------------------------- behaviours
    def explore(self, duration_s: float = 30.0) -> str:
        """Wander, steering away from obstacles. Reports what it saw."""
        seen: dict[str, int] = {}
        t_end = time.monotonic() + duration_s
        while time.monotonic() < t_end:
            for d in self.vision.detections():
                seen[d.label] = seen.get(d.label, 0) + 1
            dist = self._distance()
            if dist is not None and dist < 450:
                self.motion.set_velocity(0, 0, random.choice([-1, 1]) * math.radians(40))
                self._sleep(random.uniform(1.0, 2.2))
            else:
                self.motion.set_velocity(130, 0, random.uniform(-0.15, 0.15))
                self._sleep(0.3)
        things = ", ".join(sorted(seen, key=seen.get, reverse=True)[:8]) or "nothing I recognised"
        return f"explored for {duration_s:.0f}s; saw: {things}"

    def scan_with_head(self, label: str) -> float | None:
        """Sweep the head left-right; returns the pan angle where `label` was seen."""
        for pan in (0, 35, 60, -35, -60):
            self.motion.set_head(pan, 0)
            self._sleep(0.7)
            if self.vision.find(label):
                return float(pan)
        self.motion.set_head(0, 0)
        return None

    def find(self, label: str, timeout_s: float = 25.0) -> str:
        """Look around with the head, then turn the body until `label` is centred."""
        pan = self.scan_with_head(label)
        self.motion.set_head(0, 0)
        if pan:
            self.turn(pan)
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            d = self.vision.find(label)
            if d:
                if abs(d.cx - 0.5) < 0.08:
                    self.motion.stop()
                    return f"found {label}: {d.describe()}"
                self.motion.set_velocity(0, 0, -(d.cx - 0.5) * 1.6)   # left of centre -> turn left
            else:
                self.motion.set_velocity(0, 0, math.radians(30))
            self._sleep(0.1)
        return f"could not find {label} after looking around for {timeout_s:.0f}s"

    def go_to(self, label: str, timeout_s: float = 45.0) -> str:
        """Find `label`, then walk up to it and stop ~30 cm away."""
        r = self.find(label, timeout_s=min(20, timeout_s))
        if not r.startswith("found"):
            return r
        t_end = time.monotonic() + timeout_s
        lost_since = None
        while time.monotonic() < t_end:
            d = self.vision.find(label)
            dist = self._distance()
            if d:
                lost_since = None
                if d.h > 0.75 or (dist is not None and dist < 320):
                    self.motion.stop()
                    return f"arrived at the {label}"
                self.motion.set_velocity(140 * (1 - d.h), 0, -(d.cx - 0.5) * 1.4)
            else:
                lost_since = lost_since or time.monotonic()
                self.motion.set_velocity(0, 0, 0)
                if time.monotonic() - lost_since > 3:
                    return f"lost sight of the {label}"
            if self.motion.blocked and not d:
                return "blocked by an obstacle"
            self._sleep(0.1)
        return f"gave up walking to the {label} (timeout)"

    def follow(self, label: str = "person", duration_s: float = 60.0) -> str:
        """Keep `label` centred and at a comfortable distance."""
        t_end = time.monotonic() + duration_s
        lost_since = None
        while time.monotonic() < t_end:
            d = self.vision.find(label)
            if d:
                lost_since = None
                yaw = -(d.cx - 0.5) * 1.6
                target_h = 0.55 if label in ("person", "me") else 0.4
                vx = max(-80.0, min(160.0, (target_h - d.h) * 450))
                self.motion.set_velocity(vx, 0, yaw)
            else:
                lost_since = lost_since or time.monotonic()
                self.motion.set_velocity(0, 0, math.radians(25))   # search
                if time.monotonic() - lost_since > 8:
                    return f"lost the {label}; stopped following"
            self._sleep(0.1)
        return f"followed the {label} for {duration_s:.0f}s"
