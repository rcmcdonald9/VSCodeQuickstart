"""Distance (VL53L1X time-of-flight) and optional IMU."""
from __future__ import annotations

import logging
import threading
import time

log = logging.getLogger(__name__)


class NullRangeSensor:
    def distance_mm(self) -> float | None:
        return None


class VL53L1XSensor:
    """Polls the ToF in a background thread so the 50 Hz motion loop never blocks on I2C."""

    def __init__(self):
        import board
        import adafruit_vl53l1x

        self._dev = adafruit_vl53l1x.VL53L1X(board.I2C())
        self._dev.distance_mode = 2      # long range (up to ~4 m)
        self._dev.timing_budget = 50
        self._dev.start_ranging()
        self._last: float | None = None
        threading.Thread(target=self._poll, name="tof", daemon=True).start()

    def _poll(self):
        while True:
            try:
                if self._dev.data_ready:
                    cm = self._dev.distance
                    self._dev.clear_interrupt()
                    self._last = cm * 10.0 if cm is not None else None
            except OSError as e:
                log.warning("ToF read failed: %s", e)
            time.sleep(0.02)

    def distance_mm(self) -> float | None:
        return self._last


class NullIMU:
    def orientation(self) -> tuple[float, float] | None:
        return None


class MPU6050IMU:
    """Very small tilt estimate from the accelerometer (roll, pitch in degrees)."""

    def __init__(self):
        import board
        import adafruit_mpu6050

        self._dev = adafruit_mpu6050.MPU6050(board.I2C())

    def orientation(self):
        import math

        ax, ay, az = self._dev.acceleration
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return roll, pitch


def make_sensors(cfg: dict):
    s = cfg["sensors"]
    tof, imu = NullRangeSensor(), NullIMU()
    try:
        if s.get("tof") == "vl53l1x":
            tof = VL53L1XSensor()
    except Exception as e:  # keep running without it, but say so loudly
        log.error("ToF sensor unavailable (%s) — obstacle stop is DISABLED", e)
    try:
        if s.get("imu") == "mpu6050":
            imu = MPU6050IMU()
    except Exception as e:
        log.error("IMU unavailable (%s)", e)
    return tof, imu
