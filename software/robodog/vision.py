"""Camera capture + object detection, running in a background thread."""
from __future__ import annotations

import io
import logging
import threading
import time
from dataclasses import dataclass

log = logging.getLogger(__name__)


@dataclass
class Detection:
    label: str
    confidence: float
    cx: float   # bbox centre, 0..1 (0 = left edge of the image)
    cy: float
    w: float    # bbox size, 0..1
    h: float

    def describe(self) -> str:
        side = "left" if self.cx < 0.4 else "right" if self.cx > 0.6 else "ahead"
        return f"{self.label} ({self.confidence:.0%}, {side}, size {self.h:.2f})"


# ----------------------------------------------------------------- cameras
class MockCamera:
    def __init__(self, resolution):
        self.resolution = tuple(resolution)

    def capture(self):
        import numpy as np

        return np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)


class PiCamera:
    """Raspberry Pi Camera Module 3 via picamera2 (returns RGB numpy arrays)."""

    def __init__(self, resolution):
        from picamera2 import Picamera2

        self.cam = Picamera2()
        conf = self.cam.create_video_configuration(main={"size": tuple(resolution), "format": "RGB888"})
        self.cam.configure(conf)
        self.cam.start()
        time.sleep(0.5)

    def capture(self):
        # picamera2's "RGB888" is BGR byte order in memory; flip to RGB
        return self.cam.capture_array()[:, :, ::-1].copy()


class OpenCVCamera:
    def __init__(self, resolution, index=0):
        import cv2

        self.cv2 = cv2
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

    def capture(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        return self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)


# --------------------------------------------------------------- detectors
class NullDetector:
    def detect(self, rgb) -> list[Detection]:
        return []


class YoloDetector:
    """Ultralytics YOLO (80 COCO classes: person, dog, cat, chair, cup, bottle, ...).

    On a Pi 5 the NCNN export of yolo11n runs at roughly 8-12 FPS on the CPU.
    """

    def __init__(self, model: str, confidence: float):
        from ultralytics import YOLO

        self.model = YOLO(model, task="detect")
        self.conf = confidence

    def detect(self, rgb) -> list[Detection]:
        res = self.model.predict(rgb[:, :, ::-1], conf=self.conf, verbose=False, imgsz=480)[0]
        names = res.names
        out = []
        for box in res.boxes:
            cx, cy, w, h = box.xywhn[0].tolist()
            out.append(Detection(names[int(box.cls)], float(box.conf), cx, cy, w, h))
        return out


# ------------------------------------------------------------------ vision
class Vision:
    def __init__(self, cfg: dict):
        v = cfg["vision"]
        res = v.get("resolution", [640, 480])
        kind = v.get("camera", "mock")
        try:
            self.camera = PiCamera(res) if kind == "picamera2" else OpenCVCamera(res) if kind == "opencv" else MockCamera(res)
        except Exception as e:
            log.error("camera %s unavailable (%s); using mock camera", kind, e)
            self.camera = MockCamera(res)
        try:
            self.detector = YoloDetector(v["yolo_model"], v.get("confidence", 0.45)) if v.get("detector") == "yolo" else NullDetector()
        except Exception as e:
            log.error("detector unavailable (%s); object detection disabled", e)
            self.detector = NullDetector()
        self.period = 1.0 / max(1, v.get("fps", 8))
        self._lock = threading.Lock()
        self._frame = None
        self._detections: list[Detection] = []
        self._stamp = 0.0
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._run, name="vision", daemon=True).start()

    def stop(self):
        self._running = False

    def _run(self):
        while self._running:
            t0 = time.monotonic()
            try:
                frame = self.camera.capture()
                dets = self.detector.detect(frame)
                with self._lock:
                    self._frame, self._detections, self._stamp = frame, dets, time.monotonic()
            except Exception:
                log.exception("vision loop error")
                time.sleep(1)
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))

    def detections(self, max_age_s: float = 1.0) -> list[Detection]:
        with self._lock:
            if time.monotonic() - self._stamp > max_age_s:
                return []
            return list(self._detections)

    def find(self, label: str) -> Detection | None:
        """Best (largest) detection whose label matches; 'me'/'person' are aliases."""
        label = label.lower().strip()
        aliases = {"me": "person", "human": "person", "someone": "person", "ball": "sports ball",
                   "phone": "cell phone", "sofa": "couch", "plant": "potted plant", "table": "dining table"}
        label = aliases.get(label, label)
        hits = [d for d in self.detections() if d.label == label]
        return max(hits, key=lambda d: d.w * d.h) if hits else None

    def jpeg(self, quality: int = 80) -> bytes | None:
        """Latest frame as JPEG bytes (for sending to the AI model)."""
        with self._lock:
            frame = self._frame
        if frame is None:
            return None
        from PIL import Image

        buf = io.BytesIO()
        Image.fromarray(frame).save(buf, format="JPEG", quality=quality)
        return buf.getvalue()
