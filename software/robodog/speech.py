"""Speech in (offline Vosk) and speech out (offline Piper / espeak).

Everything runs locally on the Pi, so listening and talking work without
internet; only the optional Claude brain needs a connection.
"""
from __future__ import annotations

import json
import logging
import queue
import shutil
import subprocess
import tempfile
import threading

from .config import expand

log = logging.getLogger(__name__)


# ---------------------------------------------------------------- speaking
class Speaker:
    def __init__(self, cfg: dict):
        s = cfg["speech"]
        self.kind = s.get("tts", "print")
        self.voice = expand(s.get("piper_voice"))
        self.speaking = threading.Event()   # the listener ignores audio while set
        self._lock = threading.Lock()
        if self.kind == "piper" and not shutil.which("piper"):
            log.error("piper not found on PATH; falling back to espeak-ng")
            self.kind = "espeak"
        if self.kind == "espeak" and not shutil.which("espeak-ng"):
            self.kind = "print"

    def say(self, text: str):
        text = text.strip()
        if not text:
            return
        print(f"🐶 {text}", flush=True)
        if self.kind == "print":
            return
        with self._lock:
            self.speaking.set()
            try:
                if self.kind == "piper":
                    with tempfile.NamedTemporaryFile(suffix=".wav") as wav:
                        subprocess.run(["piper", "--model", self.voice, "--output_file", wav.name],
                                       input=text.encode(), check=True, capture_output=True)
                        subprocess.run(["aplay", "-q", wav.name], check=False)
                else:
                    subprocess.run(["espeak-ng", "-s", "165", text], check=False)
            except Exception as e:
                log.error("TTS failed: %s", e)
            finally:
                self.speaking.clear()


# --------------------------------------------------------------- listening
class TextListener:
    """Development stand-in: type commands instead of speaking them."""

    def __init__(self, wake_words):
        self.wake_words = wake_words

    def listen(self):
        while True:
            try:
                line = input("you> ").strip()
            except EOFError:
                return
            if line:
                yield line


class VoskListener:
    """Continuous offline recognition. Yields the command spoken after a wake word.

    "Hey Pip, sit down"      -> yields "sit down"
    "Hey Pip" ... "sit down" -> yields "sit down" (the next utterance within 8 s)
    """

    def __init__(self, cfg: dict, speaker: Speaker):
        import sounddevice as sd
        from vosk import KaldiRecognizer, Model, SetLogLevel

        SetLogLevel(-1)
        s = cfg["speech"]
        self.sd = sd
        self.rate = 16000
        self.device = s.get("mic_device")
        self.model = Model(expand(s["vosk_model"]))
        self.rec = KaldiRecognizer(self.model, self.rate)
        self.speaker = speaker
        self.wake_words = sorted((w.lower() for w in cfg["robot"]["wake_words"]), key=len, reverse=True)
        self.q: queue.Queue[bytes] = queue.Queue()

    def _callback(self, indata, frames, time_info, status):
        if not self.speaker.speaking.is_set():
            self.q.put(bytes(indata))

    def _strip_wake(self, text: str) -> str | None:
        for w in self.wake_words:
            if text == w or text.startswith(w + " "):
                return text[len(w):].strip()
        return None

    def listen(self):
        import time

        armed_until = 0.0
        with self.sd.RawInputStream(samplerate=self.rate, blocksize=8000, device=self.device,
                                    dtype="int16", channels=1, callback=self._callback):
            log.info("listening for %s", self.wake_words)
            while True:
                data = self.q.get()
                if not self.rec.AcceptWaveform(data):
                    continue
                text = json.loads(self.rec.Result()).get("text", "").strip().lower()
                if not text:
                    continue
                log.debug("heard: %s", text)
                cmd = self._strip_wake(text)
                if cmd is not None:
                    if cmd:
                        yield cmd
                    else:
                        armed_until = time.monotonic() + 8
                        self.speaker.say("Yes?")
                elif time.monotonic() < armed_until:
                    armed_until = 0.0
                    yield text
                elif text in ("stop", "stop stop", "halt", "freeze"):
                    yield "stop"          # stop always works, no wake word needed


def make_listener(cfg: dict, speaker: Speaker):
    if cfg["speech"].get("stt") == "vosk":
        try:
            return VoskListener(cfg, speaker)
        except Exception as e:
            log.error("Vosk unavailable (%s); falling back to typed commands", e)
    return TextListener(cfg["robot"]["wake_words"])
