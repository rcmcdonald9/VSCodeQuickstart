#!/usr/bin/env bash
# One-time setup on a fresh Raspberry Pi OS (64-bit, Bookworm or newer) on a Pi 5.
# Run from the repo:   bash software/scripts/setup_pi.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"          # .../software
CONFIG_TXT=/boot/firmware/config.txt

echo "== apt packages"
sudo apt-get update
sudo apt-get install -y python3-venv python3-picamera2 python3-libcamera rpicam-apps \
  i2c-tools espeak-ng alsa-utils portaudio19-dev libportaudio2 wget unzip

echo "== enable I2C (ToF sensor) + serial access for the servo bus adapter"
sudo raspi-config nonint do_i2c 0
sudo usermod -aG dialout "$USER"

echo "== I2S speaker amp (MAX98357A) + full USB current"
add_line() { grep -qxF "$1" "$CONFIG_TXT" || echo "$1" | sudo tee -a "$CONFIG_TXT" >/dev/null; }
add_line "dtoverlay=max98357a"
add_line "usb_max_current_enable=1"

echo "== python venv (system site packages so picamera2 is visible)"
python3 -m venv --system-site-packages "$HERE/.venv"
"$HERE/.venv/bin/pip" install --upgrade pip
"$HERE/.venv/bin/pip" install -e "$HERE[pi,dev]"

echo "== offline speech models"
mkdir -p ~/models/piper
if [ ! -d ~/models/vosk-model-small-en-us-0.15 ]; then
  wget -q -O /tmp/vosk.zip https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  unzip -q /tmp/vosk.zip -d ~/models && rm /tmp/vosk.zip
fi
VOICE=https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium
[ -f ~/models/piper/en_US-lessac-medium.onnx ] || {
  wget -q -O ~/models/piper/en_US-lessac-medium.onnx      "$VOICE/en_US-lessac-medium.onnx"
  wget -q -O ~/models/piper/en_US-lessac-medium.onnx.json "$VOICE/en_US-lessac-medium.onnx.json"
}

echo "== object detector (YOLO11n -> NCNN for fast CPU inference)"
bash "$HERE/scripts/export_yolo.sh"

echo "== API key file for the Claude brain (optional)"
mkdir -p ~/.config
[ -f ~/.config/robodog.env ] || { echo "ANTHROPIC_API_KEY=" > ~/.config/robodog.env; chmod 600 ~/.config/robodog.env; }

echo
echo "Done. Reboot once (I2C/I2S changes), then check:"
echo "  i2cdetect -y 1        # expect 0x29 (VL53L1X)"
echo "  $HERE/.venv/bin/python -m robodog.calibrate --scan   # expect servo IDs 1-10"
echo "  rpicam-hello -t 3000  # camera"
echo "  speaker-test -t wav -c 1 -l 1"
echo "Then put your key in ~/.config/robodog.env and run:  $HERE/.venv/bin/python -m robodog --text"
