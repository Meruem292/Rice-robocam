#!/usr/bin/env bash
# ==============================================================================
# RiceTrack Raspberry Pi 4B Setup Script
# ==============================================================================
# Automates dependency installation, system permissions (dialout, video),
# Python virtual environment creation, and optional systemd service installation.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================================"
echo "🌾 RiceTrack — Raspberry Pi 4B + ESP32 Environment Setup"
echo "========================================================"

# 1. Update package lists and install required system packages
echo "[1/5] Installing required Linux system libraries & camera packages..."
sudo apt-get update
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-multipart \
    python3-picamera2 \
    python3-libcamera \
    libcamera-tools \
    libgl1 \
    libglib2.0-0 \
    v4l-utils \
    network-manager

# 2. Grant serial port (/dev/serial0, /dev/ttyUSB0) and camera permissions to the current user
echo "[2/5] Configuring user hardware permissions and Raspberry Pi UART / Camera..."
sudo usermod -a -G dialout,video,tty,gpio "$USER"

# Configure hardware UART on GPIO 14/15 and disable Linux login console over serial
echo "      - Enabling Primary Hardware UART on GPIO 14 (TX) / 15 (RX)..."
sudo raspi-config nonint do_serial_cons 1 || true # 1 = Disable serial login console
sudo raspi-config nonint do_serial_hw 0 || true   # 0 = Enable hardware serial port (/dev/serial0)
sudo raspi-config nonint do_camera 0 || true      # 0 = Enable camera interface if legacy stack

# 3. Create Python virtual environment if not already present
if [ ! -d ".venv" ]; then
    echo "[3/5] Creating Python virtual environment (.venv)..."
    python3 -m venv .venv --system-site-packages
else
    echo "[3/5] Python virtual environment already exists."
fi

# 4. Install Python dependencies
echo "[4/5] Installing Python dependencies from requirements.txt..."
source .venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

# 5. Make launcher and auto-start scripts executable
chmod +x run_web.sh || true
chmod +x create_wifi_ap.sh || true
chmod +x enable_autostart.sh || true
chmod +x disable_autostart.sh || true

echo "========================================================"
echo "✅ Setup Complete!"
echo ""
echo "To test/run manually right now:"
echo "    ./run_web.sh"
echo ""
echo "To enable 100% AUTOMATIC IN-FIELD BOOT (Hotspot + Web App):"
echo "    sudo ./enable_autostart.sh"
echo ""
echo "Once enabled, you never need to touch the terminal again!"
echo "Just power on the rover, connect to 'RiceTrack-Rover' Wi-Fi,"
echo "and open: http://192.168.4.1:8000"
echo "========================================================"
