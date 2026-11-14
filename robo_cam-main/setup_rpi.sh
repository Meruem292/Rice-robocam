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
echo "[1/5] Installing required Linux system libraries..."
sudo apt-get update
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    libgl1 \
    libglib2.0-0 \
    v4l-utils \
    network-manager

# 2. Grant serial port (/dev/ttyUSB0) and camera (/dev/video*) permissions to the current user
echo "[2/5] Configuring user hardware permissions (dialout, video, tty)..."
sudo usermod -a -G dialout,video,tty "$USER"

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

# 5. Make launcher executable
chmod +x run_web.sh || true
chmod +x create_wifi_ap.sh || true

echo "========================================================"
echo "✅ Setup Complete!"
echo "To launch the web interface manually:"
echo "    ./run_web.sh"
echo ""
echo "To auto-start on boot via systemd:"
echo "    sudo cp ricetrack.service /etc/systemd/system/"
echo "    sudo systemctl daemon-reload"
echo "    sudo systemctl enable --now ricetrack.service"
echo "========================================================"
