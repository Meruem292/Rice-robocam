#!/usr/bin/env bash
# ==============================================================================
# RiceTrack Rover — Full In-Field Autonomous Boot Setup
# ==============================================================================
# Configures the Raspberry Pi 4B so that on power switch flip:
# 1. In-Field Wi-Fi Hotspot ("RiceTrack-Rover", 192.168.4.1) starts automatically.
# 2. RiceTrack Web Application & ESP32 Bridge starts automatically via systemd.
#
# No monitor, no keyboard, and no terminal typing required in the field!
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Determine the non-root user who invoked the script
ACTUAL_USER="${SUDO_USER:-$USER}"
if [ "$ACTUAL_USER" = "root" ]; then
    ACTUAL_USER=$(stat -c '%U' "$SCRIPT_DIR" 2>/dev/null || echo "root")
fi

echo "========================================================"
echo "🌾 RiceTrack — Automatic In-Field Boot Configuration"
echo "========================================================"
echo "• User account     : $ACTUAL_USER"
echo "• Install directory: $SCRIPT_DIR"
echo "========================================================"

# Step 1: Configure In-Field Wi-Fi Hotspot (Auto-Connect on Boot)
echo "[1/3] Setting up In-Field Wi-Fi Hotspot (Auto-Connect on Boot)..."
if [ -f "$SCRIPT_DIR/create_wifi_ap.sh" ]; then
    chmod +x "$SCRIPT_DIR/create_wifi_ap.sh"
    sudo "$SCRIPT_DIR/create_wifi_ap.sh"
else
    echo "❌ create_wifi_ap.sh not found!"
    exit 1
fi

# Step 2: Ensure virtual environment exists
if [ ! -f "$SCRIPT_DIR/.venv/bin/python3" ]; then
    echo "⚠️ Python virtual environment (.venv) not found. Running setup_rpi.sh..."
    chmod +x "$SCRIPT_DIR/setup_rpi.sh"
    "$SCRIPT_DIR/setup_rpi.sh"
fi

# Step 3: Install & Enable systemd Service for Web App & Hardware Controller
echo "[2/3] Generating systemd service (/etc/systemd/system/ricetrack.service)..."

SERVICE_FILE="/etc/systemd/system/ricetrack.service"

sudo bash -c "cat <<EOF > $SERVICE_FILE
[Unit]
Description=RiceTrack Agricultural Rover Web Server & Vision Service
After=network.target network-online.target multi-user.target
Wants=network.target

[Service]
Type=simple
User=$ACTUAL_USER
WorkingDirectory=$SCRIPT_DIR
Environment=\"PATH=$SCRIPT_DIR/.venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin\"
Environment=\"PYTHONUNBUFFERED=1\"
Environment=\"RICETRACK_HOST=0.0.0.0\"
Environment=\"RICETRACK_PORT=8000\"

# Ensure In-Field Hotspot is up before web server binds
ExecStartPre=-/usr/bin/nmcli connection up RiceTrack-Rover

ExecStart=$SCRIPT_DIR/.venv/bin/python3 web_app.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF"

echo "[3/3] Enabling and starting ricetrack.service..."
sudo systemctl daemon-reload
sudo systemctl enable ricetrack.service
sudo systemctl restart ricetrack.service

echo ""
echo "========================================================"
echo "🎉 In-Field Autonomous Boot is now FULLY ENABLED!"
echo "========================================================"
echo "Every time you power ON the Raspberry Pi / Rover:"
echo " 1. Wi-Fi Hotspot 'RiceTrack-Rover' will broadcast automatically."
echo " 2. Web App and ESP32 control will start automatically on port 8000."
echo ""
echo "In-Field Connection details:"
echo " • Wi-Fi Network (SSID) : RiceTrack-Rover"
echo " • Wi-Fi Password       : ricetrack123"
echo " • Browser URL           : http://192.168.4.1:8000"
echo ""
echo "To check live service status anytime:"
echo "   sudo systemctl status ricetrack.service"
echo "========================================================"
