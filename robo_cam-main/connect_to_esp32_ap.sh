#!/usr/bin/env bash
# ==============================================================================
# RiceTrack — Connect Raspberry Pi 4B to ESP32 Wi-Fi AP & Enable Auto-Connect on Boot
# ==============================================================================
# Architecture:
# - ESP32 hosts the Wi-Fi AP (SSID: RiceTrack-Rover, IP: 192.168.4.1).
# - Raspberry Pi connects as a client station with Static IP: 192.168.4.50.
# - A persistent systemd service (esp32-wifi-connect.service) ensures that the
#   Pi automatically connects on EVERY boot and reconnects if link ever drops.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SSID="RiceTrack-Rover"
PASS="ricetrack123"
PI_IP="192.168.4.50/24"
GATEWAY="192.168.4.1"
CON_NAME="RiceTrack-Rover-Client"

echo "========================================================"
echo "🌾 RiceTrack — ESP32 AP Auto-Connect Configuration"
echo "========================================================"
echo "• Target Wi-Fi SSID : $SSID"
echo "• Raspberry Pi IP   : 192.168.4.50 (Static)"
echo "• ESP32 AP Gateway  : 192.168.4.1"
echo "========================================================"

# 1. Unblock Wi-Fi radio and disable power-saving sleep
echo "[1/4] Unblocking Wi-Fi and disabling power management..."
sudo rfkill unblock wifi 2>/dev/null || true
sudo iw wlan0 set power_save off 2>/dev/null || true

# 2. Clean up any obsolete AP or conflicting profiles
echo "[2/4] Removing obsolete/conflicting NetworkManager profiles..."
sudo nmcli connection delete "$SSID" 2>/dev/null || true
sudo nmcli connection delete "$CON_NAME" 2>/dev/null || true

# 3. Create clean Station (Infrastructure) profile with infinite autoconnect retries
echo "[3/4] Creating persistent auto-connect Wi-Fi profile ($CON_NAME)..."
sudo nmcli connection add \
    type wifi \
    ifname wlan0 \
    con-name "$CON_NAME" \
    ssid "$SSID" \
    mode infrastructure

sudo nmcli connection modify "$CON_NAME" 802-11-wireless-security.key-mgmt wpa-psk
sudo nmcli connection modify "$CON_NAME" 802-11-wireless-security.psk "$PASS"
sudo nmcli connection modify "$CON_NAME" 802-11-wireless.powersave 2
sudo nmcli connection modify "$CON_NAME" ipv4.method manual
sudo nmcli connection modify "$CON_NAME" ipv4.addresses "$PI_IP"
sudo nmcli connection modify "$CON_NAME" ipv4.gateway "$GATEWAY"
sudo nmcli connection modify "$CON_NAME" ipv4.dns "$GATEWAY"
sudo nmcli connection modify "$CON_NAME" connection.autoconnect yes
sudo nmcli connection modify "$CON_NAME" connection.autoconnect-priority 100
sudo nmcli connection modify "$CON_NAME" connection.autoconnect-retries 0
sudo nmcli connection modify "$CON_NAME" connection.auth-retries 0

# 4. Install & Enable Persistent Auto-Connect Systemd Watchdog Service
echo "[4/4] Installing systemd watchdog (/etc/systemd/system/esp32-wifi-connect.service)..."
chmod +x "$SCRIPT_DIR/esp32_wifi_watchdog.sh"

WATCHDOG_SERVICE="/etc/systemd/system/esp32-wifi-connect.service"
sudo bash -c "cat <<EOF > $WATCHDOG_SERVICE
[Unit]
Description=RiceTrack ESP32 Wi-Fi Auto-Connect Watchdog
After=network.target NetworkManager.service
Wants=network.target NetworkManager.service

[Service]
Type=simple
User=root
WorkingDirectory=$SCRIPT_DIR
ExecStart=/bin/bash $SCRIPT_DIR/esp32_wifi_watchdog.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF"

sudo systemctl daemon-reload
sudo systemctl enable esp32-wifi-connect.service
sudo systemctl restart esp32-wifi-connect.service

# Attempt immediate activation
echo ""
echo "Activating Wi-Fi connection to $SSID..."
sudo nmcli connection up "$CON_NAME" 2>/dev/null || {
    echo "⚠️ Note: ESP32 AP not currently visible in immediate scan."
    echo "   The background service will auto-connect the moment the ESP32 is powered ON!"
}

echo ""
echo "========================================================"
echo "✅ Automatic Boot-Up Connection is now FULLY ENABLED!"
echo "• Every time the Raspberry Pi powers on, it will automatically"
echo "  scan and connect to '$SSID' at 192.168.4.50."
echo "• If the ESP32 is turned on after the Pi, it will connect"
echo "  within seconds automatically."
echo "• Camera Stream URL: http://192.168.4.50:8000/api/video_feed"
echo "• ESP32 Dashboard  : http://192.168.4.1"
echo "========================================================"
