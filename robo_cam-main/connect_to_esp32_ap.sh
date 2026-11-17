#!/usr/bin/env bash
# ==============================================================================
# RiceTrack — Connect Raspberry Pi 4B to ESP32 Wi-Fi AP
# ==============================================================================
# In this architecture:
# - ESP32 hosts the Wi-Fi AP (SSID: RiceTrack-Rover, IP: 192.168.4.1)
# - ESP32 runs the high-speed motor control dashboard directly on port 80.
# - Raspberry Pi connects to the ESP32 Wi-Fi AP with Static IP: 192.168.4.2
# - Raspberry Pi streams live camera (port 8000) & AI crop analysis to the page.
# ==============================================================================

SSID="RiceTrack-Rover"
PASS="ricetrack123"
PI_IP="192.168.4.2/24"
GATEWAY="192.168.4.1"

echo "========================================================"
echo "🌾 Connecting Raspberry Pi to ESP32 AP: $SSID"
echo "========================================================"

sudo rfkill unblock wifi 2>/dev/null || true

# Rescan Wi-Fi to find ESP32 AP
echo "Scanning for $SSID..."
sudo nmcli device wifi rescan 2>/dev/null || true
sleep 2

# Delete old connection if exists
sudo nmcli connection delete "$SSID" 2>/dev/null || true

# Connect to the ESP32 AP
echo "Connecting to $SSID..."
sudo nmcli device wifi connect "$SSID" password "$PASS" ifname wlan0 || {
    echo "⚠️ Direct connect failed, retrying after fresh scan..."
    sleep 3
    sudo nmcli device wifi rescan 2>/dev/null || true
    sleep 2
    sudo nmcli device wifi connect "$SSID" password "$PASS" ifname wlan0
}

# Set static IP on ESP32 subnet so the web page always finds the camera at 192.168.4.2
sudo nmcli connection modify "$SSID" ipv4.method manual ipv4.addresses "$PI_IP" ipv4.gateway "$GATEWAY" ipv4.dns "$GATEWAY"
sudo nmcli connection modify "$SSID" connection.autoconnect yes
sudo nmcli connection modify "$SSID" connection.autoconnect-priority 100

sudo nmcli connection up "$SSID"

echo ""
echo "========================================================"
echo "✅ Raspberry Pi is now connected to ESP32 AP!"
echo "• Raspberry Pi IP: 192.168.4.2"
echo "• ESP32 Dashboard: http://192.168.4.1"
echo "• Camera Stream  : http://192.168.4.2:8000/api/video_feed"
echo "========================================================"
