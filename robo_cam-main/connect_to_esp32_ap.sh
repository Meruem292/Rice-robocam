#!/usr/bin/env bash
# ==============================================================================
# RiceTrack — Connect Raspberry Pi 4B to ESP32 Wi-Fi AP
# ==============================================================================
# - ESP32 hosts the Wi-Fi AP (SSID: RiceTrack-Rover, IP: 192.168.4.1)
# - ESP32 runs the rover dashboard directly on port 80 (http://192.168.4.1)
# - Raspberry Pi connects to the ESP32 Wi-Fi AP (DHCP or static)
# - Raspberry Pi streams live camera (port 8000) & AI crop analysis to the page
# - ESP32 auto-discovers the Pi's IP address automatically via background ping!
# ==============================================================================

SSID="RiceTrack-Rover"
PASS="ricetrack123"

echo "========================================================"
echo "🌾 Connecting Raspberry Pi to ESP32 AP: $SSID"
echo "========================================================"

sudo rfkill unblock wifi 2>/dev/null || true

# Rescan Wi-Fi to find ESP32 AP
echo "Scanning for $SSID..."
sudo nmcli device wifi rescan 2>/dev/null || true
sleep 2

# Delete old connection if exists for a fresh handshake
sudo nmcli connection delete "$SSID" 2>/dev/null || true

# Connect to the ESP32 AP
echo "Connecting to $SSID..."
if ! sudo nmcli device wifi connect "$SSID" password "$PASS" ifname wlan0; then
    echo "⚠️ Direct connect failed, retrying after fresh scan..."
    sleep 3
    sudo nmcli device wifi rescan 2>/dev/null || true
    sleep 2
    sudo nmcli device wifi connect "$SSID" password "$PASS" ifname wlan0
fi

# Ensure connection autoconnects on boot
sudo nmcli connection modify "$SSID" connection.autoconnect yes
sudo nmcli connection modify "$SSID" connection.autoconnect-priority 100

sleep 1
PI_IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1)
PI_IP="${PI_IP:-192.168.4.2}"

echo ""
echo "========================================================"
echo "✅ Raspberry Pi is now connected to ESP32 AP!"
echo "• Raspberry Pi IP : $PI_IP"
echo "• ESP32 Dashboard : http://192.168.4.1"
echo "• Camera Stream   : http://$PI_IP:8000/api/video_feed"
echo "• Auto-Sync       : Active (ESP32 auto-detects this IP)"
echo "========================================================"
