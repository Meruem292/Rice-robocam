#!/usr/bin/env bash
# ==============================================================================
# RiceTrack Field Wi-Fi Access Point Setup Script (Raspberry Pi OS Bookworm/Bullseye)
# ==============================================================================
# Turns the Raspberry Pi onboard Wi-Fi into an Access Point so you can control
# the rover from your smartphone or tablet in remote fields without a router.
# ==============================================================================

SSID="RiceTrack-Rover"
PASSWORD="ricetrack123"

echo "========================================================"
echo "🌾 Configuring In-Field Wi-Fi Hotspot: $SSID"
echo "========================================================"

# Check if nmcli (NetworkManager) is available
if command -v nmcli >/dev/null 2>&1; then
    echo "Using NetworkManager (nmcli)..."
    
    # Delete existing hotspot connection if already configured
    nmcli connection delete "$SSID" 2>/dev/null || true

    # Create new Hotspot connection
    nmcli connection add \
        type wifi \
        ifname wlan0 \
        con-name "$SSID" \
        autoconnect yes \
        ssid "$SSID"

    nmcli connection modify "$SSID" 802-11-wireless.mode ap
    nmcli connection modify "$SSID" 802-11-wireless.band bg
    nmcli connection modify "$SSID" 802-11-wireless-security.key-mgmt wpa-psk
    nmcli connection modify "$SSID" 802-11-wireless-security.psk "$PASSWORD"
    nmcli connection modify "$SSID" ipv4.method shared
    nmcli connection modify "$SSID" ipv4.addresses 192.168.4.1/24

    # Activate Hotspot
    nmcli connection up "$SSID"

    echo ""
    echo "✅ Hotspot '$SSID' is now ACTIVE!"
    echo "• Connect your smartphone/laptop Wi-Fi to: $SSID"
    echo "• Wi-Fi Password: $PASSWORD"
    echo "• Rover Web Dashboard URL: http://192.168.4.1:8000"
    echo "========================================================"
else
    echo "❌ NetworkManager (nmcli) is not installed."
    echo "Install it via: sudo apt-get install -y network-manager"
    exit 1
fi
