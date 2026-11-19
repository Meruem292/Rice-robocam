#!/usr/bin/env bash
# ==============================================================================
# RiceTrack — Persistent Wi-Fi Auto-Connect Watchdog to ESP32 AP
# ==============================================================================
# Continuously monitors the Wi-Fi connection to the ESP32 AP ("RiceTrack-Rover").
# If disconnected, it unblocks rfkill, rescans, and connects with static IP 192.168.4.50.
# Retries indefinitely so the Pi reconnects on boot or after ESP32 reboot.
# ==============================================================================

SSID="RiceTrack-Rover"
PASS="ricetrack123"
PI_IP="192.168.4.50/24"
GATEWAY="192.168.4.1"
CON_NAME="RiceTrack-Rover-Client"

# Ensure Wi-Fi radio is on and disable power saving to prevent sleep drops
sudo rfkill unblock wifi 2>/dev/null || true
sudo iw wlan0 set power_save off 2>/dev/null || true

echo "🌾 [Wi-Fi Watchdog] Started for SSID: $SSID"

while true; do
    # 1. Check if we can ping the ESP32 gateway at 192.168.4.1
    if ping -c 1 -W 1 "$GATEWAY" >/dev/null 2>&1; then
        sleep 8
        continue
    fi

    # 2. Check if wlan0 is connected to RiceTrack-Rover
    CURRENT_SSID=$(iwgetid -r 2>/dev/null || nmcli -t -f active,ssid dev wifi 2>/dev/null | grep '^yes:' | cut -d: -f2 | head -n1)
    if [ "$CURRENT_SSID" = "$SSID" ]; then
        # Connected to SSID; verify IP is 192.168.4.50
        if ip -4 addr show wlan0 | grep -q "192.168.4.50"; then
            sleep 4
            continue
        fi
    fi

    echo "🌾 [Wi-Fi Watchdog] ESP32 AP ($SSID) not connected or unreachable. Connecting..."

    sudo rfkill unblock wifi 2>/dev/null || true
    sudo iw wlan0 set power_save off 2>/dev/null || true

    # 3. Try bringing up the configured NetworkManager profile
    if sudo nmcli connection up "$CON_NAME" 2>/dev/null; then
        echo "✅ [Wi-Fi Watchdog] Successfully activated $CON_NAME"
        sleep 8
        continue
    fi

    # 4. If direct activation failed, rescan Wi-Fi channels
    sudo nmcli device wifi rescan 2>/dev/null || true
    sleep 2

    # Check if ESP32 AP is visible in scan
    if nmcli -t -f SSID dev wifi 2>/dev/null | grep -q "^$SSID\$"; then
        echo "🌾 [Wi-Fi Watchdog] ESP32 AP detected in scan. Connecting..."
        sudo nmcli connection up "$CON_NAME" 2>/dev/null || \
        sudo nmcli device wifi connect "$SSID" password "$PASS" ifname wlan0 name "$CON_NAME" 2>/dev/null || true
        
        # Ensure static IP and autoconnect properties are preserved
        sudo nmcli connection modify "$CON_NAME" \
            ipv4.addresses "$PI_IP" \
            ipv4.gateway "$GATEWAY" \
            ipv4.dns "$GATEWAY" \
            ipv4.method manual \
            connection.autoconnect yes \
            connection.autoconnect-priority 100 \
            connection.autoconnect-retries 0 2>/dev/null || true
        sudo nmcli connection up "$CON_NAME" 2>/dev/null || true
    fi

    sleep 4
done
