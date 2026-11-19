#!/usr/bin/env bash
# ==============================================================================
# Disable RiceTrack Automatic Boot Services
# ==============================================================================

echo "Disabling RiceTrack auto-start services..."
sudo systemctl stop ricetrack.service 2>/dev/null || true
sudo systemctl disable ricetrack.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/ricetrack.service

sudo systemctl stop esp32-wifi-connect.service 2>/dev/null || true
sudo systemctl disable esp32-wifi-connect.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/esp32-wifi-connect.service

sudo systemctl daemon-reload
echo "✅ RiceTrack auto-start services disabled."
