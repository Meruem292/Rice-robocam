#!/usr/bin/env bash
# ==============================================================================
# Disable RiceTrack Automatic Boot Service
# ==============================================================================

echo "Disabling RiceTrack auto-start service..."
sudo systemctl stop ricetrack.service 2>/dev/null || true
sudo systemctl disable ricetrack.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/ricetrack.service
sudo systemctl daemon-reload
echo "✅ RiceTrack auto-start service disabled."
