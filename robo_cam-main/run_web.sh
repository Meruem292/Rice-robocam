#!/usr/bin/env bash
# ==============================================================================
# RiceTrack Web Application Launcher (Raspberry Pi / Linux)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -d ".venv" ]; then
    source .venv/bin/activate
fi

export RICETRACK_HOST="0.0.0.0"
export RICETRACK_PORT="8000"

# If ricetrack.service is running in background, stop it so manual session gets port 8000 & camera
WAS_SERVICE_RUNNING=0
if systemctl is-active --quiet ricetrack.service 2>/dev/null; then
    echo "⚠️  RiceTrack background service (ricetrack.service) is currently running on port 8000."
    echo "Stopping background service to give foreground manual session exclusive camera access..."
    sudo systemctl stop ricetrack.service 2>/dev/null || true
    WAS_SERVICE_RUNNING=1
    sleep 1
fi

# Ensure no stale process is holding port 8000
if command -v fuser >/dev/null 2>&1; then
    sudo fuser -k 8000/tcp >/dev/null 2>&1 || true
fi

cleanup() {
    echo ""
    echo "Stopping manual session..."
    if [ "$WAS_SERVICE_RUNNING" -eq 1 ]; then
        echo "Restarting background ricetrack.service..."
        sudo systemctl start ricetrack.service 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "🌾 Starting RiceTrack Web Application on http://0.0.0.0:8000 ..."
python3 web_app.py
cleanup
