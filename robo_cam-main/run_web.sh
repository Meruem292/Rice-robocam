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

echo "🌾 Starting RiceTrack Web Application on http://0.0.0.0:8000 ..."
python3 web_app.py
