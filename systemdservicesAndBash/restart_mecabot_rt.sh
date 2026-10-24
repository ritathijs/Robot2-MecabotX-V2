#!/usr/bin/env bash
# Zachte herstart van de RT-stack zonder terminal of bevestiging (bv. via SSH).
# Activeert de robot niet; de noodstop blijft ongemoeid.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="/home/wheeltec/wheeltec_ros2_rt/logs/restart_mecabot_rt.log"
mkdir -p "$(dirname "$LOG_FILE")"

{
    echo "=== Herstart RT-stack $(date) ==="
    bash "$SCRIPT_DIR/stop_mecabot_rt.sh"
    bash "$SCRIPT_DIR/startup_mecabot_rt.sh" --non-interactive
} >>"$LOG_FILE" 2>&1
