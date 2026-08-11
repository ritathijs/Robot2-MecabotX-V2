#!/usr/bin/env bash

# --- LOGGING EN INITIALISATIE CONSTANTEN ---
LOG_DIR="/home/wheeltec/wheeltec_ros2/log"
mkdir -p "$LOG_DIR"

log_cmd() {
    local log_file="$1"
    echo "2>&1 | tee -a $LOG_DIR/$log_file"
}

SETUP_CMD="source /opt/ros/humble/setup.bash && source /home/wheeltec/wheeltec_ros2/install/setup.bash"

start_mecabot_integration_nodes() {
    gnome-terminal --title="Mecabot Integration Hub" -- bash -c "
        $SETUP_CMD && \
        ros2 launch mecabot_integration mecabot_integration.launch.py \
            $(log_cmd mecabot_integration.log) ; \
        exec bash
    "
}

start_mecabot_integration_nodes

