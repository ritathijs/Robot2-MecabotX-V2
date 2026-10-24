#!/usr/bin/env bash
# Stopt de RT-stack gericht: eerst de herstartlussen/terminalwrappers, dan
# de ROS-processen netjes (SIGINT), daarna zo nodig geforceerd.

ROS_DISTRO="humble"
RT_WORKSPACE="/home/wheeltec/wheeltec_ros2_rt"

# De wrappers uit startup_mecabot_rt.sh bevatten deze tekst. Eerst stoppen,
# anders starten de while-true-lussen (MUX, camera) hun proces opnieuw.
pkill -TERM -f "Proces beëindigd" 2>/dev/null

pkill -INT -f "ros2 (run|launch)" 2>/dev/null
sleep 5

PATTERNS=(
    "rt_bt_node"
    "rt_robot_controller"
    "robot_state_manager"
    "quiz_socket_bridge"
    "rt_drive_to_coord"
    "rt_auto_recharger"
    "april_tabloo"
    "autocharge_batcheck"
    "drive_mux"
    "bumper"
    "visualtracker"
    "pointcloud_to_laserscan"
    "static_transform_publisher"
    "wheeltec_nav2"
    "turn_on_wheeltec_robot"
    "component_container"
    "gpio_reader"
    "lslidar_driver"
)

for pattern in "${PATTERNS[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null
done
sleep 3

for pattern in "${PATTERNS[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null
done
sleep 1

set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$RT_WORKSPACE/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1
sleep 2
ros2 daemon start >/dev/null 2>&1
sleep 2

remaining="$(ros2 node list 2>/dev/null)"
if [[ -n "$remaining" ]]; then
    echo "Nog actieve nodes:"
    echo "$remaining"
    exit 1
fi
echo "RT-stack gestopt."
