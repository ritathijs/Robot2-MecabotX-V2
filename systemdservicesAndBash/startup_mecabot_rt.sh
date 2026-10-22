#!/usr/bin/env bash
set -Eeuo pipefail

ROS_DISTRO="humble"
BASE_WORKSPACE="/home/wheeltec/wheeltec_ros2"
RT_WORKSPACE="/home/wheeltec/wheeltec_ros2_rt"
LOG_DIR="$RT_WORKSPACE/logs"
RT_CONFIG="$RT_WORKSPACE/install/mecabot_integration/share/mecabot_integration/config/robot_locations.yaml"
RT_DATA_DIR="/home/wheeltec/.local/share/mecabot_rt"
ENABLE_LOGS="${ENABLE_LOGS:-true}"

ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
BASE_SETUP="$BASE_WORKSPACE/install/setup.bash"
RT_SETUP="$RT_WORKSPACE/install/setup.bash"

mkdir -p "$LOG_DIR"

fail() {
    echo "ERROR: $*" >&2
    exit 1
}

for required_file in "$ROS_SETUP" "$BASE_SETUP" "$RT_SETUP" "$RT_CONFIG"; do
    [[ -r "$required_file" ]] || fail "Benodigd setup/configbestand ontbreekt: $required_file"
done

[[ -e /dev/wheeltec_controller ]] || fail "Wheeltec-controller ontbreekt: /dev/wheeltec_controller"
command -v lsusb >/dev/null 2>&1 || fail "lsusb is niet beschikbaar."
lsusb | grep -qi "Orbbec" || fail "Orbbec-camera niet gevonden."
command -v gnome-terminal >/dev/null 2>&1 || fail "gnome-terminal is niet beschikbaar."

[[ -r "$RT_DATA_DIR/Charger_Position.json" ]] ||
    fail "RT-laadpaalbestand ontbreekt: $RT_DATA_DIR/Charger_Position.json"
[[ -r "$RT_DATA_DIR/robot_info.yaml" ]] ||
    fail "RT-robotconfiguratie ontbreekt: $RT_DATA_DIR/robot_info.yaml"

source "$ROS_SETUP"
source "$BASE_SETUP"
source "$RT_SETUP"

required_packages=(
    april_tabloo
    bumper
    drive_mux
    mecabot_integration
    people_follower_ros2
    pointcloud_to_laserscan
    tf2_ros
    turn_on_wheeltec_robot
    wheeltec_nav2
)

rt_owned_packages=(
    bumper
    drive_mux
    mecabot_integration
    people_follower_ros2
)

for package in "${required_packages[@]}"; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 ||
        fail "ROS-package '$package' is niet vindbaar. Controleer build/install en overlays."
done

rt_install_prefix="$(realpath -m "$RT_WORKSPACE/install")"
for package in "${rt_owned_packages[@]}"; do
    package_prefix="$(realpath -m "$(ros2 pkg prefix "$package")")"
    case "$package_prefix/" in
        "$rt_install_prefix/"*)
            echo "RT-package actief: $package ($package_prefix)"
            ;;
        *)
            fail "Package '$package' wordt geladen uit '$package_prefix' in plaats van uit '$rt_install_prefix'. Bouw de mecabot_rt-versie in de RT-workspace."
            ;;
    esac
done

echo "Actieve ROS 2 nodes vóór de RT-start:"
ros2 node list
if ros2 node list | grep -q .; then
    fail "Er draaien al ROS-nodes. Stop eerst de huidige stack handmatig; deze startup beëindigt geen processen."
fi

echo "ROS 2 daemon opnieuw starten..."
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2

run_in_terminal() {
    local title="$1"
    local logfile="$2"
    local command="$3"
    local logging_setup=":"

    if [[ "$ENABLE_LOGS" == "true" ]]; then
        logging_setup="exec > >(tee -a '$LOG_DIR/$logfile') 2>&1"
    fi

    gnome-terminal --title="$title" -- bash -lc "
        source '$ROS_SETUP' &&
        source '$BASE_SETUP' &&
        source '$RT_SETUP' &&
        cd '$RT_WORKSPACE' &&
        $logging_setup &&
        $command
        result=\$?
        echo 'Proces beëindigd (exit-code: '\$result').'
        exec bash
    "
}

wait_for_topic() {
    local topic="$1"
    local timeout="${2:-45}"
    local elapsed=0

    echo "Wachten op topic $topic (maximaal ${timeout}s)..."
    until ros2 topic list 2>/dev/null | grep -Fxq "$topic"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if (( elapsed >= timeout )); then
            fail "Topic $topic kwam niet beschikbaar. Controleer de betreffende terminal/log."
        fi
    done
    echo "Topic actief: $topic"
}

echo "Start RT-stack vanuit $RT_WORKSPACE"
echo "Logs: $LOG_DIR"
echo "RT runtime-data: $RT_DATA_DIR"

# Static TF and navigation are shared robot infrastructure.
run_in_terminal "RT Static TF" "static_tf.log" \
    "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom odom_combined"
sleep 2

run_in_terminal "RT Nav2" "nav2.log" \
    "ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py"
wait_for_topic "/cmd_vel"
sleep 5

# The RT mux reads the same RT-only data directory as the RT auto-recharger.
run_in_terminal "RT Drive MUX" "mux.log" \
    "while true; do ros2 run drive_mux drive_mux --ros-args --params-file '$RT_CONFIG'; result=\$?; [[ \$result -eq 130 ]] && break; echo 'MUX stopped; restarting in 2s'; sleep 2; done"
sleep 3

run_in_terminal "RT Bumper" "bumper.log" \
    "ros2 run bumper bumper"
sleep 3

run_in_terminal "RT Camera" "camera.log" \
    "while true; do ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py; result=\$?; [[ \$result -eq 130 ]] && break; echo 'Camera stopped; restarting in 3s'; sleep 3; done"
wait_for_topic "/camera/color/image_raw" 60
sleep 5

run_in_terminal "RT People Tracker" "people_tracker.log" \
    "ros2 run people_follower_ros2 visualtracker"
sleep 5

run_in_terminal "RT Pointcloud to Laserscan" "pointcloud.log" \
    "ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --remap cloud_in:=/camera/depth/points --remap scan:=/camera_scan -p target_frame:=camera_link -p min_height:=0.1 -p max_height:=1.0 -p angle_min:=-0.6 -p angle_max:=0.6 -p angle_increment:=0.015 -p range_min:=0.5 -p range_max:=3.5 -p use_inf:=true"
wait_for_topic "/camera_scan"
sleep 5

run_in_terminal "RT Keepout Filter" "keepout.log" \
    "ros2 launch wheeltec_nav2 keepout_filter.launch.py"
sleep 2

# Replaces auto_recharge_ros2. robot_position_reset is intentionally not
# started: the RT state manager requests AprilTag resets directly.
run_in_terminal "RT Auto Recharger" "auto_recharger.log" \
    "ros2 run mecabot_integration rt_auto_recharger.py --ros-args --params-file '$RT_CONFIG'"
sleep 5

# Replaces mecabot_bt, quiz_bt_node, drive_to_coord, and the separate
# AprilTag/state-reader processes with the single RT integration launch.
run_in_terminal "Mecabot RT Integration" "mecabot_integration.log" \
    "ros2 launch mecabot_integration mecabot_integration.launch.py"
sleep 10

echo
echo "Mecabot RT-start gestart."
echo "Actieve ROS 2 nodes:"
ros2 node list
echo
echo "Belangrijke topics:"
ros2 topic list | grep -E 'cmd_vel|scan|camera|BehaviorTreeNode|bump_status' || true
echo
echo "Niet gestart: legacy quiz_bt_node, mecabot_bt, drive_to_coord,"
echo "robot_position_reset en autocharge_batcheck. Hun RT-vervangers zitten"
echo "in de RT-integratie, of worden bewust niet gebruikt."
