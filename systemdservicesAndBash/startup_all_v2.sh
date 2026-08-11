#!/bin/bash

ENABLE_LOGS=false   # logs uit
# ENABLE_LOGS=true  # logs aan

log_cmd() {
    local logfile=$1

    if [ "$ENABLE_LOGS" = true ]; then
        echo "2>&1 | tee $LOGDIR/$logfile"
    else
        echo "2>&1"
    fi
}


###############################################
# MECABOT X – PROFESSIONELE STARTUP MANAGER
# Robuust, modulair, met automatische herstart
###############################################

ROS_DISTRO="humble"
WORKSPACE_DIR="/home/wheeltec/wheeltec_ros2"
LOGDIR="$WORKSPACE_DIR/logs"
mkdir -p $LOGDIR
export LOGDIR

SETUP_CMD="source /opt/ros/$ROS_DISTRO/setup.bash && \
           if [ -f $WORKSPACE_DIR/install/setup.bash ]; then \
               source $WORKSPACE_DIR/install/setup.bash; \
           fi"

###############################################
# 1. CLEANUP
###############################################

cleanup() {
    echo "🛑 Alle oude processen worden afgesloten en daarna herstart de robot..."

    pkill -INT -f "ros2"
    pkill -INT -f "ros2 run"
    pkill -9 -f "static_transform_publisher"
    pkill -9 -f "ros2 launch"
    sleep 2

    pkill -9 -f drive_mux
    pkill -9 -f visualtracker
    pkill -9 -f auto_recharge
    pkill -9 -f mecabot_bt
    pkill -9 -f wheeltec_nav2
    pkill -9 -f bumper
    pkill -9 -f drive_to_coord
    pkill -9 -f robot_position_reset
    pkill -9 -f april_tabloo
    pkill -9 -f keepout
    pkill -9 -f quiz_bt_node
    pkill -9 -f autocharge_batcheck
    pkill -9 -f people_follower
    pkill -9 -f nav2_container
    pkill -9 -f map_server
    pkill -9 -f amcl
    pkill -9 -f controller_server
    pkill -9 -f planner_server
    pkill -9 -f smoother_server
    pkill -9 -f lifecycle_manager
    
    # Camera driver (Orbbec Astra)
    pkill -9 -f astra_camera_node
    pkill -9 -f astra_camera

    # Pointcloud converter
    pkill -9 -f pointcloud_to_laserscan_node
    pkill -9 -f pointcloud_to_laserscan

    echo "✅ Alle processen zijn afgesloten."
     # Alleen exit wanneer cleanup door een trap werd uitgevoerd
    if [ "$1" = "trap" ]; then
        exit 0
    fi
}
trap 'cleanup trap' SIGINT SIGTERM

cleanup "init"

sleep 10
ros2 node list
ros2 topic list
echo "Alle nodes moeten gestopt zijn"
#read -p "Druk op Enter om verder te gaan..."


###############################################
# 2. HEALTH CHECKS
###############################################

echo "🔍 Uitvoeren van hardware checks..."

if [ ! -e /dev/wheeltec_controller ]; then
    echo "❌ Controller niet gevonden: /dev/wheeltec_controller"
    exit 1
fi

if ! lsusb | grep -qi "Orbbec"; then
    echo "❌ Orbbec camera niet gevonden!"
    exit 1
fi

echo "✅ Hardware OK"
#read -p "Druk op Enter om verder te gaan..."


###############################################
# 2B. ROS2 DAEMON RESET
###############################################

echo "🔄 ROS2 daemon resetten..."
ros2 daemon stop
sleep 5
ros2 daemon start
sleep 5
echo "✔ ROS2 daemon opnieuw gestart"
#read -p "Druk op Enter om verder te gaan..."


###############################################
# 3. READYNESS CHECK FUNCTIE
###############################################
wait_for_topic() {
    local topic=$1
    local timeout=30
    local count=0
    
    echo "⏳ Wachten op topic: $topic (max ${timeout}s)..."
    
    until ros2 topic list | grep -q "$topic"; do
        sleep 1
        count=$((count + 1))
        
        if [ $count -ge $timeout ]; then
            echo "❌ TIMEOUT: Topic $topic is niet actief geworden binnen $timeout seconden!"
            echo "🛑 Robot start afgebroken vanwege ontbrekende hardware/data."
            exit 1
        fi
    done
    
    echo "✔ Topic actief: $topic"
}

###############################################
# 4. MODULAIRE STARTFUNCTIES
###############################################

start_static_tf() {
    gnome-terminal --title="TF: odom → odom_combined" -- bash -c "
        $SETUP_CMD && \
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom odom_combined \
        2>&1 | tee $LOGDIR/static_tf.log
        "
}

start_nav2() {
    gnome-terminal --title="NAV2" -- bash -c "
        $SETUP_CMD && \
        ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py \
        2>&1 | tee $LOGDIR/nav2.log
        "
}

start_mux() {
    gnome-terminal --title="Drive MUX" -- bash -c "
        $SETUP_CMD && \
        while true; do
            ros2 run drive_mux drive_mux \
            2>&1 | tee $LOGDIR/mux.log
            echo '⚠️ MUX crashed — restart in 2s'
            sleep 2
        done
        "
}

start_quiz_bt() {
    gnome-terminal --title="Quiz BT" -- bash -c "
        $SETUP_CMD && \
        ros2 run quiz_bt_node quiz_bt_node \
        2>&1 | tee $LOGDIR/quiz_bt.log
        "
}

start_drive_to_coord() {
    gnome-terminal --title="Drive to Coord" -- bash -c "
        $SETUP_CMD && \

        echo '🚗 drive_to_coord gestart (slimme autorestart actief)'

        while true; do
            ros2 run drive_to_coord drive_to_coord --ros-args --log-level error
            EXIT_CODE=\$?

            # Exit-code analyse
            if [ \$EXIT_CODE -eq 0 ]; then
                echo '✔ drive_to_coord stopte normaal — geen autorestart'
                break
            elif [ \$EXIT_CODE -eq 130 ]; then
                echo '🛑 drive_to_coord gestopt door SIGINT — geen autorestart'
                break
            else
                echo \"⚠️ drive_to_coord crashte met exit-code \$EXIT_CODE — herstart in 2s\"
                sleep 2
            fi
        done

        "
}


start_bumper() {
    gnome-terminal --title="Bumper" -- bash -c "
        $SETUP_CMD && \
        ros2 run bumper bumper \
        2>&1 | tee $LOGDIR/bumper.log
        "
}

start_camera() {
    gnome-terminal --title="Camera" -- bash -c "
        $SETUP_CMD && \
        ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py \
        2>&1 | tee $LOGDIR/camera.log
        "
}

start_visualtracker() {
    gnome-terminal --title="Visual Tracker" -- bash -c "
        $SETUP_CMD && \
        ros2 run people_follower_ros2 visualtracker --ros-args --log-level error \
        2>&1 | tee $LOGDIR/visualtracker.log
        "
}

start_pointcloud() {
    gnome-terminal --title="Pointcloud → Laserscan" -- bash -c "
        $SETUP_CMD && \
        ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
            --ros-args --log-level error \
            --remap cloud_in:=/camera/depth/points \
            --remap scan:=/camera_scan \
            -p target_frame:=camera_link \
            -p min_height:=0.1 \
            -p max_height:=1.0 \
            -p angle_min:=-0.6 \
            -p angle_max:=0.6 \
            -p angle_increment:=0.015 \
            -p range_min:=0.5 \
            -p range_max:=3.5 \
            -p use_inf:=true \
        2>&1 | tee $LOGDIR/pointcloud.log
        "
}

start_keepout() {
    gnome-terminal --title="Keepout Filter" -- bash -c "
        $SETUP_CMD && \
        ros2 launch wheeltec_nav2 keepout_filter.launch.py \
        2>&1 | tee $LOGDIR/keepout.log
        "
}

start_autocharge() {
    gnome-terminal --title="Auto Recharge" -- bash -c "
        $SETUP_CMD && \
        ros2 run auto_recharge_ros2 auto_recharge \
        2>&1 | tee $LOGDIR/autocharge.log
        "
}

start_robot_pos_reset() {
    gnome-terminal --title="Robot Pos Reset" -- bash -c "
        $SETUP_CMD && \
        ros2 run robot_position_reset robot_position_reset \
        2>&1 | tee $LOGDIR/robot_pos_reset.log
        "
}

start_apriltag() {
    gnome-terminal --title="Apriltag" -- bash -c "
        $SETUP_CMD && \
        ros2 run april_tabloo april_tabloo \
        2>&1 | tee $LOGDIR/apriltag.log
        "
}


start_bt() {
    gnome-terminal --title="Behavior Tree" -- bash -c "
        $SETUP_CMD && \
        ros2 run mecabot_bt bt_node \
        2>&1 | tee $LOGDIR/bt.log
        "
}

start_batcheck() {
    gnome-terminal --title="Battery Check" -- bash -c "
        $SETUP_CMD && \
        ros2 run autocharge_batcheck autocharge_batcheck \
        2>&1 | tee $LOGDIR/batcheck.log
       "
}

###############################################
# 5. STARTVOLGORDE
###############################################

start_static_tf
sleep 2

start_camera
wait_for_topic "/camera/color/image_raw"

start_nav2
wait_for_topic "/cmd_vel"
sleep 5

start_mux
sleep 5
start_quiz_bt
sleep 5

start_drive_to_coord   # slimme autorestart-loop blijft zoals besproken
sleep 3
start_bumper
sleep 5

start_visualtracker
sleep 10

start_pointcloud
wait_for_topic "/camera_scan"
sleep 10

start_keepout
sleep 10

start_autocharge
sleep 30

start_robot_pos_reset
sleep 5

start_apriltag
sleep 10

start_bt
sleep 30

start_batcheck




###############################################
# 6. STATUS DASHBOARD
###############################################
echo ""
echo "📡 Actieve ROS2 nodes:"
ros2 node list

echo ""
echo "📡 Belangrijke topics:"
ros2 topic list | grep -E "cmd_vel|scan|camera"

echo ""
echo "🚀 MECABOT X volledig opgestart!"

