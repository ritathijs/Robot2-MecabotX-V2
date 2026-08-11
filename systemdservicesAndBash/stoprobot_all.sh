#!/bin/bash

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
    echo "🛑 Alle oude robot processen worden afgesloten"

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

  
     # Alleen exit wanneer cleanup door een trap werd uitgevoerd
    if [ "$1" = "trap" ]; then
        exit 0
    fi
}
trap 'cleanup trap' SIGINT SIGTERM

cleanup "init"

sleep 10
echo "✅ Alle processen zijn afgesloten."
ros2 node list
ros2 topic list
echo "Alle nodes moeten gestopt zijn"
