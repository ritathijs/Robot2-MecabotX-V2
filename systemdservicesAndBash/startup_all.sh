#!/bin/bash

# Instellingen
ROS_DISTRO="humble" # Pas dit aan naar jouw ROS 2 distributie
WORKSPACE_DIR=$(pwd) # Gaat ervan uit dat je het script runt vanuit je workspace

# Commando om ROS 2 en de lokale workspace te sourcen in elk nieuw venster
SETUP_CMD="source /opt/ros/$ROS_DISTRO/setup.bash && if [ -f $WORKSPACE_DIR/install/setup.bash ]; then source $WORKSPACE_DIR/install/setup.bash; fi"

# ==========================================
# 1. DE OPRUIMFUNCTIE (TRAP)
# ==========================================

cleanup() {
    echo ""
    echo "🛑 Ctrl+C gedetecteerd! Bezig met veilig afsluiten van alle ROS 2 processen..."
    
    # Stuur een zacht stopsignaal (SIGINT) naar alle actieve ROS 2 processen
    pkill -INT -f "ros2 launch"
    pkill -INT -f "ros2 run"
    
    # Wacht 2 seconden om de nodes de kans te geven hun geheugen op te ruimen
    sleep 2
    
    # Forceer alles wat weigert te sluiten alsnog dicht (optioneel, maar effectief)
    killall -9 astra_camera_node visualtracker pointcloud_to_laserscan_node 2>/dev/null
    
    echo "✅ Alle vensters en processen zijn succesvol afgesloten."
    exit 0
}

# Koppel de cleanup functie aan het indrukken van Ctrl+C (SIGINT)
trap cleanup SIGINT

# ==========================================
# 2. DE LAUNCH COMMANDO'S
# ==========================================

echo "Starten van alle ROS 2 processen in aparte vensters..."

ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2



# 0 - Static TF odom -> odom_combined
gnome-terminal --title="0. Static TF odom_combined" -- bash -c "$SETUP_CMD && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom odom_combined; exec bash"

sleep 2

# 1 - Navigatie (T=0)
gnome-terminal --title="1. Navigatie" -- bash -c "$SETUP_CMD && ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py; exec bash"

sleep 5
# 1 - Eigen mux (T=0)
gnome-terminal --title="1,5. MUX" -- bash -c "$SETUP_CMD && ros2 run drive_mux drive_mux; exec bash"

sleep 5
# 2 - Quiz BT Node (T=5)
gnome-terminal --title="2. Quiz BT Node" -- bash -c "$SETUP_CMD && ros2 run quiz_bt_node quiz_bt_node; exec bash"

sleep 5
# 3 - Drive to coord (T=10)
gnome-terminal --title="3. Drive to Coord" -- bash -c "$SETUP_CMD && ros2 run drive_to_coord drive_to_coord --ros-args --log-level error; exec bash"

sleep 3
# 4 - Bumper (T=13)
gnome-terminal --title="4. Bumper" -- bash -c "$SETUP_CMD && ros2 run bumper bumper; exec bash"

sleep 5
# 5 - Camera (T=14)
gnome-terminal --title="5. Camera" -- bash -c "$SETUP_CMD && ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py; exec bash"

sleep 10
gnome-terminal --title="5. Visual Follower" -- bash -c "$SETUP_CMD && ros2 run people_follower_ros2 visualtracker --ros-args --log-level error; exec bash"

sleep 10
# 6 - Pointcloud to laserscan (T=16)
#gnome-terminal --title="6. Pointcloud to Laserscan" -- bash -c "$SETUP_CMD && ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --log-level error --remap cloud_in:=/camera/depth/points --remap scan:=/camera_scan -p target_frame:=camera_link -p min_height:=0.1 -p max_height:=1.5 -p angle_min:=-1.5708 -p angle_max:=1.5708 -p range_min:=0.5 -p range_max:=5.0 -p use_inf:=true; exec bash"

gnome-terminal --title="6. Pointcloud to Laserscan" -- bash -c "$SETUP_CMD && ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --log-level error --remap cloud_in:=/camera/depth/points --remap scan:=/camera_scan -p target_frame:=camera_link -p min_height:=0.1 -p max_height:=1.0 -p angle_min:=-0.6 -p angle_max:=0.6 -p angle_increment:=0.015 -p range_min:=0.5 -p range_max:=3.5 -p use_inf:=true; exec bash"


sleep 10
# 7 - Keepoutfilter (T=18)
gnome-terminal --title="7. Keepout Filter" -- bash -c "$SETUP_CMD && ros2 launch wheeltec_nav2 keepout_filter.launch.py; exec bash"

#sleep 10
# 8 - Visual tracker (T=19)
#gnome-terminal --title="8. Visual Tracker" -- bash -c "$SETUP_CMD && ros2 run people_follower_ros2 visualtracker --ros-args --log-level error; exec bash"

sleep 1
# 9 - Autocharge (T=20)
gnome-terminal --title="9. Auto Recharge" -- bash -c "$SETUP_CMD && ros2 run auto_recharge_ros2 auto_recharge; exec bash"


sleep 30

gnome-terminal --title="12. Apriltag" -- bash -c "$SETUP_CMD && ros2 run april_tabloo april_tabloo; exec bash"

sleep 10
# 10 - Robot position reset (T=35)
gnome-terminal --title="10. Robot Pos Reset" -- bash -c "$SETUP_CMD && ros2 run robot_position_reset robot_position_reset; exec bash"


sleep 10
# 11 - Behavior Tree (T=50)
gnome-terminal --title="11. Behavior Tree" -- bash -c "$SETUP_CMD && ros2 run mecabot_bt bt_node; exec bash"

sleep 30
# 12 - Autocharge Bat Check
gnome-terminal --title="12. Autocharge Bat Check" -- bash -c "$SETUP_CMD && ros2 run autocharge_batcheck autocharge_batcheck; exec bash"

echo "Alle processen zijn succesvol geïnitialiseerd!"
