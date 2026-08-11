#!/bin/bash

#This script is launched through the robotoo_startup.service

go_script=1

echo "robotoo_startup"

if [ $go_script -eq 1 ]; then
    echo "Robotoo awakes!"

    source /home/wheeltec/wheeltec_ros2/install/setup.bash
    ros2 launch wheeltec_roboto_launch roboto_launch.launch.py
    
else
    echo "Robotoo startup disabled (See file)"
fi

