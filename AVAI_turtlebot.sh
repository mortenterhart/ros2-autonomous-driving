# Script to start nodes run on the turtlebot

# source package and nodes
. install/setup.bash

MODEL_NAME=burger
export TURTLEBOT3_MODEL="${MODEL_NAME}"

# create logs directory
mkdir -p logs

ros2 launch turtlebot3_bringup robot.launch.py > logs/log_bringup.txt 2>&1 &   # start bringup
ros2 run camera_turtlebot camera > logs/log_camera.txt 2>&1 &               # start camera
ros2 run camera_turtlebot processing > logs/log_processing.txt 2>&1 &       # start processing



