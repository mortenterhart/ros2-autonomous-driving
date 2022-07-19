# Script to start nodes run on the turtlebot

# Build packages unless an option is specified
if [ "$1" != "--no-build" ]; then
    colcon build --packages-select camera_turtlebot
fi

# source package and nodes
. install/setup.bash

MODEL_NAME=burger
export TURTLEBOT3_MODEL="${MODEL_NAME}"

# create logs directory
mkdir -p log

ros2 launch turtlebot3_bringup robot.launch.py > log/log_bringup.txt 2>&1 &   # start bringup
ros2 run camera_turtlebot camera > log/log_camera.txt 2>&1 &               # start camera
ros2 run camera_turtlebot processing > log/log_processing.txt 2>&1 &       # start processing
