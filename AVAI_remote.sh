# script to start nodes that should run on remote pc

# Build packages unless an option is specified
if [ "$1" != "--no-build" ]; then
    colcon build --packages-select perception localization
fi

# source package and nodes
. install/setup.bash

# export models path
export PYTHONPATH="$PWD/src/perception/perception:$PYTHONPATH"

# create logs directory
mkdir -p log/

ros2 run perception cone_detection > logs/log_cone_detection.txt 2>&1 &    # start cone detection
ros2 run localization localization > logs/log_localization.txt 2>&1 &      # start localization node
ros2 run localization plot_map > logs/log_map_detection.txt 2>&1 &         # start map
