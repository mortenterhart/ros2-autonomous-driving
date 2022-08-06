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
mkdir -p log

ros2 run perception cone_detection > log/log_cone_detection.txt 2>&1 &    # start cone detection
perception_pid="$(($! + 2))"

ros2 run localization localization > log/log_localization.txt 2>&1 &      # start localization node
localization_pid="$(($! + 2))"

ros2 run localization mapping > log/log_map_detection.txt 2>&1 &          # start map
mapping_pid="$(($! + 2))"

trap "kill -9 $perception_pid $localization_pid $mapping_pid" SIGINT SIGTERM

wait "$perception_pid" "$localization_pid" "$mapping_pid"
