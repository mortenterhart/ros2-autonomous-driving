#!/bin/bash

export PYTHONPATH="$PWD/src/perception/perception:$PYTHONPATH"

source install/setup.bash

ros2 run perception cone_detection
