#!/bin/bash
set -e

source /opt/openasv/vehicle/install/setup.bash
ros2 run recorder recorder --ros-args --params-file /opt/openasv/vehicle/src/recorder/config/recorder.yaml