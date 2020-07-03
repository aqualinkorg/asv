#!/bin/bash
set -e

mkdir -p src/.build

# Install conan deps
conan install --build missing -if src/.build .

# Build colcon workspace
if [ -d "/opt/ros/foxy" ]; then
    source /opt/ros/foxy/setup.bash
elif [ -d "/opt/ros/eloquent" ]; then
    source /opt/ros/eloquent/setup.bash
else
    echo "ROS installation not found"
    exit 1
fi

colcon build --event-handlers --cmake-args "-DCMAKE_BUILD_TYPE=Release"