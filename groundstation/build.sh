#!/bin/bash
set -e

# Build colcon workspace
source /opt/ros/foxy/setup.bash
colcon build --event-handlers