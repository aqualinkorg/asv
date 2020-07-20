# Groundstation Software Overview

## ROS2 Nodes
There are two ROS2 nodes included in the groundstation software:

`asv_status`: A GUI application that allows you to monitor the vehicle, enable/disable recording of ZED2 and telemetry data on the vehicle, and see the camera stream. Below is a screenshot of asv_status in action:

![asv_status](./images/asv_status.png)

The items under System Status indicate the state of requirements necessary to enable recording of data. The system must sucessfully:

1. Receive a GPS time source
2. Attain a GPS fix
3. Find a valid location to store data in (as configured in recorder.yaml)
4. Successfully initialize the camera

Once all status elements read "OK" and the `Recording Status` on the right says "READY", you will be able to enable/disable recording by clicking the `Enable Recording` checkbox.

`mockbot`: An application that mimics the software running on the vehicle that can be used with `asv_status` to see how it works. Run this alongside `asv_status` on your desktop to get a preview of what will happen when the vehicle software is running.

## Ground Control Station

Additionally, the user is expected to use `QGroundControl`, a Qt-based Drone ground control station application, to configure and plan missions for the vehicle. For more information visit:

<http://qgroundcontrol.com/>

## Using ROS2 tools

Similarly to on the vehicle, you can use the ROS2 command line utilities to interact with any of the ROS2 nodes:

```bash
# Source the ROS2 Underlay
source /opt/ros/foxy/setup.bash

# Some commands
ros2 topic list
ros2 topic echo "gps"