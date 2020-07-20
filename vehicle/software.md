# Vehicle (Nano) Software Overview

## Conan library packages

In: `vehicle/conan-packages`:

- `concurrentqueue`: threadsafe lockfree queue used by mavchannel
- `readerwriterqueue`: threadsafe lockfree queue used by mavchannel
- `serial`: serial port library used by mavchannel
- `mavchannel`: small library that combines serial, mavlink, and the threadsafe queues to provide a serial interface for devices that speak mavlink
- `mavlink2`: packaged version of the official mavlink repository. Mavlink is a communication protocol traditionally used within the Ardupilot/Ardusub space.

## ROS2 Nodes

There are three ROS2 nodes in the software package. Here is a short description of their functions:

- `asv_bridge`: This node bridges incoming Mavlink serial data from the Pixhawk to the ROS2 bus and vice-versa
- `recorder`: This node subscribes to telemetry data from the asv_bridge and captures frames from the ZED2 camera. When recording is enabled, it saves this data to disk.
  - This node also sends status info to the groundstation's `asv_status` node. The `asv_status` node lets you enable/disable recording.
- `status_light`: This node briefly flashes a status LED on the electronics enclosure each time a data sample is recorded by `recorder`.

There is one additional experimental ROS2 node called `pinger` which connects to the Blue Robotics Ping1D depth sounder module and publishes depth readings. This node is not thoroughly tested, nor is the data currently recorded by the `recorder` node, so individuals wishing to use this sensor should review and modify the code accordingly.

## Mavproxy

There is one additional process that runs at startup along with the ROS2 nodes, which is called `mavproxy`. `mavproxy` bridges the incoming mavlink serial communication coming from the Pixhawk to a UDP interface being used to communicate with QGroundControl on the groundstation computer. This allows QGroundControl to establish a telemetry and control link with the Pixhawk when the groundstation is connected to the vehicle's WiFi access point.

## Using ROS2 tools

You can use the ROS2 command line utilities to interact with the applications in this project:

```bash
# Source the ROS2 Underlay
source /opt/ros/eloquent/setup.bash

# Some commands
ros2 topic list
ros2 topic echo "gps"
```

See the official ROS2 docs for more info about what tools are available and how to use them.

## Interacting with the vehicle software manually

The software is automatically started on boot up by Systemd. Below are some commands you can use to manually interact with systemd:

```bash
# Start a service:
sudo systemctl start asv_bridge

# Stop a service:
sudo systemctl stop asv_bridge

# Restart a service
sudo systemctl restart asv_bridge

# Disable a service so it doesn't start at startup automatically:
sudo systemctl disable asv_bridge
```

Viewing real-time console logs of the applications can be done with journalctl:

```bash
# To view an entire log:
journalctl -u asv_bridge

# To trail a log in real-time
journalctl -u asv_bridge -f
```