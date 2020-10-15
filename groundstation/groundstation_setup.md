# Development and Operation PC Setup

For this tutorial, we will be using Ubuntu 20.04. Everything we do here is also generally supported on Ubuntu 18.04, but requires some manual steps to install the correct versions of dependencies, whereas Ubuntu 20.04 has everything we need readily available.

Furthermore, you can also achieve a working setup on OSX and Windows, as ROS2, Python, and OpenCV, and Qt are all available on these platforms, but such efforts will be considered out of scope and left as an exercise to the reader.

## Installing ROS2 & Dependencies

We will generally be following, with a few additional items:

<https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/>

```sh
# Install utilities needed for next steps
sudo apt-get update
sudo apt-get install curl gnupg2 lsb-release

# Add ROS2 repositories to your apt source lists
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt-get update

# Install ROS2 Foxy Fitzroy release (2020 LTS release), as well as additional supporting python packages
sudo apt-get install \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-rosdep \
    python3-vcstool \
    python3-wheel \
    ros-foxy-desktop

# Install development tools (some of these may have already been installed)
sudo apt-get install \
    build-essential \
    cmake \
    git

# Install Qt5 and PyQt5
sudo apt-get install \
    pyqt5-dev-tools \
    python3-pyqt5 \
    qtcreator \
    qttools5-dev-tools
```

Test to make sure ROS2 installation is functional by running the following commands and making sure ROS2 utilities run:

```sh
source /opt/ros/foxy/setup.bash
ros2 topic list
```

## Installing QGroundControl

Next, we will install QGroundControl, an application that serves as a groundstation for Pixhawk-based drones, and others. QGroundControl can be used out of the box to plan autonomous surveys and send them to your vehicle to execute. It also is required to set up a few configuration items for your vehicle.

Follow the instructions provided here:
<https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html#ubuntu>

## Downloading and building the OpenASV software

First, choose a location to clone the OpenASV repository, then navigate there in a terminal. From that location, you can run the following commands to download and build the software:

```bash
# Download the software
git clone https://github.com/oceansystems/asv

# Build the groundstation software
cd asv/groundstation
./build.sh
```

## Running the software

To run the software, simply run the run_*.sh script from the groundstation/scripts/ directory for the application you wish to run, i.e.

```bash
# In one terminal, run mockbot:
./asv/groundstation/scripts/run_mockbot.sh

# In another terminal, run asv_status:
./asv/groundstation/scripts/run_asv_status.sh
```
