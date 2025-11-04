# Installations and Initial Setup Guide

The following steps are for a system or PC with Ubuntu 20.04 LTS Operating System installed.

## Make sure Ubuntu is up-to-date
```bash
sudo apt update
sudo apt upgrade
```

## ROS Noetic Installation
Follow the installation steps from the official [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)

## MAVROS Installation
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```
For ease of use on a desktop computer, please also install RQT
```bash
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins
```

## Create Catkin Workspace
To create and build a catkin workspace, follow the following commands:
```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Add the following lines at the end of ~/.bashrc file:
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## Install ArduPilot and MAVProxy
```bash
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

Install dependencies:
```bash
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload profile:
```bash
. ~/.profile
```

Checkout latest copter build:
```bash
git checkout Copter-4.4.4
git submodule update --init --recursive
```

## Install Gazebo Simulator
Setup computer to accept softwaare from osrfoundation.org:
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt update
```

Install Gazebo11:
```bash
sudo apt-get install gazebo11 libgazebo11-dev
```

Update and Upgrade:
```bash
sudo apt-get update

sudo apt-get upgrade
```
## Install Gazebo Plugin for ArduPilot
```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

Build and install plugin:
```bash
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

```bash
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```

Set paths for models in bashrc:
```bash
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc

. ~/.bashrc
```

## Test ArduPilot and Gazebo Installation
In one terminal, run Gazebo:
```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```
**Note:** Loading gazebo for the first time might take a long time especially when the internet is connected. This is because Gazebo installs the default model files from the OSRF website when it is launched for the first time.

In second terminal, run ArduPilot SITL:
```bash
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
Note: if you get an error while running this command, simply reboot your computer and try again. 

Note: After all the installation steps are complete, it is recommended to reboot the PC.

Basic commands to test if the drone is working:
mode GUIDED
arm throttle
takeoff 1
mode LAND


## References:
1. [Intelligent Quads Tutorials](https://github.com/Intelligent-Quads/iq_tutorials)
2. [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
3. [ArduPilot - Installing ROS](http://wiki.ros.org/ROS/Tutorials)
