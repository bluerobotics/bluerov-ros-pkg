# bluerov-ros-pkg

A set of ROS package for the [BlueROV2](http://www.bluerobotics.com/store/rov/bluerov2/) and [ArduSub](https://www.ardusub.com/) compatible vehicles.

## Setup
* Flash a PixHawk with the latest stable version of [ArduSub](https://www.ardusub.com/software/ardusub-firmware.html)
* Flash the companion Pi with Bluerobotic's [image](https://www.ardusub.com/getting-started/installation.html#ardusub) of the pi
* Run the script in bluerov-ros-pkg/bluerov_apps/extra/ubuntu_setup.sh on your surface machine with Ubuntu.
* Set the IP of the surface machine to ```192.168.2.1```
* Connect the teather to the surface machine and power the rover
* Run the following to connect to the PixHawk on the surface machine
```
roslaunch rover apm.launch
```
* Run one of the following to control the rover with a xbox controller on the surface machine with the controller connected
```
roslaunch rover teleop_joy.launch
```
* Install and run QGroundControl to edit pixhawk parameters, check arming status, monitor your vehicle, etc on your surface machine

The documentation that follows in this file is specifically for those interested in contributing.

## Installation

```bash
#Install ROS Kinetic

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y build-essential git
sudo apt-get install -y ros-kinetic-desktop-full ros-kinetic-mavros* ros-kinetic-joy

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd catkin_ws/src
git clone https://github.com/kdkalvik/bluerov-ros-pkg.git
cd ..
catkin_make
```
