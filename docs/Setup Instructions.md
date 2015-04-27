# Setup Instructions

The BlueROV software stack is built on [ROS](http://www.ros.org/), the Robotic Operation System. We are building off of the current version of ROS, codenamed Indigo Igloo.

## ROS Installation

The BlueROV setup includes one main computer on the ROV itself and a secondary, tele-operations computer on the ground. The following instructions will help you get a vanilla version of ROS Indigo Igloo installed on both

### Workstation

For first time ROS users, we highly recommend installing the Ubuntu 14.04 virtual machine images with ROS Indigo Igloo preinstalled by Nootrix. Choose the 64-bit version if your machine has the resources to support it, otherwise use the 32-bit version. To install using this method, please do the following:

1. visit the [Nootrix blog post](http://nootrix.com/2014/09/ros-indigo-virtual-machine/) on the subject
1. download and install [VirtualBox](https://www.virtualbox.org/) and the VirtualBox Extension Pack for your host machine
1. download one of the ROS [virtual machine images from Nootrix](http://nootrix.com/downloads/#RosVM)
1. open VirtualBox, choose File > Import Appliance and then select the virtual machine image that you downloaded

### RaspberryPi

The ROS community maintains excellent instructions on getting [ROS to run on a RaspberryPi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi). Follow those and you should be good to go.

1. Download Raspbian OS from [raspberrypi.org/](https://www.raspberrypi.org/downloads/)
1. Install OS onto SD card using [OS-specific instructions](https://www.raspberrypi.org/documentation/installation/installing-images/README.md)
1. Continue with the rest of the ROS install guide

### BlueROV ROS Package

The following steps assume that you are using the Nootrix image or have otherwise followed ROS' guidelines [configuring your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Check out the BlueROV repository:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/bluerobotics/BlueROV.git bluerov
```

You should then be able to build your catkin workspace:

```bash
cd ~/catkin_ws
catkin_make
```

You'll probably want to add the resulting setup file to your `~/.bashrc` script.

```bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

### APM ROS Installation

We use TODO to compile and program the APM from the command line.

TODO
