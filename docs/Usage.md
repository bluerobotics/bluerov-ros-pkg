# Usage

Once you have completed [Setup](Setup.md), this guide will show you how to launch the different ROS nodes.

## Safety

ROV thruster can be dangerous and are not toys. If there is a safety concern at any point while using the BlueROV, immediately disable the thrusters. Here are three ways to do this, starting with the most convenient method:

1. Hit the red "B" button on the Xbox controller (if connected and configured as such)
1. Send the following command: `rostopic pub hazard_enable std_msgs/Bool false --once`
1. Open the BlueROV and unplug the battery

## Thruster Configuration

The code base currently supports BlueRobotics T100 thrusters. The thrusters are labeled as:

Index | Code | Description
--- | --- | ---
0 | VL | Vertical Left
1 | VB | Vertical Back
2 | VR | Vertical Right
3 | FL | Forward Left
4 | LAT | Lateral
5 | FR | Forward Right


ADD TO DOCUMENTATION
---
screen shots
how to use dyn_config
how to enable the thrusters, how to wire the thrusters correctly
how to flip gains



![BlueROV Schematic](BlueROV%20Schematic.jpg)

Note: The xml version of embedded diagrams can be modified with [https://www.draw.io/](https://www.draw.io/).

## ROS Node Overview

#### roscore

todo

#### raspicam_node

todo

#### teleop_xbox

todo

#### 

todo

####

## ROS Node Topology

TODO: which nodes should be run from which computer?

Node | Location
--- | ---
roscore | ROV or Workstation (ROV preferred)
mavlink | ROV
raspi cam | ROV
pilot, simple_pilot | ROV or Workstation (ROV preferred)
teleop_xbox | Workstation

## BlueROV Core Services

Launching the BlueROV core services is as simple as:

```bash
ssh ubuntu@bluerov -c 'roslaunch bluerov core.launch'
```

If you're in the middle of development, however, you'll likely want to turn on services with a bit

For more granularity, you can run the nodes individually in separate terminal windows:

```bash
# window 1
roscore

# window 2
roslaunch bluerov apm .launch

# window 3
roslaunch bluerov pilot.launch
```

## Ground Station Tele-Operation

TODO: set remote IP, launch xbox_teleop

```bash
alias usebluerov='export ROS_MASTER_URI=http://bluerov:11311'
```

## Interrogating ROS

ROS includes a variety of tools to interrogate a running system. This is a brief overview on a few of them.

### rostopic

The [rostopic](http://wiki.ros.org/rostopic?distro=indigo) command will list and display ROS network messages from the command line. With the BlueROV core services running, try the follow commands:

```bash
rostopic list
rostopic hz thruster
rostopic echo thruster
```

### rqt_graph

The [rqt_graph](http://wiki.ros.org/rqt_graph?distro=indigo) command displays all active nodes and shows the message subscribe/publish relationships between them. With all of the BlueROV nodes turned on, you should see a graph structure similar to this:

![rqt_graph](bluerov-rqt-graph.jpg)

### rqt_plot

The [rqt_plot](http://wiki.ros.org/rqt_plot) command is a quick and dirty way to plot ROS message data over time. Try the one of the following commands with the `pilot` and `teleop_xbox` nodes running:

```bash
rqt_plot /cmd_vel/linear/x /cmd_vel/linear/y /cmd_vel/linear/z /cmd_vel/angular/x /cmd_vel/angular/y /cmd_vel/angular/z
rqt_plot /thruster/commands/data[0] /thruster/commands/data[1] /thruster/commands/data[2] /thruster/commands/data[3] /thruster/commands/data[4] /thruster/commands/data[5]
```

![rqt_plot](bluerov-rqt-plot.jpg)

### rviz

TODO
TODO: rviz image

## Dynamic Reconfiguration

The `pilot` and `teleop_xbox` nodes can be configured while running using the command:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

![rqt_reconfigure](bluerov-rqt-reconfigure.jpg)

Note that configuration changes during runtime do not persist. Make sure to update the appropriate file in the `config/` directory to persist changes.






ProTip: If you are new to ROS, check out the [ROS Cheat sheet](http://www.clearpathrobotics.com/wp-content/uploads/2014/01/ROS-Cheat-Sheet-v1.01.pdf) from Clearpath Robotics.

## Saving Data to ROS Bags

## Exporting Video from ROS Bags

Old process:

```bash
# terminal window 1
roscore
rosbag play -d 2 something.bag

# terminal window 2
mkdir export
cd export
rosrun image_view extract_images image:=/camera/image _image_transport:=compressed

# in terminal window 2 after export has finished
cd ..
mencoder "mf://export/*.jpg" -mf type=jpg:fps=30 -o output.mpg -speed 1 -ofps 30 -ovc lavc -lavcopts vcodec=mpeg2video:vbitrate=2500 -oac copy -of mpeg
 1341  mkdir bag1
```