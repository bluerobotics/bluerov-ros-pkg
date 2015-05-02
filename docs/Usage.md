# Usage

Once you have completed [Setup](Setup.md), this guide will show you how to launch the different ROS nodes.

## Safety

ROV thruster can be dangerous and are not toys. If there is a safety concern at any point while using the BlueROV, immediately disable the thrusters. Here are three ways to do this, starting with the most convenient method:

1. Hit the red "B" button on the Xbox controller (if connected and configured as such)
1. Send the following command: `rostopic pub thruster_enable std_msgs/Bool false --once`
1. Open the BlueROV and unplug the battery

## BlueROV Core Services

Launching the BlueROV core services can be as simple as:

```bash
ssh viki@bluerov -c 'roslaunch bluerov core.launch'
```

If you're in the middle of development, however, you'll likely want to turn on services with a bit more granularity. For starters, run the follow three commands in three separate terminal windows:

```bash
# window 1
roscore

# window 2
roslaunch bluerov serial.launch

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
