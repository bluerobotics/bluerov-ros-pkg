# Usage

Once you have completed [Setup](Setup.md), this guide will show you have to launch the different ROS nodes.

## BlueROV Core Services

Launching the BlueROV core services can be as simple as:

```bash
ssh viki@bluerov -p viki -c 'roslaunch bluerov core.launch'
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

## Interrogating ROS

ROS includes a variety of tools to interrogate a running system. This is a brief overview on a few of them.

## rostopic

The [rostopic](http://wiki.ros.org/rostopic?distro=indigo) command will list and display ROS network messages from the command line. With the BlueROV core services running, try the follow commands:

```bash
rostopic list
rostopic hz thruster
rostopic echo thruster
```

## rqt_graph

The [rqt_graph](http://wiki.ros.org/rqt_graph?distro=indigo) command displays all active nodes and shows the message subscribe/publish relationships between them. With all of the BlueROV nodes turned on, you should see a graph structure similar to this:

TODO: graph image

## rqt_plot

The [rqt_plot](http://wiki.ros.org/rqt_plot) command is a quick and dirty way to plot ROS message data over time. Try the follow command with the `pilot` node running:

```bash
rqt_plot /thruster/commands/data[0] /thruster/commands/data[1] /thruster/commands/data[2] /thruster/commands/data[3] /thruster/commands/data[4] /thruster/commands/data[5]
```

## rviz

TODO
