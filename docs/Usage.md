# Usage

Once you have completed [Setup](Setup.md), this guide will show you have to launch the different ROS nodes.

## BlueROV Core Services

Launching the BlueROV core services can be as simple as:

```bash
ssh viki@bluerov -p wiki -c 'roslaunch core'
```

If you're in the middle of development, however, you'll likely want to turn on services with a bit more granularity. For starters, run the follow three commands in three separate terminal windows:

```bash
# window 1
roscore

# window 2
roslaunch serial

# window 3
roslaunch pilot
```

TODO

## Ground Station Tele-Operation

TODO: set remote IP, launch xbox_teleop

## Interrogating Active Nodes

TODO: rosgraph, etc

If everything is working correctly, you should see a graph structure similar to this:

TODO
