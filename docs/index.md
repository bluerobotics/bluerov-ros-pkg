# BlueROV

The BlueROV is an underwater remotely operated vehicle (ROV) propelled by six [BlueRobotics](http://www.bluerobotics.com/) [T100s](https://www.bluerobotics.com/store/thrusters/t100-thruster/). It uses an APM and RaspberryPi internally and can be controlled from the surface. ROS is used as a software backbone.

TODO: assembled BlueROV picture

## Overview

TODO: what the sub can do...

## Bill of Materials

TODO

## Schematic

![BlueROV Schematic](BlueROV%20Schematic.jpg)

Note: The xml version of embedded diagrams can be modified with [https://www.draw.io/](https://www.draw.io/).

## Thrusters

The code base currently supports BlueRobotics T100 thrusters. The thrusters are labeled as:

Index | Code | Description
--- | --- | ---
0 | VL | Vertical Left
1 | VB | Vertical Back
2 | VR | Vertical Right
3 | FL | Forward Left
4 | LAT | Lateral
5 | FR | Forward Right
