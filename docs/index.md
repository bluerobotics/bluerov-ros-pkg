# BlueROV

A ROS package for the BlueROV and BlueROV compatible vehicles.

## Overview

The BlueROV is an underwater remotely operated vehicle (ROV) propelled by six [BlueRobotics](http://www.bluerobotics.com/) [T100s](https://www.bluerobotics.com/store/thrusters/t100-thruster/). The primary control code is built on top of [ROS](http://www.ros.org/) and runs on an embedded Linux computer (a [RaspberryPi 2 B](https://www.raspberrypi.org/products/raspberry-pi-2-model-b/) for the BlueROV) inside of the ROV. A [MAVLink](https://pixhawk.ethz.ch/MAVLink/) compatible controller (an [APM 2.6](https://store.3drobotics.com/products/apm-2-6-kit-1) for the BlueROV) is used as a sensor platform (IMU with DCM, internal pressure, battery stats) and as an interface to multiple thrusters. A ROS node proxies messages between the MAVLink compatible controller and the embedded Linux computer over serial. A camera (a [RaspberrPi Camera](https://www.raspberrypi.org/products/camera-module/) in this case) is also connected to the embedded Linux computer.

![BlueROV](bluerov_r0.png)

With this hardware in place, the BlueROV and BlueROV compatible vehicles provide reliable tele-operation from a surface computer. The platform can be extended with additional sensors and actuators through both the embedded Linux computer and the MAVLink compatible controller.

## Schematic

![BlueROV Schematic](BlueROV%20Schematic.jpg)

Note: The xml version of embedded diagrams can be modified with [https://www.draw.io/](https://www.draw.io/).

## Bill of Materials

TODO

## Configuration

The code base currently supports BlueRobotics T100 thrusters. The thrusters are labeled as:

Index | Code | Description
--- | --- | ---
0 | VL | Vertical Left
1 | VB | Vertical Back
2 | VR | Vertical Right
3 | FL | Forward Left
4 | LAT | Lateral
5 | FR | Forward Right
