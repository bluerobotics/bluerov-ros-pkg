# bluerov-ros-pkg

A ROS package for the BlueROV and BlueROV compatible vehicles.

## Overview

This package provides the tools needed to remotely pilot an underwater robotic vessel. The package includes software to view streaming video and interface with an xbox controller from a surface computer as well as software to control thrusters and cameras on the ROV computer.

## Hardware

This package is designed to work with a variety of ROV platforms, but is focused on the BlueROV (store, [docs](http://docs.bluerobotics.com/bluerov/)) from [BlueRobotics](http://www.bluerobotics.com/). Other platforms can utilize this library as well as long as they have the following items:

* A surface computer running Ubuntu (through a virtual machine is OK)
* An input device for manual control
    - We are using a [wireless Xbox 360 controller]()
* An ROV computer running Ubuntu
    - We are using a [RaspberryPi 2 B](https://www.raspberrypi.org/products/raspberry-pi-2-model-b/)
* A camera attached to the ROV computer
    - We are using a [RaspberryPi Camera](https://www.raspberrypi.org/products/camera-module/), but a standard USB webcam should work too
* A method of controlling the thrusters
    - To control thrusters over PWM, you'll want something like an [APM 2.6](https://store.3drobotics.com/products/apm-2-6-kit-1) (which also provides an IMU) or a servo driver PCB like [this one](http://www.adafruit.com/product/815)
    - To control thrusters over I2C, you simply need to have an I2C output on your ROV computer; the Raspberry Pi has this

## BlueRobotics BlueROV Configuration

![BlueROV](bluerov_r0.png)

The primary control code is built on top of [ROS](http://www.ros.org/) and runs on an embedded Linux computer (a [RaspberryPi 2 B](https://www.raspberrypi.org/products/raspberry-pi-2-model-b/) for the BlueROV) inside of the ROV. A [MAVLink](https://pixhawk.ethz.ch/MAVLink/) compatible controller (an [APM 2.6](https://store.3drobotics.com/products/apm-2-6-kit-1) for the BlueROV) is used as a sensor platform (IMU with DCM, internal pressure, battery stats) and as an interface to multiple thrusters. A ROS node proxies messages between the MAVLink compatible controller and the embedded Linux computer over serial. A camera (a [RaspberryPi Camera](https://www.raspberrypi.org/products/camera-module/) in this case) is also connected to the embedded Linux computer.

![BlueROV Schematic](BlueROV%20Schematic.jpg)

Note: The xml version of embedded diagrams can be modified with [https://www.draw.io/](https://www.draw.io/).

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
