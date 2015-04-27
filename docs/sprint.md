# BlueROV ROS Sprint

The goal of this sprint is to create a ROS package for the BlueROV that provides a solid foundation for community members to build on. See the implementation overview diagram.

Our primary concerns are:

* Interoperability with existing ROS tools
* Robust, closed-loop maneuvering
* Low-latency, real-time video
* Lowest possible learning curve for community members with custom sub configurations

## Monday

* Get the same version of ROS up and running on a RasberryPi and Josh’s Macbook
* Aim to use a virtual machine image with ROS pre-installed to make adoption easy for community * members
* Compile and run a custom, bare-bones binary on both machines to ensure proper installation

## Tuesday

* Complete percentage-based control from an Xbox controller all the way through the APM to the speed controllers
* DRY TEST: ensure proper output for all six thrusters from controller input
* DRY TEST: ensure adequate real-time video using raspicam_node or similar

## Wednesday

* Broadcast IMU message from the APM to ROS
* DRY TEST: ensure proper IMU output by rotating and translating the ROV in all directions
* Create an auto-calibration program that users can run for any sub configuration
    * This program should output test data and the config files for closed-loop control

## Thursday

* WET TEST: open-loop calibration pool test
    * goal is to get imu measurements for:
    * +x, -x, +y, -y, +z, -z movements at 20%, 40%, 60%, 80%, and 100% thrust
all six rotation speeds in the same increments as above
* Add IMU feedback into the control loop using acquired data

## Friday

* DRY TEST: ensure property output for all six thrusters based solely on IMU input
* WET TEST: closed-loop control test using real-units
