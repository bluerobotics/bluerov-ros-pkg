# Todo

The follow items are high-level areas that we'd like to improve upon.

## Improve Manual Driving from Surface Computer

Add a HUD setup to improve situational awareness to driver.

## Improve Setup for Custom ROV Platforms

* Start on the auto-gain determination from thruster configuration (URDF)
    * (there will still be a layer of manual gains on top of this)
* Better documentation for using the library with a custom robot
    * how to set up the URDF
    * simple teleop control vs auto-gain determination from URDF
    * open-loop controller vs closed-loop controller

## Visualize ROV Movement

* Work on the URDF file including transforms for thrusters and sensors
* Possibly try to extract the solidworks models for key components to display in rviz (otherwise use geometric stand-ins)
* Write a quick program to add "markers" to rviz to view commanded thruster values in 3D
* Show orientation of sub in rviz
* Publish odometry data from thruster commands (should be able to see full movement in rviz now)
    * add time-trail in rviz to see position over time

## Add Closed-Loop Position and Orientation Control

Utilize the IMU and a future depth sensor to improve tolerance of disturbances.

* Get the EKF working with IMU and odometry
* Start on the closed-loop orientation stuff
    * decouple motion
    * position hold

## Keyboard control of the ROV

Migrate teleop_keyboard from the mecanumbot-ros-pkg.
