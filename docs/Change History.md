# Change History

This project uses [semantic versioning](http://semver.org/).

## v0.2.0 - 2015/06/15

* Added dynamic reconfigure to `simple_pilot` node to control biases
* Added `buoyancy_control` offset in `simple_pilot` node to account positive or negative buoyancy.
* Added thruster enable/disable behavior to `simple_pilot` node

## v0.1.0 - 2015/05/03

* Migrated `teleop_xbox` node from [mecanumbot-ros-pkg](https://github.com/joshvillbrandt/mecanumbot-ros-pkg)
* Added dynamic reconfigure to `teleop_xbox`
* Added `simple_pilot` node which sends thruster commands over MAVLink
* Added ROS installation documentation for the ROV computer and the workstation computer
* Added documentation
* Added launch files
* Added RQT configuration file
