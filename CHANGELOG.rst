^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bluerov
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2016-02-15)
-------------------

* Added support for the Pixhawk+APM stack instead of the APM+old APM stack
* Renamed `teleop_xbox` to `teleop_joy` and refactored it for the new stack
* Organized content into two packages: `bluerov` and `bluerov_apps`
* Moved documentation to ROS wiki
* Added udev rules for the xbox controller and f310 controller

0.4.0 (2016-01-16)
-------------------

* Last planned release for APM support
* Fixed `CMD_DO_SET_SERVO` is not a member compile error to due to unexpected update from dependency packages
* Updated URL for the APM software

0.3.0 (2015-06-23)
-------------------

* Updated document ahead of public announcement

0.2.0 (2015-06-15)
-------------------

* Added dynamic reconfigure to `simple_pilot` node to control biases
* Added `buoyancy_control` offset in `simple_pilot` node to account positive or negative buoyancy.
* Removed mavros arming command from `teleop_xbox` node since APMrover wasn't listening to it anyway
* Added hazard enable/disable behavior between `teleop_xbox` and `simple_pilot` nodes

0.1.0 (2015-05-03)
-------------------

* Migrated `teleop_xbox` node from [mecanumbot-ros-pkg](https://github.com/joshvillbrandt/mecanumbot-ros-pkg)
* Added dynamic reconfigure to `teleop_xbox`
* Added `simple_pilot` node which sends thruster commands over MAVLink
* Added ROS installation documentation for the ROV computer and the workstation computer
* Added documentation
* Added launch files
* Added RQT configuration file
