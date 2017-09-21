# bluerov-ros-pkg

A set of ROS package for the [BlueROV2](http://www.bluerobotics.com/store/rov/bluerov2/) and [ArduSub](https://www.ardusub.com/) compatible vehicles.

## Setup
* Flash a PixHawk with the latest stable version of [ArduSub](https://www.ardusub.com/software/ardusub-firmware.html)
* Flash the companion Pi with Bluerobotic's [image](https://www.ardusub.com/getting-started/installation.html#ardusub) of the pi
* Run the script in bluerov-ros-pkg/bluerov_apps/extra/ubuntu_setup.sh on your surface machine with Ubuntu.
* Set the IP of the surface machine to ```192.168.2.1```
* Connect the teather to the surface machine and power the rover
* Run the following to connect to the PixHawk on the surface machine
```
roslaunch bluerov_apps apm.launch
```
* Run one of the following to control the rover with a xbox/logitech f310 controller on the surface machine with the controller connected
```
#For xbox controller
roslaunch bluerov_apps teleop_xbox.launch

#For logitech F310 controller
roslaunch bluerov_apps teleop_f310.launch
```
* Install and run QGroundControl to edit pixhawk parameters, check arming status, monitor your vehicle, etc on your surface machine
* Disable autoconnect UDP in general settings of QGC, add 14552 to comm links and connect to it.

The documentation that follows in this file is specifically for those interested in contributing.

## Installation from Source

This will install both packages from source. If you have a fork of the repository, use the URL for your fork instead (or add it later with `git remote`.)

```bash
#Install ROS Kinetic

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y build-essential git
sudo apt-get install -y ros-kinetic-desktop-full ros-kinetic-mavros* ros-kinetic-joy

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir ~/repos
git clone https://github.com/bluerobotics/bluerov-ros-pkg.git ~/repos/bluerov-ros-pkg
ln -s ~/repos/bluerov-ros-pkg/rover ~/catkin_ws/src/rover
```

## Contributing

Interested in contributing to `bluerov-ros-pkg`? Good thing you're here. Please continue reading.

### Workflow

This project's workflow is modeled after Vincent Driessen's "[git flow](http://nvie.com/posts/a-successful-git-branching-model/)" pattern. Our general workflow looks something like this:

* Identification
  * Open a new issue, assuming one does not already exist.
  * Clearly describe the issue including steps to reproduce when it is a bug.
  * If the issue was not a part of a sprint approval process, discuss with the project lead before starting implementation
* Implementation
  * Clone the repository and create a feature branch from where you want to base your work.
  * At the very least, work in the `develop` branch. Merge requests to `master` will always be denied.
  * Push your code to the server and make a pull request to have your changes reviewed.
  * For features that take multiple days to develop, push the feature branch to the server at the end of each day. Open the pull request after pushing the feature branch to the server for the first time.
* Review and Merge
  * Code reviews take place in pull requests between a feature branch a the `develop` branch.
  * Pull requests must be reviewed by at least one primary maintainer of the repository.
  * Merge at will once you've received the required reviewers have approved the request pending stipulations (like "looks good to me once the build is passing.")
  * Delete the feature branch after it has been merged into `develop`.

### Commit Guidelines

Be sure to follow the code quality guidelines below. This project currently lacks a continuous integration service, so be extra sure that you adhere to these guidelines.

* Make commits of logical units.
* Make sure your commit messages are in the [proper format](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html).
  * Messages should be in the imperative: "Fix thing" instead of "Fixed thing" or "Fixes thing")
  * Commits should usually reference an open ticket: "Fix thing (#123)"
* Make sure your code conforms to the ([ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)) and adheres to the established style within the project.
* Make sure you have added the necessary tests for your changes.
* Run all the tests to assure nothing else was accidentally broken.

### Release

* Versions should follow [semantic versioning](http://semver.org/)
* Run `git tag -a v0.0.0 -m "v0.0.0"`
* Bump up the version number for development to what ever the next release will be
* Push changes with `git push origin master --tags`

## Change History

This project uses [semantic versioning](http://semver.org/). See [CHANGELOG.rst](CHANGELOG.rst) for details.
