# bluerov-ros-pkg

A set of ROS package for the [BlueROV](https://www.bluerobotics.com/product-category/rov/) and BlueROV compatible vehicles.

> This repository is currently RETIRED, UNSUPPORTED, and UNMAINTAINED.

## Documentation

The primary documentation for the packages contained in the repository is stored on the ROS wiki. Specifically, check out the following pages:

* http://wiki.ros.org/Robots/BlueROV
* http://wiki.ros.org/bluerov
* http://wiki.ros.org/bluerov_apps

The documentation that follows in this file is specifically for those interested in contributing.

## Installation from Source

This will install both packages from source. If you have a fork of the repository, use the URL for your fork instead (or add it later with `git remote`.)

```bash
# choose indigo dependencies
sudo apt-get install -y ros-indigo-image-common ros-indigo-robot-state-publisher ros-indigo-joint-state-publisher ros-indigo-image-transport-plugins ros-indigo-mavros ros-indigo-mavros-msgs ros-indigo-mavros-extras ros-indigo-joy
# or jade dependencies
sudo apt-get install -y ros-jade-image-common ros-jade-robot-state-publisher ros-jade-joint-state-publisher ros-jade-image-transport-plugins ros-jade-mavros ros-jade-mavros-msgs ros-jade-mavros-extras ros-jade-joy

# clone
mkdir ~/repos
git clone https://github.com/bluerobotics/bluerov-ros-pkg.git ~/repos/bluerov-ros-pkg
ln -s ~/repos/bluerov-ros-pkg/bluerov ~/catkin_ws/src/bluerov
ln -s ~/repos/bluerov-ros-pkg/bluerov_apps ~/catkin_ws/src/bluerov_apps

# udev rules
sudo cp ~/catkin_ws/src/bluerov/debian/99-bluerov.rules /etc/udev/rules.d/
sudo cp ~/catkin_ws/src/bluerov_apps/debian/99-bluerov-apps.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

# build
cd ~/catkin_ws
catkin_make
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
