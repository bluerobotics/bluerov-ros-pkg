# Setup

The BlueROV software stack is built on [ROS](http://www.ros.org/), the Robotic Operation System. When operating a BlueROV, there should be one computer onboard the vehicle (as RaspberryPi in this documentation) and one computer for tele-operation (referred to here as the "workstation" computer.)

## Workstation Operating System

ROS is a powerful framework, but only once it has been installed... We highly recommend sticking with Ubuntu 14.04 for your workstation computer. If you intend to run any of the 3D ROS applications (rviz, gazebo), you should either install Ubuntu directly onto the computer (not as a VM) or make sure that your computer has a discrete graphics card if you plan on running Ubuntu through a virtual machine. You've been warned!

## Workstation ROS Setup

For first time ROS users, we highly recommend installing the Ubuntu 14.04 virtual machine images with ROS Indigo Igloo preinstalled by Nootrix. Choose the 64-bit version if your machine has the resources to support it, otherwise use the 32-bit version. To install using this method, please do the following:

1. visit the [Nootrix blog post](http://nootrix.com/2014/09/ros-indigo-virtual-machine/) on the subject
1. download and install [VirtualBox](https://www.virtualbox.org/) and the VirtualBox Extension Pack for your host machine
1. download one of the ROS [virtual machine images from Nootrix](http://nootrix.com/downloads/#RosVM)
1. open VirtualBox, choose File > Import Appliance and then select the virtual machine image that you downloaded

For reference, the hostname on the VM defaults to `c3po` with the username:password combo defaulting to `viki:viki`.

ProTip: If you decide to use a shared folder between your host machine and the VM, be sure to run `sudo usermod -aG vboxsf $(whoami)` and restart or login/logout in order to have write privileges in the shared folder from the VM.

ProTip: Check out the [ROS Cheat sheet](http://www.clearpathrobotics.com/wp-content/uploads/2014/01/ROS-Cheat-Sheet-v1.01.pdf) from Clearpath Robotics

## RaspberryPi ROS Setup

The ROS community maintains excellent instructions on getting [ROS to run on a RaspberryPi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi). Before you get started though, make sure you have the Raspbian OS installed on an SD card with enough space to install ROS:

1. Download NOOBS from [raspberrypi.org/](https://www.raspberrypi.org/downloads/)
1. Insert an SD card and format it as FAT32
1. Extract the files from the NOOBS download and copy them onto the SD card
1. Make sure you have a keyboard and monitor plugged into the Pi
1. Insert the SD card into the Pi and power it on
1. Install the OS via the on-screen menus (press `enter` and then `i`)

At this point, you'll want to make sure that the following setup items have been enabled through `sudo raspi-config`:

* your primary partition has been expanded to take the full capacity of the SD card
* the camera module is enable (if applicable)
* advanced menu > ssh server is enabled
* advanced menu > set a custom hostname if desired (we used "bluerov")

Eventually, you'll probably connect the Pi directly to a laptop during a wet test, but connect your Pi to your local network for now. If you want to use wifi, get ahold of a wireless adapter and read [some](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md) [documentation](https://kerneldriver.wordpress.com/2012/10/21/configuring-wpa2-using-wpa_supplicant-on-the-raspberry-pi/) on the subject. Be sure to add "auto wlan0" to the end of `/etc/network/interfaces` so that the wifi adapter comes up by default on boot up.

Once the network is up and running, we recommend enabling a multicast DNS address for your Pi. This enables you to access your Pi on the network by name instead of by IP address. Since we set our hostname to "bluerov", our new DNS name is `bluerov.local`. To enable this feature, follow [these instructions](http://elinux.org/RPi_Advanced_Setup) to install and configure `avahi-daemon`. It should look something like this:

```bash
sudo apt-get update
sudo apt-get install -y avahi-daemon
```

At this point, make sure you can ssh into your pi one way or another. Try `ssh pi@bluerov.local` if you set up mDNS, otherwise you can log in with the IP address which you can find via `ifconfig`. Once you have confirmed that ssh works, you can ditch the keyboard and monitor that are hooked directly to your Pi and work from your workstation instead.

You are now ready to install ROS on the Pi. The installation steps we used are provided below, but you should really consult the official [ROS installation instructions](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi) before proceeding.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu wheezy main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade

sudo apt-get install -y python-setuptools python-pip python-yaml python-argparse python-distribute python-docutils python-dateutil python-setuptools python-six
sudo pip install rosdep rosinstall_generator wstool rosinstall

sudo rosdep init
rosdep update

mkdir ~/catkin_ws
cd ~/catkin_ws

rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
wstool init src indigo-ros_comm-wet.rosinstall

sudo apt-get install -y checkinstall cmake
sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
sudo apt-get update

mkdir ~/catkin_ws/external_src
cd ~/catkin_ws/external_src

sudo apt-get build-dep console-bridge
apt-get source -b console-bridge
sudo dpkg -i libconsole-bridge0.2_*.deb libconsole-bridge-dev_*.deb

apt-get source -b lz4
sudo dpkg -i liblz4-*.deb

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:wheezy
# python-rosdep, python-catkin-pkg, python-rospkg, and python-rosdistro installs fail, but that is OK because we installed then via pip earlier

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo
echo '/opt/ros/indigo/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## BlueROV ROS Package

The following steps assume that you are using the Nootrix image or have otherwise followed ROS' guidelines [configuring your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Perform these steps for both the on-vehicle computer and the workstation computer.

Check out the BlueROV repository:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/bluerobotics/BlueROV.git bluerov
```

Install missing dependencies:

```bash
sudo rosdep init # first time only, but will gracefully error otherwise
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
# sudo apt-get install -y ros-indigo-joy
```

You should then be able to build your catkin workspace:

```bash
cd ~/catkin_ws
catkin_make
```

You'll probably want to add the resulting setup file to your `~/.bashrc` script.

```bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

Next, set up udev rules to make devices easier to find:

```bash
sudo cp ~/catkin_ws/src/bluerov/extra/99-bluerov.rules /etc/udev/rules.d/
sudo udevadm trigger # to immediately reload the rules without restarting
```

Check out [this syntax guide](http://www.reactivated.net/writing_udev_rules.html#syntax) for creating new udev rules.

## APM ROS Installation

We use TODO to compile and program the APM from the command line.

TODO

## Xbox Controller Setup

TODO
