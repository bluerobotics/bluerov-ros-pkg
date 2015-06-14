# Setup

The `bluerov-ros-pkg` software stack is built on [ROS](http://www.ros.org/), the Robotic Operation System. It is compatible with the BlueRobotics BlueROV and BlueROV-compatible vehicles. Compatible vehicles have an on-board Linux computer with a MAVLink-compatible controller interfaces with low-level sensors and actuators. Users are also expected to have an onshore computer for tele-operation (referred to here as the "workstation" computer.)

## Overview

This setup guide will help you get the workstation and ROV computers up and running with ROS. The next two sections cover installation instructions specific to the two platforms. The following section covers the installation of the BlueROV ROS package, which is needed for both platforms.

## Workstation Preparation

ROS is a powerful framework, but only once it has been installed... We highly recommend sticking with Ubuntu for your workstation computer as this is the only officially supported operating system. As of this writing, we are using Ubuntu 14.04 and ROS 8 (Indigo Igloo.)

If you intend to run any of the 3D ROS applications (rviz, gazebo), you should either install Ubuntu directly onto the computer (not as a VM) or make sure that your computer has a discrete graphics card if you plan on running Ubuntu through a virtual machine. You've been warned!

### Workstation ROS Installation - Direct 

Installing ROS directly onto an Ubuntu machine is the best way to develop with ROS. You will experience the fewest hiccups and you should be able to find plenty of support since this is the most common install method for ROS. Please view the [official ROS installation instructions](http://wiki.ros.org/indigo/Installation/Ubuntu) for the most up-to-date instructions. Those instructions basically boil down to this:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-indigo-desktop

sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Workstation ROS Installation - Virtual Machine

For first time ROS users, one option is the Ubuntu 14.04 virtual machine images with ROS Indigo Igloo preinstalled by Nootrix. This option is a good choice if Ubuntu isn't your primary operating system.

Choose the 64-bit version if your machine has the resources to support it, otherwise use the 32-bit version. To install using this method, please do the following:

1. visit the [Nootrix blog post](http://nootrix.com/2014/09/ros-indigo-virtual-machine/) on the subject
1. download and install [VirtualBox](https://www.virtualbox.org/) and the VirtualBox Extension Pack for your host machine
1. download one of the ROS [virtual machine images from Nootrix](http://nootrix.com/downloads/#RosVM)
1. open VirtualBox, choose File > Import Appliance and then select the virtual machine image that you downloaded

For reference, the hostname on the VM defaults to `c3po` with the username:password combo defaulting to `viki:viki`.

ProTip: If you decide to use a shared folder between your host machine and the VM, be sure to run `sudo usermod -aG vboxsf $(whoami)` and restart or login/logout in order to have write privileges in the shared folder from the VM.

### Xbox Controller Setup

TODO

## ROV Preparation

We are using a RaspberryPi 2 as our embedded ROV computer. We recommend installing Ubuntu 14.04 on the RaspberryPi over Rasbian OS because of the ease of installing ROS packages. On Ubuntu, packages can be installed as binaries from `apt`, but on Rasbian, most packages must be installed from source.

### Flashing an SD Card with Ubuntu

Follow [these instructions](https://wiki.ubuntu.com/ARM/RaspberryPi) to flash Ubuntu onto an SD card for the RaspberryPi. Once the install has completed, use a keyboard and monitor to complete the partition resizing and OS essentials section. The username and password for the standard Ubuntu image is `ubuntu:ubuntu`. After the SSH server has been installed, you can work remotely.

There are no Raspbian-specific utilities included, specifically no automatic root resizer. However, it's not hard to do manually. Once booted, you'll need to resize the primary partition and create a swapfile:

```bash
sudo fdisk /dev/mmcblk0
# Delete the second partition (d, 2),
# then re-create it using the defaults (n, p, 2, enter, enter),
# then write and exit (w)
# then restart.

sudo shutdown -r now

sudo resize2fs /dev/mmcblk0p2
sudo apt-get install dphys-swapfile # this step might take a while
```

You can use `dh -f` to verify the partition has been resized.

If you get stuck, this [elinux page](http://elinux.org/RPi_Resize_Flash_Partitions#Manually_resizing_the_SD_card_on_Linux
https://www.raspberrypi.org/forums/viewtopic.php?f=91&t=24993) is also a good resource for flashing SD cards.

### Setup OS Essentials

```bash
sudo hostname bluerov
sudo sh -c 'echo "bluerov" > /etc/hostname'

sudo apt-get update
sudo apt-get install libraspberrypi-bin libraspberrypi-dev openssh-server wpasupplicant git build-essential avahi-daemon screen linux-firmware -y
sudo apt-get upgrade

# some libraries require the libraspberrypi-bin and libraspberrypi-dev files at /opt/vc
sudo ln -s /usr /opt/vc
```

### ROV Wifi Network Setup

Setting up Wifi on your ROV machine will enable easy development and will leave your Ethernet port free for direct connection use (see below.)

First, write your network connection details to `/etc/wpa_supplicant/wpa_supplicant.conf`:

```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="YourWifiNetworkName"
    proto=RSN
    key_mgmt=WPA-PSK
    pairwise=CCMP TKIP
    group=CCMP TKIP
    psk="YourWifiNetworkPassword"
}
```

Next, configure your wireless adapter by adding or modifying the settings in `/etc/network/interfaces`:

```
allow-hotplug wlan0
iface wlan0 inet manual
wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
iface wlan0 inet dhcp
auto wlan0
```

Finally, restart your network interface by restarting the computer or the networking daemon and test out your new connection!

```bash
sudo /etc/init.d/networking restart
ping google.com
```

For more information, check out [this guide](https://kerneldriver.wordpress.com/2012/10/21/configuring-wpa2-using-wpa_supplicant-on-the-raspberry-pi/).

## ROV APM Setup

BlueROV compatible vehicles use a mavlink compatible device as the interface to low-level sensors and actuators. For the BlueROV, we are using an ArduPilot Mega (APM) 2.6.

We have a slight modification to the stock APMrover code for use with an APM 2.6. Please upload the following code to the APM.

```bash
git clone -b br-ros-rov https://github.com/bluerobotics/ardupilot.git
cd ardupilot/APMrover2
make configure
# gedit ../config.mk&
# sudo apt-get install gawk
make
make upload
```

Note to self: In the future, try out a PX4 with the "OFFBOARD" mode.

### ROV Camera Configuration

To get the camera working in Ubuntu 14.04:

```bash
sudo sh -c 'echo "start_x=1\ngpu_mem=128" >> /boot/config.txt'
sudo apt-get install libraspberrypi-dev libraspberrypi-bin
sudo shutdown -r now
raspistill -o test.jpg
scp ubuntu@bluerov:~/test.jpg ~/
```

For more information, check out [this forum post](https://www.raspberrypi.org/forums/viewtopic.php?f=56&t=100553).

### ROV ROS Installation

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-indigo-ros-base

sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## BlueROV ROS Package Installation

By this point, you should have already installed ROS on both the workstation and the ROV. Follow the steps below for both machines.

Let's start by making sure we have all of our dependencies installed:

```bash
sudo apt-get install python-setuptools python-pip python-yaml python-argparse python-distribute python-docutils python-dateutil python-setuptools python-six -y
sudo pip install rosdep rosinstall_generator wstool rosinstall

sudo apt-get install ros-indigo-image-common ros-indigo-image-transport-plugins ros-indigo-mavros ros-indigo-mavros-extras ros-indigo-joy -y
```

First establish a catkin workspace if you haven't already:

```bash
# create and init workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# build workspace
cd ~/catkin_ws/
catkin_make

# add workspace to path
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc

# verify path
echo $ROS_PACKAGE_PATH
```

Then install the `bluerov` and `raspicam_node` packages and rebuild.

```bash
cd ~/catkin_ws/src/
git clone https://github.com/bluerobotics/bluerov-ros-pkg.git bluerov
git clone https://github.com/fpasteau/raspicam_node

cd ~/catkin_ws/
catkin_make
```

Next, set up udev rules to make devices easier to find:

```bash
sudo cp ~/catkin_ws/src/bluerov-ros-pkg/extra/99-bluerov.rules /etc/udev/rules.d/
sudo udevadm trigger # to immediately reload the rules without restarting
```

Check out [this syntax guide](http://www.reactivated.net/writing_udev_rules.html#syntax) for creating new udev rules.

## Direct Network Configuration

When performing tele-operated tests and missions, it can be desirable to connect the workstation and ROV together using a direct Ethernet connection. Wifi connectivity obviously isn't an option through water, and a direct Ethernet connection eliminates the need for a network switch between the two computers.

On the workstation machine, choose an Ethernet interface to dedicate to the ROV. We will use `eth0`. Modify the file `/etc/network/interfaces` by adding or replacing the following information for your chosen interface:

```bash
## ROV direct connection
auto eth0
iface eth0 inet static
   address 10.0.0.10
   netmask 255.255.255.0
```

On the ROV, edit the same file but use a different IP address:

```bash
## ROV direct connection
auto eth0
iface eth0 inet static
   address 10.0.0.20
   netmask 255.255.255.0
```

Reboot or restart the networking daemon for these settings to take effect. You should now be able to ping between the two machines:

```bash
# from the workstation to the ROV
ping 10.0.0.20
# from the ROS to the workstation
ping 10.0.0.10
```

For more information, check out [this guide](http://askubuntu.com/questions/22835/how-to-network-two-ubuntu-computers-using-ethernet-without-a-router).
