sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install -y build-essential git wpasupplicant screen
sudo apt-get install -y ros-indigo-ros-base
sudo apt-get install -y ros-indigo-image-common ros-indigo-image-transport-plugins ros-indigo-mavros ros-indigo-mavros-msgs ros-indigo-mavros-extras ros-indigo-joy

sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir ~/repos
git clone https://github.com/bluerobotics/bluerov-ros-pkg.git ~/repos/bluerov-ros-pkg
ln -s ~/repos/bluerov-ros-pkg/bluerov ~/catkin_ws/src/bluerov
ln -s ~/repos/bluerov-ros-pkg/bluerov_apps ~/catkin_ws/src/bluerov_apps

sudo cp ~/catkin_ws/src/bluerov/debian/99-bluerov.rules /etc/udev/rules.d/
sudo cp ~/catkin_ws/src/bluerov_apps/debian/99-bluerov-apps.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
