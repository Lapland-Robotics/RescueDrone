#!/usr/bin/env bash
sudo apt-get update &&
sudo apt-get autoremove -y &&
sudo apt-get upgrade -y &&
sudo apt-get install gcc cmake curl nano -y &&
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - &&
sudo apt update &&
sudo apt install ros-melodic-desktop-full -y &&
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc &&
echo "export EDITOR='nano -w'" >> ~/.bashrc &&
source /opt/ros/melodic/setup.bash &&
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y &&
sudo rosdep init &&
rosdep update &&
sudo apt install ros-melodic-nmea-comms -y &&
sudo apt-get install libusb-1.0-0-dev -y &&
cd ~/ &&
mkdir catkin_ws && cd catkin_ws &&
mkdir src && cd src &&
git clone -b 3.8 https://github.com/dji-sdk/Onboard-SDK-ROS.git Onboard-SDK &&
curl https://pastebin.com/raw/bUZwn3EY > Onboard-SDK/dji_sdk/src/modules/dji_sdk_node.cpp &&
cd .. &&
rosdep install -y --rosdistro melodic --from-paths src --ignore-src &&
catkin_make_isolated &&
echo "source devel_isolated/setup.bash" >> ~/.bashrc &&
source ~/catkin_ws/devel_isolated/setup.bash &&
sudo adduser $USER dialout &&
sudo systemctl stop nvgetty &&
sudo systemctl disable nvgetty &&

cd ~/ &&
git clone https://github.com/jetsonHacksNano/installVSCode.git &&
cd installVSCode &&
sudo ./installVSCode.sh &&

sudo reboot