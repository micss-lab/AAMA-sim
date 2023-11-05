#!/bin/bash

# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install argcomplete (optional)
sudo apt install -y python3-pip
pip3 install -U argcomplete

# Install colcon (optional but recommended for development)
sudo apt install -y python3-colcon-common-extensions

# Install additional RMW implementations (optional)
sudo apt install -y ros-humble-rmw-fastrtps-cpp ros-humble-rmw-cyclonedds-cpp

# Fix any missing dependencies
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths /opt/ros/humble/share --ignore-src --rosdistro humble -y

echo "ROS 2 Humble installation complete."

# Install ROS2 Packages
sudo apt install ros-humble-ros-gz ros-humble-ros-ign-gazebo

# Install Python3 dependencies
python3 -m pip install pika build
