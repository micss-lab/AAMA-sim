# General Requirements

## Requirements for Win11

### Install WSL2
Follow instructions through [here](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-10#7-enjoy-ubuntu-on-wsl).
### Install CUDA for WSL2
Follow instructions through [here](https://docs.nvidia.com/cuda/wsl-user-guide/index.html#getting-started-with-cuda-on-wsl).
### Install Docker Windows
Follow instructions through [here](https://docs.docker.com/desktop/windows/wsl/)

## Requirements for Ubuntu
### Install Docker Engine
- Install required packages
  ```
  $ sudo apt-get update
  $ sudo apt-get install \
      ca-certificates \
      curl \
      gnupg \
      lsb-release
  ```
- Add GPG Key
  ```
   $ sudo mkdir -p /etc/apt/keyrings
   $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  ```
- Setup repository
  ```
   $ echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  ```
- Update APT
  ```
   $ sudo apt update
  ```
- Install Docker Engine
  ```
   $ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
  ```
- Execute commands below to be able to use Docker without `sudo`. Need to relog to make it work.
  ```
   $ sudo groupadd docker
   $ sudo usermod -aG docker $USER
  ```

### Install ROS for Ubuntu
These steps are same for both Ubuntu and WSL2.

- Add repo to system
  ```
   $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  ```
- Setup keys
  ```
   $ sudo apt install curl # if you haven't already installed curl
   $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  ```
- Update APT
  ```
   $ sudo apt update
  ```
- Install ROS Noetic
  ```
   $ sudo apt install ros-noetic-desktop-full
  ```
- Install ROS Noetic Gazebo Packages
  ```
   $ sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-dev ros-noetic-gazebo-plugins ros-noetic-gazebo-msgs
  ```
- Add ROS Installation to bashrc
  ```
   $ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
  ```

## Create catkin_ws
- Create catkin_ws directory
  ```
   $ cd
   $ mkdir catkin_ws && cd catkin_ws
   $ catkin_make
  ```
- Add catkin_ws to bashrc
  ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
  ```

## Install Turtlebot3

- Install Dependent ROS Packages
  ```
   $ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
      ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
      ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
      ros-noetic-rosserial-python ros-noetic-rosserial-client \
      ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
      ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
      ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
      ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
  ```
- Install TurtleBot3 Packages
  ```
  $ sudo apt install ros-noetic-dynamixel-sdk
  ```  
- Clone Turtlebot3 Source Code
  ```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
   $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
   $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  ```
- Add Default Robot Type
  ```
   $ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
   $ source ~/.bashrc
  ```
  
- Build catkin_ws
  ```
   $ cd ~/catkin_ws/src
   $ catkin_make
  ```
  
## Install RabbitMQ Docker
These commands should both work for Ubuntu and WSL2 Ubuntu. However, WSL2 Ubuntu users have access to Docker GUI in Windows and therefore can use it to run containers.

- Create RabbitMQ Docker Container and Run
  ```
   $ docker run -d --hostname my-rabbit --name myrabbit -e RABBITMQ_DEFAULT_USER=admin -e RABBITMQ_DEFAULT_PASS=123456 -p 5672:5672 -p 15672:15672 rabbitmq:3-management
  ```
- Access RabbitMQ Management Web GUI from 
  - `http://localhost:15672`
  - Login with ID: admin pass: 123456
- Create a new user with admin privilages with below ID and password.
  - ID: dogu
  - Password: dogu


