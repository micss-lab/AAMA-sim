# General Requirements

## Requirements for Win11

### Install WSL2
Follow instructions through [here](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-10#7-enjoy-ubuntu-on-wsl).
### Install CUDA for WSL2
Follow instructions through [here](https://docs.nvidia.com/cuda/wsl-user-guide/index.html#getting-started-with-cuda-on-wsl).
### Install Docker Windows
Follow instructions through [here](https://docs.docker.com/desktop/windows/wsl/)

## Requirements for Ubuntu
- This installation requires Ubuntu 22.04 LTS

### Install Git

- Install Git
  ```
  sudo apt update
  sudo apt install git
  ```

### Install Docker Engine

- Make the `install_docker.sh` script executable.
  ```
  chmod +x install_docker.sh
  ```
- Run the script.
  ```
  sudo ./install_docker.sh
  ```

- In order to use `docker` commands without `sudo` you have to log out and login.

### Install ROS for Ubuntu
These steps are same for both Ubuntu and WSL2.

This will install all dependencies and packages required.

- Make the `install_ros2.sh` script executable.
  ```
  chmod +x install_ros2.sh
  ```
- Run the script.
  ```
  sudo ./install_ros2.sh
  ```

### Install Gazebo Fortress for Ubuntu

This will install all dependencies and packages required.

- Make the `install_gazebo.sh` script executable.
  ```
  chmod +x install_gazebo.sh
  ```
- Run the script.
  ```
  sudo ./install_gazebo.sh
  ```
  
### Install RabbitMQ Docker
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
  ```
  docker exec myrabbit rabbitmqctl add_user dogu dogu
  docker exec myrabbit rabbitmqctl set_user_tags dogu administrator
  docker exec myrabbit rabbitmqctl set_permissions -p / dogu ".*" ".*" ".*"

  ```

