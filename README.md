# AAMA-Sim
AAMA-ROS Simulation and Testing Environment

## Installation

This part assumes all prerequisite tools installed. If not please follow [here](setup.md).

- Create a Catkin Workspace. Dont run if already created `catkin_ws` from `setup.md`
  ```
   $ cd ~
   $ mkdir catkin_ws && cd catkin_ws
  ```
  
- Clone AAMA-Sim Repository into `src` of Catkin Workspace
  ```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/micss-lab/AAMA-sim.git
  ```
  
- Build the Catkin Workspace
  ```
   $ cd ~/catkin_ws
   $ catkin_make
  ```
  
- Add the Catkin Workspace and Gazebo Model Path to Bashrc
  ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/AAMA-sim/models" >> ~/.bashrc
   $ source ~/.bashrc
  ```

- Verify Installation by Using `roscd`
  ```
   $ cd ~
   $ roscd aama_sim
  ```
  
## How to Use AAMA-Sim

In order to run the Gazebo simulation with multiple robot run the commands below:

- Run with Multiple Robots. This Launch file will open a Gazebo instance with
3 Turtlebot3 instances.
  ```
   $ roslaunch aama_sim multi_robot_lab.launch
  ```
  
- Make Sure RabbitMQ Docker Container is up and running. If not use commands below to create a RabbitMQ container. 
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

- Start Simulation Controller Nodes
  ```
   $ roslaunch aama_sim sim_controller.launch
  ```
  
## Useful Links

- [Example JADE Agents for AAMA-Sim](https://github.com/micss-lab/AAMA-example-agents)
- [Turtlebot3 Source](https://github.com/ROBOTIS-GIT/turtlebot3)