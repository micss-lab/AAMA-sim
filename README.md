# AAMA-Sim

AAMA-ROS Simulation and Testing Environment

## Installation

This part assumes all prerequisite tools installed. If not please follow [here](setup.md).

- Create a ROS2 Workspace
  ```
   cd ~
   mkdir -p ros2_ws/src && cd ros2_ws
  ```

- Clone AAMA-Sim Repository into `src` of ROS2 Workspace
  ```
   cd ~/ros2_ws/src
   git clone https://github.com/micss-lab/AAMA-sim.git -b ros2-devel
  ```

- Build the ROS2 Workspace
  ```
   cd ~/ros2_ws
   colcon build
  ```

- Add the ROS2 Workspace and Gazebo Model Path to Bashrc
  ```
   echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
   echo "export IGN_GAZEBO_RESOURCE_PATH=IGN_GAZEBO_RESOURCE_PATH:~/ros2_ws/src/AAMA-sim/models" >> ~/.bashrc
   source ~/.bashrc
  ```

## How to Use AAMA-Sim

### How to Run Multi Robot Simulation

- Run with Multiple Robots. This Launch file will open a Gazebo instance with
  3 Robot instances.
  ```
   $ ros2 launch aama_sim multi_robot_lab.launch
  ```

    - Make Sure RabbitMQ Docker Container is up and running. If not use commands below to create a RabbitMQ container.

### How to change robot count in Simulation

- In order to change the number of robots in the simulation you have to change a parameter
  in `multi_robot_lab.launch.py` file
  - Change the `ROBOT_COUNT` parameter to an integer between 1 and 24.
  - Execute command `colcon build` in `ros2_ws` directory.

## Useful Links

- [Example JADE Agents for AAMA-Sim](https://github.com/micss-lab/AAMA-example-agents)