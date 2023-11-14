# AAMA-Sim

AAMA-ROS Simulation and Testing Environment

## Installation

- Create a ROS2 Workspace
  ```
   cd ~
   mkdir -p ros2_ws/src && cd ros2_ws
  ```

- Clone AAMA-Sim Repository into `src` of ROS2 Workspace
  ```
   cd ~/ros2_ws/src
   git clone https://github.com/micss-lab/AAMA-sim.git
  ```

- Please install the requirements. Follow [here](setup.md).


- Build the ROS2 Workspace
  ```
   cd ~/ros2_ws
   colcon build
  ```

- Add the ROS2 Workspace and Gazebo Model Path to Bashrc
  ```
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
   echo "export IGN_GAZEBO_RESOURCE_PATH=:~/ros2_ws/src/AAMA-sim/models" >> ~/.bashrc
   source ~/.bashrc
  ```

## How to Use AAMA-Sim

### How to Run Multi Robot Simulation

- Run with Multiple Robots. This Launch file will open a Gazebo instance with
  3 Robot instances.
  ```
   ros2 launch aama_sim multi_robot_lab.launch.py
  ```

    - Make Sure RabbitMQ Docker Container is up and running. If not use commands below to create a RabbitMQ container.

### How to change robot count in Simulation

- In order to change the number of robots in the simulation you have to change a parameter
  in `multi_robot_lab.launch.py` file
    - Change the `ROBOT_COUNT` parameter to an integer between 1 and 24.
    - Execute command `colcon build` in `ros2_ws` directory.

### Sensor and Control Messages

#### Available Sensors

| Sensor Name    | ROS Topic                         | RabbitMQ Topic |
|----------------|-----------------------------------|----------------|
| IMU            | `/imu`                            | `imu`          |
| Sonar          | `/sonar`                          | `sonar`        |
| UWB            | `/model/{robot_name}/pose_static` | `uwb`          |
| Logical Camera | Not Available                     | Not Available  |
| Camera         | Not Available                     | Not Available  |
| LIDAR          | Not Available                     | Not Available  |

#### Robot Control Message

- RabbitMQ Message Queue Name: `/robot_ctrl`
- Message
  Type: `RobotMsgs/RobotControl.java` ([Check here](https://github.com/micss-lab/AAMA-example-agents/blob/main/src/main/java/RobotMsgs/RobotControl.java))
- Message Format:
  ```json5
  [
    {
      "robot_id": "0",
      "orientation": { // All angles here are in degrees
        "x": 0.0, // Angular Speed in X axis
        "y": 0.0, // Angular Speed in Y axis
        "z": 0.0 // Angular Speed in Z axis
      },
      "position": {
        "x": 1.0, // Linear Speed in X axis
        "y": 0.0, // Linear Speed in Y axis
        "z": 0.0 // Linear Speed in Z axis
      }
    },
    {
      "robot_id": "1",
      "orientation": { // All angles here are in degrees
        "x": 0.0, // Angular Speed in X axis
        "y": 0.0, // Angular Speed in Y axis
        "z": 0.0 // Angular Speed in Z axis
      },
      "position": {
        "x": 1.0, // Linear Speed in X axis
        "y": 0.0, // Linear Speed in Y axis
        "z": 0.0 // Linear Speed in Z axis
      }
    }
  ]
  ```

#### IMU Sensor Message

- RabbitMQ Message Queue Name: `/imu`
- Message
  Type: `RobotMsgs/IMU.java` ([Check here](https://github.com/micss-lab/AAMA-example-agents/blob/main/src/main/java/RobotMsgs/IMU.java))
- Message Format:
  ```json5
  [
    {
      "header": {
        "stamp": {
          "sec": 10,
          "nanosec": 0
        },
        "frame_id": "aama_robot_0"
      },
      "orientation": {
        "x": -6.108140001604861e-16,
        "y": -3.5351844305974787e-10,
        "z": 2.375124229387805e-19,
        "w": 1
      },
      "angular_velocity": {
        "x": 1.1611069558308134e-16,
        "y": 1.3836152168523395e-16,
        "z": -3.8590286387421885e-19
      },
      "linear_acceleration": {
        "x": 6.9291450541440105e-9,
        "y": -1.0794232937611885e-14,
        "z": 9.800000000000258
      }
    },
    {
      "header": {
        "stamp": {
          "sec": 10,
          "nanosec": 0
        },
        "frame_id": "aama_robot_3"
      },
      "orientation": {
        "x": -6.107806083661822e-16,
        "y": -3.535184430412162e-10,
        "z": -2.7722177154154196e-19,
        "w": 1
      },
      "angular_velocity": {
        "x": 1.1740794738697006e-16,
        "y": 1.4013788152360746e-16,
        "z": -7.58844641920445e-20
      },
      "linear_acceleration": {
        "x": 6.928961606773204e-9,
        "y": -1.3134548269889717e-14,
        "z": 9.800000000000002
      }
    }
  ]
  ```

#### Sonar Sensor Message

- RabbitMQ Message Queue Name: `/sonar`
- Message
  Type: `RobotMsgs/Sonar.java` ([Check here](https://github.com/micss-lab/AAMA-example-agents/blob/main/src/main/java/RobotMsgs/Sonar.java))
- Message Format:
  ```json5
  [
    {
      "header": {
        "stamp": {
          "sec": 1221,
          "nanosec": 0
        },
        "frame_id": "aama_robot_2"
      },
      "angle_min": 0,
      "angle_max": 0,
      "angle_increment": "NaN",
      "time_increment": 0,
      "scan_time": 0,
      "range_min": 0.07999999821186066,
      "range_max": 10,
      "ranges": [ // All distances are in meters. If no obstacle is detected, the distance is 11 meters.
        11
      ],
      "intensities": [
        0
      ]
    },
    {
      "header": {
        "stamp": {
          "sec": 1220,
          "nanosec": 900000000
        },
        "frame_id": "aama_robot_3"
      },
      "angle_min": 0,
      "angle_max": 0,
      "angle_increment": "NaN",
      "time_increment": 0,
      "scan_time": 0,
      "range_min": 0.07999999821186066,
      "range_max": 10,
      "ranges": [
        11
      ],
      "intensities": [
        0
      ]
    }
  ]
  ```

#### UWB Sensor Message

- RabbitMQ Message Queue Name: `/uwb`
- Message
  Type: `RobotMsgs/UWB.java` ([Check here](https://github.com/micss-lab/AAMA-example-agents/blob/main/src/main/java/RobotMsgs/UWB.java))
- Message Format:

  ```json5
  [
    {
      "robot_id": "1",
      "header": {
        "stamp": {
          "sec": 2,
          "nanosec": 720000000
        },
        "frame_id": "aama_robot_1"
      },
      "orientation": {
        "x": -180,
        // All angles here are in degrees
        "y": 0,
        "z": 0
      },
      "position": {
        "x": 0,
        // All distances are in meters
        "y": 12,
        "z": 0.375
      }
    },
    {
      "robot_id": "3",
      "header": {
        "stamp": {
          "sec": 2,
          "nanosec": 720000000
        },
        "frame_id": "aama_robot_3"
      },
      "orientation": {
        "x": -180,
        "y": 0,
        "z": 0
      },
      "position": {
        "x": 4,
        "y": 0,
        "z": 0.375
      }
    }
  ]
  ```

## Useful Links

- [Example JADE Agents for AAMA-Sim](https://github.com/micss-lab/AAMA-example-agents)