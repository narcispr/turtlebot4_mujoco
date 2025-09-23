# TurtleBot4 MuJoCo Simulation for ROS 2

This package provides a simple, lightweight simulation of the TurtleBot4 mobile robot using [MuJoCo](https://mujoco.org/) and [ROS 2](https://docs.ros.org/).

## Overview

The `tb4_ros2_node` node initializes a MuJoCo simulation of a TurtleBot4 Lite and exposes basic sensor and control topics to the ROS 2 graph. This allows for testing and development of ROS 2 applications without needing a physical robot or a more heavyweight simulator.

### Features
- Publishes sensor data: `/imu`, `/joint_states`, `/scan`
- Subscribes to velocity commands: `/cmd_vel`
- Optional interactive viewer for the simulation.

## Dependencies

- **ROS 2 Jazzy Jellyfish** (or your specific ROS 2 version)
- **Python 3.8+**
- **MuJoCo** (`mujoco`)
- **NumPy** (`numpy`)

It is highly recommended to use a Python **virtual environment** to manage dependencies.

## Installation and Building

1.  **Clone the Package:**
    Place this package inside your ROS 2 workspace's `src` directory.

2.  **Create and Activate Virtual Environment:**
    Navigate to a convenient location, create a virtual environment, and install the required Python packages.
    ```bash
        # create a venv that can see /opt/ros/* python packages
        python3 -m venv --system-site-packages ~/ros2_venv
        source ~/ros2_venv/bin/activate

        # install python deps you need
        pip install -U pip
        pip install mujoco glfw numpy 
    ```

3.  **Build the Workspace:**
    Navigate to the root of your ROS 2 workspace and build the package using `colcon`.
    ```bash
        # Fresh shell
        source /opt/ros/jazzy/setup.bash
        source ~/ros2_venv/bin/activate            # venv ON
        cd ~/ros2_ws
        rm -rf build/ install/ log/                # important: regenerate wrappers
        python3 -m colcon build --symlink-install  # <- uses venv’s python
        . install/setup.bash
    ```

## Running the Simulation

Due to the way ROS 2 interacts with Python environments, you must follow a specific procedure to run the node.

1.  **Run the Mujoco simulator:**
    ```bash
        source /opt/ros/jazzy/setup.bash
        source ~/ros2_venv/bin/activate        
        cd ~/ros2_ws
        . install/setup.bash
        
        # Run 
        # Use arguments: 
        #   --headless to not open MuJoCo viewer
        #   --no_realtime to run as fast as possible)

        ros2 run turtlebot4_mujoco tb4_ros2_node
    ```

2. **Robot state publisher**

    ```bash
        cd ~/ros2_ws
        . install/setup.bash
        # Launch file with simplified turtlebot4 URDF
        ros2 launch turtlebot4_mujoco robot_description.launch.py
    ```

## ROS 2 Interface

#### Published Topics

-   **/imu** (`sensor_msgs/msg/Imu`)
    -   Provides orientation, angular velocity, and linear acceleration from the IMU sensor.
-   **/joint_states** (`sensor_msgs/msg/JointState`)
    -   Publishes the position and velocity of the left and right wheel joints.
-   **/scan** (`sensor_msgs/msg/LaserScan`)
    -   Provides laser scan data from the RPLIDAR.

#### Subscribed Topics

-   **/cmd_vel** (`geometry_msgs/msg/Twist`)
    -   Receives velocity commands to control the robot's movement. Linear `x` controls forward/backward motion, and angular `z` controls turning.

