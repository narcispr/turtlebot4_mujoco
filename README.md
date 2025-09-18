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
    python3 -m venv venv
    source venv/bin/activate
    pip install mujoco numpy
    ```

3.  **Build the Workspace:**
    Navigate to the root of your ROS 2 workspace and build the package using `colcon`.
    ```bash
    cd /path/to/your/ros2_ws
    colcon build --symlink-install
    ```

## Running the Simulation

Due to the way ROS 2 interacts with Python environments, you must follow a specific procedure to run the node.

1.  **Open a new terminal.**

2.  **Source your ROS 2 workspace:**
    ```bash
    source /path/to/your/ros2_ws/install/setup.bash
    ```

3.  **Add your virtual environment's packages to the `PYTHONPATH`:**
    This step is critical for allowing ROS 2 to find the `mujoco` library.
    ```bash
    export PYTHONPATH="/path/to/your/venv/lib/python3.12/site-packages:$PYTHONPATH"
    ```
    *(Adjust the path if your virtual environment is located elsewhere or you use a different Python version.)*

4.  **Run the node:**
    You can now launch the simulation node using `ros2 run`.

    **Example (with viewer):**
    ```bash
    ros2 run turtlebot4_mujoco tb4_ros2_node --viewer
    ```

    **Example (with a custom scene):**
    ```bash
    ros2 run turtlebot4_mujoco tb4_ros2_node --xml /path/to/your/scene.xml
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

## VSCode Integration

For the best development experience with VSCode, follow these tips:

1.  **Recommended Extensions:**
    -   [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) (Microsoft)
    -   [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) (Microsoft)

2.  **Launch VSCode from a Sourced Terminal:**
    The easiest way to make VSCode aware of your ROS 2 and Python environments is to launch it from a terminal where you have already run all the sourcing commands.

    ```bash
    # In a new terminal, run all sourcing steps
    source /path/to/your/ros2_ws/install/setup.bash
    export PYTHONPATH="/home/narcis/code/venv/lib/python3.12/site-packages:$PYTHONPATH"

    # Launch VSCode from this terminal
    code .
    ```
    This ensures that VSCode's integrated terminal and Python language server inherit the correct environment, providing proper auto-completion and dependency resolution.
