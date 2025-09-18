# bringup_tb4_mujoco.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # --- Launch args ---
    # Path to your venv site-packages to prepend for the custom node
    pythonpath_arg = DeclareLaunchArgument(
        "custom_pythonpath",
        default_value="/home/narcis/code/venv/lib/python3.12/site-packages",
        description="Prepended to PYTHONPATH for turtlebot4_mujoco node only."
    )

    # Optional RViz config
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="",
        description="Path to an RViz config (.rviz/.rviz2). Leave empty for default RViz."
    )

    # Optional: start an extra robot_state_publisher (usually NOT needed because
    # turtlebot4_description.launch.py already starts it)
    launch_rsp_arg = DeclareLaunchArgument(
        "launch_rsp",
        default_value="true",
        description="Set true only if your description launch does NOT start robot_state_publisher."
    )

    # --- Include TurtleBot4 description launch ---
    tb4_description_share = get_package_share_directory("turtlebot4_description")
    include_tb4_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            tb4_description_share + "/launch/turtlebot4_description.launch.py"
        )
    )

    # --- Optional standalone robot_state_publisher ---
    # Enable only if launch_rsp:=true
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        # NOTE: We are NOT passing robot_description here because the TB4 description
        # launch usually handles it. Enable this node only if that launch does not.
        condition=lambda context: LaunchConfiguration("launch_rsp").perform(context) == "true"
    )

    # --- Custom turtlebot4_mujoco node with per-process PYTHONPATH ---
    # This prepends the provided venv path to whatever PYTHONPATH the shell already has.
    mujoco_node = Node(
        package="turtlebot4_mujoco",
        executable="tb4_ros2_node",
        name="tb4_ros2_node",
        output="screen",
        arguments=["--viewer"],
        env={
            "PYTHONPATH": [
                LaunchConfiguration("custom_pythonpath"),
                ":",
                EnvironmentVariable("PYTHONPATH")
            ]
        }
    )

    # --- RViz2 ---
    rviz_args = []
    # Only add -d if a config path was provided
    def _rviz_args_ctx(ctx):
        cfg = LaunchConfiguration("rviz_config").perform(ctx)
        return ["-d", cfg] if cfg else []

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=_rviz_args_ctx
    )

    return LaunchDescription([
        pythonpath_arg,
        rviz_config_arg,
        launch_rsp_arg,
        include_tb4_description,
        rsp_node,
        mujoco_node,
        rviz,
    ])
