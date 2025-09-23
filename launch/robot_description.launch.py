from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <-- key import

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    pkg = get_package_share_directory('turtlebot4_mujoco')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'turtlebot4.urdf.xacro'])
    namespace = LaunchConfiguration('namespace')

    # Build the xacro command -> string param
    xacro_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        # optional: '--inorder', ' ',   # enable if your xacro needs it
        'namespace:=', namespace
    ])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        namespace=namespace,  # optional, if you want namespaced rsp
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rsp)
    return ld
