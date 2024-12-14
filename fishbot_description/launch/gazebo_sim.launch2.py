import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_package_path = get_package_share_directory('fishbot_description')
    xacro_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')

    # Run the xacro command and capture the output
    try:
        urdf_content = subprocess.check_output(['xacro', xacro_path]).decode('utf-8')
    except subprocess.CalledProcessError as e:
        print(f"Error generating URDF: {e.output.decode('utf-8')}")
        urdf_content = ''

    # Load URDF content as a string for robot_description
    robot_description = {'robot_description': ParameterValue(urdf_content, value_type=str)}

    action_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    action_launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments={'world': default_gazebo_world_path, 'verbose': 'true'}.items()
    )

    action_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot']
    )

    return LaunchDescription([
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity
    ])
