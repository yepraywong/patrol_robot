import launch
import launch_ros
# find first_robot.urdf file's directory
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions


def generate_launch_description():
    # find default directory of the .urdf file
    # the code below return the fishbot_description path under the 'share' directory
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'first_robot.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')

    # declare a param for the urdf directory
    # so that we can change the path conveniently if we change a urdf file
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_urdf_path),description='file path of loaded model'
    )
    
    # convert the path to the content of the .urdf file;
    # then convert to a parameter, so as to load into the robot_state_publisher node;
    # in Linux, 'cat' is a command to obtain the context of a file;
    # we use 'DeclareLaunchArgument' to declare launch params, 
    # and use 'LaunchConfiguration' to read launch params's value.
    # 'launch.substitutions.Command' is to use external command.
    # substituted_command_result = launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')])
    substituted_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    # convert 'substituted_command_result' to ROS param value object
    # 为了将其作为 ROS 节点参数传递，必须将其转化为 ParameterValue，这样才能让节点正常接收和使用这个参数
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substituted_command_result, value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
        # 这里的parameters参数是用来向节点传递配置参数的,来获取机器人的 URDF 文件内容
    )

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # launch rviz node
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
        # -d 参数表示要加载一个指定的配置文件
    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node,
    ])