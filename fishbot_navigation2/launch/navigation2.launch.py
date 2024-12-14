import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # obtain and concat the default dir
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    # nav2_bringup is a sample file of the Nav2 that can be used for launching Nav2 quickly
    # because there is a launch file contains in the nav2_bringup, namely 'bringup_launch.py'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # obtain an rviz config under the nav2_bringup dir
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # create launch config
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))
    
    return launch.LaunchDescription([
        # declare the three params defined above
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('nav2_param_path', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        # then pass the parameters of these three declarations to the sub launch file bringup_launch.py
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # replace default params with the three params declared here
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items(),
        ),
        # this is the rviz config defined defore (rviz_config_dir)
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time':use_sim_time}],
            output='screen')                                 
    ])
    