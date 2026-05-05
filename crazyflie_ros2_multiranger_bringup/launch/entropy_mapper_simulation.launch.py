import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py'))
    )

    # Start your new entropy mapper node
    entropy_mapper = Node(
        package='crazyflie_ros2_multiranger_entropy_mapper',
        executable='entropy_mapper',
        name='entropy_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True}
        ]
    )

    # Setup RViz
    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'sim_mapping.rviz')

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                "use_sim_time": True
            }]
            )

    mock_wind_sensor = Node(
        package = 'crazyflie_wind_mapping',
        executable='mock_wind_sensor',
        name = 'mock_wind_sensor',
        output = 'screen',
        parameters=[
            {'use_sim_time':False}
        ]
    )
    
    wind_field_mapper = Node(
        package = 'crazyflie_wind_mapping',
        executable='wind_field_mapper',
        name = 'wind_field_mapper',
        output = 'screen',
        parameters=[
            {'robot_prefix': 'crazyflie_real'},
            {'use_sim_time': False}
        ]
    )
    return LaunchDescription([
        crazyflie_simulation,
        entropy_mapper,
        mock_wind_sensor,
        wind_field_mapper,
        rviz
    ])
