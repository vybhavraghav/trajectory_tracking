from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path to your world file (edit as needed)
    world_path = 'iris_ardupilot.world'

    # Find gazebo_ros package
    gazebo_ros_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        # Launch Gazebo Classic with your world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'world': world_path}.items(),
        ),

        # Launch MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://:14550@'},
                {'frame_id': 'base_link'},
                # {'config_yaml': os.path.join(FindPackageShare('mavros').find('mavros'), 'launch', 'iris_ardupilot.yaml')},
            ]
        )
    ])
    
