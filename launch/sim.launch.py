import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
        package_name = 'myslam' 
 
        gazebo = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                                get_package_share_directory(package_name), 'launch', 'gz.launch.py')])
        )
        
        rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'sim_config.rviz')],
                output='screen'
        )

        teleop_keyboard = Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                output='screen',
                prefix = 'xterm -e',
                parameters=[{'stamped': True}],
                remappings=[('/cmd_vel', '/diff_controller/cmd_vel')]
                        
        )

        return LaunchDescription([
                gazebo,
                rviz,
                teleop_keyboard
        ])