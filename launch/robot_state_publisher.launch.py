import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
        package_name = 'myslam'

        # Parse robot description from xacro
        file_subpath = 'description/robot.urdf.xacro'
        xacro_file = os.path.join(
                get_package_share_directory(package_name),
                file_subpath
        )
        robot_description = xacro.process_file(xacro_file).toxml()

        use_sim_time = LaunchConfiguration('use_sim_time')
        params = {'use_sim_time': use_sim_time, 'robot_description': robot_description}
        node_robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name="robot_state_publisher",
                output='screen',
                parameters=[params],
        )

        return LaunchDescription([
                DeclareLaunchArgument(
                        'use_sim_time',
                        default_value='true',
                        description='Use sim time if true'),
                node_robot_state_publisher
        ])