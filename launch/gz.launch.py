import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():
        package_name = 'myslam' 

        # Robot state publisher
        robot_state_publisher = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
                        )]))

        # Gazebo Sim
        world = os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'slam_world.world')    
        gazebo = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                        launch_arguments={'gz_args': ['-r ', world]}.items())

        # Spawn robot
        spawn = Node(
                package='ros_gz_sim', 
                executable='create',
                parameters=[{
                        'name': 'my_robot_sim',
                        'topic': 'robot_description',
                        'z': 0.5,}],
                        output='screen')
        
        # ROS - Gz bridge
        bridge_params = os.path.join(
                                get_package_share_directory(package_name),
                                'config',
                                'gz_bridge.yaml')
        bridge = Node(
                        package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[
                              '--ros-args',
                              '-p',
                              f'config_file:={bridge_params}'],
                              output='screen')

        # ros2 control
        diff_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_controller']
        )

        joint_broadcaster_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broadcaster'],
        )

        return LaunchDescription([
                robot_state_publisher,
                gazebo,
                spawn,
                bridge,
                diff_controller_spawner,
                joint_broadcaster_spawner,
        ])