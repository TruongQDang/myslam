import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import (LaunchConfiguration)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
        use_sim_time = LaunchConfiguration('use_sim_time')
        slam_params_file = LaunchConfiguration('slam_params_file')

        declare_use_sim_time_argument = DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation/Gazebo clock')
        
        declare_slam_params_file_cmd = DeclareLaunchArgument(
                'slam_params_file',
                default_value=os.path.join(get_package_share_directory("myslam"),
                                   'config', 'myslam_params.yaml'),
                description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
        
        start_async_slam_toolbox_node = LifecycleNode(
                parameters=[
                        slam_params_file,
                        {
                        'use_sim_time': use_sim_time
                        }
                        ],
                package='myslam',
                executable='myslam_node',
                name='myslam',
                output='screen',
                namespace=''
                )

        configure_event = EmitEvent(
                event=ChangeState(
                lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                transition_id=Transition.TRANSITION_CONFIGURE
                )
        )

        activate_event = RegisterEventHandler(
                OnStateTransition(
                target_lifecycle_node=start_async_slam_toolbox_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                        LogInfo(msg="[LifecycleLaunch] MySlam node is activating."),
                        EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                        ))
                ]
                )
        )

        ld = LaunchDescription()


        ld.add_action(declare_use_sim_time_argument)
        ld.add_action(declare_slam_params_file_cmd)
        ld.add_action(start_async_slam_toolbox_node)
        ld.add_action(configure_event)
        ld.add_action(activate_event)

        return ld