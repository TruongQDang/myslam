import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
        use_sim_time = LaunchConfiguration('use_sim_time')

        declare_use_sim_time_argument = DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation/Gazebo clock')
        
        start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
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
                        LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                        EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                        ))
                ]
                )
        )

        ld = LaunchDescription()


        ld.add_action(declare_use_sim_time_argument)
        ld.add_action(start_async_slam_toolbox_node)
        ld.add_action(configure_event)
        ld.add_action(activate_event)

        return ld