import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'headless': 'True'}.items()
    )

    random_vel_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gesture_mouse',       # <-- change to YOUR package name
                executable='random_vel_publisher',
                name='random_vel_publisher',
                output='screen',
            )
        ]
    )

    pose_publisher_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gesture_mouse',
                executable='pose_publisher',
                name='turtlebot_pose_publisher',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        # force TurtleBot3 model to burger
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        LogInfo(msg='[project.launch] Starting TurtleBot3 in Gazebo...'),
        turtlebot3_gazebo,

        LogInfo(msg='[project.launch] Scheduling velocity + pose nodes (5 s delay)...'),
        random_vel_node,
        pose_publisher_node,
    ])
