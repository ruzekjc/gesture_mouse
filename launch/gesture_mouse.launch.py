# Launch the primary deliverable: gesture publisher + OS mouse controller

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gesture_mouse',
            executable='gesture_publisher',
            name='gesture_publisher',
            output='screen',
        ),
        Node(
            package='gesture_mouse',
            executable='mouse_controller',
            name='mouse_controller',
            output='screen',
        ),
    ])
