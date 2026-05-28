# Launch the stretch goal: gesture publisher + arm controller + RViz.

# Visualizes a 2-DOF planar arm tracking the hand in RViz. Uses
# robot_state_publisher to broadcast TF from /joint_states, so the
# arm_controller's published joint angles drive the visual model.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gesture_mouse')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_dof_arm.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='gesture_mouse',
            executable='gesture_publisher',
            name='gesture_publisher',
            output='screen',
        ),
        Node(
            package='gesture_mouse',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
