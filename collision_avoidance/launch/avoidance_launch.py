from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Standard turtlesim node ───────────────────────────────────────────
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),

        # ── Custom collision avoidance node ───────────────────────────────────
        Node(
            package='collision_avoidance',
            executable='collision_avoidance_node',
            name='collision_avoidance_node',
            output='screen',
            parameters=[{
                'safety_threshold': 2.0,   # Override default (1.5) at launch time
            }],
        ),

    ])