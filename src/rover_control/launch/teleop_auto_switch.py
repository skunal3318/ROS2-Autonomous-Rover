from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # -------------------------------
        # TELEOP (keyboard → cmd_vel_teleop)
        # -------------------------------
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel_teleop')
            ],
            prefix='xterm -e'   # IMPORTANT: keeps keyboard input active
        ),

        # -------------------------------
        # OBSTACLE AVOIDANCE (auto logic)
        # publishes → /cmd_vel_auto
        # -------------------------------
        Node(
            package='my_rover_control',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen'
        ),

        # -------------------------------
        # CMD_VEL SWITCH NODE
        # -------------------------------
        Node(
            package='my_rover_control',
            executable='cmd_vel_switch',
            name='cmd_vel_switch',
            output='screen'
        ),
    ])
