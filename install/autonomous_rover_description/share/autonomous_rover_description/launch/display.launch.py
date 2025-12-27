from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_path('autonomous_rover_description'),
        'urdf',
        'rover.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        get_package_share_path('autonomous_rover_description'),
        'rviz',
        'urdf_config.rviz'
    )
    
    rover_description= ParameterValue(Command(['xacro', urdf_path]), value_type=str)
    
    rover_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'autonomous_rover_description': rover_description}]
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rover_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])