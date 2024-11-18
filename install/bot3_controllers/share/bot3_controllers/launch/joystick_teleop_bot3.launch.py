from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    joystick_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("bot3_controllers"),"config","joystick_config_bot3.yaml")]
    )

    joystick_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("bot3_controllers"),"config","joystick_teleop_config_bot3.yaml")]
    )

    return LaunchDescription([
        joystick_node,
        joystick_teleop
    ])