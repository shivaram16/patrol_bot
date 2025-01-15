from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bot3_firmware"),"launch","hardware_interface.launch.py")
    )

    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bot3_controllers"),"launch","bot3_control.launch.py")
    )

    joystick = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bot3_controllers"),"launch","joystick_teleop_bot3.launch.py")
    )

    twist_mux_launch = TimerAction(
        period=5.0,  # Wait for Navigation to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("twist_mux"), "launch", "twist_mux_launch.py")
                ),
                launch_arguments={
                    "cmd_vel_out": "mux_twist_out",
                    "config_topics": os.path.join(get_package_share_directory("custome_launches"), "config", "twist_mux_topics.yaml"),
                    "config_locks": os.path.join(get_package_share_directory("custome_launches"), "config", "twist_mux_locks.yaml"),
                    "config_joy": os.path.join(get_package_share_directory("custome_launches"), "config", "twist_mux_joy.yaml"),
                    "use_sim_time": "False"
                }.items()
            )
        ]
    )

        #Twist Transform node
    twist_relay_node = Node(
        package = "custome_launches",
        executable="twist_twistStamped_transform_node.py",
        output='screen'
    )

    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        twist_mux_launch,
        twist_relay_node
    ])