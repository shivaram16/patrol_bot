from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction, DeclareLaunchArgument,GroupAction
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


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
    start_slam = LaunchConfiguration("start_slam")
    start_slam_arg = DeclareLaunchArgument(
        "start_slam",
        default_value = "false"
    )
    start_navigation = LaunchConfiguration("start_navigation")
    start_nav_arg = DeclareLaunchArgument(
        "start_navigation",
        default_value="false"
    )


    twist_mux_launch = TimerAction(
        period=4.0,
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

    twist_relay_node = Node(
        package = "custome_launches",
        executable="twist_twistStamped_transform_node.py",
        output='screen'
    )

    lidar_ros2_driver_node = TimerAction(
        period = 5.0,
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("ydlidar_ros2_driver"),"launch","ydlidar_launch.py")
                ),
                launch_arguments={
                    "params_file": os.path.join(get_package_share_directory("ydlidar_ros2_driver"),"params","X2.yaml")
                }.items()
            )
        ]
    )

    slam_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("mapping_pkg"), "launch", "slam.launch.py")
                ),
                condition=IfCondition(start_slam)
            )
        ]
    )

    nav2_launch_node =TimerAction(
        period = 20.0,
        actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("navigation_package"), "launch","navigation2.launch.py")
            ), condition=IfCondition(start_navigation))
        ]
    )


    return LaunchDescription([
        start_slam_arg,
        start_nav_arg,
        hardware_interface,
        controller,
        # joystick,
        twist_mux_launch,
        twist_relay_node,
        lidar_ros2_driver_node,
        slam_launch,
        nav2_launch_node
    ])