from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Declare arguments
    start_slam_arg = DeclareLaunchArgument(
        "start_slam",
        default_value="False",
        description="Whether to start SLAM"
    )

    start_navigation_arg = DeclareLaunchArgument(
        "start_navigation",
        default_value="False",
        description="Whether to start Navigation"
    )

    # Launch Configuration Variables
    start_slam = LaunchConfiguration("start_slam")
    start_navigation = LaunchConfiguration("start_navigation")

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("descriptions_package"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"world_name": "small_house"}.items(),
    )

    # Joystick Teleop launch
    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bot3_controllers"),
                "launch",
                "joystick_teleop_bot3.launch.py"
            )
        )
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("custome_launches"),
                "rviz_configs",
                "navigation_mode.rviz"
            ),
        ]
    )

    # AMCL Localization launch
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("localization_package"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # SLAM launch (conditioned on `start_slam`)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mapping_pkg"),
                "launch",
                "slam.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(start_slam),
    )

    # Navigation launch (conditioned on `start_navigation`)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("navigation_package"),
                "launch",
                "navigation2.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(start_navigation),
    )

    # Final LaunchDescription
    return LaunchDescription([
        start_slam_arg,
        start_navigation_arg,
        gazebo_launch,
        joystick_teleop_launch,
        rviz_node,
        amcl_launch,
        slam_launch,
        navigation_launch
    ])
