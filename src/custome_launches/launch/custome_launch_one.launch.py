from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare arguments
    start_nav_arg = DeclareLaunchArgument(
        "start_navigation",
        default_value="False"
    )

    start_slam_arg = DeclareLaunchArgument(
        "start_slam",
        default_value="False"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True"
    )

    start_navigation = LaunchConfiguration("start_navigation")
    start_slam = LaunchConfiguration("start_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Delete existing entity
    delete_entity = ExecuteProcess(
        cmd=["ros2", "service", "call", "/gazebo/delete_entity", "gazebo_msgs/srv/DeleteEntity", "{name: 'patrol_bot'}"],
        output="screen"
    )

    # Gazebo launch
    gazebo_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("descriptions_package"), "launch", "gazebo.launch.py")
                ),
                launch_arguments={"world_name": "small_house"}.items()
            )
        ]
    )
    
    # Controller launch
    controller_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bot3_controllers"),"launch","bot3_control.launch.py"))
    
    #Twist Transform node
    twist_relay_node = Node(
        package = "custome_launches",
        executable="twist_twistStamped_transform_node.py",
        output='screen'
    )


    # Joystick teleop launch
    joystick_teleop_launch = TimerAction(
        period=6.0,  # Wait for Gazebo to fully initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("bot3_controllers"), "launch", "joystick_teleop_bot3.launch.py")
                )
            )
        ]
    )

    # AMCL launch
    amcl_launch = TimerAction(
        period=9.0,  # Wait for Joystick Teleop to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("localization_package"), "launch", "global_localization.launch.py")
                ),
                condition=IfCondition(start_navigation)
            )
        ]
    )

    # SLAM launch
    slam_launch = TimerAction(
        period=12.0,  # Wait for AMCL (or Navigation if needed) to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("mapping_pkg"), "launch", "slam.launch.py")
                ),
                condition=IfCondition(start_slam)
            )
        ]
    )

    # Navigation launch
    nav_launch = TimerAction(
        period=15.0,  # Wait for SLAM (or AMCL) to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("navigation_package"), "launch", "navigation2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "True",
                    "params_file": os.path.join(get_package_share_directory("navigation_package"), "config", "nav2_params.yaml")
                }.items(),
                condition=IfCondition(start_navigation)
            )
        ]
    )

    # Twist mux launch
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

    return LaunchDescription([
        start_nav_arg,
        start_slam_arg,
        use_sim_time_arg,
        # delete_entity,
        gazebo_launch,
        controller_launch,
        twist_relay_node,
        joystick_teleop_launch,
        amcl_launch,
        slam_launch,
        nav_launch,
        twist_mux_launch
    ])
