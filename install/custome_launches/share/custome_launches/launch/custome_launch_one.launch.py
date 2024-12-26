from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,TimerAction,ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    start_nav_arg = DeclareLaunchArgument(
        "start_navigation",
        default_value="False"
    )

    start_slam_arg = DeclareLaunchArgument(
        "start_slam",
        default_value="False"
    )

    start_navigation = LaunchConfiguration("start_navigation")
    start_slam = LaunchConfiguration("start_slam")

    delete_entity = ExecuteProcess(
        cmd=["ros2", "service", "call", "/gazebo/delete_entity", "gazebo_msgs/srv/DeleteEntity", "{name: 'patrol_bot'}"],
        output="screen"
    )

    gazebo_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("descriptions_package"),"launch","gazebo.launch.py")),
        launch_arguments={"world_name" : "small_house"}.items()
                            )
                ]       
    )

    # controller_launch = IncludeLaunchDescription(
    #     os.path.join(get_package_share_directory("bot3_controllers"),"launch","bot3_control.launch.py"))

    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("bot3_controllers"),"launch","joystick_teleop_bot3.launch.py")
        )
    )

    amcl_launch = TimerAction(
        period=3.0,
        actions=[
                IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("localization_package"),"launch","global_localization.launch.py")
                                    ),
        condition = IfCondition(start_navigation)
                            )
                ]
    )

    slam_launch = TimerAction(
        period=4.0,
        actions=[
                IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mapping_pkg"),"launch","slam.launch.py")
                                    ),
        condition = IfCondition(start_slam)
                            )
                ]
    
    )

    nav_launch = TimerAction(
        period=5.0,
        actions=[
                IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navigation_package"),"launch","navigation2.launch.py")
                                    ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": os.path.join(get_package_share_directory("navigation_package"), "config", "nav2_params.yaml")
                         }.items(),
        condition = IfCondition(start_navigation)
                                        )
                ]
    )

    return LaunchDescription([
        start_nav_arg,
        start_slam_arg,
        delete_entity,
        gazebo_launch,
        joystick_teleop_launch,
        amcl_launch,
        slam_launch,
        nav_launch
    ])