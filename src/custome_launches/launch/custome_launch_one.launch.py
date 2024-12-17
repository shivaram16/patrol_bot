from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("descriptions_package"),"launch","gazebo.launch.py"),
        launch_arguments={"world_name" : "small_house"}.items()
    )

    # controller_launch = IncludeLaunchDescription(
    #     os.path.join(get_package_share_directory("bot3_controllers"),"launch","bot3_control.launch.py"))

    joystick_teleop_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bot3_controllers"),"launch","joystick_teleop_bot3.launch.py")
    )

    return LaunchDescription([gazebo_launch,joystick_teleop_launch])