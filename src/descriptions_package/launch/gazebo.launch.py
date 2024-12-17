from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command , LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths

def generate_launch_description():
     
    bot3_description = get_package_share_directory("descriptions_package")
    bot3_description_prefix = get_package_prefix("descriptions_package")

    model_path = os.path.join(bot3_description,"models")
    model_path += pathsep + os.path.join(bot3_description_prefix,"share")
    
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH",model_path)
    

    model_arg = DeclareLaunchArgument(
        name = "patrol_bot",
        default_value= os.path.join(bot3_description,"urdf","bot3.urdf.xacro"),
        description= "complete path for the robot's urdf file"
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(get_package_share_directory("descriptions_package"), "worlds", "small_house.world"),
        description="Path to the custom world file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("patrol_bot")]),value_type= str)

    robot_state_publisher = Node(
       package = "robot_state_publisher",
       executable = "robot_state_publisher",
       output="both",
       parameters =  [{"robot_description" : robot_description},{"use_sim_time": True}]
    )


    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world")}.items()
    )
    start_gazebo_client= IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),"launch","gzclient.launch.py"
    )))    

    spwan_robot = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments=["-entity","patrol_bot","-topic","robot_description"],
        output = "screen"
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        world_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spwan_robot       
     ])