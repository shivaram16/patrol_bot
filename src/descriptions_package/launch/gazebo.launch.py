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

    model_path,plugin_path,media_path=GazeboRosPaths.get_paths()
     
    bot3_description = get_package_share_directory("descriptions_package")
    bot3_description_prefix = get_package_prefix("descriptions_package")
    model_path = os.path.join(bot3_description,"models","models")
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH",model_path)
    model_path += pathsep + os.path.join(bot3_description_prefix,"share")

    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value= os.path.join(bot3_description,"urdf","bot3.urdf.xacro"),
        description= "complete path for the robot's urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),value_type= str)

    robot_state_publisher = Node(
       package = "robot_state_publisher",
       executable = "robot_state_publisher",
       output="both",
       parameters =  [{"robot_description" : robot_description},{"use_sim_time": True}]
    )

    start_gazebo_server= IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),"launch","gzserver.launch.py"
    )))
    start_gazebo_client= IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),"launch","gzclient.launch.py"
    )))    

#     gazebo_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
#   )

    spwan_robot = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments=["-entity","bot3","-topic","robot_description"],
        output = "screen"
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        # gazebo_node,
        spwan_robot       
     ])