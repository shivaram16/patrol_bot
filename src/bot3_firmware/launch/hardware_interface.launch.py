import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
from ament_index_python.packages import get_package_share_directory 
from launch_ros.actions import Node



def generate_launch_description():

    bot3_description_dir = get_package_share_directory("descriptions_package")

    robot_description = ParameterValue(Command([
    "xacro ", 
    os.path.join(bot3_description_dir,"urdf","bot3.urdf.xacro"),
    "is_sim:=False"]),
    value_type= str
    
    )

    robot_state_publisher = Node(
       package = "robot_state_publisher",
       executable = "robot_state_publisher",
       parameters =  [{"robot_description" : robot_description}]
    )    


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description" : robot_description,
             "use_sim_time" : False},
            os.path.join(get_package_share_directory("bot3_controllers"),"config","bot3_controller.yaml")
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager
    ])