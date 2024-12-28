from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():

    navigation_dir = get_package_share_directory("navigation_package")
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False"
    )
    respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description='Whether to respawn if a node crashes.'
    )
    autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(navigation_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='log level'
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_respawn = LaunchConfiguration("use_respawn")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration('log_level')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    lifecycle_nodes=['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True
    )


    nav2_controller = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    nav2_smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    nav2_planner = Node(
        package="nav2_planner",
        executable="planner_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    nav2_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [("cmd_vel","cmd_vel_behaviour")]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    way_point_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav')]
    )


    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_nav",
        output="screen",
        parameters=[
            {"node_names":lifecycle_nodes},
            {"use_sim_time" : use_sim_time},
            {"autostart": autostart}
        ]
    )


    return LaunchDescription([
        stdout_linebuf_envvar,
        use_sim_time_arg,
        respawn_arg,
        autostart_cmd,
        params_file_arg,
        log_level_arg,
        nav2_controller,
        nav2_smoother,
        nav2_planner,
        nav2_behaviour,
        bt_navigator,
        way_point_follower,
        velocity_smoother,
        nav2_lifecycle_manager
        ])