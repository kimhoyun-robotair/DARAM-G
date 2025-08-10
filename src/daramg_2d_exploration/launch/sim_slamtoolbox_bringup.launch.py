import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:
    # custom package for autonomous exploration robot simulation
    pkg_daramg_2d_exploration = get_package_share_directory('daramg_2d_exploration')
    daramg_exploration_launch_dir = os.path.join(pkg_daramg_2d_exploration, 'launch')

    pkg_daramg_sim = get_package_share_directory('daramg_sim')
    daramg_sim_launch_dir = os.path.join(pkg_daramg_sim, 'launch')

    pkg_explore = get_package_share_directory('explore_lite')
    explore_launch_dir = os.path.join(pkg_explore, 'launch')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_dir = os.path.join(slam_toolbox_dir, 'launch')


    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rtabmap_bringup.rviz',
        description='RViz config file'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Remappings for tf
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Combine all parameter rewrites
    param_rewrites = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock if True',
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_daramg_2d_exploration, 'config', 'simple_rover.yaml'),
        description='Path to the ROS2 parameters file for all nodes',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack',
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup',
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn a node on crash',
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level')
    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='', description='Path to the graph file to load'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_daramg_2d_exploration, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Define bringup actions
    bringup_cmd_group = GroupAction([
        # Container for composable nodes
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
        ),
        # SLAM launch unconditionally
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(daramg_sim_launch_dir, 'spawn_robot.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_launch_dir, 'online_sync_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(daramg_exploration_launch_dir, 'sim_navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'graph': LaunchConfiguration('graph'),
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container',
            }.items(),
        ),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(explore_launch_dir, 'explore.launch.py')
        #    ),
        #    launch_arguments={
        #        'use_sim_time': use_sim_time,
        #    }.items(),
        #),
    ])

    # Assemble LaunchDescription
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for cmd in [
        rviz_launch_arg,
        rviz_config_arg,
        declare_use_sim_time_cmd,
        declare_graph_file_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        rviz_node
    ]:
        ld.add_action(cmd)
    ld.add_action(bringup_cmd_group)

    return ld
