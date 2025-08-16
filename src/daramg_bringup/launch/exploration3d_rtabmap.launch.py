import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    SetEnvironmentVariable, SetLaunchConfiguration
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description() -> LaunchDescription:
    pkg_daramg_2d_exploration = get_package_share_directory('daramg_bringup')
    daramg_exploration_launch_dir = os.path.join(pkg_daramg_2d_exploration, 'launch')

    pkg_daramg_sim = get_package_share_directory('daramg_sim')
    daramg_sim_launch_dir = os.path.join(pkg_daramg_sim, 'launch')

    pkg_explore = get_package_share_directory('turtlebot_exploration_3d')
    explore_launch_dir = os.path.join(pkg_explore, 'launch')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_dir = os.path.join(slam_toolbox_dir, 'launch')

    # 새 토글
    declare_using_sim_cmd = DeclareLaunchArgument(
        'using_sim', default_value='True',
        description='Run with simulation and set use_sim_time=true'
    )

    rviz_launch_arg = DeclareLaunchArgument('rviz', default_value='True', description='Open RViz')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='exploration3d.rviz', description='RViz config file')

    using_sim = LaunchConfiguration('using_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

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

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock if true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_daramg_2d_exploration, 'config', 'rover.yaml'),
        description='Path to the ROS2 parameters file for all nodes',
    )
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack')
    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='True', description='Whether to use composed bringup')
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Whether to respawn a node on crash')
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info', description='Log level')
    declare_graph_file_cmd = DeclareLaunchArgument('graph', default_value='', description='Path to the graph file to load')

    # using_sim에 따라 use_sim_time 강제 세팅 (생성자에서 condition 지정)
    force_sim_time_true = SetLaunchConfiguration('use_sim_time', 'True', condition=IfCondition(using_sim))
    force_sim_time_false = SetLaunchConfiguration('use_sim_time', 'False', condition=UnlessCondition(using_sim))

    bringup_cmd_group = GroupAction([
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

        # using_sim일 때만 스폰
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(daramg_sim_launch_dir, 'spawn_robot.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(using_sim),
        ),

        # SLAM (공용; 필요하면 여기도 조건 분기 가능)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_launch_dir, 'online_sync_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Navigation (공용)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(daramg_exploration_launch_dir, 'navigation.launch.py')),
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

        # Explore (공용)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(explore_launch_dir, 'turtlebot_exploration_3d.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
    
    # If you want to using RViz, Plz uncommnet these
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', PathJoinSubstitution([pkg_daramg_2d_exploration, 'rviz', LaunchConfiguration('rviz_config')])],
       condition=IfCondition(LaunchConfiguration('rviz')),
       parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for cmd in [
        declare_using_sim_cmd,
        rviz_launch_arg,
        rviz_config_arg,
        declare_use_sim_time_cmd,
        declare_graph_file_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        # use_sim_time 강제 오버라이드가 나머지 앞에 오도록 배치
        force_sim_time_true,
        force_sim_time_false,
        rviz_node,
    ]:
        ld.add_action(cmd)

    ld.add_action(bringup_cmd_group)
    return ld
