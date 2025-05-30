import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # path to the robot model
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='daramg_sim'
    ).find('daramg_sim')
    default_model_path = os.path.join(pkg_share, 'urdf/simple_rover.urdf')

    # param for using simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Position and orientation for spawning robot in gazebo
    # [X, Y, Z]
    position = [0.0, 0.0, 0.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'racecar',
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', str(orientation[0]),
            '-P', str(orientation[1]),
            '-Y', str(orientation[2]),
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', '-s',
                'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),
        declare_use_sim_time_cmd,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity
    ])