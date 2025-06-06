import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    #
    # 1) 패키지 경로 설정
    #
    pkg_simple_rover = get_package_share_directory('daramg_sim')
    # Nav2 관련 런치 파일이 들어 있는 디렉터리
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    #
    # 2) DeclareLaunchArgument (simple_rover)
    #
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_new_new_world.sdf',
        description='Gazebo에 로드할 world 파일 이름'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='simple_rover.urdf',
        description='스폰할 URDF 파일 이름'
    )
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='5.5',
        description='스폰할 로봇의 initial x 좌표'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='6.5',
        description='스폰할 로봇의 initial y 좌표'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='-1.5707',
        description='스폰할 로봇의 initial yaw 각도'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='시뮬레이션 시간 사용 여부'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='slamtoolbox_nav2.rviz',
        description='RViz config file'
    )

    #
    # 3) DeclareLaunchArgument (Nav2/SLAM 관련)
    #
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='로봇 네임스페이스 (필요 시)'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Nav2를 SLAM 모드로 실행할지 여부'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='turtlebot3_new_new_world.yaml',
        description='(SLAM = False 시) 미리 생성된 맵 파일 경로'
    )

    declare_use_sim_time_nav2_cmd = DeclareLaunchArgument(
        'use_sim_time_nav2',
        default_value='True',
        description='Nav2에서 시뮬레이션 시간을 쓸지 여부'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Nav2 파라미터 파일 전체 경로'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Nav2 자동 시작 여부'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Nav2 컴포지션 방식 사용 여부'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Nav2 노드 크래시 시 재시작 여부'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='RViz2 실행 여부 (Nav2 모니터링용)'
    )

    #
    # 4) Launch Configuration 변수
    #
    namespace = LaunchConfiguration('namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_nav2 = LaunchConfiguration('use_sim_time_nav2')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    world = LaunchConfiguration('world')

    #
    # 5) Gazebo World, Simple Rover 스폰
    #
    # GZ_SIM_RESOURCE_PATH 설정 (모델/월드 검색 경로 추가)
    gazebo_models_path, _ = os.path.split(pkg_simple_rover)
    os.environ["GZ_SIM_RESOURCE_PATH"] = \
        os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path

    # 5.1) Gazebo World Launch (world.launch.py 호출)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simple_rover, 'launch', 'world.launch.py'),
        ),
        launch_arguments={'world': world}.items()
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_simple_rover,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mogi_bot",
            "-topic", "robot_description",
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", "0.2", "-Y", LaunchConfiguration('yaw')  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            #"/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            #"/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            #"/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # 5.4) ros_gz_image_bridge (이미지/깊이 영상 브릿지)
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
            "/camera/depth_image"
        ],
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera.image.compressed.jpeg_quality': 75
        }],
    )

    # 5.5) camera_info 리레이
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # 7.5) Nav2 Bringup (SLAM 모드 포함)
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time_nav2,
            'params_file': PathJoinSubstitution([
                pkg_simple_rover,
                'config',
                'simple_rover.yaml'
           ]),
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'use_keepout_zones': 'False',
            'use_speed_zones': 'False',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_simple_rover, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    #
    # 8) 런치 액션 등록 순서
    #
    # 8.1) simple_rover Args
    ld.add_action(world_arg)
    ld.add_action(model_arg)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(yaw_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)

    # 8.2) Nav2 Args
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_nav2_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # 8.3) 시뮬레이터/로봇 스폰
    ld.add_action(world_launch)
    ld.add_action(spawn_urdf_node)
    ld.add_action(gz_bridge_node)
    ld.add_action(gz_image_bridge_node)
    ld.add_action(relay_camera_info_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    # 8.4) Nav2 SLAM/내비게이션 구동
    # (Gazebo 서버/클라이언트는 이미 simple_rover world 런치에서 실행될 수 있으니,
    #  여기서는 Nav2 관련 노드만 추가로 실행해도 됩니다.)
    ld.add_action(bringup_cmd)

    return ld
