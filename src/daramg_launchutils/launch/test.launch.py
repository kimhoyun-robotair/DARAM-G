# rtabmap_lidar_imu_rgbd_viz.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---- Args ----
    args = [
        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),

        DeclareLaunchArgument('scan_topic', default_value='/scan'),  # sensor_msgs/LaserScan
        DeclareLaunchArgument('imu_topic', default_value='/imu'),

        # RGB-D 카메라 토픽
        DeclareLaunchArgument('rgb_topic', default_value='/mast_camera/image'),
        DeclareLaunchArgument('depth_topic', default_value='/mast_camera/depth_image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/mast_camera/camera_info'),

        # 동기화/시간
        DeclareLaunchArgument('approx_sync', default_value='true'),
        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.02'),
        DeclareLaunchArgument('sync_queue_size', default_value='30'),
        DeclareLaunchArgument('topic_queue_size', default_value='10'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # rtabmap에도 scan을 공급해서 근접/이웃 링크 보정에 사용
        DeclareLaunchArgument('use_scan_in_rtabmap', default_value='true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true'),
    ]

    # ---- rgbd_sync: RGB+Depth(+CameraInfo) -> /rgbd_image ----
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': LaunchConfiguration('approx_sync'),
            'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
            'topic_queue_size': LaunchConfiguration('topic_queue_size'),
            'sync_queue_size': LaunchConfiguration('sync_queue_size'),  # queue_size 리네임 반영
            'qos': 2,               # SensorData QoS
            'qos_camera_info': 2,
            'depth_scale': 1.0,
            'decimation': 1
        }],
        remappings=[
            ('rgb/image',       LaunchConfiguration('rgb_topic')),
            ('depth/image',     LaunchConfiguration('depth_topic')),
            ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
            ('rgbd_image',      'rgbd_image'),  # output
        ],
    )

    # ---- icp_odometry: 2D LiDAR(+IMU) 오도메트리 ----
    icp_odom = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id':       LaunchConfiguration('frame_id'),
            'odom_frame_id':  LaunchConfiguration('odom_frame_id'),
            'use_sim_time':   LaunchConfiguration('use_sim_time'),

            'wait_imu_to_init': True,   # IMU가 라이다보다 같거나 빠른 주파수 권장
            'always_check_imu_tf': True,

            # 2D 주행 가정
            'Reg/Force3DoF': 'true',

            # ICP 튜닝(필요 시 조정)
            'Icp/MaxCorrespondenceDistance': '0.10',
            'scan_voxel_size': 0.05,    # 로그 안내대로 ros 파라미터로 사용
        }],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('imu',  LaunchConfiguration('imu_topic')),
        ],
    )

    # ---- rtabmap(슬램/맵퍼): 외부 odom + RGB-D 맵 복원 ----
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id':     LaunchConfiguration('frame_id'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'odom_frame_id':LaunchConfiguration('odom_frame_id'),

            # 입력 구성
            'subscribe_rgbd': True,
            'subscribe_scan': LaunchConfiguration('use_scan_in_rtabmap'),
            'approx_sync':    LaunchConfiguration('approx_sync'),
            'sync_queue_size':LaunchConfiguration('sync_queue_size'),

            # 시각 특징 배제 + ICP 고정(로컬라이제이션은 LiDAR 전용)
            'Kp/MaxFeatures': '-1',   # 음수 → 특징 추출 안 함
            'Reg/Strategy':   '1',    # 1 = ICP
            'Reg/Force3DoF':  'true',
            'RGBD/ForceOdom3DoF': 'true',

            # LiDAR 기반 근접/이웃 링크 정제(선택)
            'RGBD/ProximityBySpace':    'true',
            'RGBD/NeighborLinkRefining':'true',

            # 2D grid는 라이다로, 3D 복원은 RGB-D로
            'Grid/FromDepth': 'false',

            # 로그 레벨(정수형으로!)
            'log_to_rosout_level': 2,
        }],
        remappings=[
            ('odom',       '/odom'),
            ('rgbd_image', 'rgbd_image'),
            ('scan',       LaunchConfiguration('scan_topic')),
        ],
        arguments=['--delete_db_on_start'],
    )

    # ---- rtabmap_viz(뷰어) ----
    #  - rtabmap이 퍼블리시하는 맵/그래프를 시각화
    #  - 동일한 rgbd/scan/odom 구독(디버깅/오버레이용)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id':     LaunchConfiguration('frame_id'),
            'odom_frame_id':LaunchConfiguration('odom_frame_id'),
            'subscribe_rgbd': True,
            'subscribe_scan': True,
            'approx_sync':    LaunchConfiguration('approx_sync'),
            'sync_queue_size':LaunchConfiguration('sync_queue_size'),
            # 뷰어 로그(선택): 'log_to_rosout_level': 2,
        }],
        remappings=[
            ('odom',       '/odom'),
            ('rgbd_image', 'rgbd_image'),
            ('scan',       LaunchConfiguration('scan_topic')),
            # rtabmap가 퍼블리시하는 /mapData, /mapGraph, /cloud_map, /grid_map 등은 기본 이름 사용
        ],
    )

    return LaunchDescription(args + [rgbd_sync, icp_odom, rtabmap, rtabmap_viz])
