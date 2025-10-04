# your_package/launch/nav2_local_only.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'daramg_sw'  # ← 본인 패키지명
    default_params = os.path.join(
        get_package_share_directory(pkg), 'config', 'rover.yaml'
    )

    params_file = LaunchConfiguration('params_file', default=default_params)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 local-only params (odom/base_link only)'
        ),

        # ✅ 오직 controller_server만 (내부에 local_costmap을 자체 호스팅)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        # ✅ lifecycle_manager가 controller_server만 관리
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav2',
            output='screen',
            parameters=[params_file]
        ),
    ])
