# DARAM-G 2D Exploration
## 의존성
- ROS2 Humble (Ubuntu 22.04)
- Nav2, slam_toolbox, rtabmap_ros
- m-explore-ros2

## 설명
- 2D LiDAR(와 IMU, optional)에 기반한 자율 탐색을 위한 패키지이다.
- 다음 내용이 포함된다.
  1. `slam_toolbox`, `nav2`, `m-explore-ros2`를 활용한 자율 탐색
  2. `rtabmap_ros`, `nav2`, `m-explore-ros2`를 활용한 자율 탐색 
- 기본적인 사용법은 다음과 같다.
  ```bash
  $ cd /DARAM-G
  $ source install/setup.bash
  $ ros2 launch daramg_2d_exploration sim_slamtoolbox_bringup.launch.py
  ```