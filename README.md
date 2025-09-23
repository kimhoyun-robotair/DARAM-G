# DARAM-G
### Description
- DARAM-G (Dynamic Autonomous Rover for Active Mapping – Geospatial)는 자율적으로 미지의 공간을 탐사하며 지도를 그리는 로버 개발 프로젝트이다.
- 이러한 목적을 가지고 개발된 로버를 DARAM-G Mk.I으로 정의한다.
- DARAM-G 로버는 인하대학교 항공우주공학과 김호윤, 임준형, 황진성, 강인하 넷에 의해서 개발되었으며, 지도교수님은 이창열 교수님께서 담당하셨다.
- DARAM-G Mk.I 프로젝트 기한은 2025.04 ~ 2025.09 이다. 이후 DARAM-G Mk.II와 Mk.III 개발로 전향하였다.
- DARAM-G Mk.I 개발 목표는 **한국항공우주시스템공학회 전국 대학생 캡스톤 경진대회** 및 **인하대학교 종합설계 경진대회** 출전을 위함이다.
- DARAM-G 프로젝트와 관련된 보다 자세한 내용은 `https://www.notion.so/DARAM-G-Mk-I-1e94af2187bc80658ff3eea26052114c?source=copy_link` 참고할 것

### Function
- 2D LiDAR에 기반한 SLAM 및 자율 주행 (Autonomous Navigation), 자율 탐색 (Autonomous Exploration)
- 2D LiDAR에 기반한 SLAM 및 자율 주행, RGB-D 카메라에 기반한 자율 탐색
- RGB-D 카메라와 YOLO에 기반한 객체 인식 및 Object Following
- 로봇팔을 이용한 파지
- 4륜 Rocker 구조를 채택하여 험지 돌파 능력 보유
- 험지 돌파시, 센서 데이터의 안정성을 확보하기 위해 Roll 및 Pitch 짐벌링 기능을 보유한 센서 마스트 도입

### Package (Used)
- `slam_toolbox`, `Cartographer`, `RTAB-MAP`, `Nav2` : SLAM과 자율 주행을 위한 패키지들
- `csm`, `ros2_laser_scan_matcher` : TF Tree에서 Odometry를 LiDAR 기반으로 생성하기 위해 도입한 패키지
- `daramg_bringup` : 필요한 모든 패키지들을 일괄적으로 실행하기 위한 패키지
- `daramg_hw` : ESP32를 ROS2 기반으로 제어하기 위해 작성한 파이썬 코드가 포함된 패키지
- `daramg_msg` : DARAM-G 프로젝트를 위해서 작성된 커스텀 메시지들이 있는 패키지. 지금은 주로 YOLO를 위해서 사용되었다.
- `daramg_sim` : Gazebo Harmonic으로 개발된 DARAM-G의 시뮬레이션. 다양한 센서 및 알고리즘 테스트가 가능하다.
- `daramg_sim_classic` : Gazebo Classic (Gazebo11) 로 개발된 시뮬레이션. 다만 개발만 하고 성능 테스트는 해본 적이 없다.
- `daramg_yolo` : DARAM-G에서 YOLO를 이용한 어플리케이션 코드들이 포함되어 있는 패키지이다.
- `m-explore-ros2` : 2D LiDAR에 기반하여 자율 탐색을 수행하는 패키지이다.
- `turtlebot_exploration_3d` : RGB-D 카메라와 베이지안 최적화, 가우시안 프로세스에 입각해 자율 탐색을 수행하는 패키지이다.

### Result
- DARAM-G Mk.I 로버는 성공적으로 개발되어, 2D LiDAR 및 RGB-D 기반 자율 탐색 기능을 실증하였다.
- 또한, YOLO에 기반한 객체 탐지 능력 역시 실증하였다.
- 하지만 파지 부분에 있어서 로봇팔이 시뮬레이션, 혹은 기획한 바와 다르게 다소 불안정하게 작동하였다.
  - 이에 대한 피드백 및 추가적인 개발을 DARAM-G Mk.II와 Mk.III에서 진행할 예정이다.
- 대회 출전 결과는 다음과 같다.
  - **항공우주시스템공학회 전국 대학생 캡스톤 경진대회** : **한화에어로스페이스 대표이사상** 수상
  - **인하대학교 종합설계 경진대회** : 수상 실패