# DARAM-G Sim
## 요구사항 (의존성)
- Ubuntu 22.04 LTS, ROS2 Humble
- Gazebo Harmonic, ros_gz (humble branch)

## 패키지 설명
- 본 메타패키지인 DARAM-G 로버의 소프트웨어를 테스트 하기 위해서 구성된 자체적인 시뮬레이션 패키지이다.
- 내부에 로버와 로봇팔, 그리고 DARAM-G 로버에 탑재된 각종 센서들을 xacro 및 urdf 형식으로 담고 있다.
- Gazebo Sim의 world를 launch 하기 위한 `world.launch.py` 파일,
- URDF에 기반하여 로봇을 world 상에 spawn 하기 위한 `spawn_robot.launch.py` 파일을 담고 있다.
- `world` 디렉터리 안에 있는 파일들의 경우, `Gazebo Fuel` 등에서 적절한 모델링을 다운로드 받아야 사용 가능하다.
- 또한, `GZ_SIM_RESOURCE_PATH` 를 적절하게 설정해주어야 제대로 된 시뮬레이션 작동이 가능하다.
- 만약 `Gazebo Classic` 기반의 시뮬레이션 사용을 원한다면 `daramg_sim_classci` 패키지를 사용하면 된다.
- 다만, 해당 패키지는 작동이 되는지 여부만 최소한으로 검증하였을 뿐, 그 이상의 복잡한 태스크 및 조작은 검증하지 않아 작동을 담보할 수 없다.

## 패키지 사용법
- 기본적인 사용법은 다음과 같다.
```bash
$ cd /path/to/DARAM-G
$ colcon build
$ source install/setup.bash # or source install/local_setup.bash
$ ros2 launch daramg_sim spawn_robot.launch.py
```
- 만약, URDF 파일과 world 파일을 다른 것으로 변경하고 싶다면, `spawn_robot.launch.py` 파일 안의 다음 부분을 수정하고 재빌드하라.
```python
world_arg = DeclareLaunchArgument(
    'world', default_value='warehouse_old.sdf',
    description='Name of the Gazebo world file to load'
)

model_arg = DeclareLaunchArgument(
    'model', default_value='simple_rover.urdf',
    description='Name of the URDF description to load'
)
```