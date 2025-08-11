# DARAM-G Manipulator
## 의존성
- ros2_control (humble)
- Moveit2 (humble)
- ultralytics (GPU는 각자 탑재된 것 따라서 선택)

## 설명
- DARAM-G 로버에 탑재된 로봇팔의 조작과 관련된 것들을 모아둔 패키지이다.
- 구성은 다음과 같이 되어있다.
  1. YOLO 기반 특정 객체 탐지 (`yolo_node.py`)
  2. YOLO와 Nav2를 이용해 특정 객체로 이동 (`yolo_nav2.py`)
  3. YOLO와 MoveIt2를 이용해 특정 객체에 대한 pick and place 진행 (TBD)
- 해당 코드들은 모두 `daramg_manipulator` 디렉터리 내부에 존재한다.
- YOLO를 위한 가중치는 `src` 디렉터리 내부에 위치한다.

## 사용법
- 기본적으로 다음과 같이 사용한다.
- TBD