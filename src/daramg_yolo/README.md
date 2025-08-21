# DARAM-G YOLO
## Package Description
- DARAM-G 패키지의 `daramg_yolo` 패키지
- DARAM-G 패키지는 로봇팔을 활용해서 로버 전방에 위치한 특정 물체를 파지한다.
- 이를 위한 탐지 수단으로는 RGB-D 카메라 기반의 YOLO를 활용하게 된다.
- YOLO를 통해서 물체를 2차원적으로 인식하며, RGB 이미지와 align된 Depth 데이터를 활용해서 물체를 3차원적으로 인식한다.
- 이후 물체 바로 전방까지 Nav2를 활용하여 이동하며, 이동 이후 충분히 가까워졌다고 판단되면 로봇팔을 통해 파지를 진행한다.
- 로봇팔 파지의 경우 `daramg_hw` 내부에 코드가 작성되어있다.
- 로봇팔의 가동 범위가 크지 않기 때문에, `MoveIt!`과 같은 motion planning은 도입하지 않았다.

## Usage
- use YOLO node
```bash
# use YOLO
$ cd /path/to/DARAM-G
$ source install/setup.bash
$ ros2 run daramg_yolo yolo_node.py
```

- use YOLO based Nav2 moving
```bash
$ cd /path/to/DARAM-G
$ source install/setup.bash
$ ros2 run daramg_yolo yolo_nav2.py
```