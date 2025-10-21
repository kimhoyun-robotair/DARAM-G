import cv2
from math import isfinite
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List, Iterable, Dict
import sys

""""""""""""""""""" P and PD Controller Utilities """""""""""""""""""""""""
def clamp(x: float, lo: float, hi: float) -> float:
    """ 출력값을 lo와 hi 사이로 제한 """
    return lo if x < lo else hi if x > hi else x

@dataclass
class PController:
    kp : float # P gain
    out_min : float = float(-sys.maxsize) # 출력 최소값
    out_max : float = float(sys.maxsize) # 출력 최댓값

    def reset(self) -> None:
        pass

    def step(self, error: float) -> float:
        """ P 제어기 출력 계산"""
        return clamp(self.kp * error, self.out_min, self.out_max)
    
""""""""""""""""""""""""" YOLO Based Object Approaching Utilites """""""""""""""""""""""""
@dataclass
class YoloConfig:
    image_width : int = 640
    image_height : int = 480
    fov_deg : float = 60.0 # 카메라 시야각
    kp_ang : float = 0.002 # 각속도 P gain
    kp_lin : float = 0.5 # 선속도 P gain
    depth_stop_threshold : float = 0.1 # 로봇이 안전하게 객체 앞에 멈추는 거리
    stop_margin : float = 0.05 # 감속 구간 시작점
    lin_max : float = 0.3 # 최대 선속도 (m/s)
    ang_max : float = 0.5 # 각속도 상한 (rad/s)

@dataclass
class DetectionLite:
    depth : float # 카메라와 물체 사이 거리 (m)
    cx : float # 이미지의 x 좌표

def compute_cmd_vel_from_YOLO(detections: Iterable[DetectionLite], 
                              config: YoloConfig) -> Tuple[float, float, Dict[str, float]]:
    """ 객체 탐지 결과를 바탕으로 선속도 및 각속도 계산 """
    dets : List[DetectionLite] = [det for det in detections if det.depth is not None]
    info = {"reason": "",
            "selected_depth": None,
            "angle_error_deg": None}
    if not dets:
        info["reason"] = "No Detections"
        return 0.0, 0.0, info
    
    valid = [d for d in dets if d.depth > 0.0 and isfinite(d.depth)]
    if not valid:
        info["reason"] = "No Valid Depth"
        return 0.0, 0.0, info
    
    # 가장 가까운 물체 선택
    target = min(valid, key=lambda d: d.depth)
    depth = float(target.depth)
    cx = float(target.cx)

    # 이미지 중심 대비 픽셀 오프셋 및 각도 오차 산출
    offset_x = cx - (config.image_width / 2.0)
    angle_error_deg = (offset_x / config.image_width) * config.fov_deg

    # 각도 오차 및 픽셀 오차 기반 각속도 계산
    ang_z = -config.kp_ang * angle_error_deg

    if 0.0 < depth < config.depth_stop_threshold:
        # 정지 거리 내에 진입할 경우 완전 정지
        info.update(reason="inside stop threshold", selected_depth=depth, angle_erro_deg=angle_error_deg)
        return 0.0, 0.0, info
    
    lin_x = config.kp_lin * depth
    if depth <= config.depth_stop_threshold + config.stop_margin:
        # 감속 구간 진입시 선속도 절반으로 감속
        lin_x *= 0.5
    # 최대 속도 제한
    lin_x = clamp(lin_x, 0.0, config.lin_max)
    ang_z = clamp(ang_z, -config.ang_max, config.ang_max)
    
    info.update(reason="OK", selected_depth=depth, angle_erro_deg=angle_error_deg)
    return lin_x, ang_z, info
    
""""""""""""""""""""""""" Line Detection Based Object Approaching Utilites """""""""""""""""""""""""
@dataclass
class LineConfig:
    deadband_px : int = 20 # 허용 가능한 오차
    v_turn : float = 0.05 # 회전하며 직진할 때 사용하는 조향용 선속도, m/s
    v_straight : float = 0.1 # 전진할 때 사용하는 선속도, m/s
    w_turn : float = 0.15 # 각속도, rad/s
    lin_max : float = 0.3 # 선속도 최대 제한
    ang_max : float = 0.5 # 각속도 최대 제한

@dataclass
class LineObservation:
    cx : float
    width : int 
    found : bool = True # 라인 검출 여부

def compute_cmd_vel_from_line(obs: Optional[LineObservation],
                              config : LineConfig) -> Tuple[float, float, Dict[str, float]]:
    """ 라인 탐지 결과를 바탕으로 이를 추종하기 위한 선속도 및 각속도 계산 및 반환 """
    info: Dict[str, float] = {"mode": "stop", "error_px": None}

    if obs is None or not obs.found:
        return 0.0, 0.0, info
    
    cx = float(obs.cs)
    width = int(obs.width)
    center = width / 2.0

    error_px = center - cx # 왼쪽이면 +, 오른쪽이면 -
    info["error_px"] = error_px

    if abs(error_px) > config.deadband_px:
        # Turn
        lin_x = config.v_turn
        ang_z = (config.w_turn if error_px > 0 else -config.w_turn)
        info["mode"] = "turn_left" if error_px > 0 else "turn_right"
    else:
        # Moving Straight
        lin_x = config.v_straight
        ang_z = 0.0
        info["mode"] = "Straight"
    
    # 안전을 위한 상한 필터링
    lin_x = clamp(lin_x, 0.0, config.lin_max)
    ang_z = clamp(ang_z, -config.ang_max, config.ang_max)
    return lin_x, ang_z
    
