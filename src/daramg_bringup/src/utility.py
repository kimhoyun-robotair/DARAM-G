from math import isfinite
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List, Iterable, Dict
import sys, math
from yolo_msgs.msg import DetectionArray
from detection_msgs.msg import LineDetection

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
    detections : DetectionArray

def compute_cmd_vel_from_YOLO(detections : DetectionArray, 
                              config : YoloConfig) -> Tuple[float, float, Dict[str, float]]:
    """ 객체 탐지 결과를 바탕으로 선속도 및 각속도 계산 """
    if hasattr(detections, "detections"):
        detections = detections.detections

    dets = [det for det in detections if det.depth is not None]
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
    cx = float(target.bbox.center.position.x)

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
@dataclass
class LineConfig:
    # 기존 필드 유지
    deadband_px : int = 20
    v_turn : float = 0.05
    v_straight : float = 0.10
    w_turn : float = 0.15      # (하위호환용) 안 씀. 남겨둠.
    lin_max : float = 0.30
    ang_max : float = 0.50
    # 추가 파라미터
    k_p : float = 1.6          # 비례 게인 (정규화 오차에 곱)
    v_min : float = 0.02       # 너무 많이 감쇠되어 정지하지 않도록 하한
    speed_decay : float = 0.85 # |e|에 따른 속도 감쇠 세기(0.5~1.5 권장)
    soft_db_ratio : float = 0.35  # 소프트 데드밴드 강도(0=하드, 1=매우 부드럽게)

def compute_cmd_vel_from_line(obs, config: LineConfig) -> Tuple[float, float, Dict[str, float]]:
    """ 라인 탐지 결과를 바탕으로 선속도/각속도 산출 (stateless, 연속형 P, 소프트 데드밴드, 속도 감쇠) """
    info: Dict[str, float] = {"mode": "stop", "error_px": None, "error_norm": None}

    if obs is None or not obs.found:
        return 0.0, 0.0, info

    cx = float(obs.cx)
    width = int(obs.width)
    center = width / 2.0

    # 1) 픽셀 오차 및 정규화 (좌 +, 우 -)
    error_px = center - cx
    error_norm = error_px / (0.5 * width + 1e-6)   # [-1, 1] 정도 범위
    error_norm = clamp(error_norm, -1.0, 1.0)
    info["error_px"] = float(error_px)
    info["error_norm"] = float(error_norm)

    # 2) 소프트 데드밴드: 작은 오차일수록 더 눌러서 0 근처 유지
    #    hard 데드밴드 대신, 연속적으로 감쇠하는 방식(계단/진동 완화)
    #    예: db_strength=0.35 => 작은 오차일수록 더 완만
    db = max(config.deadband_px / (0.5 * width + 1e-6), 0.0)  # 픽셀 데드밴드를 정규화해 반영
    db = clamp(db, 0.0, 0.5)  # 과도한 데드밴드 방지
    # 소프트닝: 선형-쵸핑 대신 스무드 스텝
    def soft_deadband(x: float, db_lin: float, strength: float) -> float:
        # db_lin: 0~0.5, strength: 0(하드)~1(매우 소프트)
        ax = abs(x)
        if ax <= db_lin:
            # 안쪽에서는 천천히 0으로 끌어감
            s = (ax / (db_lin + 1e-6))  # 0~1
            scaled = s**(1.0 + 4.0*strength)  # 곡률 조절
            return math.copysign(db_lin * scaled, x)  # 완만하게 증가
        else:
            # 바깥에서는 원래 값으로
            return x

    e_soft = soft_deadband(error_norm, db, config.soft_db_ratio)

    # 3) 각속도: 연속형 P
    ang_z_raw = config.k_p * e_soft
    # 하드 클램프
    ang_z = clamp(ang_z_raw, -config.ang_max, config.ang_max)

    # 4) 선속도: 오차가 클수록 자동 감쇠 (턴 시 과속 방지)
    #    v = v_straight / (1 + speed_decay*|e|)
    v_raw = config.v_straight / (1.0 + config.speed_decay * abs(e_soft))
    # 턴 중 최소 전진 속도 보장
    v_raw = max(v_raw, config.v_min if abs(e_soft) > db else config.v_straight)
    lin_x = clamp(v_raw, 0.0, config.lin_max)

    # 5) 모드 라벨
    if abs(error_px) <= config.deadband_px:
        info["mode"] = "Straight(soft)"
    else:
        info["mode"] = "turn_left" if error_px > 0 else "turn_right"

    return float(lin_x), float(ang_z), info
""""""""""""""""""""""""" Publish Twist Function """""""""""""""""""""""""
def moving_forward():
    lin_x : float = 0.1
    ang_z : float = 0.0
    return lin_x, ang_z

def turning_standby():
    lin_x : float = 0.0
    ang_z : float = 0.05
    return lin_x, ang_z

def stop_robot():
    lin_x : float = 0.0
    ang_z : float = 0.0
    return lin_x, ang_z