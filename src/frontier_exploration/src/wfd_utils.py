#!/usr/bin/env python3

from typing import List, Tuple # For type hint
import numpy as np # numpy를 이용해서 occupancy grid map에 대한 벡터 연산 진행
from nav_msgs.msg import OccupancyGrid

# scipy를 통해 Known Cell들과 Frontier Cell에 대한 빠른 검출을 진행
from scipy.ndimage import binary_propagation, binary_dilation, label

UNKNOWN = -1
FREE = 0

_S8 = np.ones((3, 3), dtype=bool)  # Frontier 검출을 위한 8-이웃 검출


def _occ_to_numpy(occ: OccupancyGrid) -> np.ndarray:
    """ ROS2의 occupancy grid ma 데이터를 받아서, map의 크기를 저장
        이후 맵의 크기 기반으로 numpy 배열화를 해서 return """
    H, W = occ.info.height, occ.info.width
    return np.asarray(occ.data, dtype=np.int16).reshape(H, W)


def _world_to_map(occ: OccupancyGrid, wx: float, wy: float) -> Tuple[int, int]:
    """ 실제 세계에서의 좌표값을 map상의 어떤 셀에 해당하는지 변환하는 함수 """
    res = occ.info.resolution
    ox = occ.info.origin.position.x
    oy = occ.info.origin.position.y
    mx = int((wx - ox) / res)
    my = int((wy - oy) / res)
    return mx, my


def _map_to_world(occ: OccupancyGrid, mx: int, my: int) -> Tuple[float, float]:
    """ map상의 한 셀이 실제 세계에서 어떤 점에 해당하는지 변환하는 함수 """
    res = occ.info.resolution
    ox = occ.info.origin.position.x
    oy = occ.info.origin.position.y
    wx = ox + (mx + 0.5) * res
    wy = oy + (my + 0.5) * res
    return wx, wy


def _nearest_free(grid: np.ndarray, my: int, mx: int) -> Tuple[int, int]:
    """(my,mx)에서 가장 가까운 FREE 셀(맵 좌표) 반환. FREE가 하나도 없으면 ValueError."""
    H, W = grid.shape
    my = np.clip(my, 0, H - 1)
    mx = np.clip(mx, 0, W - 1)
    free = (grid == FREE)
    if free[my, mx]:
        return my, mx
    ys, xs = np.nonzero(free)
    if ys.size == 0:
        raise ValueError("No FREE cell exists in grid.")
    # O(N) 최근접 (10^5 셀 수준에서도 충분히 빠름)
    d2 = (ys - my) ** 2 + (xs - mx) ** 2
    i = int(np.argmin(d2))
    return int(ys[i]), int(xs[i])


def detect_frontiers_scipy(
    occ: OccupancyGrid,
    robot_xy_world: Tuple[float, float],
    min_size_cells: int = 5,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[Tuple[float, float]]]:
    """
    SciPy 벡터화만으로 WFD 구현:
    - reachable = binary_propagation(seed, mask=free, structure=8-이웃)
    - frontier  = unknown & binary_dilation(reachable, structure=8-이웃)
    - cluster   = label(frontier, structure=8-이웃)
    - 각 군집 대표 goal: frontier 클러스터 외연의 reachable free 중 seed(로봇)에서 가장 가까운 점
    반환:
      grid(int16), reachable(bool), frontier(bool), goals_world[(x,y), ...]
    """
    grid = _occ_to_numpy(occ)
    H, W = grid.shape

    # 로봇 위치(월드→맵)
    rx, ry = robot_xy_world
    mx, my = _world_to_map(occ, rx, ry)
    mx = np.clip(mx, 0, W - 1)
    my = np.clip(my, 0, H - 1)

    # 시드: 최근접 free
    sy, sx = _nearest_free(grid, my, mx)

    free = (grid == FREE)
    unknown = (grid == UNKNOWN)

    # 도달 가능한 free: 8-이웃 전파
    seed_mask = np.zeros_like(free, dtype=bool)
    seed_mask[sy, sx] = True
    reachable = binary_propagation(seed_mask, mask=free, structure=_S8)

    # frontier: unknown ∧ (8-이웃에 reachable free 존재)
    has_free_nbr = binary_dilation(reachable, structure=_S8)
    frontier = unknown & has_free_nbr

    # 연결 성분 라벨링(8-이웃)
    lbl, n = label(frontier.astype(np.uint8), structure=_S8)

    goals_world: List[Tuple[float, float]] = []
    if n == 0:
        return grid, reachable, frontier, goals_world

    # 각 라벨 별로 대표 goal 산출
    for k in range(1, n + 1):
        mask = (lbl == k)
        size = int(mask.sum())
        if size < min_size_cells:
            continue

        # 군집 외연 dilate & 도달 free와 교집합 → frontier-경계 free
        dil = binary_dilation(mask, structure=_S8)
        border_free = dil & reachable & free

        ys, xs = np.nonzero(border_free)
        if ys.size == 0:
            # 경계 free가 없으면 스킵 (혹은 군집 중심 근처 free를 찾도록 확장 가능)
            continue

        # seed(=로봇 기준)에서 가장 가까운 경계 free 선택
        d2 = (ys - sy) ** 2 + (xs - sx) ** 2
        i = int(np.argmin(d2))
        gy, gx = int(ys[i]), int(xs[i])

        wx, wy = _map_to_world(occ, gx, gy)
        goals_world.append((wx, wy))

    return grid, reachable, frontier, goals_world
