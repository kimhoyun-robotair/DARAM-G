#!/usr/bin/env python3
import numpy as np
from dataclasses import dataclass
import time, cv2

@dataclass
class MapMetaData:
    resolution : float
    width : int
    height : int
    origin : tuple

@dataclass
class OccupancyGrid:
    info : MapMetaData
    data : np.ndarray

def generate_random_occupancy_grid(width=200, height=200, resolution=0.05):
    values = np.random.choice([-1, 0, 100], size=(height, width), p=[0.3, 0.6, 0.1])

    info = MapMetaData(
        resolution=resolution,
        width=width,
        height=height,
        origin=(0.0, 0.0, 0.0)
    )

    return OccupancyGrid(info=info, data=values)

def extract_frontiers(map_data):
    start_time = time.time()
    height, width = map_data.shape
    frontier_points = []

    directions = [(-1, -1), (-1, 0), (-1, 1),
                  (0, -1),          (0, 1),
                  (1, -1),  (1, 0), (1, 1)]

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if map_data[y, x] == 0:  # free cell
                for dy, dx in directions:
                    ny, nx = y + dy, x + dx
                    if map_data[ny, nx] == -1:
                        frontier_points.append((y, x))
                        break

    frontier_count = len(frontier_points)
    elapsed_time = time.time() - start_time
    return frontier_count, frontier_points, elapsed_time

def detect_frontiers(occupancy_grid):
    start_time = time.time()
    grid = occupancy_grid.data.astype(np.int16)

    free_mask = (grid == 0).astype(np.uint8)
    unknown_mask = (grid == -1).astype(np.uint8)

    kernel = np.ones((3, 3), np.uint8)
    unknown_dilated = cv2.dilate(unknown_mask, kernel)

    frontier_mask = cv2.bitwise_and(free_mask, unknown_dilated)

    frontier_points = np.column_stack(np.where(frontier_mask > 0))
    elapsed_time = time.time() - start_time

    return frontier_mask * 255, frontier_points, elapsed_time

def compare_frontier_results(points_a, points_b, tolerance=1):
    a = np.array(points_a)
    b = np.array(points_b)

    if len(a) == 0 or len(b) == 0:
        print("⚠️ One of the frontier sets is empty.")
        return 0.0, [], points_a, points_b

    set_a = set(map(tuple, a))
    set_b = set(map(tuple, b))

    if tolerance == 0:
        matched = set_a & set_b
    else:
        matched = set()
        for ya, xa in set_a:
            for yb, xb in set_b:
                if abs(ya - yb) <= tolerance and abs(xa - xb) <= tolerance:
                    matched.add((ya, xa))
                    break

    only_in_a = set_a - matched
    only_in_b = set_b - matched

    match_ratio = (len(matched) / max(len(set_a), len(set_b))) * 100.0
    return match_ratio, list(matched), list(only_in_a), list(only_in_b)

def visualize_frontiers(grid, matched, only_in_a, only_in_b):
    """시각화: 
       - 흰색 (255,255,255): 일치한 frontier
       - 파란색 (255,0,0): For-loop만 탐지
       - 빨간색 (0,0,255): OpenCV만 탐지
       - 회색 (50): unknown
       - 검정 (0): free
       - 초록 (0,255,0): occupied
    """
    h, w = grid.data.shape
    vis = np.zeros((h, w, 3), dtype=np.uint8)

    vis[grid.data == -1] = (50, 50, 50)      # unknown
    vis[grid.data == 0] = (0, 0, 0)          # free
    vis[grid.data == 100] = (0, 255, 0)      # occupied

    for y, x in only_in_a:
        vis[y, x] = (255, 0, 0)  # blue
    for y, x in only_in_b:
        vis[y, x] = (0, 0, 255)  # red
    for y, x in matched:
        vis[y, x] = (255, 255, 255)  # white

    scale = 0.4 if h > 1000 else 1.0
    vis_resized = cv2.resize(vis, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

    cv2.imshow("Frontier Comparison (white=match, blue=for-loop only, red=OpenCV only)", vis_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    grid = generate_random_occupancy_grid(200, 200, 0.05)

    frontier_count_a, frontier_points_a, elapsed_time_a = extract_frontiers(grid.data)
    print(f"[For-loop] Frontier count: {frontier_count_a}")
    print(f"[For-loop] Time: {elapsed_time_a*1000:.2f} ms")

    frontier_mask, frontier_points_b, elapsed_time_b = detect_frontiers(grid)
    print(f"[OpenCV] Frontier count: {len(frontier_points_b)}")
    print(f"[OpenCV] Time: {elapsed_time_b*1000:.2f} ms")

    match_ratio, matched_points, only_in_a, only_in_b = compare_frontier_results(
        frontier_points_a, frontier_points_b, tolerance=0
    )

    print("\n📊 Frontier Comparison Results:")
    print(f" - Matching Points: {len(matched_points)}")
    print(f" - Only in For-loop: {len(only_in_a)}")
    print(f" - Only in OpenCV: {len(only_in_b)}")
    print(f" - Match Ratio: {match_ratio:.2f}%")

    # 시각화 실행
    visualize_frontiers(grid, matched_points, only_in_a, only_in_b)

if __name__=="__main__":
    main()
