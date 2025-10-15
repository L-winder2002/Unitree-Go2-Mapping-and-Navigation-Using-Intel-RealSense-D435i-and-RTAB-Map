# a_star_planner/a_star.py
# A* on nav_msgs/OccupancyGrid with inflation & unknown-as-free support.
import math
import heapq
from collections import deque
from typing import List, Optional, Tuple

try:
    import numpy as np
except Exception:
    np = None  # 允许没有 numpy；会退到纯 Python 膨胀（较慢）


FREE, OCC, UNKNOWN = 0, 100, -1


def build_occupancy(
    data: List[int],
    width: int,
    height: int,
    occ_threshold: int = 50,
    allow_unknown: bool = True,
) -> List[int]:
    """将 OccupancyGrid 的 data 阈值化为 {0,1} 占据标志的一维数组。"""
    out = [0] * (width * height)
    for i, v in enumerate(data):
        if v < 0:  # 未知
            out[i] = 0 if allow_unknown else 1
        else:
            out[i] = 1 if v >= occ_threshold else 0
    return out


def inflate(
    occ_flat: List[int],
    width: int,
    height: int,
    inflation_cells: int,
) -> List[int]:
    """简单栅格膨胀：以每个障碍为中心，半径 r（曼哈顿/棋盘距离）内设为障碍。"""
    if inflation_cells <= 0:
        return occ_flat

    if np is not None:
        occ = np.array(occ_flat, dtype=np.uint8).reshape((height, width))
        # 用稀疏结构做一次“方形结构元素”膨胀
        from numpy.lib.stride_tricks import as_strided

        r = inflation_cells
        # 构造膨胀核的“滑窗”法（不依赖外部库）
        pad = np.pad(occ, r, mode='edge')
        H, W = occ.shape
        shape = (H, W, 2 * r + 1, 2 * r + 1)
        strides = pad.strides * 2
        windows = as_strided(pad, shape=shape, strides=strides)
        inflated = (windows.max(axis=(2, 3)) > 0).astype(np.uint8)
        return inflated.reshape(-1).tolist()
    else:
        # 纯 Python 版本（慢但通用）
        out = occ_flat[:]
        occ_idx = [i for i, v in enumerate(occ_flat) if v == 1]
        for i in occ_idx:
            cx, cy = i % width, i // width
            for dx in range(-inflation_cells, inflation_cells + 1):
                for dy in range(-inflation_cells, inflation_cells + 1):
                    x, y = cx + dx, cy + dy
                    if 0 <= x < width and 0 <= y < height:
                        out[y * width + x] = 1
        return out


def nearest_free(
    occ_flat: List[int],
    width: int,
    height: int,
    start_xy: Tuple[int, int],
    max_radius: int = 50,
) -> Optional[Tuple[int, int]]:
    """若起/终点落在障碍上，做一个小 BFS 寻找最近可行点。"""
    sx, sy = start_xy
    if not (0 <= sx < width and 0 <= sy < height):
        return None
    if occ_flat[sy * width + sx] == 0:
        return (sx, sy)
    q = deque([(sx, sy)])
    seen = set([(sx, sy)])
    steps = [(-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while q and max_radius > 0:
        for _ in range(len(q)):
            x, y = q.popleft()
            for dx, dy in steps:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in seen:
                    if occ_flat[ny * width + nx] == 0:
                        return (nx, ny)
                    seen.add((nx, ny))
                    q.append((nx, ny))
        max_radius -= 1
    return None


class AStar:
    """8 邻域 A*"""

    def __init__(self, width: int, height: int, occ_flat: List[int]):
        self.w = width
        self.h = height
        self.occ = occ_flat

        # 8-邻域与代价
        self.moves = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
        ]

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.w and 0 <= y < self.h

    def free(self, x: int, y: int) -> bool:
        return self.occ[y * self.w + x] == 0

    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        sx, sy = start
        gx, gy = goal

        # 起终点有效化
        sfix = nearest_free(self.occ, self.w, self.h, (sx, sy))
        gfix = nearest_free(self.occ, self.w, self.h, (gx, gy))
        if sfix is None or gfix is None:
            return None
        sx, sy = sfix
        gx, gy = gfix

        h = lambda x, y: math.hypot(x - gx, y - gy)
        openq: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(openq, (h(sx, sy), (sx, sy)))

        g_cost = { (sx, sy): 0.0 }
        parent = {}

        while openq:
            _, (x, y) = heapq.heappop(openq)
            if (x, y) == (gx, gy):
                break
            gc = g_cost[(x, y)]
            for dx, dy, step in self.moves:
                nx, ny = x + dx, y + dy
                if not self.in_bounds(nx, ny): 
                    continue
                if not self.free(nx, ny): 
                    continue
                ng = gc + step
                key = (nx, ny)
                if ng < g_cost.get(key, 1e18):
                    g_cost[key] = ng
                    parent[key] = (x, y)
                    f = ng + h(nx, ny)
                    heapq.heappush(openq, (f, key))

        if (gx, gy) not in parent and (gx, gy) != (sx, sy):
            return None

        # 回溯路径
        path = [(gx, gy)]
        cur = (gx, gy)
        while cur != (sx, sy):
            cur = parent.get(cur)
            if cur is None:
                break
            path.append(cur)
        path.reverse()
        return path


def world_to_grid(x: float, y: float, info) -> Tuple[int, int]:
    ix = int(math.floor((x - info.origin.position.x) / info.resolution))
    iy = int(math.floor((y - info.origin.position.y) / info.resolution))
    return ix, iy


def grid_to_world(ix: int, iy: int, info) -> Tuple[float, float]:
    x = info.origin.position.x + (ix + 0.5) * info.resolution
    y = info.origin.position.y + (iy + 0.5) * info.resolution
    return x, y


class AStarOnOccGrid:
    """把上面的工具链好用地封装起来。"""

    def __init__(
        self,
        occ_threshold: int = 50,
        inflation_radius_m: float = 0.25,
        allow_unknown: bool = True,
    ):
        self.occ_threshold = occ_threshold
        self.inflation_radius_m = inflation_radius_m
        self.allow_unknown = allow_unknown

    def plan_on_grid(self, occ_grid, start_xy_world, goal_xy_world) -> Optional[List[Tuple[float, float]]]:
        info = occ_grid.info
        w, h = info.width, info.height
        if w == 0 or h == 0:
            return None

        # 1) 阈值化 + 膨胀
        occ = build_occupancy(
            occ_grid.data, w, h,
            occ_threshold=self.occ_threshold,
            allow_unknown=self.allow_unknown
        )
        cells = max(0, int(math.ceil(self.inflation_radius_m / info.resolution)))
        occ = inflate(occ, w, h, cells)

        # 2) A*
        sx, sy = world_to_grid(*start_xy_world, info)
        gx, gy = world_to_grid(*goal_xy_world, info)

        astar = AStar(w, h, occ)
        path_idx = astar.plan((sx, sy), (gx, gy))
        if not path_idx:
            return None

        # 3) 栅格 -> 世界坐标
        path_xy = [grid_to_world(ix, iy, info) for (ix, iy) in path_idx]
        return path_xy
