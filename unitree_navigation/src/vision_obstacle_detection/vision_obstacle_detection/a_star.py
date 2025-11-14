# a_star_planner/a_star.py
# A* on nav_msgs/OccupancyGrid with inflation, unknown-as-free support,
# diagonal "corner-cutting" prevention, optional path simplification,
# and rotation-aware world<->grid transforms.

import math
import heapq
from collections import deque
from typing import List, Optional, Tuple

try:
    import numpy as np
except Exception:
    np = None  # 允许没有 numpy；会退到纯 Python 实现（较慢）


FREE, OCC, UNKNOWN = 0, 100, -1


def build_occupancy(
    data: List[int],
    width: int,
    height: int,
    occ_threshold: int = 50,
    allow_unknown: bool = True,
) -> List[int]:
    """将 OccupancyGrid.data 阈值化为 {0,1} 占据标志的一维数组（1=障碍，0=可行）"""
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
    """简单栅格膨胀：以每个障碍为中心，方形半径 r 内设为障碍。"""
    if inflation_cells <= 0:
        return occ_flat

    if np is not None:
        occ = np.array(occ_flat, dtype=np.uint8).reshape((height, width))
        from numpy.lib.stride_tricks import as_strided

        r = int(inflation_cells)
        # 边界用常量 0（视为自由空间），避免把边缘值复制出去导致过度膨胀
        pad = np.pad(occ, r, mode='constant', constant_values=0)
        H, W = occ.shape
        shape = (H, W, 2 * r + 1, 2 * r + 1)
        strides = pad.strides * 2
        windows = as_strided(pad, shape=shape, strides=strides)
        inflated = (windows.max(axis=(2, 3)) > 0).astype(np.uint8)
        return inflated.reshape(-1).tolist()
    else:
        # 纯 Python 版本（O(N * r^2)）
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
    """若起/终点落在障碍上，做一个小 BFS 寻找最近可行点（8 邻）。"""
    sx, sy = start_xy
    if not (0 <= sx < width and 0 <= sy < height):
        return None
    if occ_flat[sy * width + sx] == 0:
        return (sx, sy)
    q = deque([(sx, sy)])
    seen = {(sx, sy)}
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
    """8 邻域 A*（阻止斜向穿墙），一致性启发（欧式距离）。"""

    def __init__(self, width: int, height: int, occ_flat: List[int]):
        self.w = width
        self.h = height
        self.occ = occ_flat

        # 8-邻域与代价（单位为“格子步长”）
        rt2 = math.sqrt(2.0)
        self.moves = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, rt2), (1, -1, rt2),
            (-1, 1, rt2), (-1, -1, rt2),
        ]

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.w and 0 <= y < self.h

    def free(self, x: int, y: int) -> bool:
        return self.occ[y * self.w + x] == 0

    def _can_move(self, x: int, y: int, dx: int, dy: int) -> bool:
        """对角移动时阻止“斜穿墙”（要求两个正交邻居都 free）。"""
        nx, ny = x + dx, y + dy
        if not self.in_bounds(nx, ny):
            return False
        if dx != 0 and dy != 0:
            # 严格：两条正交边都可行
            if not (self.free(x + dx, y) and self.free(x, y + dy)):
                return False
        return self.free(nx, ny)

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

        def h(x: int, y: int) -> float:
            return math.hypot(x - gx, y - gy)

        openq: List[Tuple[float, Tuple[int, int]]] = []
        g_cost = {(sx, sy): 0.0}
        parent: dict[Tuple[int, int], Tuple[int, int]] = {}
        closed = set()

        heapq.heappush(openq, (h(sx, sy), (sx, sy)))

        while openq:
            f_cur, (x, y) = heapq.heappop(openq)

            # 迟到条目/过期 f 检查：若不是当前最优，跳过
            if f_cur > g_cost[(x, y)] + h(x, y) + 1e-12:
                continue

            if (x, y) in closed:
                continue
            closed.add((x, y))

            if (x, y) == (gx, gy):
                break

            gc = g_cost[(x, y)]
            for dx, dy, step in self.moves:
                if not self._can_move(x, y, dx, dy):
                    continue
                nx, ny = x + dx, y + dy
                ng = gc + step
                key = (nx, ny)
                if ng < g_cost.get(key, float("inf")):
                    g_cost[key] = ng
                    parent[key] = (x, y)
                    f = ng + h(nx, ny)
                    heapq.heappush(openq, (f, key))

        if (gx, gy) != (sx, sy) and (gx, gy) not in parent:
            return None

        # 回溯路径（严格保证连通）
        path = [(gx, gy)]
        cur = (gx, gy)
        while cur != (sx, sy):
            if cur not in parent:
                return None
            cur = parent[cur]
            path.append(cur)
        path.reverse()
        return path


# ---- 旋转感知的世界<->栅格坐标变换 -----------------------------------------

def _quat_to_yaw(q) -> float:
    """从 geometry_msgs/Quaternion 取 yaw（Z 轴朝上右手系）。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def world_to_grid(x: float, y: float, info) -> Tuple[int, int]:
    """支持 origin 旋转（若 origin.orientation 不为单位朝向）"""
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    q = info.origin.orientation
    yaw = _quat_to_yaw(q)
    if abs(yaw) < 1e-9:  # 无旋转的常见情况
        ix = int(math.floor((x - ox) / res))
        iy = int(math.floor((y - oy) / res))
        return ix, iy
    # 有旋转：把世界坐标先减去平移，再乘 R^T（旋转到地图坐标系）
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = (x - ox), (y - oy)
    rx = c * dx + s * dy     # R^T = [[c, s], [-s, c]]
    ry = -s * dx + c * dy
    ix = int(math.floor(rx / res))
    iy = int(math.floor(ry / res))
    return ix, iy


def grid_to_world(ix: int, iy: int, info) -> Tuple[float, float]:
    """支持 origin 旋转（返回 cell 中心点世界坐标）"""
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    q = info.origin.orientation
    yaw = _quat_to_yaw(q)
    cx = (ix + 0.5) * res
    cy = (iy + 0.5) * res
    if abs(yaw) < 1e-9:
        return ox + cx, oy + cy
    c, s = math.cos(yaw), math.sin(yaw)
    wx = ox + c * cx - s * cy
    wy = oy + s * cx + c * cy
    return wx, wy


# ---- 直线可视化检测与路径稀疏化（可选） --------------------------------------

def _bresenham_line(x0: int, y0: int, x1: int, y1: int):
    """Bresenham 栅格直线生成器（含起止点）。"""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def _line_free(occ_flat: List[int], w: int, h: int, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    """检查 a->b 直线是否全可行（包含端点）。"""
    for x, y in _bresenham_line(a[0], a[1], b[0], b[1]):
        if not (0 <= x < w and 0 <= y < h):
            return False
        if occ_flat[y * w + x] != 0:
            return False
    return True


def _simplify_path_idx(path_idx: List[Tuple[int, int]], occ_flat: List[int], w: int, h: int) -> List[Tuple[int, int]]:
    """用直线可视性把路径稀疏化（string-pulling）。"""
    if not path_idx:
        return path_idx
    simplified = [path_idx[0]]
    anchor = path_idx[0]
    for i in range(2, len(path_idx) + 1):
        # 若 anchor -> path[i-1] 可见，就继续尝试延长；不可见则把 path[i-2] 加入
        if i == len(path_idx) or not _line_free(occ_flat, w, h, anchor, path_idx[i - 1]):
            # 回退一个点（i-2）作为新的 anchor
            new_anchor = path_idx[i - 2]
            if new_anchor != simplified[-1]:
                simplified.append(new_anchor)
            anchor = new_anchor
    if simplified[-1] != path_idx[-1]:
        simplified.append(path_idx[-1])
    return simplified


# ---- 对 OccupancyGrid 的高层封装 ---------------------------------------------

class AStarOnOccGrid:
    """把上面的工具链好用地封装起来。"""

    def __init__(
        self,
        occ_threshold: int = 50,
        inflation_radius_m: float = 0.5,
        allow_unknown: bool = True,
        enable_simplify: bool = True,
    ):
        self.occ_threshold = occ_threshold
        self.inflation_radius_m = inflation_radius_m
        self.allow_unknown = allow_unknown
        self.enable_simplify = enable_simplify

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

        # 3) 可选：路径稀疏化（直线可见性）
        if self.enable_simplify and len(path_idx) >= 3:
            path_idx = _simplify_path_idx(path_idx, occ, w, h)

        # 4) 栅格 -> 世界坐标
        path_xy = [grid_to_world(ix, iy, info) for (ix, iy) in path_idx]
        return path_xy
