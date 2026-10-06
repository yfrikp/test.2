from __future__ import annotations
import heapq
import math
from typing import List, Optional, Sequence, Tuple
import numpy as np
try:
    from .world_map import GridMap, min_clearance
except ImportError:
    from world_map import GridMap, min_clearance
Point = Tuple[float, float]

def astar(grid: GridMap, start: Point, goal: Point, allow_diagonal: bool=True, heuristic_weight: float=1.0) -> Optional[List[Point]]:
    sc = grid.to_grid(*start)
    gc = grid.to_grid(*goal)
    if not grid.is_free(*sc):
        sc = _nearest_free(grid, sc)
        if sc is None:
            return None
    if not grid.is_free(*gc):
        gc = _nearest_free(grid, gc)
        if gc is None:
            return None
    if allow_diagonal:
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    else:
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

    def h(c: Tuple[int, int]) -> float:
        dx = abs(c[0] - gc[0])
        dy = abs(c[1] - gc[1])
        if allow_diagonal:
            return dx + dy + (math.sqrt(2) - 2.0) * min(dx, dy)
        return float(dx + dy)
    open_heap: List[Tuple[float, int, Tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (h(sc) * heuristic_weight, counter, sc))
    came_from: dict = {}
    g_score: dict = {sc: 0.0}
    closed: set = set()
    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == gc:
            return _reconstruct(grid, came_from, cur)
        for dx, dy, cost in moves:
            nb = (cur[0] + dx, cur[1] + dy)
            if not grid.is_free(*nb):
                continue
            if dx != 0 and dy != 0:
                if not grid.is_free(cur[0] + dx, cur[1]):
                    continue
                if not grid.is_free(cur[0], cur[1] + dy):
                    continue
            ng = g_score[cur] + cost
            if ng < g_score.get(nb, float('inf')):
                g_score[nb] = ng
                came_from[nb] = cur
                counter += 1
                heapq.heappush(open_heap, (ng + h(nb) * heuristic_weight, counter, nb))
    return None

def _nearest_free(grid: GridMap, cell: Tuple[int, int], max_r: int=40):
    if grid.is_free(*cell):
        return cell
    cx, cy = cell
    for r in range(1, max_r + 1):
        for dx in range(-r, r + 1):
            for dy in (-r, r):
                for c in ((cx + dx, cy + dy), (cx + dy, cy + dx)):
                    if grid.is_free(*c):
                        return c
    return None

def _reconstruct(grid: GridMap, came_from: dict, cur: Tuple[int, int]) -> List[Point]:
    cells = [cur]
    while cur in came_from:
        cur = came_from[cur]
        cells.append(cur)
    cells.reverse()
    return [grid.cell_center(c[0], c[1]) for c in cells]

def simplify_collinear(path: Sequence[Point], tol: float=1e-06) -> List[Point]:
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    for i in range(1, len(path) - 1):
        p0, p1, p2 = (np.array(path[i - 1]), np.array(path[i]), np.array(path[i + 1]))
        v1 = p1 - p0
        v2 = p2 - p1
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < tol or n2 < tol:
            continue
        cross = abs(v1[0] * v2[1] - v1[1] * v2[0]) / (n1 * n2)
        if cross > 0.001:
            out.append(path[i])
    out.append(path[-1])
    return out

def catmull_rom(points: Sequence[Point], samples_per_seg: int=20, alpha: float=0.5) -> List[Point]:
    pts = [np.asarray(p, dtype=np.float64) for p in points]
    if len(pts) < 2:
        return [tuple(p) for p in pts]
    if len(pts) == 2:
        t = np.linspace(0, 1, samples_per_seg).reshape(-1, 1)
        seg = pts[0] + (pts[1] - pts[0]) * t
        return [tuple(p) for p in seg]
    ext = [pts[0] + (pts[0] - pts[1])] + pts + [pts[-1] + (pts[-1] - pts[-2])]
    out: List[Point] = []
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = (ext[i - 1], ext[i], ext[i + 1], ext[i + 2])

        def tj(ti: float, pa: np.ndarray, pb: np.ndarray) -> float:
            d = np.linalg.norm(pb - pa)
            return ti + (d ** alpha if d > 1e-09 else 1e-09)
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        if t1 == t0 or t2 == t1 or t3 == t2:
            continue
        ts = np.linspace(t1, t2, samples_per_seg, endpoint=i == len(ext) - 3)
        for t in ts:
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            c = (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2
            out.append((float(c[0]), float(c[1])))
    if not out:
        return list(points)
    out[0] = (float(pts[0][0]), float(pts[0][1]))
    out[-1] = (float(pts[-1][0]), float(pts[-1][1]))
    return out

def shorten_from_obstacles(obstacles: Sequence, path: Sequence[Point], required_clearance: float) -> List[Point]:
    if not obstacles:
        return (list(path), 0)
    keep: List[Point] = []
    removed = 0
    for p in path:
        if min_clearance(obstacles, p[0], p[1]) >= required_clearance:
            keep.append(p)
        else:
            removed += 1
    if len(keep) < 2:
        return (list(path), removed)
    return (keep, removed)

def path_length(path: Sequence[Point]) -> float:
    if len(path) < 2:
        return 0.0
    a = np.asarray(path, dtype=np.float64)
    return float(np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1)))

def max_curvature(path: Sequence[Point]) -> float:
    a = np.asarray(path, dtype=np.float64)
    if len(a) < 3:
        return 0.0
    k = 0.0
    for i in range(1, len(a) - 1):
        p0, p1, p2 = (a[i - 1], a[i], a[i + 1])
        v1 = p1 - p0
        v2 = p2 - p1
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-09 or n2 < 1e-09:
            continue
        cosang = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
        dtheta = math.acos(cosang)
        ds = 0.5 * (n1 + n2)
        k = max(k, dtheta / max(ds, 1e-09))
    return k
