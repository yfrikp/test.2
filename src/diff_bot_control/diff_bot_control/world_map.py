from __future__ import annotations
import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Sequence, Tuple
import numpy as np

@dataclass
class Circle:
    cx: float
    cy: float
    r: float
    name: str = ''

    def distance(self, x: float, y: float) -> float:
        return math.hypot(x - self.cx, y - self.cy) - self.r

@dataclass
class RotRect:
    cx: float
    cy: float
    hx: float
    hy: float
    yaw: float
    name: str = ''

    def distance(self, x: float, y: float) -> float:
        dx = x - self.cx
        dy = y - self.cy
        c, s = (math.cos(-self.yaw), math.sin(-self.yaw))
        px = dx * c - dy * s
        py = dx * s + dy * c
        qx = abs(px) - self.hx
        qy = abs(py) - self.hy
        outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
        inside = min(max(qx, qy), 0.0)
        return outside + inside
Obstacle = object

class GridMap:

    def __init__(self, width_m: float, height_m: float, resolution: float, origin: Tuple[float, float]=(0.0, 0.0)):
        self.resolution = float(resolution)
        self.origin_x, self.origin_y = origin
        self.width = int(math.ceil(width_m / self.resolution))
        self.height = int(math.ceil(height_m / self.resolution))
        self.occ = np.zeros((self.height, self.width), dtype=bool)

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        col = int(round((x - self.origin_x) / self.resolution))
        row = int(round((y - self.origin_y) / self.resolution))
        return (col, row)

    def to_world(self, col: int, row: int) -> Tuple[float, float]:
        x = col * self.resolution + self.origin_x
        y = row * self.resolution + self.origin_y
        return (x, y)

    def inside(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def is_free(self, col: int, row: int) -> bool:
        return self.inside(col, row) and (not self.occ[row, col])

    def cell_center(self, col: int, row: int) -> Tuple[float, float]:
        x = (col + 0.5) * self.resolution + self.origin_x
        y = (row + 0.5) * self.resolution + self.origin_y
        return (x, y)

    def __repr__(self) -> str:
        free = int(np.count_nonzero(~self.occ))
        return f'GridMap({self.width}x{self.height} cells, res={self.resolution}m, {free} free / {self.occ.size} total)'

def inflate(grid: GridMap, radius_m: float) -> GridMap:
    n = int(math.ceil(radius_m / grid.resolution))
    if n <= 0:
        return grid
    out = GridMap(grid.width * grid.resolution, grid.height * grid.resolution, grid.resolution, (grid.origin_x, grid.origin_y))
    occ = grid.occ.copy()
    offsets = []
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            if dx * dx + dy * dy <= n * n:
                offsets.append((dx, dy))
    for dy, dx in offsets:
        src = occ
        h, w = occ.shape
        ys0, ys1 = (max(0, dy), min(h, h + dy))
        xs0, xs1 = (max(0, dx), min(w, w + dx))
        yd0, yd1 = (max(0, -dy), min(h, h - dy))
        xd0, xd1 = (max(0, -dx), min(w, w - dx))
        out.occ[yd0:yd1, xd0:xd1] |= src[ys0:ys1, xs0:xs1]
    return out

def min_clearance(obstacles: Sequence, x: float, y: float) -> float:
    if not obstacles:
        return float('inf')
    return min((o.distance(x, y) for o in obstacles))

def _strip_ns(tag: str) -> str:
    return tag.split('}')[-1] if '}' in tag else tag

def parse_world(path: str, ignore_names: Sequence[str]=('point_A', 'point_B')) -> Tuple[List[object], dict]:
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    tree = ET.parse(path)
    root = tree.getroot()
    world = None
    for el in root.iter():
        if _strip_ns(el.tag) == 'world':
            world = el
            break
    if world is None:
        raise ValueError(f'{path} 里找不到 <world> 元素')
    obstacles: List[object] = []
    skipped: List[str] = []
    info = {'path': path, 'models': 0, 'with_collision': 0}
    for model in world:
        if _strip_ns(model.tag) != 'model':
            continue
        info['models'] += 1
        name = model.get('name', 'unnamed')
        if name in ignore_names:
            skipped.append(name)
            continue
        pose_el = None
        for ch in model:
            if _strip_ns(ch.tag) == 'pose':
                pose_el = ch
                break
        mx = my = mz = 0.0
        mroll = mpitch = myaw = 0.0
        if pose_el is not None and pose_el.text:
            vals = [float(v) for v in pose_el.text.split()]
            if len(vals) >= 6:
                mx, my, mz, mroll, mpitch, myaw = vals[:6]
            elif len(vals) >= 3:
                mx, my, mz = vals[:3]
        for link in model:
            if _strip_ns(link.tag) != 'link':
                continue
            lx = ly = lyaw = 0.0
            for ch in link:
                if _strip_ns(ch.tag) == 'pose' and ch.text:
                    vals = [float(v) for v in ch.text.split()]
                    if len(vals) >= 6:
                        lx, ly, _, _, _, lyaw = vals[:6]
                    elif len(vals) >= 3:
                        lx, ly = (vals[0], vals[1])
                    break
            for coll in link:
                if _strip_ns(coll.tag) != 'collision':
                    continue
                info['with_collision'] += 1
                cx_off = cy_off = cyaw = 0.0
                geom = None
                for ch in coll:
                    t = _strip_ns(ch.tag)
                    if t == 'pose' and ch.text:
                        vals = [float(v) for v in ch.text.split()]
                        if len(vals) >= 6:
                            cx_off, cy_off, _, _, _, cyaw = vals[:6]
                        elif len(vals) >= 3:
                            cx_off, cy_off = (vals[0], vals[1])
                    elif t == 'geometry':
                        geom = ch
                if geom is None:
                    continue
                wx = mx + lx + cx_off
                wy = my + ly + cy_off
                wyaw = myaw + lyaw + cyaw
                for g in geom:
                    gt = _strip_ns(g.tag)
                    if gt == 'box':
                        size = None
                        for s in g:
                            if _strip_ns(s.tag) == 'size' and s.text:
                                size = [float(v) for v in s.text.split()]
                        if size and len(size) >= 2:
                            obstacles.append(RotRect(wx, wy, size[0] / 2.0, size[1] / 2.0, wyaw, name))
                    elif gt == 'cylinder':
                        rad = None
                        for s in g:
                            if _strip_ns(s.tag) == 'radius' and s.text:
                                rad = float(s.text)
                        if rad:
                            obstacles.append(Circle(wx, wy, rad, name))
    info['obstacles'] = len(obstacles)
    info['skipped_markers'] = skipped
    return (obstacles, info)

def build_map(obstacles: Sequence, width_m: float=7.0, height_m: float=7.0, resolution: float=0.05, origin: Tuple[float, float]=(-3.5, -3.5), inflate_radius: float=0.25) -> Tuple[GridMap, GridMap]:
    raw = GridMap(width_m, height_m, resolution, origin)
    for o in obstacles:
        for row in range(raw.height):
            for col in range(raw.width):
                x, y = raw.cell_center(col, row)
                if o.distance(x, y) <= 0.0:
                    raw.occ[row, col] = True
    return (raw, inflate(raw, inflate_radius))
