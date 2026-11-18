#!/usr/bin/env python3
"""地图质量统计 + 从地图里挑"离障碍足够远、彼此分散"的导航目标点。

ROS 地图 pgm 语义：0 = 占用，254 = 空闲，205 = 未知。
"""
import math
import sys

PGM = sys.argv[1] if len(sys.argv) > 1 else '/home/lenovo/vision_ws/models/map_gz_clean.pgm'
YAML = PGM.replace('.pgm', '.yaml')


def load_yaml(path):
    res, origin = 0.05, (0.0, 0.0)
    try:
        with open(path, encoding='utf-8') as f:
            lines = f.read().splitlines()
        for i, ln in enumerate(lines):
            if ln.startswith('resolution:'):
                res = float(ln.split(':')[1])
            if ln.startswith('origin:'):
                vals = []
                for nx in lines[i + 1:i + 4]:
                    vals.append(float(nx.strip().lstrip('-').strip()))
                origin = (float(lines[i + 1].strip().lstrip('-')),
                          float(lines[i + 2].strip().lstrip('-')))
    except OSError:
        pass
    return res, origin


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    idx, fields = 0, []
    while len(fields) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        s = b''
        while not data[idx:idx + 1].isspace():
            s += data[idx:idx + 1]
            idx += 1
        fields.append(s)
    _, w, h, _ = fields
    return int(w), int(h), data[idx + 1:]


RES, ORIGIN = load_yaml(YAML)
W, H, px = read_pgm(PGM)
vals = list(px[:W * H])
n = len(vals)
occ = sum(1 for v in vals if v <= 10)
unk = sum(1 for v in vals if 10 < v < 250)
free = n - occ - unk
print(f'地图 {W}x{H} @ {RES} m/格 -> {W * RES:.2f} m x {H * RES:.2f} m')
print(f'占用 {occ / n * 100:.1f}%  空闲 {free / n * 100:.1f}%  未知 {unk / n * 100:.1f}%')

occ_grid = [[1 if vals[r * W + c] <= 10 else 0 for c in range(W)] for r in range(H)]
free_grid = [[1 if vals[r * W + c] >= 250 else 0 for c in range(W)] for r in range(H)]

# BFS 距离变换（格）
INF = 10 ** 9
dist = [[INF] * W for _ in range(H)]
queue = [(r, c) for r in range(H) for c in range(W) if occ_grid[r][c]]
for r, c in queue:
    dist[r][c] = 0
head = 0
while head < len(queue):
    r, c = queue[head]
    head += 1
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nr, nc = r + dr, c + dc
        if 0 <= nr < H and 0 <= nc < W and dist[nr][nc] > dist[r][c] + 1:
            dist[nr][nc] = dist[r][c] + 1
            queue.append((nr, nc))


def to_world(r, c):
    return (ORIGIN[0] + (c + 0.5) * RES, ORIGIN[1] + (H - r - 0.5) * RES)


cands = [(r, c) for r in range(H) for c in range(W)
         if free_grid[r][c] and dist[r][c] * RES >= 0.6]
print(f'净空 >= 0.6 m 的空闲格子：{len(cands)}')
picked = []
for r, c in sorted(cands, key=lambda rc: -dist[rc[0]][rc[1]]):
    x, y = to_world(r, c)
    if all(math.hypot(x - p[0], y - p[1]) > 2.0 for p in picked):
        picked.append((x, y, dist[r][c] * RES))
    if len(picked) == 3:
        break
for i, (x, y, cl) in enumerate(picked, 1):
    print(f'目标{i}: ({x:.2f}, {y:.2f})  净空 {cl:.2f} m')
print('ROS 参数: [' + ', '.join(f'{x:.2f}, {y:.2f}, 0.0' for x, y, _ in picked) + ']')
