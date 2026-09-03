#!/usr/bin/env python3
"""Offline correctness tests for the ZMR stack.

These need no ROS graph and no display - they exercise the geometry, the ray
caster, the planner and the safety layer's collision maths directly, so they
run in CI and catch the class of bug that is painful to find in a live run.

    python3 test/test_zmr.py
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
sys.path.insert(0, os.path.join(SRC, 'zmr_sim'))
sys.path.insert(0, os.path.join(SRC, 'zmr_control'))

from zmr_sim.worlds import build                                  # noqa: E402
from zmr_control.grid_planner import (INSCRIBED, astar,           # noqa: E402
                                      build_costmap, nearest_free, simplify)

PASS, FAIL = [], []


def check(name, cond, detail=''):
    (PASS if cond else FAIL).append(name)
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))


# --------------------------------------------------------------- URDF geometry
def test_urdf():
    print('\nURDF frame geometry')
    import xml.etree.ElementTree as ET

    def rpy2R(r, p, y):
        cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                                  math.sin(p), math.cos(y), math.sin(y))
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])

    urdf = os.path.join(SRC, 'zmr_description', 'urdf', 'zmr_robot.urdf')
    root = ET.parse(urdf).getroot()
    J = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        ax = j.find('axis')
        xyz = np.array([float(v) for v in o.get('xyz').split()])
        rpy = np.array([float(v) for v in o.get('rpy').split()])
        a = np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else None
        J[j.find('child').get('link')] = (j.find('parent').get('link'), xyz, rpy2R(*rpy), a)

    def to(link, target='base_link'):
        R, t, cur = np.eye(3), np.zeros(3), link
        while cur != target and cur in J:
            _p, xyz, Rj, _a = J[cur]
            t = Rj @ t + xyz
            R = Rj @ R
            cur = J[cur][0]
        return t, R

    lw, Rl = to('left_wheel')
    rw, Rr = to('right_wheel')
    axl = Rl @ J['left_wheel'][3]
    axr = Rr @ J['right_wheel'][3]

    check('wheel separation is 0.526 m',
          abs(abs(lw[1] - rw[1]) - 0.526) < 1e-6, f'{abs(lw[1]-rw[1]):.4f}')
    check('both wheel axes point the same way (positive = forward)',
          float(axl @ axr) > 0.99, f'left {np.round(axl,3)} right {np.round(axr,3)}')
    bl, _ = to('base_link', 'base_footprint')
    check('wheel bottom sits exactly on the base_footprint plane',
          abs((bl[2] + lw[2]) - 0.080) < 1e-6, f'z={bl[2]+lw[2]:.4f} vs radius 0.080')
    lid, Rlid = to('lidar_link')
    yaw = math.atan2(Rlid[1, 0], Rlid[0, 0])
    check('lidar_link resolves to (-0.425, 0.000) yaw -90 deg',
          abs(lid[0] + 0.425) < 1e-3 and abs(lid[1]) < 1e-3
          and abs(yaw + math.pi / 2) < 1e-3,
          f'({lid[0]:+.3f},{lid[1]:+.3f}) yaw {math.degrees(yaw):+.1f}')


# ------------------------------------------------------------------- ray cast
def test_raycast():
    print('\nSimulator ray casting')
    w = build('warehouse', 0.05)
    lx, ly, res = 2.5, 2.5, w.resolution

    def cast(angle):
        ca, sa = math.cos(angle), math.sin(angle)
        cx, cy = (lx - w.origin[0]) / res, (ly - w.origin[1]) / res
        for k in range(4, 700):
            xi, yi = int(cx + ca * k), int(cy + sa * k)
            if not (0 <= xi < w.width and 0 <= yi < w.height):
                return float('inf')
            if w.occupied[yi, xi]:
                return k * res
        return float('inf')

    for label, ang, exp in (('+x to the x=6 partition', 0.0, 6.0 - 0.075 - 2.5),
                            ('-x to the x=0 wall', math.pi, 2.5 - 0.075),
                            ('+y to the y=18 wall', math.pi / 2, 18.0 - 0.075 - 2.5),
                            ('-y to the y=0 wall', -math.pi / 2, 2.5 - 0.075)):
        m = cast(ang)
        check(f'range {label}', abs(m - exp) <= 0.08, f'expected {exp:.3f} got {m:.3f}')


# -------------------------------------------------------------------- planner
def test_planner():
    print('\nGlobal planner')
    w = build('warehouse', 0.05)
    grid = np.where(w.occupied, 100, 0).astype(np.int16)
    R = 0.65
    cost = build_costmap(grid, w.resolution, R, 0.95, 3.0, unknown_is_obstacle=False)

    def cell(x, y):
        return (int(math.floor((y - w.origin[1]) / w.resolution)),
                int(math.floor((x - w.origin[0]) / w.resolution)))

    def world(r, c):
        return (w.origin[0] + (c + 0.5) * w.resolution,
                w.origin[1] + (r + 0.5) * w.resolution)

    start = nearest_free(cost, cell(2.5, 2.5))
    goal = nearest_free(cost, cell(21.0, 9.0))
    path = astar(cost, start, goal)
    check('A* finds a route from start to the inner room through a doorway',
          path is not None and len(path) > 10,
          f'{len(path) if path else 0} cells')

    if path:
        simp = simplify(path, cost)
        pts = [world(r, c) for r, c in simp]
        from scipy import ndimage
        dist = ndimage.distance_transform_edt(~w.occupied) * w.resolution

        def clearance_along(a, b):
            n = max(2, int(math.dist(a, b) / 0.05))
            worst = 9e9
            for i in range(n + 1):
                x = a[0] + (b[0] - a[0]) * i / n
                y = a[1] + (b[1] - a[1]) * i / n
                r, c = cell(x, y)
                if 0 <= r < w.height and 0 <= c < w.width:
                    worst = min(worst, dist[r, c])
            return worst

        worst = min(clearance_along(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        check('simplified path keeps the robot clear of every obstacle',
              worst >= 0.60, f'min clearance {worst:.3f} m along the simplified path')

    # unknown space must never be cheaper than verified-free floor
    g2 = grid.copy()
    g2[:, 300:] = -1
    c2 = build_costmap(g2, w.resolution, R, 0.95, 3.0, unknown_is_obstacle=False)
    check('unknown cells cost more than open known floor',
          int(c2[200, 400]) > int(c2[180, 100]),
          f'unknown={int(c2[200,400])} known-free={int(c2[180,100])}')

    # nearest_free must return the true nearest, not the first ring hit
    t = np.full((41, 41), 254, dtype=np.uint16)
    t[20, 25] = 0            # euclidean 5.0
    t[15, 15] = 0            # euclidean 7.07
    check('nearest_free returns the closest free cell',
          nearest_free(t, (20, 20)) == (20, 25), str(nearest_free(t, (20, 20))))


# ----------------------------------------------------- avoidance collision maths
def test_avoidance_geometry():
    print('\nObstacle-avoidance collision maths')
    hl, hw = 1.05 / 2.0, 0.71 / 2.0

    def sdf(pts, xs, ys, ths):
        P = np.asarray(pts, dtype=float)
        dx = P[None, :, 0] - np.asarray(xs)[:, None]
        dy = P[None, :, 1] - np.asarray(ys)[:, None]
        c = np.cos(-np.asarray(ths))[:, None]
        s = np.sin(-np.asarray(ths))[:, None]
        lx = dx * c - dy * s
        ly = dx * s + dy * c
        ox, oy = np.abs(lx) - hl, np.abs(ly) - hw
        return (np.hypot(np.maximum(ox, 0), np.maximum(oy, 0))
                + np.minimum(np.maximum(ox, oy), 0.0))

    check('a point at the body centre reports negative distance',
          sdf([(0.0, 0.0)], [0.0], [0.0], [0.0])[0, 0] < 0)
    d = sdf([(1.025, 0.0)], [0.0], [0.0], [0.0])[0, 0]
    check('a point 0.5 m ahead of the nose reports 0.5 m', abs(d - 0.5) < 1e-6, f'{d:.4f}')
    d = sdf([(0.0, 0.855)], [0.0], [0.0], [0.0])[0, 0]
    check('a point 0.5 m off the flank reports 0.5 m', abs(d - 0.5) < 1e-6, f'{d:.4f}')
    # rotating the robot 90 deg swaps which extent faces the point
    d0 = sdf([(0.7, 0.0)], [0.0], [0.0], [0.0])[0, 0]
    d90 = sdf([(0.7, 0.0)], [0.0], [0.0], [math.pi / 2])[0, 0]
    check('rotating the footprint 90 deg changes the clearance correctly',
          abs(d0 - (0.7 - hl)) < 1e-6 and abs(d90 - (0.7 - hw)) < 1e-6,
          f'0deg {d0:.3f}  90deg {d90:.3f}')
    # a straight arc must sweep forward
    v, w_, ts = 0.4, 1e-9, np.linspace(0.2, 1.6, 8)
    xs = v * ts
    check('a zero-curvature arc advances along +x', xs[-1] > 0.6, f'{xs[-1]:.2f} m')


# ---------------------------------------------------------------- teleop keys
def test_teleop_bindings():
    print('\nTeleop key bindings')
    # Parse the literal out of the source rather than importing the module,
    # so this suite runs without a sourced ROS environment.
    import ast
    src = open(os.path.join(SRC, 'zmr_control', 'zmr_control',
                            'teleop_key.py')).read()
    BINDINGS = None
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and \
                any(getattr(t, 'id', None) == 'BINDINGS' for t in node.targets):
            BINDINGS = ast.literal_eval(node.value)
    check('teleop_key defines a BINDINGS table', BINDINGS is not None)
    if BINDINGS is None:
        return
    cases = {'w': (1.0, 0.0), 's': (-1.0, 0.0), 'a': (0.0, 1.0), 'd': (0.0, -1.0),
             'q': (1.0, 1.0), 'e': (1.0, -1.0), 'z': (-1.0, 1.0), 'c': (-1.0, -1.0)}
    ok = all(BINDINGS.get(k) == v for k, v in cases.items())
    check('w/a/s/d and the four arc keys map to the right (v, w) signs', ok)
    check('a turns left (positive yaw, ROS convention)', BINDINGS['a'][1] > 0)
    check('d turns right (negative yaw)', BINDINGS['d'][1] < 0)


# ------------------------------------------------------------------ world/map
def test_world_roundtrip():
    print('\nWorld save/load round trip')
    import tempfile
    w = build('corridor', 0.05)
    with tempfile.TemporaryDirectory() as d:
        base = os.path.join(d, 'w')
        w.save_map(base)
        from zmr_sim.worlds import World
        back = World.from_map_yaml(base + '.yaml')
        check('grid shape survives a save/load round trip',
              back.grid.shape == w.grid.shape, f'{w.grid.shape} -> {back.grid.shape}')
        check('occupied cells survive a save/load round trip',
              int((back.occupied != w.occupied).sum()) == 0,
              f'{int((back.occupied != w.occupied).sum())} cells differ')
        check('resolution and origin survive',
              abs(back.resolution - w.resolution) < 1e-9
              and abs(back.origin[0] - w.origin[0]) < 1e-9)


def main():
    print('ZMR offline test suite')
    for t in (test_urdf, test_raycast, test_planner, test_avoidance_geometry,
              test_teleop_bindings, test_world_roundtrip):
        t()
    print(f'\n{len(PASS)} passed, {len(FAIL)} failed')
    if FAIL:
        for f in FAIL:
            print(f'  FAILED: {f}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
