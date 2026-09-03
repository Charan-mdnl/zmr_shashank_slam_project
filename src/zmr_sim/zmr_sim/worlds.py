#!/usr/bin/env python3
"""World generation and loading for the ZMR headless simulator.

A world is an occupancy grid: uint8 array where 0 = free and 100 = occupied,
plus a resolution (m/cell) and an origin (world coords of cell [0, 0]).

Worlds can either be generated procedurally (``make_world``) or loaded from a
standard ROS map pair (``map.yaml`` + ``map.pgm``), so a map produced by
slam_toolbox can be fed straight back in as a simulation world.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import yaml


class World:
    """An occupancy grid with world <-> cell coordinate helpers."""

    def __init__(self, grid: np.ndarray, resolution: float, origin=(0.0, 0.0)):
        if grid.ndim != 2:
            raise ValueError('world grid must be 2-D')
        if resolution <= 0.0:
            raise ValueError('resolution must be positive')
        self.grid = grid.astype(np.uint8)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.height, self.width = self.grid.shape
        # Boolean view used by the ray caster; built once.
        self.occupied = self.grid > 50

    # ---------------------------------------------------------------- coords
    def world_to_cell(self, x, y):
        """World metres -> (col, row) indices. Accepts scalars or arrays."""
        col = (np.asarray(x) - self.origin[0]) / self.resolution
        row = (np.asarray(y) - self.origin[1]) / self.resolution
        return col, row

    def cell_to_world(self, col, row):
        x = np.asarray(col) * self.resolution + self.origin[0]
        y = np.asarray(row) * self.resolution + self.origin[1]
        return x, y

    def is_occupied(self, x, y) -> bool:
        """True if the world point falls in an occupied cell or outside the map."""
        col, row = self.world_to_cell(x, y)
        c, r = int(np.floor(col)), int(np.floor(row))
        if c < 0 or c >= self.width or r < 0 or r >= self.height:
            return True  # outside the world counts as blocked
        return bool(self.occupied[r, c])

    def any_occupied(self, xs, ys) -> bool:
        """Vectorised occupancy test over arrays of world points."""
        col, row = self.world_to_cell(xs, ys)
        c = np.floor(col).astype(np.int64)
        r = np.floor(row).astype(np.int64)
        outside = (c < 0) | (c >= self.width) | (r < 0) | (r >= self.height)
        if np.any(outside):
            return True
        return bool(np.any(self.occupied[r, c]))

    # ------------------------------------------------------------------- io
    @classmethod
    def from_map_yaml(cls, yaml_path: str) -> 'World':
        """Load a standard ROS map (yaml + pgm) as a simulation world."""
        with open(yaml_path, 'r') as fh:
            meta = yaml.safe_load(fh)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
        pgm = read_pgm(image)
        negate = int(meta.get('negate', 0))
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        # ROS map convention: p_occ = (255 - value) / 255, and row 0 of the
        # image is the TOP of the map, so the image must be flipped vertically.
        vals = pgm.astype(np.float64)
        if negate:
            vals = 255.0 - vals
        p_occ = (255.0 - vals) / 255.0
        grid = np.where(p_occ >= occupied_thresh, 100, 0).astype(np.uint8)
        grid = np.flipud(grid)
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        return cls(grid, float(meta['resolution']), (origin[0], origin[1]))

    def save_map(self, path_no_ext: str) -> None:
        """Write this world as a ROS map pair, for inspection or reuse."""
        img = np.where(self.grid > 50, 0, 254).astype(np.uint8)
        img = np.flipud(img)
        write_pgm(path_no_ext + '.pgm', img)
        meta = {
            'image': os.path.basename(path_no_ext) + '.pgm',
            'mode': 'trinary',
            'resolution': self.resolution,
            'origin': [self.origin[0], self.origin[1], 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }
        with open(path_no_ext + '.yaml', 'w') as fh:
            yaml.safe_dump(meta, fh, default_flow_style=False, sort_keys=False)


# -------------------------------------------------------------------- pgm io
def read_pgm(path: str) -> np.ndarray:
    """Read a binary (P5) or ASCII (P2) PGM."""
    with open(path, 'rb') as fh:
        data = fh.read()

    def tokens(buf):
        i = 0
        while i < len(buf):
            if buf[i:i + 1] == b'#':
                while i < len(buf) and buf[i:i + 1] not in b'\r\n':
                    i += 1
            elif buf[i:i + 1].isspace():
                i += 1
            else:
                j = i
                while j < len(buf) and not buf[j:j + 1].isspace():
                    j += 1
                yield buf[i:j], j
                i = j

    it = tokens(data)
    magic, _ = next(it)
    width, _ = next(it)
    height, _ = next(it)
    maxval, end = next(it)
    w, h = int(width), int(height)
    if magic == b'P5':
        start = end + 1  # exactly one whitespace byte after maxval
        return np.frombuffer(data[start:start + w * h], dtype=np.uint8).reshape(h, w)
    if magic == b'P2':
        rest = data[end:].split()
        return np.array([int(v) for v in rest[:w * h]], dtype=np.uint8).reshape(h, w)
    raise ValueError(f'unsupported PGM magic {magic!r} in {path}')


def write_pgm(path: str, img: np.ndarray) -> None:
    h, w = img.shape
    with open(path, 'wb') as fh:
        fh.write(b'P5\n%d %d\n255\n' % (w, h))
        fh.write(img.astype(np.uint8).tobytes())


# ------------------------------------------------------------ world builders
def warehouse(resolution: float = 0.05) -> World:
    """A 30 x 20 m indoor site: outer shell, rooms, a corridor loop, racks.

    Sized for the ZMR (1.05 x 0.71 m): doorways are 1.6 m, aisles 2.5 m or
    wider, so the robot fits with margin on every legal route.
    """
    ox, oy = -1.0, -1.0
    w = int(round(30.0 / resolution))
    h = int(round(20.0 / resolution))
    g = np.zeros((h, w), dtype=np.uint8)

    def rect(x0, y0, x1, y1, v=100):
        c0 = int(round((min(x0, x1) - ox) / resolution))
        c1 = int(round((max(x0, x1) - ox) / resolution))
        r0 = int(round((min(y0, y1) - oy) / resolution))
        r1 = int(round((max(y0, y1) - oy) / resolution))
        c0, c1 = max(0, c0), min(w, c1)
        r0, r1 = max(0, r0), min(h, r1)
        g[r0:r1, c0:c1] = v

    def wall(x0, y0, x1, y1, t=0.15):
        rect(min(x0, x1) - t / 2, min(y0, y1) - t / 2,
             max(x0, x1) + t / 2, max(y0, y1) + t / 2)

    # outer shell, 28 x 18 m interior
    wall(0, 0, 28, 0)
    wall(0, 18, 28, 18)
    wall(0, 0, 0, 18)
    wall(28, 0, 28, 18)

    # internal partitions -> a corridor loop around the middle
    wall(6, 0, 6, 6)
    wall(6, 6, 12, 6)
    wall(12, 0, 12, 6)
    wall(6, 12, 6, 18)
    wall(6, 12, 12, 12)
    wall(12, 12, 12, 18)
    wall(18, 4, 18, 14)
    wall(18, 4, 24, 4)
    wall(18, 14, 24, 14)
    wall(24, 4, 24, 14)

    # Doorways, 2.2 m clear. The robot is 1.05 x 0.71 m, so its circumscribed
    # radius is 0.634 m; a global planner that keeps that much clearance needs
    # ~1.3 m of the opening, and 1.6 m doors left only a 0.3 m ribbon to steer
    # down. 2.2 m is the realistic industrial width for a truck this size.
    rect(7.9, 5.7, 10.1, 6.4, 0)
    rect(7.9, 11.7, 10.1, 12.4, 0)
    rect(17.7, 7.9, 18.4, 10.1, 0)
    rect(23.7, 7.9, 24.4, 10.1, 0)

    # storage racks
    for x in (14.0, 15.6):
        rect(x, 1.0, x + 0.5, 4.0)
        rect(x, 14.0, x + 0.5, 17.0)

    # pillars
    for px, py in ((3.0, 9.0), (9.0, 9.0), (15.0, 9.0), (21.0, 16.0)):
        rect(px - 0.2, py - 0.2, px + 0.2, py + 0.2)

    return World(g, resolution, (ox, oy))


def empty(resolution: float = 0.05, size=(12.0, 12.0)) -> World:
    """A bare walled room - useful for isolating controller behaviour."""
    ox, oy = -1.0, -1.0
    w = int(round((size[0] + 2.0) / resolution))
    h = int(round((size[1] + 2.0) / resolution))
    g = np.zeros((h, w), dtype=np.uint8)
    t = max(1, int(round(0.15 / resolution)))
    g[:t, :] = 100
    g[-t:, :] = 100
    g[:, :t] = 100
    g[:, -t:] = 100
    return World(g, resolution, (ox, oy))


def corridor(resolution: float = 0.05) -> World:
    """A 3 m wide, 20 m long corridor with two obstacles - avoidance testing."""
    ox, oy = -1.0, -1.0
    w = int(round(24.0 / resolution))
    h = int(round(8.0 / resolution))
    g = np.zeros((h, w), dtype=np.uint8)

    def rect(x0, y0, x1, y1, v=100):
        c0 = max(0, int(round((x0 - ox) / resolution)))
        c1 = min(w, int(round((x1 - ox) / resolution)))
        r0 = max(0, int(round((y0 - oy) / resolution)))
        r1 = min(h, int(round((y1 - oy) / resolution)))
        g[r0:r1, c0:c1] = v

    rect(0, 0, 21, 0.15)
    rect(0, 3.0, 21, 3.15)
    rect(0, 0, 0.15, 3.15)
    rect(21, 0, 21.15, 3.15)
    rect(6.0, 0.15, 6.6, 1.7)     # obstacle forcing a pass on the left
    rect(13.0, 1.6, 13.6, 3.0)    # obstacle forcing a pass on the right
    return World(g, resolution, (ox, oy))


BUILDERS = {'warehouse': warehouse, 'empty': empty, 'corridor': corridor}


def build(name: str, resolution: float = 0.05) -> World:
    if name not in BUILDERS:
        raise KeyError(f'unknown world {name!r}; choose from {sorted(BUILDERS)}')
    return BUILDERS[name](resolution)


def main(argv=None) -> int:
    """CLI: write a built-in world out as a ROS map pair."""
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ('-h', '--help'):
        print('usage: make_world <warehouse|empty|corridor> <output_path_no_ext> '
              '[resolution]')
        return 0 if argv else 2
    name = argv[0]
    out = argv[1] if len(argv) > 1 else name
    res = float(argv[2]) if len(argv) > 2 else 0.05
    world = build(name, res)
    os.makedirs(os.path.dirname(os.path.abspath(out)) or '.', exist_ok=True)
    world.save_map(out)
    print(f'wrote {out}.pgm and {out}.yaml '
          f'({world.width}x{world.height} cells @ {world.resolution} m)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
