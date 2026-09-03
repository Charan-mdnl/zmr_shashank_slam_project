#!/usr/bin/env python3
"""Occupancy-grid path planning for the ZMR AMR.

A compact global planner: inflate the map by the robot's radius, then run A*
over the inflated grid. This is the job nav2's planner_server would do; the
nav2 stack is not part of this workspace, and a purely reactive controller
cannot get a robot around a wall, so the navigator carries its own.

The inflation gradient matters as much as the obstacle test: without it A*
returns paths that graze every corner, and the local safety layer then fights
the plan the whole way.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

# cost values, mirroring nav2's convention
FREE = 0
INSCRIBED = 253
LETHAL = 254
UNKNOWN = 255


def build_costmap(grid: np.ndarray, resolution: float, robot_radius: float,
                  inflation_radius: float, cost_scaling: float = 3.0,
                  unknown_is_obstacle: bool = True,
                  unknown_cost: int = 200) -> np.ndarray:
    """Turn a raw occupancy grid (-1 unknown, 0..100) into a cost grid.

    Cells within `robot_radius` of an obstacle are INSCRIBED (never traversable);
    cells within `inflation_radius` get an exponentially decaying cost so the
    planner prefers the middle of free space.
    """
    from scipy import ndimage

    occ = grid >= 50
    unknown = grid < 0

    blocked = (occ | unknown) if unknown_is_obstacle else occ
    # distance in cells from every free cell to the nearest blocked cell
    dist = ndimage.distance_transform_edt(~blocked) * resolution

    cost = np.zeros(grid.shape, dtype=np.uint16)
    cost[blocked] = LETHAL
    inscribed = dist <= robot_radius
    cost[inscribed & ~blocked] = INSCRIBED

    # Inflation band, normalised so the cost decays continuously to 0 at
    # inflation_radius. Without the normalisation the band ends in a ~100-unit
    # cliff and the planner tracks that contour instead of open floor.
    band = (dist > robot_radius) & (dist <= inflation_radius)
    if np.any(band):
        width = max(inflation_radius - robot_radius, 1e-6)
        edge = math.exp(-cost_scaling * width)
        raw = np.exp(-cost_scaling * (dist[band] - robot_radius))
        cost[band] = (252.0 * (raw - edge) / max(1.0 - edge, 1e-6)).astype(np.uint16)

    # Unknown space is traversable when unknown_is_obstacle is False, but it
    # must never be CHEAPER than verified-free floor, or A* will route through
    # unmapped space in preference to a doorway it can actually see.
    if not unknown_is_obstacle:
        cost[unknown & ~occ] = np.maximum(cost[unknown & ~occ], unknown_cost)
    return cost


_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
               (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def astar(cost: np.ndarray, start: tuple[int, int], goal: tuple[int, int],
          max_nodes: int = 4_000_000):
    """A* over a cost grid. start/goal are (row, col). Returns a list of cells."""
    h, w = cost.shape
    if not max_nodes:
        # every cell closes at most once, so the grid size is the real bound
        max_nodes = cost.size
    sr, sc = start
    gr, gc = goal
    if not (0 <= sr < h and 0 <= sc < w and 0 <= gr < h and 0 <= gc < w):
        return None
    if cost[gr, gc] >= INSCRIBED:
        return None

    def heuristic(r, c):
        dr, dc = abs(r - gr), abs(c - gc)
        return (dr + dc) + (math.sqrt(2) - 2.0) * min(dr, dc)

    g_score = np.full(cost.shape, np.inf, dtype=np.float32)
    g_score[sr, sc] = 0.0
    came = {}
    open_heap = [(heuristic(sr, sc), 0.0, sr, sc)]
    closed = np.zeros(cost.shape, dtype=bool)
    expanded = 0

    while open_heap:
        _f, g, r, c = heapq.heappop(open_heap)
        if closed[r, c]:
            continue
        closed[r, c] = True
        expanded += 1
        if expanded > max_nodes:
            return None
        if r == gr and c == gc:
            path = [(r, c)]
            while (r, c) in came:
                r, c = came[(r, c)]
                path.append((r, c))
            path.reverse()
            return path
        for dr, dc, step in _NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= h or nc < 0 or nc >= w or closed[nr, nc]:
                continue
            cval = cost[nr, nc]
            if cval >= INSCRIBED:
                continue
            # penalise proximity to obstacles so paths keep to open space
            ng = g + step * (1.0 + 0.04 * float(cval))
            if ng < g_score[nr, nc]:
                g_score[nr, nc] = ng
                came[(nr, nc)] = (r, c)
                heapq.heappush(open_heap, (ng + heuristic(nr, nc), ng, nr, nc))
    return None


def nearest_free(cost: np.ndarray, cell: tuple[int, int], max_radius: int = 60):
    """Nearest traversable cell to `cell`, for when a pose lands in inflation."""
    h, w = cost.shape
    r0, c0 = cell
    if 0 <= r0 < h and 0 <= c0 < w and cost[r0, c0] < INSCRIBED:
        return (r0, c0)
    for rad in range(1, max_radius + 1):
        ring = []
        for dr in range(-rad, rad + 1):
            for dc in (-rad, rad) if abs(dr) != rad else range(-rad, rad + 1):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < h and 0 <= c < w and cost[r, c] < INSCRIBED:
                    ring.append((dr * dr + dc * dc, r, c))
        if ring:
            # true nearest within the ring, not the first one enumerated:
            # a Chebyshev ring spans r..r*sqrt(2) in Euclidean distance
            ring.sort()
            return (ring[0][1], ring[0][2])
    return None


def simplify(path, cost: np.ndarray, tolerance_cells: int = 2):
    """Drop intermediate cells that add nothing, keeping the path collision-free."""
    if not path or len(path) < 3:
        return path

    def clear_line(a, b, ceiling):
        (r0, c0), (r1, c1) = a, b
        n = max(abs(r1 - r0), abs(c1 - c0)) * 2      # oversample the segment
        if n == 0:
            return True
        for i in range(n + 1):
            fr = r0 + (r1 - r0) * i / n
            fc = c0 + (c1 - c0) * i / n
            # check both cells a diagonal step could pass between
            for r, c in ((int(math.floor(fr)), int(math.floor(fc))),
                         (int(math.ceil(fr)), int(math.ceil(fc)))):
                if not (0 <= r < cost.shape[0] and 0 <= c < cost.shape[1]):
                    return False
                if cost[r, c] >= INSCRIBED or cost[r, c] > ceiling:
                    return False
        return True

    # A shortcut may not be worse than the worst cell the sub-path it replaces
    # already occupied, so simplification can never push the route closer to an
    # obstacle than A* chose to go.
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            ceiling = max(int(cost[r, c]) for r, c in path[i:j + 1])
            if clear_line(path[i], path[j], ceiling):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out
