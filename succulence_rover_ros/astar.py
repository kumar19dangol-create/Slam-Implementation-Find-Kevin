"""
A* Path Planner (no ROS dependencies)

8-connected grid search with octile heuristic. Designed to run against a
nav_msgs/OccupancyGrid payload where cell values are 0..100 for known cells
and -1 for unknown.
"""

import heapq
import numpy as np
from typing import List, Optional, Tuple

Cell = Tuple[int, int]

# Octile step cost lookup (8-connected grid)
_SQRT2 = np.sqrt(2.0)
_NEIGHBORS = [
    (-1,  0, 1.0),
    ( 1,  0, 1.0),
    ( 0, -1, 1.0),
    ( 0,  1, 1.0),
    (-1, -1, _SQRT2),
    (-1,  1, _SQRT2),
    ( 1, -1, _SQRT2),
    ( 1,  1, _SQRT2),
]


def _octile(a: Cell, b: Cell) -> float:
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (dr + dc) + (_SQRT2 - 2.0) * min(dr, dc)


def inflate_obstacles(grid: np.ndarray, radius_cells: int,
                      occupancy_threshold: int,
                      treat_unknown_as_obstacle: bool) -> np.ndarray:
    """
    Return a bool grid of "blocked" cells after inflating obstacles by radius_cells.

    Unknown cells (-1) are either blocked (if treat_unknown_as_obstacle) or
    treated as free. Inflation uses a square dilation for simplicity.
    """
    blocked = grid >= occupancy_threshold
    if treat_unknown_as_obstacle:
        blocked |= grid < 0

    if radius_cells <= 0:
        return blocked

    h, w = blocked.shape
    out = np.zeros_like(blocked)
    rows, cols = np.where(blocked)
    for r, c in zip(rows, cols):
        r0 = max(0, r - radius_cells)
        r1 = min(h, r + radius_cells + 1)
        c0 = max(0, c - radius_cells)
        c1 = min(w, c + radius_cells + 1)
        out[r0:r1, c0:c1] = True
    return out


def astar_search(blocked: np.ndarray,
                 start: Cell,
                 goal: Cell) -> Optional[List[Cell]]:
    """
    Run A* over an 8-connected grid.

    Args:
        blocked: 2D bool array — True where the robot cannot pass.
        start:   (row, col) start cell (must be unblocked).
        goal:    (row, col) goal cell (must be unblocked).

    Returns:
        List of (row, col) cells from start to goal inclusive, or None if unreachable.
    """
    h, w = blocked.shape

    if not (0 <= start[0] < h and 0 <= start[1] < w):
        return None
    if not (0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    if blocked[start] or blocked[goal]:
        return None
    if start == goal:
        return [start]

    open_heap: List[Tuple[float, float, int, Cell]] = []
    counter = 0
    g_score = {start: 0.0}
    came_from = {}

    h0 = _octile(start, goal)
    heapq.heappush(open_heap, (h0, h0, counter, start))
    closed = set()

    while open_heap:
        _, _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)
        cg = g_score[current]

        cr, cc = current
        for dr, dc, step in _NEIGHBORS:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if blocked[nr, nc]:
                continue
            neighbor = (nr, nc)
            if neighbor in closed:
                continue
            tentative = cg + step
            if tentative < g_score.get(neighbor, np.inf):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                hscore = _octile(neighbor, goal)
                f = tentative + hscore
                counter += 1
                heapq.heappush(open_heap, (f, hscore, counter, neighbor))

    return None
