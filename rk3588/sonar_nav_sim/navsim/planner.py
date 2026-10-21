"""Reachable frontier viewpoints and A* on observed free space only."""
from __future__ import annotations
from collections import deque
import heapq
import math
from .mapping import OccupancyGrid


def neighbors(cell: tuple[int, int], free: set[tuple[int, int]]):
    c, r = cell
    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
        nxt = (c + dc, r + dr)
        if nxt in free and (not dc or not dr or ((c + dc, r) in free and (c, r + dr) in free)):
            yield nxt


def astar(start: tuple[int, int], goal: tuple[int, int], free: set[tuple[int, int]]) -> list[tuple[int, int]]:
    if start not in free or goal not in free:
        return []
    queue = [(0.0, start)]
    parent = {}
    costs = {start: 0.0}
    while queue:
        _, node = heapq.heappop(queue)
        if node == goal:
            route = [node]
            while node in parent:
                node = parent[node]
                route.append(node)
            return route[::-1]
        for nxt in neighbors(node, free):
            cost = costs[node] + math.hypot(nxt[0] - node[0], nxt[1] - node[1])
            if cost < costs.get(nxt, float("inf")):
                costs[nxt] = cost
                parent[nxt] = node
                heapq.heappush(queue, (cost + math.hypot(nxt[0] - goal[0], nxt[1] - goal[1]), nxt))
    return []


def choose_frontier(grid: OccupancyGrid, start: tuple[int, int], free: set[tuple[int, int]],
                    visits: dict[tuple[int, int], int], blocked: set[tuple[int, int]]) -> list[tuple[int, int]]:
    if start not in free:
        return []
    distances = {start: 0}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        for nxt in neighbors(cell, free):
            if nxt not in distances:
                distances[nxt] = distances[cell] + 1
                queue.append(nxt)
    # Viewpoints sit back from the frontier so the whole robot remains observed.
    search_radius = max(3, math.ceil(0.65 / grid.resolution))
    candidates = []
    for cell, steps in distances.items():
        if steps < 4 or cell in blocked or cell[0] % 2 or cell[1] % 2:
            continue
        c, r = cell
        unknown = sum(grid.in_bounds(c + dc, r + dr) and grid.cell_state(c + dc, r + dr) == -1
                      for dc in range(-search_radius, search_radius + 1, 2)
                      for dr in range(-search_radius, search_radius + 1, 2)
                      if dc * dc + dr * dr <= search_radius ** 2)
        if unknown < 3:
            continue
        local_visits = sum(visits.get((c + dc, r + dr), 0) for dc in range(-2, 3) for dr in range(-2, 3))
        score = unknown * 1.5 + min(steps, 30) * 0.18 - steps * 0.07 - local_visits * 0.35
        candidates.append((score, cell))
    for _, goal in sorted(candidates, reverse=True)[:12]:
        route = astar(start, goal, free)
        if route:
            return route
    return []
