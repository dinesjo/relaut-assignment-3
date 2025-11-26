"""
Dijkstra's pathfinding algorithm for robot navigation.
Finds the shortest path from start to goal avoiding obstacles.
"""

import heapq
from typing import List, Optional
from robot import Robot
import logging

logger = logging.getLogger(__name__)


def find_path(robot: Robot, goal: int) -> Optional[List[int]]:
    """
    Find shortest path from robot's current position to goal using Dijkstra's algorithm

    Args:
        robot: Robot instance with current position
        goal: Target position to reach

    Returns:
        List of positions from start to goal, or None if no path exists
    """
    assert robot.grid.is_valid_position(goal), f"Goal {goal} is out of bounds"

    if not robot.grid.is_walkable(goal):
        logger.error(f"Goal position {goal} is blocked by a shelf")
        return None

    logger.info(f"Running Dijkstra from {robot.position} to {goal}")

    start = robot.position

    # Priority queue: (distance, position)
    pq = [(0, start)]
    distances: dict[int, int] = {start: 0}
    previous: dict[int, Optional[int]] = {start: None}
    visited: set[int] = set()

    while pq:
        current_dist, current = heapq.heappop(pq)

        if current in visited:
            continue

        visited.add(current)

        if current == goal:
            # Reconstruct path
            path = []
            pos: Optional[int] = goal
            while pos is not None:
                path.append(pos)
                pos = previous[pos]
            path.reverse()

            logger.info(f"Dijkstra found path of length {len(path)}: {path}")
            return path

        for neighbor in robot.grid.get_neighbors(current):
            if neighbor in visited or not robot.grid.is_walkable(neighbor):
                continue

            new_dist = current_dist + 1  # All moves have cost 1

            if neighbor not in distances or new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                previous[neighbor] = current
                heapq.heappush(pq, (new_dist, neighbor))

    logger.warning(f"No path found from {robot.position} to {goal}")
    return None


def navigate(robot: Robot, goal: int) -> bool:
    """
    Navigate robot to goal using Dijkstra's algorithm

    Args:
        robot: Robot instance to navigate
        goal: Target position

    Returns:
        True if goal reached, False otherwise
    """
    path = find_path(robot, goal)

    if not path:
        logger.error("Cannot navigate: no path found")
        return False

    # Execute the path
    for i in range(1, len(path)):
        next_pos = path[i]
        success = robot.move_to(next_pos)

        if not success:
            logger.error(f"Navigation failed at position {robot.position}")
            return False

    logger.info(f"Successfully navigated to goal at position {goal}")
    return True
