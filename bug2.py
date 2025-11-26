"""
Bug2 algorithm for robot navigation with dynamic obstacle avoidance.
Implements boundary following when obstacles are encountered.
"""

from typing import List, Set, Optional
from robot import Robot
import logging

logger = logging.getLogger(__name__)


def navigate(
    robot: Robot, goal: int, dynamic_obstacles: Optional[Set[int]] = None
) -> bool:
    """
    Navigate to goal using Bug2 algorithm with dynamic obstacle detection

    Bug2 algorithm:
    1. Move toward goal on the m-line (straight line to goal)
    2. If obstacle encountered, enter boundary following mode
    3. Follow obstacle boundary until m-line is reached at a point closer to goal
    4. Resume moving toward goal

    Args:
        robot: Robot instance to navigate
        goal: Target position
        dynamic_obstacles: Set of positions with dynamic obstacles

    Returns:
        True if goal reached, False if path blocked
    """
    assert robot.grid.is_valid_position(goal), f"Goal {goal} is out of bounds"

    if dynamic_obstacles is None:
        dynamic_obstacles = set()

    logger.info(f"Starting Bug2 navigation from {robot.position} to {goal}")
    logger.info(f"Dynamic obstacles at: {dynamic_obstacles}")

    max_iterations = robot.grid.total_cells * 2
    iteration = 0

    # Calculate initial m-line distance
    def get_m_line_distance(pos: int) -> int:
        return robot.grid.manhattan_distance(pos, goal)

    while robot.position != goal and iteration < max_iterations:
        iteration += 1

        # Check if we've reached the goal
        if robot.position == goal:
            logger.info(f"Bug2: Reached goal at position {goal}")
            return True

        # Get neighbors sorted by distance to goal (greedy approach on m-line)
        neighbors = robot.grid.get_neighbors(robot.position)
        valid_neighbors = [
            n
            for n in neighbors
            if robot.grid.is_valid_position(n)
            and n not in robot.grid.shelves
            and n not in dynamic_obstacles
        ]

        if not valid_neighbors:
            logger.error("Bug2: No valid neighbors, path is blocked")
            return False

        # Sort by Manhattan distance to goal (m-line heuristic)
        valid_neighbors.sort(key=lambda n: get_m_line_distance(n))

        # Try to move toward goal (best neighbor)
        best_neighbor = valid_neighbors[0]
        current_m_distance = get_m_line_distance(robot.position)
        best_m_distance = get_m_line_distance(best_neighbor)

        # Check if obstacle is blocking the direct path
        if best_m_distance >= current_m_distance and len(valid_neighbors) > 1:
            # We're hitting an obstacle, enter boundary following mode
            logger.info(f"Bug2: Obstacle detected, entering boundary following mode")

            # Try to circumnavigate by following boundary
            # Choose alternative neighbor (wall following)
            for alt_neighbor in valid_neighbors[1:]:
                if robot.move_to(alt_neighbor):
                    logger.info(f"Bug2: Following boundary to {alt_neighbor}")
                    break
            else:
                logger.error("Bug2: Cannot follow boundary, path blocked")
                return False
        else:
            # Move toward goal
            if robot.move_to(best_neighbor):
                logger.info(f"Bug2: Moving toward goal to {best_neighbor}")
            else:
                logger.error(f"Bug2: Cannot move to {best_neighbor}")
                return False

    if robot.position == goal:
        logger.info(f"Bug2: Successfully reached goal at position {goal}")
        return True
    else:
        logger.error(f"Bug2: Max iterations reached without finding goal")
        return False


def execute_path(
    robot: Robot, path: List[int], dynamic_obstacles: Optional[Set[int]] = None
) -> bool:
    """
    Execute a precomputed path with dynamic obstacle detection
    If obstacle detected, switch to Bug2 algorithm

    Args:
        robot: Robot instance
        path: Precomputed path to execute
        dynamic_obstacles: Set of dynamic obstacle positions

    Returns:
        True if goal reached, False otherwise
    """
    assert path, "Path cannot be empty"
    assert (
        path[0] == robot.position
    ), f"Path must start from current position {robot.position}, got {path[0]}"

    if dynamic_obstacles is None:
        dynamic_obstacles = set()

    logger.info(f"Executing path: {path}")
    logger.info(f"Dynamic obstacles: {dynamic_obstacles}")

    goal = path[-1]

    for i in range(1, len(path)):
        next_pos = path[i]

        # Check for obstacle before moving
        if next_pos in dynamic_obstacles or robot.grid.is_shelf(next_pos):
            logger.warning(f"Obstacle detected at position {next_pos}!")
            logger.info("Switching to Bug2 algorithm for obstacle avoidance")

            # Switch to Bug2 to circumnavigate
            return navigate(robot, goal, dynamic_obstacles)

        # Move to next position
        success = robot.move_to(next_pos)
        assert success, f"Failed to move to position {next_pos}"

    logger.info(f"Successfully executed path, reached goal at {goal}")
    return True
