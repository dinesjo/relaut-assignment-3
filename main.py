"""
Main demonstration module for robot navigation.
Demonstrates Dijkstra's algorithm and Bug2 algorithm with various scenarios.
"""

from grid import Grid
from robot import Robot
import dijkstra
import bug2
import logger as log_config

log_config.configure_root_logger()
import logging

logger = logging.getLogger(__name__)

GRID_HEIGHT = 7
GRID_WIDTH = 6


def demonstrate_dijkstra(grid: Grid, start: int, goal: int):
    """Demonstrate Dijkstra's algorithm pathfinding"""
    logger.info("=" * 60)
    logger.info("DEMONSTRATING DIJKSTRA'S ALGORITHM")
    logger.info("=" * 60)

    robot = Robot(grid, start)

    logger.info("Initial grid:")
    print(grid.visualize(robot_pos=robot.position))

    result = dijkstra.find_path(robot, goal)

    if result and isinstance(result, list):
        path = result
        logger.info(f"Path found: {path}")
        logger.info(f"Path length: {len(path)} steps")
        logger.info("Grid with path:")
        print(grid.visualize(path=path, robot_pos=robot.position))
    else:
        logger.error("No path found!")


def demonstrate_dijkstra_all(grid: Grid, start: int):
    """Demonstrate Dijkstra's algorithm finding all reachable positions"""
    logger.info("=" * 60)
    logger.info("DEMONSTRATING DIJKSTRA'S ALGORITHM - ALL REACHABLE POSITIONS")
    logger.info("=" * 60)

    robot = Robot(grid, start)

    logger.info("Initial grid:")
    print(grid.visualize(robot_pos=robot.position))

    result = dijkstra.find_path(robot, goal=None)

    if not result or not isinstance(result, dict):
        logger.error("No reachable positions found!")
        return

    all_paths = result
    logger.info(f"Number of reachable positions: {len(all_paths)}")
    for destination, path in all_paths.items():
        logger.info(f"Path to {destination}: {path} (length: {len(path)} steps)")
    
    for destination, path in all_paths.items():
        assert path[0] == start, f"Path to {destination} does not start at {start}"
        assert path[-1] == destination, f"Path to {destination} does not end at {destination}"
        assert all(
            robot.grid.is_walkable(pos) for pos in path
        ), f"Path to {destination} contains non-walkable positions"


def demonstrate_bug2_success(grid: Grid, start: int, goal: int, obstacle_pos: int):
    """Demonstrate Bug2 algorithm successfully circumnavigating an obstacle"""
    logger.info("=" * 60)
    logger.info("DEMONSTRATING BUG2 - SUCCESSFUL CIRCUMNAVIGATION")
    logger.info("=" * 60)

    robot = Robot(grid, start)

    # First, compute path with Dijkstra (before dynamic obstacle)
    logger.info(f"Computing initial path from {start} to {goal}")
    result = dijkstra.find_path(robot, goal)

    if not result or not isinstance(result, list):
        logger.error("No initial path found!")
        return

    path = result
    logger.info(f"Initial path: {path}")
    logger.info("Grid with planned path:")
    print(grid.visualize(path=path, robot_pos=robot.position))

    # Now place a dynamic obstacle on the path
    logger.info(f"Placing dynamic obstacle at position {obstacle_pos}")
    dynamic_obstacles = {obstacle_pos}

    logger.info("Grid with obstacle:")
    print(grid.visualize(path=path, robot_pos=robot.position))
    print(f"Dynamic obstacle at: {obstacle_pos}")

    # Execute path with obstacle detection
    logger.info("Robot starts executing path...")
    success = bug2.execute_path(robot, path, dynamic_obstacles)

    if success:
        logger.info(f"SUCCESS: Robot reached goal at position {goal}")
        logger.info(f"Final robot position: {robot.position}")
    else:
        logger.error("FAILED: Robot could not reach goal")
        logger.error(f"Final robot position: {robot.position}")


def demonstrate_bug2_blocked(grid: Grid, start: int, goal: int):
    """Demonstrate Bug2 algorithm when path is completely blocked"""
    logger.info("=" * 60)
    logger.info("DEMONSTRATING BUG2 - BLOCKED PATH")
    logger.info("=" * 60)

    # Create a wall of obstacles blocking the path
    logger.info("Creating wall of obstacles to block path")

    # Add obstacles to create a complete wall (entire row 3, positions 13-18)
    wall_obstacles = set()
    for col in range(GRID_WIDTH):
        pos = grid.coords_to_position(2, col)  # row 3 (0-indexed row 2)
        wall_obstacles.add(pos)
        grid.add_shelf(pos)

    logger.info(f"Wall obstacles: {sorted(wall_obstacles)}")

    robot = Robot(grid, start)

    logger.info("Grid with wall:")
    print(grid.visualize(robot_pos=robot.position))

    # Try to navigate to goal
    logger.info(f"Attempting Bug2 navigation from {start} to {goal}")
    success = bug2.navigate(robot, goal)

    if success:
        logger.warning(f"Unexpectedly reached goal at {goal}")
    else:
        logger.info("EXPECTED RESULT: Path is blocked, cannot reach goal")
        logger.info(f"Final robot position: {robot.position}")

    # Clean up obstacles for other demonstrations
    for pos in wall_obstacles:
        grid.remove_shelf(pos)


def main():
    """Main function demonstrating all functionality"""

    # Create grid
    grid = Grid(GRID_HEIGHT, GRID_WIDTH)

    # Add shelves at specified positions
    grid.add_shelf(8)
    grid.add_shelf(9)
    grid.add_shelf(29)
    grid.add_shelf(35)

    logger.info("Initial Grid Configuration:")
    print(grid.visualize())
    print("\nLegend: X = Shelf, R = Robot, * = Path\n")

    # Test Manhattan distance
    logger.info("Testing Manhattan Distance:")
    test_distances = [
        (1, 7, 1),
        (1, 2, 1),
        (1, 8, 2),
        (1, 42, 11),  # Position 1 at (0,0), position 42 at (6,5): |0-6| + |0-5| = 11
    ]

    for pos1, pos2, expected in test_distances:
        distance = grid.manhattan_distance(pos1, pos2)
        logger.info(
            f"Distance from {pos1} to {pos2}: {distance} (expected: {expected})"
        )
        assert (
            distance == expected
        ), f"Distance mismatch: got {distance}, expected {expected}"

    # Demonstration 1: Dijkstra's algorithm
    demonstrate_dijkstra(grid, start=1, goal=42)

    # Demonstration 2: Dijkstra's algorithm - all reachable positions
    demonstrate_dijkstra_all(grid, start=1)

    # Demonstration 3: Bug2 successful circumnavigation
    # Path from 1 to 12, with dynamic obstacle at position 6
    demonstrate_bug2_success(grid, start=1, goal=12, obstacle_pos=6)

    # Demonstration 4: Bug2 with blocked path
    demonstrate_bug2_blocked(grid, start=1, goal=42)

    logger.info("=" * 60)
    logger.info("ALL DEMONSTRATIONS COMPLETED")
    logger.info("=" * 60)


if __name__ == "__main__":
    main()
