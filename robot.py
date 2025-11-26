"""
Base Robot class for grid navigation.
Provides movement capabilities with safety checks.
"""

from grid import Grid
import logging

logger = logging.getLogger(__name__)


class Robot:
    """Base robot class that navigates the grid with safety checks"""

    def __init__(self, grid: Grid, start_position: int):
        assert grid.is_valid_position(
            start_position
        ), f"Start position {start_position} is out of bounds"
        assert grid.is_walkable(
            start_position
        ), f"Start position {start_position} is blocked by a shelf"

        self.grid = grid
        self.position = start_position
        logger.info(f"Robot initialized at position {start_position}")

    def move_to(self, new_position: int) -> bool:
        """
        Move robot to a new position with safety checks
        Returns True if successful, False otherwise
        """
        assert self.grid.is_valid_position(
            new_position
        ), f"Target position {new_position} is out of bounds"

        # Check if positions are adjacent
        neighbors = self.grid.get_neighbors(self.position)
        assert (
            new_position in neighbors
        ), f"Position {new_position} is not adjacent to current position {self.position}"

        # Check for collision
        if self.grid.is_shelf(new_position):
            logger.warning(f"Cannot move to position {new_position}: shelf detected")
            return False

        old_position = self.position
        self.position = new_position
        logger.info(f"Robot moved from {old_position} to {new_position}")
        return True
