"""
Base Robot class for grid navigation.
Provides movement capabilities with safety checks and messaging.
"""

from grid import Grid
import logging
import time
from typing import Any, Dict, List, Optional, Tuple
from enum import Enum

logger = logging.getLogger(__name__)


class MessageType(Enum):
    """Message types for robot-to-robot communication."""
    COLLISION_DETECTED = "collision_detected"
    COLLISION_ACK = "collision_ack"
    FINISHED = "finished"


class Robot:
    """Base robot class that navigates the grid with safety checks and messaging."""

    # Class-level message queue and registry
    _message_queue: Dict[int, List[Tuple[int, MessageType, Any]]] = {}
    _registry: Dict[int, "Robot"] = {}
    _next_robot_id = 1

    def __init__(self, grid: Grid, start_position: int, robot_id: Optional[int] = None):
        assert grid.is_valid_position(
            start_position
        ), f"Start position {start_position} is out of bounds"
        assert grid.is_walkable(
            start_position
        ), f"Start position {start_position} is blocked by a shelf"

        self.grid = grid
        self.position = start_position
        
        # Assign robot ID
        if robot_id is None:
            robot_id = Robot._next_robot_id
            Robot._next_robot_id += 1
        self.robot_id = robot_id
        
        # Initialize private obstacle map (for dynamic obstacles detected at runtime)
        self.private_obstacles: set = set()
        
        # Register robot and message queue
        Robot._registry[self.robot_id] = self
        Robot._message_queue[self.robot_id] = []
        
        logger.info(f"Robot {self.robot_id} initialized at position {start_position}")

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

        # Check for collision with shelves
        if self.grid.is_shelf(new_position):
            logger.warning(f"Robot {self.robot_id}: Cannot move to {new_position}: shelf detected")
            return False

        # Check private obstacle map (includes obstacles added by this robot during collision avoidance)
        if new_position in self.private_obstacles:
            logger.warning(f"Robot {self.robot_id}: Cannot move to {new_position}: in private obstacle map")
            return False

        old_position = self.position
        self.position = new_position
        logger.info(f"Robot {self.robot_id} moved from {old_position} to {new_position}")
        return True

    # --- Messaging API ---
    def send_message(self, target_id: int, msg_type: MessageType, payload: Any = None) -> None:
        """Send a message to another robot."""
        if target_id in Robot._message_queue:
            Robot._message_queue[target_id].append((self.robot_id, msg_type, payload))
            logger.info(f"Robot {self.robot_id}: sent {msg_type.value} to Robot {target_id}")

    def receive_message(self, msg_type: Optional[MessageType] = None, timeout: float = 1.0) -> Optional[Tuple[int, MessageType, Any]]:
        """
        Receive a message of a specific type, or any message if msg_type is None.
        Returns (sender_id, msg_type, payload) or None on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            queue = Robot._message_queue.get(self.robot_id, [])
            if msg_type is None:
                if queue:
                    msg = queue.pop(0)
                    logger.info(f"Robot {self.robot_id}: received {msg[1].value} from Robot {msg[0]}")
                    return msg
            else:
                for i, (sender_id, mtype, payload) in enumerate(queue):
                    if mtype == msg_type:
                        queue.pop(i)
                        logger.info(f"Robot {self.robot_id}: received {msg_type.value} from Robot {sender_id}")
                        return (sender_id, mtype, payload)
            time.sleep(0.01)
        return None

    @classmethod
    def get_robot_at(cls, position: int) -> Optional["Robot"]:
        """Get the robot at a given position, if any."""
        for robot in cls._registry.values():
            if robot.position == position:
                return robot
        return None

    def collision_protocol(self, other_robot: "Robot") -> bool:
        """
        Execute collision detection and ordering protocol.
        Returns True if this robot should go first (leader), False if it should wait.
        
        Protocol:
        - Send collision message with (timestamp, position)
        - Receive other's collision message
        - Compare: lower timestamp wins, tie-break by lower position
        - Both robots acknowledge the order
        """
        timestamp = time.time()
        message_payload = (timestamp, self.position)
        
        logger.info(f"Robot {self.robot_id}: sending collision message to Robot {other_robot.robot_id}")
        self.send_message(other_robot.robot_id, MessageType.COLLISION_DETECTED, message_payload)
        
        # Wait for other robot's collision message
        other_msg = self.receive_message(MessageType.COLLISION_DETECTED, timeout=1.0)
        if other_msg is None:
            logger.warning(f"Robot {self.robot_id}: no collision response from {other_robot.robot_id}, assuming leader")
            return True
        
        other_robot_id, _, other_payload = other_msg
        other_timestamp, other_position = other_payload
        
        # Determine order: lower timestamp wins, tie-break by lower position
        if other_timestamp < timestamp or (other_timestamp == timestamp and other_position < self.position):
            # Other robot goes first
            logger.info(f"Robot {self.robot_id}: other robot goes first (ts {other_timestamp} vs {timestamp}, pos {other_position} vs {self.position})")
            self.send_message(other_robot_id, MessageType.COLLISION_ACK, None)
            return False
        else:
            # This robot goes first
            logger.info(f"Robot {self.robot_id}: this robot goes first (ts {timestamp} vs {other_timestamp}, pos {self.position} vs {other_position})")
            self.send_message(other_robot_id, MessageType.COLLISION_ACK, None)
            return True
