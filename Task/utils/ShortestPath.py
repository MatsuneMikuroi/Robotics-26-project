from utils.Wall import Wall
import math


def ShortestPath(CURRENT: (float, float), TARGET: (float, float), WALL: Wall,
                 EPS_X: float, EPS_Y: float) -> (float, float):
    """Calculate the shortest path to the target while avoiding the wall.

    Args:
        CURRENT: tuple[float, float] - Current position (x, y)
        TARGET: tuple[float, float] - Target position (x, y)
        WALL: Wall - Wall object representing the wall in the environment, defined by its minimum and maximum x and y
        coordinates.

    Returns:
        tuple[float, float] - Next waypoint to steer toward to avoid the wall.
    """
    if not WALL.is_on_the_way(CURRENT[0], CURRENT[1], TARGET[0], TARGET[1]):
        return TARGET;

    # Try all 4 padded corners of the wall rectangle as candidate waypoints.
    # Pick the one whose path from CURRENT is clear (doesn't cross the wall)
    # and whose total distance (CURRENT->corner->TARGET) is shortest.
    corners = [
        (WALL.min_x - EPS_X, WALL.min_y - EPS_Y),
        (WALL.max_x + EPS_X, WALL.min_y - EPS_Y),
        (WALL.min_x - EPS_X, WALL.max_y + EPS_Y),
        (WALL.max_x + EPS_X, WALL.max_y + EPS_Y),
    ]

    best = None
    best_len = float('inf')
    for c in corners:
        if not WALL.is_on_the_way(CURRENT[0], CURRENT[1], c[0], c[1]):
            total = math.dist(CURRENT, c) + math.dist(c, TARGET)
            if total < best_len:
                best_len = total
                best = c

    if best is None:
        # Robot is inside the wall bounding box (e.g. just after wall-following):
        # any path from inside the box to a corner exits the box, which Liang-Barsky
        # flags as "blocked". Head toward the nearest corner to exit the box first;
        # normal avoidance resumes once the robot is outside.
        return min(corners, key=lambda c: math.dist(CURRENT, c))

    return best
