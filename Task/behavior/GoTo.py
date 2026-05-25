from utils.ShortestPath import ShortestPath
from utils.Block import Block
from utils.Wall import Wall
import numpy as np
import math


def goto(tx: float, ty: float, yaw: float, TARGET: Block = None, NORM_SPEED: float = 2.0,
         WALL: Wall = Wall(), EPS_X: float = 0.1, EPS_Y: float = 0.1,
         SLOWDOWN_DIST: float = 0.15, min_forward: float = 0.0,
         max_turn: float = 1.0) -> (float, float):
    """GoTo behavior. The robot will try to go to the target while avoiding the wall if it is on its way. Uses the yaw
    angle from the ArUco marker for heading estimation."""

    # Handle case if no infos about the robot position are available or if the target is not defined
    if tx is None or ty is None or yaw is None or TARGET is None:
        return 0, 0;

    # Check if the wall is on the way to the target
    if WALL.is_on_the_way(tx, ty, TARGET.x, TARGET.y):
        path = ShortestPath((tx, ty), (TARGET.x, TARGET.y), WALL, EPS_X, EPS_Y);

    else:
        path = (TARGET.x, TARGET.y);

    # Calculate the angle to the target.
    # The ArUco marker's x-axis is 90° CW from the robot's actual forward direction,
    # so the heading vector is (cos(yaw+π/2), sin(yaw+π/2)) = (-sin(yaw), cos(yaw)).
    dx_r = -math.sin(yaw);
    dy_r = math.cos(yaw);
    dx_t = path[0] - tx;
    dy_t = path[1] - ty;
    cross = dx_r * dy_t - dy_r * dx_t;
    dot = dx_r * dx_t + dy_r * dy_t;
    # Signed heading error in [-pi, pi]. Positive means target is on the left -> turn left.
    angle_diff = math.atan2(cross, dot);

    # Calculate the speed to set to the wheels based on the angle difference and the distance to the target
    dist_to_target = np.sqrt((TARGET.x - tx) ** 2 + (TARGET.y - ty) ** 2);
    speed_scale = min(1.0, dist_to_target / SLOWDOWN_DIST);
    forward = max(min_forward, math.cos(angle_diff));  # min_forward prevents pure pivot
    turn = max(-max_turn, min(max_turn, math.sin(angle_diff)));  # capped; >0 = turn left
    speed_left = NORM_SPEED * speed_scale * (forward - turn);
    speed_right = NORM_SPEED * speed_scale * (forward + turn);
    if min_forward > 0:  # guarantee both wheels stay forward when a floor is set
        speed_left = max(0.0, speed_left);
        speed_right = max(0.0, speed_right);

    return speed_left, speed_right;
