from behavior.GoTo import goto
from utils.Wall import Wall
from utils.Scan import Scan
import numpy as np
import math


def _drive_to_waypoint_lane(tx: float, ty: float, yaw: float, target, norm_speed: float) -> tuple[float, float]:
    """Rotate-first lane controller for practical scanning.

    Compared to min_forward-based motion, this avoids forced forward drift while
    heading is still far from the waypoint direction.
    """
    dx_r = -math.sin(yaw)
    dy_r = math.cos(yaw)
    dx_t = target.x - tx
    dy_t = target.y - ty
    cross = dx_r * dy_t - dy_r * dx_t
    dot = dx_r * dx_t + dy_r * dy_t
    angle_diff = -math.atan2(cross, dot)

    # If heading error is large, rotate in place first.
    if abs(angle_diff) > 0.50:
        turn = max(-1.0, min(1.0, angle_diff / (math.pi / 2)))
        speed_left = -norm_speed * 0.45 * turn
        speed_right = norm_speed * 0.45 * turn
        return speed_left, speed_right

    # Once mostly aligned, move forward with gentle steering.
    steer = max(-0.6, min(0.6, math.sin(angle_diff)))
    base = norm_speed * 0.7
    speed_left = base * (1.0 - steer)
    speed_right = base * (1.0 + steer)
    return speed_left, speed_right


def BlockSearch(color: str, ROBOT, tx: float, ty: float, yaw: float,
                scan: Scan, NORM_SPEED: float = 2.0, WALL: Wall = Wall(),
                CAM_WIDTH: int = None, CAM_HEIGHT: int = None,
                EPS_DIST: int = 5, EPS_ROT: int = 8,
                HEIGHT: int = None, WIDTH: int = None,
                eps_y: float = 0.05,
                scan_min_forward: float = 0.0) -> (float, float, bool):
    """Block search behavior. The robot will follow a loverColor behavior when the target block
    is visible in camera. Otherwise it performs a boustrophedon scan: sweeping y_max<->y_min and
    stepping in x after each sweep, toward x_min (green) or x_max (red).

    Returns:
        (speed_left, speed_right, found) — found is True when the block is detected and
        the robot has reached the target size (close enough and centered).
    """

    if tx is None or ty is None:
        return 0, 0, False;

    if scan.is_done:
        raise SystemExit(f"Search area fully covered, no {color} block found.");

    img: np.ndarray = np.array(ROBOT.get_camera());
    objects = ROBOT.get_detection(img);

    found: bool = False;
    block = None;
    if objects is not None:
        target_label = f"{color} block".lower();
        block = next((i for i in objects if str(i.label).lower() == target_label), None);

    if block is not None:
        # Practical approach controller:
        # 1) Rotate until block is centered in the image.
        # 2) Drive forward while keeping a small steering correction.
        center_error = float(block.x_center - CAM_WIDTH / 2);
        centered = abs(center_error) <= EPS_ROT;

        close_enough = block.width >= WIDTH - EPS_DIST and block.height >= HEIGHT - EPS_DIST;
        not_too_close = block.width - WIDTH <= EPS_DIST and block.height - HEIGHT <= EPS_DIST;
        found = centered and close_enough and not_too_close;

        turn = max(-1.0, min(1.0, center_error / (CAM_WIDTH / 2)));
        if centered:
            base = NORM_SPEED * 0.6;
            speed_left = base - (NORM_SPEED * 0.25 * turn);
            speed_right = base + (NORM_SPEED * 0.25 * turn);
        else:
            speed_left = -NORM_SPEED * 0.5 * turn;
            speed_right = NORM_SPEED * 0.5 * turn;

        speed_left = max(-NORM_SPEED, min(NORM_SPEED, speed_left));
        speed_right = max(-NORM_SPEED, min(NORM_SPEED, speed_right));
    else:
        if hasattr(scan, "reached"):
            reached_waypoint = scan.reached(tx, ty, eps_y)
        else:
            reached_waypoint = abs(ty - scan.target_y) < eps_y

        if reached_waypoint:
            scan.step();
        if hasattr(scan, "reached"):
            speed_left, speed_right = _drive_to_waypoint_lane(
                tx=tx,
                ty=ty,
                yaw=yaw,
                target=scan.waypoint,
                norm_speed=NORM_SPEED,
            )
        else:
            speed_left, speed_right = goto(tx, ty, yaw,
                                           TARGET=scan.waypoint, NORM_SPEED=NORM_SPEED, WALL=WALL,
                                           min_forward=scan_min_forward);

    return speed_left, speed_right, found;
