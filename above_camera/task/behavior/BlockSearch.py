from utils.Wall import Wall
from utils.Scan import Scan
import numpy as np
import math


def drive_to_waypoint(tx: float, ty: float, yaw: float, target, norm_speed: float) -> tuple[float, float]:
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
    angle_diff = math.atan2(cross, dot)

    # Keep translation while steering to avoid pivot oscillations during scan.
    forward = max(0.55, math.cos(angle_diff))
    turn = max(-0.22, min(0.22, math.sin(angle_diff)))
    speed_left = norm_speed * 0.8 * (forward - turn)
    speed_right = norm_speed * 0.8 * (forward + turn)
    return speed_left, speed_right


def BlockSearch(color: str, ROBOT, tx: float, ty: float, yaw: float,
                scan: Scan, NORM_SPEED: float = 2.0, WALL: Wall = Wall(),
                CAM_WIDTH: int = None, CAM_HEIGHT: int = None,
                EPS_DIST: int = 5, EPS_ROT: int = 8,
                HEIGHT: int = None, WIDTH: int = None,
                eps_y: float = 0.05,
                scan_min_forward: float = 0.0, step: int | None = None) -> (float, float, bool):
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

    # if step is not None and step > 0 and step % 500 == 0:
    #     ROBOT.init_camera()

    img: np.ndarray = np.array(ROBOT.get_camera());
    objects = ROBOT.get_detection(img);

    found: bool = False;
    block = None;
    if objects is not None:
        target_label = f"{color} block".lower();
        block = next((i for i in objects if str(i.label).lower() == target_label), None);

    if block is not None:
        # colorLover approach:
        # 1) Spin to center block horizontally (x_center → cam_width/2).
        # 2) Drive forward to match size; keep gentle heading correction.
        # 3) Also check vertical centering (y_center → cam_height/2).
        x_err = float(block.x_center - CAM_WIDTH / 2);
        y_err = float(block.y_center - CAM_HEIGHT / 2);  # +ve = block below centre = too far

        centered   = abs(x_err) <= EPS_ROT;
        y_centered = abs(y_err) <= EPS_DIST * 3;         # wider tolerance (y_center is noisier)

        close_enough  = block.width  >= WIDTH  - EPS_DIST and block.height >= HEIGHT - EPS_DIST;
        not_too_close = block.width  - WIDTH   <= EPS_DIST and block.height - HEIGHT <= EPS_DIST;
        found = centered and y_centered and close_enough and not_too_close;

        # Block RIGHT (x_err > 0): turn > 0 → left forward, right backward → pivot CW → block centres
        turn = max(-1.0, min(1.0, x_err / (CAM_WIDTH / 2)));
        if centered:
            base = NORM_SPEED * 0.6;
            speed_left  = base + (NORM_SPEED * 0.25 * turn);
            speed_right = base - (NORM_SPEED * 0.25 * turn);
        else:
            # Crawl forward while steering to reduce in-place left/right dithering.
            base = NORM_SPEED * 0.18;
            speed_left  = base + (NORM_SPEED * 0.55 * turn);
            speed_right = base - (NORM_SPEED * 0.55 * turn);

        speed_left = max(-NORM_SPEED, min(NORM_SPEED, speed_left));
        speed_right = max(-NORM_SPEED, min(NORM_SPEED, speed_right));
    else:
        if hasattr(scan, "reached"):
            reached_waypoint = scan.reached(tx, ty, eps_y)
        else:
            reached_waypoint = abs(ty - scan.target_y) < eps_y

        if reached_waypoint:
            scan.step();
        speed_left, speed_right = drive_to_waypoint(
            tx=tx, ty=ty, yaw=yaw,
            target=scan.waypoint, norm_speed=NORM_SPEED,
        )

    return speed_left, speed_right, found;
