from utils.SizedLinkedList import SizedLinkedList as SLL


def loverProx(ROBOT, NORM_SPEED: float = 2.0, TH_PROX: float = 250,
              LEFT_WEIGHTS: list[float] | None = None, RIGHT_WEIGHTS: list[float] | None = None,
              sll: SLL | None = None) -> (float, float):
    """Simple LOVER behavior. The robot will try to find the wall and equilibriate itself near it.

    Args:
        ROBOT: epuck_wifi.WifiEpuck - The robot object to control the e-puck.
        NORM_SPEED: float - The base speed of the robot.
        TH_PROX: float - The proximity threshold to determine how close the robot is to an object.
    Returns:
        tuple[float, float] - The left and right wheel speeds to set for the robot.
    """

    prox_values: list[float] = ROBOT.get_calibrate_prox();

    if RIGHT_WEIGHTS is None:
        RIGHT_WEIGHTS = [4, 3, 2, 1, 0, 0, 0, 0];
    if LEFT_WEIGHTS is None:
        LEFT_WEIGHTS = [0, 0, 0, 0, 1, 2, 3, 4];
    # Proximity of the object
    prox_left: float = sum(prox_values[i] * LEFT_WEIGHTS[i] for i in range(8)) / sum(LEFT_WEIGHTS);
    prox_right: float = sum(prox_values[i] * RIGHT_WEIGHTS[i] for i in range(8)) / sum(RIGHT_WEIGHTS);

    if sll is not None:
        sll.append(prox=(prox_left, prox_right));

    # Change of speed
    ds_left: float = (NORM_SPEED * prox_left) / TH_PROX;
    ds_right: float = (NORM_SPEED * prox_right) / TH_PROX;

    # Value of speed
    speed_left: float = NORM_SPEED - ds_left;
    speed_right: float = NORM_SPEED - ds_right;

    return speed_left, speed_right;


def loverColor(block, ROBOT, NORM_SPEED: float = 2.0, 
               CAM_WIDTH: int = 160, CAM_HEIGHT: int = 120,
               EPS_DIST: int = 10, EPS_ROT: int = 5,
               HEIGHT: int = 10, WIDTH: int = 10) -> (float, float):
    """Simple LOVER behavior. The robot will try to find the wall and equilibriate itself near it.

    Args:
        block: DetectedObject - The detected block object containing its position and size in the camera frame.
        ROBOT: epuck_wifi.WifiEpuck - The robot object to control the e-puck.
        NORM_SPEED: float - The base speed of the robot.
        CAM_WIDTH: int - The width of the camera frame in pixels.
        CAM_HEIGHT: int - The height of the camera frame in pixels.
        EPS_DIST: int - The distance threshold in pixels to consider that the robot is close enough to the block to
            stop.
        EPS_ROT: int - The rotation threshold in degrees to consider that the robot is well oriented towards the block
            to stop.
        HEIGHT: int - The height of the block in centimeters at a certain distance (defined experimentally, stop
            condition).
        WIDTH: int - The width of the block in centimeters at a certain distance (defined experimentally, stop
            condition).
    Returns:
        tuple[float, float] - The left and right wheel speeds to set for the robot.
    """

    # Get the position of the block in the camera frame
    block_x: float = block.x_center;
    block_y: float = block.y_center;
    speed_left: float = NORM_SPEED / 2;
    speed_right: float = NORM_SPEED / 2;

    if WIDTH - block.width > EPS_DIST and HEIGHT - block.height > EPS_DIST:
        speed_left += NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);
        speed_right += NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);

    if block.width - WIDTH > EPS_DIST or block.height - HEIGHT > EPS_DIST:
        speed_left -= NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);
        speed_right -= NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);

    if block_x - CAM_WIDTH / 2 > EPS_ROT:
        speed_left += NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);
        speed_right -= NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);
    elif block_x - CAM_WIDTH / 2 < -EPS_ROT:
        speed_left -= NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);
        speed_right += NORM_SPEED * (block_x - CAM_WIDTH / 2) / (CAM_WIDTH / 2);

    return speed_left, speed_right;
