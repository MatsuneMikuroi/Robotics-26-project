from utils.PID import PID


def wall_following(ROBOT, NORM_SPEED: float, PID_MAX_DS: float, PID_WALL_TARGET: float,
                   RIGHT_WEIGHTS: list[float], pid: PID) -> (float, float):
    """Wall following behavior using a PID controller. The robot will try to maintain a certain distance from the wall
    while moving forward.

    Args:
        ROBOT: wrapper.Robot - The robot object to control the e-puck.
        NORM_SPEED: float - The base speed of the robot.
        ...: Additional parameters for the wall following behavior (e.g., PID constants, target distance, etc.)

    Returns:
        tuple[float, float] - The left and right wheel speeds to set for the robot.
    """

    # compute proximity value from the right sensors
    prox_values: list[float] = ROBOT.get_calibrate_prox();
    proxR: float = sum([prox_values[i] * RIGHT_WEIGHTS[i] for i in range(len(RIGHT_WEIGHTS))]) / sum(RIGHT_WEIGHTS);

    # compute PID response according to IR sensor value and make it turn towards the wall by default
    ds = pid.compute(proxR, PID_WALL_TARGET) + .05;
    ds = max(-PID_MAX_DS, min(PID_MAX_DS, ds));

    speed_left = max(0.0, NORM_SPEED / 2 - ds);
    speed_right = max(0.0, NORM_SPEED / 2 + ds);
    return (speed_left, speed_right);
