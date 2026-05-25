# region Import
from unifr_api_epuck import wrapper
import cv2
from vision import ArUcoCamera
import numpy as np
import math
import sys
import os
import re
import signal
from datetime import datetime

from behavior.Lover import loverProx
from behavior.WallFollowing import wall_following
from behavior.BlockSearch import BlockSearch, drive_to_waypoint
from behavior.GoTo import goto

from utils.SizedLinkedList import SizedLinkedList as SLL
from utils.PID import PID
from utils.State import State
from utils.Wall import Wall
from utils.Block import Block
from utils.Scan import Scan
# endregion


# region Robot and Camera Initialization
# Ensure to have the correct cam, robot and marker IDs as command line arguments
if __name__ == "__main__" and len(sys.argv) >= 4:
    CAM_ID: str = sys.argv[1];
    """Camera ID to connect to. Example: 'cam2'"""
    ROBOT_ID: int = int(sys.argv[2]);
    """Robot ID to control. Example: 208"""
    ARUCO_ID: int = int(sys.argv[3]);
    """ArUco marker ID to track. Default: 0 for robot 208, 1 else"""
else:
    print("Usage: python main.py <camera_rtsp_url> <robot_id> <aruco_id>")
    sys.exit(1)


# Connect to the camera stream
RTSP_URL: str = f"rtsp://192.168.2.150:8554/{CAM_ID}";
""" rtsp url of the camera""";
print(f"Connecting to {RTSP_URL}...");

try:
    camera: ArUcoCamera = ArUcoCamera(RTSP_URL, marker_size_mm=40);
except Exception as e:
    print(f"Error initializing tracking stream: {e}");
    sys.exit(1);


# Initialize robot
ROBOT = wrapper.get_robot(f"192.168.2.{ROBOT_ID}");
"""Robot object to control the e-puck. Initialized with the given robot's ID.""";


def handler(signum, frame):
    ROBOT.clean_up();


signal.signal(signal.SIGINT, handler);


# Create "img" folder and a run-specific subfolder (timestamped) to keep all frames
try:
    os.makedirs("img", exist_ok=True);
    _run_ts: str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S");
    IMG_FOLDER: str = os.path.join("img", _run_ts);
    os.makedirs(IMG_FOLDER, exist_ok=True);
except Exception as e:
    print(f"Error creating img folder for the robot's camera: {e}");
    sys.exit(1);

# Create "logs" folder and open per-state log files (truncated fresh each run, line-buffered)
try:
    os.makedirs("logs", exist_ok=True);
except Exception as e:
    print(f"Error creating logs folder: {e}");
    sys.exit(1);

_ANSI_RE = re.compile(r'\033\[[0-9;]*m');
_log_handles: dict = {
    name: open(f"logs/{name}.txt", "w", buffering=1)
    for name in ["WALL_SEARCH", "WALL_FOLLOW", "GO_TO_BOUND",
                 "TARGET_SEARCH", "GO_TO_ORIGIN", "OBJECTIVE_SEARCH", "GO_TO_TARGET"]
};


def log_state(name: str, msg: str):
    """Print to console (with ANSI colours) and write plain text to logs/{name}.txt."""
    print(msg);
    _log_handles[name].write(_ANSI_RE.sub('', msg) + "\n");


# endregion


# region Camera calibration and transformation matrices

ROTATION_WORLD_CAMERA: np.ndarray = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]);
"""Rotation matrix to transform coordinates from camera frame to world frame."""

POSITION_CAMERA_WORLD: np.ndarray = np.array([[0], [0], [1.1]]);
"""Position of the camera in the world frame. (x, y, z) in meters."""

# endregion


# region Constants for control and mapping

NORM_SPEED: float = 2.0;
"""Base speed of the robot"""
TH_PROX: float = 100;
"""Proximity threshold"""
EPS_PROX: float = 20;
"""Proximity threshold to consider that the robot is close enough to the wall during the wall search behavior"""

MAX_X: float = 0.60;
"""Maximum x coordinate in the world frame."""
MIN_X: float = -0.50;
"""Minimum x coordinate in the world frame."""
EPS_X: float = 0.05;
"""Distance threshold in meters to consider that the robot has reached the x_max/x_min bound during the go to bound
state"""

MAX_Y: float = 0.25;
"""Maximum y coordinate in the world frame."""
MIN_Y: float = -0.36;
"""Minimum y coordinate in the world frame."""
EPS_Y: float = 0.05;
"""Distance threshold in meters to consider that the robot has reached the y_max/y_min bound during the go to bound
state"""

EPS_YAW: float = 0.3;
"""Angle threshold in radians to consider that the robot is well oriented during the go to bound state"""
MAX_DELTA_YAW: float = 0.3;
"""Max yaw change per frame (rad) accepted when accumulating rotation.
Filters ArUco tracking glitches and erratic spikes when the robot collides with the wall."""
ROTATION_MARGIN: float = math.pi / 4;
"""Tolerance margin (rad) subtracted from the 4π threshold (= 45°)."""
WALL_FOLLOW_YAW_THRESHOLD: float = -(4 * math.pi - ROTATION_MARGIN);
"""Yaw threshold (rad) ≈ -11.78 rad. When accumulated clockwise rotation reaches this value
(≈ two full CW loops minus a small margin), WALL_FOLLOW transitions to GO_TO_BOUND.
Empirically: one full perimeter traverse ≈ -11 rad, so this fires just after one complete loop."""

# PID constants and parameters for wall following behavior
PID_MAX_DS: float = 1.5;
"""Max change in speed for the PID controller to avoid too sharp turns"""
PID_WALL_TARGET: float = 100;
"""Target proximity value for the wall following behavior. The robot will try to maintain this distance from the wall.
"""
SENSORS_WEIGHTS_RIGHT: list[float] = [4, 2, 1, 0, 0, 0, 0, 0];
SENSORS_WEIGHTS_LEFT: list[float] = [0, 0, 0, 0, 0, 1, 2, 4];

K: float = 0.0065;
"""Proportional constant for the PID controller in the wall following behavior."""
T_D: float = 0.15;
"""Derivative time constant for the PID controller in the wall following behavior."""
T_I: float = 9999999999;
"""Integral time constant for the PID controller in the wall following behavior.
Set to a very high value to effectively disable the integral term."""

# Robot's camera constants for object detection and tracking
CAM_HEIGHT: int = 120;
"""Height of the camera in pixels"""
CAM_WIDTH: int = 160;
"""Width of the camera in pixels"""
EPS_DIST: int = 5;
"""Distance threshold in pixels to consider that the robot is close enough to the block to stop"""
EPS_ROT: int = 8;
"""Rotation threshold in degrees to consider that the robot is well oriented towards the block to stop"""
TARGET_BLOCK_HEIGHT: int = 123;
"""Height of the target block in pixels at a XXX centimeters from the robot."""
TARGET_BLOCK_WIDTH: int = 109;
"""Width of the target block in pixels at a XXX centimeters from the robot."""
TARGET_COLOR: str = "green";
"""Color of the target block to search for."""
OBJECTIVE_BLOCK_HEIGHT: int = 125;
"""Height of the objective block in pixels at a XXX centimeters from the robot."""
OBJECTIVE_BLOCK_WIDTH: int = 123;
"""Width of the objective block in pixels at a XXX centimeters from the robot."""
OBJECTIVE_COLOR: str = "red";
"""Color of the objective block to search for as the final objective."""

# Occupancy map settings (adapted from calibration workflow)
MAP_RESOLUTION: float = 0.02;
"""Grid cell size in meters for the saved occupancy map."""
MAP_SAVE_EVERY: int = 200;
"""Save map.npy every N detected poses."""
MAP_X_MIN: float = MIN_X;
MAP_X_MAX: float = MAX_X;
MAP_Y_MIN: float = MIN_Y;
MAP_Y_MAX: float = MAX_Y;


def world_to_grid(x: float, y: float) -> tuple[int, int]:
    """Convert world coordinates to grid indices."""
    ix: int = int((x - MAP_X_MIN) / MAP_RESOLUTION);
    iy: int = int((y - MAP_Y_MIN) / MAP_RESOLUTION);
    return ix, iy;


def set_cell_if_empty(grid: np.ndarray, ix: int, iy: int, value: int):
    """Set a grid cell only if the index is valid and currently empty."""
    if 0 <= ix < grid.shape[1] and 0 <= iy < grid.shape[0]:
        if grid[iy, ix] == 0:
            grid[iy, ix] = value;


MAP_NX: int = int((MAP_X_MAX - MAP_X_MIN) / MAP_RESOLUTION);
MAP_NY: int = int((MAP_Y_MAX - MAP_Y_MIN) / MAP_RESOLUTION);
MAP_GRID: np.ndarray = np.zeros((MAP_NY, MAP_NX), dtype=np.uint8);

# endregion


# region Initialize robot and callibrate it

ROBOT.init_sensors();
ROBOT.calibrate_prox();
ROBOT.init_camera("img");
ROBOT.initiate_model();

_, markers = camera.get_marker_positions(draw=False);
if markers and ARUCO_ID in markers.keys():
    _t0 = (ROTATION_WORLD_CAMERA @ np.array(markers[ARUCO_ID]['tvec']).reshape(3, 1) + POSITION_CAMERA_WORLD);
    _R0 = ROTATION_WORLD_CAMERA @ cv2.Rodrigues(markers[ARUCO_ID]['rvec'])[0];
    ORIGIN: Block = Block(float(_t0[0].item()), float(_t0[1].item()),
                          math.atan2(_R0[1, 0], _R0[0, 0]));
    """ Origin position of the robot in the world frame, determined by the initial position of the ArUco marker. """
else:
    ORIGIN: Block = Block(0.0, 0.0, 0.0);
    """ Origin position of the robot in the world frame, determined by the initial position of the ArUco marker. """

# endregion


# region Main loop

state: State = State.WAITING;
end_task: list[bool] = [];

values: SLL = SLL(max_size=20);

wall: Wall = Wall();

pid = PID(K, T_I, T_D);

pid_start_x: float;
pid_start_y: float;
accumulated_yaw: float = 0.0;
"""Signed yaw accumulated since WALL_FOLLOW entry. Right-wall following → always negative. Full loop ≈ −2π."""
prev_yaw_wall: float = 0.0;
"""Previous yaw used to compute the per-frame delta for accumulated_yaw."""
wall_align: bool = False;
"""True while the robot spins CCW (~90°) to put the wall on its right before the PID starts."""
wall_align_start_yaw: float = 0.0;
"""Yaw recorded at WALL_FOLLOW entry to measure the CCW alignment rotation."""

has_mapped_wall: bool = False;

target_scan: Scan = None;
"""Boustrophedon scan object for the TARGET_SEARCH state. Initialized when GO_TO_BOUND ends."""
objective_scan: Scan = None;
"""Boustrophedon scan object for the OBJECTIVE_SEARCH state. Initialized when GO_TO_ORIGIN ends."""

green_block_world_pos: Block = None;
"""World position where the green block was found (saved when TARGET_SEARCH completes)."""


ROBOT.sleep(2);
_frame_failures: int = 0;
_frame_counter: int = 0;
_map_updates: int = 0;

step = 0

while ROBOT.go_on():
    ROBOT.enable_led(0);
    ROBOT.enable_led(4);
    # region tracking infos and logging
    # get tracking infos
    frame, markers = camera.get_marker_positions(draw=False);
    if frame is None:
        _frame_failures += 1;
        print(f"\033[91mFailed to get frame ({_frame_failures}).\033[0m");
        if _frame_failures >= 5:
            print("Reconnecting to camera stream...");
            camera.reconnect();
            _frame_failures = 0;
        else:
            ROBOT.set_speed(0, 0);
        continue
    _frame_failures = 0;
    _frame_counter += 1;
    cv2.imwrite(os.path.join(IMG_FOLDER, f"frame_{_frame_counter:06d}.jpg"), frame);

    tx = None;
    ty = None;
    yaw = None;
    # Log marker data
    if markers:
        if ARUCO_ID in markers.keys():
            tx = markers[ARUCO_ID]['tvec'][0];
            ty = markers[ARUCO_ID]['tvec'][1];

            tvec = np.array(markers[ARUCO_ID]['tvec']);
            t_cm = tvec.reshape(3, 1);

            # Marker position in world frame
            t_wm = ROTATION_WORLD_CAMERA @ t_cm + POSITION_CAMERA_WORLD
            tx = float(t_wm[0].item());
            ty = float(t_wm[1].item());

            # Convert rvec to rotation matrix
            R_cm, _ = cv2.Rodrigues(markers[ARUCO_ID]['rvec']);
            R_wm = ROTATION_WORLD_CAMERA @ R_cm;
            yaw = math.atan2(R_wm[1, 0], R_wm[0, 0]);

            # Update occupancy map at each valid pose detection
            _ix, _iy = world_to_grid(tx, ty);
            set_cell_if_empty(MAP_GRID, _ix, _iy, 1);
            _map_updates += 1;
            if _map_updates % MAP_SAVE_EVERY == 0:
                np.save("map.npy", MAP_GRID);

        elif state not in [State.WAITING, State.WALL_SEARCH]:
            print(f"\033[91mFailed to dectect marker with ID {ARUCO_ID}. Detected IDs: {list(markers.keys())}\033[0m");
            state = State.WAITING;
            continue

    # endregion

    if state == State.WAITING:
        state = len(end_task);

    if (tx is None or ty is None or yaw is None) and state not in [State.WAITING, State.WALL_SEARCH]:
        print("\033[91mNo position data available.\033[0m");
        ROBOT.set_speed(0, 0);
        state = State.WAITING;
        continue

    # region Wall Search
    if state == State.WALL_SEARCH:
        ROBOT.disable_all_led();
        ROBOT.enable_led(1);
        log_state("WALL_SEARCH", f"\033[94m[WALL_SEARCH] Proximity values: {values.getAverage()}, POS=({tx if tx is not None else '?'}, {ty if ty is not None else '?'})\033[0m");
        if values.getAverage()[0] >= TH_PROX and values.getAverage()[1] >= TH_PROX:
            ROBOT.set_speed(0, 0);
            state = State.WALL_FOLLOW;
            end_task.append(True);
            pid_start_x = tx if tx is not None else 0.0;
            pid_start_y = ty if ty is not None else 0.0;
            accumulated_yaw = 0.0;
            prev_yaw_wall = yaw if yaw is not None else 0.0;
            log_state("WALL_SEARCH", f"Wall found at ({tx if tx is not None else '?'}, {ty if ty is not None else '?'}), switching to wall follow")
        else:
            speed_left, speed_right = loverProx(ROBOT=ROBOT,
                                                NORM_SPEED=NORM_SPEED,
                                                TH_PROX=TH_PROX,
                                                sll=values);
            ROBOT.set_speed(speed_left, speed_right);
            continue
    # endregion

    # region Wall Follow
    elif state == State.WALL_FOLLOW:
        ROBOT.disable_all_led();
        ROBOT.enable_led(2);

        # Accumulate clockwise (negative) yaw only
        if yaw is not None:
            delta_yaw = math.atan2(math.sin(yaw - prev_yaw_wall), math.cos(yaw - prev_yaw_wall));
            if abs(delta_yaw) < MAX_DELTA_YAW and delta_yaw < 0:
                accumulated_yaw += delta_yaw;
            prev_yaw_wall = yaw;
        log_state("WALL_FOLLOW", f"\033[93m[WALL_FOLLOW] tx={(tx if tx is not None else 0):.3f} ty={(ty if ty is not None else 0):.3f} acc={accumulated_yaw:.3f}/{WALL_FOLLOW_YAW_THRESHOLD:.1f}\033[0m")
        if accumulated_yaw <= WALL_FOLLOW_YAW_THRESHOLD:
            state = State.TARGET_SEARCH;
            end_task.append(True);
            target_scan = Scan(start_x=MIN_X,
                               x_target=MAX_X,
                               y_min=MIN_Y,
                               y_max=MAX_Y,
                               x_step=0.15,
                               start_going_to_max=False);
            log_state("WALL_FOLLOW", f"Wall follow done (acc={accumulated_yaw:.2f}), starting TARGET_SEARCH scan from x={MIN_X:.2f}")
        else:
            speed_left, speed_right = wall_following(ROBOT=ROBOT,
                                                     NORM_SPEED=NORM_SPEED,
                                                     PID_MAX_DS=PID_MAX_DS,
                                                     PID_WALL_TARGET=PID_WALL_TARGET,
                                                     RIGHT_WEIGHTS=SENSORS_WEIGHTS_RIGHT,
                                                     pid=pid);
            ROBOT.set_speed(speed_left, speed_right);
            if tx is not None and ty is not None:
                wall.update(tx, ty);
            continue
    # endregion

    # region TARGET Search
    elif state == State.TARGET_SEARCH:
        ROBOT.disable_all_led();
        ROBOT.enable_led(3, red=0, green=100, blue=0);
        log_state("TARGET_SEARCH", f"\033[92m[TARGET_SEARCH] tx={tx:.3f} ty={ty:.3f} yaw={yaw:.3f}\033[0m")
        step += 1
        speed_left, speed_right, block_found = BlockSearch(color=TARGET_COLOR, ROBOT=ROBOT, tx=tx, ty=ty, yaw=yaw,
                                                           scan=target_scan, NORM_SPEED=NORM_SPEED, WALL=wall,
                                                           CAM_WIDTH=CAM_WIDTH, CAM_HEIGHT=CAM_HEIGHT,
                                                           EPS_DIST=EPS_DIST, EPS_ROT=EPS_ROT,
                                                           HEIGHT=TARGET_BLOCK_HEIGHT, WIDTH=TARGET_BLOCK_WIDTH,
                                                           eps_y=EPS_Y, step=step);
        if block_found:
            state = State.GO_TO_ORIGIN;
            end_task.append(True);
            green_block_world_pos = Block(tx, ty);
            target_x = (wall.max_x + wall.min_x) / 2;
            target_y = MAX_Y;
            ROBOT.set_speed(0, 0);
            log_state("TARGET_SEARCH", f"Green block found at ({tx:.3f},{ty:.3f}), heading to ({target_x:.3f},{target_y:.3f})")
            step = 0;
        else:
            ROBOT.set_speed(speed_left, speed_right);

    # endregion

    # region Go to origin (from green block position, avoiding wall)
    elif state == State.GO_TO_ORIGIN:
        ROBOT.disable_all_led();
        ROBOT.enable_led(4); ROBOT.enable_led(6);
        log_state("GO_TO_ORIGIN", f"\033[94m[GO_TO_ORIGIN] tx={tx:.3f} ty={ty:.3f} → ({target_x:.3f},{target_y:.3f})\033[0m");
        if ty >= MAX_Y - 0.10:
            objective_scan = Scan(start_x=tx if ORIGIN.yaw < 0 else MIN_X,
                                  x_target=MAX_X,
                                  y_min=MIN_Y,
                                  y_max=MAX_Y,
                                  x_step=0.15,
                                  start_going_to_max=ORIGIN.yaw < 0)
            state = State.OBJECTIVE_SEARCH;
            end_task.append(True);
            ROBOT.set_speed(0, 0);
            log_state("GO_TO_ORIGIN", f"Reached MAX_Y bound, switching to objective search (scan_x={objective_scan.scan_x:.2f})")
        else:
            speed_left, speed_right = goto(tx, ty, yaw, TARGET=Block(target_x, target_y),
                                           NORM_SPEED=NORM_SPEED, WALL=Wall(),
                                           SLOWDOWN_DIST=0.25, min_forward=0.35, max_turn=0.45)
            log_state("GO_TO_ORIGIN", f"  goto L={speed_left:+.2f} R={speed_right:+.2f}")
            ROBOT.set_speed(speed_left, speed_right)
            continue
    # endregion

    # region OBJECTIVE Search (red block)
    elif state == State.OBJECTIVE_SEARCH:
        ROBOT.disable_all_led();
        ROBOT.enable_led(3, red=255, green=0, blue=0);  # mandatory: RGB LED 3 in block color (red)
        log_state("OBJECTIVE_SEARCH", f"\033[91m[OBJECTIVE_SEARCH] tx={tx:.3f} ty={ty:.3f} yaw={yaw:.3f}\033[0m");
        step += 1
        speed_left, speed_right, block_found = BlockSearch(color=OBJECTIVE_COLOR, ROBOT=ROBOT, tx=tx, ty=ty, yaw=yaw,
                                                           scan=objective_scan, NORM_SPEED=NORM_SPEED, WALL=wall,
                                                           CAM_WIDTH=CAM_WIDTH, CAM_HEIGHT=CAM_HEIGHT,
                                                           EPS_DIST=EPS_DIST, EPS_ROT=EPS_ROT,
                                                           HEIGHT=OBJECTIVE_BLOCK_HEIGHT, WIDTH=OBJECTIVE_BLOCK_WIDTH,
                                                           eps_y=EPS_Y, step=step);
        if block_found:
            state = State.GO_TO_TARGET;
            end_task.append(True);
            ROBOT.set_speed(0, 0);
            log_state("OBJECTIVE_SEARCH", f"Red block found at ({tx:.3f},{ty:.3f}), heading to green at ({green_block_world_pos.x:.3f},{green_block_world_pos.y:.3f})");
            step = 0;
        else:
            ROBOT.set_speed(speed_left, speed_right);
    # endregion

    # region Go to target (green block — from red block position)
    elif state == State.GO_TO_TARGET:
        ROBOT.disable_all_led();
        ROBOT.enable_led(0); ROBOT.enable_led(3, red=0, green=100, blue=0);  # custom: front + dim green = navigating to green goal
        log_state("GO_TO_TARGET", f"\033[92m[GO_TO_TARGET] tx={tx:.3f} ty={ty:.3f} → green=({green_block_world_pos.x:.3f},{green_block_world_pos.y:.3f})\033[0m");
        if math.sqrt((tx - green_block_world_pos.x)**2 + (ty - green_block_world_pos.y)**2) < 0.12:
            ROBOT.set_speed(0, 0);
            ROBOT.disable_all_led();
            ROBOT.enable_led(0); ROBOT.enable_led(2); ROBOT.enable_led(4); ROBOT.enable_led(6);  # mandatory: second goal reached
            log_state("GO_TO_TARGET", "Reached green block — task complete.");
            break
        else:
            speed_left, speed_right = goto(tx, ty, yaw, TARGET=green_block_world_pos,
                                           NORM_SPEED=NORM_SPEED, WALL=Wall(),
                                           SLOWDOWN_DIST=0.25, min_forward=0.35, max_turn=0.35);
            ROBOT.set_speed(speed_left, speed_right);
    # endregion

# endregion


# region Clean up
np.save("map.npy", MAP_GRID);
camera.release();
cv2.destroyAllWindows();
ROBOT.clean_up();
# endregion
