from unifr_api_epuck import wrapper
from vision import ArUcoCamera

import sys
import cv2
import math
import time
import signal
import numpy as np


if len(sys.argv) < 3:
    print("Usage: python final_project.py <camera_id> <robot_id>")
    print("Example: python final_project.py cam2 216")
    sys.exit(1)

CAMERA_ID = sys.argv[1]
ROBOT_ID = sys.argv[2]

RTSP_URL = f"rtsp://192.168.2.150:8554/{CAMERA_ID}"
MY_IP = f"192.168.2.{ROBOT_ID}"

MARKER_ID = 0

print("Connecting robot:", MY_IP)
robot = wrapper.get_robot(MY_IP)

print("Connecting camera:", RTSP_URL)
camera = ArUcoCamera(RTSP_URL, marker_size_mm=40)

robot.init_sensors()
robot.init_camera()
robot.calibrate_prox()
robot.sleep(2)

BASE_SPEED = 2.0

PID_WALL_TARGET = 200
PID_MAX_DS = 1.5

K = 0.004
T_D = 0.0013
T_I = 9999999999

GOAL_DISTANCE = 0.08
TOF_GOAL_THRESHOLD = 60

MAP_SIZE = 120
MAP_SCALE = 50

WAITING = "WAITING"
MAPPING = "MAPPING"
NAVIGATION = "NAVIGATION"
GOAL = "GOAL"

state = WAITING

red_position = None
green_position = None
target_position = None
first_goal_color = None
second_goal_color = None

frame_counter = 0
goal_counter = 0

grid = np.zeros((MAP_SIZE, MAP_SIZE))

R_wc = np.array([
    [1,  0,  0],
    [0, -1,  0],
    [0,  0, -1]
])

C_w = np.array([[0],
                [0],
                [1.1]])

def clean_exit(signum=None, frame=None):
    robot.set_speed(0, 0)
    np.save("map.npy", grid)
    camera.release()
    cv2.destroyAllWindows()
    robot.clean_up()
    print("Clean exit. Map saved as map.npy")
    sys.exit(0)


signal.signal(signal.SIGINT, clean_exit)


class PID:
    TIME_STEP = 64

    def __init__(self, k, t_i, t_d):
        self.error = 0
        self.deriv = 0
        self.integ = 0
        self.K = k
        self.T_I = t_i
        self.T_D = t_d

    def compute(self, prox, target):
        prev_err = self.error
        self.error = prox - target
        self.deriv = (self.error - prev_err) * 1000 / self.TIME_STEP
        self.integ += self.error * self.TIME_STEP / 1000

        p = self.K * self.error
        i = self.K * (self.integ / self.T_I)
        d = self.K * (self.T_D * self.deriv)

        return p + i + d


pid = PID(K, T_I, T_D)


def stop():
    robot.set_speed(0, 0)


def forward():
    robot.set_speed(BASE_SPEED, BASE_SPEED)


def turn_left(speed=1.5):
    robot.set_speed(-speed, speed)


def turn_right(speed=1.5):
    robot.set_speed(speed, -speed)


def get_right_wall_value():
    ps = robot.get_calibrate_prox()

    a = 4
    b = 2
    c = 1
    d = 0

    return (a * ps[0] + b * ps[1] + c * ps[2] + d * ps[3]) / (a + b + c + d)


def front_obstacle():
    ps = robot.get_calibrate_prox()
    return ps[0] > 300 or ps[7] > 300


def wall_follow():
    prox_r = get_right_wall_value()

    ds = pid.compute(prox_r, PID_WALL_TARGET)
    ds += 0.05

    speed_r = BASE_SPEED + ds
    speed_l = BASE_SPEED - ds

    if abs(ds) > PID_MAX_DS:
        speed_r = ds
        speed_l = -ds

    robot.set_speed(speed_l, speed_r)


def avoid_obstacle():
    if front_obstacle():
        turn_left()
        robot.sleep(0.25)
    else:
        wall_follow()

def get_robot_pose():
    frame, markers = camera.get_marker_positions()

    if frame is None:
        return None, None

    if not markers or MARKER_ID not in markers:
        return frame, None

    R_cm, _ = cv2.Rodrigues(markers[MARKER_ID]["rvec"])
    R_wm = R_wc @ R_cm

    yaw = math.atan2(R_wm[1, 0], R_wm[0, 0])

    tvec = np.array(markers[MARKER_ID]["tvec"])
    t_cm = tvec.reshape(3, 1)
    t_wm = R_wc @ t_cm + C_w

    x = float(t_wm[0, 0])
    y = float(t_wm[1, 0])

    return frame, (x, y, yaw)

def world_to_grid(x, y):
    gx = int(MAP_SIZE / 2 + x * MAP_SCALE)
    gy = int(MAP_SIZE / 2 + y * MAP_SCALE)

    gx = max(0, min(MAP_SIZE - 1, gx))
    gy = max(0, min(MAP_SIZE - 1, gy))

    return gx, gy


def update_map(pose, value):
    if pose is None:
        return

    x, y, yaw = pose
    gx, gy = world_to_grid(x, y)
    grid[gy, gx] = value

def search_blocks(min_area=500, y_threshold=40):
    img = np.array(robot.get_camera())
    detections = robot.get_colordetection(img, min_area)

    red_obj = None
    green_obj = None

    for obj in detections:
        if obj.y_center > y_threshold:
            if obj.label == "Red":
                red_obj = obj
            elif obj.label == "Green":
                green_obj = obj

    return red_obj, green_obj


def detect_color():
    red, green = search_blocks()

    if red is not None:
        return "Red"

    if green is not None:
        return "Green"

    return None

def angle_normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def distance_to_target(pose, target):
    x, y, yaw = pose
    tx, ty = target
    return math.sqrt((tx - x) ** 2 + (ty - y) ** 2)


def go_to_target(pose, target):
    x, y, yaw = pose
    tx, ty = target

    target_angle = math.atan2(ty - y, tx - x)
    error = angle_normalize(target_angle - yaw)

    ds = 3.0 * error

    left_speed = BASE_SPEED - ds
    right_speed = BASE_SPEED + ds

    left_speed = max(-4, min(4, left_speed))
    right_speed = max(-4, min(4, right_speed))

    if front_obstacle():
        avoid_obstacle()
    else:
        robot.set_speed(left_speed, right_speed)

def leds_waiting():
    robot.disable_all_led()
    robot.enable_led(0)
    robot.enable_led(4)


def leds_mapping(color):
    robot.disable_all_led()

    robot.enable_led(2)

    if color == "Red":
        robot.enable_led(3, 100, 0, 0)
    elif color == "Green":
        robot.enable_led(3, 0, 100, 0)


def leds_navigation():
    robot.disable_all_led()
    robot.enable_led(2)


def leds_goal():
    robot.enable_all_led()
    robot.enable_body_led()

start_time = time.time()

while robot.go_on():

    frame, pose = get_robot_pose()

    if frame is not None:
        cv2.imshow("Overhead camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            clean_exit()

    if state == WAITING:
        leds_waiting()
        stop()

        if pose is not None and time.time() - start_time > 2:
            print("Start mapping")
            state = MAPPING
          
    elif state == MAPPING:
        update_map(pose, 1)

        color = None

        if frame_counter % 15 == 0:
            color = detect_color()

            if color == "Red" and pose is not None and red_position is None:
                x, y, yaw = pose
                red_position = (x, y)
                update_map(pose, 2)
                print("Red goal detected at:", red_position)

                if first_goal_color is None:
                    first_goal_color = "Red"

            elif color == "Green" and pose is not None and green_position is None:
                x, y, yaw = pose
                green_position = (x, y)
                update_map(pose, 3)
                print("Green goal detected at:", green_position)

                if first_goal_color is None:
                    first_goal_color = "Green"

        leds_mapping(color)

        if front_obstacle():
            update_map(pose, 4)
            avoid_obstacle()
        else:
            wall_follow()

        if red_position is not None and green_position is not None:
            print("Mapping finished")

            if first_goal_color == "Red":
                second_goal_color = "Green"
                target_position = green_position
            else:
                second_goal_color = "Red"
                target_position = red_position

            print("Navigation target:", second_goal_color, target_position)

            robot.sleep(0.5)
            state = NAVIGATION

    elif state == NAVIGATION:
        leds_navigation()
        update_map(pose, 1)

        if pose is None or target_position is None:
            stop()
        else:
            dist = distance_to_target(pose, target_position)
            print("Distance to target:", dist)

            if dist < GOAL_DISTANCE or robot.get_tof() < TOF_GOAL_THRESHOLD:
                goal_counter += 1
                stop()
            else:
                goal_counter = 0
                go_to_target(pose, target_position)

            if goal_counter > 20:
                state = GOAL

    elif state == GOAL:
        print("GOAL REACHED")
        stop()
        leds_goal()
        np.save("map.npy", grid)
        robot.sleep(5)
        break

    frame_counter += 1


clean_exit()
