from unifr_api_epuck import wrapper
import time
import sys
import numpy as np

import finalR1 as r1
import blue11 as bluecolor
import rouge11 as redcolor
import vert11 as greencolor
import noir11 as blackcolor


# =========================================
# ROBOT IPS
# =========================================

ROBOT_1_IP = "192.168.2.209"
ROBOT_2_IP = "192.168.2.210"


# =========================================
# ROBOT 1 MISSION
# =========================================

def Robot1Mission():

    try:

        print("[ROBOT 1] Starting mission...")

        mission_robot = r1.GrabberRobot(ROBOT_1_IP)

        mission = mission_robot.run_mission()

        if not mission:

            print("[ROBOT 1] Failed mission")

            return False

        print("[ROBOT 1] Mission completed")

        return True

    except Exception as ex:

        print(f"[Robot 1 ERROR] {ex}")

        return False


# =========================================
# ROBOT 2 MISSION
# =========================================

def Robot2Mission():

    try:

        print("[ROBOT 2] Starting mission...")

        # =====================================
        # CONNECT ROBOT 2
        # =====================================

        robot2 = wrapper.get_robot(ROBOT_2_IP)

        robot2.init_camera("img")

        robot2.initiate_model()

        print("[ROBOT 2] Camera initializing...")

        time.sleep(4)

        # =====================================
        # TAKE IMAGE
        # =====================================

        img = np.array(robot2.get_camera())

        print("[ROBOT 2] Image captured")

        # =====================================
        # DETECTION
        # =====================================

        detections = robot2.get_detection(img)

        print(f"[ROBOT 2] Detections: {detections}")

        object_color = "Unknown"

        for obj in detections:

            print(f"[ROBOT 2] Found label: {obj.label}")

            # =====================================
            # BLACK
            # =====================================

            if "Black" in obj.label:

                object_color = "Black"

                break

            # =====================================
            # BLUE
            # =====================================

            elif "Blue" in obj.label:

                object_color = "Blue"

                break

            # =====================================
            # GREEN
            # =====================================

            elif "Green" in obj.label:

                object_color = "Green"

                break

            # =====================================
            # RED
            # =====================================

            elif "Red" in obj.label:

                object_color = "Red"

                break

        print(f"[ROBOT 2] Detected color: {object_color}")

        # =====================================
        # RUN COLOR MISSION
        # =====================================

        if object_color == "Red":

            print("[ROBOT 2] START RED MISSION")

            red_handler = redcolor.ObjectIsRed(robot2)

            red_handler.run_mission()

        elif object_color == "Blue":

            print("[ROBOT 2] START BLUE MISSION")

            blue_handler = bluecolor.ObjectIsBlue(robot2)

            blue_handler.run_mission()

        elif object_color == "Green":

            print("[ROBOT 2] START GREEN MISSION")

            green_handler = greencolor.ObjectIsGreen(robot2)

            green_handler.run_mission()

        elif object_color == "Black":

            print("[ROBOT 2] START BLACK MISSION")

            black_handler = blackcolor.ObjectIsBlack(robot2)

            black_handler.run_mission()

        else:

            print("[ROBOT 2] NO COLOR DETECTED")

            return False

        print("[ROBOT 2] Mission completed")

        return True

    except Exception as ex:

        print(f"[Robot 2 ERROR] {ex}")

        return False


# =========================================
# MAIN FUNCTION
# =========================================

def main():

    while True:

        # =====================================
        # ROBOT 1
        # =====================================

        success1 = Robot1Mission()

        if not success1:

            print("[MAIN] Robot 1 failed")

            continue

        # =====================================
        # WAIT BEFORE ROBOT 2
        # =====================================

        print("[MAIN] Starting Robot 2 in 2 seconds...")

        time.sleep(2)

        # =====================================
        # ROBOT 2
        # =====================================

        success2 = Robot2Mission()

        if not success2:

            print("[MAIN] Robot 2 failed")

            continue

        # =====================================
        # RESTART LOOP
        # =====================================

        print("[MAIN] Restarting cycle...")

        time.sleep(2)


# =========================================
# RUN PROGRAM
# =========================================

if __name__ == "__main__":

    main()