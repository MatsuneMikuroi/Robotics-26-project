from unifr_api_epuck import wrapper
import time
import sys

ROBOT_1_IP = "192.168.2.216"

GROUND_SENSOR_THRESHOLD = 650

BASE_SPEED = 2.0
TURN_SPEED = 1.0
SEARCH_SPEED = 1.5

# ========================================
# ROBOT CLASS
# ========================================

class GrabberRobot:

    def __init__(self, ip_address):

        self.ip = ip_address
        self.robot = wrapper.get_robot(ip_address)

        self.state = "INIT"

        self.last_direction = "LEFT"

        # STATES:
        # FORWARD
        # WAIT_LINE
        # REVERSE
        # RECULE
        # TURN_180_FINAL
        # FINISHED
        self.mode = "FORWARD"

        self.initialize()

    # ========================================
    # INITIALIZATION
    # ========================================

    def initialize(self):

        print(f"[ROBOT 1] Initializing robot at {self.ip}...")

        try:

            self.robot.initiate_model()

            self.robot.init_ground()

            self.robot.init_sensors()

            self.state = "READY"

            print("[ROBOT 1] Initialization complete")

        except Exception as e:

            print(f"[ROBOT 1] Initialization error: {e}")

            self.state = "ERROR"

    # ========================================
    # FORWARD LINE FOLLOWING
    # ========================================

    def follow_line(self):

        ground_values = self.robot.get_ground()

        left_sensor = ground_values[0]
        middle_sensor = ground_values[1]
        right_sensor = ground_values[2]

        print(
            f"[FORWARD] "
            f"L={left_sensor} "
            f"M={middle_sensor} "
            f"R={right_sensor}"
        )

        left_on_line = left_sensor < GROUND_SENSOR_THRESHOLD
        middle_on_line = middle_sensor < GROUND_SENSOR_THRESHOLD
        right_on_line = right_sensor < GROUND_SENSOR_THRESHOLD

        # CENTER
        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED, BASE_SPEED)

        # LEFT
        elif left_on_line and not middle_on_line:

            print("[FORWARD] LEFT")

            self.robot.set_speed(
                0.3 * TURN_SPEED,
                BASE_SPEED
            )

            self.last_direction = "LEFT"

        # RIGHT
        elif right_on_line and not middle_on_line:

            print("[FORWARD] RIGHT")

            self.robot.set_speed(
                BASE_SPEED,
                0.5 * TURN_SPEED
            )

            self.last_direction = "RIGHT"

        # DIRECT
        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(
                BASE_SPEED,
                BASE_SPEED
            )

        # SLIGHT RIGHT
        elif right_on_line and middle_on_line:

            self.robot.set_speed(
                0.5 * TURN_SPEED,
                BASE_SPEED
            )

        # LEFT CORRECTION
        elif (
            not right_on_line
            and not middle_on_line
            and left_on_line
        ):

            self.robot.set_speed(
                0.5 * TURN_SPEED,
                BASE_SPEED
            )

        # ========================================
        # FIRST WHITE AREA
        # ========================================

        elif (
            not left_on_line
            and not middle_on_line
            and not right_on_line
        ):

            print("[MISSION] FIRST WHITE AREA")

            # TURN 180°
            self.robot.set_speed(
                BASE_SPEED,
                -BASE_SPEED
            )

            time.sleep(0.1)

            # STOP
            self.robot.set_speed(0, 0)

            time.sleep(0.1)

            # CHANGE STATE
            self.mode = "WAIT_LINE"

            print("[MISSION] WAITING FOR BLACK LINE")

            return

        # SEARCH LINE
        else:

            print("[FORWARD] Searching line")

            if self.last_direction == "LEFT":

                self.robot.set_speed(
                    -SEARCH_SPEED,
                    SEARCH_SPEED
                )

            else:

                self.robot.set_speed(
                    SEARCH_SPEED,
                    -SEARCH_SPEED
                )

    # ========================================
    # REVERSE LINE FOLLOWING
    # ========================================

    def follow_line_reverse(self):

        ground_values = self.robot.get_ground()

        left_sensor = ground_values[0]
        middle_sensor = ground_values[1]
        right_sensor = ground_values[2]

        print(
            f"[REVERSE] "
            f"L={left_sensor} "
            f"M={middle_sensor} "
            f"R={right_sensor}"
        )

        left_on_line = left_sensor < GROUND_SENSOR_THRESHOLD
        middle_on_line = middle_sensor < GROUND_SENSOR_THRESHOLD
        right_on_line = right_sensor < GROUND_SENSOR_THRESHOLD

        # CENTER
        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED, BASE_SPEED)

        # LEFT / RIGHT INVERTED

        elif right_on_line and not middle_on_line:

            print("[REVERSE] LEFT")

            self.robot.set_speed(
                BASE_SPEED,
                0.5 * TURN_SPEED
            )

        elif (
            not left_on_line
            and not middle_on_line
            and right_on_line
        ):

            print("[REVERSE] RIGHT")

            self.robot.set_speed(
                BASE_SPEED,
                0.3 * TURN_SPEED
            )

        elif left_on_line and middle_on_line:

            self.robot.set_speed(
                0.5 * TURN_SPEED,
                BASE_SPEED
            )

        elif (
            not left_on_line
            and right_on_line
            and middle_on_line
        ):

            self.robot.set_speed(
                BASE_SPEED,
                BASE_SPEED
            )

        # ========================================
        # SECOND WHITE AREA
        # ========================================

        elif (
            not left_on_line
            and not middle_on_line
            and not right_on_line
        ):

            print("[MISSION] SECOND WHITE AREA")

            self.recule_start = time.time()

            self.mode = "RECULE"

            return

        # SEARCH LINE
        else:

            print("[REVERSE] Searching line")

            self.robot.set_speed(
                -SEARCH_SPEED,
                +SEARCH_SPEED
            )

    # ========================================
    # MAIN LOOP
    # ========================================

    def ReachGrabArea(self):

        print("[MISSION] Starting mission")

        try:

            while self.robot.go_on():

                # ========================================
                # FORWARD
                # ========================================

                if self.mode == "FORWARD":

                    self.follow_line()

                # ========================================
                # WAIT FOR BLACK LINE
                # ========================================

                elif self.mode == "WAIT_LINE":

                    ground_values = self.robot.get_ground()

                    left_sensor = ground_values[0]
                    middle_sensor = ground_values[1]
                    right_sensor = ground_values[2]

                    left_on_line = (
                        left_sensor < GROUND_SENSOR_THRESHOLD
                    )

                    middle_on_line = (
                        middle_sensor < GROUND_SENSOR_THRESHOLD
                    )

                    right_on_line = (
                        right_sensor < GROUND_SENSOR_THRESHOLD
                    )

                    print(
                        "[WAIT_LINE] Waiting for black line..."
                    )

                    # Line found again
                    if (
                        left_on_line
                        or middle_on_line
                        or right_on_line
                    ):

                        print("[MISSION] LINE FOUND AGAIN")

                        self.mode = "REVERSE"

                    else:

                        print("[WAIT_LINE] Searching black line")

                        # Move slowly
                        self.robot.set_speed(
                            -BASE_SPEED,
                            BASE_SPEED
                        )

                # ========================================
                # REVERSE
                # ========================================

                elif self.mode == "REVERSE":

                    self.follow_line_reverse()

                # ========================================
                # RECULE
                # ========================================

                elif self.mode == "RECULE":

                    print("[MISSION] RECULING")

                    # BACKWARD MOVEMENT
                    self.robot.set_speed(
                        -BASE_SPEED,
                        -BASE_SPEED
                    )

                    # wait a little
                    time.sleep(0.01)

                    # after 3 seconds stop
                    if time.time() - self.recule_start > 3.0:

                        self.robot.set_speed(0, 0)

                        # START FINAL TURN
                        self.turn_final_start = time.time()

                        self.mode = "TURN_180_FINAL"

                # ========================================
                # FINAL 180 TURN
                # ========================================

                elif self.mode == "TURN_180_FINAL":

                    print("[MISSION] FINAL TURN")

                    # TURN
                    self.robot.set_speed(
                        BASE_SPEED,
                        -BASE_SPEED
                    )

                    # turn duration
                    if time.time() - self.turn_final_start > 2.7:

                        # STOP
                        self.robot.set_speed(0, 0)

                        # FINISH
                        self.mode = "FINISHED"

                # ========================================
                # FINISHED
                # ========================================

                elif self.mode == "FINISHED":

                    print("[MISSION] FINISHED")

                    break

                time.sleep(0.01)

        except Exception as e:

            print(f"[ROBOT 1] Error: {e}")

            return False

    # ========================================
    # RUN MISSION
    # ========================================

    def run_mission(self):

        print("[ROBOT 1] Starting mission")

        try:

            if self.state != "READY":

                print("[ROBOT 1] Robot not ready")

                return False

            self.ReachGrabArea()

            return True

        except Exception as e:

            print(f"[MISSION] Error: {e}")

            return False

        finally:

            self.cleanup()

    # ========================================
    # CLEANUP
    # ========================================

    def cleanup(self):

        try:

            self.robot.set_speed(0, 0)

            self.robot.clean_up()

            print("[ROBOT 1] Cleanup complete")

        except:

            pass

# ========================================
# MAIN
# ========================================

def main():

    print("=" * 60)

    print("E-PUCK LINE FOLLOWER")

    print("=" * 60)

    try:

        robot_grabber = GrabberRobot(ROBOT_1_IP)

        success = robot_grabber.run_mission()

        print(f"Mission success = {success}")

        return success

    except Exception as e:

        print(f"[FATAL ERROR] {e}")

        return False

if __name__ == "__main__":

    success = main()

    sys.exit(0 if success else 1)
