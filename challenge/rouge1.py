from unifr_api_epuck import wrapper
import time
import sys

ROBOT_1_IP = "192.168.2.210"

GROUND_SENSOR_THRESHOLD = 650

BASE_SPEED = 2.0
TURN_SPEED = 1.0
SEARCH_SPEED = 1.5


class GrabberRobot:

    def __init__(self, ip_address):

        self.ip = ip_address
        self.robot = wrapper.get_robot(ip_address)

        self.state = "INIT"

        self.reverse_white_count = 0
        self.home_white_count = 0

        # STATES:
        # FORWARD
        # TURNING
        # SEARCH_LINE
        # REVERSE
        # SMALL_FORWARD
        # REVERSE2
        # THIRD_FORWARD
        # REVERSE3
        # RECULE
        # TURNING2
        # HOME
        # HOME_FORWARD
        # HOME_REVERSE2
        # FINAL_STOP_FORWARD
        # FINISHED

        self.mode = "FORWARD"

        self.initialize()

    # ========================================
    # INIT
    # ========================================

    def initialize(self):

        try:

            self.robot.initiate_model()

            self.robot.init_ground()

            self.robot.init_sensors()

            self.state = "READY"

            print("[ROBOT] READY")

        except Exception as e:

            print(e)

            self.state = "ERROR"

    # ========================================
    # READ GROUND
    # ========================================

    def read_ground(self):

        g = self.robot.get_ground()

        left = g[0]
        middle = g[1]
        right = g[2]

        left_on_line = left < GROUND_SENSOR_THRESHOLD
        middle_on_line = middle < GROUND_SENSOR_THRESHOLD
        right_on_line = right < GROUND_SENSOR_THRESHOLD

        return left,middle,right,left_on_line,middle_on_line,right_on_line

    # ========================================
    # FORWARD
    # ========================================

    def follow_line_forward(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[FORWARD] L={left} \t M={middle} \t R={right}")

        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.3 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and not middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)
            
        elif right_on_line and middle_on_line and left_on_line:
            
            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        # FIRST WHITE
        elif not left_on_line and not middle_on_line and not right_on_line:

            print("[MISSION] FIRST WHITE")

            self.turn_start = time.time()

            self.mode = "TURNING"

        else:

            self.robot.set_speed(-SEARCH_SPEED,SEARCH_SPEED)

    # ========================================
    # REVERSE1
    # ========================================

    def follow_line_reverse(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[REVERSE1] L={left} \t M={middle} \t R={right}")

        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif right_on_line and not middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(0.3 * TURN_SPEED,BASE_SPEED)

        elif not left_on_line and not middle_on_line and not right_on_line:

            self.reverse_white_count += 1

            print(f"[MISSION] WHITE COUNT = {self.reverse_white_count}")

            if self.reverse_white_count == 1:

                print("[MISSION] SMALL FORWARD")

                self.small_forward_start = time.time()

                self.mode = "SMALL_FORWARD"

        else:

            self.robot.set_speed(
                -BASE_SPEED,
                BASE_SPEED
            )

    # ========================================
    # REVERSE2
    # ========================================

    def follow_line_reverse2(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[REVERSE2] L={left} \t M={middle} \t R={right}")

        if left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif not left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)

        elif not left_on_line and not middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,0.2 * TURN_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.2 * TURN_SPEED,BASE_SPEED)

        elif not left_on_line and not middle_on_line and not right_on_line:

            print("[MISSION] THIRD WHITE")

            self.third_forward_start = time.time()

            self.mode = "THIRD_FORWARD"

        else:

            self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

    # ========================================
    # REVERSE3
    # ========================================

    def follow_line_reverse3(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[REVERSE3] L={left} \t M={middle} \t R={right}")

        if not left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(0.3 * TURN_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif not left_on_line and not middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)

        elif not left_on_line and not middle_on_line and not right_on_line:

            print("[MISSION] RECULE")

            self.recule_start = time.time()

            self.mode = "RECULE"

        else:

            self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

    # ========================================
    # HOME
    # ========================================

    def follow_line_home(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[HOME] L={left} \t M={middle} \t R={right}")

        if left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.3 * TURN_SPEED)

        elif left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,0.3 * TURN_SPEED)

        elif not left_on_line and not middle_on_line and not right_on_line:

            print("[MISSION] HOME WHITE")

            self.home_white_count += 1

            if self.home_white_count == 1:

                self.home_forward_start = time.time()

                self.mode = "HOME_FORWARD"

        else:

            self.robot.set_speed(SEARCH_SPEED,-SEARCH_SPEED)

    # ========================================
    # HOME REVERSE2
    # ========================================

    def follow_line_home_reverse2(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[HOME REVERSE2] L={left} \t M={middle} \t R={right}")

        if left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(0.5*TURN_SPEED,BASE_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.3 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)
        
        elif not left_on_line and not middle_on_line and right_on_line:
            
            self.robot.set_speed(BASE_SPEED, 0.3*TURN_SPEED)
            
        # 6eme blanc
        elif not left_on_line and not middle_on_line and not right_on_line:

            print("[MISSION] FINAL WHITE")

            self.final_stop_start = time.time()

            self.mode = "FINAL_STOP_FORWARD"

        else:

            self.robot.set_speed(SEARCH_SPEED,-SEARCH_SPEED)

    # ========================================
    # MAIN LOOP
    # ========================================

    def ReachGrabArea(self):

        print("[MISSION] START")

        while self.robot.go_on():

            # ========================================
            # FORWARD
            # ========================================

            if self.mode == "FORWARD":

                self.follow_line_forward()

            # ========================================
            # TURNING
            # ========================================

            elif self.mode == "TURNING":

                print("[MISSION] TURNING")

                self.robot.set_speed(BASE_SPEED,-BASE_SPEED)

                if time.time() - self.turn_start > 2.8:

                    self.robot.set_speed(0, 0)

                    time.sleep(0.1)

                    self.mode = "SEARCH_LINE"

            # ========================================
            # SEARCH LINE
            # ========================================

            elif self.mode == "SEARCH_LINE":

                left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

                print("[MISSION] SEARCHING LINE")

                if left_on_line or middle_on_line or right_on_line:

                    print("[MISSION] LINE FOUND")

                    self.mode = "REVERSE"

                else:

                    self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

            # ========================================
            # REVERSE1
            # ========================================

            elif self.mode == "REVERSE":

                self.follow_line_reverse()

            # ========================================
            # SMALL FORWARD
            # ========================================

            elif self.mode == "SMALL_FORWARD":

                print("[MISSION] SMALL FORWARD")

                self.robot.set_speed(BASE_SPEED,BASE_SPEED)

                if time.time() - self.small_forward_start > 1:

                    self.mode = "REVERSE2"

            # ========================================
            # REVERSE2
            # ========================================

            elif self.mode == "REVERSE2":

                self.follow_line_reverse2()

            # ========================================
            # THIRD FORWARD
            # ========================================

            elif self.mode == "THIRD_FORWARD":

                print("[MISSION] THIRD FORWARD")

                self.robot.set_speed(BASE_SPEED,BASE_SPEED)

                if time.time() - self.third_forward_start > 1:

                    self.mode = "REVERSE3"

            # ========================================
            # REVERSE3
            # ========================================

            elif self.mode == "REVERSE3":

                self.follow_line_reverse3()

            # ========================================
            # RECULE
            # ========================================

            elif self.mode == "RECULE":

                print("[MISSION] RECULE")

                self.robot.set_speed(-BASE_SPEED,-BASE_SPEED)

                if time.time() - self.recule_start > 2:

                    self.turn2_start = time.time()

                    self.mode = "TURNING2"

            # ========================================
            # TURNING2
            # ========================================

            elif self.mode == "TURNING2":

                print("[MISSION] TURNING 180")

                self.robot.set_speed(BASE_SPEED,-BASE_SPEED)

                if time.time() - self.turn2_start > 2.9:

                    self.mode = "HOME"

            # ========================================
            # HOME
            # ========================================

            elif self.mode == "HOME":

                self.follow_line_home()

            # ========================================
            # HOME FORWARD
            # ========================================

            elif self.mode == "HOME_FORWARD":

                print("[MISSION] HOME FORWARD")

                self.robot.set_speed(BASE_SPEED,BASE_SPEED)

                if time.time() - self.home_forward_start > 1:

                    self.mode = "HOME_REVERSE2"

            # ========================================
            # HOME REVERSE2
            # ========================================

            elif self.mode == "HOME_REVERSE2":

                self.follow_line_home_reverse2()

            # ========================================
            # FINAL STOP FORWARD
            # ========================================

            elif self.mode == "FINAL_STOP_FORWARD":

                print("[MISSION] FINAL STOP FORWARD")

                self.robot.set_speed(BASE_SPEED,BASE_SPEED)

                if time.time() - self.final_stop_start > 1:

                    self.robot.set_speed(0, 0)

                    self.mode = "FINISHED"

            # ========================================
            # FINISHED
            # ========================================

            elif self.mode == "FINISHED":

                print("[MISSION] FINISHED")

                break

            time.sleep(0.01)

    # ========================================
    # RUN
    # ========================================

    def run_mission(self):

        try:

            if self.state != "READY":

                return False

            self.ReachGrabArea()

            return True

        finally:

            self.cleanup()

    # ========================================
    # CLEANUP
    # ========================================

    def cleanup(self):

        try:

            self.robot.set_speed(0, 0)

            self.robot.clean_up()

        except:

            pass


# ========================================
# MAIN
# ========================================

def main():

    try:

        robot = GrabberRobot(ROBOT_1_IP)

        success = robot.run_mission()

        return success

    except Exception as e:

        print(e)

        return False


if __name__ == "__main__":

    success = main()

    sys.exit(0 if success else 1)