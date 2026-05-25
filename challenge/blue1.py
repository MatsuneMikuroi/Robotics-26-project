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

        self.last_direction = "LEFT"

        self.reverse_white_count = 0

        # STATES:
        # FORWARD
        # TURNING
        # SEARCH_LINE_AFTER_TURN
        # REVERSE
        # SMALL_FORWARD
        # RECULE
        # TURNING2
        # SEARCH_LINE_AFTER_TURN2
        # FORWARD2
        # FINAL_FORWARD
        # FINISHED

        self.mode = "FORWARD"

        self.initialize()

    # ========================================
    # INIT
    # ========================================

    def initialize(self):

        print(f"[ROBOT] Initializing {self.ip}")

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
    # NORMAL LINE FOLLOWING
    # ========================================

    def follow_line_normal(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[FORWARD] L={left} \t M={middle} \t R={right}")

        
        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.2 * TURN_SPEED,BASE_SPEED)

            self.last_direction = "LEFT"

        elif right_on_line and not middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.2 * TURN_SPEED)

            self.last_direction = "RIGHT"

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.8 * TURN_SPEED)

        elif right_on_line and middle_on_line and left_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        
        elif not left_on_line and not middle_on_line and not right_on_line:

            print("[MISSION] FIRST WHITE AREA")

            self.turn_start = time.time()

            self.mode = "TURNING"

       
        else:

            if self.last_direction == "LEFT":

                self.robot.set_speed(-SEARCH_SPEED,SEARCH_SPEED)

            else:

                self.robot.set_speed(SEARCH_SPEED,-SEARCH_SPEED)

    # ========================================
    # REVERSE LINE FOLLOWING
    # ========================================

    def follow_line_reverse(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[REVERSE] L={left} \t M={middle} R={right}")

        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif right_on_line and not middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.4 * TURN_SPEED)

        elif left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)

        elif not left_on_line and not middle_on_line and not right_on_line:

            self.reverse_white_count += 1

            print(f"[MISSION] WHITE AREA COUNT = {self.reverse_white_count}")

            if self.reverse_white_count == 1:

                print("[MISSION] SMALL FORWARD")

                self.small_forward_start = time.time()

                self.mode = "SMALL_FORWARD"

            elif self.reverse_white_count == 2:

                print("[MISSION] THIRD WHITE AREA")

                self.robot.set_speed(0, 0)

                self.recule_start = time.time()

                self.mode = "RECULE"

        else:

            self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

    # ========================================
    # FORWARD2
    # ========================================

    def follow_line_forward2(self):

        left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

        print(f"[FORWARD2] L={left} \t M={middle} \t R={right}")

        if left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and not middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.3 * TURN_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(0.2 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(0.5*TURN_SPEED,BASE_SPEED)

        elif not left_on_line and not middle_on_line and not right_on_line:

            print("[MISSION] FOURTH WHITE AREA")

            self.final_forward_start = time.time()

            self.mode = "FINAL_FORWARD"

        elif left_on_line and middle_on_line and right_on_line:

            self.robot.set_speed(0.2 * TURN_SPEED,BASE_SPEED)

        else:

            self.robot.set_speed(-SEARCH_SPEED,SEARCH_SPEED)

    # ========================================
    # MAIN LOOP
    # ========================================

    def ReachGrabArea(self):

        print("[MISSION] START")

        try:

            while self.robot.go_on():

                # ========================================
                # FORWARD
                # ========================================

                if self.mode == "FORWARD":

                    self.follow_line_normal()

                # ========================================
                # FIRST TURN
                # ========================================

                elif self.mode == "TURNING":

                    print("[MISSION] TURNING")

                    self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

                    if time.time() - self.turn_start > 2.8:

                        self.robot.set_speed(0, 0)

                        time.sleep(0.1)

                        self.mode = "SEARCH_LINE_AFTER_TURN"

                # ========================================
                # SEARCH LINE AFTER TURN
                # ========================================

                elif self.mode == "SEARCH_LINE_AFTER_TURN":

                    left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()

                    print("[MISSION] SEARCHING LINE")

                    if left_on_line or middle_on_line or right_on_line:

                        print("[MISSION] LINE FOUND")

                        self.mode = "REVERSE"

                    else:

                        self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

                # ========================================
                # REVERSE
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

                        self.mode = "REVERSE"

                # ========================================
                # RECULE
                # ========================================

                elif self.mode == "RECULE":

                    print("[MISSION] RECULING")

                    self.robot.set_speed(-BASE_SPEED,-BASE_SPEED)

                    
                    if time.time() - self.recule_start > 2.0:

                        self.robot.set_speed(0, 0)

                        time.sleep(0.1)

                        self.turn2_start = time.time()

                        self.mode = "TURNING2"

                # ========================================
                # SECOND TURN
                # ========================================

                elif self.mode == "TURNING2":

                    print("[MISSION] SECOND TURN")

                    self.robot.set_speed(BASE_SPEED,-BASE_SPEED)

                    if time.time() - self.turn2_start > 2.6:

                        self.robot.set_speed(0, 0)

                        time.sleep(0.1)

                        self.mode = "SEARCH_LINE_AFTER_TURN2"

                # ========================================
                # SEARCH LINE AFTER TURN2
                # ========================================

                elif self.mode == "SEARCH_LINE_AFTER_TURN2":

                    left,middle,right,left_on_line,middle_on_line,right_on_line = self.read_ground()
                    print("[MISSION] SEARCHING LINE 2")

                    if left_on_line or middle_on_line or right_on_line:

                        print("[MISSION] LINE FOUND 2")

                        self.mode = "FORWARD2"

                    else:

                        self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

                # ========================================
                # FORWARD2
                # ========================================

                elif self.mode == "FORWARD2":

                    self.follow_line_forward2()

                # ========================================
                # FINAL FORWARD
                # ========================================

                elif self.mode == "FINAL_FORWARD":

                    print("[MISSION] FINAL FORWARD")

                    self.robot.set_speed(BASE_SPEED,BASE_SPEED)

                    if time.time() - self.final_forward_start > 0.5:

                        self.robot.set_speed(0, 0)

                        self.mode = "FINISHED"

                # ========================================
                # FINISHED
                # ========================================

                elif self.mode == "FINISHED":

                    print("[MISSION] FINISHED")

                    break

                time.sleep(0.01)

        except Exception as e:

            print(e)

            return False

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