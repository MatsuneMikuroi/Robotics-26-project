from unifr_api_epuck import wrapper
import time
import sys

ROBOT_1_IP = "192.168.2.209"

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
        # RECULE
        # TURNING2
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

        return (left,middle,right,left_on_line,middle_on_line,right_on_line)

    # ========================================
    # NORMAL LINE FOLLOWING
    # ========================================

    def follow_line_normal(self):

        (left,middle,right,left_on_line,middle_on_line,right_on_line) = self.read_ground()

        print(f"[FORWARD] "f"L={left} "f"M={middle} "f"R={right}")

        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.2 * TURN_SPEED,BASE_SPEED)

            self.last_direction = "LEFT"

        elif right_on_line and not middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.2 * TURN_SPEED)

            self.last_direction = "RIGHT"

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.5 * TURN_SPEED)
            
        elif left_on_line and middle_on_line and right_on_line:
            
            self.robot.set_speed(0.2*TURN_SPEED, BASE_SPEED)
            
        # FIRST WHITE AREA
        elif (not left_on_line and not middle_on_line and not right_on_line):

            print("[MISSION] FIRST WHITE AREA")

            self.turn_start = time.time()

            self.mode = "TURNING"

        # SEARCH LINE
        else:

            if self.last_direction == "LEFT":

                self.robot.set_speed(-SEARCH_SPEED,SEARCH_SPEED)

            else:

                self.robot.set_speed(SEARCH_SPEED,-SEARCH_SPEED)

    # ========================================
    # REVERSE LINE FOLLOWING
    # ========================================

    def follow_line_reverse(self):

        (left,middle,right,left_on_line,middle_on_line,right_on_line) = self.read_ground()

        print(f"[REVERSE] "f"L={left} "f"M={middle} "f"R={right}")

        if middle_on_line and not left_on_line and not right_on_line:

            self.robot.set_speed(BASE_SPEED,)

        elif right_on_line and not middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,0.3 * TURN_SPEED)

        elif left_on_line and not middle_on_line and not right_on_line:

            self.robot.set_speed(0.3 * TURN_SPEED,BASE_SPEED)

        elif left_on_line and middle_on_line and not right_on_line:

            self.robot.set_speed(0.5 * TURN_SPEED,BASE_SPEED)

        elif right_on_line and middle_on_line and not left_on_line:

            self.robot.set_speed(BASE_SPEED,BASE_SPEED)
        
        elif left_on_line and middle_on_line and right_on_line:
            
            self.robot.set_speed(BASE_SPEED, 0.2*TURN_SPEED)
            
        # WHITE AREA
        elif (not left_on_line and not middle_on_line and not right_on_line ):

            print("[MISSION] SECOND WHITE AREA")

            self.small_forward_start = time.time()

            self.mode = "RECULE"

        # SEARCH BLACK LINE
        else:

            self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

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

                    self.robot.set_speed(BASE_SPEED,-BASE_SPEED)

                    if time.time() - self.turn_start > 2.8:

                        self.robot.set_speed(0, 0)

                        time.sleep(0.1)

                        self.mode = "SEARCH_LINE_AFTER_TURN"

                # ========================================
                # SEARCH LINE AFTER TURN
                # ========================================

                elif self.mode == "SEARCH_LINE_AFTER_TURN":

                    (left,middle,right,left_on_line,middle_on_line,right_on_line) = self.read_ground()

                    print("[MISSION] SEARCHING LINE")

                    if (left_on_line or middle_on_line or right_on_line):

                        print("[MISSION] LINE FOUND")

                        self.mode = "REVERSE"

                    else:

                        self.robot.set_speed(-BASE_SPEED,BASE_SPEED)

                # ========================================
                # REVERSE
                # ========================================

                elif self.mode == "REVERSE":

                    self.follow_line_reverse()

                elif self.mode == "RECULE":

                    print("[MISSION] RECULE")

                    self.robot.set_speed(-BASE_SPEED,-BASE_SPEED)

                    if time.time() - self.small_forward_start > 2:
                        
                        self.turn2_start = time.time()
                        
                        self.mode = "TURNING2"

                # ========================================
                # SECOND TURN
                # ========================================

                elif self.mode == "TURNING2":

                    print("[MISSION] SECOND TURN")

                    self.robot.set_speed(BASE_SPEED,-BASE_SPEED)

                    if time.time() - self.turn2_start > 3:

                        self.robot.set_speed(0, 0)

                        time.sleep(0.1)

                        self.mode = "FINAL_FORWARD"


                elif self.mode == "FINAL_FORWARD":

                    print("[MISSION] FINAL FORWARD")

                    self.robot.set_speed(0,0)

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


