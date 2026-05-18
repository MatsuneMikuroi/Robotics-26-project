from unifr_api_epuck import wrapper
import numpy as np
import os
import time
import signal
import sys

# ========================================
# GLOBAL CONFIGURATION
# ========================================

# Robot IP Addresses (CONFIGURE THESE FOR YOUR ROBOTS)
ROBOT_1_IP = '192.168.2.206'  # Grabber Robot
ROBOT_2_IP = '192.168.2.207'  # Sorting Robot

# ========================================
# LINE FOLLOWING PARAMETERS
# ========================================

#Adjust speed according to what u want
BASE_SPEED = 2.0
TURN_SPEED = 1.0


# ========================================
# COLOR DETECTION PARAMETERS
# ========================================

AREA_THRESHOLD_MIN = 5000
DETECTION_CONFIDENCE = 0.9 # needed to be added


#=========================================
#Gray Color Detection Setting
#=========================================

GRAY_MIN = 700
GRAY_MAX = 850


class PIDLineFollower:

    def __init__(self, robot):

        self.robot = robot

        # PID constants
        self.Kp = 25
        self.Kd = 10

        # previous error for derivative term
        self.previous_error = 0

        # base speed
        self.BASE_SPEED = 200

        self.THRESHOLD = 500

    def follow_line_pid(self):

        # Read sensors
        ground = self.robot.get_ground()

        left = ground[0]
        middle = ground[1]
        right = ground[2]

        # Detect black line
        left_detected = left < self.THRESHOLD
        middle_detected = middle < self.THRESHOLD
        right_detected = right < self.THRESHOLD

        # Compute error
        error = 0

        if left_detected:
            error -= 1

        if right_detected:
            error += 1

        # Special case: centered
        if middle_detected and not left_detected and not right_detected:
            error = 0

        # PID terms
        proportional = error

        derivative = error - self.previous_error

        correction = (
            self.Kp * proportional +
            self.Kd * derivative
        )

        # Save error
        self.previous_error = error

        # Motor speeds
        left_speed = self.BASE_SPEED - correction
        right_speed = self.BASE_SPEED + correction

        # Apply speeds
        self.robot.set_speed(left_speed, right_speed)

    def follow_line_pid_backward(self):

        # Read sensors
        ground = self.robot.get_ground()

        left = ground[0]
        middle = ground[1]
        right = ground[2]

        # Detect black line
        left_detected = left < self.THRESHOLD
        middle_detected = middle < self.THRESHOLD
        right_detected = right < self.THRESHOLD

        # Compute error
        error = 0

        if left_detected:
            error -= 1

        if right_detected:
            error += 1

        # Centered on line
        if middle_detected and not left_detected and not right_detected:
            error = 0

        # PID terms
        proportional = error
        derivative = error - self.previous_error

        correction = (
                self.Kp * proportional +
                self.Kd * derivative
        )

        # Save error
        self.previous_error = error

        # BACKWARD movement
        # Correction is inverted because robot is reversed
        left_speed = -(self.BASE_SPEED + correction)
        right_speed = -(self.BASE_SPEED - correction)

        # Apply speeds
        self.robot.set_speed(left_speed, right_speed)

    def detect_gray_bar(self):

        g = self.robot.get_ground()

        left = g[0]
        middle = g[1]
        right = g[2]

        left_gray = self.GRAY_MIN < left < self.GRAY_MAX
        mid_gray = self.GRAY_MIN < middle < self.GRAY_MAX
        right_gray = self.GRAY_MIN < right < self.GRAY_MAX

        # condition: horizontal gray line (all sensors see gray)
        if left_gray and mid_gray and right_gray:
            print("Gray line is detected")
            return True

        return False

class GrabberRobot:

    def __init__(self, ip_address):
        self.ip = ip_address
        self.robot = wrapper.get_robot(ip_address)
        self.state = "INIT"
        self.initialize()
        self.controller = PIDLineFollower(ROBOT_1_IP)

    #Initialize robot hardware and sensors
    def initialize(self):

        print(f"[ROBOT 1] Initializing grabber at {self.ip}...")
        try:
            self.robot.initiate_model()
            self.robot.init_camera("./vision_capture")
            self.robot.init_ground()
            self.robot.init_sensors()
            self.state = "READY"
            print("[ROBOT 1] Initialization complete")
        except Exception as ex:
            print(f"[ROBOT 1] Initialization error: {ex}")
            self.state = "ERROR"

    def ReachGrabArea(self):
        print("Going to Grab Area")

        try:
            while self.robot.go_on():

                #line following to transition area
                self.controller.follow_line_pid()

                # Check for block detection
                image = np.array(self.robot.get_camera())
                block = self.detect_block(image)

                if block:
                    print("[ROBOT 1] Block detected! Stopping line follow")
                    self.robot.set_speed(0, 0)
                    return True

                time.sleep(1)

        except Exception as e:
            print(f"[ROBOT 1] Line follow error: {e}")
            time.sleep(1)
            return False

        print("[ROBOT 1] failed to reach grab area")
        return False

    def turn_180(self):
        print("[ROBOT] Turning 180 degrees...")

        self.robot.set_speed(2, -2)
        time.sleep(1)

        self.robot.set_speed(0, 0)
        time.sleep(0.5)

        print("[ROBOT] Turn complete")

    def ReachTransitionArea(self):
        print("Going to Grab Area")

        try:
            self.turn_180()
            while self.robot.go_on():
                #follow the black line
                self.controller.follow_line_pid()

                image = np.array(self.robot.get_camera())

                if self.detect_yellow_area(image):
                    print("[ROBOT] Yellow area detected in transition zone")
                    self.robot.set_speed(0, 0)
                    return True

        except Exception as e:
            print(f"[ROBOT 1] Line follow error: {e}")
            time.sleep(1)
            return False

        print("[ROBOT 1] Failed to reach Transition Area")
        return False

    def signalSender(self):

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # IP of second robot controller PC
            client.connect(("192.168.1.20", 5000))

            message = "START"

            client.send(message.encode())

            client.close()

            print("[ROBOT 1] Signal sent successfully")

        except Exception as e:
            print(f"[ROBOT 1] Signal error: {e}")

    def run_mission(self):
        print("[ROBOT 1] Starting mission...")

        try:
            # Check if robot initialized correctly
            if self.state != "READY":
                print("[ROBOT 1] Robot is not ready!")
                return False

            # =========================================
            # 1. Go to grab area
            # =========================================
            print("[MISSION] Step 1: Reach Grab Area")

            reached_grab = self.ReachGrabArea()

            if not reached_grab:
                print("[MISSION] Failed reaching grab area")
                return False

            print("[MISSION] Grab area reached")

            # =========================================
            # 2. Go to transition area
            # =========================================
            print("[MISSION] Step 3: Reach Transition Area")

            reached_transition = self.ReachTransitionArea()

            if not reached_transition:
                print("[MISSION] Failed reaching transition area")
                return False

            print("[MISSION] Transition area reached")

            # =========================================
            # 3. Stop robot
            # =========================================
            self.robot.set_speed(0, 0)

            print("[MISSION] Mission completed successfully!")
            self.robot.clean_up()

            return True

        except Exception as e:
            print(f"[MISSION] Error: {e}")

            self.state = "ERROR"

            return False

        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup robot resources"""
        self.robot.set_speed(0, 0)
        self.robot.clean_up()
        print("[ROBOT 1] Cleanup complete")

class SorterRobbot:

    def __init__(self, ip_address):
        self.ip = ip_address
        self.robot = wrapper.get_robot(ip_address)
        self.state = "INIT"
        self.initialize()
        self.controller = PIDLineFollower(ROBOT_2_IP)

    def initialize(self):
        print(f"[ROBOT 2] Initializing grabber at {self.ip}...")
        try:
            self.robot.initiate_model()
            self.robot.init_camera("./vision_capture")
            self.robot.init_ground()
            self.robot.init_sensors()
            self.state = "READY"
            print("[ROBOT 2] Initialization complete")
        except Exception as e:
            print(f"[ROBOT 2] Initialization error: {e}")
            self.state = "ERROR"

    def wait_for_signal(self):

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        server.bind(("192.168.2.206", 5000))

        server.listen(1)

        print("[ROBOT 2] Waiting for signal...")

        conn, addr = server.accept()

        data = conn.recv(1024).decode()

        if data == "START":
            print("[ROBOT 2] Start signal received!")

            self.run_mission()

        conn.close()

    def wait_for_gray_lines(self, target_count=2):

        gray_count = 0
        last_state = False

        while self.robot.go_on():

            self.controller.follow_line_pid()

            gray = self.controller.detect_gray_bar()

            if gray and not last_state:
                gray_count += 1
                print(f"Gray detected: {gray_count}")

                time.sleep(0.25)

            last_state = gray

            if gray_count >= target_count:
                return True

            time.sleep(0.01)

        print("[Robot 2] Failed to detect Gray lines")
        return False

    class ObjectIsBlue:

        def move_to_blue_area(self):

            print("[ROBOT 2] Moving to BLUE area...")

            start_time = time.time()
            timeout = 60

            try:

                while self.robot.go_on() and (time.time() - start_time) < timeout:

                    self.controller.follow_line_pid()

                    gray_detection = self.controller.detect_gray_bar()

                    if gray_detection:
                        left_speed = BASE_SPEED  # normal
                        right_speed = BASE_SPEED - TURN_SPEED  # slower
                        self.robot.set_speed(left_speed, right_speed)
                        continue

                    # OPTIONAL:
                    # Detect blue area using camera
                    image = np.array(self.robot.get_camera())
                    colors = self.robot.get_colordetection(image, min_area=AREA_THRESHOLD_MIN)

                    for color in colors:
                        if color.label == "Blue":
                            print("[ROBOT 2] BLUE area reached!")
                            self.robot.set_speed(0, 0)
                            return True

                    time.sleep(0.1)

            except Exception as e:
                print(f"[ROBOT 2] Error moving to blue area: {e}")
                time.sleep(0.1)
                return False

            print("[Robot 2] Failed to reach Blue area")
            return False

        def turn_180(self):

            print("[Robot 2] Turning 180 degrees...")

            self.robot.set_speed(2, -2)
            time.sleep(1)

            self.robot.set_speed(0, 0)
            time.sleep(0.5)

            print("[Robot 2] Turn complete")

        def GoingBackward_Trun180(self):

            print("[ROBOT 2] Going Backward to gray line...")

            try:
                while self.robot.go_on():
                    # Move backward
                    self.controller.follow_line_pid_backward()

                    GrayLineDetection = self.controller.detect_gray_bar()

                    if GrayLineDetection:
                        self.turn_180()
                        self.robot.set_speed(0,0)
                        return True

            except Exception as e:
                print(f"[ROBOT 2] Backward error: {e}")
                time.sleep(0.5)

            print("[ROBOT 2] Failed to reach Gray line")
            return False

        def returnToYellow(self):

            print("[ROBOT 2] Following line back to YELLOW...")

            try:
                while self.robot.go_on():
                    self.controller.follow_line_pid()

                    # =========================================
                    # DETECT YELLOW AREA USING CAMERA
                    # =========================================

                    image = np.array(self.robot.get_camera())

                    colors = self.robot.get_colordetection(
                        image,
                        min_area=AREA_THRESHOLD_MIN
                    )

                    for color in colors:

                        if color.label == "Yellow":
                            print("[ROBOT 2] Yellow area reached!")

                            self.robot.set_speed(0, 0)
                            return True

                    time.sleep(0.01)

            except Exception as e:
                print(f"[ROBOT 2] Return error: {e}")
                time.sleep(0.1)

            print("[Robot 2] Failed to reach yellow area")
            return False

        def cleanup(self):
            """Stop robot and release resources safely"""

            print(f"[{self.ip}] Cleaning up robot...")

            try:
                # Stop motors
                self.robot.set_speed(0, 0)

                time.sleep(0.5)

                # Close robot connection
                self.robot.clean_up()

                print(f"[{self.ip}] Cleanup complete")

            except Exception as e:
                print(f"[{self.ip}] Cleanup error: {e}")

        def runMissionBlueObject(self):

            print("\n" + "=" * 50)
            print("[ROBOT 2] STARTING FULL MISSION")
            print("=" * 50)

            try:

                # =========================================
                # STEP 1 -> GO TO BLUE AREA
                # =========================================
                print("\n[STEP 1] Moving to BLUE area...\n")

                blue_reached = self.move_to_blue_area()

                if not blue_reached:
                    print("[MISSION] Failed to reach BLUE area")
                    return False

                print("[MISSION] BLUE area reached successfully")

                # =========================================
                # STEP 2 -> GO BACKWARD TO GRAY LINE
                # =========================================
                print("\n[STEP 2] Returning backward to gray line...\n")

                backward_done = self.GoingBackward_Trun180()

                if not backward_done:
                    print("[MISSION] Failed during backward return")
                    return False

                print("[MISSION] Backward return completed")

                time.sleep(1)

                # =========================================
                # STEP 3 -> RETURN TO YELLOW AREA
                # =========================================
                print("\n[STEP 3] Returning to YELLOW area...\n")

                yellow_reached = self.returnToYellow()

                if not yellow_reached:
                    print("[MISSION] Failed to reach YELLOW area")
                    return False

                print("[MISSION] YELLOW area reached successfully")

                # =========================================
                # MISSION COMPLETE
                # =========================================
                print("\n" + "=" * 50)
                print("[ROBOT 2] MISSION COMPLETED SUCCESSFULLY")
                print("=" * 50)

                self.robot.set_speed(0, 0)

                return True

            except Exception as e:

                print(f"[MISSION ERROR] {e}")

                self.robot.set_speed(0, 0)

                return False

            finally:

                self.cleanup()
                pass

    class OBjectIsBlack:

        def move_to_black_area(self):

            print("[ROBOT] Moving toward BLACK area...")

            try:

                while self.robot.go_on():

                    self.controller.follow_line_pid()

                    GrayDetection = self.controller.detect_gray_bar()

                    if GrayDetection:
                        left_speed = BASE_SPEED - TURN_SPEED  # slower
                        right_speed = BASE_SPEED  # normal
                        self.robot.set_speed(left_speed, right_speed)
                        continue

                    # Detect BLACK area
                    image = np.array(self.robot.get_camera())

                    colors = self.robot.get_colordetection(
                        image,
                        min_area=AREA_THRESHOLD_MIN
                    )

                    for color in colors:

                        if color.label == "Black":
                            print("[ROBOT] BLACK area reached!")

                            self.robot.set_speed(0, 0)
                            return True

                time.sleep(0.1)

            except Exception as e:
                print(f"[ROBOT] Error: {e}")
                time.sleep(0.1)

            print("[Robot 2] Failed to reach black area")
            return False

        def turn_180(self):

            print("[ROBOT] Turning 180 degrees...")

            self.robot.set_speed(BASE_SPEED, -BASE_SPEED)

            time.sleep(2)

            self.robot.set_speed(0, 0)

        def GoingBackward_Trun180(self):

            print("[ROBOT 2] Going Backward to gray line...")

            try:
                while self.robot.go_on():
                    # Follow line backward
                    self.controller.follow_line_pid_backward()

                    # Detect gray line
                    GrayLineDetection = self.controller.detect_gray_bar()

                    if GrayLineDetection:
                        print("[ROBOT 2] Gray line detected!")

                        # Stop robot
                        self.robot.set_speed(0, 0)

                        time.sleep(0.3)

                        # Turn 180 degrees
                        self.turn_180()

                        return True

                time.sleep(0.1)

            except Exception as e:

                print(f"[ROBOT 2] Backward error: {e}")
                return False
                time.sleep(0.1)

            print("[Robot 2] Failed to go backward")
            return False

        def return_to_yellow_area(self):

            print("[ROBOT] Returning to YELLOW area...")

            try:
                while self.robot.go_on():

                    self.controller.follow_line_pid()
                    # Detect YELLOW area
                    image = np.array(self.robot.get_camera())

                    colors = self.robot.get_colordetection(
                        image,
                        min_area=AREA_THRESHOLD_MIN
                    )

                    for color in colors:

                        if color.label == "Yellow":
                            print("[ROBOT] YELLOW area reached!")

                            self.robot.set_speed(0, 0)
                            return True

                time.sleep(0.1)

            except Exception as ex:
                print(f"[ROBOT] Return error: {ex}")
                time.sleep(0.1)

            print("[Robot 2] Failed to reach yellow area!")
            return False

        def cleanup(self):

            print(f"[{self.ip}] Cleaning up robot...")

            try:
                # Stop motors
                self.robot.set_speed(0, 0)

                time.sleep(0.5)

                # Close robot connection
                self.robot.clean_up()

                print(f"[{self.ip}] Cleanup complete")

            except Exception as e:
                print(f"[{self.ip}] Cleanup error: {e}")

        def runMissionBlackObject(self):

            print("\n========== BLACK MISSION START ==========")

            try:

                self.turn_180()

                if not self.move_to_black_area():
                    return False

                time.sleep(1)

                if not self.GoingBackward_Trun180():
                    return False

                time.sleep(1)

                if not self.return_to_yellow_area():
                    return False

                print("\n[ROBOT] BLACK MISSION COMPLETE!")

                return True

            except Exception as e:

                print(f"[ROBOT] Mission error: {e}")

                return False

            finally:

                self.cleanup()

    class ObjectIsGreen:

        def follow_line_forward(self):

            print("[ROBOT 2] Moving to BLUE area...")

            start_time = time.time()
            timeout = 60

            try:
                 self.turn_180()

                 while self.robot.go_on() and (time.time() - start_time) < timeout:

                     #follow the line
                     self.controller.follow_line_pid()

                     if self.wait_for_gray_lines(2):
                         left_speed = BASE_SPEED  # normal
                         right_speed = BASE_SPEED - TURN_SPEED  # slower
                         self.robot.set_speed(left_speed, right_speed)
                         continue

                     image = np.array(self.robot.get_camera())
                     colors = self.robot.get_colordetection(image, min_area=AREA_THRESHOLD_MIN)

                     for color in colors:
                         if color.label == "Green":
                             print("[ROBOT 2] Green area reached!")
                             self.robot.set_speed(0, 0)
                             return True

                     time.sleep(0.1)

            except Exception as ex:
                 print(f"[ROBOT 2] Line follow error: {ex}")
                 time.sleep(0.1)
                 return False

            print("[Robot 2] Failed to reach Green area")
            return False

        def return_to_gray_from_green(self):
            try:
                print("[MISSION] Returning from GREEN to GRAY line...")

                while self.robot.go_on():

                    self.controller.follow_line_pid_backward()

                    GrayDetection = self.controller.detect_gray_bar()

                    if GrayDetection:
                        self.turn_180()
                        self.robot.set_speed(0, 0)
                        return True
                    time.sleep(0.1)

            except Exception as ex:
                print(f"[ERROR] return_to_gray_from_green: {ex}")
                return False

            print("failed to go back")
            return False

        def turn_180(self):

            print("[ROBOT] Turning 180 degrees...")

            self.robot.set_speed(BASE_SPEED, -BASE_SPEED)

            time.sleep(2)

            self.robot.set_speed(0, 0)

        def return_to_yellow_area(self):
            try:
                print("[MISSION] Returning to YELLOW area using black line...")

                while self.robot.go_on():

                    self.controller.follow_line_pid()

                    image = np.array(self.robot.get_camera())
                    colors = self.robot.get_colordetection(image, min_area=AREA_THRESHOLD_MIN)

                    for color in colors:
                        if color.label == "Yellow":
                            print("[ROBOT 2] Yellow area reached!")
                            self.robot.set_speed(0, 0)
                            return True

                    time.sleep(0.1)


            except Exception as ex:
                print(f"[ERROR] return_to_yellow_area: {ex}")
                return False

            print("failed to reach yellow area")
            return False

        def runMissionGreenObject(self):

            try:
                print("[MISSION] Starting mission...")

                print("[MISSION] Moving Forward...")
                if not self.follow_line_forward():
                    print("[MISSION] Failed to move forward")
                    return False

                print("[MISSION] Returning Back...")
                if not self.return_to_gray_from_green():
                    print("[MISSION] Failed to return to gray")
                    return False

                print("[MISSION] Going back to yellow...")
                if not self.return_to_yellow_area():
                    print("[MISSION] Failed returning to yellow")
                    return False

                print("[MISSION] Mission completed successfully")
                return True

            except KeyboardInterrupt:
                print("[MISSION] Interrupted by user")

            except Exception as e:
                print(f"[MISSION] Critical error: {e}")

            finally:
                self.robot.set_speed(0, 0)
                self.cleanup()

        def cleanup(self):

            self.robot.set_speed(0, 0)
            self.robot.clean_up()
            print("[ROBOT 1] Cleanup complete")

    class ObjectIsRed:

        def follow_line_forward(self):

            start_time = time.time()
            timeout = 60

            try:
               self.turn_180()

               while self.robot.go_on() and (time.time() - start_time) < timeout:

                   self.controller.follow_line_pid()

                   if self.wait_for_gray_lines(2):
                       left_speed = BASE_SPEED - TURN_SPEED  # slower
                       right_speed = BASE_SPEED  # normal
                       self.robot.set_speed(left_speed, right_speed)
                       continue

                   # OPTIONAL:
                   # Detect blue area using camera
                   image = np.array(self.robot.get_camera())
                   colors = self.robot.get_colordetection(image, min_area=AREA_THRESHOLD_MIN)

                   for color in colors:
                       if color.label == "Red":
                           print("[ROBOT 2] Red area reached!")
                           self.robot.set_speed(0, 0)
                           return True

            except Exception as ex:
                print(f"[ROBOT 2] Line follow error: {ex}")
                return False
                time.sleep(0.1)

            print("Failed to go forward")
            return False

        def return_to_gray_from_red(self):
            try:
                print("[MISSION] Returning from Red to GRAY line...")

                while self.robot.go_on():
                    self.controller.follow_line_pid_backward()

                    garyDetection = self.controller.detect_gray_bar()

                    if garyDetection:
                        self.turn_180()
                        self.robot.set_speed(0,0)
                        return True

            except Exception as e:
                print(f"[ERROR] return_to_gray_from_green: {e}")
                return False

            print("Failed to return to gray line")
            return False

        def turn_180(self):

            print("[ROBOT] Turning 180 degrees...")

            self.robot.set_speed(BASE_SPEED, -BASE_SPEED)

            time.sleep(2)

            self.robot.set_speed(0, 0)

        def return_to_yellow_area(self):

            print("[ROBOT] Returning to YELLOW area...")

            try:

                while self.robot.go_on():

                    self.controller.follow_line_pid()

                    image = np.array(self.robot.get_camera())

                    colors = self.robot.get_colordetection(
                        image,
                        min_area=AREA_THRESHOLD_MIN
                    )

                    for color in colors:

                        if color.label == "Yellow":
                            print("[ROBOT 2] Yellow area reached!")

                            self.robot.set_speed(0, 0)
                            return True

                    time.sleep(0.01)

            except Exception as e:
                print(f"[ROBOT] Return error: {e}")
                time.sleep(0.1)
                return False

            print("Failed to reach yellow area")
            return False

        def runMissionRedObject(self):
            try:
                print("[MISSION] Starting Red Object Mission...")

                # Follow line until first gray line
                print("[MISSION]following the Line Forward to reach Red area...")
                if not self.follow_line_forward():
                    print("[MISSION] Failed to to reach Red area")
                    return False

                print("[MISSION] Red Area Reached Successfully")

                print("[MISSION] Going backward to gray line..")
                if not self.return_to_gray_from_red():
                    print("[MISSION] Failed to go backward to Gray line...")
                    return False

                print("[MISSION] Gary Line Rached....")

                print("[MISSION] returning to yellow area")
                if not self.return_to_yellow_area():
                    print("[MISSION] Fialed to return to the yellow area")
                    return False

                print("[MISSION] Mission Completed Successfully")

            except Exception as e:
                print(f"[MISSION] Error: {e}")
                return False

            finally:
                self.cleanup()

    class ObjectIsUnknown:

        def follow_line_forward(self):

            try:
                 self.turn_180()
                 while self.robot.go_on():

                     self.controller.follow_line_pid()

                     # 1️⃣ Check camera first
                     image = np.array(self.robot.get_camera())
                     colors = self.robot.get_colordetection(image, min_area=AREA_THRESHOLD_MIN)
                     for color in colors:
                         if color.label.lower() == "Orange":
                             print("[ROBOT] Orange area reached!")
                             self.robot.set_speed(0, 0)
                             return True

                 self.turn_180()

            except Exception as ex:
                print(f"[ROBOT 2] Line follow error: {e}")
                time.sleep(0.1)

            print("Failed to go forward")
            return False

        def turn_180(self):

            print("[ROBOT] Turning 180 degrees...")

            self.robot.set_speed(BASE_SPEED, -BASE_SPEED)

            time.sleep(2)

            self.robot.set_speed(0, 0)

        def return_to_yellow_area(self):

            print("[ROBOT] Returning to Yellow area...")

            try:
                while self.robot.go_on():

                    self.controller.follow_line_pid()

                    image = np.array(self.robot.get_camera())
                    colors = self.robot.get_colordetection(image, min_area=AREA_THRESHOLD_MIN)
                    for color in colors:
                        if color.label.lower() == "yellow":
                            print("[ROBOT] Yellow area reached!")
                            self.robot.set_speed(0, 0)
                            return True

            except Exception as e:
                print(f"[ROBOT] Error returning to Yellow area: {e}")
                return False
                time.sleep(0.1)

            print("Failed to reach yellow area")
            return False

        def runMissionObjectUnknown(self):

            try:
                print("\n[MISSION] Starting Unknown Object Mission...")


                print("[MISSION] Following line forward to find Orange area...")
                if not self.follow_line_forward():
                    print("[MISSION] Failed to go forward..")
                    return False

                print("[MISSION] Orange Area Reached...")

                print("[MISSION] Returning to Yellow area...")
                if not self.return_to_yellow_area():
                    print("[MISSION] Failed to reach Yellow area!")
                    return False
                print("[MISSION] Yellow area Reached!")

                print("[MISSION] Unknown Object Mission completed!")


            except KeyboardInterrupt:
                print("\n[MISSION] Mission interrupted by user")
                self.robot.set_speed(0, 0)
                return False

            except Exception as e:
                print(f"[MISSION] Critical error: {e}")
                self.robot.set_speed(0, 0)
                return False

            finally:
                print("[MISSION] Cleaning up robot...")
                self.robot.set_speed(0, 0)
                self.cleanup()

    def run_mission(self):
       try:

            #  Detect object color at yellow area
            object_color = "Unknown"
            img = np.array(self.robot.get_camera())
            detections = self.robot.get_colordetection(img, AREA_THRESHOLD_MIN)
            for obj in detections:
                if obj.label in ["Red", "Blue", "Green", "Black"]:
                    object_color = obj.label
                    break

            print(f"[MISSION] Detected object color: {object_color}")

            # 3️⃣ Run the corresponding sub-class mission
            if object_color == "Red":
                red_handler = self.ObjectIsRed()
                red_handler.runMissionRedObject()
            elif object_color == "Blue":
                blue_handler = self.ObjectIsBlue()
                blue_handler.runMissionBlueObject()
            elif object_color == "Green":
                green_handler = self.ObjectIsGreen()
                green_handler.runMissionGreenObject()
            elif object_color == "Black":
                black_handler = self.ObjectIsBlack()
                black_handler.runMissionBlackObject()
            else:
                unknown_handler = self.ObjectIsUnknown()
                unknown_handler.runMissionObjectUnknown()

            print("\n========== SORTER ROBOT MISSION COMPLETE ==========")
            return True

       except KeyboardInterrupt:
            print("[MISSION] Mission interrupted by user")
            self.robot.set_speed(0, 0)
            return False

       except Exception as e:
            print(f"[MISSION] Critical error: {e}")
            self.robot.set_speed(0, 0)
            return False

       finally:
            # cleanup at the end
            print("[MISSION] Cleaning up robot...")
            self.robot.set_speed(0, 0)
            self.cleanup()

def main():
    """Main execution function"""
    print("\n" + "=" * 60)
    print("  E-PUCK BLOCK SORTING SYSTEM - PROJECT 2026")
    print("=" * 60)

    try:
        # Initialize robots
        print("\nInitializing robots...\n")
        robot_grabber = GrabberRobot(ROBOT_1_IP)
        robot_sorter = SorterRobbot(ROBOT_2_IP)

        # ----------------------------------------
        # Run Robot 1 mission (blocking)
        # ----------------------------------------
        print("\nStarting Robot 1 mission (Grabber)...")
        robot_1_success = robot_grabber.run_mission()

        # ----------------------------------------
        # Run Robot 2 mission (Sorter)
        # ----------------------------------------
        print("\nStarting Robot 2 mission (Sorter)...")
        robot_2_success = robot_sorter.run_mission()

        # ----------------------------------------
        # Summary
        # ----------------------------------------
        print("\n" + "=" * 60)
        print("  MISSION SUMMARY")
        print("=" * 60)
        print(f"Robot 1 (Grabber): {'SUCCESS' if robot_1_success else 'FAILED'}")
        print(f"Robot 2 (Sorter): {'SUCCESS' if robot_2_success else 'FAILED'}")
        print("=" * 60 + "\n")

        return robot_1_success and robot_2_success

    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        return False

    finally:
        # Cleanup
        try:
            robot_grabber.cleanup()
        except:
            pass
        try:
            robot_sorter.cleanup()
        except:
            pass

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
