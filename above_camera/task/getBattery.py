from unifr_api_epuck import wrapper
import sys

robot = wrapper.get_robot(f"192.168.2.{sys.argv[1]}")

robot.sleep(5)

robot.clean_up()