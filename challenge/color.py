<<<<<<< HEAD

=======
>>>>>>> 8e8480266fea0af42f9e425fadfce0108763bbc1
from unifr_api_epuck import wrapper
import numpy as np


ROBOT_ID = 210
ROBOT = wrapper.get_robot(f"192.168.2.{ROBOT_ID}")

ROBOT.init_camera("img");
ROBOT.initiate_model();
ROBOT.sleep(2);

img = np.array(ROBOT.get_camera())
colors = ROBOT.get_detection(img)
<<<<<<< HEAD
print([color.label for color in colors])
=======
print([color.label for color in colors])
>>>>>>> 8e8480266fea0af42f9e425fadfce0108763bbc1
