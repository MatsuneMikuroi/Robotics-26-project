class Block:
    """Class representing a block (green or red). It is defined by its x and y coordinates, which can be used to
    determine its position relative to the robot and to navigate towards it."""
    def __init__(self, x: float, y: float, yaw: float = 0.0):
        self.x: float = x;
        self.y: float = y;
        self.yaw: float = yaw;
