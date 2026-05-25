class State:
    """Class representing the different states of the robot during the task. The robot will transition between these states based on its sensor readings and the task requirements."""
    WAITING = -1;
    """State for waiting before starting the task or if it is unsynced"""
    WALL_SEARCH = 0;
    """State for searching the wall"""
    WALL_FOLLOW = 1;
    """State for following the wall"""
    GO_TO_BOUND = 2;
    """State for going to the y_max/y_min bound depending on the direction of wall following"""
    TARGET_SEARCH = 3;
    """State for searching the target block"""
    GO_TO_ORIGIN = 4;
    """State for going to the origin (starting point of the robot)"""
    OBJECTIVE_SEARCH = 5;
    """State for searching the objective block"""
    GO_TO_TARGET = 6;
    """State for going to the target block from the objective block"""