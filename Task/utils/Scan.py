from utils.Block import Block


class Scan:
    """Holds the state for a boustrophedon (lawnmower) scan of a rectangular area.

    The robot sweeps between y_min and y_max, stepping in x after each sweep.
    - Target search: x_step < 0  (scanning toward x_min)
    - Objective search:   x_step > 0  (scanning toward x_max)
    """

    def __init__(self, start_x: float, x_target: float, x_step: float,
                 y_min: float, y_max: float, start_going_to_max: bool = True):
        self.scan_x: float = start_x;
        """Current x sweep line."""
        self.x_target: float = x_target;
        """x boundary at which the scan ends."""
        self.x_step: float = x_step;
        """x offset applied after each full y sweep. Negative → toward x_min, positive → toward x_max."""
        self.y_min: float = y_min;
        self.y_max: float = y_max;
        self.going_to_max: bool = start_going_to_max;
        """Current y direction. True = moving toward y_max, False = toward y_min."""

    @property
    def target_y(self) -> float:
        """The y coordinate the robot is currently heading to."""
        return self.y_max if self.going_to_max else self.y_min;

    @property
    def waypoint(self) -> Block:
        """Next navigation target as a Block."""
        return Block(self.scan_x, self.target_y);

    @property
    def is_done(self) -> bool:
        """True when the robot has stepped past x_target."""
        if self.x_step > 0:
            return self.scan_x > self.x_target;
        else:
            return self.scan_x < self.x_target;

    def step(self):
        """Call when the robot reaches the current y target: flips y direction and advances x."""
        self.going_to_max = not self.going_to_max;
        self.scan_x += self.x_step;
