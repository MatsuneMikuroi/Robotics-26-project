class Wall:
    """Class representing the wall. It is defined by its minimum and maximum x and y coordinates, which are updated as
    the robot explores the environment."""
    def __init__(self):
        self.max_x: float = float("-inf");
        self.min_x: float = float("inf");
        self.max_y: float = float("-inf");
        self.min_y: float = float("inf");

    def update(self, x: float, y: float):
        self.max_x = max(self.max_x, x);
        self.min_x = min(self.min_x, x);
        self.max_y = max(self.max_y, y);
        self.min_y = min(self.min_y, y);

    def is_on_the_way(self, x: float, y: float, target_x: float, target_y: float, threshold: float = 0.0) -> bool:
        """Check if the line segment from (x, y) to (target_x, target_y) actually intersects the rectangle
        [min_x-t, max_x+t] x [min_y-t, max_y+t] using the Liang-Barsky algorithm."""
        rx_min = self.min_x - threshold
        rx_max = self.max_x + threshold
        ry_min = self.min_y - threshold
        ry_max = self.max_y + threshold
        dx = target_x - x
        dy = target_y - y
        t_min, t_max = 0.0, 1.0
        for p, q in [(-dx, x - rx_min), (dx, rx_max - x),
                     (-dy, y - ry_min), (dy, ry_max - y)]:
            if p == 0:
                if q < 0:
                    return False
            elif p < 0:
                t_min = max(t_min, q / p)
            else:
                t_max = min(t_max, q / p)
        return t_min < t_max  # strict: single-point boundary touch does not count as blocking
