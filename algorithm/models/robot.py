from algorithm.enums import Direction


class Robot:
    def __init__(
        self,
        x: float = 1.0,
        y: float = 1.0,
        heading: Direction = Direction.NORTH,
        theta: float = 0.0
    ):
        self.x = x
        self.y = y
        self.heading = heading
        self.theta = theta