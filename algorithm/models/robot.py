from algorithm.enums import Direction

class Robot:
    def __init__(
        self,
        x: int = 1,     # Default starting
        y: int = 1,
        heading: Direction = Direction.NORTH
    ):
        self.x = x                  # Grid x-coordinate
        self.y = y                  # Grid y-coordinate
        self.heading = heading      # Facing direction