from algorithm.models import Robot, Obstacle

class Arena:
    def __init__(
        self,
        robot: Robot,
        obstacles: list[Obstacle]
    ):
        self.robot = robot
        self.obstacles = obstacles