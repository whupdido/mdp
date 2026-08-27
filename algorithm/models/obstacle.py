from algorithm.enums import Direction


class Obstacle:
    def __init__(
        self,
        obstacle_id: int,
        x: int,
        y: int,
        face: Direction,
        image_id: int
    ):
        self.obstacle_id = obstacle_id
        self.x = x
        self.y = y
        self.face = face
        self.image_id = image_id