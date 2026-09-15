from dataclasses import dataclass


@dataclass(frozen=True)
class Wall:
    wall_id: str
    min_x_cm: float
    min_y_cm: float
    max_x_cm: float
    max_y_cm: float