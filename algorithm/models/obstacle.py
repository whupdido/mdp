"""Obstacle domain models for grid and continuous arenas."""

from __future__ import annotations

from dataclasses import dataclass

from algorithm.constants import MAX_IMAGE_ID, MIN_IMAGE_ID
from algorithm.enums import Direction
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from algorithm.geometry.shapes import AxisAlignedRectangle
from algorithm.models.pose import GridCell


@dataclass(frozen=True, slots=True)
class Obstacle:
    """A legacy 10 cm grid obstacle with an optional image face."""

    obstacle_id: int
    cell: GridCell
    face: Direction | None = None
    image_id: int | None = None

    def __post_init__(self) -> None:
        if isinstance(self.obstacle_id, bool) or not isinstance(self.obstacle_id, int):
            raise TypeError("obstacle_id must be an integer")
        if self.obstacle_id <= 0:
            raise ValueError("obstacle_id must be positive")
        if not isinstance(self.cell, GridCell):
            raise TypeError("cell must be a GridCell")
        if self.face is not None and not isinstance(self.face, Direction):
            raise TypeError("face must be a Direction or None")
        if self.image_id is not None and not (MIN_IMAGE_ID <= self.image_id <= MAX_IMAGE_ID):
            raise ValueError(f"image_id must be between {MIN_IMAGE_ID} and {MAX_IMAGE_ID}")

    @property
    def x(self) -> int:
        return self.cell.x

    @property
    def y(self) -> int:
        return self.cell.y


@dataclass(frozen=True, slots=True)
class RectangleObstacle:
    """Task 2 obstacles, dimensions in cm."""

    obstacle_id: int
    min_x_cm: float
    min_y_cm: float
    max_x_cm: float
    max_y_cm: float
    face: Direction | None = None
    image_id: int | None = None

    def __post_init__(self) -> None:
        if isinstance(self.obstacle_id, bool) or not isinstance(self.obstacle_id, int) or self.obstacle_id <= 0:
            raise ValueError("obstacle_id must be a positive integer")
        values = (self.min_x_cm, self.min_y_cm, self.max_x_cm, self.max_y_cm)
        import math
        if not all(math.isfinite(value) for value in values):
            raise ValueError("rectangle coordinates must be finite")
        if self.max_x_cm <= self.min_x_cm or self.max_y_cm <= self.min_y_cm:
            raise ValueError("rectangle must have positive width and height")
        if self.face is not None and not isinstance(self.face, Direction):
            raise TypeError("face must be a Direction or None")
        if self.image_id is not None and not (MIN_IMAGE_ID <= self.image_id <= MAX_IMAGE_ID):
            raise ValueError(f"image_id must be between {MIN_IMAGE_ID} and {MAX_IMAGE_ID}")

    @property
    def bounds(self) -> "AxisAlignedRectangle":
        from algorithm.geometry.shapes import AxisAlignedRectangle
        return AxisAlignedRectangle(self.min_x_cm, self.min_y_cm, self.max_x_cm, self.max_y_cm)


ObstacleLike = Obstacle | RectangleObstacle
