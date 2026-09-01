"""Obstacle domain model."""

from __future__ import annotations

from dataclasses import dataclass

from algorithm.constants import MAX_IMAGE_ID, MIN_IMAGE_ID
from algorithm.enums import Direction
from algorithm.models.pose import GridCell


@dataclass(frozen=True, slots=True)
class Obstacle:
    """One 10 cm arena obstacle and its annotated image face."""

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
