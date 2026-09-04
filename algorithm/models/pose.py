"""Coordinate primitives and Android/planner coordinate transforms."""

from __future__ import annotations

import math
from dataclasses import dataclass

from algorithm.constants import CELL_SIZE_CM, GRID_SIZE
from algorithm.enums import Direction


def normalize_heading(heading_rad: float) -> float:
    """Normalize a heading to ``[-pi, pi)``."""
    if not math.isfinite(heading_rad):
        raise ValueError("heading_rad must be finite")
    normalized = (heading_rad + math.pi) % (2.0 * math.pi) - math.pi
    return 0.0 if normalized == -0.0 else normalized


@dataclass(frozen=True, slots=True, order=True)
class GridCell:
    """An Android arena cell using the bottom-left, zero-based convention."""

    x: int
    y: int

    def __post_init__(self) -> None:
        if isinstance(self.x, bool) or not isinstance(self.x, int):
            raise TypeError("x must be an integer")
        if isinstance(self.y, bool) or not isinstance(self.y, int):
            raise TypeError("y must be an integer")
        if not (0 <= self.x < GRID_SIZE and 0 <= self.y < GRID_SIZE):
            raise ValueError(f"cell ({self.x}, {self.y}) is outside the {GRID_SIZE}x{GRID_SIZE} arena")

    def center_cm(self, cell_size_cm: float = CELL_SIZE_CM) -> tuple[float, float]:
        if cell_size_cm <= 0.0 or not math.isfinite(cell_size_cm):
            raise ValueError("cell_size_cm must be positive and finite")
        return ((self.x + 0.5) * cell_size_cm, (self.y + 0.5) * cell_size_cm)


@dataclass(frozen=True, slots=True)
class Pose:
    """Continuous rear-axle pose in centimetres and radians."""

    x_cm: float
    y_cm: float
    heading_rad: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.x_cm) or not math.isfinite(self.y_cm):
            raise ValueError("pose coordinates must be finite")
        object.__setattr__(self, "heading_rad", normalize_heading(self.heading_rad))

    @classmethod
    def from_direction(cls, x_cm: float, y_cm: float, direction: Direction) -> Pose:
        return cls(x_cm=x_cm, y_cm=y_cm, heading_rad=direction.heading_rad)

    def translated_local(self, forward_cm: float, left_cm: float) -> tuple[float, float]:
        """Translate a local forward/left vector into arena coordinates."""
        cosine = math.cos(self.heading_rad)
        sine = math.sin(self.heading_rad)
        return (
            self.x_cm + forward_cm * cosine - left_cm * sine,
            self.y_cm + forward_cm * sine + left_cm * cosine,
        )
