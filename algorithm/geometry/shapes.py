"""Small immutable geometry value types used by collision checks."""

from __future__ import annotations

import math
from dataclasses import dataclass


# Numerical tolerance only. Physical clearance comes exclusively from the
# configured robot safety margin.
NUMERIC_TOLERANCE_CM = 1e-9


@dataclass(frozen=True, slots=True)
class Point:
    x_cm: float
    y_cm: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.x_cm) or not math.isfinite(self.y_cm):
            raise ValueError("point coordinates must be finite")


@dataclass(frozen=True, slots=True)
class AxisAlignedRectangle:
    min_x_cm: float
    min_y_cm: float
    max_x_cm: float
    max_y_cm: float

    def __post_init__(self) -> None:
        values = (self.min_x_cm, self.min_y_cm, self.max_x_cm, self.max_y_cm)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("rectangle coordinates must be finite")
        if self.min_x_cm >= self.max_x_cm or self.min_y_cm >= self.max_y_cm:
            raise ValueError("rectangle minimums must be less than maximums")

    @property
    def corners(self) -> tuple[Point, Point, Point, Point]:
        """Return corners counter-clockwise from the north-east corner."""
        return (
            Point(self.max_x_cm, self.max_y_cm),
            Point(self.min_x_cm, self.max_y_cm),
            Point(self.min_x_cm, self.min_y_cm),
            Point(self.max_x_cm, self.min_y_cm),
        )
