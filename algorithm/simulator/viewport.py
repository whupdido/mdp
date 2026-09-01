"""Pygame-independent world/screen coordinate transformation."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class WorldViewport:
    arena_size_cm: float
    left_px: float
    top_px: float
    size_px: float

    def __post_init__(self) -> None:
        values = (self.arena_size_cm, self.left_px, self.top_px, self.size_px)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("viewport values must be finite")
        if self.arena_size_cm <= 0.0 or self.size_px <= 0.0:
            raise ValueError("arena_size_cm and size_px must be positive")

    @property
    def pixels_per_cm(self) -> float:
        return self.size_px / self.arena_size_cm

    def world_to_screen(self, x_cm: float, y_cm: float) -> tuple[float, float]:
        return (
            self.left_px + x_cm * self.pixels_per_cm,
            self.top_px + (self.arena_size_cm - y_cm) * self.pixels_per_cm,
        )

    def screen_to_world(self, x_px: float, y_px: float) -> tuple[float, float]:
        return (
            (x_px - self.left_px) / self.pixels_per_cm,
            self.arena_size_cm - (y_px - self.top_px) / self.pixels_per_cm,
        )
