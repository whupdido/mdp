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
    arena_height_cm: float | None = None

    def __post_init__(self) -> None:
        values = (self.arena_size_cm, self.left_px, self.top_px, self.size_px)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("viewport values must be finite")
        if self.arena_size_cm <= 0.0 or self.size_px <= 0.0:
            raise ValueError("arena_size_cm and size_px must be positive")
        if self.arena_height_cm is not None and (not math.isfinite(self.arena_height_cm) or self.arena_height_cm <= 0.0):
            raise ValueError("arena_height_cm must be positive and finite")

    @property
    def pixels_per_cm(self) -> float:
        return self.size_px / self.arena_size_cm

    def world_to_screen(self, x_cm: float, y_cm: float) -> tuple[float, float]:
        height = self.arena_height_cm or self.arena_size_cm
        return (
            self.left_px + x_cm * self.pixels_per_cm,
            self.top_px + (height - y_cm) * self.pixels_per_cm,
        )

    def screen_to_world(self, x_px: float, y_px: float) -> tuple[float, float]:
        height = self.arena_height_cm or self.arena_size_cm
        return (
            (x_px - self.left_px) / self.pixels_per_cm,
            height - (y_px - self.top_px) / self.pixels_per_cm,
        )
