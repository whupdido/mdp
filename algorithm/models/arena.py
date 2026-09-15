"""Validated arena input model."""

from __future__ import annotations

import math
from dataclasses import dataclass

from algorithm.models.obstacle import ObstacleLike
from algorithm.models.pose import Pose
from algorithm.models.planning import PlanningIssue
from algorithm.models.wall import Wall


@dataclass(frozen=True, slots=True)
class ArenaInput:
    start_pose: Pose
    obstacles: tuple[ObstacleLike, ...] = ()
    width_cm: float | None = None
    height_cm: float | None = None
    walls: tuple[Wall, ...] = ()

    def __post_init__(self) -> None:
        obstacles = tuple(self.obstacles)
        object.__setattr__(self, "obstacles", obstacles)
        walls = tuple(self.walls)
        object.__setattr__(self, "walls", walls)
        ids = tuple(obstacle.obstacle_id for obstacle in obstacles)
        if len(set(ids)) != len(ids):
            raise ValueError("obstacle IDs must be unique")
        grid_cells = tuple(getattr(obstacle, "cell", None) for obstacle in obstacles)
        grid_cells = tuple(cell for cell in grid_cells if cell is not None)
        if len(set(grid_cells)) != len(grid_cells):
            raise ValueError("obstacles cannot occupy the same cell")
        for name, value in (("width_cm", self.width_cm), ("height_cm", self.height_cm)):
            if value is not None and (not math.isfinite(value) or value <= 0.0):
                raise ValueError(f"{name} must be positive and finite when provided")
        if (self.width_cm is None) != (self.height_cm is None):
            raise ValueError("width_cm and height_cm must be provided together")

    def task1_issues(self) -> tuple[PlanningIssue, ...]:
        """Return input problems that prevent image observation planning."""
        return tuple(
            PlanningIssue(
                code="missing_image_face",
                message=f"obstacle {obstacle.obstacle_id} has no image face",
                obstacle_id=obstacle.obstacle_id,
            )
            for obstacle in self.obstacles
            if obstacle.face is None
        )


Arena = ArenaInput
