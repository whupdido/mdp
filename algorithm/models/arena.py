"""Validated arena input model."""

from __future__ import annotations

from dataclasses import dataclass

from algorithm.models.obstacle import Obstacle
from algorithm.models.pose import Pose
from algorithm.models.planning import PlanningIssue


@dataclass(frozen=True, slots=True)
class ArenaInput:
    start_pose: Pose
    obstacles: tuple[Obstacle, ...] = ()

    def __post_init__(self) -> None:
        obstacles = tuple(self.obstacles)
        object.__setattr__(self, "obstacles", obstacles)
        ids = tuple(obstacle.obstacle_id for obstacle in obstacles)
        cells = tuple(obstacle.cell for obstacle in obstacles)
        if len(set(ids)) != len(ids):
            raise ValueError("obstacle IDs must be unique")
        if len(set(cells)) != len(cells):
            raise ValueError("obstacles cannot occupy the same cell")

    def task1_issues(self) -> tuple[PlanningIssue, ...]:
        """Return input problems that prevent Task 1 observation planning."""
        return tuple(
            PlanningIssue(
                code="missing_image_face",
                message=f"obstacle {obstacle.obstacle_id} has no image face",
                obstacle_id=obstacle.obstacle_id,
            )
            for obstacle in self.obstacles
            if obstacle.face is None
        )


# Preserve the original public import while giving the contract an explicit
# integration-facing name.
Arena = ArenaInput
