"""Immutable observation-candidate results for routing and simulation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from algorithm.enums import Direction
from algorithm.geometry.shapes import Point
from algorithm.models.planning import ObservationPose, PlanningIssue


class ObservationCandidateKind(Enum):
    NOMINAL = "nominal"
    LEFT = "left"
    RIGHT = "right"
    ALTERNATIVE = "alternative"


class ObservationLateralClass(Enum):
    CENTER = "center"
    LEFT = "left"
    RIGHT = "right"
    OFFSET = "offset"


@dataclass(frozen=True, slots=True)
class ObservationCandidate:
    observation_pose: ObservationPose
    face: Direction
    kind: ObservationCandidateKind
    lateral_offset_cm: float
    camera_position: Point
    target_point: Point
    collision_free: bool
    line_of_sight_clear: bool
    standoff_cm: float = 20.0
    lateral_class: ObservationLateralClass = ObservationLateralClass.CENTER
    preference_rank: int = 0

    def __post_init__(self) -> None:
        if self.standoff_cm <= 0.0:
            raise ValueError("candidate standoff must be positive")
        if self.preference_rank < 0:
            raise ValueError("candidate preference rank cannot be negative")

    @property
    def display_label(self) -> str:
        suffix = {
            ObservationLateralClass.CENTER: "C",
            ObservationLateralClass.LEFT: "L",
            ObservationLateralClass.RIGHT: "R",
            ObservationLateralClass.OFFSET: "O",
        }[self.lateral_class]
        distance = int(self.standoff_cm) if float(self.standoff_cm).is_integer() else self.standoff_cm
        return f"{distance}{suffix}"

    @property
    def valid(self) -> bool:
        """Geometric validity only; motion-planner reachability is unknown."""
        return self.collision_free and self.line_of_sight_clear

    @property
    def rejection_reasons(self) -> tuple[str, ...]:
        reasons: list[str] = []
        if not self.collision_free:
            reasons.append("pose_collision")
        if not self.line_of_sight_clear:
            reasons.append("line_of_sight_blocked")
        return tuple(reasons)


@dataclass(frozen=True, slots=True)
class ObservationCandidateGroup:
    obstacle_id: int
    face: Direction | None
    candidates: tuple[ObservationCandidate, ...] = ()
    issues: tuple[PlanningIssue, ...] = ()

    def __post_init__(self) -> None:
        candidates = tuple(self.candidates)
        issues = tuple(self.issues)
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "issues", issues)
        if self.obstacle_id <= 0:
            raise ValueError("obstacle_id must be positive")
        if any(candidate.observation_pose.obstacle_id != self.obstacle_id for candidate in candidates):
            raise ValueError("all candidates must belong to the group obstacle")
        if self.face is None and candidates:
            raise ValueError("an obstacle without an image face cannot have candidates")
        if self.face is not None and any(candidate.face is not self.face for candidate in candidates):
            raise ValueError("all candidates must use the group image face")

    @property
    def valid_candidates(self) -> tuple[ObservationPose, ...]:
        return tuple(candidate.observation_pose for candidate in self.candidates if candidate.valid)

    @property
    def has_valid_candidate(self) -> bool:
        return bool(self.valid_candidates)
