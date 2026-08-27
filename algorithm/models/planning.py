"""Planner outputs shared by pathfinding, routing, integration, and simulation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from algorithm.enums import PlanningStatus
from algorithm.models.motion import ExecutionStep, MotionSegment
from algorithm.models.pose import Pose


@dataclass(frozen=True, slots=True)
class PathMetrics:
    geometric_distance_cm: float = 0.0
    estimated_time_s: float = 0.0
    forward_distance_cm: float = 0.0
    reverse_distance_cm: float = 0.0
    direction_changes: int = 0
    steering_changes: int = 0
    command_count: int = 0
    nodes_expanded: int = 0
    nodes_generated: int = 0
    collision_checks: int = 0
    planning_time_s: float = 0.0
    turn_count: int = 0

    def __post_init__(self) -> None:
        distances = (
            self.geometric_distance_cm,
            self.estimated_time_s,
            self.forward_distance_cm,
            self.reverse_distance_cm,
            self.planning_time_s,
        )
        counts = (
            self.direction_changes,
            self.steering_changes,
            self.turn_count,
            self.command_count,
            self.nodes_expanded,
            self.nodes_generated,
            self.collision_checks,
        )
        if not all(math.isfinite(value) for value in distances):
            raise ValueError("path metrics must be finite")
        if any(value < 0.0 for value in distances) or any(value < 0 for value in counts):
            raise ValueError("path metrics cannot be negative")


@dataclass(frozen=True, slots=True)
class PairwisePath:
    start: Pose
    goal: Pose
    segments: tuple[MotionSegment, ...]
    metrics: PathMetrics

    def __post_init__(self) -> None:
        object.__setattr__(self, "segments", tuple(self.segments))


@dataclass(frozen=True, slots=True)
class ObservationPose:
    obstacle_id: int
    candidate_index: int
    pose: Pose
    nominal: bool = False

    def __post_init__(self) -> None:
        if self.obstacle_id <= 0:
            raise ValueError("obstacle_id must be positive")
        if self.candidate_index < 0:
            raise ValueError("candidate_index cannot be negative")


@dataclass(frozen=True, slots=True)
class PlanningMetrics:
    local_paths_requested: int = 0
    local_paths_succeeded: int = 0
    cache_hits: int = 0
    permutations_evaluated: int = 0
    total_planning_time_s: float = 0.0

    def __post_init__(self) -> None:
        counts = (
            self.local_paths_requested,
            self.local_paths_succeeded,
            self.cache_hits,
            self.permutations_evaluated,
        )
        if not math.isfinite(self.total_planning_time_s):
            raise ValueError("total_planning_time_s must be finite")
        if any(value < 0 for value in counts) or self.total_planning_time_s < 0.0:
            raise ValueError("planning metrics cannot be negative")


@dataclass(frozen=True, slots=True)
class RoutePlan:
    start: Pose
    target_order: tuple[int, ...]
    observation_poses: tuple[ObservationPose, ...]
    local_paths: tuple[PairwisePath, ...]
    execution_steps: tuple[ExecutionStep, ...]
    metrics: PathMetrics

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_order", tuple(self.target_order))
        object.__setattr__(self, "observation_poses", tuple(self.observation_poses))
        object.__setattr__(self, "local_paths", tuple(self.local_paths))
        object.__setattr__(self, "execution_steps", tuple(self.execution_steps))
        if len(set(self.target_order)) != len(self.target_order):
            raise ValueError("target_order cannot contain duplicates")
        observed_ids = tuple(item.obstacle_id for item in self.observation_poses)
        if observed_ids != self.target_order:
            raise ValueError("observation poses must match target_order exactly")


@dataclass(frozen=True, slots=True)
class PlanningIssue:
    code: str
    message: str
    obstacle_id: int | None = None

    def __post_init__(self) -> None:
        if not self.code.strip() or not self.message.strip():
            raise ValueError("planning issues require a code and message")


@dataclass(frozen=True, slots=True)
class PlanningResult:
    status: PlanningStatus
    route: RoutePlan | None = None
    issues: tuple[PlanningIssue, ...] = ()
    metrics: PlanningMetrics = field(default_factory=PlanningMetrics)

    def __post_init__(self) -> None:
        object.__setattr__(self, "issues", tuple(self.issues))
        if self.status is PlanningStatus.SUCCESS:
            if self.route is None or self.issues:
                raise ValueError("successful results require a route and no issues")
        else:
            if self.route is not None:
                raise ValueError("unsuccessful results cannot contain a route")
            if not self.issues:
                raise ValueError("unsuccessful results require at least one issue")
