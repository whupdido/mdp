"""Structured local-planning results shared with routing and simulation."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol

from algorithm.enums import CostMetric, Gear, Steering
from algorithm.models.arena import ArenaInput
from algorithm.models.motion import MotionPrimitive, MotionSegment
from algorithm.models.planning import PathMetrics
from algorithm.models.pose import Pose


class LocalPlanningStatus(Enum):
    SUCCESS = "success"
    NO_PATH = "no_path"
    INVALID_START = "invalid_start"
    INVALID_GOAL = "invalid_goal"
    SEARCH_LIMIT_REACHED = "search_limit_reached"


@dataclass(frozen=True, slots=True)
class HybridSearchKey:
    """Discretized bookkeeping key for an otherwise continuous state."""

    x_index: int
    y_index: int
    heading_index: int
    previous_gear: Gear | None = None
    previous_steering: Steering | None = None


@dataclass(frozen=True, slots=True)
class HybridPath:
    start: Pose
    requested_goal: Pose
    final_pose: Pose
    segments: tuple[MotionSegment, ...]
    sampled_poses: tuple[Pose, ...]
    metrics: PathMetrics
    objective: CostMetric
    objective_cost: float
    cumulative_costs: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "segments", tuple(self.segments))
        object.__setattr__(self, "sampled_poses", tuple(self.sampled_poses))
        object.__setattr__(self, "cumulative_costs", tuple(self.cumulative_costs))
        if not self.sampled_poses or self.sampled_poses[0] != self.start:
            raise ValueError("sampled path must begin at the requested start")
        if self.sampled_poses[-1] != self.final_pose:
            raise ValueError("sampled path must end at the final reached pose")
        if len(self.cumulative_costs) != len(self.segments) + 1:
            raise ValueError("cumulative costs must include start and every segment")
        if self.cumulative_costs[0] != 0.0:
            raise ValueError("the start cumulative cost must be zero")
        if any(
            following < current
            for current, following in zip(self.cumulative_costs, self.cumulative_costs[1:])
        ):
            raise ValueError("cumulative path cost cannot decrease")
        if self.objective_cost != self.cumulative_costs[-1]:
            raise ValueError("objective cost must equal the final cumulative cost")

    @property
    def primitives(self) -> tuple[MotionPrimitive, ...]:
        return tuple(segment.primitive for segment in self.segments)

    @property
    def key_poses(self) -> tuple[Pose, ...]:
        return (self.start,) + tuple(segment.end for segment in self.segments)


@dataclass(frozen=True, slots=True)
class HybridSearchDebug:
    expanded_states: tuple[Pose, ...] = ()
    generated_states: tuple[Pose, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "expanded_states", tuple(self.expanded_states))
        object.__setattr__(self, "generated_states", tuple(self.generated_states))


@dataclass(frozen=True, slots=True)
class LocalPlanningResult:
    status: LocalPlanningStatus
    start: Pose
    requested_goal: Pose
    path: HybridPath | None = None
    metrics: PathMetrics = field(default_factory=PathMetrics)
    debug: HybridSearchDebug = field(default_factory=HybridSearchDebug)
    message: str = ""

    def __post_init__(self) -> None:
        if self.status is LocalPlanningStatus.SUCCESS:
            if self.path is None:
                raise ValueError("successful local planning requires a path")
            if self.path.metrics != self.metrics:
                raise ValueError("result and path metrics must match")
        elif self.path is not None:
            raise ValueError("failed local planning cannot contain a path")

    @property
    def succeeded(self) -> bool:
        return self.status is LocalPlanningStatus.SUCCESS


class PathPlanner(Protocol):
    def plan(
        self,
        start: Pose,
        goal: Pose,
        arena: ArenaInput,
        *,
        objective: CostMetric = CostMetric.ESTIMATED_TIME,
        collect_debug: bool = False,
    ) -> LocalPlanningResult:
        """Find one directed local path between continuous poses."""


__all__ = [
    "HybridPath",
    "HybridSearchDebug",
    "HybridSearchKey",
    "LocalPlanningResult",
    "LocalPlanningStatus",
    "PathPlanner",
]
