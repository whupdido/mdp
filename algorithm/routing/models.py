"""Immutable contracts for directed pairwise routing and global optimization."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Protocol

from algorithm.config import PlanningConfig
from algorithm.enums import CostMetric, RoutingMode
from algorithm.models.arena import ArenaInput
from algorithm.models.planning import ObservationPose
from algorithm.models.pose import Pose
from algorithm.pathfinding.models import LocalPlanningResult, LocalPlanningStatus
from algorithm.targets.models import (
    ObservationCandidate,
    ObservationCandidateKind,
    ObservationLateralClass,
)


class RouteEndpointKind(Enum):
    START = "start"
    CANDIDATE = "candidate"


@dataclass(frozen=True, slots=True)
class RouteEndpoint:
    """Stable identity for the start or one observation candidate."""

    kind: RouteEndpointKind
    pose: Pose
    obstacle_id: int | None = None
    candidate_index: int | None = None
    candidate_kind: ObservationCandidateKind | None = None
    nominal: bool = False
    standoff_cm: float | None = None
    lateral_class: ObservationLateralClass | None = None
    preference_rank: int = 0
    candidate_label: str | None = None

    def __post_init__(self) -> None:
        candidate_fields = (
            self.obstacle_id, self.candidate_index, self.candidate_kind,
            self.standoff_cm, self.lateral_class, self.candidate_label,
        )
        if self.kind is RouteEndpointKind.START:
            if any(value is not None for value in candidate_fields) or self.nominal:
                raise ValueError("start endpoints cannot contain candidate identity")
            return
        if self.obstacle_id is None or self.obstacle_id <= 0:
            raise ValueError("candidate endpoints require a positive obstacle ID")
        if self.candidate_index is None or self.candidate_index < 0:
            raise ValueError("candidate endpoints require a non-negative candidate index")
        if self.candidate_kind is None:
            raise ValueError("candidate endpoints require a candidate kind")
        if self.standoff_cm is None:
            object.__setattr__(self, "standoff_cm", 20.0)
        if self.lateral_class is None:
            object.__setattr__(self, "lateral_class", {
                ObservationCandidateKind.NOMINAL: ObservationLateralClass.CENTER,
                ObservationCandidateKind.LEFT: ObservationLateralClass.LEFT,
                ObservationCandidateKind.RIGHT: ObservationLateralClass.RIGHT,
                ObservationCandidateKind.ALTERNATIVE: ObservationLateralClass.OFFSET,
            }[self.candidate_kind])
        if self.candidate_label is None:
            suffix = {
                ObservationLateralClass.CENTER: "C",
                ObservationLateralClass.LEFT: "L",
                ObservationLateralClass.RIGHT: "R",
                ObservationLateralClass.OFFSET: "O",
            }[self.lateral_class]
            object.__setattr__(self, "candidate_label", f"20{suffix}")
        if self.standoff_cm is None or self.standoff_cm <= 0.0 or self.lateral_class is None:
            raise ValueError("candidate endpoints require standoff and lateral identity")
        if self.preference_rank < 0 or not self.candidate_label:
            raise ValueError("candidate endpoints require a valid preference and label")

    @classmethod
    def start(cls, pose: Pose) -> RouteEndpoint:
        return cls(RouteEndpointKind.START, pose)

    @classmethod
    def candidate(cls, candidate: ObservationCandidate) -> RouteEndpoint:
        observation = candidate.observation_pose
        return cls(
            RouteEndpointKind.CANDIDATE,
            observation.pose,
            obstacle_id=observation.obstacle_id,
            candidate_index=observation.candidate_index,
            candidate_kind=candidate.kind,
            nominal=observation.nominal,
            standoff_cm=candidate.standoff_cm,
            lateral_class=candidate.lateral_class,
            preference_rank=candidate.preference_rank,
            candidate_label=candidate.display_label,
        )

    @property
    def observation_pose(self) -> ObservationPose:
        if self.kind is not RouteEndpointKind.CANDIDATE:
            raise ValueError("the start endpoint has no observation pose")
        assert self.obstacle_id is not None and self.candidate_index is not None
        return ObservationPose(self.obstacle_id, self.candidate_index, self.pose, self.nominal)

    @property
    def stable_key(self) -> tuple[int, int, int]:
        if self.kind is RouteEndpointKind.START:
            return (-1, -1, -1)
        assert self.obstacle_id is not None and self.candidate_index is not None
        return (self.obstacle_id, self.preference_rank, self.candidate_index)


@dataclass(frozen=True, slots=True)
class PairwiseCacheKey:
    """Complete deterministic identity for one directed local query."""

    arena: ArenaInput
    config: PlanningConfig
    objective: CostMetric
    start: RouteEndpoint
    goal: RouteEndpoint

    def __post_init__(self) -> None:
        if self.start.pose == self.goal.pose and self.start == self.goal:
            raise ValueError("pairwise cache queries require distinct endpoints")


@dataclass(frozen=True, slots=True)
class PairwiseCacheEntry:
    key: PairwiseCacheKey
    result: LocalPlanningResult
    expansion_budget: int | None = None
    attempts: int = 1
    cumulative_nodes_expanded: int = 0
    cumulative_planning_time_s: float = 0.0

    def __post_init__(self) -> None:
        if self.result.start != self.key.start.pose:
            raise ValueError("cached result start does not match its key")
        if self.result.requested_goal != self.key.goal.pose:
            raise ValueError("cached result goal does not match its key")
        if self.expansion_budget is not None and self.expansion_budget <= 0:
            raise ValueError("cache expansion budget must be positive")
        if self.attempts <= 0 or self.cumulative_nodes_expanded < 0 or self.cumulative_planning_time_s < 0.0:
            raise ValueError("cache attempt metadata cannot be negative")

    @property
    def succeeded(self) -> bool:
        return self.result.succeeded

    @property
    def selected_cost(self) -> float | None:
        if self.result.path is None:
            return None
        return self.result.path.objective_cost


@dataclass(frozen=True, slots=True)
class PairwiseCacheStats:
    requests: int = 0
    hits: int = 0
    misses: int = 0
    successful_paths: int = 0
    failed_paths: int = 0
    retries: int = 0
    retry_recoveries: int = 0
    nodes_expanded: int = 0
    planning_time_s: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.requests, self.hits, self.misses, self.successful_paths,
            self.failed_paths, self.retries, self.retry_recoveries, self.nodes_expanded,
        )
        if any(value < 0 for value in values):
            raise ValueError("pairwise cache counters cannot be negative")
        if self.planning_time_s < 0.0:
            raise ValueError("pairwise planning time cannot be negative")

    def difference(self, earlier: PairwiseCacheStats) -> PairwiseCacheStats:
        return PairwiseCacheStats(
            requests=self.requests - earlier.requests,
            hits=self.hits - earlier.hits,
            misses=self.misses - earlier.misses,
            successful_paths=self.successful_paths - earlier.successful_paths,
            failed_paths=self.failed_paths - earlier.failed_paths,
            retries=self.retries - earlier.retries,
            retry_recoveries=self.retry_recoveries - earlier.retry_recoveries,
            nodes_expanded=self.nodes_expanded - earlier.nodes_expanded,
            planning_time_s=self.planning_time_s - earlier.planning_time_s,
        )


@dataclass(frozen=True, slots=True)
class DirectedPairwiseGraph:
    """Candidate graph whose directed local edges resolve lazily through a provider."""

    arena: ArenaInput
    config: PlanningConfig
    objective: CostMetric
    start: RouteEndpoint
    candidate_groups: tuple[tuple[RouteEndpoint, ...], ...]
    entries: Mapping[tuple[RouteEndpoint, RouteEndpoint], PairwiseCacheEntry]
    provider: PairwisePathProvider | None = None
    deadline_monotonic: float | None = None
    minimum_expansion_budget: int | None = None

    def __post_init__(self) -> None:
        groups = tuple(tuple(group) for group in self.candidate_groups)
        object.__setattr__(self, "candidate_groups", groups)
        object.__setattr__(self, "entries", dict(self.entries))
        obstacle_ids = tuple(group[0].obstacle_id for group in groups if group)
        if len(obstacle_ids) != len(groups) or len(set(obstacle_ids)) != len(obstacle_ids):
            raise ValueError("each graph candidate group must identify one unique obstacle")
        if any(any(endpoint.obstacle_id != group[0].obstacle_id for endpoint in group) for group in groups):
            raise ValueError("graph groups cannot mix obstacle identities")

    @property
    def target_ids(self) -> tuple[int, ...]:
        return tuple(group[0].obstacle_id for group in self.candidate_groups)  # type: ignore[misc]

    def candidates_for(self, obstacle_id: int) -> tuple[RouteEndpoint, ...]:
        for group in self.candidate_groups:
            if group and group[0].obstacle_id == obstacle_id:
                return group
        raise KeyError(obstacle_id)

    def entry(self, start: RouteEndpoint, goal: RouteEndpoint) -> PairwiseCacheEntry:
        if start.obstacle_id is not None and start.obstacle_id == goal.obstacle_id:
            raise KeyError("same-target candidate transitions are not part of Task 1 routing")
        key = (start, goal)
        entry = self.entries.get(key)
        if entry is None:
            if self.provider is None:
                raise KeyError(key)
            if self.deadline_monotonic is not None and time.perf_counter() >= self.deadline_monotonic:
                cache_key = PairwiseCacheKey(
                    self.arena, self.config, self.objective, start, goal
                )
                entry = PairwiseCacheEntry(
                    cache_key,
                    LocalPlanningResult(
                        LocalPlanningStatus.PLANNING_TIMEOUT,
                        start.pose,
                        goal.pose,
                        message="overall Task 1 planning budget reached before local query",
                    ),
                    expansion_budget=self.config.adaptive_initial_expansions,
                )
            else:
                entry = self.provider.get_or_plan(
                    start,
                    goal,
                    self.arena,
                    self.config,
                    self.objective,
                    minimum_expansion_budget=self.minimum_expansion_budget,
                )
            self.entries[key] = entry  # type: ignore[index]
        return entry


@dataclass(frozen=True, slots=True)
class RouteOptimizationSolution:
    endpoints: tuple[RouteEndpoint, ...]
    entries: tuple[PairwiseCacheEntry, ...]
    cost: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "endpoints", tuple(self.endpoints))
        object.__setattr__(self, "entries", tuple(self.entries))
        if not math.isfinite(self.cost) or self.cost < 0.0:
            raise ValueError("optimized route cost must be finite and non-negative")
        if len(self.endpoints) != len(self.entries):
            raise ValueError("one pairwise entry is required per selected endpoint")

    @property
    def target_order(self) -> tuple[int, ...]:
        return tuple(endpoint.obstacle_id for endpoint in self.endpoints)  # type: ignore[misc]


@dataclass(frozen=True, slots=True)
class RouteOptimizationResult:
    solution: RouteOptimizationSolution | None
    permutations_evaluated: int
    candidate_transitions_evaluated: int

    def __post_init__(self) -> None:
        if self.permutations_evaluated < 0 or self.candidate_transitions_evaluated < 0:
            raise ValueError("optimization counters cannot be negative")


class RouteOrderOptimizer(Protocol):
    def optimize(self, graph: DirectedPairwiseGraph) -> RouteOptimizationResult:
        """Choose a complete target order and one candidate per target."""


class PairwisePathProvider(Protocol):
    @property
    def stats(self) -> PairwiseCacheStats:
        """Return current cache counters."""

    def get_or_plan(
        self,
        start: RouteEndpoint,
        goal: RouteEndpoint,
        arena: ArenaInput,
        config: PlanningConfig,
        objective: CostMetric,
        *,
        minimum_expansion_budget: int | None = None,
    ) -> PairwiseCacheEntry:
        """Return an existing directed query or invoke the local planner once."""


__all__ = [
    "DirectedPairwiseGraph",
    "PairwiseCacheEntry",
    "PairwiseCacheKey",
    "PairwiseCacheStats",
    "PairwisePathProvider",
    "RouteEndpoint",
    "RouteEndpointKind",
    "RouteOptimizationResult",
    "RouteOptimizationSolution",
    "RouteOrderOptimizer",
    "RoutingMode",
]
