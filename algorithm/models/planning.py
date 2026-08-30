"""Planner outputs shared by pathfinding, routing, integration, and simulation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from algorithm.enums import CostMetric, PlanningStatus
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
    collision_rejected_successors: int = 0
    dominated_successors: int = 0

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
    final_pose: Pose | None = None
    sampled_poses: tuple[Pose, ...] = ()
    objective: CostMetric = CostMetric.ESTIMATED_TIME
    objective_cost: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "segments", tuple(self.segments))
        object.__setattr__(self, "sampled_poses", tuple(self.sampled_poses))
        if not math.isfinite(self.objective_cost) or self.objective_cost < 0.0:
            raise ValueError("pairwise objective cost must be finite and non-negative")
        if self.sampled_poses:
            if self.sampled_poses[0] != self.start:
                raise ValueError("pairwise sampled poses must begin at start")
            if self.sampled_poses[-1] != self.reached_pose:
                raise ValueError("pairwise sampled poses must end at final pose")

    @property
    def reached_pose(self) -> Pose:
        return self.final_pose or self.goal

    @property
    def primitives(self):
        return tuple(segment.primitive for segment in self.segments)


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
    targets_requested: int = 0
    targets_routed: int = 0
    candidate_transitions_evaluated: int = 0
    pairwise_cache_misses: int = 0
    unreachable_pairwise_paths: int = 0
    selected_route_cost: float | None = None
    optimized_candidate_chain_cost: float | None = None
    total_distance_cm: float = 0.0
    total_estimated_time_s: float = 0.0
    nearest_neighbour_route_cost: float | None = None
    nearest_neighbour_distance_cm: float | None = None
    nearest_neighbour_estimated_time_s: float | None = None
    candidate_generation_time_s: float = 0.0
    pairwise_planning_time_s: float = 0.0
    global_routing_time_s: float = 0.0
    target_reachability: tuple[TargetReachability, ...] = ()
    candidate_tiers_activated: int = 0
    candidate_count_considered: int = 0
    hybrid_astar_retries: int = 0
    hybrid_astar_retry_recoveries: int = 0
    total_nodes_expanded: int = 0
    planning_budget_exhausted: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_reachability", tuple(self.target_reachability))
        counts = (
            self.local_paths_requested,
            self.local_paths_succeeded,
            self.cache_hits,
            self.permutations_evaluated,
            self.targets_requested,
            self.targets_routed,
            self.candidate_transitions_evaluated,
            self.pairwise_cache_misses,
            self.unreachable_pairwise_paths,
            self.candidate_tiers_activated,
            self.candidate_count_considered,
            self.hybrid_astar_retries,
            self.hybrid_astar_retry_recoveries,
            self.total_nodes_expanded,
        )
        measurements = (
            self.total_planning_time_s,
            self.total_distance_cm,
            self.total_estimated_time_s,
            self.candidate_generation_time_s,
            self.pairwise_planning_time_s,
            self.global_routing_time_s,
        )
        optional_costs = (
            self.selected_route_cost,
            self.optimized_candidate_chain_cost,
            self.nearest_neighbour_route_cost,
            self.nearest_neighbour_distance_cm,
            self.nearest_neighbour_estimated_time_s,
        )
        if not all(math.isfinite(value) for value in measurements):
            raise ValueError("planning measurements must be finite")
        if any(value is not None and (not math.isfinite(value) or value < 0.0) for value in optional_costs):
            raise ValueError("optional route costs must be finite and non-negative")
        if any(value < 0 for value in counts) or any(value < 0.0 for value in measurements):
            raise ValueError("planning metrics cannot be negative")


@dataclass(frozen=True, slots=True)
class TargetReachability:
    target_id: int
    geometric_candidates: int
    reachable_candidates: int
    no_path_edges: int = 0
    search_limit_edges: int = 0
    activated_candidates: int = 0

    def __post_init__(self) -> None:
        values = (
            self.target_id,
            self.geometric_candidates,
            self.reachable_candidates,
            self.no_path_edges,
            self.search_limit_edges,
            self.activated_candidates,
        )
        if self.target_id <= 0 or any(value < 0 for value in values[1:]):
            raise ValueError("target reachability values must be non-negative")
        if self.reachable_candidates > self.geometric_candidates:
            raise ValueError("reachable candidates cannot exceed geometric candidates")
        if self.activated_candidates > self.geometric_candidates:
            raise ValueError("activated candidates cannot exceed geometric candidates")


@dataclass(frozen=True, slots=True)
class RoutePlan:
    start: Pose
    target_order: tuple[int, ...]
    observation_poses: tuple[ObservationPose, ...]
    local_paths: tuple[PairwisePath, ...]
    execution_steps: tuple[ExecutionStep, ...]
    metrics: PathMetrics
    objective: CostMetric = CostMetric.ESTIMATED_TIME
    objective_cost: float = 0.0
    sampled_poses: tuple[Pose, ...] = ()
    key_poses: tuple[Pose, ...] = ()
    selected_candidate_kinds: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_order", tuple(self.target_order))
        object.__setattr__(self, "observation_poses", tuple(self.observation_poses))
        object.__setattr__(self, "local_paths", tuple(self.local_paths))
        object.__setattr__(self, "execution_steps", tuple(self.execution_steps))
        object.__setattr__(self, "sampled_poses", tuple(self.sampled_poses))
        object.__setattr__(self, "key_poses", tuple(self.key_poses))
        object.__setattr__(self, "selected_candidate_kinds", tuple(self.selected_candidate_kinds))
        if not math.isfinite(self.objective_cost) or self.objective_cost < 0.0:
            raise ValueError("route objective cost must be finite and non-negative")
        if len(set(self.target_order)) != len(self.target_order):
            raise ValueError("target_order cannot contain duplicates")
        observed_ids = tuple(item.obstacle_id for item in self.observation_poses)
        if observed_ids != self.target_order:
            raise ValueError("observation poses must match target_order exactly")
        if self.selected_candidate_kinds and len(self.selected_candidate_kinds) != len(self.target_order):
            raise ValueError("selected candidate kinds must match target_order length")
        if self.sampled_poses and self.sampled_poses[0] != self.start:
            raise ValueError("route sampled poses must begin at start")

    @property
    def primitives(self):
        return tuple(segment.primitive for path in self.local_paths for segment in path.segments)


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
