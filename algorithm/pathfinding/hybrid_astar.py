"""Deterministic configurable command-aligned Hybrid A* local planner."""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass

from algorithm.config import PlanningConfig
from algorithm.enums import CostMetric, Gear, Steering
from algorithm.geometry import (
    is_motion_collision_free,
    is_pose_collision_free,
    propagate_motion,
    sample_motion,
)
from algorithm.models.arena import ArenaInput
from algorithm.models.motion import MotionPrimitive, MotionSegment
from algorithm.models.planning import PathMetrics
from algorithm.models.pose import Pose

from .costs import primitive_execution_time_s, transition_cost
from .models import (
    HybridPath,
    HybridSearchDebug,
    HybridSearchKey,
    LocalPlanningResult,
    LocalPlanningStatus,
)


_COST_EPSILON = 1e-12


def angular_distance(first_rad: float, second_rad: float) -> float:
    """Return the smallest unsigned angular separation."""
    return abs((first_rad - second_rad + math.pi) % (2.0 * math.pi) - math.pi)


def goal_reached(current: Pose, goal: Pose, config: PlanningConfig) -> bool:
    return (
        math.hypot(current.x_cm - goal.x_cm, current.y_cm - goal.y_cm)
        <= config.goal_position_tolerance_cm
        and angular_distance(current.heading_rad, goal.heading_rad)
        <= config.goal_heading_tolerance_rad
    )


def search_key(
    pose: Pose,
    config: PlanningConfig,
    previous_gear: Gear | None = None,
    previous_steering: Steering | None = None,
) -> HybridSearchKey:
    """Bucket a continuous pose without modifying or snapping that pose."""
    heading_bucket_count = max(1, round(2.0 * math.pi / config.heading_bin_rad))
    positive_heading = pose.heading_rad % (2.0 * math.pi)
    return HybridSearchKey(
        x_index=math.floor(pose.x_cm / config.position_bin_cm + 0.5),
        y_index=math.floor(pose.y_cm / config.position_bin_cm + 0.5),
        heading_index=(
            math.floor(positive_heading / config.heading_bin_rad + 0.5)
            % heading_bucket_count
        ),
        previous_gear=previous_gear,
        previous_steering=previous_steering,
    )


@dataclass(frozen=True, slots=True)
class _SearchNode:
    pose: Pose
    g_cost: float
    parent_index: int | None
    primitive: MotionPrimitive | None
    previous_gear: Gear | None
    previous_steering: Steering | None


class HybridAStarPlanner:
    """One-to-one local planner using configured continuous motion primitives."""

    def __init__(self, config: PlanningConfig) -> None:
        self.config = config

    def plan(
        self,
        start: Pose,
        goal: Pose,
        arena: ArenaInput,
        *,
        objective: CostMetric = CostMetric.ESTIMATED_TIME,
        collect_debug: bool = False,
        max_expanded_nodes: int | None = None,
        max_planning_time_s: float | None = None,
    ) -> LocalPlanningResult:
        if not isinstance(start, Pose) or not isinstance(goal, Pose):
            raise TypeError("start and goal must be Pose instances")
        if not isinstance(arena, ArenaInput):
            raise TypeError("arena must be an ArenaInput")
        if not isinstance(objective, CostMetric):
            raise TypeError("objective must be a CostMetric")

        started_at = time.perf_counter()
        expansion_limit = max_expanded_nodes or self.config.max_expanded_nodes
        if expansion_limit <= 0:
            raise ValueError("max_expanded_nodes must be positive")
        if max_planning_time_s is not None and max_planning_time_s <= 0.0:
            raise ValueError("max_planning_time_s must be positive")
        collision_checks = 1
        if not is_pose_collision_free(start, arena, self.config):
            return self._failure(
                LocalPlanningStatus.INVALID_START,
                start,
                goal,
                "start pose is outside the collision-free configuration space",
                started_at,
                collision_checks=collision_checks,
            )

        collision_checks += 1
        if not is_pose_collision_free(goal, arena, self.config):
            return self._failure(
                LocalPlanningStatus.INVALID_GOAL,
                start,
                goal,
                "goal pose is outside the collision-free configuration space",
                started_at,
                collision_checks=collision_checks,
            )

        start_node = _SearchNode(start, 0.0, None, None, None, None)
        nodes: list[_SearchNode] = [start_node]
        tie_breaker = itertools.count()
        start_h = self._heuristic(start, goal, objective)
        frontier: list[tuple[float, float, float, int, int]] = [
            (start_h, angular_distance(start.heading_rad, goal.heading_rad), start_h, next(tie_breaker), 0)
        ]
        best_cost: dict[HybridSearchKey, float] = {
            self._dominance_key(start, None, None): 0.0
        }
        expanded_states: list[Pose] = []
        generated_states: list[Pose] = []
        nodes_expanded = 0
        nodes_generated = 0
        collision_rejected = 0
        dominated = 0

        while frontier:
            _, _, _, _, node_index = heapq.heappop(frontier)
            node = nodes[node_index]
            node_key = self._dominance_key(
                node.pose,
                node.previous_gear,
                node.previous_steering,
            )
            if node.g_cost > best_cost.get(node_key, math.inf) + _COST_EPSILON:
                continue

            if goal_reached(node.pose, goal, self.config):
                return self._success(
                    nodes,
                    node_index,
                    start,
                    goal,
                    objective,
                    started_at,
                    nodes_expanded,
                    nodes_generated,
                    collision_checks,
                    expanded_states,
                    generated_states,
                    collect_debug,
                    collision_rejected=collision_rejected,
                    dominated=dominated,
                )

            timed_out = (
                max_planning_time_s is not None
                and time.perf_counter() - started_at >= max_planning_time_s
            )
            if timed_out:
                return self._failure(
                    LocalPlanningStatus.PLANNING_TIMEOUT,
                    start,
                    goal,
                    (
                        f"search reached the configured {max_planning_time_s:.3f}s timeout"
                    ),
                    started_at,
                    nodes_expanded=nodes_expanded,
                    nodes_generated=nodes_generated,
                    collision_checks=collision_checks,
                    expanded_states=expanded_states,
                    generated_states=generated_states,
                    collect_debug=collect_debug,
                    collision_rejected=collision_rejected,
                    dominated=dominated,
                )
            if nodes_expanded >= expansion_limit:
                return self._failure(
                    LocalPlanningStatus.SEARCH_LIMIT_REACHED,
                    start,
                    goal,
                    f"search reached the configured {expansion_limit}-node expansion limit",
                    started_at,
                    nodes_expanded=nodes_expanded,
                    nodes_generated=nodes_generated,
                    collision_checks=collision_checks,
                    expanded_states=expanded_states,
                    generated_states=generated_states,
                    collect_debug=collect_debug,
                    collision_rejected=collision_rejected,
                    dominated=dominated,
                )

            nodes_expanded += 1
            if collect_debug:
                expanded_states.append(node.pose)

            for primitive in self._successor_primitives():
                if self._is_redundant_immediate_inverse(node.primitive, primitive):
                    continue
                successor_pose = propagate_motion(node.pose, primitive, self.config)
                nodes_generated += 1
                if collect_debug:
                    generated_states.append(successor_pose)
                collision_checks += 1
                if not is_motion_collision_free(node.pose, primitive, arena, self.config):
                    collision_rejected += 1
                    continue

                edge_cost = transition_cost(
                    primitive,
                    self.config.motion,
                    objective,
                    node.previous_gear,
                    node.previous_steering,
                )
                successor_g = node.g_cost + edge_cost
                successor_key = self._dominance_key(
                    successor_pose,
                    primitive.gear,
                    primitive.steering,
                )
                if successor_g >= best_cost.get(successor_key, math.inf) - _COST_EPSILON:
                    dominated += 1
                    continue

                best_cost[successor_key] = successor_g
                successor_index = len(nodes)
                nodes.append(
                    _SearchNode(
                        pose=successor_pose,
                        g_cost=successor_g,
                        parent_index=node_index,
                        primitive=primitive,
                        previous_gear=primitive.gear,
                        previous_steering=primitive.steering,
                    )
                )
                heuristic = self._heuristic(successor_pose, goal, objective)
                heapq.heappush(
                    frontier,
                    (
                        successor_g + heuristic,
                        angular_distance(successor_pose.heading_rad, goal.heading_rad),
                        heuristic,
                        next(tie_breaker),
                        successor_index,
                    ),
                )

        return self._failure(
            LocalPlanningStatus.NO_PATH,
            start,
            goal,
            "no path exists under the configured command-aligned motion model",
            started_at,
            nodes_expanded=nodes_expanded,
            nodes_generated=nodes_generated,
            collision_checks=collision_checks,
            expanded_states=expanded_states,
            generated_states=generated_states,
            collision_rejected=collision_rejected,
            dominated=dominated,
            collect_debug=collect_debug,
        )

    def _is_redundant_immediate_inverse(
        self,
        previous: MotionPrimitive | None,
        candidate: MotionPrimitive,
    ) -> bool:
        """Prune only equal-length opposite straight commands that return to the parent pose."""
        return (
            previous is not None
            and self.config.motion.direction_change_penalty_s == 0.0
            and self.config.motion.steering_change_penalty_s == 0.0
            and previous.steering is Steering.STRAIGHT
            and candidate.steering is Steering.STRAIGHT
            and previous.gear is not candidate.gear
            and math.isclose(previous.travel_cm, candidate.travel_cm, abs_tol=1e-12)
        )

    def _successor_primitives(self) -> tuple[MotionPrimitive, ...]:
        """Expand configured turn-angle data while retaining legacy 90-degree verbs."""
        expanded: list[MotionPrimitive] = []
        angles = self.config.search_turn_angles_deg or self.config.turn_angles_deg
        for primitive in self.config.motion.primitives:
            if primitive.steering is Steering.STRAIGHT or primitive.radius_cm is None:
                expanded.append(primitive)
                continue
            for angle in angles:
                sign = 1.0 if primitive.turn_angle_rad > 0 else -1.0
                expanded.append(
                    MotionPrimitive(
                        primitive.command,
                        primitive.gear,
                        primitive.steering,
                        turn_angle_rad=sign * math.radians(angle),
                        radius_cm=primitive.radius_cm,
                        estimated_duration_s=primitive.estimated_duration_s * angle / 90.0,
                        physically_calibrated=primitive.physically_calibrated,
                    )
                )
        return tuple(expanded)

    def _dominance_key(
        self,
        pose: Pose,
        previous_gear: Gear | None,
        previous_steering: Steering | None,
    ) -> HybridSearchKey:
        """Retain history only when it can change a future transition cost."""
        motion = self.config.motion
        return search_key(
            pose,
            self.config,
            previous_gear if motion.direction_change_penalty_s > 0.0 else None,
            previous_steering if motion.steering_change_penalty_s > 0.0 else None,
        )

    def _heuristic(self, current: Pose, goal: Pose, objective: CostMetric) -> float:
        distance = math.hypot(current.x_cm - goal.x_cm, current.y_cm - goal.y_cm)
        if objective is CostMetric.DISTANCE:
            return distance
        # The configured straight speed is the documented v1 upper bound. If a
        # future provisional primitive duration implies a still higher speed,
        # include it so this remains a lower bound under that configuration.
        effective_speeds: list[float] = [self.config.motion.straight_speed_cm_s]
        for primitive in self._successor_primitives():
            duration = primitive_execution_time_s(primitive, self.config.motion)
            if duration <= 0.0:
                return 0.0
            effective_speeds.append(primitive.geometric_length_cm / duration)
        return distance / max(effective_speeds)

    def _success(
        self,
        nodes: list[_SearchNode],
        goal_index: int,
        start: Pose,
        goal: Pose,
        objective: CostMetric,
        started_at: float,
        nodes_expanded: int,
        nodes_generated: int,
        collision_checks: int,
        expanded_states: list[Pose],
        generated_states: list[Pose],
        collect_debug: bool,
        collision_rejected: int = 0,
        dominated: int = 0,
    ) -> LocalPlanningResult:
        chain_indices: list[int] = []
        current_index: int | None = goal_index
        while current_index is not None:
            chain_indices.append(current_index)
            current_index = nodes[current_index].parent_index
        chain_indices.reverse()
        chain = [nodes[index] for index in chain_indices]

        segments: list[MotionSegment] = []
        sampled_poses: list[Pose] = [start]
        for previous, current in zip(chain, chain[1:]):
            assert current.primitive is not None
            segments.append(MotionSegment(current.primitive, previous.pose, current.pose))
            sampled_poses.extend(sample_motion(previous.pose, current.primitive, self.config)[1:])

        primitives = tuple(segment.primitive for segment in segments)
        metrics = self._path_metrics(
            primitives,
            nodes_expanded,
            nodes_generated,
            collision_checks,
            time.perf_counter() - started_at,
            collision_rejected,
            dominated,
        )
        path = HybridPath(
            start=start,
            requested_goal=goal,
            final_pose=chain[-1].pose,
            segments=tuple(segments),
            sampled_poses=tuple(sampled_poses),
            metrics=metrics,
            objective=objective,
            objective_cost=chain[-1].g_cost,
            cumulative_costs=tuple(node.g_cost for node in chain),
        )
        debug = HybridSearchDebug(
            tuple(expanded_states) if collect_debug else (),
            tuple(generated_states) if collect_debug else (),
        )
        return LocalPlanningResult(
            status=LocalPlanningStatus.SUCCESS,
            start=start,
            requested_goal=goal,
            path=path,
            metrics=metrics,
            debug=debug,
        )

    def _path_metrics(
        self,
        primitives: tuple[MotionPrimitive, ...],
        nodes_expanded: int,
        nodes_generated: int,
        collision_checks: int,
        planning_time_s: float,
        collision_rejected: int = 0,
        dominated: int = 0,
    ) -> PathMetrics:
        forward_distance = sum(
            primitive.geometric_length_cm
            for primitive in primitives
            if primitive.gear is Gear.FORWARD
        )
        reverse_distance = sum(
            primitive.geometric_length_cm
            for primitive in primitives
            if primitive.gear is Gear.REVERSE
        )
        direction_changes = sum(
            first.gear is not second.gear
            for first, second in zip(primitives, primitives[1:])
        )
        steering_changes = sum(
            first.steering is not second.steering
            for first, second in zip(primitives, primitives[1:])
        )
        estimated_time = sum(
            primitive_execution_time_s(primitive, self.config.motion)
            for primitive in primitives
        )
        estimated_time += direction_changes * self.config.motion.direction_change_penalty_s
        estimated_time += steering_changes * self.config.motion.steering_change_penalty_s
        return PathMetrics(
            geometric_distance_cm=forward_distance + reverse_distance,
            estimated_time_s=estimated_time,
            forward_distance_cm=forward_distance,
            reverse_distance_cm=reverse_distance,
            direction_changes=direction_changes,
            steering_changes=steering_changes,
            turn_count=sum(primitive.steering is not Steering.STRAIGHT for primitive in primitives),
            command_count=len(primitives),
            nodes_expanded=nodes_expanded,
            nodes_generated=nodes_generated,
            collision_checks=collision_checks,
            planning_time_s=planning_time_s,
            collision_rejected_successors=collision_rejected,
            dominated_successors=dominated,
        )

    @staticmethod
    def _failure(
        status: LocalPlanningStatus,
        start: Pose,
        goal: Pose,
        message: str,
        started_at: float,
        *,
        nodes_expanded: int = 0,
        nodes_generated: int = 0,
        collision_checks: int = 0,
        expanded_states: list[Pose] | None = None,
        generated_states: list[Pose] | None = None,
        collect_debug: bool = False,
        collision_rejected: int = 0,
        dominated: int = 0,
    ) -> LocalPlanningResult:
        metrics = PathMetrics(
            nodes_expanded=nodes_expanded,
            nodes_generated=nodes_generated,
            collision_checks=collision_checks,
            planning_time_s=time.perf_counter() - started_at,
            collision_rejected_successors=collision_rejected,
            dominated_successors=dominated,
        )
        debug = HybridSearchDebug(
            tuple(expanded_states or ()) if collect_debug else (),
            tuple(generated_states or ()) if collect_debug else (),
        )
        return LocalPlanningResult(
            status=status,
            start=start,
            requested_goal=goal,
            metrics=metrics,
            debug=debug,
            message=message,
        )


__all__ = ["HybridAStarPlanner", "angular_distance", "goal_reached", "search_key"]
