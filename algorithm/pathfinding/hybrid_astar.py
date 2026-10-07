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

from .costs import primitive_execution_time_s
from .models import (
    HybridPath,
    HybridSearchDebug,
    HybridSearchKey,
    LocalPlanningResult,
    LocalPlanningStatus,
)


_COST_EPSILON = 1e-12
TURN_DURATION_BY_ANGLE = {
    15.0: 1.6,
    30.0: 2.4,
    45.0: 2.4,   # temporary estimate; calibrate this
    60.0: 2.4,
    90.0: 2.4,
}


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
    search_key: tuple  # Swapped to native tuple for C-level hashing speed


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
                LocalPlanningStatus.INVALID_START, start, goal,
                "start pose is outside the collision-free configuration space",
                started_at, collision_checks=collision_checks,
            )

        collision_checks += 1
        if not is_pose_collision_free(goal, arena, self.config):
            return self._failure(
                LocalPlanningStatus.INVALID_GOAL, start, goal,
                "goal pose is outside the collision-free configuration space",
                started_at, collision_checks=collision_checks,
            )

        # =====================================================================
        # LOWER-LEVEL PYTHON OPTIMIZATIONS (FLATTENING)
        # =====================================================================
        
        # 1. Localize constant coordinates & configs to bypass dot-lookups
        gx, gy, gh = goal.x_cm, goal.y_cm, goal.heading_rad
        pos_tol = self.config.goal_position_tolerance_cm
        head_tol = self.config.goal_heading_tolerance_rad
        is_time_obj = objective is CostMetric.ESTIMATED_TIME
        
        dir_pen = self.config.motion.direction_change_penalty_s
        steer_pen = self.config.motion.steering_change_penalty_s
        rev_pen = self.config.motion.consecutive_reverse_penalty_s
        track_dir = dir_pen > 0.0
        track_steer = steer_pen > 0.0
        
        # 2. Precompute math constants to use fast multiplication over slow division
        inv_pos_bin = 1.0 / self.config.position_bin_cm
        inv_head_bin = 1.0 / self.config.heading_bin_rad
        two_pi = 2.0 * math.pi
        pi = math.pi
        heading_buckets = max(1, round(two_pi * inv_head_bin))
        
        # 3. Precalculate primitive base costs so they aren't computed dynamically
        best_speed, min_turn_s, max_turn_rad = self._time_heuristic_bounds()
        primitives = self._successor_primitives()
        prim_data = []
        for p in primitives:
            base_cost = primitive_execution_time_s(p, self.config.motion) if is_time_obj else p.geometric_length_cm
            prim_data.append((p, base_cost, p.gear, p.steering, p.travel_cm))

        # 4. Fast Inline Math Functions
        hypot = math.hypot
        floor = math.floor
        ceil = math.ceil

        def get_h(px: float, py: float, ph: float) -> float:
            dist = hypot(px - gx, py - gy)
            if not is_time_obj: return dist
            if best_speed <= 0.0: return 0.0
            est = dist / best_speed
            if max_turn_rad > 0.0:
                h_diff = abs((ph - gh + pi) % two_pi - pi)
                turns = ceil(h_diff / max_turn_rad - 1e-9)
                if turns * min_turn_s > est: return turns * min_turn_s
            return est
            
        def get_key(px: float, py: float, ph: float, pgear: Gear | None, psteer: Steering | None) -> tuple:
            return (
                floor(px * inv_pos_bin + 0.5),
                floor(py * inv_pos_bin + 0.5),
                floor((ph % two_pi) * inv_head_bin + 0.5) % heading_buckets,
                pgear if track_dir else None,
                psteer if track_steer else None
            )

        # =====================================================================

        start_key = get_key(start.x_cm, start.y_cm, start.heading_rad, None, None)
        start_node = _SearchNode(start, 0.0, None, None, None, None, start_key)
        nodes: list[_SearchNode] = [start_node]
        
        start_h = get_h(start.x_cm, start.y_cm, start.heading_rad)
        tie_breaker = 0
        
        frontier = [(start_h, abs((start.heading_rad - gh + pi) % two_pi - pi), start_h, tie_breaker, 0)]
        best_cost: dict[tuple, float] = {start_key: 0.0}
        
        expanded_states: list[Pose] = []
        generated_states: list[Pose] = []
        nodes_expanded = 0
        nodes_generated = 0
        collision_rejected = 0
        dominated = 0

        # Localize hot loop functions
        heappop = heapq.heappop
        heappush = heapq.heappush
        prop_motion = propagate_motion
        check_col = is_motion_collision_free

        while frontier:
            f_cost, _, _, _, node_index = heappop(frontier)
            node = nodes[node_index]
            n_pose = node.pose
            n_g = node.g_cost
            
            if n_g > best_cost.get(node.search_key, math.inf) + _COST_EPSILON:
                continue

            # Inline Goal Check
            if hypot(n_pose.x_cm - gx, n_pose.y_cm - gy) <= pos_tol:
                if abs((n_pose.heading_rad - gh + pi) % two_pi - pi) <= head_tol:
                    return self._success(
                        nodes, node_index, start, goal, objective, started_at,
                        nodes_expanded, nodes_generated, collision_checks,
                        expanded_states, generated_states, collect_debug,
                        collision_rejected=collision_rejected, dominated=dominated,
                    )

            if max_planning_time_s is not None and time.perf_counter() - started_at >= max_planning_time_s:
                return self._failure(
                    LocalPlanningStatus.PLANNING_TIMEOUT, start, goal,
                    f"search reached the configured {max_planning_time_s:.3f}s timeout",
                    started_at, nodes_expanded=nodes_expanded, nodes_generated=nodes_generated,
                    collision_checks=collision_checks, expanded_states=expanded_states,
                    generated_states=generated_states, collect_debug=collect_debug,
                    collision_rejected=collision_rejected, dominated=dominated,
                )
                
            if nodes_expanded >= expansion_limit:
                return self._failure(
                    LocalPlanningStatus.SEARCH_LIMIT_REACHED, start, goal,
                    f"search reached the configured {expansion_limit}-node expansion limit",
                    started_at, nodes_expanded=nodes_expanded, nodes_generated=nodes_generated,
                    collision_checks=collision_checks, expanded_states=expanded_states,
                    generated_states=generated_states, collect_debug=collect_debug,
                    collision_rejected=collision_rejected, dominated=dominated,
                )

            nodes_expanded += 1
            if collect_debug:
                expanded_states.append(n_pose)

            n_gear = node.previous_gear
            n_steer = node.previous_steering
            n_prim = node.primitive

            for prim, base_cost, p_gear, p_steer, p_travel in prim_data:
                
                # Fast Inline Redundant Inverse Check
                if n_prim is not None and not track_dir and not track_steer:
                    if n_prim.steering is Steering.STRAIGHT and p_steer is Steering.STRAIGHT:
                        if n_prim.gear is not p_gear and abs(n_prim.travel_cm - p_travel) < 1e-12:
                            continue
                
                succ_pose = prop_motion(n_pose, prim, self.config)
                nodes_generated += 1
                if collect_debug:
                    generated_states.append(succ_pose)
                
                # Fast Inline Transition Cost
                edge_cost = base_cost
                if is_time_obj:
                    if n_gear is not None and n_gear is not p_gear:
                        edge_cost += dir_pen
                    if n_steer is not None and n_steer is not p_steer:
                        edge_cost += steer_pen
                    if p_gear is Gear.REVERSE and n_gear is Gear.REVERSE:
                        edge_cost += rev_pen
                        
                succ_g = n_g + edge_cost
                succ_key = get_key(succ_pose.x_cm, succ_pose.y_cm, succ_pose.heading_rad, p_gear, p_steer)
                
                if succ_g >= best_cost.get(succ_key, math.inf) - _COST_EPSILON:
                    dominated += 1
                    continue

                collision_checks += 1
                if not check_col(n_pose, prim, arena, self.config):
                    collision_rejected += 1
                    continue

                best_cost[succ_key] = succ_g
                succ_idx = len(nodes)
                nodes.append(
                    _SearchNode(
                        pose=succ_pose,
                        g_cost=succ_g,
                        parent_index=node_index,
                        primitive=prim,
                        previous_gear=p_gear,
                        previous_steering=p_steer,
                        search_key=succ_key,
                    )
                )
                
                h_val = get_h(succ_pose.x_cm, succ_pose.y_cm, succ_pose.heading_rad)
                tie_breaker += 1
                
                heappush(
                    frontier,
                    (
                        succ_g + h_val,
                        abs((succ_pose.heading_rad - gh + pi) % two_pi - pi),
                        h_val,
                        tie_breaker,
                        succ_idx,
                    ),
                )

        return self._failure(
            LocalPlanningStatus.NO_PATH, start, goal,
            "no path exists under the configured command-aligned motion model",
            started_at, nodes_expanded=nodes_expanded, nodes_generated=nodes_generated,
            collision_checks=collision_checks, expanded_states=expanded_states,
            generated_states=generated_states, collision_rejected=collision_rejected,
            dominated=dominated, collect_debug=collect_debug,
        )

    def _is_redundant_immediate_inverse(self, previous: MotionPrimitive | None, candidate: MotionPrimitive) -> bool:
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
        cached = getattr(self, "_successor_cache", None)
        if cached is not None and cached[0] is self.config:
            return cached[1]
        primitives = self._build_successor_primitives()
        self._successor_cache = (self.config, primitives)
        return primitives

    def _build_successor_primitives(self) -> tuple[MotionPrimitive, ...]:
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
                        primitive.command, primitive.gear, primitive.steering,
                        turn_angle_rad=sign * math.radians(angle),
                        radius_cm=primitive.radius_cm,
                        estimated_duration_s=TURN_DURATION_BY_ANGLE[angle],
                        physically_calibrated=primitive.physically_calibrated,
                    )
                )
        return tuple(expanded)

    def _dominance_key(self, pose: Pose, previous_gear: Gear | None, previous_steering: Steering | None) -> tuple:
        motion = self.config.motion
        inv_pos_bin = 1.0 / self.config.position_bin_cm
        inv_head_bin = 1.0 / self.config.heading_bin_rad
        heading_buckets = max(1, round(2.0 * math.pi * inv_head_bin))
        return (
            math.floor(pose.x_cm * inv_pos_bin + 0.5),
            math.floor(pose.y_cm * inv_pos_bin + 0.5),
            math.floor((pose.heading_rad % (2.0 * math.pi)) * inv_head_bin + 0.5) % heading_buckets,
            previous_gear if motion.direction_change_penalty_s > 0.0 else None,
            previous_steering if motion.steering_change_penalty_s > 0.0 else None
        )

    def _heuristic(self, current: Pose, goal: Pose, objective: CostMetric) -> float:
        distance = math.hypot(current.x_cm - goal.x_cm, current.y_cm - goal.y_cm)
        if objective is CostMetric.DISTANCE:
            return distance
        best_speed, min_turn_s, max_turn_rad = self._time_heuristic_bounds()
        if best_speed <= 0.0:
            return 0.0
        estimate = distance / best_speed
        if max_turn_rad > 0.0:
            heading_gap = angular_distance(current.heading_rad, goal.heading_rad)
            turns_left = math.ceil(heading_gap / max_turn_rad - 1e-9)
            estimate = max(estimate, turns_left * min_turn_s)
        return estimate

    def _time_heuristic_bounds(self) -> tuple[float, float, float]:
        cached = getattr(self, "_bounds_cache", None)
        if cached is not None and cached[0] is self.config:
            return cached[1]
        best_speed = 0.0
        min_turn_s = math.inf
        max_turn_rad = 0.0
        bounds = None
        for primitive in self._successor_primitives():
            duration = primitive_execution_time_s(primitive, self.config.motion)
            if duration <= 0.0:
                bounds = (0.0, 0.0, 0.0)
                break
            best_speed = max(best_speed, primitive.geometric_length_cm / duration)
            if primitive.steering is not Steering.STRAIGHT:
                min_turn_s = min(min_turn_s, duration)
                max_turn_rad = max(max_turn_rad, abs(primitive.turn_angle_rad))
        if bounds is None:
            bounds = (best_speed, min_turn_s if max_turn_rad > 0.0 else 0.0, max_turn_rad)
        self._bounds_cache = (self.config, bounds)
        return bounds

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
            primitives, nodes_expanded, nodes_generated, collision_checks,
            time.perf_counter() - started_at, collision_rejected, dominated,
        )
        path = HybridPath(
            start=start, requested_goal=goal, final_pose=chain[-1].pose,
            segments=tuple(segments), sampled_poses=tuple(sampled_poses),
            metrics=metrics, objective=objective, objective_cost=chain[-1].g_cost,
            cumulative_costs=tuple(node.g_cost for node in chain),
        )
        debug = HybridSearchDebug(
            tuple(expanded_states) if collect_debug else (),
            tuple(generated_states) if collect_debug else (),
        )
        return LocalPlanningResult(
            status=LocalPlanningStatus.SUCCESS, start=start, requested_goal=goal,
            path=path, metrics=metrics, debug=debug,
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
        forward_distance = sum(p.geometric_length_cm for p in primitives if p.gear is Gear.FORWARD)
        reverse_distance = sum(p.geometric_length_cm for p in primitives if p.gear is Gear.REVERSE)
        direction_changes = sum(first.gear is not second.gear for first, second in zip(primitives, primitives[1:]))
        steering_changes = sum(first.steering is not second.steering for first, second in zip(primitives, primitives[1:]))
        estimated_time = sum(primitive_execution_time_s(p, self.config.motion) for p in primitives)
        estimated_time += direction_changes * self.config.motion.direction_change_penalty_s
        estimated_time += steering_changes * self.config.motion.steering_change_penalty_s
        
        return PathMetrics(
            geometric_distance_cm=forward_distance + reverse_distance,
            estimated_time_s=estimated_time,
            forward_distance_cm=forward_distance,
            reverse_distance_cm=reverse_distance,
            direction_changes=direction_changes,
            steering_changes=steering_changes,
            turn_count=sum(p.steering is not Steering.STRAIGHT for p in primitives),
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
        status: LocalPlanningStatus, start: Pose, goal: Pose, message: str, started_at: float, *,
        nodes_expanded: int = 0, nodes_generated: int = 0, collision_checks: int = 0,
        expanded_states: list[Pose] | None = None, generated_states: list[Pose] | None = None,
        collect_debug: bool = False, collision_rejected: int = 0, dominated: int = 0,
    ) -> LocalPlanningResult:
        metrics = PathMetrics(
            nodes_expanded=nodes_expanded, nodes_generated=nodes_generated, collision_checks=collision_checks,
            planning_time_s=time.perf_counter() - started_at, collision_rejected_successors=collision_rejected,
            dominated_successors=dominated,
        )
        debug = HybridSearchDebug(
            tuple(expanded_states or ()) if collect_debug else (),
            tuple(generated_states or ()) if collect_debug else (),
        )
        return LocalPlanningResult(
            status=status, start=start, requested_goal=goal,
            metrics=metrics, debug=debug, message=message,
        )


__all__ = ["HybridAStarPlanner", "angular_distance", "goal_reached", "search_key"]