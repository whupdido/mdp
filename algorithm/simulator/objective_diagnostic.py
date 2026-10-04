"""Paired Task 1 objective and observation-candidate diagnostics.

This module is deliberately observational: every variant is made with
``dataclasses.replace`` and planning receives an explicit objective, so the
caller’s immutable production configuration is never changed.
"""

from __future__ import annotations

import math
import random
import statistics
from dataclasses import dataclass, replace
from typing import Iterable

from algorithm.config import PlanningConfig, task1_robot_config
from algorithm.enums import CostMetric, Direction, PlanningStatus, Steering
from algorithm.models import ArenaInput, GridCell, MoveStep, Obstacle, Pose
from algorithm.pathfinding.costs import primitive_execution_time_s, transition_cost
from algorithm.routing import Task1Planner
from algorithm.simulator.task1_demo import task1_demo_obstacles
from algorithm.targets import generate_arena_observation_candidates, generate_observation_candidates
from algorithm.coordinates import default_start_pose


@dataclass(frozen=True, slots=True)
class SelectedCandidate:
    obstacle_id: int
    candidate_index: int
    x_cm: float
    y_cm: float
    heading_deg: float
    standoff_cm: float
    lateral_offset_cm: float
    label: str


@dataclass(frozen=True, slots=True)
class DiagnosticStep:
    command: str
    angle_deg: float | None
    straight_distance_cm: float | None
    geometric_length_cm: float
    estimated_primitive_time_s: float
    transition_cost: float
    cumulative_objective_cost: float


@dataclass(frozen=True, slots=True)
class DiagnosticRun:
    arena_name: str
    candidate_limit: int
    objective: CostMetric
    status: PlanningStatus
    target_order: tuple[int, ...]
    selected_candidates: tuple[SelectedCandidate, ...]
    command_sequence: tuple[str, ...]
    command_counts: tuple[tuple[str, int], ...]
    turn_commands: int
    straight_commands: int
    direction_changes: int
    steering_changes: int
    geometric_distance_cm: float | None
    estimated_execution_time_s: float | None
    objective_cost: float | None
    materialized_step_cost: float | None
    planning_time_s: float
    nodes_expanded: int
    steps: tuple[DiagnosticStep, ...]

    def count(self, command: str) -> int:
        return dict(self.command_counts).get(command, 0)


@dataclass(frozen=True, slots=True)
class SwitchAssessment:
    should_switch: bool
    distance_successes: int
    smoothness_wins: int
    median_distance_change_percent: float | None
    median_planning_time_change_percent: float | None
    reasons: tuple[str, ...]


def compare_objectives(
    arena: ArenaInput,
    config: PlanningConfig,
    *,
    arena_name: str = "arena",
    candidate_limits: tuple[int, ...] = (1, 3),
) -> tuple[DiagnosticRun, ...]:
    """Run both objective functions for immutable candidate-cap variants."""
    if not isinstance(arena, ArenaInput):
        raise TypeError("arena must be an ArenaInput")
    if not isinstance(config, PlanningConfig):
        raise TypeError("config must be a PlanningConfig")
    if not candidate_limits or any(limit <= 0 for limit in candidate_limits):
        raise ValueError("candidate limits must be positive")

    runs: list[DiagnosticRun] = []
    for limit in candidate_limits:
        variant = replace(config, guaranteed_max_candidates_per_target=limit)
        for objective in (CostMetric.ESTIMATED_TIME, CostMetric.DISTANCE):
            result = Task1Planner(variant).plan(arena, objective=objective)
            runs.append(_make_run(arena_name, limit, objective, arena, variant, result))
    return tuple(runs)


def _make_run(arena_name, limit, objective, arena, config, result) -> DiagnosticRun:
    route = result.route
    if route is None:
        return DiagnosticRun(
            arena_name, limit, objective, result.status, (), (), (),
            tuple((command, 0) for command in ("FW", "BW", "FL", "FR", "BL", "BR")),
            0, 0, 0, 0, None, None, None, None,
            result.metrics.total_planning_time_s, result.metrics.total_nodes_expanded, (),
        )

    candidate_groups = generate_arena_observation_candidates(arena, config)
    candidate_by_target = {
        group.obstacle_id: {item.observation_pose.candidate_index: item for item in group.candidates}
        for group in candidate_groups
    }
    selected = []
    for observation in route.observation_poses:
        item = candidate_by_target[observation.obstacle_id][observation.candidate_index]
        pose = observation.pose
        selected.append(SelectedCandidate(
            observation.obstacle_id,
            observation.candidate_index,
            pose.x_cm,
            pose.y_cm,
            math.degrees(pose.heading_rad),
            item.standoff_cm,
            item.lateral_offset_cm,
            item.display_label,
        ))

    moves = tuple(step for step in route.execution_steps if isinstance(step, MoveStep))
    commands = tuple(step.command for step in moves)
    command_counts = tuple((command, commands.count(command)) for command in ("FW", "BW", "FL", "FR", "BL", "BR"))
    turn_commands = sum(step.segment.primitive.steering is not Steering.STRAIGHT for step in moves)
    previous_gear = None
    previous_steering = None
    cumulative = 0.0
    details = []
    for step in moves:
        primitive = step.segment.primitive
        cost = transition_cost(primitive, config.motion, objective, previous_gear, previous_steering)
        cumulative += cost
        details.append(DiagnosticStep(
            step.command,
            math.degrees(primitive.turn_angle_rad) if primitive.steering is not Steering.STRAIGHT else None,
            primitive.travel_cm if primitive.steering is Steering.STRAIGHT else None,
            primitive.geometric_length_cm,
            primitive_execution_time_s(primitive, config.motion),
            cost,
            cumulative,
        ))
        previous_gear = primitive.gear
        previous_steering = primitive.steering

    return DiagnosticRun(
        arena_name=arena_name,
        candidate_limit=limit,
        objective=objective,
        status=result.status,
        target_order=route.target_order,
        selected_candidates=tuple(selected),
        command_sequence=commands,
        command_counts=command_counts,
        turn_commands=turn_commands,
        straight_commands=len(moves) - turn_commands,
        direction_changes=route.metrics.direction_changes,
        steering_changes=route.metrics.steering_changes,
        geometric_distance_cm=route.metrics.geometric_distance_cm,
        estimated_execution_time_s=route.metrics.estimated_time_s,
        # This is canonical pairwise cost. The separately exposed materialized
        # sum helps identify the known pairwise/materialization mismatch.
        objective_cost=route.objective_cost,
        materialized_step_cost=cumulative,
        planning_time_s=result.metrics.total_planning_time_s,
        nodes_expanded=result.metrics.total_nodes_expanded,
        steps=tuple(details),
    )


def core_benchmark_arenas(config: PlanningConfig) -> tuple[tuple[str, ArenaInput], ...]:
    """Return four stable fixtures: open, awkward nominal, turning, and repo demo."""
    start = default_start_pose(config.robot)
    awkward_target = Obstacle(1, GridCell(10, 10), face=Direction.NORTH)
    reference_arena = ArenaInput(Pose(100.0, 100.0, 0.0), (awkward_target,))
    ten_cm_pose = generate_observation_candidates(
        awkward_target, reference_arena, config
    ).candidates[1].observation_pose.pose
    return (
        ("open", ArenaInput(start, (Obstacle(1, GridCell(5, 5), face=Direction.SOUTH),))),
        # This start is at the clear 10 cm observation pose. The nominal 20 cm
        # pose needs an extra reverse command, isolating standoff flexibility.
        ("awkward_20cm", ArenaInput(ten_cm_pose, (awkward_target,))),
        ("meaningful_turning", ArenaInput(start, (
            Obstacle(1, GridCell(12, 11), face=Direction.WEST),
            Obstacle(2, GridCell(6, 13), face=Direction.NORTH),
        ))),
        ("task1_example", ArenaInput(start, task1_demo_obstacles())),
    )


def seeded_valid_benchmark_arenas(
    config: PlanningConfig,
    seeds: tuple[int, ...] = (20261004, 20261005, 20261006, 20261007, 20261008),
) -> tuple[tuple[str, ArenaInput], ...]:
    """Create reproducible one-target interior arenas from fixed RNG seeds."""
    arenas = []
    for seed in seeds:
        rng = random.Random(seed)
        obstacle = Obstacle(
            1,
            GridCell(rng.randint(6, 13), rng.randint(6, 13)),
            rng.choice(tuple(Direction)),
        )
        probe = ArenaInput(Pose(100.0, 100.0, 0.0), (obstacle,))
        group = generate_observation_candidates(obstacle, probe, config)
        valid = tuple(candidate for candidate in group.candidates if candidate.valid)
        if not valid:
            raise RuntimeError(f"seed {seed} did not produce a valid observation candidate")
        start = default_start_pose(config.robot)
        arenas.append((f"seed_{seed}", ArenaInput(start, (obstacle,))))
    return tuple(arenas)


def assess_distance_switch(runs: Iterable[DiagnosticRun]) -> SwitchAssessment:
    """Apply the agreed four-arena production objective switch gate."""
    names = ("open", "awkward_20cm", "meaningful_turning", "task1_example")
    selected = tuple(run for run in runs if run.candidate_limit == 3 and run.arena_name in names)
    by_key = {(run.arena_name, run.objective): run for run in selected}
    reasons: list[str] = []
    distance_runs = [by_key.get((name, CostMetric.DISTANCE)) for name in names]
    time_runs = [by_key.get((name, CostMetric.ESTIMATED_TIME)) for name in names]
    successes = sum(run is not None and run.status is PlanningStatus.SUCCESS for run in distance_runs)
    if successes != 4:
        reasons.append(f"DISTANCE succeeded on {successes}/4 core arenas")

    wins = 0
    for distance_run, time_run in zip(distance_runs, time_runs):
        if distance_run is None or time_run is None or distance_run.status is not PlanningStatus.SUCCESS or time_run.status is not PlanningStatus.SUCCESS:
            continue
        if (
            distance_run.turn_commands < time_run.turn_commands
            or distance_run.steering_changes < time_run.steering_changes
            or len(distance_run.command_sequence) < len(time_run.command_sequence)
        ):
            wins += 1
    if wins < 3:
        reasons.append(f"smoothness improved on {wins}/4 core arenas")

    distance_change = _median_change(
        [run.geometric_distance_cm for run in distance_runs if run is not None],
        [run.geometric_distance_cm for run in time_runs if run is not None],
    )
    if distance_change is None or distance_change > 5.0 + 1e-9:
        reasons.append("median geometric route distance increased by more than 5% or is unavailable")

    time_change = _median_change(
        [run.planning_time_s for run in distance_runs if run is not None],
        [run.planning_time_s for run in time_runs if run is not None],
    )
    if time_change is None or time_change > 20.0 + 1e-9:
        reasons.append("median planning time increased by more than 20% or is unavailable")
    return SwitchAssessment(not reasons, successes, wins, distance_change, time_change, tuple(reasons))


def _median_change(candidate: list[float | None], baseline: list[float | None]) -> float | None:
    if len(candidate) != 4 or len(baseline) != 4 or any(v is None for v in candidate + baseline):
        return None
    baseline_median = statistics.median(baseline)
    if baseline_median == 0:
        return None
    return (statistics.median(candidate) / baseline_median - 1.0) * 100.0


def run_core_benchmark(config: PlanningConfig | None = None) -> tuple[DiagnosticRun, ...]:
    """Run all four fixtures with both candidate caps and both objectives."""
    production = config or task1_robot_config()
    return tuple(
        run
        for name, arena in core_benchmark_arenas(production)
        for run in compare_objectives(arena, production, arena_name=name)
    )


if __name__ == "__main__":
    production_config = task1_robot_config()
    benchmark_runs = []
    for arena_name, arena in core_benchmark_arenas(production_config):
        print(f"[BENCH] {arena_name}", flush=True)
        arena_runs = compare_objectives(arena, production_config, arena_name=arena_name)
        benchmark_runs.extend(arena_runs)
        for run in arena_runs:
            print(
                f"{run.arena_name:20} candidates={run.candidate_limit} "
                f"objective={run.objective.value:14} status={run.status.value:20} "
                f"commands={len(run.command_sequence):3} turns={run.turn_commands:3} "
                f"steering_changes={run.steering_changes:3} "
                f"distance_cm={run.geometric_distance_cm} "
                f"planning_s={run.planning_time_s:.3f} nodes={run.nodes_expanded}",
                flush=True,
            )
    for arena_name, arena in seeded_valid_benchmark_arenas(production_config):
        print(f"[BENCH] {arena_name}", flush=True)
        arena_runs = compare_objectives(arena, production_config, arena_name=arena_name)
        benchmark_runs.extend(arena_runs)
        for run in arena_runs:
            print(
                f"{run.arena_name:20} candidates={run.candidate_limit} "
                f"objective={run.objective.value:14} status={run.status.value:20} "
                f"commands={len(run.command_sequence):3} turns={run.turn_commands:3} "
                f"steering_changes={run.steering_changes:3} "
                f"distance_cm={run.geometric_distance_cm} "
                f"planning_s={run.planning_time_s:.3f} nodes={run.nodes_expanded}",
                flush=True,
            )
    assessment = assess_distance_switch(benchmark_runs)
    print(f"switch_to_distance={assessment.should_switch}; reasons={assessment.reasons}")


__all__ = [
    "DiagnosticRun", "DiagnosticStep", "SelectedCandidate", "SwitchAssessment",
    "assess_distance_switch", "compare_objectives", "core_benchmark_arenas",
    "run_core_benchmark", "seeded_valid_benchmark_arenas",
]
