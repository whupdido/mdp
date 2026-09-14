"""Deterministic five-target Phase 6 planning and playback scenario."""

from __future__ import annotations

from dataclasses import dataclass, replace

from algorithm.config import PlanningConfig, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.coordinates import default_start_pose
from algorithm.enums import CostMetric, Direction, PlanningStatus
from algorithm.models import ArenaInput, GridCell, Obstacle, PlanningResult
from algorithm.routing import Task1Planner
from algorithm.targets import ObservationCandidateGroup, generate_arena_observation_candidates

from .headless import HeadlessSimulator, simulation_steps_from_execution


@dataclass(frozen=True, slots=True)
class Task1DemoScenario:
    simulator: HeadlessSimulator
    config: PlanningConfig
    planning_result: PlanningResult
    candidate_groups: tuple[ObservationCandidateGroup, ...]


def task1_demo_config() -> PlanningConfig:
    """Return the bounded simulation profile used by the five-target demo.

    The 3 cm margin matches the existing B.1 visual profile. One nominal
    candidate per target keeps startup to 25 deterministic directed searches;
    the production router and automated tests retain full 8 x 3 support.
    """
    base = UNCALIBRATED_SIMULATION_CONFIG
    return replace(
        base,
        robot=replace(base.robot, safety_margin_cm=3.0),
        observation_lateral_offsets_cm=(0.0,),
        guaranteed_max_candidates_per_target=1,
        max_expanded_nodes=3000,
        adaptive_initial_expansions=200,
        adaptive_max_expansions=3000,
        local_planning_timeout_s=2.0,
        overall_planning_timeout_s=60.0,
        # Keep the fixed regression demo deterministic and fast; the editor
        # and production profile exercise the Phase 6.5 partial-angle set.
        turn_angles_deg=(90.0,),
        search_turn_angles_deg=(90.0,),
        heading_bin_rad=3.141592653589793 / 2.0,
    )


def task1_demo_obstacles() -> tuple[Obstacle, ...]:
    """Return the stable five-target reference layout."""
    return (
        Obstacle(1, GridCell(15, 12), Direction.WEST),
        Obstacle(2, GridCell(12, 9), Direction.NORTH),
        Obstacle(3, GridCell(2, 7), Direction.NORTH),
        Obstacle(4, GridCell(2, 19), Direction.SOUTH),
        Obstacle(5, GridCell(7, 14), Direction.WEST),
    )


def build_task1_demo() -> Task1DemoScenario:
    """Plan and adapt a real complete five-target Task 1 route."""
    config = task1_demo_config()
    start = default_start_pose(config.robot)
    arena = ArenaInput(
        start,
        task1_demo_obstacles(),
    )
    candidate_groups = generate_arena_observation_candidates(arena, config)
    result = Task1Planner(config).plan(arena, objective=CostMetric.ESTIMATED_TIME)
    if result.status is not PlanningStatus.SUCCESS or result.route is None:
        issue_text = "; ".join(issue.message for issue in result.issues)
        raise RuntimeError(f"bundled Task 1 demo is not routable: {issue_text}")

    route = result.route
    steps = simulation_steps_from_execution(route.execution_steps, config)
    simulator = HeadlessSimulator(
        arena,
        candidate_groups,
        steps,
        planned_path=route.sampled_poses,
        target_order=route.target_order,
        selected_candidates=tuple(zip(route.target_order, route.selected_candidate_kinds)),
    )
    return Task1DemoScenario(simulator, config, result, candidate_groups)


__all__ = [
    "Task1DemoScenario",
    "build_task1_demo",
    "task1_demo_config",
    "task1_demo_obstacles",
]
