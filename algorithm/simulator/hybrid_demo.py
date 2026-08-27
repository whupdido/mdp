"""Pygame-free adapter from one Hybrid A* query to simulator playback."""

from __future__ import annotations

from dataclasses import dataclass

from algorithm.config import PlanningConfig, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import CostMetric, Direction
from algorithm.models.arena import ArenaInput
from algorithm.models.obstacle import Obstacle
from algorithm.models.pose import GridCell, Pose
from algorithm.pathfinding import LocalPlanningResult, LocalPlanningStatus
from algorithm.pathfinding.hybrid_astar import HybridAStarPlanner
from algorithm.targets import generate_arena_observation_candidates

from .headless import HeadlessSimulator, simulation_steps_from_primitives


@dataclass(frozen=True, slots=True)
class HybridDemoScenario:
    simulator: HeadlessSimulator
    config: PlanningConfig
    planning_result: LocalPlanningResult


def build_hybrid_demo() -> HybridDemoScenario:
    """Plan to one real Phase 3 candidate in a dedicated Phase 5 arena."""
    config = UNCALIBRATED_SIMULATION_CONFIG
    start = Pose.from_direction(95.0, 50.0, Direction.NORTH)
    arena = ArenaInput(
        start_pose=start,
        obstacles=(Obstacle(1, GridCell(9, 11), Direction.SOUTH),),
    )
    candidate_groups = generate_arena_observation_candidates(arena, config)
    goal = candidate_groups[0].valid_candidates[0].pose
    result = HybridAStarPlanner(config).plan(
        arena.start_pose,
        goal,
        arena,
        objective=CostMetric.ESTIMATED_TIME,
        collect_debug=True,
    )
    if result.status is not LocalPlanningStatus.SUCCESS or result.path is None:
        raise RuntimeError(f"bundled Hybrid A* demo is not reachable: {result.status.value}")
    steps = simulation_steps_from_primitives(
        arena.start_pose,
        result.path.primitives,
        config,
    )
    simulator = HeadlessSimulator(
        arena,
        candidate_groups,
        steps,
        planned_path=result.path.sampled_poses,
    )
    return HybridDemoScenario(simulator, config, result)


__all__ = ["HybridDemoScenario", "build_hybrid_demo"]
