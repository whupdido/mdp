"""Headless one-query Hybrid A* diagnostic used during Phase 6.5.1."""

from __future__ import annotations

import math

from algorithm.config import UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import CostMetric
from algorithm.geometry import propagate_motion
from algorithm.models import ArenaInput
from algorithm.models import GridCell, Obstacle
from algorithm.enums import Direction
from algorithm.targets import generate_arena_observation_candidates
from .task1_editor_model import task1_editor_config
from algorithm.models.pose import Pose
from algorithm.pathfinding import HybridAStarPlanner


def run_local_plan_demo() -> None:
    config = UNCALIBRATED_SIMULATION_CONFIG
    start = Pose(80.0, 80.0, 0.0)
    primitive = next(
        item for item in config.motion.primitives if item.command == "FL"
    )
    goal = propagate_motion(start, primitive, config)
    result = HybridAStarPlanner(config).plan(
        start, goal, ArenaInput(start), objective=CostMetric.DISTANCE,
        max_expanded_nodes=5000, max_planning_time_s=5.0,
    )
    print("Local Hybrid A* diagnostic")
    print(f"start=({start.x_cm:.2f},{start.y_cm:.2f},{math.degrees(start.heading_rad):.1f}deg)")
    print(f"goal=({goal.x_cm:.2f},{goal.y_cm:.2f},{math.degrees(goal.heading_rad):.1f}deg)")
    print(f"angles={config.turn_angles_deg} heading_bin={math.degrees(config.heading_bin_rad):.1f}deg")
    print(f"status={result.status.value} expanded={result.metrics.nodes_expanded} generated={result.metrics.nodes_generated}")
    print(f"runtime={result.metrics.planning_time_s:.4f}s distance={result.metrics.geometric_distance_cm:.2f}cm")
    if result.path is not None:
        print("primitives=" + " ".join(item.command for item in result.path.primitives))
        print(f"final=({result.path.final_pose.x_cm:.2f},{result.path.final_pose.y_cm:.2f},{math.degrees(result.path.final_pose.heading_rad):.1f}deg)")


def run_open_arena_local_diagnostic() -> None:
    """Run independent START and representative candidate local queries."""
    config = task1_editor_config()
    start = Pose(15.0, 15.0, math.pi / 2.0)
    arena = ArenaInput(start, (
        Obstacle(1, GridCell(16, 1), Direction.WEST),
        Obstacle(2, GridCell(18, 15), Direction.WEST),
        Obstacle(3, GridCell(18, 6), Direction.WEST),
        Obstacle(4, GridCell(1, 6), Direction.EAST),
        Obstacle(5, GridCell(2, 14), Direction.EAST),
    ))
    groups = generate_arena_observation_candidates(arena, config)
    planner = HybridAStarPlanner(config)
    print("Real-arena local Hybrid A* diagnostic (independent queries)")
    print("target/candidate | goal x,y,heading | geometric | status | expanded/generated | collision/dominated | runtime | primitives")
    valid = []
    for group in groups:
        for candidate in group.candidates:
            if not candidate.valid:
                print(f"{group.obstacle_id}/{candidate.display_label} | ({candidate.observation_pose.pose.x_cm:.1f},{candidate.observation_pose.pose.y_cm:.1f},{math.degrees(candidate.observation_pose.pose.heading_rad):.0f}) | no | INVALID | - | - | - | -")
                continue
            valid.append((group.obstacle_id, candidate))
            result = planner.plan(start, candidate.observation_pose.pose, arena, objective=CostMetric.ESTIMATED_TIME, max_expanded_nodes=20000, max_planning_time_s=5.0)
            metrics = result.metrics
            commands = " ".join(item.command for item in result.path.primitives) if result.path else "-"
            print(f"{group.obstacle_id}/{candidate.display_label} | ({candidate.observation_pose.pose.x_cm:.1f},{candidate.observation_pose.pose.y_cm:.1f},{math.degrees(candidate.observation_pose.pose.heading_rad):.0f}) | yes | {result.status.value.upper()} | {metrics.nodes_expanded}/{metrics.nodes_generated} | {metrics.collision_rejected_successors}/{metrics.dominated_successors} | {metrics.planning_time_s:.3f}s | {commands}")
    print("Representative candidate-to-candidate queries")
    for (source_id, source), (goal_id, goal) in zip(valid, valid[1:]):
        if source_id == goal_id:
            continue
        result = planner.plan(source.observation_pose.pose, goal.observation_pose.pose, arena, objective=CostMetric.ESTIMATED_TIME, max_expanded_nodes=20000, max_planning_time_s=5.0)
        print(f"{source_id}/{source.display_label} -> {goal_id}/{goal.display_label}: {result.status.value.upper()} expanded={result.metrics.nodes_expanded} runtime={result.metrics.planning_time_s:.3f}s")
        if goal_id >= source_id + 1:
            break


__all__ = ["run_local_plan_demo", "run_open_arena_local_diagnostic"]
