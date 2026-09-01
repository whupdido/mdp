"""Deterministic, Pygame-free Phase 4 demonstration scenario."""

from __future__ import annotations

from dataclasses import replace

from algorithm.config import PlanningConfig, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import Direction
from algorithm.geometry import is_motion_collision_free, is_pose_collision_free, propagate_motion
from algorithm.models.arena import ArenaInput
from algorithm.models.obstacle import Obstacle
from algorithm.models.pose import GridCell, Pose
from algorithm.targets import generate_arena_observation_candidates

from .headless import HeadlessSimulator, SimulationStep, simulation_steps_from_primitives


DEMO_COMMAND_SEQUENCE = (
    "FW", "FW", "FW", "FW", "FW",
    "FR", "FW", "FW", "FL", "BW", "BL", "BR",
)


def demo_config() -> PlanningConfig:
    """Return the simulation-only profile used by the bundled demo.

    The documented start axle is only 15 cm from the arena edges. The default
    5 cm safety expansion makes the 23 cm body extend 1.5 cm outside the arena,
    so this visual demonstration explicitly uses a provisional 3 cm margin.
    The production profile remains unchanged.
    """
    base = UNCALIBRATED_SIMULATION_CONFIG
    return replace(base, robot=replace(base.robot, safety_margin_cm=3.0))


def build_demo_simulator() -> tuple[HeadlessSimulator, PlanningConfig]:
    """Build a collision-checked route exercising every v1 command kind."""
    config = demo_config()
    start = Pose.from_direction(15.0, 15.0, Direction.NORTH)
    arena = ArenaInput(
        start_pose=start,
        obstacles=(
            # Keep the primary visual scenario comfortably inside the arena;
            # boundary-rejection cases belong to geometry/target tests.
            Obstacle(1, GridCell(14, 4), Direction.WEST),
            Obstacle(2, GridCell(9, 15), Direction.SOUTH),
            Obstacle(3, GridCell(14, 11), Direction.WEST),
        ),
    )
    if not is_pose_collision_free(start, arena, config):
        raise RuntimeError("the configured demo start pose is not collision-free")

    # The repeated second capture is deliberate: primitive/event stepping can
    # visibly verify that duplicate capture events do not increase visited
    # target count.
    captures_after_command = {8: (2, 2)}
    current = start
    steps: list[SimulationStep] = []
    for index, command in enumerate(DEMO_COMMAND_SEQUENCE):
        primitive = config.motion.primitives_for(command)[0]
        if not is_motion_collision_free(current, primitive, arena, config):
            raise RuntimeError(f"demo command {index + 1} ({command}) is not collision-free")
        steps.extend(simulation_steps_from_primitives(current, (primitive,), config))
        current = propagate_motion(current, primitive, config)
        for obstacle_id in captures_after_command.get(index, ()):
            steps.append(SimulationStep.capture(obstacle_id))

    candidate_groups = generate_arena_observation_candidates(arena, config)
    return HeadlessSimulator(arena, candidate_groups, tuple(steps)), config


__all__ = ["DEMO_COMMAND_SEQUENCE", "build_demo_simulator", "demo_config"]
