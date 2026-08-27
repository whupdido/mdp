"""Deterministic, Pygame-free Task 1 playback state machine."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum

from algorithm.config import PlanningConfig
from algorithm.geometry import sample_motion
from algorithm.models.arena import ArenaInput
from algorithm.models.motion import CaptureStep, ExecutionStep, MoveStep, MotionPrimitive
from algorithm.models.pose import Pose
from algorithm.targets.models import ObservationCandidateGroup


class PlaybackState(Enum):
    READY = "ready"
    PLAYING = "playing"
    PAUSED = "paused"
    COMPLETE = "complete"


@dataclass(frozen=True, slots=True)
class SimulationStep:
    """One logical playback step: a sampled pose or a target capture."""

    duration_s: float
    pose: Pose | None = None
    capture_obstacle_id: int | None = None
    motion_command: str | None = None
    ends_primitive: bool = False

    def __post_init__(self) -> None:
        if not math.isfinite(self.duration_s) or self.duration_s < 0.0:
            raise ValueError("duration_s must be finite and non-negative")
        if (self.pose is None) == (self.capture_obstacle_id is None):
            raise ValueError("a simulation step must contain exactly one pose or capture event")
        if self.capture_obstacle_id is not None and self.capture_obstacle_id <= 0:
            raise ValueError("capture_obstacle_id must be positive")
        if self.pose is None and self.motion_command is not None:
            raise ValueError("capture events cannot define a motion command")
        if self.pose is None and self.ends_primitive:
            raise ValueError("capture events cannot end a motion primitive")

    @classmethod
    def motion(
        cls,
        pose: Pose,
        duration_s: float,
        command: str | None = None,
        *,
        ends_primitive: bool = False,
    ) -> SimulationStep:
        return cls(
            duration_s=duration_s,
            pose=pose,
            motion_command=command,
            ends_primitive=ends_primitive,
        )

    @classmethod
    def capture(cls, obstacle_id: int) -> SimulationStep:
        return cls(duration_s=0.0, capture_obstacle_id=obstacle_id)


@dataclass(frozen=True, slots=True)
class SimulationState:
    arena: ArenaInput
    candidate_groups: tuple[ObservationCandidateGroup, ...]
    initial_pose: Pose
    robot_pose: Pose
    planned_path: tuple[Pose, ...]
    executed_path: tuple[Pose, ...]
    current_step_index: int
    total_steps: int
    current_motion_command: str | None
    visited_target_ids: tuple[int, ...]
    simulation_time_s: float
    playback_state: PlaybackState


def _motion_duration(primitive: MotionPrimitive, config: PlanningConfig) -> float:
    if primitive.estimated_duration_s > 0.0:
        return primitive.estimated_duration_s
    return primitive.geometric_length_cm / config.motion.straight_speed_cm_s


def simulation_steps_from_primitives(
    start: Pose,
    primitives: tuple[MotionPrimitive, ...],
    config: PlanningConfig,
) -> tuple[SimulationStep, ...]:
    """Convert configured primitives into deterministic sampled pose steps."""
    current = start
    steps: list[SimulationStep] = []
    for primitive in primitives:
        samples = sample_motion(current, primitive, config)
        transition_count = len(samples) - 1
        duration_per_sample = _motion_duration(primitive, config) / transition_count
        steps.extend(
            SimulationStep.motion(
                pose,
                duration_per_sample,
                primitive.command,
                ends_primitive=index == transition_count,
            )
            for index, pose in enumerate(samples[1:], start=1)
        )
        current = samples[-1]
    return tuple(steps)


def simulation_steps_from_poses(
    poses: tuple[Pose, ...],
    duration_per_pose_s: float,
    *,
    motion_command: str | None = None,
) -> tuple[SimulationStep, ...]:
    """Adapt an already sampled path for the same playback state machine."""
    return tuple(
        SimulationStep.motion(
            pose,
            duration_per_pose_s,
            motion_command,
            ends_primitive=True,
        )
        for pose in poses
    )


def simulation_steps_from_execution(
    execution_steps: tuple[ExecutionStep, ...],
    config: PlanningConfig,
) -> tuple[SimulationStep, ...]:
    """Adapt future route execution steps without coupling playback to a planner."""
    result: list[SimulationStep] = []
    for execution_step in execution_steps:
        if isinstance(execution_step, CaptureStep):
            result.append(SimulationStep.capture(execution_step.obstacle_id))
            continue
        if not isinstance(execution_step, MoveStep):
            raise TypeError(f"unsupported execution step: {type(execution_step)!r}")
        result.extend(
            simulation_steps_from_primitives(
                execution_step.segment.start,
                (execution_step.segment.primitive,),
                config,
            )
        )
    return tuple(result)


def _path_from_steps(initial_pose: Pose, steps: tuple[SimulationStep, ...]) -> tuple[Pose, ...]:
    path: list[Pose] = [initial_pose]
    for step in steps:
        if step.pose is not None and step.pose != path[-1]:
            path.append(step.pose)
    return tuple(path)


class HeadlessSimulator:
    """Mutable controller exposing immutable deterministic state snapshots."""

    _TIME_EPSILON_S = 1e-12

    def __init__(
        self,
        arena: ArenaInput,
        candidate_groups: tuple[ObservationCandidateGroup, ...],
        steps: tuple[SimulationStep, ...],
        *,
        planned_path: tuple[Pose, ...] | None = None,
    ) -> None:
        self._arena = arena
        self._candidate_groups = tuple(candidate_groups)
        self._steps = tuple(steps)
        obstacle_ids = {obstacle.obstacle_id for obstacle in arena.obstacles}
        unknown_captures = {
            step.capture_obstacle_id
            for step in self._steps
            if step.capture_obstacle_id is not None and step.capture_obstacle_id not in obstacle_ids
        }
        if unknown_captures:
            raise ValueError(f"capture events reference unknown obstacles: {sorted(unknown_captures)}")
        self._planned_path = (
            tuple(planned_path)
            if planned_path is not None
            else _path_from_steps(arena.start_pose, self._steps)
        )
        if not self._planned_path:
            raise ValueError("planned_path cannot be empty")
        self._elapsed_in_step_s = 0.0
        self._state = self._initial_state()

    def _initial_state(self) -> SimulationState:
        return SimulationState(
            arena=self._arena,
            candidate_groups=self._candidate_groups,
            initial_pose=self._arena.start_pose,
            robot_pose=self._arena.start_pose,
            planned_path=self._planned_path,
            executed_path=(self._arena.start_pose,),
            current_step_index=0,
            total_steps=len(self._steps),
            current_motion_command=None,
            visited_target_ids=(),
            simulation_time_s=0.0,
            playback_state=PlaybackState.READY,
        )

    @property
    def state(self) -> SimulationState:
        return self._state

    @property
    def steps(self) -> tuple[SimulationStep, ...]:
        return self._steps

    def play(self) -> None:
        if self._state.playback_state is PlaybackState.COMPLETE:
            return
        self._state = replace(
            self._state,
            playback_state=PlaybackState.PLAYING,
            current_motion_command=self._pending_motion_command(),
        )

    def pause(self) -> None:
        if self._state.playback_state is PlaybackState.PLAYING:
            self._state = replace(self._state, playback_state=PlaybackState.PAUSED)

    def reset(self) -> None:
        self._elapsed_in_step_s = 0.0
        self._state = self._initial_state()

    def step_once(self) -> bool:
        """Complete exactly one logical step, independent of playback state."""
        if self._state.current_step_index >= len(self._steps):
            self._mark_complete()
            return False
        step = self._steps[self._state.current_step_index]
        remaining_duration = max(0.0, step.duration_s - self._elapsed_in_step_s)
        self._state = replace(
            self._state,
            simulation_time_s=self._state.simulation_time_s + remaining_duration,
        )
        self._elapsed_in_step_s = 0.0
        self._execute_step(step)
        if self._state.playback_state is not PlaybackState.COMPLETE:
            self._state = replace(
                self._state,
                playback_state=PlaybackState.PAUSED,
                current_motion_command=self._pending_motion_command(),
            )
        return True

    def step_primitive(self) -> bool:
        """Complete one primitive or capture event while remaining paused."""
        if self._state.current_step_index >= len(self._steps):
            self._mark_complete()
            return False
        advanced = False
        while self._state.current_step_index < len(self._steps):
            step = self._steps[self._state.current_step_index]
            self.step_once()
            advanced = True
            if step.capture_obstacle_id is not None or step.ends_primitive:
                break
        return advanced

    def advance(self, delta_s: float) -> None:
        """Advance logical time; rendering frame count has no authority here."""
        if not math.isfinite(delta_s) or delta_s < 0.0:
            raise ValueError("delta_s must be finite and non-negative")
        if self._state.playback_state is not PlaybackState.PLAYING:
            return

        remaining = delta_s
        while self._state.current_step_index < len(self._steps):
            step = self._steps[self._state.current_step_index]
            step_remaining = max(0.0, step.duration_s - self._elapsed_in_step_s)
            if step_remaining <= self._TIME_EPSILON_S:
                self._elapsed_in_step_s = 0.0
                self._execute_step(step)
                continue
            if remaining <= self._TIME_EPSILON_S:
                break
            consumed = min(remaining, step_remaining)
            remaining -= consumed
            self._elapsed_in_step_s += consumed
            self._state = replace(
                self._state,
                simulation_time_s=self._state.simulation_time_s + consumed,
            )
            if step_remaining - consumed <= self._TIME_EPSILON_S:
                self._elapsed_in_step_s = 0.0
                self._execute_step(step)
        if self._state.current_step_index >= len(self._steps):
            self._mark_complete()

    def _execute_step(self, step: SimulationStep) -> None:
        robot_pose = self._state.robot_pose
        executed_path = self._state.executed_path
        visited = self._state.visited_target_ids
        if step.pose is not None:
            robot_pose = step.pose
            if step.pose != executed_path[-1]:
                executed_path = executed_path + (step.pose,)
        else:
            assert step.capture_obstacle_id is not None
            if step.capture_obstacle_id not in visited:
                visited = visited + (step.capture_obstacle_id,)

        next_index = self._state.current_step_index + 1
        self._state = replace(
            self._state,
            robot_pose=robot_pose,
            executed_path=executed_path,
            visited_target_ids=visited,
            current_step_index=next_index,
            current_motion_command=self._pending_motion_command(next_index),
        )
        if next_index >= len(self._steps):
            self._mark_complete()

    def _pending_motion_command(self, index: int | None = None) -> str | None:
        pending_index = self._state.current_step_index if index is None else index
        if pending_index >= len(self._steps):
            return None
        return self._steps[pending_index].motion_command

    def _mark_complete(self) -> None:
        self._state = replace(
            self._state,
            playback_state=PlaybackState.COMPLETE,
            current_motion_command=None,
        )
