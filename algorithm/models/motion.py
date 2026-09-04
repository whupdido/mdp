"""Configurable command-aligned motion and execution contracts."""

from __future__ import annotations

import math
from dataclasses import dataclass

from algorithm.enums import Gear, Steering
from algorithm.models.pose import Pose


@dataclass(frozen=True, slots=True)
class MotionPrimitive:
    """One planner successor definition and its executable STM32 command kind."""

    command: str
    gear: Gear
    steering: Steering
    travel_cm: float = 0.0
    turn_angle_rad: float = 0.0
    radius_cm: float | None = None
    estimated_duration_s: float = 0.0
    physically_calibrated: bool = False

    def __post_init__(self) -> None:
        command = self.command.strip().upper()
        if command not in {"FW", "BW", "FL", "FR", "BL", "BR"}:
            raise ValueError(f"unsupported motion command: {self.command!r}")
        object.__setattr__(self, "command", command)
        numeric = (self.travel_cm, self.turn_angle_rad, self.estimated_duration_s)
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("motion primitive values must be finite")
        if self.estimated_duration_s < 0.0:
            raise ValueError("estimated_duration_s cannot be negative")

        if self.steering is Steering.STRAIGHT:
            if self.travel_cm <= 0.0:
                raise ValueError("straight primitives require positive travel_cm")
            if self.turn_angle_rad != 0.0 or self.radius_cm is not None:
                raise ValueError("straight primitives cannot define an angle or radius")
            expected = "FW" if self.gear is Gear.FORWARD else "BW"
        else:
            if self.travel_cm != 0.0:
                raise ValueError("turn primitives derive travel from radius and angle")
            if self.turn_angle_rad == 0.0:
                raise ValueError("turn primitives require a non-zero heading change")
            if self.radius_cm is None or self.radius_cm <= 0.0 or not math.isfinite(self.radius_cm):
                raise ValueError("turn primitives require a positive finite radius")
            prefix = "F" if self.gear is Gear.FORWARD else "B"
            suffix = "L" if self.steering is Steering.LEFT else "R"
            expected = prefix + suffix
            expected_sign = {"FL": 1.0, "FR": -1.0, "BL": -1.0, "BR": 1.0}[expected]
            if self.turn_angle_rad * expected_sign <= 0.0:
                raise ValueError(f"turn angle sign conflicts with command {expected}")

        if command != expected:
            raise ValueError(f"command {command} conflicts with {self.gear.value}/{self.steering.value}")

    @property
    def geometric_length_cm(self) -> float:
        if self.steering is Steering.STRAIGHT:
            return self.travel_cm
        assert self.radius_cm is not None
        return abs(self.turn_angle_rad) * self.radius_cm


@dataclass(frozen=True, slots=True)
class MotionSegment:
    primitive: MotionPrimitive
    start: Pose
    end: Pose


@dataclass(frozen=True, slots=True)
class MoveStep:
    segment: MotionSegment
    command: str

    def __post_init__(self) -> None:
        if not self.command.strip():
            raise ValueError("command cannot be empty")


@dataclass(frozen=True, slots=True)
class CaptureStep:
    obstacle_id: int
    pose: Pose

    def __post_init__(self) -> None:
        if self.obstacle_id <= 0:
            raise ValueError("obstacle_id must be positive")


ExecutionStep = MoveStep | CaptureStep
