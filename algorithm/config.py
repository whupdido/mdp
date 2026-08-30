"""Immutable physical and planner configuration.

The bundled profile is deliberately suitable for deterministic simulation,
not a claim that the physical robot has been fully calibrated.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from algorithm.constants import ARENA_SIZE_CM, CELL_SIZE_CM
from algorithm.enums import Gear, Steering
from algorithm.models.motion import MotionPrimitive


def _positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")


@dataclass(frozen=True, slots=True)
class RobotGeometry:
    """Body dimensions and rear-axle-to-body-center offset in local axes."""

    length_cm: float
    width_cm: float
    rear_axle_to_body_center_forward_cm: float = 0.0
    rear_axle_to_body_center_left_cm: float = 0.0
    safety_margin_cm: float = 0.0

    def __post_init__(self) -> None:
        _positive_finite("length_cm", self.length_cm)
        _positive_finite("width_cm", self.width_cm)
        offsets = (
            self.rear_axle_to_body_center_forward_cm,
            self.rear_axle_to_body_center_left_cm,
            self.safety_margin_cm,
        )
        if not all(math.isfinite(value) for value in offsets):
            raise ValueError("robot offsets and margin must be finite")
        if self.safety_margin_cm < 0.0:
            raise ValueError("safety_margin_cm cannot be negative")

    @property
    def collision_length_cm(self) -> float:
        return self.length_cm + 2.0 * self.safety_margin_cm

    @property
    def collision_width_cm(self) -> float:
        return self.width_cm + 2.0 * self.safety_margin_cm


@dataclass(frozen=True, slots=True)
class CameraGeometry:
    """Rear-axle-to-camera offset and desired image-face gap in local axes."""

    forward_offset_cm: float
    left_offset_cm: float
    image_gap_cm: float

    def __post_init__(self) -> None:
        values = (self.forward_offset_cm, self.left_offset_cm, self.image_gap_cm)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera values must be finite")
        if self.image_gap_cm <= 0.0:
            raise ValueError("image_gap_cm must be positive")


@dataclass(frozen=True, slots=True)
class MotionModel:
    """The successor set and provisional execution timing configuration."""

    primitives: tuple[MotionPrimitive, ...]
    straight_speed_cm_s: float
    serial_overhead_s: float = 0.05
    straight_settle_s: float = 0.2
    straight_fixed_time_s: float = 0.6
    straight_deceleration_cm: float = 8.2
    capture_delay_s: float = 0.0
    direction_change_penalty_s: float = 0.0
    steering_change_penalty_s: float = 0.0

    def __post_init__(self) -> None:
        primitives = tuple(self.primitives)
        object.__setattr__(self, "primitives", primitives)
        if not primitives:
            raise ValueError("motion model requires at least one primitive")
        _positive_finite("straight_speed_cm_s", self.straight_speed_cm_s)
        timings = (
            self.serial_overhead_s,
            self.straight_settle_s,
            self.straight_fixed_time_s,
            self.straight_deceleration_cm,
            self.capture_delay_s,
            self.direction_change_penalty_s,
            self.steering_change_penalty_s,
        )
        if not all(math.isfinite(value) for value in timings) or any(value < 0.0 for value in timings):
            raise ValueError("motion timing values must be finite and non-negative")

    def primitives_for(self, command: str) -> tuple[MotionPrimitive, ...]:
        normalized = command.strip().upper()
        matches = tuple(primitive for primitive in self.primitives if primitive.command == normalized)
        if not matches:
            raise KeyError(normalized)
        return matches


@dataclass(frozen=True, slots=True)
class PlanningConfig:
    robot: RobotGeometry
    camera: CameraGeometry
    motion: MotionModel
    arena_size_cm: float = ARENA_SIZE_CM
    cell_size_cm: float = CELL_SIZE_CM
    observation_lateral_offsets_cm: tuple[float, ...] = (0.0, -10.0, 10.0)
    observation_standoff_distances_cm: tuple[float, ...] = ()
    collision_translation_step_cm: float = 1.0
    collision_arc_step_rad: float = math.radians(2.0)
    position_bin_cm: float = 5.0
    # Search bookkeeping bins may be finer than the cardinal observation
    # headings.  Continuous poses are never snapped to these bins.
    heading_bin_rad: float = math.radians(15.0)
    turn_angles_deg: tuple[float, ...] = (30.0, 45.0, 60.0, 90.0)
    search_turn_angles_deg: tuple[float, ...] | None = None
    goal_position_tolerance_cm: float = 5.0
    goal_heading_tolerance_rad: float = 1e-6
    guaranteed_max_targets: int = 8
    guaranteed_max_candidates_per_target: int = 9
    physically_calibrated: bool = False
    # Partial-angle branching is wider than the original six-successor model;
    # keep the default bounded while allowing callers to raise it explicitly.
    max_expanded_nodes: int = 10_000
    candidate_activation_tiers: tuple[tuple[int, ...], ...] = (
        (0,),
        (1, 2),
        (3, 6),
        (4, 5, 7, 8),
    )
    adaptive_initial_expansions: int = 20
    adaptive_max_expansions: int = 100
    adaptive_growth_factor: float = 5.0
    local_planning_timeout_s: float = 2.0
    overall_planning_timeout_s: float = 30.0

    def __post_init__(self) -> None:
        offsets = tuple(self.observation_lateral_offsets_cm)
        standoffs = tuple(self.observation_standoff_distances_cm) or (self.camera.image_gap_cm,)
        if standoffs[0] != self.camera.image_gap_cm:
            standoffs = (self.camera.image_gap_cm,) + tuple(
                value for value in standoffs[1:] if value != self.camera.image_gap_cm
            )
        object.__setattr__(self, "observation_lateral_offsets_cm", offsets)
        object.__setattr__(self, "observation_standoff_distances_cm", standoffs)
        tiers = tuple(tuple(tier) for tier in self.candidate_activation_tiers)
        object.__setattr__(self, "candidate_activation_tiers", tiers)
        positive_values = {
            "arena_size_cm": self.arena_size_cm,
            "cell_size_cm": self.cell_size_cm,
            "collision_translation_step_cm": self.collision_translation_step_cm,
            "collision_arc_step_rad": self.collision_arc_step_rad,
            "position_bin_cm": self.position_bin_cm,
            "heading_bin_rad": self.heading_bin_rad,
            "goal_position_tolerance_cm": self.goal_position_tolerance_cm,
            "goal_heading_tolerance_rad": self.goal_heading_tolerance_rad,
        }
        for name, value in positive_values.items():
            _positive_finite(name, value)
        if not offsets:
            raise ValueError("at least one observation offset is required")
        if offsets[0] != 0.0:
            raise ValueError("the nominal zero observation offset must be first")
        if not all(math.isfinite(value) for value in offsets):
            raise ValueError("observation offsets must be finite")
        if not standoffs:
            raise ValueError("at least one observation standoff is required")
        if len(set(standoffs)) != len(standoffs):
            raise ValueError("observation standoffs must be unique")
        if not all(math.isfinite(value) and value > 0.0 for value in standoffs):
            raise ValueError("observation standoffs must be positive and finite")
        angles = tuple(float(value) for value in self.turn_angles_deg)
        search_angles = tuple(float(value) for value in (self.search_turn_angles_deg or angles))
        object.__setattr__(self, "search_turn_angles_deg", search_angles)
        # Custom compact test/development motion models that explicitly use a
        # single symmetric radius retain their declared six successors unless
        # a profile opts into partial-angle expansion.
        turn_radii = tuple(
            primitive.radius_cm for primitive in self.motion.primitives
            if primitive.steering is not Steering.STRAIGHT and primitive.radius_cm is not None
        )
        if turn_radii and len(set(turn_radii)) == 1 and angles == (30.0, 45.0, 60.0, 90.0):
            angles = (90.0,)
        if any(value not in angles for value in search_angles):
            search_angles = angles
            object.__setattr__(self, "search_turn_angles_deg", search_angles)
        object.__setattr__(self, "turn_angles_deg", angles)
        if not angles or len(set(angles)) != len(angles):
            raise ValueError("turn_angles_deg must contain unique angles")
        if any(not math.isfinite(value) or value <= 0.0 or value > 360.0 for value in angles):
            raise ValueError("turn_angles_deg must be in (0, 360]")
        if not search_angles or any(value not in angles for value in search_angles):
            raise ValueError("search_turn_angles_deg must be a non-empty subset of turn_angles_deg")
        if self.guaranteed_max_targets <= 0 or self.guaranteed_max_candidates_per_target <= 0:
            raise ValueError("performance bounds must be positive")
        if (
            isinstance(self.max_expanded_nodes, bool)
            or not isinstance(self.max_expanded_nodes, int)
            or self.max_expanded_nodes <= 0
        ):
            raise ValueError("max_expanded_nodes must be a positive integer")
        budgets = (self.adaptive_initial_expansions, self.adaptive_max_expansions)
        if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in budgets):
            raise ValueError("adaptive expansion budgets must be positive integers")
        if self.adaptive_initial_expansions > self.adaptive_max_expansions:
            raise ValueError("initial adaptive budget cannot exceed maximum budget")
        _positive_finite("adaptive_growth_factor", self.adaptive_growth_factor)
        if self.adaptive_growth_factor <= 1.0:
            raise ValueError("adaptive_growth_factor must exceed one")
        _positive_finite("local_planning_timeout_s", self.local_planning_timeout_s)
        _positive_finite("overall_planning_timeout_s", self.overall_planning_timeout_s)
        flattened = tuple(index for tier in tiers for index in tier)
        if not tiers or not all(tier for tier in tiers):
            raise ValueError("candidate activation tiers cannot be empty")
        if any(index < 0 for index in flattened) or len(set(flattened)) != len(flattened):
            raise ValueError("candidate activation indices must be unique and non-negative")


def _simulation_motion_model() -> MotionModel:
    quarter_turn = math.pi / 2.0
    return MotionModel(
        primitives=(
            MotionPrimitive("FW", Gear.FORWARD, Steering.STRAIGHT, travel_cm=10.0),
            MotionPrimitive("BW", Gear.REVERSE, Steering.STRAIGHT, travel_cm=10.0),
            MotionPrimitive("FL", Gear.FORWARD, Steering.LEFT, turn_angle_rad=quarter_turn, radius_cm=26.1, estimated_duration_s=2.4),
            MotionPrimitive("FR", Gear.FORWARD, Steering.RIGHT, turn_angle_rad=-quarter_turn, radius_cm=31.8, estimated_duration_s=2.9),
            MotionPrimitive("BL", Gear.REVERSE, Steering.LEFT, turn_angle_rad=-quarter_turn, radius_cm=24.6, estimated_duration_s=2.3),
            MotionPrimitive("BR", Gear.REVERSE, Steering.RIGHT, turn_angle_rad=quarter_turn, radius_cm=30.3, estimated_duration_s=2.8),
        ),
        straight_speed_cm_s=27.3,
    )


UNCALIBRATED_SIMULATION_CONFIG = PlanningConfig(
    robot=RobotGeometry(
        length_cm=23.0,
        width_cm=18.8,
        rear_axle_to_body_center_forward_cm=0.0,
        rear_axle_to_body_center_left_cm=0.0,
        safety_margin_cm=5.0,
    ),
    camera=CameraGeometry(
        forward_offset_cm=11.5,
        left_offset_cm=0.0,
        image_gap_cm=20.0,
    ),
    motion=_simulation_motion_model(),
    observation_standoff_distances_cm=(20.0, 10.0, 30.0),
)
