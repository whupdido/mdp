"""Continuous sampling and swept-footprint validation for motion primitives."""

from __future__ import annotations

import math

from algorithm.config import PlanningConfig
from algorithm.enums import Gear, Steering
from algorithm.models.arena import ArenaInput
from algorithm.models.motion import MotionPrimitive
from algorithm.models.pose import Pose

from .collision import _is_xyh_collision_free, collision_obstacle_bounds, is_pose_collision_free


def _sample_count(magnitude: float, maximum_step: float, step_name: str) -> int:
    if not math.isfinite(maximum_step) or maximum_step <= 0.0:
        raise ValueError(f"{step_name} must be positive and finite")
    return max(1, math.ceil(magnitude / maximum_step))


def sample_straight(
    start: Pose,
    primitive: MotionPrimitive,
    maximum_step_cm: float,
) -> tuple[Pose, ...]:
    """Sample a straight primitive, including its exact start and endpoint."""
    if primitive.steering is not Steering.STRAIGHT:
        raise ValueError("sample_straight requires a straight primitive")
    steps = _sample_count(primitive.travel_cm, maximum_step_cm, "maximum_step_cm")
    direction = 1.0 if primitive.gear is Gear.FORWARD else -1.0
    return tuple(
        Pose(
            *start.translated_local(direction * primitive.travel_cm * index / steps, 0.0),
            start.heading_rad,
        )
        for index in range(steps + 1)
    )


def sample_arc(
    start: Pose,
    primitive: MotionPrimitive,
    maximum_step_rad: float,
) -> tuple[Pose, ...]:
    """Sample a constant-radius Ackermann arc at the rear-axle center."""
    if primitive.steering is Steering.STRAIGHT:
        raise ValueError("sample_arc requires a turning primitive")
    assert primitive.radius_cm is not None
    steps = _sample_count(abs(primitive.turn_angle_rad), maximum_step_rad, "maximum_step_rad")
    gear_sign = 1.0 if primitive.gear is Gear.FORWARD else -1.0
    signed_distance = gear_sign * abs(primitive.turn_angle_rad) * primitive.radius_cm
    signed_radius = signed_distance / primitive.turn_angle_rad

    samples: list[Pose] = []
    for index in range(steps + 1):
        heading_change = primitive.turn_angle_rad * index / steps
        heading = start.heading_rad + heading_change
        if index == steps:
            # Keep the sampled trajectory endpoint bit-for-bit aligned with
            # propagate_motion(), which is used by Hybrid A* successor states.
            endpoint_heading = start.heading_rad + primitive.turn_angle_rad
            samples.append(
                Pose(
                    x_cm=start.x_cm
                    + signed_radius * (
                        math.sin(endpoint_heading) - math.sin(start.heading_rad)
                    ),
                    y_cm=start.y_cm
                    - signed_radius * (
                        math.cos(endpoint_heading) - math.cos(start.heading_rad)
                    ),
                    heading_rad=endpoint_heading,
                )
            )
        else:
            samples.append(
                Pose(
                    x_cm=start.x_cm
                    + signed_radius * (math.sin(heading) - math.sin(start.heading_rad)),
                    y_cm=start.y_cm
                    - signed_radius * (math.cos(heading) - math.cos(start.heading_rad)),
                    heading_rad=heading,
                )
            )
    return tuple(samples)


def sample_motion(
    start: Pose,
    primitive: MotionPrimitive,
    config: PlanningConfig,
) -> tuple[Pose, ...]:
    """Sample any configured primitive using the central collision resolution."""
    if primitive.steering is Steering.STRAIGHT:
        return sample_straight(start, primitive, config.collision_translation_step_cm)
    return sample_arc(start, primitive, config.collision_arc_step_rad)


def propagate_motion(start: Pose, primitive: MotionPrimitive, config: PlanningConfig) -> Pose:
    """Return the exact endpoint produced by a configured primitive without sampling."""
    if primitive.steering is Steering.STRAIGHT:
        direction = 1.0 if primitive.gear is Gear.FORWARD else -1.0
        x, y = start.translated_local(direction * primitive.travel_cm, 0.0)
        return Pose(x, y, start.heading_rad)

    assert primitive.radius_cm is not None
    gear_sign = 1.0 if primitive.gear is Gear.FORWARD else -1.0
    signed_distance = gear_sign * abs(primitive.turn_angle_rad) * primitive.radius_cm
    signed_radius = signed_distance / primitive.turn_angle_rad
    heading = start.heading_rad + primitive.turn_angle_rad
    start_sin = math.sin(start.heading_rad)
    start_cos = math.cos(start.heading_rad)
    return Pose(
        x_cm=start.x_cm + signed_radius * (math.sin(heading) - start_sin),
        y_cm=start.y_cm - signed_radius * (math.cos(heading) - start_cos),
        heading_rad=heading,
    )


def is_motion_collision_free(
    start: Pose,
    primitive: MotionPrimitive,
    arena: ArenaInput,
    config: PlanningConfig,
    *,
    sampled_poses: tuple[Pose, ...] | None = None,
) -> bool:
    """Validate the full swept footprint.

    During search, callers omit ``sampled_poses`` and this routine performs
    allocation-free raw-pose sampling.  The optional tuple remains supported
    for callers that already have sampled poses (e.g. tests/integration).
    """
    if sampled_poses is not None:
        return all(
            is_pose_collision_free(pose, arena, config)
            for pose in sampled_poses
        )

    obstacle_bounds = collision_obstacle_bounds(arena, config)

    if primitive.steering is Steering.STRAIGHT:
        steps = _sample_count(
            primitive.travel_cm,
            config.collision_translation_step_cm,
            "maximum_step_cm",
        )
        direction = 1.0 if primitive.gear is Gear.FORWARD else -1.0
        distance = primitive.travel_cm
        start_heading = start.heading_rad
        cosine = math.cos(start_heading)
        sine = math.sin(start_heading)
        for index in range(steps + 1):
            distance_i = direction * distance * index / steps
            x = start.x_cm + distance_i * cosine
            y = start.y_cm + distance_i * sine
            if not _is_xyh_collision_free(
                x, y, start_heading, arena, config,
                cosine=cosine, sine=sine, obstacle_bounds=obstacle_bounds,
            ):
                return False
        return True

    assert primitive.radius_cm is not None
    steps = _sample_count(
        abs(primitive.turn_angle_rad),
        config.collision_arc_step_rad,
        "maximum_step_rad",
    )
    gear_sign = 1.0 if primitive.gear is Gear.FORWARD else -1.0
    signed_distance = gear_sign * abs(primitive.turn_angle_rad) * primitive.radius_cm
    signed_radius = signed_distance / primitive.turn_angle_rad
    start_heading = start.heading_rad
    start_sin = math.sin(start_heading)
    start_cos = math.cos(start_heading)

    for index in range(steps + 1):
        heading_change = primitive.turn_angle_rad * index / steps
        heading = start_heading + heading_change
        sin_heading = math.sin(heading)
        cos_heading = math.cos(heading)
        x = start.x_cm + signed_radius * (sin_heading - start_sin)
        y = start.y_cm - signed_radius * (cos_heading - start_cos)
        if not _is_xyh_collision_free(
            x, y, heading, arena, config,
            cosine=cos_heading, sine=sin_heading,
            obstacle_bounds=obstacle_bounds,
        ):
            return False
    return True

