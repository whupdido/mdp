"""Configurable local-path cost calculations."""

from __future__ import annotations

from algorithm.config import MotionModel
from algorithm.enums import CostMetric, Gear, Steering
from algorithm.models.motion import MotionPrimitive


def primitive_execution_time_s(primitive: MotionPrimitive, motion: MotionModel) -> float:
    """Return provisional execution time for one uncoalesced primitive."""
    if primitive.estimated_duration_s > 0.0:
        movement_time = primitive.estimated_duration_s
    else:
        cruising_distance = max(0.0, primitive.geometric_length_cm - motion.straight_deceleration_cm)
        movement_time = (
            motion.straight_fixed_time_s
            + cruising_distance / motion.straight_speed_cm_s
            + motion.straight_settle_s
        )
    return movement_time + motion.serial_overhead_s


def transition_cost(
    primitive: MotionPrimitive,
    motion: MotionModel,
    objective: CostMetric,
    previous_gear: Gear | None,
    previous_steering: Steering | None,
) -> float:
    if objective is CostMetric.DISTANCE:
        return primitive.geometric_length_cm
    if objective is not CostMetric.ESTIMATED_TIME:
        raise ValueError(f"unsupported cost metric: {objective!r}")
    cost = primitive_execution_time_s(primitive, motion)
    if previous_gear is not None and previous_gear is not primitive.gear:
        cost += motion.direction_change_penalty_s
    if previous_steering is not None and previous_steering is not primitive.steering:
        cost += motion.steering_change_penalty_s
    return cost


__all__ = ["primitive_execution_time_s", "transition_cost"]
