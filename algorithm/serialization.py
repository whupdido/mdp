"""Shared Task 1 movement-command serialization for simulator and server."""

from __future__ import annotations

import math

from algorithm.enums import PlanningStatus, Steering
from algorithm.models.motion import CaptureStep, MoveStep


def serialize_stm_command(move: MoveStep) -> str:
    """Return the five-character movement command accepted by the STM."""
    primitive = move.segment.primitive
    if primitive.steering is Steering.STRAIGHT:
        magnitude = round(primitive.travel_cm)
    else:
        magnitude = round(abs(math.degrees(primitive.turn_angle_rad)))
    return f"{primitive.command}{magnitude:03d}"


def serialize_planning_result(result) -> dict:
    """Serialize a planner result using the production server wire shape."""
    if result.status is not PlanningStatus.SUCCESS:
        return {
            "status": result.status.value,
            "issues": [
                {"code": issue.code, "message": issue.message, "obstacle_id": issue.obstacle_id}
                for issue in result.issues
            ],
        }
    steps = []
    for step in result.route.execution_steps:
        if isinstance(step, MoveStep):
            steps.append({"type": "move", "command": serialize_stm_command(step)})
        elif isinstance(step, CaptureStep):
            steps.append({"type": "capture", "obstacle_id": step.obstacle_id})
    return {"status": "success", "steps": steps}


__all__ = ["serialize_planning_result", "serialize_stm_command"]
