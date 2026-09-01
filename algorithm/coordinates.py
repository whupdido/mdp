"""Transforms between Android display cells and rear-axle planner poses."""

from __future__ import annotations

import math

from algorithm.config import RobotGeometry
from algorithm.constants import CELL_SIZE_CM, START_CELL_X, START_CELL_Y
from algorithm.enums import Direction
from algorithm.models.pose import GridCell, Pose


def android_cell_to_planner_pose(
    cell: GridCell,
    heading: Direction,
    geometry: RobotGeometry,
    *,
    cell_size_cm: float = CELL_SIZE_CM,
) -> Pose:
    """Convert Android's body-center cell pose to a rear-axle pose."""
    body_x, body_y = cell.center_cm(cell_size_cm)
    body_pose = Pose.from_direction(body_x, body_y, heading)
    rear_x, rear_y = body_pose.translated_local(
        -geometry.rear_axle_to_body_center_forward_cm,
        -geometry.rear_axle_to_body_center_left_cm,
    )
    return Pose(
        x_cm=rear_x,
        y_cm=rear_y,
        heading_rad=body_pose.heading_rad,
    )


def planner_pose_to_body_center(pose: Pose, geometry: RobotGeometry) -> Pose:
    x_cm, y_cm = pose.translated_local(
        geometry.rear_axle_to_body_center_forward_cm,
        geometry.rear_axle_to_body_center_left_cm,
    )
    return Pose(x_cm=x_cm, y_cm=y_cm, heading_rad=pose.heading_rad)


def planner_pose_to_android_cell(
    pose: Pose,
    geometry: RobotGeometry,
    *,
    cell_size_cm: float = CELL_SIZE_CM,
) -> GridCell:
    """Map the configured body center to its containing Android grid cell."""
    if cell_size_cm <= 0.0 or not math.isfinite(cell_size_cm):
        raise ValueError("cell_size_cm must be positive and finite")
    body = planner_pose_to_body_center(pose, geometry)
    return GridCell(x=math.floor(body.x_cm / cell_size_cm), y=math.floor(body.y_cm / cell_size_cm))


def default_start_pose(
    geometry: RobotGeometry,
    *,
    cell_size_cm: float = CELL_SIZE_CM,
) -> Pose:
    """Return the documented Android start cell as a rear-axle pose."""
    return android_cell_to_planner_pose(
        GridCell(START_CELL_X, START_CELL_Y),
        Direction.NORTH,
        geometry,
        cell_size_cm=cell_size_cm,
    )
