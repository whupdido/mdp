"""Robot footprint and obstacle geometry in continuous arena coordinates."""

from __future__ import annotations

import math

from algorithm.config import RobotGeometry
from algorithm.coordinates import planner_pose_to_body_center
from algorithm.models.obstacle import Obstacle
from algorithm.models.pose import Pose

from .shapes import AxisAlignedRectangle, Point


def robot_footprint(pose: Pose, geometry: RobotGeometry) -> tuple[Point, Point, Point, Point]:
    """Return the safety-expanded oriented robot rectangle.

    The safety margin is applied exactly once by expanding each side of the
    configured physical body. The rear-axle pose is first transformed to the
    configured body center, so a later calibrated axle offset needs no change
    to collision code.
    """
    body_center = planner_pose_to_body_center(pose, geometry)
    half_length = geometry.collision_length_cm / 2.0
    half_width = geometry.collision_width_cm / 2.0
    cosine = math.cos(pose.heading_rad)
    sine = math.sin(pose.heading_rad)

    def world_point(forward_cm: float, left_cm: float) -> Point:
        return Point(
            body_center.x_cm + forward_cm * cosine - left_cm * sine,
            body_center.y_cm + forward_cm * sine + left_cm * cosine,
        )

    return (
        world_point(half_length, half_width),
        world_point(-half_length, half_width),
        world_point(-half_length, -half_width),
        world_point(half_length, -half_width),
    )


def obstacle_bounds(obstacle: Obstacle, cell_size_cm: float) -> AxisAlignedRectangle:
    """Return an obstacle's exact cell rectangle in centimetres."""
    if not math.isfinite(cell_size_cm) or cell_size_cm <= 0.0:
        raise ValueError("cell_size_cm must be positive and finite")
    return AxisAlignedRectangle(
        min_x_cm=obstacle.cell.x * cell_size_cm,
        min_y_cm=obstacle.cell.y * cell_size_cm,
        max_x_cm=(obstacle.cell.x + 1) * cell_size_cm,
        max_y_cm=(obstacle.cell.y + 1) * cell_size_cm,
    )
