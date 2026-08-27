"""Authoritative pose-level arena and obstacle collision checks."""

from __future__ import annotations

import math
from collections.abc import Sequence

from algorithm.config import PlanningConfig
from algorithm.models.arena import ArenaInput
from algorithm.models.pose import Pose

from .footprint import obstacle_bounds, robot_footprint
from .shapes import NUMERIC_TOLERANCE_CM, Point


def _separating_axes(polygon: Sequence[Point]) -> tuple[tuple[float, float], ...]:
    if len(polygon) < 3:
        raise ValueError("a collision polygon requires at least three points")
    axes: list[tuple[float, float]] = []
    for index, point in enumerate(polygon):
        following = polygon[(index + 1) % len(polygon)]
        edge_x = following.x_cm - point.x_cm
        edge_y = following.y_cm - point.y_cm
        if abs(edge_x) <= NUMERIC_TOLERANCE_CM and abs(edge_y) <= NUMERIC_TOLERANCE_CM:
            continue
        length = math.hypot(edge_x, edge_y)
        axes.append((-edge_y / length, edge_x / length))
    if not axes:
        raise ValueError("collision polygon cannot be degenerate")
    return tuple(axes)


def _projection(polygon: Sequence[Point], axis: tuple[float, float]) -> tuple[float, float]:
    axis_x, axis_y = axis
    values = tuple(point.x_cm * axis_x + point.y_cm * axis_y for point in polygon)
    return min(values), max(values)


def polygons_intersect(first: Sequence[Point], second: Sequence[Point]) -> bool:
    """Return whether two convex polygons overlap or touch using SAT.

    Physical contact is considered a collision. ``NUMERIC_TOLERANCE_CM`` only
    absorbs floating-point noise and is not a substitute for safety margin.
    """
    axes = _separating_axes(first) + _separating_axes(second)
    for axis in axes:
        first_min, first_max = _projection(first, axis)
        second_min, second_max = _projection(second, axis)
        if (
            first_max < second_min - NUMERIC_TOLERANCE_CM
            or second_max < first_min - NUMERIC_TOLERANCE_CM
        ):
            return False
    return True


def footprint_within_arena(footprint: Sequence[Point], arena_size_cm: float) -> bool:
    """Return whether every footprint corner lies inside the square arena."""
    if not math.isfinite(arena_size_cm) or arena_size_cm <= 0.0:
        raise ValueError("arena_size_cm must be positive and finite")
    return all(
        -NUMERIC_TOLERANCE_CM <= point.x_cm <= arena_size_cm + NUMERIC_TOLERANCE_CM
        and -NUMERIC_TOLERANCE_CM <= point.y_cm <= arena_size_cm + NUMERIC_TOLERANCE_CM
        for point in footprint
    )


def is_pose_collision_free(pose: Pose, arena: ArenaInput, config: PlanningConfig) -> bool:
    """Authoritative collision query for a robot pose in an arena."""
    footprint = robot_footprint(pose, config.robot)
    if not footprint_within_arena(footprint, config.arena_size_cm):
        return False
    return not any(
        polygons_intersect(footprint, obstacle_bounds(obstacle, config.cell_size_cm).corners)
        for obstacle in arena.obstacles
    )
