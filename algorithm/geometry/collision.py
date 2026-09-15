"""Authoritative pose-level arena and obstacle collision checks."""

from __future__ import annotations

import math
from functools import lru_cache
from collections.abc import Sequence

from algorithm.config import PlanningConfig
from algorithm.models.arena import ArenaInput
from algorithm.models.pose import Pose
from algorithm.models.obstacle import RectangleObstacle

from .footprint import robot_footprint
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


def footprint_within_arena(
    footprint: Sequence[Point],
    arena_width_cm: float,
    arena_height_cm: float | None = None,
) -> bool:
    """Return whether every footprint point lies inside the arena.

    If arena_height_cm is omitted, the arena is treated as square for
    backwards compatibility with the original Task 1 implementation.
    """
    if arena_height_cm is None:
        arena_height_cm = arena_width_cm

    if not math.isfinite(arena_width_cm) or arena_width_cm <= 0.0:
        raise ValueError("arena_width_cm must be positive and finite")

    if not math.isfinite(arena_height_cm) or arena_height_cm <= 0.0:
        raise ValueError("arena_height_cm must be positive and finite")

    return all(
        -NUMERIC_TOLERANCE_CM
        <= point.x_cm
        <= arena_width_cm + NUMERIC_TOLERANCE_CM
        and -NUMERIC_TOLERANCE_CM
        <= point.y_cm
        <= arena_height_cm + NUMERIC_TOLERANCE_CM
        for point in footprint
    )


def is_pose_collision_free(pose: Pose, arena: ArenaInput, config: PlanningConfig) -> bool:
    """Authoritative collision query for a robot pose in an arena."""
    footprint = robot_footprint(pose, config.robot)
    arena_width = arena.width_cm if arena.width_cm is not None else config.arena_size_cm
    arena_height = arena.height_cm if arena.height_cm is not None else config.arena_height_cm
    if not footprint_within_arena(footprint, arena_width, arena_height):
        return False
    footprint_min_x = min(point.x_cm for point in footprint)
    footprint_max_x = max(point.x_cm for point in footprint)
    footprint_min_y = min(point.y_cm for point in footprint)
    footprint_max_y = max(point.y_cm for point in footprint)
    for obstacle in arena.obstacles:
        if isinstance(obstacle, RectangleObstacle):
            rectangle = obstacle.bounds
            bounds_min_x, bounds_min_y = rectangle.min_x_cm, rectangle.min_y_cm
            bounds_max_x, bounds_max_y = rectangle.max_x_cm, rectangle.max_y_cm
            polygon = (
                Point(bounds_min_x, bounds_min_y),
                Point(bounds_max_x, bounds_min_y),
                Point(bounds_max_x, bounds_max_y),
                Point(bounds_min_x, bounds_max_y),
            )
        else:
            cached = _cached_obstacle_bounds(obstacle.cell.x, obstacle.cell.y, config.cell_size_cm)
            bounds_min_x, bounds_min_y, bounds_max_x, bounds_max_y, polygon = cached
        if (bounds_max_x < footprint_min_x or bounds_min_x > footprint_max_x or
                bounds_max_y < footprint_min_y or bounds_min_y > footprint_max_y):
            continue
        if polygons_intersect(footprint, polygon):
            return False
    return True


@lru_cache(maxsize=512)
def _cached_obstacle_bounds(x: int, y: int, cell_size_cm: float):
    min_x = x * cell_size_cm
    min_y = y * cell_size_cm
    max_x = (x + 1) * cell_size_cm
    max_y = (y + 1) * cell_size_cm
    return (min_x, min_y, max_x, max_y, (
        Point(min_x, min_y), Point(max_x, min_y),
        Point(max_x, max_y), Point(min_x, max_y),
    ))
