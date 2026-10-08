"""Authoritative pose-level arena and obstacle collision checks."""

from __future__ import annotations

import math
from collections.abc import Sequence
from functools import lru_cache

from algorithm.config import PlanningConfig
from algorithm.models.arena import ArenaInput
from algorithm.models.pose import Pose

from .shapes import NUMERIC_TOLERANCE_CM, Point


def _separating_axes(polygon: Sequence[Point]) -> tuple[tuple[float, float], ...]:
    if len(polygon) < 3:
        raise ValueError("a collision polygon requires at least three points")

    axes: list[tuple[float, float]] = []

    for index, point in enumerate(polygon):
        following = polygon[(index + 1) % len(polygon)]

        edge_x = following.x_cm - point.x_cm
        edge_y = following.y_cm - point.y_cm

        if (
            abs(edge_x) <= NUMERIC_TOLERANCE_CM
            and abs(edge_y) <= NUMERIC_TOLERANCE_CM
        ):
            continue

        length = math.hypot(edge_x, edge_y)

        axes.append(
            (
                -edge_y / length,
                edge_x / length,
            )
        )

    if not axes:
        raise ValueError("collision polygon cannot be degenerate")

    return tuple(axes)


def _projection(
    polygon: Sequence[Point],
    axis: tuple[float, float],
) -> tuple[float, float]:
    axis_x, axis_y = axis

    values = tuple(
        point.x_cm * axis_x + point.y_cm * axis_y
        for point in polygon
    )

    return min(values), max(values)


def polygons_intersect(
    first: Sequence[Point],
    second: Sequence[Point],
) -> bool:
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
    arena_size_cm: float,
) -> bool:
    """Return whether every footprint corner lies inside the square arena."""
    if not math.isfinite(arena_size_cm) or arena_size_cm <= 0.0:
        raise ValueError("arena_size_cm must be positive and finite")

    arena_buffer_cm = 5.0
    total_allowance = NUMERIC_TOLERANCE_CM + arena_buffer_cm

    return all(
        -total_allowance
        <= point.x_cm
        <= arena_size_cm + total_allowance
        and -total_allowance
        <= point.y_cm
        <= arena_size_cm + total_allowance
        for point in footprint
    )


@lru_cache(maxsize=128)
def _cached_arena_obstacle_bounds(
    obstacle_cells: tuple[tuple[int, int], ...],
    cell_size_cm: float,
    obstacle_buffer_cm: float,
) -> tuple[tuple[float, ...], ...]:
    """Precompute expanded AABBs once per arena/config combination."""
    result = []
    for x, y in obstacle_cells:
        min_x = x * cell_size_cm - obstacle_buffer_cm
        min_y = y * cell_size_cm - obstacle_buffer_cm
        max_x = (x + 1) * cell_size_cm + obstacle_buffer_cm
        max_y = (y + 1) * cell_size_cm + obstacle_buffer_cm
        result.append((
            min_x, min_y, max_x, max_y,
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (max_x - min_x) * 0.5,
            (max_y - min_y) * 0.5,
        ))
    return tuple(result)


def collision_obstacle_bounds(
    arena: ArenaInput,
    config: PlanningConfig,
) -> tuple[tuple[float, ...], ...]:
    return _cached_arena_obstacle_bounds(
        tuple((obstacle.cell.x, obstacle.cell.y) for obstacle in arena.obstacles),
        config.cell_size_cm,
        config.obstacle_buffer_cm,
    )


def _is_xyh_collision_free(
    x_cm: float,
    y_cm: float,
    heading_rad: float,
    arena: ArenaInput,
    config: PlanningConfig,
    *,
    cosine: float | None = None,
    sine: float | None = None,
    obstacle_bounds: tuple[tuple[float, ...], ...] | None = None,
) -> bool:
    """Fast collision query for a raw continuous pose.

    Keeping this path allocation-free is important because Hybrid A* calls it
    tens of thousands of times per route.
    """
    geometry = config.robot
    half_length = geometry.collision_length_cm * 0.5
    half_width = geometry.collision_width_cm * 0.5

    if cosine is None or sine is None:
        cosine = math.cos(heading_rad)
        sine = math.sin(heading_rad)

    body_x = (
        x_cm
        + geometry.rear_axle_to_body_center_forward_cm * cosine
        - geometry.rear_axle_to_body_center_left_cm * sine
    )
    body_y = (
        y_cm
        + geometry.rear_axle_to_body_center_forward_cm * sine
        + geometry.rear_axle_to_body_center_left_cm * cosine
    )

    extent_x = half_length * abs(cosine) + half_width * abs(sine)
    extent_y = half_length * abs(sine) + half_width * abs(cosine)

    tolerance = NUMERIC_TOLERANCE_CM
    if (
        body_x - extent_x < -tolerance - 10
        or body_x + extent_x > config.arena_size_cm + tolerance + 10
        or body_y - extent_y < -tolerance -10
        or body_y + extent_y > config.arena_size_cm + tolerance + 10
    ):
        return False

    ux, uy = cosine, sine
    vx, vy = -sine, cosine

    bounds_iter = (
        obstacle_bounds
        if obstacle_bounds is not None
        else collision_obstacle_bounds(arena, config)
    )
    for min_x, min_y, max_x, max_y, obstacle_cx, obstacle_cy, obstacle_hx, obstacle_hy in bounds_iter:

        if (
            max_x < body_x - extent_x
            or min_x > body_x + extent_x
            or max_y < body_y - extent_y
            or min_y > body_y + extent_y
        ):
            continue

        dx = obstacle_cx - body_x
        dy = obstacle_cy - body_y

        if abs(dx) > extent_x + obstacle_hx + tolerance:
            continue
        if abs(dy) > extent_y + obstacle_hy + tolerance:
            continue
        if abs(dx * ux + dy * uy) > half_length + obstacle_hx * abs(ux) + obstacle_hy * abs(uy) + tolerance:
            continue
        if abs(dx * vx + dy * vy) > half_width + obstacle_hx * abs(vx) + obstacle_hy * abs(vy) + tolerance:
            continue

        return False

    return True


def is_pose_collision_free(
    pose: Pose,
    arena: ArenaInput,
    config: PlanningConfig,
) -> bool:
    """Authoritative pose-level collision query."""
    return _is_xyh_collision_free(
        pose.x_cm,
        pose.y_cm,
        pose.heading_rad,
        arena,
        config,
    )


@lru_cache(maxsize=512)
def _cached_obstacle_bounds(
    x: int,
    y: int,
    cell_size_cm: float,
    obstacle_buffer_cm: float,
):
    """Return an obstacle rectangle expanded by the obstacle-only buffer."""

    min_x = (
        x * cell_size_cm
        - obstacle_buffer_cm
    )
    min_y = (
        y * cell_size_cm
        - obstacle_buffer_cm
    )
    max_x = (
        (x + 1) * cell_size_cm
        + obstacle_buffer_cm
    )
    max_y = (
        (y + 1) * cell_size_cm
        + obstacle_buffer_cm
    )

    return (
        min_x,
        min_y,
        max_x,
        max_y,
        (
            Point(min_x, min_y),
            Point(max_x, min_y),
            Point(max_x, max_y),
            Point(min_x, max_y),
        ),
    )