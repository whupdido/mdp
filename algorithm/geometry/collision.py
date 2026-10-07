"""Authoritative pose-level arena and obstacle collision checks."""

from __future__ import annotations

import math
from collections.abc import Sequence
from functools import lru_cache

from algorithm.config import PlanningConfig
from algorithm.models.arena import ArenaInput
from algorithm.models.pose import Pose

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
    """Return whether two convex polygons overlap or touch using SAT."""
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

    return all(
        -NUMERIC_TOLERANCE_CM <= point.x_cm <= arena_size_cm + NUMERIC_TOLERANCE_CM
        and -NUMERIC_TOLERANCE_CM <= point.y_cm <= arena_size_cm + NUMERIC_TOLERANCE_CM
        for point in footprint
    )


# Localized bindings to bypass Python's global namespace dictionary lookups
_min = min
_max = max
_abs = abs
_hypot = math.hypot

def is_pose_collision_free(
    pose: Pose,
    arena: ArenaInput,
    config: PlanningConfig,
) -> bool:
    """Authoritative collision query for a robot pose in an arena."""

    footprint = robot_footprint(pose, config.robot)

    # 1. Fast list comprehension extraction (avoids generator allocations)
    arena_size = config.arena_size_cm
    tol = NUMERIC_TOLERANCE_CM
    
    fp_x = [p.x_cm for p in footprint]
    fp_y = [p.y_cm for p in footprint]
    num_points = len(fp_x)

    # 2. Inlined arena boundary check
    for i in range(num_points):
        x, y = fp_x[i], fp_y[i]
        if x < -tol or x > arena_size + tol or y < -tol or y > arena_size + tol:
            return False

    fp_min_x = _min(fp_x)
    fp_max_x = _max(fp_x)
    fp_min_y = _min(fp_y)
    fp_max_y = _max(fp_y)

    # 3. Lazy SAT Context
    # We only compute the footprint's normals and projections if the robot
    # is mathematically guaranteed to be overlapping an obstacle's AABB.
    axes_computed = False
    fp_axes_x = []
    fp_axes_y = []
    fp_mins = []
    fp_maxs = []

    for obstacle in arena.obstacles:
        bounds = _cached_obstacle_bounds(
            obstacle.cell.x,
            obstacle.cell.y,
            config.cell_size_cm,
            config.obstacle_buffer_cm,
        )

        o_min_x, o_min_y, o_max_x, o_max_y, _ = bounds

        # 4. Fast AABB rejection 
        if (
            o_max_x < fp_min_x - tol
            or o_min_x > fp_max_x + tol
            or o_max_y < fp_min_y - tol
            or o_min_y > fp_max_y + tol
        ):
            continue

        # 5. Generate Footprint Normals (Only runs once per pose, and only if near an obstacle)
        if not axes_computed:
            for i in range(num_points):
                nxt = (i + 1) % num_points
                edge_x = fp_x[nxt] - fp_x[i]
                edge_y = fp_y[nxt] - fp_y[i]

                if _abs(edge_x) <= tol and _abs(edge_y) <= tol:
                    continue

                length = _hypot(edge_x, edge_y)
                ax_x = -edge_y / length
                ax_y = edge_x / length
                
                fp_axes_x.append(ax_x)
                fp_axes_y.append(ax_y)
                
                projs = [fp_x[j] * ax_x + fp_y[j] * ax_y for j in range(num_points)]
                fp_mins.append(_min(projs))
                fp_maxs.append(_max(projs))
            
            axes_computed = True

        # 6. Specialized SAT Test
        # We skip evaluating the obstacle's axes because the AABB check already did it.
        # We only project the 4 AABB obstacle corners onto the robot footprint's axes.
        op_x = (o_min_x, o_max_x, o_max_x, o_min_x)
        op_y = (o_min_y, o_min_y, o_max_y, o_max_y)

        collision = True
        for ax_x, ax_y, f_min, f_max in zip(fp_axes_x, fp_axes_y, fp_mins, fp_maxs):
            o_projs = [x * ax_x + y * ax_y for x, y in zip(op_x, op_y)]
            
            if f_max < _min(o_projs) - tol or _max(o_projs) < f_min - tol:
                collision = False
                break
                
        if collision:
            return False

    return True


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