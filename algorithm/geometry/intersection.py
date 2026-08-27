"""Reusable continuous segment intersection helpers."""

from __future__ import annotations

from .shapes import AxisAlignedRectangle, NUMERIC_TOLERANCE_CM, Point


def segment_intersects_rectangle(
    start: Point,
    end: Point,
    rectangle: AxisAlignedRectangle,
) -> bool:
    """Return whether a closed line segment touches or enters a rectangle.

    This is the Liang-Barsky clipping test. Contact with a rectangle boundary
    counts as an intersection, matching the collision subsystem's treatment of
    physical contact.
    """
    delta_x = end.x_cm - start.x_cm
    delta_y = end.y_cm - start.y_cm
    entering = 0.0
    leaving = 1.0

    boundaries = (
        (-delta_x, start.x_cm - rectangle.min_x_cm),
        (delta_x, rectangle.max_x_cm - start.x_cm),
        (-delta_y, start.y_cm - rectangle.min_y_cm),
        (delta_y, rectangle.max_y_cm - start.y_cm),
    )
    for direction, distance in boundaries:
        if abs(direction) <= NUMERIC_TOLERANCE_CM:
            if distance < -NUMERIC_TOLERANCE_CM:
                return False
            continue
        ratio = distance / direction
        if direction < 0.0:
            entering = max(entering, ratio)
        else:
            leaving = min(leaving, ratio)
        if entering > leaving + NUMERIC_TOLERANCE_CM:
            return False
    return True
