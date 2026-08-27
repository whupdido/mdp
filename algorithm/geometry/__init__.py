"""Reusable Pygame-independent geometry and collision subsystem."""

from .collision import footprint_within_arena, is_pose_collision_free, polygons_intersect
from .footprint import obstacle_bounds, robot_footprint
from .intersection import segment_intersects_rectangle
from .motion import (
    is_motion_collision_free,
    propagate_motion,
    sample_arc,
    sample_motion,
    sample_straight,
)
from .shapes import AxisAlignedRectangle, Point

__all__ = [
    "AxisAlignedRectangle",
    "Point",
    "footprint_within_arena",
    "is_motion_collision_free",
    "is_pose_collision_free",
    "obstacle_bounds",
    "polygons_intersect",
    "propagate_motion",
    "robot_footprint",
    "sample_arc",
    "sample_motion",
    "sample_straight",
    "segment_intersects_rectangle",
]
