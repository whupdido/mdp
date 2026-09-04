"""Shared enums used by the planning contracts and wire adapters."""

from __future__ import annotations

import math
from enum import Enum


class Direction(Enum):
    NORTH = "N"
    EAST = "E"
    SOUTH = "S"
    WEST = "W"

    @property
    def heading_rad(self) -> float:
        """Return the mathematical heading: East is zero, CCW is positive."""
        return {
            Direction.EAST: 0.0,
            Direction.NORTH: math.pi / 2.0,
            Direction.WEST: math.pi,
            Direction.SOUTH: -math.pi / 2.0,
        }[self]

    @property
    def grid_vector(self) -> tuple[int, int]:
        return {
            Direction.NORTH: (0, 1),
            Direction.EAST: (1, 0),
            Direction.SOUTH: (0, -1),
            Direction.WEST: (-1, 0),
        }[self]

    def opposite(self) -> Direction:
        return {
            Direction.NORTH: Direction.SOUTH,
            Direction.EAST: Direction.WEST,
            Direction.SOUTH: Direction.NORTH,
            Direction.WEST: Direction.EAST,
        }[self]

    @classmethod
    def from_token(cls, token: str) -> Direction:
        normalized = token.strip().upper()
        aliases = {
            "N": cls.NORTH,
            "NORTH": cls.NORTH,
            "UP": cls.NORTH,
            "E": cls.EAST,
            "EAST": cls.EAST,
            "RIGHT": cls.EAST,
            "S": cls.SOUTH,
            "SOUTH": cls.SOUTH,
            "DOWN": cls.SOUTH,
            "W": cls.WEST,
            "WEST": cls.WEST,
            "LEFT": cls.WEST,
        }
        try:
            return aliases[normalized]
        except KeyError as exc:
            raise ValueError(f"unknown direction: {token!r}") from exc

    @classmethod
    def from_heading_rad(cls, heading_rad: float, *, tolerance_rad: float = 1e-6) -> Direction:
        """Convert a cardinal mathematical heading to its wire direction."""
        if not math.isfinite(heading_rad):
            raise ValueError("heading_rad must be finite")
        if not math.isfinite(tolerance_rad) or tolerance_rad < 0.0:
            raise ValueError("tolerance_rad must be finite and non-negative")

        def angular_distance(first: float, second: float) -> float:
            return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)

        closest = min(cls, key=lambda direction: angular_distance(heading_rad, direction.heading_rad))
        if angular_distance(heading_rad, closest.heading_rad) > tolerance_rad:
            raise ValueError(f"heading is not cardinal within {tolerance_rad} radians")
        return closest


class Gear(Enum):
    FORWARD = "forward"
    REVERSE = "reverse"


class Steering(Enum):
    STRAIGHT = "straight"
    LEFT = "left"
    RIGHT = "right"


class CostMetric(Enum):
    DISTANCE = "distance"
    ESTIMATED_TIME = "estimated_time"


class RoutingMode(Enum):
    FEASIBILITY = "feasibility"
    FULL_OPTIMIZATION = "full_optimization"


class PlanningStatus(Enum):
    SUCCESS = "success"
    INVALID_INPUT = "invalid_input"
    NO_FEASIBLE_ROUTE = "no_feasible_route"
    PLANNING_TIMEOUT = "planning_timeout"
    SEARCH_LIMIT_REACHED = "search_limit_reached"
    INCONCLUSIVE = "inconclusive"
