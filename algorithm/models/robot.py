"""Robot state model.

Physical dimensions live in :class:`algorithm.config.RobotGeometry`; this
type only carries the robot's current rear-axle pose.
"""

from dataclasses import dataclass

from algorithm.models.pose import Pose


@dataclass(frozen=True, slots=True)
class Robot:
    pose: Pose


RobotState = Robot
