"""Task 1 image observation-pose generation and validation."""

from .generator import generate_arena_observation_candidates, generate_observation_candidates
from .geometry import (
    camera_world_position,
    desired_camera_position,
    has_clear_line_of_sight,
    image_face_target_point,
    rear_axle_pose_for_camera,
)
from .models import (
    ObservationCandidate,
    ObservationCandidateGroup,
    ObservationCandidateKind,
    ObservationLateralClass,
)

__all__ = [
    "ObservationCandidate",
    "ObservationCandidateGroup",
    "ObservationCandidateKind",
    "ObservationLateralClass",
    "camera_world_position",
    "desired_camera_position",
    "generate_arena_observation_candidates",
    "generate_observation_candidates",
    "has_clear_line_of_sight",
    "image_face_target_point",
    "rear_axle_pose_for_camera",
]
