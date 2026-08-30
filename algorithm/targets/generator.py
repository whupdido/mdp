"""Ordered generation and geometric validation of Task 1 observation poses."""

from __future__ import annotations

from algorithm.config import PlanningConfig
from algorithm.geometry import is_pose_collision_free
from algorithm.models.arena import ArenaInput
from algorithm.models.obstacle import Obstacle
from algorithm.models.planning import ObservationPose, PlanningIssue

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


def _lateral_class(offset_cm: float, lateral_index: int) -> ObservationLateralClass:
    if offset_cm == 0.0:
        return ObservationLateralClass.CENTER
    if lateral_index == 1:
        return ObservationLateralClass.LEFT
    if lateral_index == 2:
        return ObservationLateralClass.RIGHT
    return ObservationLateralClass.OFFSET


def _candidate_kind(distance_index: int, lateral: ObservationLateralClass) -> ObservationCandidateKind:
    if distance_index:
        return ObservationCandidateKind.ALTERNATIVE
    return {
        ObservationLateralClass.CENTER: ObservationCandidateKind.NOMINAL,
        ObservationLateralClass.LEFT: ObservationCandidateKind.LEFT,
        ObservationLateralClass.RIGHT: ObservationCandidateKind.RIGHT,
    }.get(lateral, ObservationCandidateKind.ALTERNATIVE)


def _missing_face_issue(obstacle: Obstacle, arena: ArenaInput) -> PlanningIssue:
    for issue in arena.task1_issues():
        if issue.obstacle_id == obstacle.obstacle_id:
            return issue
    raise ValueError("obstacle has no face but ArenaInput reported no Task 1 issue")


def generate_observation_candidates(
    obstacle: Obstacle,
    arena: ArenaInput,
    config: PlanningConfig,
) -> ObservationCandidateGroup:
    """Generate ordered geometric candidates for one obstacle.

    Candidate validity does not imply that Hybrid A* can reach the pose.
    """
    if obstacle.face is None:
        return ObservationCandidateGroup(
            obstacle_id=obstacle.obstacle_id,
            face=None,
            issues=(_missing_face_issue(obstacle, arena),),
        )

    face = obstacle.face
    heading = face.opposite()
    target_point = image_face_target_point(obstacle, face, config.cell_size_cm)
    parameter_pairs = (
        (distance_index, standoff_cm, lateral_index, lateral_offset_cm)
        for distance_index, standoff_cm in enumerate(config.observation_standoff_distances_cm)
        for lateral_index, lateral_offset_cm in enumerate(config.observation_lateral_offsets_cm)
    )
    candidates: list[ObservationCandidate] = []

    for index, (distance_index, standoff_cm, lateral_index, lateral_offset_cm) in enumerate(
        tuple(parameter_pairs)[: config.guaranteed_max_candidates_per_target]
    ):
        lateral = _lateral_class(lateral_offset_cm, lateral_index)
        desired_camera = desired_camera_position(
            obstacle, face, lateral_offset_cm, config, standoff_cm=standoff_cm
        )
        pose = rear_axle_pose_for_camera(desired_camera, heading, config.camera)
        actual_camera = camera_world_position(pose, config.camera)
        collision_free = is_pose_collision_free(pose, arena, config)
        line_of_sight_clear = has_clear_line_of_sight(
            actual_camera,
            target_point,
            obstacle.obstacle_id,
            arena,
            config,
        )
        candidates.append(
            ObservationCandidate(
                observation_pose=ObservationPose(
                    obstacle_id=obstacle.obstacle_id,
                    candidate_index=index,
                    pose=pose,
                    nominal=index == 0,
                ),
                face=face,
                kind=_candidate_kind(distance_index, lateral),
                lateral_offset_cm=lateral_offset_cm,
                standoff_cm=standoff_cm,
                lateral_class=lateral,
                preference_rank=index,
                camera_position=actual_camera,
                target_point=target_point,
                collision_free=collision_free,
                line_of_sight_clear=line_of_sight_clear,
            )
        )

    issues: tuple[PlanningIssue, ...] = ()
    if not any(candidate.valid for candidate in candidates):
        issues = (
            PlanningIssue(
                code="no_geometrically_valid_observation_pose",
                message=f"obstacle {obstacle.obstacle_id} has no geometrically valid observation pose",
                obstacle_id=obstacle.obstacle_id,
            ),
        )
    return ObservationCandidateGroup(
        obstacle_id=obstacle.obstacle_id,
        face=face,
        candidates=tuple(candidates),
        issues=issues,
    )


def generate_arena_observation_candidates(
    arena: ArenaInput,
    config: PlanningConfig,
) -> tuple[ObservationCandidateGroup, ...]:
    """Generate grouped candidates in deterministic arena obstacle order."""
    return tuple(
        generate_observation_candidates(obstacle, arena, config)
        for obstacle in arena.obstacles
    )
