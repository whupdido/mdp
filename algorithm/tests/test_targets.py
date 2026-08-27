from dataclasses import replace

import pytest

from algorithm.config import CameraGeometry, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import Direction
from algorithm.models import ArenaInput, GridCell, Obstacle, Pose
from algorithm.targets import (
    ObservationCandidateKind,
    camera_world_position,
    generate_arena_observation_candidates,
    generate_observation_candidates,
    image_face_target_point,
)


CONFIG = UNCALIBRATED_SIMULATION_CONFIG


def target(obstacle_id: int = 1, cell: tuple[int, int] = (10, 10), face=Direction.NORTH):
    return Obstacle(obstacle_id, GridCell(*cell), face)


def arena_with(*obstacles: Obstacle) -> ArenaInput:
    return ArenaInput(Pose(15.0, 15.0, Direction.NORTH.heading_rad), obstacles)


def group_for(obstacle: Obstacle, *others: Obstacle, config=CONFIG):
    arena = arena_with(obstacle, *others)
    return generate_observation_candidates(obstacle, arena, config)


@pytest.mark.parametrize(
    ("face", "expected_heading"),
    [
        (Direction.NORTH, Direction.SOUTH),
        (Direction.SOUTH, Direction.NORTH),
        (Direction.EAST, Direction.WEST),
        (Direction.WEST, Direction.EAST),
    ],
)
def test_image_face_produces_robot_heading_toward_obstacle(face, expected_heading):
    candidate = group_for(target(face=face)).candidates[0]
    assert Direction.from_heading_rad(candidate.observation_pose.pose.heading_rad) is expected_heading
    assert candidate.face is face


@pytest.mark.parametrize(
    ("face", "expected"),
    [
        (Direction.NORTH, (105.0, 110.0)),
        (Direction.EAST, (110.0, 105.0)),
        (Direction.SOUTH, (105.0, 100.0)),
        (Direction.WEST, (100.0, 105.0)),
    ],
)
def test_image_target_is_center_of_annotated_face(face, expected):
    point = image_face_target_point(target(face=face), face, CONFIG.cell_size_cm)
    assert (point.x_cm, point.y_cm) == expected


def test_nominal_camera_and_rear_axle_positions_use_camera_transform():
    candidate = group_for(target()).candidates[0]
    assert (candidate.target_point.x_cm, candidate.target_point.y_cm) == (105.0, 110.0)
    assert (candidate.camera_position.x_cm, candidate.camera_position.y_cm) == (105.0, 130.0)
    assert candidate.observation_pose.pose.x_cm == pytest.approx(105.0)
    assert candidate.observation_pose.pose.y_cm == pytest.approx(141.5)
    assert camera_world_position(candidate.observation_pose.pose, CONFIG.camera) == candidate.camera_position


def test_camera_offset_changes_rear_axle_without_changing_desired_camera_position():
    shifted_camera = CameraGeometry(
        forward_offset_cm=20.0,
        left_offset_cm=3.0,
        image_gap_cm=CONFIG.camera.image_gap_cm,
    )
    shifted_config = replace(CONFIG, camera=shifted_camera)
    baseline = group_for(target()).candidates[0]
    shifted = group_for(target(), config=shifted_config).candidates[0]
    assert shifted.camera_position == baseline.camera_position
    assert shifted.observation_pose.pose.x_cm == pytest.approx(102.0)
    assert shifted.observation_pose.pose.y_cm == pytest.approx(150.0)


def test_configured_standoff_changes_camera_and_rear_axle_position():
    farther = replace(CONFIG, camera=replace(CONFIG.camera, image_gap_cm=30.0))
    baseline = group_for(target()).candidates[0]
    shifted = group_for(target(), config=farther).candidates[0]
    assert shifted.camera_position.y_cm - baseline.camera_position.y_cm == pytest.approx(10.0)
    assert shifted.observation_pose.pose.y_cm - baseline.observation_pose.pose.y_cm == pytest.approx(10.0)


def test_nominal_left_right_candidates_are_ordered_and_configured():
    candidates = group_for(target()).candidates
    assert tuple(candidate.kind for candidate in candidates) == (
        ObservationCandidateKind.NOMINAL,
        ObservationCandidateKind.LEFT,
        ObservationCandidateKind.RIGHT,
    )
    assert tuple(candidate.observation_pose.candidate_index for candidate in candidates) == (0, 1, 2)
    assert tuple(candidate.observation_pose.nominal for candidate in candidates) == (True, False, False)
    assert tuple(candidate.lateral_offset_cm for candidate in candidates) == (0.0, -10.0, 10.0)
    # A south-facing robot's local left is East, so the left fallback has
    # greater x than nominal and the right fallback has smaller x.
    assert tuple(candidate.camera_position.x_cm for candidate in candidates) == (105.0, 115.0, 95.0)


def test_candidate_identity_remains_associated_with_obstacle_and_face():
    obstacle = target(obstacle_id=7, face=Direction.WEST)
    group = group_for(obstacle)
    assert group.obstacle_id == 7
    assert group.face is Direction.WEST
    assert all(candidate.observation_pose.obstacle_id == 7 for candidate in group.candidates)
    assert all(candidate.face is Direction.WEST for candidate in group.candidates)


def test_candidate_rejected_when_footprint_crosses_boundary():
    group = group_for(target(cell=(10, 19), face=Direction.NORTH))
    assert not group.has_valid_candidate
    assert all("pose_collision" in candidate.rejection_reasons for candidate in group.candidates)


def test_candidate_rejected_when_robot_footprint_hits_other_obstacle():
    blocker = target(2, cell=(10, 14), face=Direction.SOUTH)
    nominal = group_for(target(), blocker).candidates[0]
    assert not nominal.collision_free
    assert "pose_collision" in nominal.rejection_reasons


def test_candidate_can_be_rejected_only_by_configured_safety_margin():
    blocker = target(2, cell=(8, 14), face=Direction.SOUTH)
    no_margin = replace(CONFIG, robot=replace(CONFIG.robot, safety_margin_cm=0.0))
    expanded = replace(CONFIG, robot=replace(CONFIG.robot, safety_margin_cm=6.0))
    assert group_for(target(), blocker, config=no_margin).candidates[0].collision_free
    assert not group_for(target(), blocker, config=expanded).candidates[0].collision_free


def test_blocked_nominal_keeps_valid_fallback():
    blocker = target(2, cell=(11, 14), face=Direction.SOUTH)
    group = group_for(target(), blocker)
    assert not group.candidates[0].valid
    assert group.candidates[2].valid
    assert group.has_valid_candidate
    assert tuple(candidate.candidate_index for candidate in group.valid_candidates) == (2,)


def test_all_blocked_candidates_produce_structured_issue():
    blockers = (
        target(2, cell=(9, 14), face=Direction.SOUTH),
        target(3, cell=(10, 14), face=Direction.SOUTH),
        target(4, cell=(11, 14), face=Direction.SOUTH),
    )
    group = group_for(target(), *blockers)
    assert not group.has_valid_candidate
    assert group.valid_candidates == ()
    assert group.issues[0].code == "no_geometrically_valid_observation_pose"


def test_clear_camera_line_of_sight_and_target_obstacle_endpoint_are_accepted():
    nominal = group_for(target()).candidates[0]
    assert nominal.line_of_sight_clear
    assert nominal.valid


def test_other_obstacle_blocking_viewing_segment_rejects_candidate():
    # This obstacle lies on the camera ray but remains 5 cm clear of the
    # safety-expanded robot footprint.
    blocker = target(2, cell=(10, 11), face=Direction.SOUTH)
    nominal = group_for(target(), blocker).candidates[0]
    assert nominal.collision_free
    assert not nominal.line_of_sight_clear
    assert nominal.rejection_reasons == ("line_of_sight_blocked",)


def test_missing_face_uses_existing_structured_task1_issue():
    obstacle = target(face=None)
    group = group_for(obstacle)
    assert group.face is None
    assert group.candidates == ()
    assert group.issues[0].code == "missing_image_face"
    assert group.issues == arena_with(obstacle).task1_issues()


def test_multiple_obstacles_retain_deterministic_candidate_grouping():
    obstacles = (
        target(3, (6, 6), Direction.NORTH),
        target(1, (10, 10), Direction.EAST),
        target(8, (14, 14), Direction.SOUTH),
    )
    groups = generate_arena_observation_candidates(arena_with(*obstacles), CONFIG)
    assert tuple(group.obstacle_id for group in groups) == (3, 1, 8)
    assert all(len(group.candidates) == 3 for group in groups)
    assert all(
        candidate.observation_pose.obstacle_id == group.obstacle_id
        for group in groups
        for candidate in group.candidates
    )


def test_zero_targets_returns_no_groups():
    assert generate_arena_observation_candidates(arena_with(), CONFIG) == ()


def test_eight_targets_with_three_candidates_each_are_supported():
    obstacles = tuple(
        target(index + 1, (index + 5, 10), Direction.NORTH)
        for index in range(8)
    )
    groups = generate_arena_observation_candidates(arena_with(*obstacles), CONFIG)
    assert len(groups) == CONFIG.guaranteed_max_targets == 8
    assert all(len(group.candidates) == CONFIG.guaranteed_max_candidates_per_target == 3 for group in groups)


def test_candidate_generation_is_deterministic():
    obstacle = target(4, (12, 8), Direction.EAST)
    arena = arena_with(obstacle)
    first = generate_observation_candidates(obstacle, arena, CONFIG)
    second = generate_observation_candidates(obstacle, arena, CONFIG)
    assert first == second
