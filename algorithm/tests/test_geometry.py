import math
from dataclasses import replace

import pytest

from algorithm.config import RobotGeometry, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import Direction
from algorithm.geometry import (
    is_motion_collision_free,
    is_pose_collision_free,
    obstacle_bounds,
    propagate_motion,
    robot_footprint,
    sample_arc,
    sample_motion,
    sample_straight,
)
from algorithm.models import ArenaInput, GridCell, Obstacle, Pose
from algorithm.models.motion import MotionPrimitive
from algorithm.enums import Gear, Steering


CONFIG = UNCALIBRATED_SIMULATION_CONFIG
EMPTY_ARENA = ArenaInput(Pose(100.0, 100.0, 0.0))


def arena_with(*obstacles: Obstacle, start: Pose | None = None) -> ArenaInput:
    return ArenaInput(start or Pose(100.0, 100.0, 0.0), obstacles)


def primitive(command: str) -> MotionPrimitive:
    return CONFIG.motion.primitives_for(command)[0]


def without_safety_margin():
    return replace(CONFIG, robot=replace(CONFIG.robot, safety_margin_cm=0.0))


def test_robot_footprint_uses_body_offset_and_one_safety_expansion():
    geometry = RobotGeometry(
        length_cm=20.0,
        width_cm=10.0,
        rear_axle_to_body_center_forward_cm=5.0,
        rear_axle_to_body_center_left_cm=2.0,
        safety_margin_cm=3.0,
    )
    footprint = robot_footprint(Pose(50.0, 50.0, 0.0), geometry)
    assert tuple((point.x_cm, point.y_cm) for point in footprint) == (
        (68.0, 60.0),
        (42.0, 60.0),
        (42.0, 44.0),
        (68.0, 44.0),
    )


def test_obstacle_bounds_use_the_configured_cell_size():
    bounds = obstacle_bounds(Obstacle(1, GridCell(3, 7)), cell_size_cm=12.5)
    assert (bounds.min_x_cm, bounds.min_y_cm) == (37.5, 87.5)
    assert (bounds.max_x_cm, bounds.max_y_cm) == (50.0, 100.0)


def test_robot_safely_inside_empty_arena():
    assert is_pose_collision_free(Pose(100.0, 100.0, math.pi / 4.0), EMPTY_ARENA, CONFIG)


@pytest.mark.parametrize(
    "pose",
    [
        Pose(100.0, 190.0, Direction.NORTH.heading_rad),
        Pose(100.0, 10.0, Direction.NORTH.heading_rad),
        Pose(190.0, 100.0, Direction.EAST.heading_rad),
        Pose(10.0, 100.0, Direction.EAST.heading_rad),
    ],
    ids=["north", "south", "east", "west"],
)
def test_entire_footprint_must_remain_inside_arena(pose):
    assert not is_pose_collision_free(pose, EMPTY_ARENA, CONFIG)


def test_boundary_validity_depends_on_orientation():
    position = (15.0, 100.0)
    assert is_pose_collision_free(Pose(*position, Direction.NORTH.heading_rad), EMPTY_ARENA, CONFIG)
    assert not is_pose_collision_free(Pose(*position, Direction.EAST.heading_rad), EMPTY_ARENA, CONFIG)


def test_robot_overlapping_obstacle_is_rejected():
    arena = arena_with(Obstacle(1, GridCell(10, 10), Direction.NORTH))
    assert not is_pose_collision_free(Pose(105.0, 105.0, 0.0), arena, CONFIG)


def test_robot_immediately_beside_obstacle_is_valid_when_separated():
    arena = arena_with(Obstacle(1, GridCell(10, 10), Direction.NORTH))
    assert is_pose_collision_free(Pose(126.6, 105.0, 0.0), arena, CONFIG)


def test_physical_contact_with_obstacle_counts_as_collision():
    config = without_safety_margin()
    arena = arena_with(Obstacle(1, GridCell(10, 10), Direction.NORTH))
    # The robot's rear edge is exactly on the obstacle's east edge at x=110.
    assert not is_pose_collision_free(Pose(121.5, 105.0, 0.0), arena, config)


def test_collision_can_be_caused_only_by_safety_margin():
    arena = arena_with(Obstacle(1, GridCell(10, 10), Direction.NORTH))
    pose = Pose(122.0, 105.0, 0.0)
    assert is_pose_collision_free(pose, arena, without_safety_margin())
    assert not is_pose_collision_free(pose, arena, CONFIG)


def test_straight_motion_samples_start_and_exact_endpoint():
    start = Pose(80.0, 80.0, Direction.EAST.heading_rad)
    samples = sample_straight(start, primitive("FW"), maximum_step_cm=3.0)
    assert len(samples) == 5
    assert samples[0] == start
    assert samples[-1] == Pose(90.0, 80.0, Direction.EAST.heading_rad)
    assert max(
        math.dist((first.x_cm, first.y_cm), (second.x_cm, second.y_cm))
        for first, second in zip(samples, samples[1:])
    ) <= 3.0


def test_clear_forward_and_reverse_straight_motion():
    start = Pose(100.0, 100.0, Direction.EAST.heading_rad)
    assert is_motion_collision_free(start, primitive("FW"), EMPTY_ARENA, CONFIG)
    assert is_motion_collision_free(start, primitive("BW"), EMPTY_ARENA, CONFIG)
    assert propagate_motion(start, primitive("BW"), CONFIG) == Pose(90.0, 100.0, 0.0)


def test_straight_endpoints_can_be_valid_while_middle_collides():
    long_forward = MotionPrimitive("FW", Gear.FORWARD, Steering.STRAIGHT, travel_cm=100.0)
    start = Pose(50.0, 105.0, Direction.EAST.heading_rad)
    arena = arena_with(Obstacle(1, GridCell(10, 10), Direction.NORTH), start=start)
    end = propagate_motion(start, long_forward, CONFIG)
    assert is_pose_collision_free(start, arena, CONFIG)
    assert is_pose_collision_free(end, arena, CONFIG)
    assert not is_motion_collision_free(start, long_forward, arena, CONFIG)


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ("FL", Pose(126.1, 126.1, Direction.NORTH.heading_rad)),
        ("FR", Pose(131.8, 68.2, Direction.SOUTH.heading_rad)),
        ("BL", Pose(75.4, 124.6, Direction.SOUTH.heading_rad)),
        ("BR", Pose(69.7, 69.7, Direction.NORTH.heading_rad)),
    ],
)
def test_configured_arc_endpoint_and_reverse_assumptions(command, expected):
    start = Pose(100.0, 100.0, Direction.EAST.heading_rad)
    actual = propagate_motion(start, primitive(command), CONFIG)
    assert actual.x_cm == pytest.approx(expected.x_cm)
    assert actual.y_cm == pytest.approx(expected.y_cm)
    assert actual.heading_rad == pytest.approx(expected.heading_rad)
    assert is_motion_collision_free(start, primitive(command), EMPTY_ARENA, CONFIG)


def test_arc_samples_respect_configured_angular_resolution():
    turn = primitive("FL")
    maximum_step = math.radians(20.0)
    samples = sample_arc(Pose(100.0, 100.0, 0.0), turn, maximum_step)
    assert len(samples) == math.ceil(abs(turn.turn_angle_rad) / maximum_step) + 1
    assert max(
        abs(second.heading_rad - first.heading_rad)
        for first, second in zip(samples, samples[1:])
    ) <= maximum_step


def test_arc_endpoints_can_be_valid_while_swept_footprint_collides():
    start = Pose(60.0, 60.0, Direction.EAST.heading_rad)
    turn = primitive("FL")
    arena = arena_with(Obstacle(1, GridCell(8, 5), Direction.NORTH), start=start)
    end = propagate_motion(start, turn, CONFIG)
    assert is_pose_collision_free(start, arena, CONFIG)
    assert is_pose_collision_free(end, arena, CONFIG)
    assert not is_motion_collision_free(start, turn, arena, CONFIG)


def test_sample_motion_reads_translation_and_angular_steps_from_config():
    coarse = replace(
        CONFIG,
        collision_translation_step_cm=4.0,
        collision_arc_step_rad=math.radians(30.0),
    )
    assert len(sample_motion(Pose(100.0, 100.0, 0.0), primitive("FW"), coarse)) == 4
    assert len(sample_motion(Pose(100.0, 100.0, 0.0), primitive("FL"), coarse)) == 4


def test_geometry_sampling_is_deterministic():
    start = Pose(100.0, 100.0, math.pi / 3.0)
    first = sample_motion(start, primitive("BR"), CONFIG)
    second = sample_motion(start, primitive("BR"), CONFIG)
    assert first == second
    assert is_motion_collision_free(start, primitive("BR"), EMPTY_ARENA, CONFIG) == (
        is_motion_collision_free(start, primitive("BR"), EMPTY_ARENA, CONFIG)
    )
