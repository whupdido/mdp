import math
from dataclasses import FrozenInstanceError, replace

import pytest

from algorithm.config import MotionModel, RobotGeometry, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.coordinates import (
    android_cell_to_planner_pose,
    default_start_pose,
    planner_pose_to_android_cell,
    planner_pose_to_body_center,
)
from algorithm.enums import Direction, Gear, PlanningStatus, Steering
from algorithm.models import (
    ArenaInput,
    GridCell,
    Obstacle,
    PlanningIssue,
    PathMetrics,
    PlanningResult,
    Pose,
    Robot,
    RoutePlan,
    normalize_heading,
)
from algorithm.models.motion import MotionPrimitive


@pytest.mark.parametrize(
    ("direction", "heading", "vector"),
    [
        (Direction.EAST, 0.0, (1, 0)),
        (Direction.NORTH, math.pi / 2.0, (0, 1)),
        (Direction.WEST, -math.pi, (-1, 0)),
        (Direction.SOUTH, -math.pi / 2.0, (0, -1)),
    ],
)
def test_direction_contract(direction, heading, vector):
    assert normalize_heading(direction.heading_rad) == pytest.approx(heading)
    assert direction.grid_vector == vector
    assert direction.opposite().opposite() is direction


@pytest.mark.parametrize("token", ["N", "north", " UP "])
def test_direction_parser_accepts_documented_aliases(token):
    assert Direction.from_token(token) is Direction.NORTH


def test_direction_parser_rejects_unknown_tokens():
    with pytest.raises(ValueError, match="unknown direction"):
        Direction.from_token("Q")


def test_direction_converts_only_cardinal_headings():
    assert Direction.from_heading_rad(2.0 * math.pi) is Direction.EAST
    with pytest.raises(ValueError, match="not cardinal"):
        Direction.from_heading_rad(math.pi / 4.0)


def test_pose_is_normalized_and_immutable():
    pose = Pose(15.0, 15.0, 5.0 * math.pi / 2.0)
    assert pose.heading_rad == pytest.approx(math.pi / 2.0)
    with pytest.raises(FrozenInstanceError):
        pose.x_cm = 20.0


@pytest.mark.parametrize("coordinates", [(-1, 0), (0, -1), (20, 0), (0, 20)])
def test_grid_cell_rejects_out_of_bounds_coordinates(coordinates):
    with pytest.raises(ValueError, match="outside"):
        GridCell(*coordinates)


def test_android_start_cell_maps_to_rear_axle_pose():
    pose = android_cell_to_planner_pose(
        GridCell(1, 1),
        Direction.NORTH,
        UNCALIBRATED_SIMULATION_CONFIG.robot,
    )
    assert pose == Pose(15.0, 15.0, math.pi / 2.0)
    assert default_start_pose(UNCALIBRATED_SIMULATION_CONFIG.robot) == pose


def test_coordinate_transform_honours_body_center_offset():
    geometry = RobotGeometry(
        length_cm=23.0,
        width_cm=18.8,
        rear_axle_to_body_center_forward_cm=4.0,
        rear_axle_to_body_center_left_cm=2.0,
    )
    rear_axle = android_cell_to_planner_pose(GridCell(1, 1), Direction.NORTH, geometry)
    assert rear_axle.x_cm == pytest.approx(17.0)
    assert rear_axle.y_cm == pytest.approx(11.0)
    assert planner_pose_to_body_center(rear_axle, geometry) == Pose(15.0, 15.0, math.pi / 2.0)
    assert planner_pose_to_android_cell(rear_axle, geometry) == GridCell(1, 1)


def test_robot_state_does_not_conflate_pose_and_geometry():
    robot = Robot(Pose(15.0, 15.0, Direction.NORTH.heading_rad))
    assert robot.pose.x_cm == 15.0
    assert not hasattr(robot, "length_cm")


def test_arena_freezes_input_and_rejects_duplicate_obstacles():
    first = Obstacle(1, GridCell(4, 4), Direction.NORTH)
    arena = ArenaInput(Pose(15.0, 15.0, Direction.NORTH.heading_rad), [first])
    assert arena.obstacles == (first,)

    with pytest.raises(ValueError, match="IDs"):
        ArenaInput(arena.start_pose, (first, Obstacle(1, GridCell(5, 5), Direction.SOUTH)))
    with pytest.raises(ValueError, match="same cell"):
        ArenaInput(arena.start_pose, (first, Obstacle(2, GridCell(4, 4), Direction.SOUTH)))


def test_missing_face_is_a_structured_task1_issue():
    obstacle = Obstacle(3, GridCell(8, 8))
    arena = ArenaInput(Pose(15.0, 15.0, Direction.NORTH.heading_rad), (obstacle,))
    assert arena.task1_issues() == (
        PlanningIssue(
            code="missing_image_face",
            message="obstacle 3 has no image face",
            obstacle_id=3,
        ),
    )


def test_image_id_uses_android_contract_range():
    Obstacle(1, GridCell(4, 4), Direction.NORTH, image_id=11)
    Obstacle(2, GridCell(5, 5), Direction.NORTH, image_id=40)
    with pytest.raises(ValueError, match="between 11 and 40"):
        Obstacle(3, GridCell(6, 6), Direction.NORTH, image_id=41)


def test_simulation_profile_is_explicit_and_command_aligned():
    config = UNCALIBRATED_SIMULATION_CONFIG
    assert config.physically_calibrated is False
    assert config.robot.collision_length_cm == pytest.approx(33.0)
    assert config.robot.collision_width_cm == pytest.approx(28.8)
    assert config.observation_lateral_offsets_cm == (0.0, -10.0, 10.0)
    assert tuple(item.command for item in config.motion.primitives) == (
        "FW", "BW", "FL", "FR", "BL", "BR"
    )
    assert config.motion.primitives_for("fl")[0].turn_angle_rad == pytest.approx(math.pi / 2.0)
    assert config.motion.primitives_for("BL")[0].turn_angle_rad == pytest.approx(-math.pi / 2.0)
    assert config.motion.primitives_for("BR")[0].turn_angle_rad == pytest.approx(math.pi / 2.0)


def test_motion_model_accepts_multiple_configurable_angles_for_one_command():
    base = UNCALIBRATED_SIMULATION_CONFIG.motion
    forty_five = MotionPrimitive(
        "FL",
        Gear.FORWARD,
        Steering.LEFT,
        turn_angle_rad=math.pi / 4.0,
        radius_cm=26.1,
    )
    model = MotionModel(
        primitives=base.primitives + (forty_five,),
        straight_speed_cm_s=base.straight_speed_cm_s,
    )
    assert len(model.primitives_for("FL")) == 2


def test_primitive_validation_catches_command_semantic_mismatch():
    with pytest.raises(ValueError, match="conflicts"):
        MotionPrimitive("FR", Gear.FORWARD, Steering.LEFT, turn_angle_rad=1.0, radius_cm=20.0)

    with pytest.raises(ValueError, match="angle sign"):
        MotionPrimitive("BL", Gear.REVERSE, Steering.LEFT, turn_angle_rad=1.0, radius_cm=20.0)


def test_planning_failure_requires_a_structured_issue():
    with pytest.raises(ValueError, match="at least one issue"):
        PlanningResult(status=PlanningStatus.INVALID_INPUT)
    result = PlanningResult(
        status=PlanningStatus.INVALID_INPUT,
        issues=(PlanningIssue("invalid", "invalid arena"),),
    )
    assert result.route is None


def test_successful_planning_result_requires_a_consistent_route():
    start = default_start_pose(UNCALIBRATED_SIMULATION_CONFIG.robot)
    route = RoutePlan(
        start=start,
        target_order=(),
        observation_poses=(),
        local_paths=(),
        execution_steps=(),
        metrics=PathMetrics(),
    )
    result = PlanningResult(status=PlanningStatus.SUCCESS, route=route)
    assert result.route is route


def test_configuration_can_replace_the_command_aligned_primitive_set():
    fine_arc = MotionPrimitive(
        "FL",
        Gear.FORWARD,
        Steering.LEFT,
        turn_angle_rad=math.pi / 4.0,
        radius_cm=26.1,
    )
    new_motion = replace(
        UNCALIBRATED_SIMULATION_CONFIG.motion,
        primitives=(fine_arc,),
    )
    config = replace(UNCALIBRATED_SIMULATION_CONFIG, motion=new_motion)
    assert config.motion.primitives == (fine_arc,)
    assert config.heading_bin_rad == pytest.approx(math.radians(15.0))
