from dataclasses import FrozenInstanceError

import pytest

from algorithm.config import task1_robot_config
from algorithm.enums import Direction, PlanningStatus, RoutingMode
from algorithm.models import GridCell, Obstacle, Pose
from algorithm.serialization import serialize_planning_result
from algorithm.simulator.profiles import CandidatePolicy, candidate_profile, turn_profile
from algorithm.simulator.scenarios import (
    build_run_record,
    build_scenario_record,
    attach_measured_capture_pose,
    compare_capture_poses,
    config_fingerprint,
    historical_simulator,
    load_run,
    save_run,
)
from algorithm.simulator.task1_demo import build_task1_demo
from algorithm.simulator.task1_editor_model import EditorState, Task1EditorController, validate_editor_arena
from algorithm.simulator.production import TurnProfile, build_production_config, configuration_banner
from server.algo_server import _serialize_result


def test_production_profile_and_diagnostic_profiles_are_immutable_copies():
    production, live, _ = build_production_config()
    assert production.search_turn_angles_deg == (60.0, 90.0)
    assert production.observation_standoff_distances_cm == (20.0,)
    assert production.observation_lateral_offsets_cm == (0.0,)
    assert production.guaranteed_max_candidates_per_target == 1
    assert production.motion.turn_penalty_s == 0.0
    diagnostic = candidate_profile(production, CandidatePolicy.CENTERED_20_10_30)
    diagnostic = turn_profile(diagnostic, (30.0,))
    assert diagnostic.observation_standoff_distances_cm == (20.0, 10.0, 30.0)
    assert diagnostic.search_turn_angles_deg == (30.0,)
    assert production.observation_standoff_distances_cm == (20.0,)
    assert production.search_turn_angles_deg == (60.0, 90.0)
    with pytest.raises(FrozenInstanceError):
        production.arena_size_cm = 1


def test_production_banner_exposes_live_or_fallback_radii_and_diagnostic_delta():
    production, live, _ = build_production_config()
    diagnostic, _, _ = build_production_config(turn=TurnProfile.ONLY_90)
    banner = configuration_banner(diagnostic, live_calibration=live,
                                  diagnostic=True, production_config=production)
    assert "PRODUCTION CONFIG" in banner
    assert "ACTIVE DIAGNOSTIC CONFIG" in banner
    assert "60 degrees" in banner and "90 degrees" in banner
    assert "Calibration:" in banner
    assert "Candidate policy: centered 20 cm" in banner


def test_scenario_save_load_fingerprint_and_historical_replay_never_replan(tmp_path):
    demo = build_task1_demo()
    record = build_run_record(demo.simulator.state.arena, demo.config, demo.planning_result)
    path = tmp_path / "run.json"
    save_run(path, record)
    restored = load_run(path)
    assert restored["planner_config_fingerprint"] == config_fingerprint(demo.config)
    replay = historical_simulator(restored, demo.config)
    assert replay.state.planned_path == demo.planning_result.route.sampled_poses
    assert len(replay.steps) > 0
    assert [step.capture_obstacle_id for step in replay.steps if step.capture_obstacle_id] == list(
        demo.planning_result.route.target_order
    )


def test_pose_error_calculation_reports_signed_axes_and_wrapped_heading():
    result = compare_capture_poses(Pose(10, 20, 3.12), Pose(13, 24, -3.12))
    assert result["x_error_cm"] == 3
    assert result["y_error_cm"] == 4
    assert result["position_error_cm"] == 5
    assert result["heading_error_deg"] == pytest.approx(2.48, abs=0.1)


def test_unplanned_scenario_record_and_measured_capture_pose_round_trip(tmp_path):
    demo = build_task1_demo()
    layout = build_scenario_record(demo.simulator.state.arena, demo.config)
    assert layout["planned_route"]["sampled_poses"] == []
    layout_path = tmp_path / "layout.json"
    save_run(layout_path, layout)
    assert load_run(layout_path)["original_arena_payload"] == layout["original_arena_payload"]

    record = build_run_record(demo.simulator.state.arena, demo.config, demo.planning_result)
    target_id = record["target_order"][0]
    planned = record["planned_capture_poses"][str(target_id)]
    measured = Pose(planned["x_cm"] + 3, planned["y_cm"] + 4, planned["heading_rad"])
    attach_measured_capture_pose(record, target_id, measured)
    capture = next(item for item in record["planned_route"]["captures"]
                   if item["target_obstacle"] == target_id)
    assert capture["physical_vs_planned_error"]["position_error_cm"] == pytest.approx(5)


def test_production_editor_accepts_one_through_eight_targets_without_replanning_on_load():
    config, _, _ = build_production_config()
    controller = Task1EditorController(config, target_count_range=(1, 8), routing_mode=RoutingMode.FULL_OPTIMIZATION)
    for obstacle_id in range(1, 9):
        controller.add_obstacle(obstacle_id, GridCell(5 + obstacle_id, 8), Direction.NORTH)
    assert len(controller.obstacles) == 8
    with pytest.raises(ValueError, match="at most 8"):
        controller.add_obstacle(9, GridCell(17, 8), Direction.WEST)
    assert controller.planning_result is None
    assert controller.state in (EditorState.EDITING, EditorState.READY_TO_PLAN)


def test_editing_loaded_historical_scenario_discards_stale_route_record():
    config, _, _ = build_production_config()
    arena = build_task1_demo().simulator.state.arena
    controller = Task1EditorController(config, target_count_range=(1, 8))
    controller.load_arena(arena)
    controller.loaded_scenario_record = {"planned_route": {"sampled_poses": ["old"]}}
    controller.move_obstacle(arena.obstacles[0].obstacle_id, arena.obstacles[0].cell)
    assert controller.loaded_scenario_record is None
    assert controller.planning_result is None
    assert controller.simulator is None


def test_production_and_server_use_identical_wire_serialization():
    demo = build_task1_demo()
    assert serialize_planning_result(demo.planning_result) == _serialize_result(demo.planning_result)


def test_clearance_diagnostics_cover_complete_sampled_60_and_90_degree_motion():
    from algorithm.geometry import is_motion_collision_free, sample_motion
    from algorithm.models import ArenaInput
    from algorithm.models.motion import MotionPrimitive
    from algorithm.enums import Gear, Steering

    config = task1_robot_config(emit=lambda _message: None)
    arena = ArenaInput(Pose(100, 100, 0), (Obstacle(1, GridCell(18, 18), Direction.NORTH),))
    for command, gear, steering, sign in (
        ("FL", Gear.FORWARD, Steering.LEFT, 1), ("FR", Gear.FORWARD, Steering.RIGHT, -1),
        ("BL", Gear.REVERSE, Steering.LEFT, -1), ("BR", Gear.REVERSE, Steering.RIGHT, 1),
    ):
        for degrees in (60, 90):
            primitive = MotionPrimitive(command, gear, steering, turn_angle_rad=sign * degrees * 3.141592653589793 / 180,
                                        radius_cm=30)
            samples = sample_motion(arena.start_pose, primitive, config)
            assert len(samples) > 2
            assert is_motion_collision_free(arena.start_pose, primitive, arena, config)
    for command, gear in (("FW", Gear.FORWARD), ("BW", Gear.REVERSE)):
        primitive = MotionPrimitive(command, gear, Steering.STRAIGHT, travel_cm=10.0)
        assert len(sample_motion(arena.start_pose, primitive, config)) > 2
        assert is_motion_collision_free(arena.start_pose, primitive, arena, config)


def test_turn_sweep_rejects_intermediate_arena_boundary_crossing_with_legal_endpoints():
    import math

    from algorithm.geometry import is_motion_collision_free, is_pose_collision_free, sample_motion
    from algorithm.models import ArenaInput
    from algorithm.models.motion import MotionPrimitive
    from algorithm.enums import Gear, Steering

    config = task1_robot_config(emit=lambda _message: None)
    empty_arena = ArenaInput(Pose(100, 100, 0), ())
    cases = (
        ("FL", Gear.FORWARD, Steering.LEFT, 1, 60, Pose(17, 35, -1.3089969389957474)),
        ("FR", Gear.FORWARD, Steering.RIGHT, -1, 60, Pose(19, 19, 2.094395102393195)),
        ("BL", Gear.REVERSE, Steering.LEFT, -1, 60, Pose(19, 173, -2.0943951023931957)),
        ("BR", Gear.REVERSE, Steering.RIGHT, 1, 60, Pose(19, 19, 2.617993877991495)),
        ("FL", Gear.FORWARD, Steering.LEFT, 1, 90, Pose(15, 41, -math.pi / 2)),
        ("FR", Gear.FORWARD, Steering.RIGHT, -1, 90, Pose(15, 149, math.pi / 2)),
        ("BL", Gear.REVERSE, Steering.LEFT, -1, 90, Pose(15, 159, -math.pi / 2)),
        ("BR", Gear.REVERSE, Steering.RIGHT, 1, 90, Pose(15, 51, math.pi / 2)),
    )
    for command, gear, steering, sign, degrees, start in cases:
        radius = next(item.radius_cm for item in config.motion.primitives if item.command == command)
        primitive = MotionPrimitive(command, gear, steering,
                                    turn_angle_rad=sign * math.radians(degrees), radius_cm=radius)
        samples = sample_motion(start, primitive, config)
        assert is_pose_collision_free(samples[0], empty_arena, config)
        assert is_pose_collision_free(samples[-1], empty_arena, config)
        assert any(not is_pose_collision_free(sample, empty_arena, config) for sample in samples[1:-1])
        assert not is_motion_collision_free(start, primitive, empty_arena, config)


def test_scenario_fingerprint_changes_when_effective_settings_change():
    production = task1_robot_config(emit=lambda _message: None)
    assert config_fingerprint(production) != config_fingerprint(turn_profile(production, (90.0,)))


def test_seeded_stress_arena_is_reproducible_and_has_geometric_control_candidates():
    from algorithm.simulator.benchmark import seeded_arena
    from algorithm.targets import generate_arena_observation_candidates

    config, _, _ = build_production_config()
    first = seeded_arena(81000, 8, config)
    second = seeded_arena(81000, 8, config)
    assert first == second
    assert len(first.obstacles) == 8
    assert all(group.has_valid_candidate
               for group in generate_arena_observation_candidates(first, config))
