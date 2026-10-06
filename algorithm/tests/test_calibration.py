import ast
import math
from pathlib import Path

import pytest

from algorithm.calibration import load_turn_radius_calibration
from algorithm.config import UNCALIBRATED_SIMULATION_CONFIG, task1_robot_config
from algorithm.enums import Steering
from algorithm.geometry import propagate_motion
from algorithm.models import Pose
from algorithm.pathfinding.hybrid_astar import HybridAStarPlanner


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CALIBRATION_HEADER = REPOSITORY_ROOT / "stm32" / "Core" / "Inc" / "calib.h"


def write_header(path: Path, **values: object) -> None:
    lines = [f"#define TURN_RADIUS_{command}_MM {value}" for command, value in values.items()]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def radii(config):
    return {
        primitive.command: primitive.radius_cm
        for primitive in config.motion.primitives
        if primitive.steering is not Steering.STRAIGHT
    }


def test_current_calibration_header_is_loaded_and_converted_to_cm():
    result = load_turn_radius_calibration(CALIBRATION_HEADER)

    assert dict(result.values_mm) == {"FL": 272.0, "FR": 366.0, "BL": 279.0, "BR": 370.0}
    assert result.failures == ()


@pytest.mark.parametrize(
    ("macro", "value", "reason"),
    (
        ("TURN_RADIUS_FL_MM", "not-a-number", "malformed"),
        ("TURN_RADIUS_FR_MM", "0", "positive"),
        ("TURN_RADIUS_BL_MM", "-1", "positive"),
    ),
)
def test_invalid_macro_is_reported_per_field(tmp_path, macro, value, reason):
    path = tmp_path / "calib.h"
    path.write_text(f"#define {macro} {value}\n", encoding="utf-8")

    result = load_turn_radius_calibration(path)

    failure = next(item for item in result.failures if item.macro == macro)
    assert failure.reason.find(reason) >= 0


def test_missing_calibration_file_reports_every_fallback_field(tmp_path):
    result = load_turn_radius_calibration(tmp_path / "missing-calib.h")

    assert result.values_mm == ()
    assert tuple(item.command for item in result.failures) == ("FL", "FR", "BL", "BR")


def test_production_config_overlays_by_command_and_preserves_primitive_fields(tmp_path):
    path = tmp_path / "calib.h"
    path.write_text(
        "\n".join(
            (
                "#define TURN_RADIUS_BR_MM 370",
                "#define TURN_RADIUS_FL_MM 272",
                "#define TURN_RADIUS_BL_MM 279",
                "#define TURN_RADIUS_FR_MM 366",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    messages = []

    config = task1_robot_config(path, emit=messages.append)

    assert radii(config) == {"FL": 27.2, "FR": 36.6, "BL": 27.9, "BR": 37.0}
    assert messages == ["[CALIB] STM32 turn radii: FL=27.2 FR=36.6 BL=27.9 BR=37.0 cm"]
    for original, calibrated in zip(
        UNCALIBRATED_SIMULATION_CONFIG.motion.primitives,
        config.motion.primitives,
    ):
        assert calibrated.command == original.command
        assert calibrated.gear is original.gear
        assert calibrated.steering is original.steering
        assert calibrated.travel_cm == original.travel_cm
        assert calibrated.turn_angle_rad == original.turn_angle_rad
        assert calibrated.estimated_duration_s == original.estimated_duration_s


def test_mixed_calibration_uses_only_the_failed_field_fallback(tmp_path):
    path = tmp_path / "calib.h"
    write_header(path, FL=272, FR=366, BL=279)
    messages = []

    config = task1_robot_config(path, emit=messages.append)

    assert radii(config) == {"FL": 27.2, "FR": 36.6, "BL": 27.9, "BR": 38.3}
    assert messages[0].startswith("[CALIB] WARNING: TURN_RADIUS_BR_MM macro not found")
    assert messages[-1].endswith("[BR fallback]")


def test_all_fallback_config_keeps_existing_radii_and_warns(tmp_path):
    messages = []

    config = task1_robot_config(tmp_path / "missing-calib.h", emit=messages.append)

    assert radii(config) == {"FL": 27.7, "FR": 36.5, "BL": 28.1, "BR": 38.3}
    assert len(messages) == 5
    assert all("using Python fallback" in message for message in messages[:4])
    assert messages[-1].endswith("[FL fallback, FR fallback, BL fallback, BR fallback]")


def test_production_profile_preserves_integrated_bounds_and_runtime_search():
    config = task1_robot_config(CALIBRATION_HEADER, emit=lambda _message: None)

    assert config.robot.safety_margin_cm == 3.0
    assert config.observation_lateral_offsets_cm == (0.0,)
    assert config.guaranteed_max_candidates_per_target == 1
    assert config.max_expanded_nodes == 5000
    assert config.adaptive_initial_expansions == 200
    assert config.adaptive_max_expansions == 5000
    assert config.local_planning_timeout_s == 5.0
    assert config.overall_planning_timeout_s == 120.0
    assert config.turn_angles_deg == (30.0, 45.0, 60.0, 90.0)
    assert config.search_turn_angles_deg == (30.0,)
    assert config.heading_bin_rad == pytest.approx(math.radians(15.0))
    assert config.arena_size_cm == UNCALIBRATED_SIMULATION_CONFIG.arena_size_cm
    assert config.cell_size_cm == UNCALIBRATED_SIMULATION_CONFIG.cell_size_cm
    assert config.camera == UNCALIBRATED_SIMULATION_CONFIG.camera
    assert config.observation_standoff_distances_cm == (20.0, 10.0, 30.0)
    assert config.collision_translation_step_cm == UNCALIBRATED_SIMULATION_CONFIG.collision_translation_step_cm
    assert config.collision_arc_step_rad == UNCALIBRATED_SIMULATION_CONFIG.collision_arc_step_rad
    assert config.motion.straight_speed_cm_s == UNCALIBRATED_SIMULATION_CONFIG.motion.straight_speed_cm_s
    assert config.motion.serial_overhead_s == UNCALIBRATED_SIMULATION_CONFIG.motion.serial_overhead_s
    assert config.motion.straight_settle_s == UNCALIBRATED_SIMULATION_CONFIG.motion.straight_settle_s
    assert config.motion.straight_fixed_time_s == UNCALIBRATED_SIMULATION_CONFIG.motion.straight_fixed_time_s


def test_demo_profile_remains_ninety_degree_only():
    from algorithm.simulator.task1_demo import task1_demo_config

    demo = task1_demo_config()

    assert demo.turn_angles_deg == (90.0,)
    assert demo.search_turn_angles_deg == (90.0,)
    assert demo.heading_bin_rad == pytest.approx(math.pi / 2.0)


def test_partial_angle_successors_use_only_the_30_degree_runtime_branch():
    config = task1_robot_config(CALIBRATION_HEADER, emit=lambda _message: None)
    successors = HybridAStarPlanner(config)._successor_primitives()
    turns = tuple(primitive for primitive in successors if primitive.steering is not Steering.STRAIGHT)

    assert tuple(primitive.command for primitive in turns) == ("FL", "FR", "BL", "BR")
    assert tuple(abs(math.degrees(primitive.turn_angle_rad)) for primitive in turns) == pytest.approx((30.0,) * 4)
    assert tuple(primitive.radius_cm for primitive in turns) == (27.2, 36.6, 27.9, 37.0)


def test_calibrated_radius_changes_partial_arc_geometry(tmp_path):
    path = tmp_path / "calib.h"
    write_header(path, FL=300, FR=366, BL=279, BR=370)
    config = task1_robot_config(path, emit=lambda _message: None)
    primitive = next(item for item in config.motion.primitives if item.command == "FL")
    partial = primitive.__class__(
        primitive.command,
        primitive.gear,
        primitive.steering,
        turn_angle_rad=math.radians(30.0),
        radius_cm=primitive.radius_cm,
        estimated_duration_s=primitive.estimated_duration_s / 3.0,
    )

    endpoint = propagate_motion(
        Pose(100.0, 100.0, 0.0),
        partial,
        config,
    )

    assert partial.geometric_length_cm == pytest.approx(math.radians(30.0) * 30.0)
    assert endpoint.x_cm != pytest.approx(100.0)


def test_server_no_longer_imports_demo_configuration():
    source = (REPOSITORY_ROOT / "server" / "algo_server.py").read_text(encoding="utf-8")
    tree = ast.parse(source)

    imported_modules = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    assert "algorithm.simulator.task1_demo" not in imported_modules


def test_calibration_is_reloaded_for_each_production_config_request(tmp_path):
    path = tmp_path / "calib.h"
    write_header(path, FL=272, FR=366, BL=279, BR=370)
    first = task1_robot_config(path, emit=lambda _message: None)
    write_header(path, FL=300, FR=366, BL=279, BR=370)
    second = task1_robot_config(path, emit=lambda _message: None)

    assert radii(first)["FL"] == 27.2
    assert radii(second)["FL"] == 30.0


def test_server_creates_a_fresh_planner_for_each_request(tmp_path, monkeypatch):
    from server import algo_server

    path = tmp_path / "calib.h"
    write_header(path, FL=272, FR=366, BL=279, BR=370)
    planners = []

    class FakePlanner:
        def __init__(self, config):
            planners.append(config)

        def plan(self, _arena):
            return object()

    monkeypatch.setattr(algo_server, "Task1Planner", FakePlanner)
    monkeypatch.setattr(
        algo_server,
        "task1_robot_config",
        lambda: task1_robot_config(path, emit=lambda _message: None),
    )

    payload = {"obstacles": []}
    algo_server._plan_payload(payload)
    write_header(path, FL=300, FR=366, BL=279, BR=370)
    algo_server._plan_payload(payload)

    assert len(planners) == 2
    assert radii(planners[0])["FL"] == 27.2
    assert radii(planners[1])["FL"] == 30.0
    assert planners[0] is not planners[1]
