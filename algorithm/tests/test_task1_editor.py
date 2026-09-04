import math
import subprocess
import sys

import pytest

from algorithm.config import UNCALIBRATED_SIMULATION_CONFIG
from algorithm.constants import START_ZONE_GRID_CELLS, START_ZONE_SIZE_CM
from algorithm.coordinates import default_start_pose
from algorithm.enums import CostMetric, Direction, Gear, PlanningStatus, Steering
from algorithm.geometry import is_motion_collision_free, robot_footprint
from algorithm.models import (
    ArenaInput,
    CaptureStep,
    GridCell,
    Obstacle,
    PlanningIssue,
    PlanningResult,
)
from algorithm.models.motion import MotionPrimitive, MotionSegment
from algorithm.models.planning import PathMetrics
from algorithm.pathfinding import HybridPath, LocalPlanningResult, LocalPlanningStatus
from algorithm.routing import Task1Planner
from algorithm.simulator import PlaybackState
from algorithm.simulator.task1_demo import (
    build_task1_demo,
    task1_demo_obstacles,
)
from algorithm.simulator.task1_editor_model import (
    EditorState,
    RandomFailureCategory,
    Task1EditorController,
    benchmark_assessment_like_scenarios,
    benchmark_one_cell_perturbations,
    classify_planning_result,
    generate_random_task1_arena,
    generate_assessment_like_task1_arena,
    scenario_signature,
    task1_editor_config,
    validate_editor_arena,
)
from algorithm.targets import generate_arena_observation_candidates


class StraightFakePathPlanner:
    def plan(self, start, goal, arena, *, objective=CostMetric.ESTIMATED_TIME, collect_debug=False):
        distance = math.hypot(goal.x_cm - start.x_cm, goal.y_cm - start.y_cm)
        estimated_time = distance / 10.0
        cost = distance if objective is CostMetric.DISTANCE else estimated_time
        primitive = MotionPrimitive(
            "FW",
            Gear.FORWARD,
            Steering.STRAIGHT,
            travel_cm=max(distance, 0.001),
        )
        segment = MotionSegment(primitive, start, goal)
        metrics = PathMetrics(
            geometric_distance_cm=distance,
            estimated_time_s=estimated_time,
            forward_distance_cm=distance,
            command_count=1,
        )
        path = HybridPath(
            start,
            goal,
            goal,
            (segment,),
            (start, goal),
            metrics,
            objective,
            cost,
            (0.0, cost),
        )
        return LocalPlanningResult(LocalPlanningStatus.SUCCESS, start, goal, path, metrics)


class AlwaysFailingTask1Planner:
    def __init__(self):
        self.calls = []

    def plan(self, arena, *, objective=CostMetric.ESTIMATED_TIME):
        self.calls.append(arena)
        return PlanningResult(
            PlanningStatus.NO_FEASIBLE_ROUTE,
            issues=(PlanningIssue("test_no_route", "configured unsolvable random arena"),),
        )


class NominalFailingPathPlanner(StraightFakePathPlanner):
    def __init__(self, nominal_goals):
        self.nominal_goals = set(nominal_goals)

    def plan(self, start, goal, arena, *, objective=CostMetric.ESTIMATED_TIME, collect_debug=False):
        if goal in self.nominal_goals:
            return LocalPlanningResult(
                LocalPlanningStatus.NO_PATH,
                start,
                goal,
                metrics=PathMetrics(nodes_expanded=1),
                message="nominal deliberately unreachable",
            )
        return super().plan(start, goal, arena, objective=objective, collect_debug=collect_debug)


def fake_editor(*, obstacles=task1_demo_obstacles()):
    config = task1_editor_config()
    planner = Task1Planner(config, path_planner=StraightFakePathPlanner())
    return Task1EditorController(config, obstacles=obstacles, planner=planner)


@pytest.fixture(scope="module")
def alternate_fixed_editor():
    controller = Task1EditorController(obstacles=task1_demo_obstacles())
    controller.move_obstacle(1, GridCell(9, 4))
    result = controller.plan()
    assert result.status is PlanningStatus.SUCCESS
    return controller


@pytest.fixture(scope="module")
def seeded_solvable_editor():
    controller = Task1EditorController()
    outcome = controller.randomize(seed=1, require_solvable=True, retry_limit=10)
    assert outcome.succeeded
    return controller


def test_interactive_model_add_move_face_remove_and_exact_count_validation():
    controller = fake_editor(obstacles=())
    assert controller.state is EditorState.EDITING
    for obstacle_id, cell in enumerate(
        (GridCell(8, 4), GridCell(6, 10), GridCell(13, 8), GridCell(10, 14), GridCell(16, 12)),
        start=1,
    ):
        controller.add_obstacle(obstacle_id, cell, Direction.WEST)

    assert controller.state is EditorState.READY_TO_PLAN
    controller.move_obstacle(1, GridCell(9, 4))
    controller.change_face(1, Direction.SOUTH)
    assert controller.obstacles[0].cell == GridCell(9, 4)
    assert controller.obstacles[0].face is Direction.SOUTH

    controller.remove_obstacle(5)
    assert controller.state is EditorState.EDITING
    result = controller.plan()
    assert result.status is PlanningStatus.INVALID_INPUT
    assert result.issues[0].code == "incorrect_target_count"


def test_duplicate_cells_and_start_zone_are_rejected_without_duplicate_geometry_logic():
    controller = fake_editor()
    with pytest.raises(ValueError, match="same cell"):
        controller.move_obstacle(1, controller.obstacles[1].cell)
    assert len(controller.obstacles) == 5

    controller.move_obstacle(1, GridCell(2, 2))
    codes = {issue.code for issue in validate_editor_arena(controller.arena, controller.config)}
    assert "obstacle_in_start_zone" in codes
    assert "robot_start_collision" in codes


def test_edit_invalidates_old_plan_and_replan_uses_updated_arena():
    controller = fake_editor()
    first = controller.plan()
    assert first.status is PlanningStatus.SUCCESS
    assert controller.simulator is not None

    controller.move_obstacle(1, GridCell(9, 4))
    assert controller.planning_result is None
    assert controller.simulator is None
    assert controller.state is EditorState.READY_TO_PLAN

    second = controller.plan()
    assert second.status is PlanningStatus.SUCCESS
    assert second.metrics.pairwise_cache_misses > 0
    assert controller.simulator.state.arena.obstacles[0].cell == GridCell(9, 4)


def test_solvable_randomization_replans_new_arena_without_stale_cache_entries():
    config = task1_editor_config()
    planner = Task1Planner(config, path_planner=StraightFakePathPlanner())
    controller = Task1EditorController(config, obstacles=task1_demo_obstacles(), planner=planner)
    original = controller.plan()
    outcome = controller.randomize(seed=33, require_solvable=True, retry_limit=5)

    assert original.status is PlanningStatus.SUCCESS
    assert outcome.succeeded
    assert outcome.arena != ArenaInput(default_start_pose(config.robot), task1_demo_obstacles())
    assert outcome.planning_result.metrics.pairwise_cache_misses > 0
    assert controller.simulator.state.arena == outcome.arena
    assert controller.planning_result.route.observation_poses != original.route.observation_poses


def test_active_playback_prevents_obstacle_teleport():
    controller = fake_editor()
    controller.plan()
    controller.play_pause()
    assert controller.state is EditorState.PLAYING
    with pytest.raises(RuntimeError, match="stop active playback"):
        controller.move_obstacle(1, GridCell(9, 4))


def test_raw_random_seed_is_reproducible_unique_and_seed_sensitive():
    config = task1_editor_config()
    first = generate_random_task1_arena(config, seed=42)
    repeated = generate_random_task1_arena(config, seed=42)
    different = generate_random_task1_arena(config, seed=43)

    assert first == repeated
    assert first != different
    assert first.start_pose == default_start_pose(config.robot)
    assert len(first.obstacles) == len({item.cell for item in first.obstacles}) == 5
    assert all(item.face in tuple(Direction) for item in first.obstacles)
    assert all(5 <= value <= 14 for item in first.obstacles for value in (item.cell.x, item.cell.y))


def test_solvable_random_retry_is_bounded_and_failure_remains_structured():
    failing = AlwaysFailingTask1Planner()
    controller = Task1EditorController(planner=failing)
    outcome = controller.randomize(seed=7, require_solvable=True, retry_limit=3)

    assert not outcome.succeeded
    assert outcome.attempts == len(failing.calls) == 3
    assert controller.state is EditorState.EDITING
    assert controller.simulator is None
    assert outcome.planning_result.status is PlanningStatus.NO_FEASIBLE_ROUTE
    assert "RANDOM SOLVABLE GENERATION FAILED" in controller.status_message


def test_alternate_fixed_five_target_map_uses_real_planner(alternate_fixed_editor):
    controller = alternate_fixed_editor
    route = controller.planning_result.route
    assert controller.obstacles != task1_demo_obstacles()
    assert controller.obstacles[0].cell == GridCell(9, 4)
    assert route.target_order == (1, 2, 3, 4, 5)
    assert len(route.local_paths) == 5


def test_mixed_image_faces_drive_opposite_robot_observation_headings(alternate_fixed_editor):
    controller = alternate_fixed_editor
    route = controller.planning_result.route
    faces = {item.face for item in controller.obstacles}
    assert faces == {Direction.SOUTH, Direction.WEST}
    face_by_id = {item.obstacle_id: item.face for item in controller.obstacles}
    for observation in route.observation_poses:
        heading = Direction.from_heading_rad(observation.pose.heading_rad)
        assert heading is face_by_id[observation.obstacle_id].opposite()


def test_geometrically_invalid_nominal_selects_real_fallback_with_five_targets():
    config = UNCALIBRATED_SIMULATION_CONFIG
    arena = ArenaInput(
        default_start_pose(config.robot),
        (
            Obstacle(1, GridCell(10, 10), Direction.NORTH),
            Obstacle(2, GridCell(11, 14), Direction.SOUTH),
            Obstacle(3, GridCell(5, 5), Direction.EAST),
            Obstacle(4, GridCell(15, 5), Direction.WEST),
            Obstacle(5, GridCell(5, 15), Direction.SOUTH),
        ),
    )
    groups = generate_arena_observation_candidates(arena, config)
    assert not groups[0].candidates[0].valid
    assert groups[0].candidates[2].valid

    result = Task1Planner(config, path_planner=StraightFakePathPlanner()).plan(arena)
    selected = dict(zip(result.route.target_order, result.route.selected_candidate_kinds))
    assert result.status is PlanningStatus.SUCCESS
    assert selected[1] == "10R"
    reachability = {item.target_id: item for item in result.metrics.target_reachability}
    assert reachability[1].geometric_candidates == 3
    assert reachability[1].reachable_candidates == 3


def test_seeded_solvable_random_route_is_complete_continuous_and_collision_free(
    seeded_solvable_editor,
):
    controller = seeded_solvable_editor
    result = controller.planning_result
    route = result.route
    assert len(controller.obstacles) == len(route.observation_poses) == 5
    assert len(set(route.target_order)) == 5
    assert all(
        is_motion_collision_free(segment.start, segment.primitive, controller.arena, controller.config)
        for path in route.local_paths
        for segment in path.segments
    )
    segments = tuple(segment for path in route.local_paths for segment in path.segments)
    assert all(first.end == second.start for first, second in zip(segments, segments[1:]))
    captures = tuple(step for step in route.execution_steps if isinstance(step, CaptureStep))
    assert tuple(step.obstacle_id for step in captures) == route.target_order

    controller.play_pause()
    controller.advance(10_000.0)
    assert controller.state is EditorState.COMPLETE
    assert controller.simulator.state.playback_state is PlaybackState.COMPLETE
    assert controller.simulator.state.visited_target_ids == route.target_order


def test_seeded_solvable_start_footprint_remains_inside_start_zone(seeded_solvable_editor):
    controller = seeded_solvable_editor
    start = controller.arena.start_pose
    assert start == default_start_pose(controller.config.robot)
    assert all(
        0.0 <= coordinate <= 40.0
        for point in robot_footprint(start, controller.config.robot)
        for coordinate in (point.x_cm, point.y_cm)
    )


def test_corrected_start_zone_is_four_cells_and_preserves_authoritative_start_pose():
    config = task1_editor_config()
    start = default_start_pose(config.robot)
    assert START_ZONE_SIZE_CM == 40
    assert START_ZONE_GRID_CELLS == 4
    assert (start.x_cm, start.y_cm, start.heading_rad) == (15.0, 15.0, math.pi / 2.0)
    assert all(
        0.0 <= coordinate <= START_ZONE_SIZE_CM
        for point in robot_footprint(start, config.robot)
        for coordinate in (point.x_cm, point.y_cm)
    )
    arena = generate_assessment_like_task1_arena(config, seed=42)
    assert all(not (item.cell.x < 4 and item.cell.y < 4) for item in arena.obstacles)


def test_planning_timing_breakdown_is_reported(alternate_fixed_editor):
    metrics = alternate_fixed_editor.planning_result.metrics
    assert metrics.candidate_generation_time_s >= 0.0
    assert metrics.pairwise_planning_time_s > 0.0
    assert metrics.global_routing_time_s >= 0.0
    assert metrics.total_planning_time_s >= (
        metrics.candidate_generation_time_s
        + metrics.pairwise_planning_time_s
        + metrics.global_routing_time_s
    )


def test_editor_core_import_does_not_load_pygame():
    script = (
        "import sys; import algorithm.simulator.task1_editor_model; "
        "assert 'pygame' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", script], check=True)


def test_editor_renderer_smoke(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    from algorithm.simulator.task1_editor import Task1EditorApp

    app = Task1EditorApp(fake_editor())
    app.renderer.initialize()
    app._font = pygame.font.Font(None, 22)
    app._small_font = pygame.font.Font(None, 17)
    try:
        app.render()
        pygame.display.flip()
    finally:
        app.renderer.shutdown()
    assert not pygame.get_init()


def test_shift_f5_handler_advances_persistent_random_stream(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    from algorithm.simulator.task1_editor import Task1EditorApp

    config = task1_editor_config()
    controller = Task1EditorController(
        config,
        planner=Task1Planner(config, path_planner=StraightFakePathPlanner()),
        random_seed=321,
    )
    app = Task1EditorApp(controller)
    app.renderer.initialize()
    app._font = pygame.font.Font(None, 22)
    app._small_font = pygame.font.Font(None, 17)
    try:
        shift_f5 = pygame.event.Event(
            pygame.KEYDOWN,
            key=pygame.K_F5,
            mod=pygame.KMOD_SHIFT,
        )
        app._handle_key(shift_f5)
        first = scenario_signature(controller.arena)
        app._handle_key(shift_f5)
        second = scenario_signature(controller.arena)
    finally:
        app.renderer.shutdown()
    assert first != second


def test_shift_f5_retains_plan_and_space_plays_without_enter():
    config = task1_editor_config()
    controller = Task1EditorController(
        config,
        planner=Task1Planner(config, path_planner=StraightFakePathPlanner()),
        random_seed=44,
    )
    outcome = controller.randomize(require_solvable=True, retry_limit=5)
    assert outcome.succeeded
    assert controller.state is EditorState.PLAN_READY
    assert controller.planning_result is outcome.planning_result
    controller.play_pause()
    assert controller.state is EditorState.PLAYING


def test_live_editor_expands_to_fallback_candidates_and_completes_route():
    config = task1_editor_config()
    arena = ArenaInput(default_start_pose(config.robot), task1_demo_obstacles())
    groups = generate_arena_observation_candidates(arena, config)
    nominal_goals = tuple(group.candidates[0].observation_pose.pose for group in groups)
    planner = Task1Planner(config, path_planner=NominalFailingPathPlanner(nominal_goals))
    controller = Task1EditorController(
        config,
        obstacles=task1_demo_obstacles(),
        planner=planner,
    )

    result = controller.plan()

    assert result.status is PlanningStatus.SUCCESS
    assert result.metrics.candidate_tiers_activated == 2
    assert result.metrics.candidate_count_considered > 5
    assert all(label != "20C" for label in result.route.selected_candidate_kinds)
    assert len([step for step in result.route.execution_steps if isinstance(step, CaptureStep)]) == 5
    controller.play_pause()
    controller.advance(10_000.0)
    assert controller.simulator.state.visited_target_ids == result.route.target_order


def test_edit_after_retained_random_plan_invalidates_every_execution_artifact():
    controller = fake_editor()
    outcome = controller.randomize(seed=9, require_solvable=True, retry_limit=5)
    assert outcome.succeeded and controller.simulator is not None
    obstacle = controller.obstacles[0]
    controller.move_obstacle(obstacle.obstacle_id, GridCell(obstacle.cell.x + 1, obstacle.cell.y))
    assert controller.planning_result is None
    assert controller.simulator is None
    assert controller.state is EditorState.READY_TO_PLAN


def test_editor_wasd_face_mapping_r_reset_and_z_is_not_reset(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    from algorithm.simulator.task1_editor import Task1EditorApp

    controller = fake_editor()
    app = Task1EditorApp(controller)
    app.selected_obstacle_id = 1
    for key, expected in (
        (pygame.K_w, Direction.NORTH),
        (pygame.K_a, Direction.WEST),
        (pygame.K_s, Direction.SOUTH),
        (pygame.K_d, Direction.EAST),
    ):
        app._handle_key(pygame.event.Event(pygame.KEYDOWN, key=key, mod=0))
        assert controller.obstacles[0].face is expected
    controller.plan()
    controller.play_pause()
    controller.advance(1.0)
    app._handle_key(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_r, mod=0))
    assert controller.state is EditorState.PLAN_READY
    before = controller.simulator.state.current_step_index
    app._handle_key(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_z, mod=0))
    assert controller.simulator.state.current_step_index == before


def test_assessment_generator_and_benchmarks_are_seeded_and_structured():
    config = task1_editor_config()
    assert generate_assessment_like_task1_arena(config, seed=42) == generate_assessment_like_task1_arena(config, seed=42)
    planner = Task1Planner(config, path_planner=StraightFakePathPlanner())
    report = benchmark_assessment_like_scenarios(config, count=100, seed=5, planner=planner)
    assert report.count == 100
    assert sum(count for _, count in report.category_counts) == 100
    assert report.candidate_poses_generated == 4500
    perturbations = benchmark_one_cell_perturbations(
        ArenaInput(default_start_pose(config.robot), task1_demo_obstacles()),
        config,
        planner=planner,
    )
    assert perturbations.tested > 0
    assert perturbations.successes + perturbations.no_geometric_candidate + perturbations.no_path + perturbations.search_limit_reached + perturbations.global_connectivity_failure == perturbations.tested


def test_editor_and_seeded_random_cli_branches_smoke():
    base = (
        "import sys,runpy; import algorithm.simulator.app as app; "
        "import algorithm.simulator.task1_editor as editor; "
        "app.run_simulator=lambda *a,**k: None; "
        "editor.run_task1_editor=lambda controller: print(controller.state.value); "
    )
    editor = subprocess.run(
        [
            sys.executable,
            "-c",
            base
            + "sys.argv=['algorithm.simulator','--task1-editor']; "
            + "runpy.run_module('algorithm.simulator',run_name='__main__')",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    seeded = subprocess.run(
        [
            sys.executable,
            "-c",
            base
            + "sys.argv=['algorithm.simulator','--task1-random','--seed','42']; "
            + "runpy.run_module('algorithm.simulator',run_name='__main__')",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "ready_to_plan" in editor.stdout
    assert "seed=42" in seeded.stdout


def test_original_task1_demo_remains_deterministic_after_editor_addition():
    first = build_task1_demo().planning_result.route
    second = build_task1_demo().planning_result.route
    assert first.target_order == second.target_order
    assert first.primitives == second.primitives
    assert first.sampled_poses == second.sampled_poses


def test_unseeded_solvable_requests_advance_persistent_rng_and_change_multiple_cells():
    config = task1_editor_config()
    controller = Task1EditorController(
        config,
        planner=Task1Planner(config, path_planner=StraightFakePathPlanner()),
        random_seed=123,
    )
    first = controller.randomize(require_solvable=True, retry_limit=5)
    second = controller.randomize(require_solvable=True, retry_limit=5)
    first_cells = {item.obstacle_id: item.cell for item in first.arena.obstacles}
    second_cells = {item.obstacle_id: item.cell for item in second.arena.obstacles}

    assert first.succeeded and second.succeeded
    assert first.request_number == 1 and second.request_number == 2
    assert scenario_signature(first.arena) != scenario_signature(second.arena)
    assert sum(first_cells[target] != second_cells[target] for target in first_cells) >= 2


def test_explicit_seed_is_reproducible_across_fresh_controllers():
    config = task1_editor_config()

    def generate():
        controller = Task1EditorController(
            config,
            planner=Task1Planner(config, path_planner=StraightFakePathPlanner()),
        )
        return controller.randomize(seed=88, require_solvable=True, retry_limit=5)

    first = generate()
    second = generate()
    assert first.succeeded and second.succeeded
    assert first.arena == second.arena
    assert scenario_signature(first.arena) == scenario_signature(second.arena)


def test_failed_solvable_request_preserves_current_arena_without_demo_fallback():
    original = task1_demo_obstacles()
    controller = Task1EditorController(obstacles=original, planner=AlwaysFailingTask1Planner())
    before = controller.arena
    outcome = controller.randomize(seed=9, require_solvable=True, retry_limit=2)

    assert not outcome.succeeded
    assert controller.arena == before == outcome.arena
    assert controller.obstacles == original
    assert "previous scenario preserved" in controller.status_message


def test_scenario_signature_is_stable_and_changes_with_coordinates_or_faces():
    config = task1_editor_config()
    first = generate_random_task1_arena(config, seed=1)
    repeated = generate_random_task1_arena(config, seed=1)
    different = generate_random_task1_arena(config, seed=2)
    assert scenario_signature(first) == scenario_signature(repeated)
    assert scenario_signature(first) != scenario_signature(different)
    assert "1@" in scenario_signature(first)


def test_search_limit_and_no_path_reachability_failures_are_distinguished():
    search_limit = PlanningResult(
        PlanningStatus.NO_FEASIBLE_ROUTE,
        issues=(PlanningIssue("pairwise_search_limit_reached", "search bound reached"),),
    )
    no_path = PlanningResult(
        PlanningStatus.NO_FEASIBLE_ROUTE,
        issues=(PlanningIssue("no_reachable_observation_pose", "no local path"),),
    )
    assert classify_planning_result(search_limit) is RandomFailureCategory.SEARCH_LIMIT_REACHED
    assert classify_planning_result(no_path) is RandomFailureCategory.LOCAL_REACHABILITY_FAILURE


def test_ten_real_solvable_random_maps_are_diverse_complete_and_collision_free():
    controller = Task1EditorController(random_seed=100)
    signatures = []
    face_configurations = []
    target_orders = []
    for _ in range(10):
        outcome = controller.randomize(require_solvable=True, retry_limit=50)
        assert outcome.succeeded
        result = controller.planning_result
        route = result.route
        assert len(controller.obstacles) == len({item.cell for item in controller.obstacles}) == 5
        assert len(route.observation_poses) == len(set(route.target_order)) == 5
        assert all(
            first.end == second.start
            for first, second in zip(
                tuple(segment for path in route.local_paths for segment in path.segments),
                tuple(segment for path in route.local_paths for segment in path.segments)[1:],
            )
        )
        assert all(
            is_motion_collision_free(
                segment.start,
                segment.primitive,
                controller.arena,
                controller.config,
            )
            for path in route.local_paths
            for segment in path.segments
        )
        assert len(
            [step for step in route.execution_steps if isinstance(step, CaptureStep)]
        ) == 5

        controller.play_pause()
        controller.advance(10_000.0)
        assert controller.state is EditorState.COMPLETE
        assert len(controller.simulator.state.visited_target_ids) == 5
        signatures.append(scenario_signature(controller.arena))
        face_configurations.append(
            tuple((item.obstacle_id, item.face) for item in controller.obstacles)
        )
        target_orders.append(route.target_order)

    assert len(set(signatures)) == 10
    assert len(set(face_configurations)) > 1
    assert len(set(target_orders)) > 1
