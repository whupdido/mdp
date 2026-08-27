import math
import subprocess
import sys
from dataclasses import replace

import pytest

from algorithm.config import UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import CostMetric, Direction, Gear, Steering
from algorithm.geometry import (
    is_motion_collision_free,
    is_pose_collision_free,
    propagate_motion,
    sample_motion,
)
from algorithm.models import ArenaInput, GridCell, Obstacle, Pose
from algorithm.models.motion import MotionPrimitive
from algorithm.pathfinding import (
    HybridAStarPlanner,
    LocalPlanningStatus,
    goal_reached,
    search_key,
)
from algorithm.simulator.hybrid_demo import build_hybrid_demo


CONFIG = UNCALIBRATED_SIMULATION_CONFIG


def primitive(command: str, config=CONFIG) -> MotionPrimitive:
    return config.motion.primitives_for(command)[0]


def endpoint(start: Pose, commands: tuple[str, ...], config=CONFIG) -> Pose:
    pose = start
    for command in commands:
        pose = propagate_motion(pose, primitive(command, config), config)
    return pose


def compact_config(
    *,
    travel_cm: float = 10.0,
    radius_cm: float = 10.0,
    arena_size_cm: float = 200.0,
    robot_size_cm: float = 4.0,
):
    quarter_turn = math.pi / 2.0
    primitives = (
        MotionPrimitive("FW", Gear.FORWARD, Steering.STRAIGHT, travel_cm=travel_cm),
        MotionPrimitive("BW", Gear.REVERSE, Steering.STRAIGHT, travel_cm=travel_cm),
        MotionPrimitive(
            "FL", Gear.FORWARD, Steering.LEFT,
            turn_angle_rad=quarter_turn, radius_cm=radius_cm,
            estimated_duration_s=1.0,
        ),
        MotionPrimitive(
            "FR", Gear.FORWARD, Steering.RIGHT,
            turn_angle_rad=-quarter_turn, radius_cm=radius_cm,
            estimated_duration_s=1.0,
        ),
        MotionPrimitive(
            "BL", Gear.REVERSE, Steering.LEFT,
            turn_angle_rad=-quarter_turn, radius_cm=radius_cm,
            estimated_duration_s=1.0,
        ),
        MotionPrimitive(
            "BR", Gear.REVERSE, Steering.RIGHT,
            turn_angle_rad=quarter_turn, radius_cm=radius_cm,
            estimated_duration_s=1.0,
        ),
    )
    return replace(
        CONFIG,
        arena_size_cm=arena_size_cm,
        robot=replace(
            CONFIG.robot,
            length_cm=robot_size_cm,
            width_cm=robot_size_cm,
            safety_margin_cm=0.0,
        ),
        motion=replace(CONFIG.motion, primitives=primitives),
        position_bin_cm=2.5,
        goal_position_tolerance_cm=0.25,
        max_expanded_nodes=5_000,
    )


def assert_successful_path(result, arena: ArenaInput, config) -> None:
    assert result.status is LocalPlanningStatus.SUCCESS
    assert result.path is not None
    path = result.path
    assert path.start == result.start
    assert path.sampled_poses[0] == result.start
    assert path.sampled_poses[-1] == path.final_pose
    assert goal_reached(path.final_pose, result.requested_goal, config)
    assert all(segment.primitive in config.motion.primitives for segment in path.segments)
    assert tuple(segment.start for segment in path.segments) == path.key_poses[:-1]
    assert tuple(segment.end for segment in path.segments) == path.key_poses[1:]
    assert all(
        is_motion_collision_free(segment.start, segment.primitive, arena, config)
        for segment in path.segments
    )
    assert all(is_pose_collision_free(pose, arena, config) for pose in path.sampled_poses)
    assert path.metrics.geometric_distance_cm >= 0.0
    assert path.metrics.estimated_time_s >= 0.0
    assert all(
        following >= current
        for current, following in zip(path.cumulative_costs, path.cumulative_costs[1:])
    )


def test_straight_reachable_goal_and_parent_reconstruction():
    start = Pose(100.0, 100.0, 0.0)
    goal = Pose(130.0, 100.0, 0.0)
    arena = ArenaInput(start)
    result = HybridAStarPlanner(CONFIG).plan(start, goal, arena, objective=CostMetric.DISTANCE)

    assert_successful_path(result, arena, CONFIG)
    assert [item.command for item in result.path.primitives] == ["FW", "FW", "FW"]
    assert result.path.key_poses[0] == start
    assert result.path.final_pose == goal
    assert result.metrics.geometric_distance_cm == pytest.approx(30.0)
    assert result.metrics.forward_distance_cm == pytest.approx(30.0)
    assert result.metrics.reverse_distance_cm == 0.0


def test_forward_turn_goal_preserves_continuous_arc_endpoint_and_heading():
    start = Pose(80.0, 80.0, 0.0)
    goal = endpoint(start, ("FL",))
    arena = ArenaInput(start)
    result = HybridAStarPlanner(CONFIG).plan(start, goal, arena, objective=CostMetric.DISTANCE)

    assert_successful_path(result, arena, CONFIG)
    assert [item.command for item in result.path.primitives] == ["FL"]
    assert result.path.final_pose.x_cm == pytest.approx(106.1)
    assert result.path.final_pose.y_cm == pytest.approx(106.1)
    assert result.path.final_pose.heading_rad == pytest.approx(math.pi / 2.0)
    assert result.metrics.turn_count == 1


@pytest.mark.parametrize("command", ["FR", "BL", "BR"])
def test_other_configured_arc_successors_are_searchable(command):
    start = Pose(100.0, 100.0, 0.0)
    goal = endpoint(start, (command,))
    arena = ArenaInput(start)
    result = HybridAStarPlanner(CONFIG).plan(start, goal, arena, objective=CostMetric.DISTANCE)

    assert_successful_path(result, arena, CONFIG)
    assert [item.command for item in result.path.primitives] == [command]


def test_initial_planner_successor_set_comes_from_configuration():
    assert tuple(item.command for item in CONFIG.motion.primitives) == (
        "FW", "BW", "FL", "FR", "BL", "BR"
    )


def test_reverse_goal_uses_configured_reverse_primitive():
    start = Pose(80.0, 80.0, 0.0)
    goal = endpoint(start, ("BW",))
    arena = ArenaInput(start)
    result = HybridAStarPlanner(CONFIG).plan(start, goal, arena, objective=CostMetric.DISTANCE)

    assert_successful_path(result, arena, CONFIG)
    assert [item.command for item in result.path.primitives] == ["BW"]
    assert result.metrics.reverse_distance_cm == pytest.approx(10.0)
    assert result.metrics.forward_distance_cm == 0.0


def test_multiple_primitives_track_direction_steering_and_turn_metrics():
    config = compact_config()
    config = replace(
        config,
        motion=replace(
            config.motion,
            direction_change_penalty_s=1.25,
            steering_change_penalty_s=0.75,
        ),
    )
    start = Pose(50.0, 50.0, 0.0)
    goal = endpoint(start, ("FW", "FL", "BW"), config)
    arena = ArenaInput(start)
    result = HybridAStarPlanner(config).plan(
        start,
        goal,
        arena,
        objective=CostMetric.ESTIMATED_TIME,
    )

    assert_successful_path(result, arena, config)
    assert [item.command for item in result.path.primitives] == ["FW", "FL", "BW"]
    assert result.metrics.direction_changes == 1
    assert result.metrics.steering_changes == 2
    assert result.metrics.turn_count == 1
    assert result.path.objective_cost == pytest.approx(result.metrics.estimated_time_s)


def test_obstacle_requires_collision_free_command_aligned_detour():
    config = compact_config()
    start = Pose(50.0, 50.0, 0.0)
    arena = ArenaInput(
        start,
        (Obstacle(1, GridCell(7, 4), Direction.NORTH),),
    )
    goal = Pose(90.0, 50.0, 0.0)
    result = HybridAStarPlanner(config).plan(start, goal, arena, objective=CostMetric.DISTANCE)

    assert_successful_path(result, arena, config)
    assert [item.command for item in result.path.primitives] == ["FL", "FR", "FR", "FL"]
    assert result.metrics.turn_count == 4


def test_invalid_start_is_rejected_before_search():
    start = Pose(95.0, 95.0, 0.0)
    arena = ArenaInput(start, (Obstacle(1, GridCell(9, 9), Direction.NORTH),))
    result = HybridAStarPlanner(CONFIG).plan(start, Pose(150.0, 150.0, 0.0), arena)

    assert result.status is LocalPlanningStatus.INVALID_START
    assert result.path is None
    assert result.metrics.nodes_expanded == 0
    assert result.metrics.collision_checks == 1


@pytest.mark.parametrize(
    "goal",
    [Pose(105.0, 105.0, 0.0), Pose(5.0, 100.0, 0.0)],
    ids=["inside_obstacle", "outside_footprint_area"],
)
def test_invalid_goal_is_rejected_before_search(goal):
    start = Pose(150.0, 150.0, 0.0)
    arena = ArenaInput(start, (Obstacle(1, GridCell(10, 10), Direction.NORTH),))
    result = HybridAStarPlanner(CONFIG).plan(start, goal, arena)

    assert result.status is LocalPlanningStatus.INVALID_GOAL
    assert result.path is None
    assert result.metrics.nodes_expanded == 0
    assert result.metrics.collision_checks == 2


def test_blocked_start_region_returns_no_path_not_exception():
    start = Pose(50.0, 50.0, 0.0)
    arena = ArenaInput(
        start,
        (
            Obstacle(1, GridCell(7, 5), Direction.NORTH),
            Obstacle(2, GridCell(2, 5), Direction.NORTH),
        ),
    )
    result = HybridAStarPlanner(CONFIG).plan(start, Pose(100.0, 100.0, 0.0), arena)

    assert result.status is LocalPlanningStatus.NO_PATH
    assert result.path is None
    assert result.metrics.nodes_expanded == 1
    assert result.metrics.nodes_generated == len(CONFIG.motion.primitives)


def test_valid_goal_enclosed_by_obstacles_returns_no_path():
    config = compact_config(
        travel_cm=5.0,
        radius_cm=5.0,
        arena_size_cm=40.0,
        robot_size_cm=2.0,
    )
    start = Pose(5.0, 5.0, 0.0)
    ring = ((1, 1), (2, 1), (3, 1), (1, 2), (3, 2), (1, 3), (2, 3), (3, 3))
    obstacles = tuple(
        Obstacle(index + 1, GridCell(x, y), Direction.NORTH)
        for index, (x, y) in enumerate(ring)
    )
    arena = ArenaInput(start, obstacles)
    goal = Pose(25.0, 25.0, 0.0)
    assert is_pose_collision_free(goal, arena, config)

    result = HybridAStarPlanner(config).plan(start, goal, arena)
    assert result.status is LocalPlanningStatus.NO_PATH


def test_search_limit_returns_structured_status():
    config = replace(CONFIG, max_expanded_nodes=1)
    start = Pose(100.0, 100.0, 0.0)
    result = HybridAStarPlanner(config).plan(start, Pose(150.0, 100.0, 0.0), ArenaInput(start))

    assert result.status is LocalPlanningStatus.SEARCH_LIMIT_REACHED
    assert result.path is None
    assert result.metrics.nodes_expanded == 1


@pytest.mark.parametrize("invalid_limit", [0, -1, 1.5, True])
def test_search_limit_configuration_requires_positive_integer(invalid_limit):
    with pytest.raises(ValueError, match="max_expanded_nodes"):
        replace(CONFIG, max_expanded_nodes=invalid_limit)


def test_goal_position_and_heading_tolerances_are_configurable():
    start = Pose(100.0, 100.0, 0.0)
    position_goal = Pose(115.0, 100.0, 0.0)
    result = HybridAStarPlanner(CONFIG).plan(start, position_goal, ArenaInput(start))
    assert result.status is LocalPlanningStatus.SUCCESS
    assert result.path.final_pose == Pose(110.0, 100.0, 0.0)

    heading_goal = Pose(100.0, 100.0, 0.05)
    permissive = replace(CONFIG, goal_heading_tolerance_rad=0.1)
    strict = replace(CONFIG, goal_heading_tolerance_rad=0.01)
    assert goal_reached(start, heading_goal, permissive)
    assert not goal_reached(start, heading_goal, strict)
    zero_motion = HybridAStarPlanner(permissive).plan(start, heading_goal, ArenaInput(start))
    assert zero_motion.status is LocalPlanningStatus.SUCCESS
    assert zero_motion.metrics.command_count == 0


def test_search_key_discretizes_without_snapping_continuous_successor():
    first = Pose(100.1, 100.1, 0.01)
    second = Pose(102.4, 102.4, 0.02)
    assert first != second
    assert search_key(first, CONFIG) == search_key(second, CONFIG)

    successor = propagate_motion(Pose(80.0, 80.0, 0.0), primitive("FL"), CONFIG)
    assert successor.x_cm == pytest.approx(106.1)
    assert successor.x_cm % CONFIG.position_bin_cm != pytest.approx(0.0)


def test_configured_straight_distance_changes_planner_successor():
    primitives = tuple(
        replace(item, travel_cm=7.5)
        if item.steering is Steering.STRAIGHT
        else item
        for item in CONFIG.motion.primitives
    )
    config = replace(
        CONFIG,
        motion=replace(CONFIG.motion, primitives=primitives),
        goal_position_tolerance_cm=0.1,
        position_bin_cm=2.5,
    )
    start = Pose(100.0, 100.0, 0.0)
    result = HybridAStarPlanner(config).plan(start, Pose(107.5, 100.0, 0.0), ArenaInput(start))

    assert result.status is LocalPlanningStatus.SUCCESS
    assert [item.command for item in result.path.primitives] == ["FW"]
    assert result.path.primitives[0].travel_cm == 7.5
    assert result.path.final_pose == Pose(107.5, 100.0, 0.0)


def test_distance_and_provisional_time_objectives_are_reported_separately():
    start = Pose(100.0, 100.0, 0.0)
    goal = Pose(130.0, 100.0, 0.0)
    arena = ArenaInput(start)
    distance = HybridAStarPlanner(CONFIG).plan(start, goal, arena, objective=CostMetric.DISTANCE)
    estimated_time = HybridAStarPlanner(CONFIG).plan(
        start,
        goal,
        arena,
        objective=CostMetric.ESTIMATED_TIME,
    )

    assert distance.path.objective_cost == pytest.approx(distance.metrics.geometric_distance_cm)
    assert estimated_time.path.objective_cost == pytest.approx(estimated_time.metrics.estimated_time_s)
    assert distance.metrics.geometric_distance_cm == estimated_time.metrics.geometric_distance_cm
    assert estimated_time.metrics.estimated_time_s != estimated_time.metrics.geometric_distance_cm


def test_identical_queries_return_identical_paths_and_search_counts():
    start = Pose(80.0, 80.0, 0.0)
    goal = endpoint(start, ("FL", "FW", "FR"))
    arena = ArenaInput(start)
    planner = HybridAStarPlanner(CONFIG)
    first = planner.plan(start, goal, arena, objective=CostMetric.DISTANCE)
    second = planner.plan(start, goal, arena, objective=CostMetric.DISTANCE)

    assert first.status == second.status
    assert first.path.primitives == second.path.primitives
    assert first.path.key_poses == second.path.key_poses
    assert first.path.sampled_poses == second.path.sampled_poses
    assert first.metrics.nodes_expanded == second.metrics.nodes_expanded
    assert first.metrics.nodes_generated == second.metrics.nodes_generated
    assert first.metrics.collision_checks == second.metrics.collision_checks


def test_optional_debug_data_matches_search_metrics():
    start = Pose(100.0, 100.0, 0.0)
    arena = ArenaInput(start)
    planner = HybridAStarPlanner(CONFIG)
    collected = planner.plan(start, Pose(120.0, 100.0, 0.0), arena, collect_debug=True)
    omitted = planner.plan(start, Pose(120.0, 100.0, 0.0), arena, collect_debug=False)

    assert len(collected.debug.expanded_states) == collected.metrics.nodes_expanded
    assert len(collected.debug.generated_states) == collected.metrics.nodes_generated
    assert omitted.debug.expanded_states == ()
    assert omitted.debug.generated_states == ()


def test_hybrid_demo_plans_to_real_phase3_candidate_and_adapts_to_playback():
    scenario = build_hybrid_demo()
    result = scenario.planning_result

    assert result.status is LocalPlanningStatus.SUCCESS
    assert result.path is not None
    assert scenario.simulator.state.planned_path == result.path.sampled_poses
    assert scenario.simulator.state.executed_path == (result.start,)
    assert result.debug.expanded_states
    scenario.simulator.play()
    scenario.simulator.advance(100.0)
    assert scenario.simulator.state.robot_pose == result.path.final_pose


def test_hybrid_demo_renderer_smoke_with_debug_states(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    from algorithm.simulator.renderer import PygameRenderer, RenderOptions

    scenario = build_hybrid_demo()
    renderer = PygameRenderer(scenario.config, width_px=960, height_px=700)
    try:
        renderer.initialize()
        renderer.render(
            scenario.simulator.state,
            RenderOptions(show_debug_nodes=True),
            debug_nodes=scenario.planning_result.debug.expanded_states,
        )
        pygame.display.flip()
    finally:
        renderer.shutdown()
    assert not pygame.get_init()


def test_pathfinding_import_does_not_load_pygame():
    script = "import sys; import algorithm.pathfinding; assert 'pygame' not in sys.modules"
    subprocess.run([sys.executable, "-c", script], check=True)
