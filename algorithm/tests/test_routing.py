import math
import subprocess
import sys
from dataclasses import replace

import pytest

from algorithm.config import CameraGeometry, RobotGeometry, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.coordinates import default_start_pose, planner_pose_to_body_center
from algorithm.enums import CostMetric, Direction, Gear, PlanningStatus, RoutingMode, Steering
from algorithm.geometry import is_motion_collision_free, robot_footprint
from algorithm.models import ArenaInput, CaptureStep, GridCell, MoveStep, Obstacle, Pose
from algorithm.models.motion import MotionPrimitive, MotionSegment
from algorithm.models.planning import PathMetrics
from algorithm.pathfinding import (
    HybridAStarPlanner,
    HybridPath,
    LocalPlanningResult,
    LocalPlanningStatus,
    primitive_execution_time_s,
)
from algorithm.routing import (
    DirectedPairwiseGraph,
    DirectedPairwisePathCache,
    RouteEndpoint,
    Task1Planner,
)
from algorithm.simulator import PlaybackState
from algorithm.simulator.task1_demo import build_task1_demo
from algorithm.targets import ObservationCandidateKind, generate_arena_observation_candidates


def routing_config(*, candidates=1):
    return replace(
        UNCALIBRATED_SIMULATION_CONFIG,
        robot=RobotGeometry(2.0, 2.0),
        camera=CameraGeometry(0.0, 0.0, 10.0),
        observation_lateral_offsets_cm=(0.0, -10.0, 10.0)[:candidates],
        guaranteed_max_candidates_per_target=candidates,
    )


def arena_for(count, *, y=4):
    return ArenaInput(
        Pose.from_direction(20.0, 20.0, Direction.EAST),
        tuple(
            Obstacle(index + 1, GridCell(3 + index * 3, y), Direction.NORTH)
            for index in range(count)
        ),
    )


class FakePathPlanner:
    def __init__(self, costs=None, failures=None):
        self.costs = costs or {}
        self.failures = failures or {}
        self.calls = []

    def plan(self, start, goal, arena, *, objective=CostMetric.ESTIMATED_TIME, collect_debug=False):
        self.calls.append((start, goal, objective))
        status = self.failures.get((start, goal, objective), self.failures.get((start, goal)))
        if status is not None:
            return LocalPlanningResult(
                status,
                start,
                goal,
                metrics=PathMetrics(nodes_expanded=1, nodes_generated=1, collision_checks=2),
                message="configured fake failure",
            )

        distance, estimated_time = self.costs.get(
            (start, goal),
            (math.hypot(goal.x_cm - start.x_cm, goal.y_cm - start.y_cm), 1.0),
        )
        if start == goal:
            distance = estimated_time = 0.0
        objective_cost = distance if objective is CostMetric.DISTANCE else estimated_time
        metrics = PathMetrics(
            geometric_distance_cm=distance,
            estimated_time_s=estimated_time,
            forward_distance_cm=distance,
            command_count=0 if start == goal else 1,
            nodes_expanded=1,
            nodes_generated=1,
            collision_checks=2,
            planning_time_s=0.001,
        )
        if start == goal:
            segments = ()
            sampled = (start,)
            cumulative = (0.0,)
        else:
            primitive = MotionPrimitive(
                "FW",
                Gear.FORWARD,
                Steering.STRAIGHT,
                travel_cm=max(distance, 0.001),
            )
            segments = (MotionSegment(primitive, start, goal),)
            sampled = (start, goal)
            cumulative = (0.0, objective_cost)
        path = HybridPath(
            start,
            goal,
            goal,
            segments,
            sampled,
            metrics,
            objective,
            objective_cost,
            cumulative,
        )
        return LocalPlanningResult(LocalPlanningStatus.SUCCESS, start, goal, path, metrics)


class BudgetRecoveringPlanner(FakePathPlanner):
    def plan(
        self,
        start,
        goal,
        arena,
        *,
        objective=CostMetric.ESTIMATED_TIME,
        collect_debug=False,
        max_expanded_nodes=None,
        max_planning_time_s=None,
    ):
        self.calls.append((start, goal, objective, max_expanded_nodes))
        if max_expanded_nodes < 100:
            return LocalPlanningResult(
                LocalPlanningStatus.SEARCH_LIMIT_REACHED,
                start,
                goal,
                metrics=PathMetrics(nodes_expanded=max_expanded_nodes),
                message="test budget exhausted",
            )
        self.calls.pop()
        return super().plan(start, goal, arena, objective=objective)


def endpoints(arena, config):
    groups = generate_arena_observation_candidates(arena, config)
    return RouteEndpoint.start(arena.start_pose), tuple(
        tuple(RouteEndpoint.candidate(candidate) for candidate in group.candidates if candidate.valid)
        for group in groups
    )


def test_actual_hybrid_pairwise_start_and_candidate_queries_are_directed():
    config = UNCALIBRATED_SIMULATION_CONFIG
    arena = ArenaInput(Pose(80.0, 80.0, 0.0))
    start = RouteEndpoint.start(arena.start_pose)
    first = RouteEndpoint(
        kind=start.kind.CANDIDATE,
        pose=Pose(90.0, 80.0, 0.0),
        obstacle_id=1,
        candidate_index=0,
        candidate_kind=ObservationCandidateKind.NOMINAL,
        nominal=True,
    )
    second = replace(first, pose=Pose(100.0, 80.0, 0.0), obstacle_id=2)
    cache = DirectedPairwisePathCache(HybridAStarPlanner(config))

    start_path = cache.get_or_plan(start, first, arena, config, CostMetric.DISTANCE)
    candidate_path = cache.get_or_plan(first, second, arena, config, CostMetric.DISTANCE)
    reverse_path = cache.get_or_plan(second, first, arena, config, CostMetric.DISTANCE)

    assert start_path.succeeded and candidate_path.succeeded and reverse_path.succeeded
    assert candidate_path.key.start == first
    assert reverse_path.key.start == second
    assert cache.size == 3


def test_cache_hit_prevents_replanning_and_identity_includes_direction_candidate_config_and_objective():
    config = routing_config(candidates=2)
    arena = arena_for(2)
    start, groups = endpoints(arena, config)
    first, first_fallback = groups[0]
    second = groups[1][0]
    fake = FakePathPlanner()
    cache = DirectedPairwisePathCache(fake)

    original = cache.get_or_plan(first, second, arena, config, CostMetric.DISTANCE)
    assert cache.get_or_plan(first, second, arena, config, CostMetric.DISTANCE) is original
    cache.get_or_plan(second, first, arena, config, CostMetric.DISTANCE)
    cache.get_or_plan(first_fallback, second, arena, config, CostMetric.DISTANCE)
    cache.get_or_plan(first, second, arena, config, CostMetric.ESTIMATED_TIME)
    changed_config = replace(config, position_bin_cm=2.5)
    cache.get_or_plan(first, second, arena, changed_config, CostMetric.DISTANCE)

    assert len(fake.calls) == 5
    assert cache.stats.requests == 6
    assert cache.stats.hits == 1
    assert cache.stats.misses == 5
    assert start != first


def test_structured_unreachable_pair_is_cached():
    config = routing_config()
    arena = arena_for(1)
    start, groups = endpoints(arena, config)
    goal = groups[0][0]
    fake = FakePathPlanner(failures={(start.pose, goal.pose): LocalPlanningStatus.NO_PATH})
    cache = DirectedPairwisePathCache(fake)

    first = cache.get_or_plan(start, goal, arena, config, CostMetric.DISTANCE)
    second = cache.get_or_plan(start, goal, arena, config, CostMetric.DISTANCE)

    assert first is second
    assert first.result.status is LocalPlanningStatus.NO_PATH
    assert first.selected_cost is None
    assert len(fake.calls) == 1


def test_graph_resolves_local_paths_lazily_and_cache_reuses_the_result():
    config = routing_config()
    arena = arena_for(1)
    start, groups = endpoints(arena, config)
    goal = groups[0][0]
    fake = FakePathPlanner()
    cache = DirectedPairwisePathCache(fake)
    graph = DirectedPairwiseGraph(arena, config, CostMetric.DISTANCE, start, groups, {}, cache)

    assert fake.calls == [] and graph.entries == {}
    first = graph.entry(start, goal)
    second = graph.entry(start, goal)
    assert first is second
    assert len(fake.calls) == 1
    assert len(graph.entries) == 1


def test_search_limit_is_retried_with_larger_budget_then_success_is_final():
    config = replace(
        routing_config(),
        adaptive_initial_expansions=20,
        adaptive_max_expansions=100,
        adaptive_growth_factor=5.0,
    )
    arena = arena_for(1)
    start, groups = endpoints(arena, config)
    goal = groups[0][0]
    fake = BudgetRecoveringPlanner()
    cache = DirectedPairwisePathCache(fake)

    recovered = cache.get_or_plan(start, goal, arena, config, CostMetric.DISTANCE)
    repeated = cache.get_or_plan(start, goal, arena, config, CostMetric.DISTANCE)

    assert recovered is repeated and recovered.succeeded
    assert recovered.attempts == 2
    assert recovered.expansion_budget == 100
    assert cache.stats.retries == cache.stats.retry_recoveries == 1
    assert [call[3] for call in fake.calls if len(call) == 4] == [20]


def test_feasibility_mode_expands_candidate_tier_when_preferred_pose_fails():
    config = routing_config(candidates=3)
    arena = arena_for(1)
    start, groups = endpoints(arena, config)
    nominal, left, right = groups[0]
    fake = FakePathPlanner(
        costs={(start.pose, left.pose): (2.0, 2.0), (start.pose, right.pose): (3.0, 3.0)},
        failures={(start.pose, nominal.pose): LocalPlanningStatus.NO_PATH},
    )
    result = Task1Planner(
        config,
        path_planner=fake,
        routing_mode=RoutingMode.FEASIBILITY,
    ).plan(arena, objective=CostMetric.DISTANCE)

    assert result.status is PlanningStatus.SUCCESS
    assert result.route.selected_candidate_kinds == (left.candidate_label,)
    assert result.metrics.candidate_tiers_activated == 2
    assert result.metrics.candidate_count_considered == 3
    assert result.metrics.local_paths_requested == 4
    assert result.metrics.pairwise_cache_misses == 3
    assert result.metrics.cache_hits == 1


def test_exact_two_target_order_composes_cached_paths_captures_and_no_return():
    config = routing_config()
    arena = arena_for(2)
    start, groups = endpoints(arena, config)
    first, second = groups[0][0], groups[1][0]
    costs = {
        (start.pose, first.pose): (10.0, 10.0),
        (start.pose, second.pose): (1.0, 1.0),
        (first.pose, second.pose): (10.0, 10.0),
        (second.pose, first.pose): (1.0, 1.0),
    }
    fake = FakePathPlanner(costs)
    planner = Task1Planner(config, path_planner=fake)

    result = planner.plan(arena, objective=CostMetric.DISTANCE)

    assert result.status is PlanningStatus.SUCCESS
    assert result.route.target_order == (2, 1)
    assert result.route.objective_cost == pytest.approx(2.0)
    assert result.route.sampled_poses == (start.pose, second.pose, first.pose)
    assert result.route.sampled_poses[-1] != arena.start_pose
    assert len(result.route.primitives) == 2
    assert result.route.primitives == tuple(
        segment.primitive
        for local_path in result.route.local_paths
        for segment in local_path.segments
    )
    assert tuple(
        step.segment.primitive
        for step in result.route.execution_steps
        if isinstance(step, MoveStep)
    ) == result.route.primitives
    captures = [step for step in result.route.execution_steps if isinstance(step, CaptureStep)]
    assert [step.obstacle_id for step in captures] == [2, 1]
    assert [step.pose for step in captures] == [second.pose, first.pose]
    assert result.metrics.local_paths_requested == 4
    assert result.metrics.pairwise_cache_misses == 4
    assert result.metrics.permutations_evaluated == 2

    repeated = planner.plan(arena, objective=CostMetric.DISTANCE)
    assert repeated.route.target_order == result.route.target_order
    assert repeated.metrics.cache_hits == 4
    assert repeated.metrics.pairwise_cache_misses == 0
    assert len(fake.calls) == 4


def test_three_target_exact_order_is_deterministic():
    config = routing_config()
    arena = arena_for(3)
    start, groups = endpoints(arena, config)
    a, b, c = (group[0] for group in groups)
    all_nodes = (a, b, c)
    costs = {(start.pose, node.pose): (20.0, 20.0) for node in all_nodes}
    costs.update({(source.pose, goal.pose): (20.0, 20.0) for source in all_nodes for goal in all_nodes if source != goal})
    costs[(start.pose, c.pose)] = (1.0, 1.0)
    costs[(c.pose, a.pose)] = (1.0, 1.0)
    costs[(a.pose, b.pose)] = (1.0, 1.0)
    planner = Task1Planner(config, path_planner=FakePathPlanner(costs))

    first = planner.plan(arena, objective=CostMetric.DISTANCE)
    second = planner.plan(arena, objective=CostMetric.DISTANCE)

    assert first.route.target_order == (3, 1, 2)
    assert second.route.target_order == first.route.target_order
    assert first.metrics.permutations_evaluated == math.factorial(3)


def test_candidate_layer_dp_chooses_cheaper_reachable_fallback_over_nominal():
    config = routing_config(candidates=3)
    arena = arena_for(1)
    start, groups = endpoints(arena, config)
    nominal, left, right = groups[0]
    fake = FakePathPlanner(
        {
            (start.pose, nominal.pose): (10.0, 10.0),
            (start.pose, left.pose): (2.0, 2.0),
            (start.pose, right.pose): (3.0, 3.0),
        }
    )

    result = Task1Planner(config, path_planner=fake).plan(arena, objective=CostMetric.DISTANCE)

    assert result.status is PlanningStatus.SUCCESS
    assert result.route.observation_poses[0].candidate_index == left.candidate_index
    assert result.route.selected_candidate_kinds == ("10L",)
    assert result.route.objective_cost == 2.0


def test_one_reachable_fallback_keeps_target_routable_and_geometric_validity_unchanged():
    config = routing_config(candidates=3)
    arena = arena_for(1)
    groups_before = generate_arena_observation_candidates(arena, config)
    start, groups = endpoints(arena, config)
    nominal, left, right = groups[0]
    failures = {
        (start.pose, nominal.pose): LocalPlanningStatus.NO_PATH,
        (start.pose, right.pose): LocalPlanningStatus.SEARCH_LIMIT_REACHED,
    }
    result = Task1Planner(config, path_planner=FakePathPlanner(failures=failures)).plan(
        arena, objective=CostMetric.DISTANCE
    )

    assert result.status is PlanningStatus.SUCCESS
    assert result.route.observation_poses == (left.observation_pose,)
    assert all(candidate.valid for candidate in groups_before[0].candidates)
    assert result.metrics.target_reachability[0].geometric_candidates == 3
    assert result.metrics.target_reachability[0].reachable_candidates == 1


def test_candidate_dp_recovers_from_unreachable_inter_target_transition():
    config = routing_config(candidates=2)
    arena = arena_for(2)
    start, groups = endpoints(arena, config)
    a_nominal, a_left = groups[0]
    b_nominal, b_left = groups[1]
    failures = {
        (start.pose, b_nominal.pose): LocalPlanningStatus.NO_PATH,
        (start.pose, b_left.pose): LocalPlanningStatus.NO_PATH,
        (a_nominal.pose, b_nominal.pose): LocalPlanningStatus.NO_PATH,
        (a_nominal.pose, b_left.pose): LocalPlanningStatus.NO_PATH,
        (b_nominal.pose, a_nominal.pose): LocalPlanningStatus.NO_PATH,
        (b_nominal.pose, a_left.pose): LocalPlanningStatus.NO_PATH,
        (b_left.pose, a_nominal.pose): LocalPlanningStatus.NO_PATH,
        (b_left.pose, a_left.pose): LocalPlanningStatus.NO_PATH,
    }
    result = Task1Planner(config, path_planner=FakePathPlanner(failures=failures)).plan(
        arena, objective=CostMetric.ESTIMATED_TIME
    )

    assert result.status is PlanningStatus.SUCCESS
    assert result.route.target_order == (1, 2)
    assert result.route.observation_poses[0].candidate_index == a_left.candidate_index


def test_target_with_no_reachable_candidates_is_structured_failure():
    config = routing_config(candidates=3)
    arena = arena_for(1)
    start, groups = endpoints(arena, config)
    failures = {
        (start.pose, endpoint.pose): LocalPlanningStatus.NO_PATH
        for endpoint in groups[0]
    }
    result = Task1Planner(config, path_planner=FakePathPlanner(failures=failures)).plan(arena)

    assert result.status is PlanningStatus.NO_FEASIBLE_ROUTE
    assert result.route is None
    assert result.metrics.targets_routed == 0
    assert {issue.code for issue in result.issues} == {
        "no_reachable_observation_pose",
        "no_complete_task1_route",
    }


def test_search_limit_failure_is_preserved_in_global_diagnostics():
    config = routing_config()
    arena = arena_for(1)
    start, groups = endpoints(arena, config)
    fake = FakePathPlanner(
        failures={(start.pose, groups[0][0].pose): LocalPlanningStatus.SEARCH_LIMIT_REACHED}
    )
    result = Task1Planner(config, path_planner=fake).plan(arena)
    assert "pairwise_search_limit_reached" in {issue.code for issue in result.issues}


def test_distance_and_time_objectives_can_choose_different_orders():
    config = routing_config()
    arena = arena_for(2)
    start, groups = endpoints(arena, config)
    a, b = groups[0][0], groups[1][0]
    costs = {
        (start.pose, a.pose): (1.0, 10.0),
        (start.pose, b.pose): (5.0, 1.0),
        (a.pose, b.pose): (1.0, 1.0),
        (b.pose, a.pose): (5.0, 1.0),
    }
    planner = Task1Planner(config, path_planner=FakePathPlanner(costs))

    distance = planner.plan(arena, objective=CostMetric.DISTANCE)
    estimated_time = planner.plan(arena, objective=CostMetric.ESTIMATED_TIME)

    assert distance.route.target_order == (1, 2)
    assert estimated_time.route.target_order == (2, 1)
    assert distance.route.objective_cost == 2.0
    assert estimated_time.route.objective_cost == 2.0
    assert distance.route.metrics.estimated_time_s == 11.0
    assert estimated_time.route.metrics.geometric_distance_cm == 10.0


def test_exact_cost_is_no_worse_than_nearest_neighbour_baseline():
    config = routing_config()
    arena = arena_for(2)
    start, groups = endpoints(arena, config)
    a, b = groups[0][0], groups[1][0]
    costs = {
        (start.pose, a.pose): (1.0, 1.0),
        (start.pose, b.pose): (2.0, 2.0),
        (a.pose, b.pose): (100.0, 100.0),
        (b.pose, a.pose): (2.0, 2.0),
    }
    result = Task1Planner(config, path_planner=FakePathPlanner(costs)).plan(
        arena, objective=CostMetric.DISTANCE
    )

    assert result.route.target_order == (2, 1)
    assert result.route.objective_cost == 4.0
    assert result.metrics.nearest_neighbour_route_cost == 101.0
    assert result.route.objective_cost <= result.metrics.nearest_neighbour_route_cost


def test_equal_cost_tie_prefers_nominal_then_stable_target_order():
    config = routing_config(candidates=2)
    arena = arena_for(2)
    result = Task1Planner(config, path_planner=FakePathPlanner()).plan(
        arena, objective=CostMetric.ESTIMATED_TIME
    )

    assert result.route.target_order == (1, 2)
    assert tuple(item.candidate_index for item in result.route.observation_poses) == (0, 0)


def test_zero_targets_returns_success_without_pairwise_searches():
    config = routing_config()
    arena = ArenaInput(Pose(20.0, 20.0, 0.0))
    fake = FakePathPlanner()
    result = Task1Planner(config, path_planner=fake).plan(arena)

    assert result.status is PlanningStatus.SUCCESS
    assert result.route.target_order == ()
    assert result.route.sampled_poses == (arena.start_pose,)
    assert result.metrics.local_paths_requested == 0
    assert fake.calls == []


def test_missing_face_and_geometric_candidate_failure_stop_before_pairwise_planning():
    config = routing_config()
    missing = ArenaInput(Pose(20.0, 20.0, 0.0), (Obstacle(1, GridCell(4, 4)),))
    fake = FakePathPlanner()
    invalid = Task1Planner(config, path_planner=fake).plan(missing)
    assert invalid.status is PlanningStatus.INVALID_INPUT
    assert fake.calls == []

    boundary = ArenaInput(
        Pose(20.0, 20.0, 0.0),
        (Obstacle(1, GridCell(4, 19), Direction.NORTH),),
    )
    no_route = Task1Planner(config, path_planner=fake).plan(boundary)
    assert no_route.status is PlanningStatus.NO_FEASIBLE_ROUTE
    assert no_route.issues[0].code == "no_geometrically_valid_observation_pose"
    assert fake.calls == []


def test_eight_targets_and_three_candidates_complete_with_exact_optimizer():
    config = routing_config(candidates=3)
    arena = ArenaInput(
        Pose(20.0, 20.0, 0.0),
        tuple(
            Obstacle(index + 1, GridCell(2 + index * 2, 8), Direction.NORTH)
            for index in range(8)
        ),
    )
    result = Task1Planner(config, path_planner=FakePathPlanner()).plan(
        arena, objective=CostMetric.ESTIMATED_TIME
    )

    assert result.status is PlanningStatus.SUCCESS
    assert len(result.route.target_order) == 8
    assert len(set(result.route.target_order)) == 8
    assert len(result.route.observation_poses) == 8
    assert result.metrics.permutations_evaluated == math.factorial(8)
    assert result.metrics.local_paths_requested == 24 + 8 * 7 * 9


def test_routing_core_import_does_not_load_pygame():
    script = "import sys; import algorithm.routing; assert 'pygame' not in sys.modules"
    subprocess.run([sys.executable, "-c", script], check=True)


@pytest.fixture(scope="module")
def task1_demo_scenario():
    return build_task1_demo()


def test_five_target_demo_uses_real_cached_paths_and_complete_capture_sequence(task1_demo_scenario):
    scenario = task1_demo_scenario
    result = scenario.planning_result
    route = result.route

    assert result.status is PlanningStatus.SUCCESS
    assert route is not None
    assert len(route.target_order) == 5
    assert route.target_order == (1, 2, 3, 4, 5)
    assert route.selected_candidate_kinds == ("20C",) * 5
    assert result.metrics.local_paths_requested == 23
    assert result.metrics.pairwise_cache_misses == 23
    assert result.metrics.local_paths_succeeded == 11
    assert result.metrics.permutations_evaluated == math.factorial(5)
    assert (
        result.metrics.optimized_candidate_chain_cost
        <= result.metrics.nearest_neighbour_route_cost
    )
    assert result.metrics.selected_route_cost == pytest.approx(route.objective_cost)
    assert all(
        3 <= coordinate <= 16
        for obstacle in scenario.simulator.state.arena.obstacles
        for coordinate in (obstacle.cell.x, obstacle.cell.y)
    )

    captures = [step for step in route.execution_steps if isinstance(step, CaptureStep)]
    assert [step.obstacle_id for step in captures] == list(route.target_order)
    assert len(captures) == len(set(step.obstacle_id for step in captures)) == 5
    for observation, local_path, capture in zip(route.observation_poses, route.local_paths, captures):
        assert capture.pose == local_path.reached_pose
        assert capture.pose.heading_rad == observation.pose.heading_rad
        assert math.hypot(
            capture.pose.x_cm - observation.pose.x_cm,
            capture.pose.y_cm - observation.pose.y_cm,
        ) <= scenario.config.goal_position_tolerance_cm
        assert all(
            is_motion_collision_free(
                segment.start,
                segment.primitive,
                scenario.simulator.state.arena,
                scenario.config,
            )
            for segment in local_path.segments
        )

    assert route.sampled_poses[0] == scenario.simulator.state.arena.start_pose
    assert route.sampled_poses[-1] != scenario.simulator.state.arena.start_pose
    assert scenario.simulator.state.target_order == route.target_order
    assert scenario.simulator.state.selected_candidates == tuple(
        zip(route.target_order, route.selected_candidate_kinds)
    )


def test_task1_demo_has_one_authoritative_start_pose_inside_start_zone(task1_demo_scenario):
    scenario = task1_demo_scenario
    route = scenario.planning_result.route
    expected = default_start_pose(scenario.config.robot)

    assert expected == Pose.from_direction(15.0, 15.0, Direction.NORTH)
    assert route.start == expected
    assert route.sampled_poses[0] == expected
    assert route.local_paths[0].start == expected
    assert route.local_paths[0].segments[0].start == expected
    assert scenario.simulator.state.initial_pose == expected
    assert scenario.simulator.state.robot_pose == expected
    assert planner_pose_to_body_center(expected, scenario.config.robot) == expected
    assert all(
        0.0 <= coordinate <= 30.0
        for point in robot_footprint(expected, scenario.config.robot)
        for coordinate in (point.x_cm, point.y_cm)
    )


def test_task1_demo_pairwise_and_motion_boundaries_are_physically_continuous(task1_demo_scenario):
    route = task1_demo_scenario.planning_result.route
    for previous, following in zip(route.local_paths, route.local_paths[1:]):
        assert following.start.x_cm == pytest.approx(previous.reached_pose.x_cm, abs=1e-9)
        assert following.start.y_cm == pytest.approx(previous.reached_pose.y_cm, abs=1e-9)
        assert following.start.heading_rad == pytest.approx(previous.reached_pose.heading_rad, abs=1e-9)

    segments = tuple(segment for path in route.local_paths for segment in path.segments)
    for previous, following in zip(segments, segments[1:]):
        assert following.start.x_cm == pytest.approx(previous.end.x_cm, abs=1e-9)
        assert following.start.y_cm == pytest.approx(previous.end.y_cm, abs=1e-9)
        assert following.start.heading_rad == pytest.approx(previous.end.heading_rad, abs=1e-9)


def test_task1_demo_direction_changes_are_counted_without_redundant_inverse_straights(
    task1_demo_scenario,
):
    route = task1_demo_scenario.planning_result.route
    primitives = route.primitives
    expected_changes = sum(
        first.gear is not second.gear
        for first, second in zip(primitives, primitives[1:])
    )
    assert route.metrics.direction_changes == expected_changes == 8
    assert not any(
        {first.command, second.command} == {"FW", "BW"}
        for first, second in zip(primitives, primitives[1:])
    )


def test_provisional_profile_costs_both_gears_turns_and_command_overhead_explicitly():
    motion = UNCALIBRATED_SIMULATION_CONFIG.motion
    forward = motion.primitives_for("FW")[0]
    reverse = motion.primitives_for("BW")[0]
    assert motion.serial_overhead_s == pytest.approx(0.05)
    assert motion.direction_change_penalty_s == 0.0
    assert motion.steering_change_penalty_s == 0.0
    assert primitive_execution_time_s(forward, motion) == pytest.approx(
        motion.straight_fixed_time_s
        + (forward.travel_cm - motion.straight_deceleration_cm) / motion.straight_speed_cm_s
        + motion.straight_settle_s
        + motion.serial_overhead_s
    )
    assert primitive_execution_time_s(reverse, motion) == pytest.approx(
        primitive_execution_time_s(forward, motion)
    )
    for command in ("FL", "FR", "BL", "BR"):
        primitive = motion.primitives_for(command)[0]
        assert primitive_execution_time_s(primitive, motion) == pytest.approx(
            primitive.estimated_duration_s + motion.serial_overhead_s
        )


def test_task1_demo_is_deterministic(task1_demo_scenario):
    repeated = build_task1_demo()
    first_route = task1_demo_scenario.planning_result.route
    repeated_route = repeated.planning_result.route

    assert repeated_route.target_order == first_route.target_order
    assert repeated_route.observation_poses == first_route.observation_poses
    assert repeated_route.primitives == first_route.primitives
    assert repeated_route.sampled_poses == first_route.sampled_poses
    assert repeated_route.objective_cost == pytest.approx(first_route.objective_cost)


def test_five_target_demo_playback_finishes_with_every_target_visited_once(task1_demo_scenario):
    simulator = task1_demo_scenario.simulator
    simulator.reset()
    assert simulator.state.visited_target_ids == ()
    simulator.play()
    simulator.advance(10_000.0)

    assert simulator.state.playback_state is PlaybackState.COMPLETE
    assert simulator.state.visited_target_ids == task1_demo_scenario.planning_result.route.target_order
    assert len(set(simulator.state.visited_target_ids)) == 5


def test_task1_capture_does_not_reset_robot_pose(task1_demo_scenario):
    simulator = task1_demo_scenario.simulator
    simulator.reset()
    assert simulator.step_primitive()
    reached_pose = simulator.state.robot_pose
    assert simulator.state.visited_target_ids == ()

    assert simulator.step_primitive()
    assert simulator.state.visited_target_ids == (1,)
    assert simulator.state.robot_pose == reached_pose


def test_task1_demo_renderer_smoke(task1_demo_scenario, monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    from algorithm.simulator.renderer import PygameRenderer, RenderOptions

    renderer = PygameRenderer(task1_demo_scenario.config, width_px=960, height_px=700)
    try:
        renderer.initialize()
        renderer.render(task1_demo_scenario.simulator.state, RenderOptions())
        pygame.display.flip()
    finally:
        renderer.shutdown()
    assert not pygame.get_init()
