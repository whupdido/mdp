from __future__ import annotations
import logging
import math
from dataclasses import dataclass, replace

from .task2_editor_model import (
    FastestCarSetup,
    random_fastest_car_setup,
)
from algorithm.config import FASTEST_CAR_CONFIG, PlanningConfig
from algorithm.enums import CostMetric, Direction, PlanningStatus, Steering
from algorithm.models import (
    ArenaInput,
    RectangleObstacle,
    PlanningIssue,
    PlanningResult,
    Pose,
)
from algorithm.models.wall import Wall
from algorithm.routing import Task1Planner
from algorithm.targets import generate_arena_observation_candidates, ObservationCandidateKind
from algorithm.models.motion import MoveStep
from algorithm.routing.models import RouteEndpoint, RouteOptimizationSolution
from algorithm.targets.geometry import (
    desired_camera_position,
    rear_axle_pose_for_camera,
)
from .headless import HeadlessSimulator, simulation_steps_from_execution

# Setup Logger
logger = logging.getLogger("FastestCarPlanner")
logging.basicConfig(level=logging.INFO)


@dataclass(frozen=True, slots=True)
class FastestCarScenario:
    simulator: HeadlessSimulator
    config: PlanningConfig
    planning_result: PlanningResult
    setup: FastestCarSetup


def fastest_car_demo_arena(
    setup: FastestCarSetup | None = None,
    config: PlanningConfig = FASTEST_CAR_CONFIG,
) -> ArenaInput:
    """Build a Fastest Car arena from a geometry setup."""

    if setup is None:
        setup = FastestCarSetup()

    top_clearance_cm = setup.top_clearance_cm
    bottom_clearance_cm = setup.bottom_clearance_cm
    carpark_to_obstacle_1_cm = setup.carpark_to_obstacle_1_cm
    obstacle_1_to_obstacle_2_cm = setup.obstacle_1_to_obstacle_2_cm
    carpark_width_cm = setup.carpark_width_cm
    obstacle_width_cm = setup.obstacle_width_cm
    obstacle_height_cm = setup.obstacle_height_cm
    arena_height_cm = setup.arena_height_cm
    center_y = arena_height_cm / 2.0

    robot_start = Pose(
        x_cm=setup.start_x_cm,
        y_cm=center_y,
        heading_rad=0.0,
    )

    obstacle_1_min_x = carpark_width_cm + carpark_to_obstacle_1_cm
    obstacle_1_max_x = obstacle_1_min_x + obstacle_width_cm

    obstacle_2_min_x = obstacle_1_max_x + obstacle_1_to_obstacle_2_cm
    obstacle_2_max_x = obstacle_2_min_x + obstacle_width_cm

    obstacle_min_y = center_y - obstacle_height_cm / 2.0
    obstacle_max_y = center_y + obstacle_height_cm / 2.0

    obstacles = (
        RectangleObstacle(
            obstacle_id=1,
            min_x_cm=obstacle_1_min_x,
            min_y_cm=obstacle_min_y,
            max_x_cm=obstacle_1_max_x,
            max_y_cm=obstacle_max_y,
            face=Direction.WEST,
        ),
        RectangleObstacle(
            obstacle_id=2,
            min_x_cm=obstacle_2_min_x,
            min_y_cm=obstacle_min_y,
            max_x_cm=obstacle_2_max_x,
            max_y_cm=obstacle_max_y,
            face=Direction.WEST,
        ),
    )

    walls = (
        Wall(
            wall_id="arena_bottom",
            min_x_cm=carpark_width_cm,
            min_y_cm=obstacle_min_y - bottom_clearance_cm - setup.wall_width_cm,
            max_x_cm=obstacle_2_max_x,
            max_y_cm=obstacle_min_y - bottom_clearance_cm,
        ),
        Wall(
            wall_id="arena_top",
            min_x_cm=carpark_width_cm,
            min_y_cm=obstacle_max_y + top_clearance_cm,
            max_x_cm=obstacle_2_max_x,
            max_y_cm=obstacle_max_y + top_clearance_cm + setup.wall_width_cm,
        ),
    )

    return ArenaInput(
        robot_start,
        obstacles,
        setup.arena_width_cm,
        arena_height_cm,
        walls=walls,
    )


def build_fastest_car_demo(
    setup: FastestCarSetup | None = None,
) -> FastestCarScenario:
    config = FASTEST_CAR_CONFIG
    if setup is None:
        setup = FastestCarSetup()
    arena = fastest_car_demo_arena(setup, config)

    planner = Task1Planner(config)

    candidates = generate_arena_observation_candidates(
        arena,
        config,
    )

    if len(candidates) != 2:
        raise RuntimeError(
            f"Fastest Car expects 2 obstacles, got {len(candidates)}"
        )

    obstacle_1_candidates = tuple(
        RouteEndpoint.candidate(candidate)
        for candidate in candidates[0].candidates
        if candidate.valid
    )

    obstacle_2_candidates = tuple(
        RouteEndpoint.candidate(candidate)
        for candidate in candidates[1].candidates
        if candidate.valid
    )

    if not obstacle_1_candidates or not obstacle_2_candidates:
        raise RuntimeError(
            "Missing valid observation candidates"
        )

    start = RouteEndpoint.start(arena.start_pose)

    best_solution = None
    best_cost = None

    logger.info(f"Starting search across {len(obstacle_1_candidates)} O1 candidates and {len(obstacle_2_candidates)} O2 candidates...")

    for i, obstacle_1 in enumerate(obstacle_1_candidates):

        first_leg = planner.pairwise_cache.get_or_plan(
            start,
            obstacle_1,
            arena,
            config,
            CostMetric.ESTIMATED_TIME,
        )

        if not first_leg.succeeded or first_leg.result.path is None:
            logger.warning(f"[O1 Candidate {i}] Leg 1 (Start -> O1) FAILED")
            continue
        
        logger.info(f"[O1 Candidate {i}] Leg 1 (Start -> O1) SUCCEEDED | Cost: {first_leg.selected_cost:.2f}")

        reached_obstacle_1 = RouteEndpoint(
            obstacle_1.kind,
            first_leg.result.path.final_pose,
            obstacle_id=obstacle_1.obstacle_id,
            candidate_index=obstacle_1.candidate_index,
            candidate_kind=obstacle_1.candidate_kind,
            nominal=obstacle_1.nominal,
            standoff_cm=obstacle_1.standoff_cm,
            lateral_class=obstacle_1.lateral_class,
            preference_rank=obstacle_1.preference_rank,
            candidate_label=obstacle_1.candidate_label,
        )

        for j, obstacle_2 in enumerate(obstacle_2_candidates):

            second_leg = planner.pairwise_cache.get_or_plan(
                reached_obstacle_1,
                obstacle_2,
                arena,
                config,
                CostMetric.ESTIMATED_TIME,
                required_first_steering=setup.obstacle_1_direction,
            )

            if not second_leg.succeeded or second_leg.result.path is None:
                logger.warning(f"  [O2 Candidate {j}] Leg 2 (O1 -> O2) FAILED")
                continue

            logger.info(f"  [O2 Candidate {j}] Leg 2 (O1 -> O2) SUCCEEDED | Cost: {second_leg.selected_cost:.2f}")

            reached_obstacle_2 = RouteEndpoint(
                obstacle_2.kind,
                second_leg.result.path.final_pose,
                obstacle_id=obstacle_2.obstacle_id,
                candidate_index=obstacle_2.candidate_index,
                candidate_kind=obstacle_2.candidate_kind,
                nominal=obstacle_2.nominal,
                standoff_cm=obstacle_2.standoff_cm,
                lateral_class=obstacle_2.lateral_class,
                preference_rank=obstacle_2.preference_rank,
                candidate_label=obstacle_2.candidate_label,
            )

            obstacle_2_geometry = next(
                obstacle
                for obstacle in arena.obstacles
                if isinstance(obstacle, RectangleObstacle)
                and obstacle.obstacle_id == obstacle_2.obstacle_id
            )

            collision_half_length = config.robot.collision_length_cm / 2.0
            corner_offset_x = collision_half_length + setup.loop_clearance_cm
            corner_offset_y = setup.loop_clearance_cm
            exit_lane_offset_y = setup.loop_exit_lane_offset_cm

            is_right_turn = setup.obstacle_2_direction is Steering.RIGHT

            # 1. Turning around Obstacle 2 to enter loop
            loop_x = obstacle_2_geometry.max_x_cm + corner_offset_x
            if is_right_turn:
                loop_y = obstacle_2_geometry.min_y_cm - corner_offset_y
                loop_heading = math.pi / 2.0  # Facing North
                exit_y = obstacle_2_geometry.max_y_cm + exit_lane_offset_y
            else:
                loop_y = obstacle_2_geometry.max_y_cm + corner_offset_y
                loop_heading = -math.pi / 2.0  # Facing South
                exit_y = obstacle_2_geometry.min_y_cm - exit_lane_offset_y

            loop_pose = Pose(
                x_cm=loop_x,
                y_cm=loop_y,
                heading_rad=loop_heading,
            )

            # 2. Waypoint to exit out of Obstacle 2
            exit_x = obstacle_2_geometry.min_x_cm - setup.loop_exit_x_offset_cm
            exit_pose = Pose(
                x_cm=exit_x,
                y_cm=exit_y,
                heading_rad=math.pi,  # Facing West
            )

            # 3. Final Carpark Return Pose (Facing West into the Carpark)
            return_pose = Pose(
                x_cm=arena.start_pose.x_cm,
                y_cm=arena.start_pose.y_cm,
                heading_rad=math.pi,
            )

            # Leg 3A: O2 -> Rear Corner Apex Waypoint
            third_leg_a = planner.path_planner.plan(
                reached_obstacle_2.pose,
                loop_pose,
                arena,
                objective=CostMetric.ESTIMATED_TIME,
            )

            if not third_leg_a.succeeded or third_leg_a.path is None:
                logger.warning(f"  [O2 Candidate {j}] Leg 3A (O2 -> Corner) FAILED")
                continue

            # Leg 3B: Rear Corner Apex -> Return Lane Alignment
            third_leg_b = planner.path_planner.plan(
                third_leg_a.path.final_pose,
                exit_pose,
                arena,
                objective=CostMetric.ESTIMATED_TIME,
            )

            if not third_leg_b.succeeded or third_leg_b.path is None:
                logger.warning(f"  [O2 Candidate {j}] Leg 3B (Corner -> Exit) FAILED")
                continue

            # Leg 3C: Return Lane -> Final Carpark Start
            third_leg_c = planner.path_planner.plan(
                third_leg_b.path.final_pose,
                return_pose,
                arena,
                objective=CostMetric.ESTIMATED_TIME,
            )

            if not third_leg_c.succeeded or third_leg_c.path is None:
                logger.warning(f"  [O2 Candidate {j}] Leg 3C (Exit -> Start) FAILED")
                continue

            logger.info(f"  [O2 Candidate {j}] Leg 3 Return Sequence SUCCEEDED")

            total_cost = (
                (first_leg.selected_cost or 0.0)
                + (second_leg.selected_cost or 0.0)
                + third_leg_a.path.objective_cost
                + third_leg_b.path.objective_cost
                + third_leg_c.path.objective_cost
            )

            if best_cost is None or total_cost < best_cost:
                best_cost = total_cost
                best_solution = (
                    obstacle_1,
                    obstacle_2,
                    first_leg,
                    second_leg,
                    third_leg_a,
                    third_leg_b,
                    third_leg_c,
                )

    if best_solution is None:
        logger.error("Planning completed: No valid route found across all legs.")
        raise RuntimeError(
            "Fastest Car could not find a route "
            "from START -> Obstacle 1 -> Obstacle 2 -> Loop Exit -> START"
        )

    logger.info(f"Planning complete! Optimal route found with total cost: {best_cost:.2f}")

    (
        obstacle_1,
        obstacle_2,
        first_leg,
        second_leg,
        third_leg_a,
        third_leg_b,
        third_leg_c,
    ) = best_solution

    continuous_solution = planner._materialize_continuous_solution(
        arena,
        RouteOptimizationSolution(
            endpoints=(
                obstacle_1,
                obstacle_2,
            ),
            entries=(
                first_leg,
                second_leg,
            ),
            cost=best_cost,
        ),
        CostMetric.ESTIMATED_TIME,
    )[0]

    if continuous_solution is None:
        raise RuntimeError(
            "Fastest Car route could not be materialized continuously"
        )

    route = planner._compose_route(
        arena,
        continuous_solution,
        CostMetric.ESTIMATED_TIME,
    )

    path_3a, path_3b, path_3c = third_leg_a.path, third_leg_b.path, third_leg_c.path
    assert path_3a is not None and path_3b is not None and path_3c is not None

    route = replace(
        route,
        sampled_poses=(
            route.sampled_poses
            + path_3a.sampled_poses[1:]
            + path_3b.sampled_poses[1:]
            + path_3c.sampled_poses[1:]
        ),
        execution_steps=(
            route.execution_steps
            + tuple(
                MoveStep(segment, segment.primitive.command)
                for segment in path_3a.segments
            )
            + tuple(
                MoveStep(segment, segment.primitive.command)
                for segment in path_3b.segments
            )
            + tuple(
                MoveStep(segment, segment.primitive.command)
                for segment in path_3c.segments
            )
        ),
    )

    time_3a = getattr(third_leg_a.metrics, "planning_time_s", 0.0) if hasattr(third_leg_a, "metrics") else 0.0
    time_3b = getattr(third_leg_b.metrics, "planning_time_s", 0.0) if hasattr(third_leg_b, "metrics") else 0.0
    time_3c = getattr(third_leg_c.metrics, "planning_time_s", 0.0) if hasattr(third_leg_c, "metrics") else 0.0

    planning_result = PlanningResult(
        PlanningStatus.SUCCESS,
        route=route,
        metrics=planner._planning_metrics(
            targets_requested=2,
            targets_routed=2,
            planning_time_s=(
                first_leg.result.metrics.planning_time_s
                + second_leg.result.metrics.planning_time_s
                + time_3a
                + time_3b
                + time_3c
            ),
            route=route,
            optimized_cost=best_cost,
            candidate_groups=candidates,
        ),
    )

    simulator = HeadlessSimulator(
        arena,
        candidates,
        simulation_steps_from_execution(
            route.execution_steps,
            config,
        ),
        planned_path=route.sampled_poses,
        target_order=route.target_order,
        selected_candidates=tuple(
            zip(
                route.target_order,
                route.selected_candidate_kinds,
            )
        ),
    )

    return FastestCarScenario(
        simulator,
        config,
        planning_result,
        setup,
    )


__all__ = [
    "FastestCarScenario",
    "build_fastest_car_demo",
    "fastest_car_demo_arena",
]