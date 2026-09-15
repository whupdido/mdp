from __future__ import annotations
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

    # ------------------------------------------------------------
    # Fixed geometry
    # ------------------------------------------------------------
    carpark_width_cm = 60.0

    obstacle_width_cm = 40.0
    obstacle_height_cm = 60.0

    # ------------------------------------------------------------
    # Default clearance unless randomized
    # ------------------------------------------------------------
    if setup is None:
        setup = FastestCarSetup()

    top_clearance_cm = setup.top_clearance_cm
    bottom_clearance_cm = setup.bottom_clearance_cm
    carpark_to_obstacle_1_cm = setup.carpark_to_obstacle_1_cm
    obstacle_1_to_obstacle_2_cm = setup.obstacle_1_to_obstacle_2_cm

    # ------------------------------------------------------------
    # Arena height
    # ------------------------------------------------------------
    arena_height_cm = config.arena_height_cm

    center_y = arena_height_cm / 2.0

    # ------------------------------------------------------------
    # Robot starts at the right side of the carpark.
    # Carpark interior = 60 x 60 cm.
    # Robot = 30 x 30 cm.
    # Right edge of robot aligns with x = 60 cm entrance.
    # ------------------------------------------------------------
    robot_start = Pose(
        x_cm=45.0,
        y_cm=center_y,
        heading_rad=0.0,
    )

    # ------------------------------------------------------------
    # Obstacle 1
    #
    # Carpark right edge = x = 60.
    # ------------------------------------------------------------
    obstacle_1_min_x = (
        carpark_width_cm
        + carpark_to_obstacle_1_cm
    )

    obstacle_1_max_x = (
        obstacle_1_min_x
        + obstacle_width_cm
    )

    # ------------------------------------------------------------
    # Obstacle 2
    #
    # Gap measured from obstacle 1's right edge.
    # ------------------------------------------------------------
    obstacle_2_min_x = (
        obstacle_1_max_x
        + obstacle_1_to_obstacle_2_cm
    )

    obstacle_2_max_x = (
        obstacle_2_min_x
        + obstacle_width_cm
    )

    # ------------------------------------------------------------
    # Centre obstacles vertically.
    # ------------------------------------------------------------
    obstacle_min_y = (
        center_y
        - obstacle_height_cm / 2.0
    )

    obstacle_max_y = (
        center_y
        + obstacle_height_cm / 2.0
    )

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
        # Bottom physical wall — 20 cm thick.
        Wall(
            wall_id="arena_bottom",
            min_x_cm=60.0,
            min_y_cm=(
                obstacle_min_y
                - bottom_clearance_cm
                - 20.0
            ),
            max_x_cm=obstacle_2_max_x,
            max_y_cm=(
                obstacle_min_y
                - bottom_clearance_cm
            ),
        ),

        # Top physical wall — 20 cm thick.
        Wall(
            wall_id="arena_top",
            min_x_cm=60.0,
            min_y_cm=(
                obstacle_max_y
                + top_clearance_cm
            ),
            max_x_cm=obstacle_2_max_x,
            max_y_cm=(
                obstacle_max_y
                + top_clearance_cm
                + 20.0
            ),
        ),
    )

    return ArenaInput(
        robot_start,
        obstacles,
        config.arena_size_cm,
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

    # Fastest Car: fixed target order
    # START -> Obstacle 1 -> Obstacle 2
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

    if not obstacle_1_candidates:
        raise RuntimeError(
            "Obstacle 1 has no valid observation candidates"
        )

    if not obstacle_2_candidates:
        raise RuntimeError(
            "Obstacle 2 has no valid observation candidates"
        )

    start = RouteEndpoint.start(arena.start_pose)

    best_solution = None
    best_cost = None

    # Try:
    #
    #     START -> Obstacle 1 candidate -> Obstacle 2 candidate
    #
    # We deliberately do NOT try Obstacle 2 -> Obstacle 1.
    for obstacle_1 in obstacle_1_candidates:

        print(
            f"\nTrying START -> O1: "
            f"{obstacle_1.candidate_label}"
        )

        first_leg = planner.pairwise_cache.get_or_plan(
            start,
            obstacle_1,
            arena,
            config,
            CostMetric.ESTIMATED_TIME,
        )

        print(
            f"  START -> O1: "
            f"succeeded={first_leg.succeeded}, "
            f"status={first_leg.result.status}"
        )

        if not first_leg.succeeded or first_leg.result.path is None:
            continue

        # Use the actual pose reached after obstacle 1.
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

        print(
            f"  Reached O1 at: "
            f"x={first_leg.result.path.final_pose.x_cm:.1f}, "
            f"y={first_leg.result.path.final_pose.y_cm:.1f}"
        )

        for obstacle_2 in obstacle_2_candidates:

            print(
                f"  Trying O1 -> O2: "
                f"{obstacle_2.candidate_label}"
            )

            second_leg = planner.pairwise_cache.get_or_plan(
                reached_obstacle_1,
                obstacle_2,
                arena,
                config,
                CostMetric.ESTIMATED_TIME,
                required_first_steering=setup.obstacle_1_direction,
            )

            print(
                f"    O1 -> O2: "
                f"succeeded={second_leg.succeeded}, "
                f"status={second_leg.result.status}"
            )

            if not second_leg.succeeded or second_leg.result.path is None:
                continue

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

            # The robot has reached the front of O2.
            # Now plan to a waypoint on the back side of O2.
            obstacle_2_geometry = next(
                obstacle
                for obstacle in arena.obstacles
                if obstacle.obstacle_id == obstacle_2.obstacle_id
            )

            back_camera_position = desired_camera_position(
                obstacle_2_geometry,
                Direction.EAST,
                0.0,
                config,
            )

            back_of_o2 = rear_axle_pose_for_camera(
                back_camera_position,
                Direction.WEST,
                config.camera,
            )

            required_steering = setup.obstacle_2_direction

            third_result = planner.path_planner.plan(
                reached_obstacle_2.pose,
                back_of_o2,
                arena,
                objective=CostMetric.ESTIMATED_TIME,
                required_first_steering=required_steering,
            )

            print(
                f"    O2 -> back of O2: "
                f"succeeded={third_result.succeeded}, "
                f"status={third_result.status}"
            )

            if not third_result.succeeded or third_result.path is None:
                continue

            total_cost = (
                (first_leg.selected_cost or 0.0)
                + (second_leg.selected_cost or 0.0)
            )

            if best_cost is None or total_cost < best_cost:
                best_cost = total_cost
                best_solution = (
                    obstacle_1,
                    obstacle_2,
                    first_leg,
                    second_leg,
                    third_result,
                )

    if best_solution is None:
        raise RuntimeError(
            "Fastest Car could not find a route "
            "from START -> Obstacle 1 -> Obstacle 2"
        )

    obstacle_1, obstacle_2, first_leg, second_leg, third_result = best_solution

    # Build the continuous route using the actual reached pose
    # from the first leg.
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

    # Append O2 -> back of O2 to the displayed/executed route.
    third_path = third_result.path
    assert third_path is not None

    route = replace(
        route,
        sampled_poses=(
            route.sampled_poses
            + third_path.sampled_poses[1:]
        ),
        execution_steps=(
            route.execution_steps
            + tuple(
                MoveStep(segment, segment.primitive.command)
                for segment in third_path.segments
            )
        ),
    )

    planning_result = PlanningResult(
        PlanningStatus.SUCCESS,
        route=route,
        metrics=planner._planning_metrics(
            targets_requested=2,
            targets_routed=2,
            planning_time_s=(
                first_leg.result.metrics.planning_time_s
                + second_leg.result.metrics.planning_time_s
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
    "build_fastest_car_geometry_preview",
    "fastest_car_demo_arena",
]
