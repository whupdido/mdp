"""Command-line entrypoint for the optional Pygame simulator."""

from __future__ import annotations
from .app import run_simulator

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="MDP Task 1 simulator")
    scenarios = parser.add_mutually_exclusive_group(required=True)
    scenarios.add_argument("--demo", action="store_true", help="run the bundled Phase 4 route")
    scenarios.add_argument(
        "--hybrid-demo",
        action="store_true",
        help="plan and play one Phase 5 Hybrid A* query",
    )
    scenarios.add_argument(
        "--task1-demo",
        action="store_true",
        help="plan and play the complete five-target Phase 6 route",
    )
    scenarios.add_argument(
        "--task1-editor",
        action="store_true",
        help="edit, plan, and play a five-target Task 1 arena",
    )
    scenarios.add_argument(
        "--task1-random",
        action="store_true",
        help="open the editor with a seeded random five-target arena",
    )
    scenarios.add_argument(
        "--fastest-car-demo",
        action="store_true",
        help="run the continuous-coordinate Fastest Car development demo",
    )
    scenarios.add_argument(
        "--fastest-car-edit",
        action="store_true",
        help="edit the Fastest Car geometry",
    )
    scenarios.add_argument(
        "--fastest-car-geometry",
        action="store_true",
        help="show the Fastest Car geometry without path planning",
    )
    scenarios.add_argument(
        "--local-plan-demo",
        action="store_true",
        help="run one headless local Hybrid A* diagnostic query",
    )
    scenarios.add_argument("--local-arena-diagnostic", action="store_true", help="diagnose real editor arena local queries")
    parser.add_argument("--seed", type=int, help="seed for --task1-random")
    parser.add_argument(
        "--solvable",
        action="store_true",
        help="retry random maps until a complete route is found",
    )
    parser.add_argument(
        "--retry-limit",
        type=int,
        default=50,
        help="bounded attempts for --task1-random --solvable (default: 50)",
    )
    args = parser.parse_args()
    if (args.seed is not None or args.solvable or args.retry_limit != 50) and not args.task1_random:
        parser.error("--seed, --solvable, and --retry-limit require --task1-random")

    if args.fastest_car_geometry:
        from .task2_demo import build_fastest_car_geometry_preview

        scenario = build_fastest_car_geometry_preview()
        arena = scenario.simulator.state.arena

        print("Fastest Car geometry preview")
        print(
            f"Arena: {scenario.config.arena_size_cm:.1f} x "
            f"{scenario.config.arena_height_cm:.1f} cm"
        )
        print(
            f"Robot start: ({arena.start_pose.x_cm:.1f}, "
            f"{arena.start_pose.y_cm:.1f}) cm"
        )
        print("Carpark: 60 x 60 cm, centred on robot start")
        for obstacle in arena.obstacles:
            print(
                f"Obstacle {obstacle.obstacle_id}: "
                f"x={obstacle.min_x_cm:.1f}..{obstacle.max_x_cm:.1f}, "
                f"y={obstacle.min_y_cm:.1f}..{obstacle.max_y_cm:.1f}, "
                f"face={obstacle.face.value}"
            )

        run_simulator(
            scenario.simulator,
            scenario.config,
            fastest_car_mode=True,
        )
        return

    if args.fastest_car_edit:
        from .task2_editor import run_task2_editor
        from .task2_editor_model import Task2EditorController

        controller = Task2EditorController()
        run_task2_editor(controller)
        return

    if args.fastest_car_demo:
        from .task2_demo import build_fastest_car_demo

        scenario = build_fastest_car_demo()
        result = scenario.planning_result
        assert result.route is not None
        print(
            "Fastest Car:", result.status.value,
            f"targets={len(result.route.target_order)}",
            f"distance={result.route.metrics.geometric_distance_cm:.1f} cm",
            f"estimated_time={result.route.metrics.estimated_time_s:.2f} s",
            f"planning_time={result.metrics.total_planning_time_s:.2f} s",
        )
        print("Order:", " -> ".join(map(str, result.route.target_order)))
        print("Commands:", " ".join(p.command for p in result.route.primitives))
        run_simulator(
            scenario.simulator,
            scenario.config,
            fastest_car_mode=True,
        )
        return

    if args.local_plan_demo:
        from .local_plan_demo import run_local_plan_demo
        run_local_plan_demo()
        return
    if args.local_arena_diagnostic:
        from .local_plan_demo import run_open_arena_local_diagnostic
        run_open_arena_local_diagnostic()
        return

    if args.task1_editor or args.task1_random:
        from .task1_demo import task1_demo_obstacles
        from .task1_editor import run_task1_editor
        from .task1_editor_model import Task1EditorController

        controller = Task1EditorController(
            obstacles=task1_demo_obstacles() if args.task1_editor else (),
        )
        if args.task1_random:
            outcome = controller.randomize(
                seed=args.seed,
                require_solvable=args.solvable,
                retry_limit=args.retry_limit,
            )
            print(
                "Random Task 1:",
                f"seed={args.seed}",
                f"attempts={outcome.attempts}",
                f"solvable_requested={outcome.solvable_requested}",
                f"status={'success' if outcome.succeeded else 'no_route'}",
            )
        run_task1_editor(controller)
    elif args.task1_demo:
        from .task1_demo import build_task1_demo

        scenario = build_task1_demo()
        result = scenario.planning_result
        assert result.route is not None
        metrics = result.metrics
        print("Task 1:", result.status.value)
        start = result.route.start
        print(
            "Initial pose:",
            f"({start.x_cm:.1f}, {start.y_cm:.1f}) cm",
            f"heading={start.heading_rad:.6f} rad",
        )
        print("Order:", " -> ".join(map(str, result.route.target_order)))
        print(
            "Candidates:",
            ", ".join(
                f"{target_id}:{kind}"
                for target_id, kind in zip(
                    result.route.target_order,
                    result.route.selected_candidate_kinds,
                )
            ),
        )
        print("Primitive legs:")
        for target_id, candidate_kind, local_path in zip(
            result.route.target_order,
            result.route.selected_candidate_kinds,
            result.route.local_paths,
        ):
            commands = " ".join(primitive.command for primitive in local_path.primitives) or "HOLD"
            print(f"  target {target_id}:{candidate_kind} <- {commands}")
        forward_count = sum(
            primitive.gear.value == "forward" for primitive in result.route.primitives
        )
        reverse_count = len(result.route.primitives) - forward_count
        print(
            f"Primitives: forward={forward_count}",
            f"reverse={reverse_count}",
            f"direction changes={result.route.metrics.direction_changes}",
        )
        optimized_cost = metrics.optimized_candidate_chain_cost
        nearest_cost = metrics.nearest_neighbour_route_cost
        assert optimized_cost is not None and nearest_cost is not None
        print(
            f"Optimized chain cost={optimized_cost:.3f}",
            f"materialized route cost={result.route.objective_cost:.3f}",
            f"nearest-neighbour={nearest_cost:.3f}",
        )
        print(
            f"distance={result.route.metrics.geometric_distance_cm:.1f} cm",
            f"provisional time={result.route.metrics.estimated_time_s:.3f} s",
            f"planning runtime={metrics.total_planning_time_s:.3f} s",
        )
        run_simulator(scenario.simulator, scenario.config)
    elif args.hybrid_demo:
        from .hybrid_demo import build_hybrid_demo

        scenario = build_hybrid_demo()
        print(
            "Hybrid A*:",
            scenario.planning_result.status.value,
            f"commands={scenario.planning_result.metrics.command_count}",
            f"expanded={scenario.planning_result.metrics.nodes_expanded}",
        )
        run_simulator(
            scenario.simulator,
            scenario.config,
            debug_nodes=scenario.planning_result.debug.expanded_states,
            show_debug_nodes=True,
        )
    else:
        from .demo import build_demo_simulator

        simulator, config = build_demo_simulator()
        run_simulator(simulator, config)


if __name__ == "__main__":
    main()
