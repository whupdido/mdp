"""Command-line entrypoint for the optional Pygame simulator."""

from __future__ import annotations

import argparse
from pathlib import Path

from .production import CandidatePolicy, TurnProfile
from algorithm.enums import RoutingMode


def main() -> None:
    parser = argparse.ArgumentParser(description="MDP Task 1 simulator")
    scenarios = parser.add_mutually_exclusive_group(required=True)
    scenarios.add_argument("--demo", action="store_true", help="B.1 simulator demo: play the deterministic scripted movement demonstration")
    scenarios.add_argument(
        "--hybrid-demo",
        action="store_true",
        help="Hybrid A* debug demo: plan and play one local car-like path",
    )
    scenarios.add_argument(
        "--task1-demo",
        action="store_true",
        help="B.2 Task 1 demo: plan and play the deterministic five-image route",
    )
    scenarios.add_argument(
        "--task1-editor",
        action="store_true",
        help="B.2 Task 1 editor: edit obstacles/faces, plan, and play a five-image route",
    )
    scenarios.add_argument("--task1-production", action="store_true", help="production Task 1 editor and physical-run replay")
    scenarios.add_argument("--task1-benchmark", action="store_true", help="run repeated Task 1 profile and scalability benchmarks")
    scenarios.add_argument(
        "--task1-random",
        action="store_true",
        help="B.2 random arena: open a seeded five-obstacle Task 1 scenario",
    )
    scenarios.add_argument(
        "--local-plan-demo",
        action="store_true",
        help="Debug: run one synthetic local Hybrid A* primitive/path diagnostic",
    )
    scenarios.add_argument("--local-arena-diagnostic", action="store_true", help="Debug: run independent local Hybrid A* queries on the known real editor arena")
    parser.add_argument("--seed", type=int, help="deterministic random seed used by --task1-random")
    parser.add_argument(
        "--solvable",
        action="store_true",
        help="with --task1-random, retry until Task1Planner verifies a complete route",
    )
    parser.add_argument("--scenario-file", type=Path, help="load a Task 1 scenario (requires --task1-production)")
    parser.add_argument("--turn-profile", choices=[item.value for item in TurnProfile],
                        default=TurnProfile.PRODUCTION_60_90.value)
    parser.add_argument("--candidate-policy", choices=[item.value for item in CandidatePolicy],
                        default=CandidatePolicy.CONTROL_20.value)
    parser.add_argument("--benchmark-output", type=Path, help="write benchmark results to JSON")
    parser.add_argument(
        "--retry-limit",
        type=int,
        default=50,
        help="maximum solvable-generation attempts for --task1-random --solvable (default: 50)",
    )
    args = parser.parse_args()
    if (args.seed is not None or args.solvable or args.retry_limit != 50) and not args.task1_random:
        parser.error("--seed, --solvable, and --retry-limit require --task1-random")
    if args.scenario_file is not None and not args.task1_production:
        parser.error("--scenario-file requires --task1-production")
    if (args.turn_profile != TurnProfile.PRODUCTION_60_90.value or
            args.candidate_policy != CandidatePolicy.CONTROL_20.value) and not args.task1_production:
        parser.error("diagnostic profiles require --task1-production")
    if args.benchmark_output is not None and not args.task1_benchmark:
        parser.error("--benchmark-output requires --task1-benchmark")

    if args.task1_benchmark:
        import json
        from .benchmark import run_full_benchmark_suite
        from .production import build_production_config

        config, _live, _calibration = build_production_config()
        print("Benchmarking deterministic fixtures and fixed-seed solvable 7/8-target scenarios.", flush=True)
        report = run_full_benchmark_suite(config, Path(__file__).parent / "scenarios")
        rendered = json.dumps(report, indent=2, sort_keys=True)
        if args.benchmark_output is not None:
            args.benchmark_output.parent.mkdir(parents=True, exist_ok=True)
            args.benchmark_output.write_text(rendered + "\n", encoding="utf-8")
            print(f"Benchmark report saved to {args.benchmark_output}")
        print(rendered)
        return

    if args.local_plan_demo:
        from .local_plan_demo import run_local_plan_demo
        run_local_plan_demo()
        return
    if args.local_arena_diagnostic:
        from .local_plan_demo import run_open_arena_local_diagnostic
        run_open_arena_local_diagnostic()
        return

    # Importing these modules is intentionally delayed until the Pygame
    # executable is requested. Core simulator imports remain dependency-free.
    from .app import run_simulator

    if args.task1_production:
        from .task1_editor import run_task1_editor
        from .task1_editor_model import Task1EditorController
        from .production import build_production_config, configuration_banner
        from .scenarios import load_run, arena_from_record, config_fingerprint, historical_simulator
        from .task1_editor_model import EditorState

        production, production_live, _ = build_production_config()
        active, live, calibration = build_production_config(
            turn=TurnProfile(args.turn_profile), candidates=CandidatePolicy(args.candidate_policy))
        diagnostic = active != production
        route_mode = (RoutingMode.FEASIBILITY if CandidatePolicy(args.candidate_policy) is CandidatePolicy.LAZY_20_THEN_30
                      else RoutingMode.FULL_OPTIMIZATION)
        print(configuration_banner(active, live_calibration=live, diagnostic=diagnostic,
                                    production_config=production if diagnostic else None,
                                    routing_mode=route_mode))
        controller = Task1EditorController(active, target_count_range=(1, 8), routing_mode=route_mode)
        controller.production_mode = True
        controller.calibration_metadata = calibration
        controller.loaded_scenario_record = None
        if args.scenario_file is not None:
            record = load_run(args.scenario_file)
            controller.load_arena(arena_from_record(record["original_arena_payload"]))
            controller.loaded_scenario_record = record
            recorded = record.get("planner_config_fingerprint", "not recorded")
            current = config_fingerprint(active, routing_mode=route_mode)
            print(f"RECORDED CONFIG: {recorded}\nCURRENT CONFIG:  {current}\nCONFIG MATCH: {'YES' if recorded == current else 'NO'}")
            route = record.get("planned_route", {})
            if route.get("sampled_poses"):
                controller.simulator = historical_simulator(record, active)
                controller.state = EditorState.PLAN_READY
                controller.status_message = "Historical route loaded; Enter explicitly replans"
        run_task1_editor(controller)
    elif args.task1_editor or args.task1_random:
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
        run_simulator(scenario.simulator, scenario.config, planning_result=result)
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
