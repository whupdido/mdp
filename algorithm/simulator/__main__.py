"""Command-line entrypoint for the optional Pygame simulator."""

from __future__ import annotations

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
    args = parser.parse_args()

    # Importing these modules is intentionally delayed until the Pygame
    # executable is requested. Core simulator imports remain dependency-free.
    from .app import run_simulator

    if args.hybrid_demo:
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
