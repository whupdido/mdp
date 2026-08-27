"""Command-line entrypoint for the optional Pygame simulator."""

from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="MDP Task 1 simulator")
    parser.add_argument("--demo", action="store_true", help="run the bundled Phase 4 route")
    args = parser.parse_args()
    if not args.demo:
        parser.error("select the currently available scenario with --demo")

    # Importing these modules is intentionally delayed until the Pygame
    # executable is requested. Core simulator imports remain dependency-free.
    from .app import run_simulator
    from .demo import build_demo_simulator

    simulator, config = build_demo_simulator()
    run_simulator(simulator, config)


if __name__ == "__main__":
    main()
