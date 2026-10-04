"""Repeatable Task 1 route-quality and performance comparisons."""

from __future__ import annotations

import random
import statistics
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from algorithm.config import PlanningConfig
from algorithm.enums import CostMetric, Direction, PlanningStatus, RoutingMode
from algorithm.models import ArenaInput, GridCell, Obstacle
from algorithm.routing import Task1Planner
from algorithm.simulator.production import TURN_ANGLES, TurnProfile
from algorithm.simulator.profiles import CandidatePolicy, candidate_profile, modest_turn_cost_profile, strong_turn_cost_profile, turn_profile
from algorithm.simulator.scenarios import build_run_record


@dataclass(frozen=True, slots=True)
class BenchmarkRun:
    status: str
    elapsed_s: float
    route_quality: dict | None
    planner_diagnostics: dict
    issues: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class RepeatedBenchmark:
    obstacle_count: int
    runs: tuple[BenchmarkRun, ...]
    median_s: float
    worst_s: float
    success_rate: float


def route_quality(arena: ArenaInput, config: PlanningConfig, result) -> dict | None:
    if result.status is not PlanningStatus.SUCCESS or result.route is None:
        return None
    record = build_run_record(arena, config, result)
    moves = record["planned_route"]["movements"]
    primitives = result.route.primitives
    clearance = min((item["minimum_obstacle_clearance_cm"] for item in moves), default=float("inf"))
    boundary = min((item["minimum_arena_clearance_cm"] for item in moves), default=float("inf"))
    warnings = [f"obstacle clearance below {threshold} cm" for threshold in (8, 5, 3)
                if clearance < threshold]
    if boundary < 5:
        warnings.append("arena clearance below 5 cm")
    reverse_turns = sum(item.gear.value == "reverse" and item.steering.value != "straight" for item in primitives)
    if reverse_turns >= 3:
        warnings.append("unusually high reverse-turn count")
    minimum_obstacle_event = min(moves, key=lambda item: item["minimum_obstacle_clearance_cm"], default=None)
    minimum_arena_event = min(moves, key=lambda item: item["minimum_arena_clearance_cm"], default=None)
    return {
        "objective": result.route.objective.value,
        "objective_cost": result.route.objective_cost,
        "serialized_movement_commands": len(moves),
        "straight_commands": sum(item.steering.value == "straight" for item in primitives),
        "turn_commands": sum(item.steering.value != "straight" for item in primitives),
        "reverse_commands": sum(item.gear.value == "reverse" for item in primitives),
        "reverse_turns": reverse_turns,
        "steering_changes": result.route.metrics.steering_changes,
        "direction_changes": result.route.metrics.direction_changes,
        "geometric_distance_cm": result.route.metrics.geometric_distance_cm,
        "estimated_execution_time_s": result.route.metrics.estimated_time_s,
        "minimum_obstacle_clearance_cm": clearance,
        "minimum_arena_clearance_cm": boundary,
        "minimum_obstacle_clearance_event": ({"sequence": minimum_obstacle_event["sequence"],
                                               "command": minimum_obstacle_event["command"],
                                               "motion": minimum_obstacle_event["motion"],
                                               "gear": minimum_obstacle_event["gear"],
                                               "obstacle_id": minimum_obstacle_event["closest_obstacle_id"]}
                                              if minimum_obstacle_event else None),
        "minimum_arena_clearance_event": ({"sequence": minimum_arena_event["sequence"],
                                            "command": minimum_arena_event["command"],
                                            "motion": minimum_arena_event["motion"],
                                            "gear": minimum_arena_event["gear"]}
                                           if minimum_arena_event else None),
        "target_order": list(result.route.target_order),
        "selected_observation_candidates": list(result.route.selected_candidate_kinds),
        "candidate_count_considered": result.metrics.candidate_count_considered,
        "candidate_tiers_activated": result.metrics.candidate_tiers_activated,
        "candidate_poses_generated": result.metrics.candidate_poses_generated,
        "hybrid_astar_nodes_expanded": result.metrics.total_nodes_expanded,
        "warnings": warnings,
    }


def benchmark_once(arena: ArenaInput, config: PlanningConfig, *, objective: CostMetric = CostMetric.ESTIMATED_TIME,
                   routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION) -> BenchmarkRun:
    started = time.perf_counter()
    planner = Task1Planner(config, routing_mode=routing_mode)
    result = planner.plan(arena, objective=objective)
    elapsed = time.perf_counter() - started
    serialization_started = time.perf_counter()
    # The same serializer used by the server is part of the measured route path.
    from algorithm.serialization import serialize_planning_result
    serialized = serialize_planning_result(result)
    serialization_s = time.perf_counter() - serialization_started
    metrics = result.metrics
    diag = {
        "candidate_generation_time_s": metrics.candidate_generation_time_s,
        "candidate_poses": metrics.candidate_poses_generated,
        "pairwise_path_requests": metrics.local_paths_requested,
        "pairwise_cache_hits": metrics.cache_hits,
        "pairwise_cache_misses": metrics.pairwise_cache_misses,
        "hybrid_astar_calls_estimate": metrics.pairwise_cache_misses + metrics.hybrid_astar_retries,
        "hybrid_astar_retries": metrics.hybrid_astar_retries,
        "hybrid_astar_nodes_expanded": metrics.total_nodes_expanded,
        "hybrid_astar_time_s": metrics.pairwise_planning_time_s,
        "target_order_optimization_time_s": metrics.global_routing_time_s,
        "continuous_materialization_time_s": metrics.materialization_time_s,
        "materialization_replans": metrics.materialization_replans,
        "serialization_time_s": serialization_s,
        "total_planning_time_s": metrics.total_planning_time_s,
        "total_elapsed_s": elapsed,
        "serialized_step_count": len(serialized.get("steps", ())),
        "candidate_combinations": metrics.candidate_transitions_evaluated,
        "target_orders_evaluated": metrics.permutations_evaluated,
    }
    return BenchmarkRun(result.status.value, elapsed, route_quality(arena, config, result), diag,
                        tuple(issue.message for issue in result.issues))


def repeat_benchmark(arena: ArenaInput, config: PlanningConfig, repeats: int, *,
                     objective: CostMetric = CostMetric.ESTIMATED_TIME,
                     routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION) -> RepeatedBenchmark:
    if repeats <= 0:
        raise ValueError("repeats must be positive")
    runs = tuple(benchmark_once(arena, config, objective=objective, routing_mode=routing_mode)
                 for _ in range(repeats))
    times = [run.elapsed_s for run in runs]
    return RepeatedBenchmark(len(arena.obstacles), runs, statistics.median(times), max(times),
                             sum(run.route_quality is not None for run in runs) / repeats)


def seeded_arena(seed: int, target_count: int, config: PlanningConfig) -> ArenaInput:
    """Generate a reproducible arena with geometrically valid control poses.

    Route reachability remains measured with Task1Planner; this helper only
    avoids spending an expensive Hybrid A* run on maps whose required 20 cm
    image poses are already invalid from overlap, obstacle occlusion, or the
    modeled arena boundary.
    """
    if not 1 <= target_count <= 8:
        raise ValueError("target_count must be between 1 and 8")
    rng = random.Random(seed)
    from algorithm.coordinates import default_start_pose
    from algorithm.targets import generate_arena_observation_candidates

    cells = [GridCell(x, y) for x in range(4, 18) for y in range(4, 18)
             if not (x <= 3 and y <= 3)]
    start = default_start_pose(config.robot)
    for _attempt in range(500):
        rng.shuffle(cells)
        chosen: list[GridCell] = []
        for cell in cells:
            if all((cell.x-other.x)**2 + (cell.y-other.y)**2 >= 9 for other in chosen):
                chosen.append(cell)
                if len(chosen) == target_count:
                    break
        if len(chosen) != target_count:
            continue
        obstacles = tuple(Obstacle(i, cell, rng.choice(tuple(Direction)))
                          for i, cell in enumerate(chosen, 1))
        arena = ArenaInput(start, obstacles)
        groups = generate_arena_observation_candidates(arena, config)
        if all(any(candidate.valid for candidate in group.candidates) for group in groups):
            return arena
    raise RuntimeError(f"seed {seed} could not generate {target_count} valid control poses")


def comparison_profiles(config: PlanningConfig) -> tuple[tuple[str, PlanningConfig, CostMetric, RoutingMode], ...]:
    profiles: list[tuple[str, PlanningConfig, CostMetric, RoutingMode]] = []
    for turn in TurnProfile:
        turn_cfg = turn_profile(config, TURN_ANGLES[turn])
        profiles.append((f"turns-{turn.value}", turn_cfg, CostMetric.ESTIMATED_TIME, RoutingMode.FULL_OPTIMIZATION))
    for policy in CandidatePolicy:
        candidate_cfg = candidate_profile(config, policy)
        mode = RoutingMode.FEASIBILITY if policy is CandidatePolicy.LAZY_20_THEN_30 else RoutingMode.FULL_OPTIMIZATION
        profiles.append((f"candidates-{policy.value}", candidate_cfg, CostMetric.ESTIMATED_TIME, mode))
    profiles.extend((
        ("objective-estimated-time", config, CostMetric.ESTIMATED_TIME, RoutingMode.FULL_OPTIMIZATION),
        ("objective-distance", config, CostMetric.DISTANCE, RoutingMode.FULL_OPTIMIZATION),
        ("modest-turn-cost", modest_turn_cost_profile(config), CostMetric.ESTIMATED_TIME, RoutingMode.FULL_OPTIMIZATION),
        ("strong-turn-cost-reference", strong_turn_cost_profile(config), CostMetric.ESTIMATED_TIME, RoutingMode.FULL_OPTIMIZATION),
    ))
    return tuple(profiles)


def run_full_benchmark_suite(config: PlanningConfig, scenario_directory: Path, *, stress_count: int = 10) -> dict:
    """Run repeated deterministic and fixed-seed solvable scalability comparisons."""
    from algorithm.simulator.scenarios import load_run, arena_from_record
    from algorithm.serialization import serialize_planning_result

    if stress_count < 1:
        raise ValueError("stress_count must be positive")
    fixture_paths = [scenario_directory / name for name in (
        "four_obstacles_simple.json", "five_obstacles_example.json",
        "six_obstacles_complex.json", "eight_obstacles_challenging.json")]
    fixtures = {path.stem: arena_from_record(load_run(path)["original_arena_payload"]) for path in fixture_paths}
    fixtures["one_obstacle_control"] = ArenaInput(
        fixtures["four_obstacles_simple"].start_pose, fixtures["four_obstacles_simple"].obstacles[:1])

    comparisons = []
    representative = ("four_obstacles_simple", "six_obstacles_complex", "eight_obstacles_challenging")
    for name in representative:
        for profile_name, profile_config, objective, mode in comparison_profiles(config):
            run = benchmark_once(fixtures[name], profile_config, objective=objective, routing_mode=mode)
            comparisons.append({"arena": name, "profile": profile_name, "result": asdict(run)})

    deterministic = {}
    for name, arena in fixtures.items():
        repeat_count = 5
        runs = [benchmark_once(arena, config) for _ in range(repeat_count)]
        elapsed = [run.elapsed_s for run in runs]
        deterministic[name] = {
            "obstacles": len(arena.obstacles), "runs": [asdict(run) for run in runs],
            "median_s": statistics.median(elapsed), "worst_s": max(elapsed),
            "success_rate": sum(run.route_quality is not None for run in runs) / repeat_count,
        }

    stress = {}
    candidate_scaling = []
    for target_count in (7, 8):
        accepted = []
        rejected = []
        seed = 73000 + target_count * 1000
        while len(accepted) < stress_count and seed < 73000 + target_count * 1000 + stress_count * 100:
            arena = seeded_arena(seed, target_count, config)
            run = benchmark_once(arena, config)
            if run.route_quality is not None:
                accepted.append((seed, arena))
            else:
                rejected.append({"seed": seed, "status": run.status, "elapsed_s": run.elapsed_s,
                                 "issues": run.issues, "diagnostics": run.planner_diagnostics})
            seed += 1
        measurements = []
        for accepted_seed, arena in accepted:
            repeats = [benchmark_once(arena, config) for _ in range(3)]
            times = [item.elapsed_s for item in repeats]
            measurements.append({"seed": accepted_seed, "obstacles": arena_record_for_benchmark(arena),
                                 "runs": [asdict(item) for item in repeats],
                                 "median_s": statistics.median(times), "worst_s": max(times),
                                 "success_rate": sum(item.route_quality is not None for item in repeats)/3})
        stress[str(target_count)] = {
            "requested_solvable_scenarios": stress_count,
            "accepted_solvable_scenarios": len(accepted),
            "aggregate_plan_success_rate": sum(item["success_rate"] for item in measurements) / len(measurements)
            if measurements else 0.0,
            "scenarios": measurements,
            "rejected_proposals": rejected,
        }
        if accepted:
            seed_sample, arena_sample = accepted[0]
            for policy in CandidatePolicy:
                candidate_config = candidate_profile(config, policy)
                candidate_mode = (RoutingMode.FEASIBILITY
                                  if policy is CandidatePolicy.LAZY_20_THEN_30
                                  else RoutingMode.FULL_OPTIMIZATION)
                outcome = benchmark_once(arena_sample, candidate_config,
                                         routing_mode=candidate_mode)
                candidate_scaling.append({
                    "obstacle_count": target_count, "seed": seed_sample,
                    "policy": policy.value, "result": asdict(outcome),
                })
    return {"production_config_fingerprint": _fingerprint_for_benchmark(config),
            "fixture_repeats": 5, "stress_repeats": 3,
            "deterministic": deterministic, "profile_comparisons": comparisons,
            "candidate_policy_scaling": candidate_scaling, "seeded_stress": stress}


def arena_record_for_benchmark(arena):
    from algorithm.simulator.scenarios import arena_record
    return arena_record(arena)


def _fingerprint_for_benchmark(config):
    from algorithm.simulator.scenarios import config_fingerprint
    return config_fingerprint(config)


__all__ = ["BenchmarkRun", "RepeatedBenchmark", "benchmark_once", "comparison_profiles", "repeat_benchmark", "route_quality", "run_full_benchmark_suite", "seeded_arena"]
