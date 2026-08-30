"""Task 1 façade composing targets, pairwise Hybrid A*, and exact routing."""

from __future__ import annotations

import time

from algorithm.config import PlanningConfig, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import CostMetric, Gear, PlanningStatus, RoutingMode, Steering
from algorithm.models.arena import ArenaInput
from algorithm.models.motion import CaptureStep, MoveStep
from algorithm.models.planning import (
    PairwisePath,
    PathMetrics,
    PlanningIssue,
    PlanningMetrics,
    PlanningResult,
    RoutePlan,
    TargetReachability,
)
from algorithm.pathfinding.hybrid_astar import HybridAStarPlanner
from algorithm.pathfinding.models import LocalPlanningStatus, PathPlanner
from algorithm.targets import ObservationCandidateGroup, generate_arena_observation_candidates

from .cache import DirectedPairwisePathCache
from .models import (
    DirectedPairwiseGraph,
    PairwiseCacheEntry,
    PairwiseCacheStats,
    PairwisePathProvider,
    RouteEndpoint,
    RouteOptimizationSolution,
    RouteOrderOptimizer,
)
from .optimizers import ExhaustiveRouteOptimizer, NearestNeighbourRouteOptimizer


class Task1Planner:
    """Plan a complete no-return Task 1 route through every required target."""

    def __init__(
        self,
        config: PlanningConfig = UNCALIBRATED_SIMULATION_CONFIG,
        *,
        path_planner: PathPlanner | None = None,
        route_optimizer: RouteOrderOptimizer | None = None,
        pairwise_cache: PairwisePathProvider | None = None,
        routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION,
    ) -> None:
        if not isinstance(config, PlanningConfig):
            raise TypeError("config must be a PlanningConfig")
        self.config = config
        local_planner = path_planner or HybridAStarPlanner(config)
        self.path_planner = local_planner
        self.route_optimizer = route_optimizer or ExhaustiveRouteOptimizer()
        self.pairwise_cache = pairwise_cache or DirectedPairwisePathCache(local_planner)
        self.routing_mode = routing_mode

    def plan(
        self,
        arena: ArenaInput,
        *,
        objective: CostMetric = CostMetric.ESTIMATED_TIME,
        routing_mode: RoutingMode | None = None,
    ) -> PlanningResult:
        if not isinstance(arena, ArenaInput):
            raise TypeError("arena must be an ArenaInput")
        if not isinstance(objective, CostMetric):
            raise TypeError("objective must be a CostMetric")
        mode = routing_mode or self.routing_mode
        if not isinstance(mode, RoutingMode):
            raise TypeError("routing_mode must be a RoutingMode")
        started_at = time.perf_counter()
        targets_requested = len(arena.obstacles)

        input_issues = arena.task1_issues()
        if targets_requested > self.config.guaranteed_max_targets:
            input_issues += (
                PlanningIssue(
                    "target_bound_exceeded",
                    f"Task 1 v1 supports at most {self.config.guaranteed_max_targets} targets",
                ),
            )
        if input_issues:
            return PlanningResult(
                PlanningStatus.INVALID_INPUT,
                issues=input_issues,
                metrics=self._planning_metrics(
                    targets_requested=targets_requested,
                    planning_time_s=time.perf_counter() - started_at,
                ),
            )

        candidate_started_at = time.perf_counter()
        candidate_groups = generate_arena_observation_candidates(arena, self.config)
        candidate_generation_time_s = time.perf_counter() - candidate_started_at
        geometric_issues = tuple(
            issue
            for group in candidate_groups
            if not group.has_valid_candidate
            for issue in group.issues
        )
        if geometric_issues:
            return PlanningResult(
                PlanningStatus.NO_FEASIBLE_ROUTE,
                issues=geometric_issues,
                metrics=self._planning_metrics(
                    targets_requested=targets_requested,
                    planning_time_s=time.perf_counter() - started_at,
                    candidate_generation_time_s=candidate_generation_time_s,
                    target_reachability=tuple(
                        TargetReachability(
                            group.obstacle_id,
                            sum(candidate.valid for candidate in group.candidates),
                            0,
                        )
                        for group in candidate_groups
                    ),
                ),
            )

        stats_before = self.pairwise_cache.stats
        valid_by_target = tuple(
            tuple(candidate for candidate in group.candidates if candidate.valid)
            for group in candidate_groups
        )
        if mode is RoutingMode.FULL_OPTIMIZATION:
            activation_tiers = (tuple(range(self.config.guaranteed_max_candidates_per_target)),)
        else:
            activation_tiers = self.config.candidate_activation_tiers

        active_ranks: set[int] = set()
        graph = None
        optimization = None
        tiers_activated = 0
        total_permutations = 0
        total_transitions = 0
        routing_started_at = time.perf_counter()
        planning_budget_exhausted = False
        for tier in activation_tiers:
            tiers_activated += 1
            active_ranks.update(tier)
            active_groups = tuple(
                tuple(candidate for candidate in group if candidate.preference_rank in active_ranks)
                for group in valid_by_target
            )
            # A target may have no valid candidate in an early tier. Expanding
            # the next tier requires no path query for that incomplete graph.
            if any(not group for group in active_groups):
                continue
            graph = self._build_graph(
                arena,
                active_groups,
                objective,
                deadline_monotonic=started_at + self.config.overall_planning_timeout_s,
                minimum_expansion_budget=(
                    self.config.max_expanded_nodes
                    if mode is RoutingMode.FULL_OPTIMIZATION else None
                ),
            )
            optimization = self.route_optimizer.optimize(graph)
            total_permutations += optimization.permutations_evaluated
            total_transitions += optimization.candidate_transitions_evaluated
            if optimization.solution is not None:
                break
            if time.perf_counter() - started_at >= self.config.overall_planning_timeout_s:
                planning_budget_exhausted = True
                break

        if graph is None:
            # All tiers were geometrically incomplete. The earlier full-set
            # geometric check means this can only occur with a malformed tier policy.
            graph = self._build_graph(
                arena,
                valid_by_target,
                objective,
                deadline_monotonic=started_at + self.config.overall_planning_timeout_s,
                minimum_expansion_budget=(
                    self.config.max_expanded_nodes
                    if mode is RoutingMode.FULL_OPTIMIZATION else None
                ),
            )
            optimization = self.route_optimizer.optimize(graph)
            total_permutations += optimization.permutations_evaluated
            total_transitions += optimization.candidate_transitions_evaluated
        assert optimization is not None
        baseline = (
            NearestNeighbourRouteOptimizer().optimize(graph)
            if optimization.solution is not None else None
        )
        routing_elapsed_s = time.perf_counter() - routing_started_at
        cache_delta = self.pairwise_cache.stats.difference(stats_before)
        pairwise_planning_time_s = cache_delta.planning_time_s
        global_routing_time_s = max(0.0, routing_elapsed_s - pairwise_planning_time_s)

        if optimization.solution is None:
            elapsed = time.perf_counter() - started_at
            issues = self._unreachable_issues(graph)
            local_budget_exhausted = any(
                entry.result.status in (
                    LocalPlanningStatus.SEARCH_LIMIT_REACHED,
                    LocalPlanningStatus.PLANNING_TIMEOUT,
                )
                for entry in graph.entries.values()
            )
            local_timeout = any(
                entry.result.status is LocalPlanningStatus.PLANNING_TIMEOUT
                for entry in graph.entries.values()
            )
            if planning_budget_exhausted:
                issues = (
                    PlanningIssue(
                        "task1_planning_budget_reached",
                        "Task 1 feasibility planning reached its configured overall time bound",
                    ),
                ) + issues
            metrics = self._planning_metrics(
                targets_requested=targets_requested,
                cache_stats=cache_delta,
                graph=graph,
                permutations=total_permutations,
                transitions=total_transitions,
                planning_time_s=elapsed,
                nearest_solution=baseline.solution if baseline is not None else None,
                candidate_generation_time_s=candidate_generation_time_s,
                pairwise_planning_time_s=pairwise_planning_time_s,
                global_routing_time_s=global_routing_time_s,
                candidate_groups=candidate_groups,
                candidate_tiers_activated=tiers_activated,
                planning_budget_exhausted=planning_budget_exhausted,
            )
            return PlanningResult(
                PlanningStatus.PLANNING_TIMEOUT
                if planning_budget_exhausted or local_timeout
                else PlanningStatus.SEARCH_LIMIT_REACHED
                if local_budget_exhausted
                else PlanningStatus.NO_FEASIBLE_ROUTE,
                issues=issues,
                metrics=metrics,
            )

        materialization_started_at = time.perf_counter()
        continuous_solution, continuity_issue = self._materialize_continuous_solution(
            arena,
            optimization.solution,
            objective,
        )
        pairwise_planning_time_s += time.perf_counter() - materialization_started_at
        cache_delta = self.pairwise_cache.stats.difference(stats_before)
        elapsed = time.perf_counter() - started_at
        if continuous_solution is None:
            metrics = self._planning_metrics(
                targets_requested=targets_requested,
                cache_stats=cache_delta,
                graph=graph,
                permutations=total_permutations,
                transitions=total_transitions,
                planning_time_s=elapsed,
                nearest_solution=baseline.solution if baseline is not None else None,
                optimized_cost=optimization.solution.cost,
                candidate_generation_time_s=candidate_generation_time_s,
                pairwise_planning_time_s=pairwise_planning_time_s,
                global_routing_time_s=global_routing_time_s,
                candidate_groups=candidate_groups,
                candidate_tiers_activated=tiers_activated,
            )
            assert continuity_issue is not None
            return PlanningResult(
                PlanningStatus.NO_FEASIBLE_ROUTE,
                issues=(continuity_issue,),
                metrics=metrics,
            )

        route = self._compose_route(arena, continuous_solution, objective)
        metrics = self._planning_metrics(
            targets_requested=targets_requested,
            targets_routed=len(route.target_order),
            cache_stats=cache_delta,
            graph=graph,
            permutations=total_permutations,
            transitions=total_transitions,
            planning_time_s=elapsed,
            route=route,
            nearest_solution=baseline.solution if baseline is not None else None,
            optimized_cost=optimization.solution.cost,
            candidate_generation_time_s=candidate_generation_time_s,
            pairwise_planning_time_s=pairwise_planning_time_s,
            global_routing_time_s=global_routing_time_s,
            candidate_groups=candidate_groups,
            candidate_tiers_activated=tiers_activated,
        )
        return PlanningResult(PlanningStatus.SUCCESS, route=route, metrics=metrics)

    def _materialize_continuous_solution(
        self,
        arena: ArenaInput,
        solution: RouteOptimizationSolution,
        objective: CostMetric,
    ) -> tuple[RouteOptimizationSolution | None, PlanningIssue | None]:
        """Replan selected legs from actual reached poses, never nominal resets.

        The global optimizer selects target/candidate identities from the
        precomputed canonical graph. A local goal may be accepted inside the
        configured position tolerance, so the next physical leg must start at
        that reached pose rather than at the ideal candidate coordinate.
        """
        current = RouteEndpoint.start(arena.start_pose)
        entries: list[PairwiseCacheEntry] = []
        total_cost = 0.0
        for endpoint, canonical_entry in zip(solution.endpoints, solution.entries):
            entry = canonical_entry
            if canonical_entry.key.start.pose != current.pose:
                entry = self.pairwise_cache.get_or_plan(
                    current,
                    endpoint,
                    arena,
                    self.config,
                    objective,
                )
            if not entry.succeeded or entry.result.path is None:
                assert endpoint.obstacle_id is not None
                return None, PlanningIssue(
                    "continuous_route_materialization_failed",
                    (
                        f"target {endpoint.obstacle_id} is reachable in the canonical pairwise graph "
                        "but not from the preceding leg's actual reached pose"
                    ),
                    obstacle_id=endpoint.obstacle_id,
                )
            entries.append(entry)
            assert entry.selected_cost is not None
            total_cost += entry.selected_cost
            reached = entry.result.path.final_pose
            current = RouteEndpoint(
                endpoint.kind,
                reached,
                obstacle_id=endpoint.obstacle_id,
                candidate_index=endpoint.candidate_index,
                candidate_kind=endpoint.candidate_kind,
                nominal=endpoint.nominal,
                standoff_cm=endpoint.standoff_cm,
                lateral_class=endpoint.lateral_class,
                preference_rank=endpoint.preference_rank,
                candidate_label=endpoint.candidate_label,
            )
        return (
            RouteOptimizationSolution(solution.endpoints, tuple(entries), total_cost),
            None,
        )

    def _build_graph(
        self,
        arena: ArenaInput,
        candidate_groups: tuple[tuple, ...],
        objective: CostMetric,
        *,
        deadline_monotonic: float | None = None,
        minimum_expansion_budget: int | None = None,
    ) -> DirectedPairwiseGraph:
        start = RouteEndpoint.start(arena.start_pose)
        endpoint_groups = tuple(
            tuple(RouteEndpoint.candidate(candidate) for candidate in group)
            for group in candidate_groups
        )
        return DirectedPairwiseGraph(
            arena,
            self.config,
            objective,
            start,
            endpoint_groups,
            {},
            self.pairwise_cache,
            deadline_monotonic,
            minimum_expansion_budget,
        )

    def _unreachable_issues(self, graph: DirectedPairwiseGraph) -> tuple[PlanningIssue, ...]:
        issues: list[PlanningIssue] = []
        all_candidates = tuple(endpoint for group in graph.candidate_groups for endpoint in group)
        for group in graph.candidate_groups:
            obstacle_id = group[0].obstacle_id
            has_incoming = any(
                graph.entries.get((source, goal)) is not None
                and graph.entries[(source, goal)].succeeded
                for goal in group
                for source in (graph.start,) + tuple(
                    endpoint for endpoint in all_candidates if endpoint.obstacle_id != obstacle_id
                )
            )
            if not has_incoming:
                issues.append(
                    PlanningIssue(
                        "no_reachable_observation_pose",
                        f"target {obstacle_id} has no observation candidate reachable from any route context",
                        obstacle_id=obstacle_id,
                    )
                )

        failed_entries = tuple(entry for entry in graph.entries.values() if not entry.succeeded)
        search_limits = sum(
            entry.result.status is LocalPlanningStatus.SEARCH_LIMIT_REACHED
            for entry in failed_entries
        )
        timeouts = sum(
            entry.result.status is LocalPlanningStatus.PLANNING_TIMEOUT
            for entry in failed_entries
        )
        if search_limits:
            issues.append(
                PlanningIssue(
                    "pairwise_search_limit_reached",
                    f"{search_limits} directed pairwise searches reached the configured expansion limit",
                )
            )
        if timeouts:
            issues.append(
                PlanningIssue(
                    "pairwise_planning_timeout",
                    f"{timeouts} directed pairwise searches exceeded their local or overall time bound",
                )
            )
        issues.append(
            PlanningIssue(
                "no_complete_task1_route",
                "no directed candidate chain visits every required target",
            )
        )
        return tuple(issues)

    def _compose_route(
        self,
        arena: ArenaInput,
        solution: RouteOptimizationSolution,
        objective: CostMetric,
    ) -> RoutePlan:
        local_paths: list[PairwisePath] = []
        execution_steps = []
        sampled_poses = [arena.start_pose]
        key_poses = [arena.start_pose]

        for endpoint, entry in zip(solution.endpoints, solution.entries):
            path = entry.result.path
            if path is None:
                raise ValueError("cannot compose a failed cached local path")
            expected_start = local_paths[-1].reached_pose if local_paths else arena.start_pose
            if path.start != expected_start:
                raise ValueError("composed local paths must share one continuous physical boundary pose")
            local_paths.append(
                PairwisePath(
                    start=path.start,
                    goal=path.requested_goal,
                    segments=path.segments,
                    metrics=path.metrics,
                    final_pose=path.final_pose,
                    sampled_poses=path.sampled_poses,
                    objective=path.objective,
                    objective_cost=path.objective_cost,
                )
            )
            self._append_path(sampled_poses, path.sampled_poses)
            self._append_path(key_poses, path.key_poses)
            execution_steps.extend(
                MoveStep(segment, segment.primitive.command)
                for segment in path.segments
            )
            assert endpoint.obstacle_id is not None
            execution_steps.append(CaptureStep(endpoint.obstacle_id, path.final_pose))

        metrics = self._aggregate_path_metrics(solution.entries, len(solution.endpoints))
        return RoutePlan(
            start=arena.start_pose,
            target_order=solution.target_order,
            observation_poses=tuple(endpoint.observation_pose for endpoint in solution.endpoints),
            local_paths=tuple(local_paths),
            execution_steps=tuple(execution_steps),
            metrics=metrics,
            objective=objective,
            objective_cost=solution.cost,
            sampled_poses=tuple(sampled_poses),
            key_poses=tuple(key_poses),
            selected_candidate_kinds=tuple(
                endpoint.candidate_label for endpoint in solution.endpoints  # type: ignore[misc]
            ),
        )

    @staticmethod
    def _append_path(destination: list, source: tuple) -> None:
        if not source:
            return
        destination.extend(source[1:] if destination and source[0] == destination[-1] else source)

    def _aggregate_path_metrics(
        self,
        entries: tuple[PairwiseCacheEntry, ...],
        capture_count: int,
    ) -> PathMetrics:
        paths = tuple(entry.result.path for entry in entries)
        if any(path is None for path in paths):
            raise ValueError("cannot aggregate failed paths")
        successful_paths = tuple(path for path in paths if path is not None)
        primitives = tuple(primitive for path in successful_paths for primitive in path.primitives)
        forward_distance = sum(
            primitive.geometric_length_cm for primitive in primitives if primitive.gear is Gear.FORWARD
        )
        reverse_distance = sum(
            primitive.geometric_length_cm for primitive in primitives if primitive.gear is Gear.REVERSE
        )
        return PathMetrics(
            geometric_distance_cm=forward_distance + reverse_distance,
            estimated_time_s=(
                sum(path.metrics.estimated_time_s for path in successful_paths)
                + capture_count * self.config.motion.capture_delay_s
            ),
            forward_distance_cm=forward_distance,
            reverse_distance_cm=reverse_distance,
            direction_changes=sum(
                first.gear is not second.gear for first, second in zip(primitives, primitives[1:])
            ),
            steering_changes=sum(
                first.steering is not second.steering for first, second in zip(primitives, primitives[1:])
            ),
            turn_count=sum(primitive.steering is not Steering.STRAIGHT for primitive in primitives),
            command_count=len(primitives),
            nodes_expanded=sum(path.metrics.nodes_expanded for path in successful_paths),
            nodes_generated=sum(path.metrics.nodes_generated for path in successful_paths),
            collision_checks=sum(path.metrics.collision_checks for path in successful_paths),
            planning_time_s=sum(path.metrics.planning_time_s for path in successful_paths),
        )

    def _solution_totals(
        self,
        solution: RouteOptimizationSolution | None,
    ) -> tuple[float | None, float | None, float | None]:
        if solution is None:
            return None, None, None
        entries = solution.entries
        distance = sum(entry.result.metrics.geometric_distance_cm for entry in entries)
        estimated_time = (
            sum(entry.result.metrics.estimated_time_s for entry in entries)
            + len(entries) * self.config.motion.capture_delay_s
        )
        return solution.cost, distance, estimated_time

    def _planning_metrics(
        self,
        *,
        targets_requested: int,
        targets_routed: int = 0,
        cache_stats: PairwiseCacheStats = PairwiseCacheStats(),
        graph: DirectedPairwiseGraph | None = None,
        permutations: int = 0,
        transitions: int = 0,
        planning_time_s: float,
        route: RoutePlan | None = None,
        nearest_solution: RouteOptimizationSolution | None = None,
        optimized_cost: float | None = None,
        candidate_generation_time_s: float = 0.0,
        pairwise_planning_time_s: float = 0.0,
        global_routing_time_s: float = 0.0,
        target_reachability: tuple[TargetReachability, ...] = (),
        candidate_groups: tuple[ObservationCandidateGroup, ...] = (),
        candidate_tiers_activated: int = 0,
        planning_budget_exhausted: bool = False,
    ) -> PlanningMetrics:
        nearest_cost, nearest_distance, nearest_time = self._solution_totals(nearest_solution)
        if graph is not None:
            geometric_counts = {
                group.obstacle_id: sum(candidate.valid for candidate in group.candidates)
                for group in candidate_groups
            }
            target_reachability = self._target_reachability(graph, geometric_counts)
        return PlanningMetrics(
            local_paths_requested=cache_stats.requests,
            local_paths_succeeded=cache_stats.successful_paths,
            cache_hits=cache_stats.hits,
            permutations_evaluated=permutations,
            total_planning_time_s=planning_time_s,
            targets_requested=targets_requested,
            targets_routed=targets_routed,
            candidate_transitions_evaluated=transitions,
            pairwise_cache_misses=cache_stats.misses,
            unreachable_pairwise_paths=cache_stats.failed_paths,
            selected_route_cost=route.objective_cost if route is not None else None,
            optimized_candidate_chain_cost=optimized_cost,
            total_distance_cm=route.metrics.geometric_distance_cm if route is not None else 0.0,
            total_estimated_time_s=route.metrics.estimated_time_s if route is not None else 0.0,
            nearest_neighbour_route_cost=nearest_cost,
            nearest_neighbour_distance_cm=nearest_distance,
            nearest_neighbour_estimated_time_s=nearest_time,
            candidate_generation_time_s=candidate_generation_time_s,
            pairwise_planning_time_s=pairwise_planning_time_s,
            global_routing_time_s=global_routing_time_s,
            target_reachability=target_reachability,
            candidate_tiers_activated=candidate_tiers_activated,
            candidate_count_considered=(
                sum(len(group) for group in graph.candidate_groups) if graph is not None else 0
            ),
            hybrid_astar_retries=cache_stats.retries,
            hybrid_astar_retry_recoveries=cache_stats.retry_recoveries,
            total_nodes_expanded=cache_stats.nodes_expanded,
            planning_budget_exhausted=planning_budget_exhausted,
        )

    @staticmethod
    def _target_reachability(
        graph: DirectedPairwiseGraph,
        geometric_counts: dict[int, int] | None = None,
    ) -> tuple[TargetReachability, ...]:
        all_candidates = tuple(endpoint for group in graph.candidate_groups for endpoint in group)
        result: list[TargetReachability] = []
        for group in graph.candidate_groups:
            target_id = group[0].obstacle_id
            sources = (graph.start,) + tuple(
                endpoint for endpoint in all_candidates if endpoint.obstacle_id != target_id
            )
            inbound = tuple(
                entry
                for goal in group
                for source in sources
                if (entry := graph.entries.get((source, goal))) is not None
            )
            reachable = sum(
                any(
                    graph.entries.get((source, goal)) is not None
                    and graph.entries[(source, goal)].succeeded
                    for source in sources
                )
                for goal in group
            )
            result.append(
                TargetReachability(
                    target_id=target_id,
                    geometric_candidates=(geometric_counts or {}).get(target_id, len(group)),
                    reachable_candidates=reachable,
                    no_path_edges=sum(
                        entry.result.status is LocalPlanningStatus.NO_PATH for entry in inbound
                    ),
                    activated_candidates=len(group),
                    search_limit_edges=sum(
                        entry.result.status is LocalPlanningStatus.SEARCH_LIMIT_REACHED
                        for entry in inbound
                    ),
                )
            )
        return tuple(result)


__all__ = ["Task1Planner"]
