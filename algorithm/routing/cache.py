"""Reusable directed cache around a dependency-injected local path planner."""

from __future__ import annotations

import math
import time
import concurrent.futures

from algorithm.config import PlanningConfig
from algorithm.enums import CostMetric
from algorithm.models.arena import ArenaInput
from algorithm.pathfinding.models import PathPlanner
from algorithm.pathfinding.hybrid_astar import HybridAStarPlanner
from algorithm.pathfinding.models import LocalPlanningStatus

from .models import (
    PairwiseCacheEntry,
    PairwiseCacheKey,
    PairwiseCacheStats,
    RouteEndpoint,
)


def _parallel_plan_worker(payload: tuple) -> tuple:
    """Module-level worker for pickling compatibility in ProcessPoolExecutor."""
    start, goal, arena, config, objective, requested_budget, maximum_budget, timeout = payload
    planner = HybridAStarPlanner(config)
    
    budget = config.adaptive_initial_expansions
    deadline = time.perf_counter() + timeout
    attempts = 0
    cumulative_nodes = 0
    cumulative_time = 0.0
    
    while True:
        result = planner.plan(
            start.pose,
            goal.pose,
            arena,
            objective=objective,
            max_expanded_nodes=budget,
            max_planning_time_s=max(0.001, deadline - time.perf_counter()),
        )
        attempts += 1
        cumulative_nodes += result.metrics.nodes_expanded
        cumulative_time += result.metrics.planning_time_s
        final_budget = budget
        
        if result.status not in (LocalPlanningStatus.SEARCH_LIMIT_REACHED,):
            break
        if budget >= requested_budget or budget >= maximum_budget:
            break
        if time.perf_counter() >= deadline:
            break
        budget = min(
            requested_budget,
            maximum_budget,
            max(budget + 1, math.ceil(budget * config.adaptive_growth_factor)),
        )
        
    return start, goal, result, final_budget, attempts, cumulative_nodes, cumulative_time


class DirectedPairwisePathCache:
    """Memoize successes and structured failures without mirroring directions."""

    def __init__(self, planner: PathPlanner) -> None:
        self._planner = planner
        self._entries: dict[PairwiseCacheKey, PairwiseCacheEntry] = {}
        self._requests = 0
        self._hits = 0
        self._misses = 0
        self._successful_paths = 0
        self._failed_paths = 0
        self._retries = 0
        self._retry_recoveries = 0
        self._nodes_expanded = 0
        self._planning_time_s = 0.0

    @property
    def stats(self) -> PairwiseCacheStats:
        return PairwiseCacheStats(
            requests=self._requests,
            hits=self._hits,
            misses=self._misses,
            successful_paths=self._successful_paths,
            failed_paths=self._failed_paths,
            retries=self._retries,
            retry_recoveries=self._retry_recoveries,
            nodes_expanded=self._nodes_expanded,
            planning_time_s=self._planning_time_s,
        )

    @property
    def size(self) -> int:
        return len(self._entries)

    def warmup(
        self,
        pairs: list[tuple[RouteEndpoint, RouteEndpoint]],
        arena: ArenaInput,
        config: PlanningConfig,
        objective: CostMetric,
        minimum_expansion_budget: int | None = None,
    ) -> None:
        """Eagerly compute multiple directed paths across CPU cores."""
        # Workers construct Hybrid A*, so custom planners must resolve through
        # get_or_plan instead of silently being replaced during graph warmup.
        if type(self._planner) is not HybridAStarPlanner:
            return
        requested_budget = minimum_expansion_budget or config.adaptive_max_expansions
        maximum_budget = max(config.adaptive_max_expansions, requested_budget)
        timeout = config.local_planning_timeout_s
        
        tasks = []
        for start, goal in pairs:
            key = PairwiseCacheKey(arena, config, objective, start, goal)
            if key not in self._entries:
                tasks.append((start, goal, arena, config, objective, requested_budget, maximum_budget, timeout))
                
        if not tasks:
            return

        self._requests += len(tasks)
        self._misses += len(tasks)
        
        # Farm the requests out to all available CPU cores.
        # ProcessPoolExecutor naturally defaults to os.cpu_count().
        with concurrent.futures.ProcessPoolExecutor() as executor:
            results = executor.map(_parallel_plan_worker, tasks)
            
            # The results are collected sequentially in the main process,
            # meaning dictionary assignment remains 100% thread-safe.
            for start, goal, result, final_budget, attempts, c_nodes, c_time in results:
                key = PairwiseCacheKey(arena, config, objective, start, goal)
                entry = PairwiseCacheEntry(
                    key,
                    result,
                    expansion_budget=final_budget,
                    attempts=attempts,
                    cumulative_nodes_expanded=c_nodes,
                    cumulative_planning_time_s=c_time,
                )
                self._entries[key] = entry
                self._nodes_expanded += c_nodes
                self._planning_time_s += c_time
                if entry.succeeded:
                    self._successful_paths += 1
                else:
                    self._failed_paths += 1

    def get_or_plan(
        self,
        start: RouteEndpoint,
        goal: RouteEndpoint,
        arena: ArenaInput,
        config: PlanningConfig,
        objective: CostMetric,
        *,
        minimum_expansion_budget: int | None = None,
    ) -> PairwiseCacheEntry:
        key = PairwiseCacheKey(arena, config, objective, start, goal)
        self._requests += 1
        cached = self._entries.get(key)
        requested_budget = minimum_expansion_budget or config.adaptive_max_expansions
        maximum_budget = max(config.adaptive_max_expansions, requested_budget)
        
        if cached is not None and (
            cached.result.status is not LocalPlanningStatus.SEARCH_LIMIT_REACHED
            or (cached.expansion_budget or 0) >= requested_budget
        ):
            self._hits += 1
            if cached.succeeded:
                self._successful_paths += 1
            else:
                self._failed_paths += 1
            return cached

        if cached is None:
            self._misses += 1
            attempts = 0
            cumulative_nodes = 0
            cumulative_time = 0.0
            budget = config.adaptive_initial_expansions
        else:
            self._hits += 1
            attempts = cached.attempts
            cumulative_nodes = cached.cumulative_nodes_expanded
            cumulative_time = cached.cumulative_planning_time_s
            budget = min(
                maximum_budget,
                max(
                    (cached.expansion_budget or config.adaptive_initial_expansions) + 1,
                    math.ceil((cached.expansion_budget or 1) * config.adaptive_growth_factor),
                ),
            )

        deadline = time.perf_counter() + config.local_planning_timeout_s
        result = None
        final_budget = budget
        recovered = False
        while True:
            if attempts:
                self._retries += 1
            result = self._plan(start, goal, arena, config, objective, budget, deadline)
            attempts += 1
            cumulative_nodes += result.metrics.nodes_expanded
            cumulative_time += result.metrics.planning_time_s
            self._nodes_expanded += result.metrics.nodes_expanded
            self._planning_time_s += result.metrics.planning_time_s
            final_budget = budget
            if result.status not in (LocalPlanningStatus.SEARCH_LIMIT_REACHED,):
                recovered = attempts > 1 and result.succeeded
                break
            if budget >= requested_budget or budget >= maximum_budget:
                break
            if time.perf_counter() >= deadline:
                break
            budget = min(
                requested_budget,
                maximum_budget,
                max(budget + 1, math.ceil(budget * config.adaptive_growth_factor)),
            )

        assert result is not None
        entry = PairwiseCacheEntry(
            key,
            result,
            expansion_budget=final_budget,
            attempts=attempts,
            cumulative_nodes_expanded=cumulative_nodes,
            cumulative_planning_time_s=cumulative_time,
        )
        self._entries[key] = entry
        if recovered:
            self._retry_recoveries += 1
        if entry.succeeded:
            self._successful_paths += 1
        else:
            self._failed_paths += 1
        return entry

    def _plan(
        self,
        start: RouteEndpoint,
        goal: RouteEndpoint,
        arena: ArenaInput,
        config: PlanningConfig,
        objective: CostMetric,
        budget: int,
        deadline: float,
    ):
        if isinstance(self._planner, HybridAStarPlanner):
            return self._planner.plan(
                start.pose,
                goal.pose,
                arena,
                objective=objective,
                max_expanded_nodes=budget,
                max_planning_time_s=max(0.001, deadline - time.perf_counter()),
            )
        try:
            return self._planner.plan(
                start.pose,
                goal.pose,
                arena,
                objective=objective,
                max_expanded_nodes=budget,
                max_planning_time_s=max(0.001, deadline - time.perf_counter()),
            )
        except TypeError as exc:
            if "unexpected keyword" not in str(exc):
                raise
            return self._planner.plan(start.pose, goal.pose, arena, objective=objective)


__all__ = ["DirectedPairwisePathCache"]
