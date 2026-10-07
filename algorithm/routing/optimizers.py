"""Exact and comparison global optimizers over cached directed costs."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from .models import (
    DirectedPairwiseGraph,
    PairwiseCacheEntry,
    RouteEndpoint,
    RouteOptimizationResult,
    RouteOptimizationSolution,
)


_COST_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class _PartialRoute:
    cost: float
    endpoints: tuple[RouteEndpoint, ...]
    entries: tuple[PairwiseCacheEntry, ...]

    @property
    def tie_key(self) -> tuple[int, tuple[tuple[int, int, int], ...]]:
        return (
            sum(endpoint.preference_rank for endpoint in self.endpoints),
            tuple(endpoint.stable_key for endpoint in self.endpoints),
        )


def _is_better(candidate: _PartialRoute, current: _PartialRoute | None) -> bool:
    if current is None:
        return True
    if candidate.cost < current.cost - _COST_EPSILON:
        return True
    if candidate.cost > current.cost + _COST_EPSILON:
        return False
    return candidate.tie_key < current.tie_key


def _extend(
    partial: _PartialRoute,
    endpoint: RouteEndpoint,
    entry: PairwiseCacheEntry,
) -> _PartialRoute | None:
    edge_cost = entry.selected_cost
    if edge_cost is None:
        return None
    return _PartialRoute(
        partial.cost + edge_cost,
        partial.endpoints + (endpoint,),
        partial.entries + (entry,),
    )


class ExhaustiveRouteOptimizer:
    """Enumerate target orders and solve each candidate chain exactly by DP."""

    def optimize(self, graph: DirectedPairwiseGraph) -> RouteOptimizationResult:
        target_ids = graph.target_ids
        if not target_ids:
            return RouteOptimizationResult(RouteOptimizationSolution((), (), 0.0), 1, 0)

        # --- BRANCH AND BOUND OPTIMIZATION ---
        # 1. Establish an aggressive initial upper bound using the greedy NN router.
        # This allows the exhaustive search to instantly prune permutations that are 
        # mathematically guaranteed to be slower than a path we've already found.
        nn_solution = NearestNeighbourRouteOptimizer().optimize(graph).solution
        best_complete: _PartialRoute | None = None
        global_upper_bound = math.inf
        
        if nn_solution is not None:
            global_upper_bound = nn_solution.cost
            best_complete = _PartialRoute(
                cost=nn_solution.cost,
                endpoints=nn_solution.endpoints,
                entries=nn_solution.entries,
            )

        permutations_evaluated = 0
        transitions_evaluated = 0
        # Different permutations share many leading target layers. Reusing a
        # completed prefix preserves explicit permutation evaluation while
        # avoiding identical numeric DP work at that prefix.
        prefix_layers: dict[tuple[int, ...], dict[RouteEndpoint, _PartialRoute]] = {}

        for target_order in itertools.permutations(target_ids):
            permutations_evaluated += 1
            layer: dict[RouteEndpoint, _PartialRoute] = {}
            for depth, target_id in enumerate(target_order):
                prefix = target_order[: depth + 1]
                cached_layer = prefix_layers.get(prefix)
                if cached_layer is not None:
                    layer = cached_layer
                    if not layer:
                        break
                    continue

                next_layer: dict[RouteEndpoint, _PartialRoute] = {}
                if depth == 0:
                    for endpoint in graph.candidates_for(target_id):
                        transitions_evaluated += 1
                        entry = graph.entry(graph.start, endpoint)
                        edge_cost = entry.selected_cost
                        if edge_cost is not None:
                            partial = _PartialRoute(
                                edge_cost,
                                (endpoint,),
                                (entry,),
                            )
                            # PRUNE: Only keep branches strictly competitive with our best known route
                            if partial.cost <= global_upper_bound + _COST_EPSILON:
                                next_layer[endpoint] = partial
                else:
                    for endpoint in graph.candidates_for(target_id):
                        best_endpoint: _PartialRoute | None = None
                        for previous_endpoint, partial in layer.items():
                            transitions_evaluated += 1
                            candidate = _extend(
                                partial,
                                endpoint,
                                graph.entry(previous_endpoint, endpoint),
                            )
                            # PRUNE: Abort extension if it exceeds the global upper bound
                            if candidate is not None and candidate.cost <= global_upper_bound + _COST_EPSILON:
                                if _is_better(candidate, best_endpoint):
                                    best_endpoint = candidate
                        if best_endpoint is not None:
                            next_layer[endpoint] = best_endpoint
                            
                prefix_layers[prefix] = next_layer
                layer = next_layer
                if not layer:
                    break

            for partial in layer.values():
                if len(partial.endpoints) == len(target_ids) and _is_better(partial, best_complete):
                    best_complete = partial
                    global_upper_bound = partial.cost  # TIGHTEN THE UPPER BOUND

        solution = None
        if best_complete is not None:
            solution = RouteOptimizationSolution(
                best_complete.endpoints,
                best_complete.entries,
                best_complete.cost,
            )
        return RouteOptimizationResult(solution, permutations_evaluated, transitions_evaluated)


class NearestNeighbourRouteOptimizer:
    """Greedily choose the cheapest reachable next target/candidate pair.

    Ties prefer a nominal candidate, then the stable obstacle/candidate identity.
    The method never backtracks, so it can fail even when the exact optimizer
    finds a complete route.
    """

    def optimize(self, graph: DirectedPairwiseGraph) -> RouteOptimizationResult:
        remaining = set(graph.target_ids)
        current = graph.start
        endpoints: list[RouteEndpoint] = []
        entries: list[PairwiseCacheEntry] = []
        total_cost = 0.0
        transitions_evaluated = 0

        while remaining:
            best: tuple[float, int, tuple[int, int, int], RouteEndpoint, PairwiseCacheEntry] | None = None
            for target_id in sorted(remaining):
                for endpoint in graph.candidates_for(target_id):
                    transitions_evaluated += 1
                    entry = graph.entry(current, endpoint)
                    edge_cost = entry.selected_cost
                    if edge_cost is None:
                        continue
                    rank = (
                        edge_cost,
                        endpoint.preference_rank,
                        endpoint.stable_key,
                        endpoint,
                        entry,
                    )
                    if best is None or rank[:3] < best[:3]:
                        best = rank
            if best is None:
                return RouteOptimizationResult(None, 0, transitions_evaluated)
            edge_cost, _, _, endpoint, entry = best
            endpoints.append(endpoint)
            entries.append(entry)
            total_cost += edge_cost
            current = endpoint
            assert endpoint.obstacle_id is not None
            remaining.remove(endpoint.obstacle_id)

        return RouteOptimizationResult(
            RouteOptimizationSolution(tuple(endpoints), tuple(entries), total_cost),
            0,
            transitions_evaluated,
        )


__all__ = ["ExhaustiveRouteOptimizer", "NearestNeighbourRouteOptimizer"]