"""Pygame-free editable Task 1 scenarios, diagnostics, and replanning."""

from __future__ import annotations

import random
import math
import statistics
import time
from dataclasses import dataclass, replace
from enum import Enum
from typing import Protocol

from algorithm.config import PlanningConfig
from algorithm.constants import START_ZONE_GRID_CELLS
from algorithm.coordinates import default_start_pose
from algorithm.enums import CostMetric, Direction, PlanningStatus, RoutingMode
from algorithm.geometry import is_motion_collision_free, is_pose_collision_free, propagate_motion
from algorithm.models import ArenaInput, GridCell, Obstacle, PlanningIssue, PlanningResult
from algorithm.routing import Task1Planner
from algorithm.targets import generate_arena_observation_candidates

from .headless import HeadlessSimulator, PlaybackState, simulation_steps_from_execution
from .task1_demo import task1_demo_config


REQUIRED_TASK1_TARGETS = 5
DEFAULT_RANDOM_RETRY_LIMIT = 50
START_ZONE_MAX_CELL = START_ZONE_GRID_CELLS - 1
MIN_RANDOM_CELL_SEPARATION_SQUARED = 9


class Task1PlanningFacade(Protocol):
    def plan(
        self,
        arena: ArenaInput,
        *,
        objective: CostMetric = CostMetric.ESTIMATED_TIME,
    ) -> PlanningResult: ...


class EditorState(Enum):
    EDITING = "editing"
    READY_TO_PLAN = "ready_to_plan"
    PLANNING = "planning"
    PLAN_READY = "plan_ready"
    PLAYING = "playing"
    PAUSED = "paused"
    COMPLETE = "complete"
    NO_ROUTE = "no_route"


class RandomFailureCategory(Enum):
    SUCCESS = "SUCCESS"
    INVALID_SCENARIO = "INVALID_SCENARIO"
    NO_GEOMETRIC_CANDIDATE = "NO_GEOMETRIC_CANDIDATE"
    LOCAL_REACHABILITY_FAILURE = "LOCAL_REACHABILITY_FAILURE"
    NO_COMPLETE_GLOBAL_ROUTE = "NO_COMPLETE_GLOBAL_ROUTE"
    SEARCH_LIMIT_REACHED = "SEARCH_LIMIT_REACHED"
    PLANNING_TIMEOUT = "PLANNING_TIMEOUT"


@dataclass(frozen=True, slots=True)
class RandomAttemptDiagnostic:
    attempt: int
    signature: str
    geometric_candidates: tuple[tuple[int, int], ...]
    reachable_candidates: tuple[tuple[int, int], ...]
    status: PlanningStatus
    category: RandomFailureCategory
    planning_time_s: float
    pairwise_searches: int


@dataclass(frozen=True, slots=True)
class RandomScenarioOutcome:
    arena: ArenaInput
    seed: int | None
    request_number: int
    attempts: int
    solvable_requested: bool
    planning_result: PlanningResult | None = None
    diagnostics: tuple[RandomAttemptDiagnostic, ...] = ()

    @property
    def succeeded(self) -> bool:
        return (
            not self.solvable_requested
            or self.planning_result is not None
            and self.planning_result.status is PlanningStatus.SUCCESS
        )


@dataclass(frozen=True, slots=True)
class RandomBatchReport:
    count: int
    category_counts: tuple[tuple[RandomFailureCategory, int], ...]
    average_planning_time_s: float
    median_planning_time_s: float
    total_pairwise_searches: int

    @property
    def success_count(self) -> int:
        return dict(self.category_counts).get(RandomFailureCategory.SUCCESS, 0)


@dataclass(frozen=True, slots=True)
class AssessmentBenchmarkReport:
    count: int
    category_counts: tuple[tuple[RandomFailureCategory, int], ...]
    average_planning_time_s: float
    median_planning_time_s: float
    candidate_poses_generated: int
    geometrically_valid_candidates: int
    directed_pairwise_queries: int
    successful_paths: int
    failed_paths: int
    estimated_cache_bytes: int
    percentile_95_planning_time_s: float = 0.0
    hybrid_astar_retries: int = 0
    hybrid_astar_retry_recoveries: int = 0
    total_nodes_expanded: int = 0
    tier_activation_counts: tuple[tuple[int, int], ...] = ()

    @property
    def complete_successes(self) -> int:
        return dict(self.category_counts).get(RandomFailureCategory.SUCCESS, 0)


@dataclass(frozen=True, slots=True)
class PerturbationBenchmarkReport:
    tested: int
    successes: int
    lost_one_candidate: int
    no_geometric_candidate: int
    no_path: int
    search_limit_reached: int
    global_connectivity_failure: int
    hybrid_astar_retries: int = 0
    hybrid_astar_retry_recoveries: int = 0
    total_nodes_expanded: int = 0
    successful_tiers: tuple[int, ...] = ()


def task1_editor_config() -> PlanningConfig:
    """Return the full candidate profile used lazily by the B.2 editor."""
    base = task1_demo_config()
    return replace(
        base,
        observation_lateral_offsets_cm=(0.0, -10.0, 10.0),
        observation_standoff_distances_cm=(20.0, 10.0, 30.0),
        guaranteed_max_candidates_per_target=9,
        adaptive_initial_expansions=20,
        adaptive_max_expansions=5000,
        adaptive_growth_factor=5.0,
        local_planning_timeout_s=5.0,
        overall_planning_timeout_s=60.0,
        turn_angles_deg=(30.0, 45.0, 60.0, 90.0),
        search_turn_angles_deg=(30.0,),
        heading_bin_rad=math.radians(15.0),
    )


def scenario_signature(arena: ArenaInput) -> str:
    """Return a stable cross-process signature without Python's hash()."""
    return "|".join(
        f"{item.obstacle_id}@{item.cell.x:02d},{item.cell.y:02d}:{item.face.value if item.face else '-'}"
        for item in sorted(arena.obstacles, key=lambda obstacle: obstacle.obstacle_id)
    )


def validate_editor_arena(
    arena: ArenaInput,
    config: PlanningConfig,
) -> tuple[PlanningIssue, ...]:
    issues: list[PlanningIssue] = []
    if len(arena.obstacles) != REQUIRED_TASK1_TARGETS:
        issues.append(
            PlanningIssue(
                "incorrect_target_count",
                f"Task 1 requires exactly {REQUIRED_TASK1_TARGETS} targets; found {len(arena.obstacles)}",
            )
        )
    issues.extend(arena.task1_issues())
    for obstacle in arena.obstacles:
        if obstacle.cell.x <= START_ZONE_MAX_CELL and obstacle.cell.y <= START_ZONE_MAX_CELL:
            issues.append(
                PlanningIssue(
                    "obstacle_in_start_zone",
                    f"target {obstacle.obstacle_id} occupies the protected 4 x 4 start zone",
                    obstacle_id=obstacle.obstacle_id,
                )
            )
    if not is_pose_collision_free(arena.start_pose, arena, config):
        issues.append(
            PlanningIssue(
                "robot_start_collision",
                "the edited obstacles collide with the robot at its authoritative start pose",
            )
        )
    return tuple(issues)


def generate_random_task1_arena(
    config: PlanningConfig,
    *,
    seed: int | None = None,
    rng: random.Random | None = None,
) -> ArenaInput:
    """Generate five unique interior targets without claiming routability."""
    if rng is not None and seed is not None:
        raise ValueError("provide either seed or rng, not both")
    source = rng or random.Random(seed)
    cells = tuple(GridCell(x, y) for x in range(5, 15) for y in range(5, 15))
    selected = source.sample(cells, REQUIRED_TASK1_TARGETS)
    obstacles = tuple(
        Obstacle(index, cell, source.choice(tuple(Direction)))
        for index, cell in enumerate(selected, start=1)
    )
    return ArenaInput(default_start_pose(config.robot), obstacles)


def generate_assessment_like_task1_arena(
    config: PlanningConfig,
    *,
    seed: int | None = None,
    rng: random.Random | None = None,
) -> ArenaInput:
    """Generate independently sampled five-target course-shaped input.

    The repository establishes only the 20 x 20 arena, 10 cm obstacles, five
    targets, cardinal image faces, and protected 4 x 4 start zone.  No planner
    walk or reachability test influences this distribution.
    """
    if rng is not None and seed is not None:
        raise ValueError("provide either seed or rng, not both")
    source = rng or random.Random(seed)
    cells = tuple(
        GridCell(x, y)
        for x in range(20)
        for y in range(20)
        if not (x <= START_ZONE_MAX_CELL and y <= START_ZONE_MAX_CELL)
    )
    selected = source.sample(cells, REQUIRED_TASK1_TARGETS)
    return ArenaInput(
        default_start_pose(config.robot),
        tuple(
            Obstacle(index, cell, source.choice(tuple(Direction)))
            for index, cell in enumerate(selected, start=1)
        ),
    )


def generate_command_reachable_task1_arena(
    config: PlanningConfig,
    *,
    rng: random.Random,
    generation_retries: int = 100,
) -> ArenaInput:
    """Generate diverse coordinates from random executable command walks.

    These are proposal heuristics only. The complete Task 1 planner remains
    the authority that accepts or rejects the resulting arena.
    """
    if generation_retries <= 0:
        raise ValueError("generation_retries must be positive")
    for _ in range(generation_retries):
        arena = _try_command_walk_arena(config, rng)
        if arena is not None:
            return arena
    raise RuntimeError("could not construct a command-walk random arena")


def _try_command_walk_arena(config: PlanningConfig, rng: random.Random) -> ArenaInput | None:
    start = default_start_pose(config.robot)
    current = start
    obstacles: list[Obstacle] = []
    legs: list[tuple[object, tuple[str, ...], object]] = []
    for target_id in range(1, REQUIRED_TASK1_TARGETS + 1):
        placed = False
        for _ in range(30):
            if target_id == 1:
                commands = (
                    ("FW",) * rng.randint(0, 2)
                    + ("FR",)
                )
            else:
                commands = (
                    ("BW",) * rng.randint(1, 2)
                    + (rng.choice(("FL", "FR")),)
                )
            existing_arena = ArenaInput(start, tuple(obstacles))
            reached = _apply_commands(current, commands, existing_arena, config)
            if reached is None:
                continue
            obstacle = _obstacle_for_observation(target_id, reached, config)
            if obstacle is None or any(
                (obstacle.cell.x - item.cell.x) ** 2 + (obstacle.cell.y - item.cell.y) ** 2
                < MIN_RANDOM_CELL_SEPARATION_SQUARED
                for item in obstacles
            ):
                continue
            candidate_arena = ArenaInput(start, tuple(obstacles) + (obstacle,))
            group = generate_arena_observation_candidates(candidate_arena, config)[-1]
            matching = next(
                (
                    candidate
                    for candidate in group.candidates
                    if candidate.valid
                    and candidate.observation_pose.pose.heading_rad == reached.heading_rad
                    and (
                        (candidate.observation_pose.pose.x_cm - reached.x_cm) ** 2
                        + (candidate.observation_pose.pose.y_cm - reached.y_cm) ** 2
                    ) ** 0.5
                    <= config.goal_position_tolerance_cm
                ),
                None,
            )
            if matching is None:
                continue
            obstacles.append(obstacle)
            goal_pose = matching.observation_pose.pose
            legs.append((current, commands, goal_pose))
            current = goal_pose
            placed = True
            break
        if not placed:
            return None
    arena = ArenaInput(start, tuple(obstacles))
    groups = generate_arena_observation_candidates(arena, config)
    if any(not group.has_valid_candidate for group in groups):
        return None
    for leg_start, commands, goal_pose in legs:
        reached = _apply_commands(leg_start, commands, arena, config)
        if reached is None or reached.heading_rad != goal_pose.heading_rad:
            return None
        if (
            (reached.x_cm - goal_pose.x_cm) ** 2 + (reached.y_cm - goal_pose.y_cm) ** 2
        ) ** 0.5 > config.goal_position_tolerance_cm:
            return None
    shuffled_ids = list(range(1, REQUIRED_TASK1_TARGETS + 1))
    rng.shuffle(shuffled_ids)
    randomized_obstacles = tuple(
        Obstacle(shuffled_ids[index], obstacle.cell, obstacle.face)
        for index, obstacle in enumerate(obstacles)
    )
    return ArenaInput(start, randomized_obstacles)


def _apply_commands(current, commands, arena, config):
    pose = current
    for command in commands:
        primitive = config.motion.primitives_for(command)[0]
        if not is_motion_collision_free(pose, primitive, arena, config):
            return None
        pose = propagate_motion(pose, primitive, config)
    return pose


def _obstacle_for_observation(target_id: int, pose, config) -> Obstacle | None:
    try:
        heading = Direction.from_heading_rad(
            pose.heading_rad,
            tolerance_rad=config.goal_heading_tolerance_rad,
        )
    except ValueError:
        return None
    face = heading.opposite()
    target_x, target_y = pose.translated_local(
        config.camera.forward_offset_cm + config.camera.image_gap_cm,
        config.camera.left_offset_cm,
    )
    cell_size = config.cell_size_cm

    def nearest(value: float) -> int:
        return int(value + 0.5)

    if face is Direction.WEST:
        cell = (nearest(target_x / cell_size), nearest(target_y / cell_size - 0.5))
    elif face is Direction.EAST:
        cell = (nearest(target_x / cell_size - 1.0), nearest(target_y / cell_size - 0.5))
    elif face is Direction.SOUTH:
        cell = (nearest(target_x / cell_size - 0.5), nearest(target_y / cell_size))
    else:
        cell = (nearest(target_x / cell_size - 0.5), nearest(target_y / cell_size - 1.0))
    try:
        grid_cell = GridCell(*cell)
    except (TypeError, ValueError):
        return None
    if grid_cell.x <= START_ZONE_MAX_CELL and grid_cell.y <= START_ZONE_MAX_CELL:
        return None
    return Obstacle(target_id, grid_cell, face)


def classify_planning_result(result: PlanningResult) -> RandomFailureCategory:
    if result.status is PlanningStatus.SUCCESS:
        return RandomFailureCategory.SUCCESS
    codes = {issue.code for issue in result.issues}
    if result.status is PlanningStatus.INVALID_INPUT:
        return RandomFailureCategory.INVALID_SCENARIO
    if "no_geometrically_valid_observation_pose" in codes:
        return RandomFailureCategory.NO_GEOMETRIC_CANDIDATE
    if "pairwise_search_limit_reached" in codes:
        return RandomFailureCategory.SEARCH_LIMIT_REACHED
    if result.status is PlanningStatus.PLANNING_TIMEOUT or "task1_planning_budget_reached" in codes:
        return RandomFailureCategory.PLANNING_TIMEOUT
    if "no_reachable_observation_pose" in codes:
        return RandomFailureCategory.LOCAL_REACHABILITY_FAILURE
    return RandomFailureCategory.NO_COMPLETE_GLOBAL_ROUTE


def _plan_for_b2(facade: Task1PlanningFacade, arena: ArenaInput) -> PlanningResult:
    if isinstance(facade, Task1Planner):
        return facade.plan(arena, routing_mode=RoutingMode.FEASIBILITY)
    return facade.plan(arena)


def _attempt_diagnostic(attempt, arena, result, planning_time_s, config):
    groups = generate_arena_observation_candidates(arena, config)
    geometric = tuple(
        (group.obstacle_id, sum(candidate.valid for candidate in group.candidates))
        for group in groups
    )
    reachable_by_id = {
        item.target_id: item.reachable_candidates for item in result.metrics.target_reachability
    }
    return RandomAttemptDiagnostic(
        attempt=attempt,
        signature=scenario_signature(arena),
        geometric_candidates=geometric,
        reachable_candidates=tuple(
            (target_id, reachable_by_id.get(target_id, 0)) for target_id, _ in geometric
        ),
        status=result.status,
        category=classify_planning_result(result),
        planning_time_s=planning_time_s,
        pairwise_searches=result.metrics.local_paths_requested,
    )


def analyze_random_scenarios(
    config: PlanningConfig,
    *,
    count: int = 100,
    seed: int = 6200,
    planner: Task1PlanningFacade | None = None,
) -> RandomBatchReport:
    if count <= 0:
        raise ValueError("count must be positive")
    source = random.Random(seed)
    facade = planner or Task1Planner(config)
    categories = {category: 0 for category in RandomFailureCategory}
    times: list[float] = []
    pairwise_searches = 0
    for _ in range(count):
        arena = generate_random_task1_arena(config, rng=source)
        started = time.perf_counter()
        result = _plan_for_b2(facade, arena)
        elapsed = time.perf_counter() - started
        categories[classify_planning_result(result)] += 1
        times.append(elapsed)
        pairwise_searches += result.metrics.local_paths_requested
    return RandomBatchReport(
        count=count,
        category_counts=tuple(categories.items()),
        average_planning_time_s=statistics.fmean(times),
        median_planning_time_s=statistics.median(times),
        total_pairwise_searches=pairwise_searches,
    )


def benchmark_assessment_like_scenarios(
    config: PlanningConfig,
    *,
    count: int = 100,
    seed: int = 6300,
    planner: Task1PlanningFacade | None = None,
) -> AssessmentBenchmarkReport:
    """Benchmark independent assessment-like inputs without planner-biased generation."""
    if count <= 0:
        raise ValueError("count must be positive")
    source = random.Random(seed)
    facade = planner or Task1Planner(config)
    categories = {category: 0 for category in RandomFailureCategory}
    times: list[float] = []
    candidates = valid_candidates = queries = successes = failures = 0
    retries = recoveries = nodes_expanded = 0
    tier_counts: dict[int, int] = {}
    for _ in range(count):
        arena = generate_assessment_like_task1_arena(config, rng=source)
        groups = generate_arena_observation_candidates(arena, config)
        candidates += sum(len(group.candidates) for group in groups)
        valid_candidates += sum(candidate.valid for group in groups for candidate in group.candidates)
        started = time.perf_counter()
        result = _plan_for_b2(facade, arena)
        times.append(time.perf_counter() - started)
        categories[classify_planning_result(result)] += 1
        queries += result.metrics.local_paths_requested
        successes += result.metrics.local_paths_succeeded
        failures += result.metrics.unreachable_pairwise_paths
        retries += result.metrics.hybrid_astar_retries
        recoveries += result.metrics.hybrid_astar_retry_recoveries
        nodes_expanded += result.metrics.total_nodes_expanded
        tiers = result.metrics.candidate_tiers_activated
        tier_counts[tiers] = tier_counts.get(tiers, 0) + 1
    return AssessmentBenchmarkReport(
        count=count,
        category_counts=tuple(categories.items()),
        average_planning_time_s=statistics.fmean(times),
        median_planning_time_s=statistics.median(times),
        candidate_poses_generated=candidates,
        geometrically_valid_candidates=valid_candidates,
        directed_pairwise_queries=queries,
        successful_paths=successes,
        failed_paths=failures,
        # A transparent shallow diagnostic estimate, not a process RSS claim.
        estimated_cache_bytes=queries * 256,
        percentile_95_planning_time_s=(
            statistics.quantiles(times, n=20, method="inclusive")[18]
            if len(times) > 1 else times[0]
        ),
        hybrid_astar_retries=retries,
        hybrid_astar_retry_recoveries=recoveries,
        total_nodes_expanded=nodes_expanded,
        tier_activation_counts=tuple(sorted(tier_counts.items())),
    )


def benchmark_one_cell_perturbations(
    arena: ArenaInput,
    config: PlanningConfig,
    *,
    planner: Task1PlanningFacade | None = None,
) -> PerturbationBenchmarkReport:
    """Move each obstacle by one cardinal grid cell and classify every valid pair."""
    facade = planner or Task1Planner(config)
    baseline_groups = generate_arena_observation_candidates(arena, config)
    baseline_valid = {
        group.obstacle_id: sum(candidate.valid for candidate in group.candidates)
        for group in baseline_groups
    }
    tested = successes = lost_one = no_candidate = no_path = limits = global_failure = 0
    retries = recoveries = nodes_expanded = 0
    successful_tiers: list[int] = []
    occupied = {item.cell for item in arena.obstacles}
    for obstacle in arena.obstacles:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            x, y = obstacle.cell.x + dx, obstacle.cell.y + dy
            if not (0 <= x < 20 and 0 <= y < 20):
                continue
            cell = GridCell(x, y)
            if cell in occupied or (x <= START_ZONE_MAX_CELL and y <= START_ZONE_MAX_CELL):
                continue
            moved = tuple(
                Obstacle(item.obstacle_id, cell, item.face, item.image_id)
                if item.obstacle_id == obstacle.obstacle_id else item
                for item in arena.obstacles
            )
            candidate_arena = ArenaInput(arena.start_pose, moved)
            groups = generate_arena_observation_candidates(candidate_arena, config)
            current_valid = {
                group.obstacle_id: sum(candidate.valid for candidate in group.candidates)
                for group in groups
            }
            if sum(baseline_valid.values()) - sum(current_valid.values()) == 1:
                lost_one += 1
            result = _plan_for_b2(facade, candidate_arena)
            retries += result.metrics.hybrid_astar_retries
            recoveries += result.metrics.hybrid_astar_retry_recoveries
            nodes_expanded += result.metrics.total_nodes_expanded
            tested += 1
            category = classify_planning_result(result)
            if category is RandomFailureCategory.SUCCESS:
                successes += 1
                successful_tiers.append(result.metrics.candidate_tiers_activated)
            elif category is RandomFailureCategory.NO_GEOMETRIC_CANDIDATE:
                no_candidate += 1
            elif category is RandomFailureCategory.SEARCH_LIMIT_REACHED:
                limits += 1
            elif category is RandomFailureCategory.NO_COMPLETE_GLOBAL_ROUTE:
                global_failure += 1
            else:
                no_path += 1
    return PerturbationBenchmarkReport(
        tested,
        successes,
        lost_one,
        no_candidate,
        no_path,
        limits,
        global_failure,
        retries,
        recoveries,
        nodes_expanded,
        tuple(successful_tiers),
    )


class Task1EditorController:
    """Own editable arena state; route logic remains in :class:`Task1Planner`."""

    def __init__(
        self,
        config: PlanningConfig | None = None,
        *,
        obstacles: tuple[Obstacle, ...] = (),
        planner: Task1PlanningFacade | None = None,
        random_seed: int | None = None,
    ) -> None:
        self.config = config or task1_editor_config()
        self._planner = planner or Task1Planner(self.config)
        # Shift+F5 proposals are constructed around their preferred 20C poses.
        # Verify and retain them with the same bounded tier-1 physical profile;
        # manually edited arenas continue through the full adaptive B.2 planner.
        self._random_solvable_planner = (
            self._planner
            if planner is not None
            else Task1Planner(
                replace(
                    self.config,
                    observation_lateral_offsets_cm=(0.0,),
                    observation_standoff_distances_cm=(self.config.camera.image_gap_cm,),
                    guaranteed_max_candidates_per_target=1,
                    adaptive_initial_expansions=20,
                    adaptive_max_expansions=20,
                    overall_planning_timeout_s=15.0,
                    search_turn_angles_deg=(90.0,),
                    turn_angles_deg=(30.0, 45.0, 60.0, 90.0),
                ),
                routing_mode=RoutingMode.FEASIBILITY,
            )
        )
        self._rng = random.Random(random_seed)
        self._random_request_count = 0
        self._last_random_arena: ArenaInput | None = None
        self._accepted_random_signatures: set[str] = set()
        self._obstacles = {obstacle.obstacle_id: obstacle for obstacle in obstacles}
        if len(self._obstacles) != len(obstacles):
            raise ValueError("obstacle IDs must be unique")
        self._arena = self._make_arena()
        self.planning_result: PlanningResult | None = None
        self.simulator: HeadlessSimulator | None = None
        self.state = EditorState.EDITING
        self.status_message = "Add exactly five targets"
        self.random_attempts = 0
        self.last_random_diagnostics: tuple[RandomAttemptDiagnostic, ...] = ()
        self._refresh_editing_state()

    @property
    def arena(self) -> ArenaInput:
        return self._arena

    @property
    def obstacles(self) -> tuple[Obstacle, ...]:
        return self._arena.obstacles

    @property
    def can_edit(self) -> bool:
        return self.state not in {EditorState.PLANNING, EditorState.PLAYING, EditorState.PAUSED}

    @property
    def validation_issues(self) -> tuple[PlanningIssue, ...]:
        return validate_editor_arena(self._arena, self.config)

    def add_obstacle(self, obstacle_id: int, cell: GridCell, face: Direction) -> None:
        self._require_editable()
        if obstacle_id in self._obstacles:
            raise ValueError(f"target ID {obstacle_id} already exists")
        if len(self._obstacles) >= REQUIRED_TASK1_TARGETS:
            raise ValueError("the Task 1 editor accepts exactly five targets")
        updated = dict(self._obstacles)
        updated[obstacle_id] = Obstacle(obstacle_id, cell, face)
        self._replace_obstacles(updated)

    def move_obstacle(self, obstacle_id: int, cell: GridCell) -> None:
        self._require_editable()
        obstacle = self._required_obstacle(obstacle_id)
        self._replace_one(Obstacle(obstacle_id, cell, obstacle.face, obstacle.image_id))

    def change_face(self, obstacle_id: int, face: Direction) -> None:
        self._require_editable()
        obstacle = self._required_obstacle(obstacle_id)
        self._replace_one(Obstacle(obstacle_id, obstacle.cell, face, obstacle.image_id))

    def remove_obstacle(self, obstacle_id: int) -> None:
        self._require_editable()
        self._required_obstacle(obstacle_id)
        updated = dict(self._obstacles)
        del updated[obstacle_id]
        self._replace_obstacles(updated)

    def plan(self) -> PlanningResult:
        if self.state in {EditorState.PLAYING, EditorState.PAUSED}:
            raise RuntimeError("reset active playback before planning again")
        print(f"Scenario signature: {scenario_signature(self._arena)}")
        issues = self.validation_issues
        if issues:
            result = PlanningResult(PlanningStatus.INVALID_INPUT, issues=issues)
            self._set_failed_result(result)
            return result
        self.state = EditorState.PLANNING
        self.status_message = "Planning all five targets..."
        result = self._plan_arena(self._arena)
        self._accept_result(self._arena, result)
        return result

    def announce_planning(self) -> bool:
        if self.state in {EditorState.PLAYING, EditorState.PAUSED}:
            raise RuntimeError("reset active playback before planning again")
        issues = self.validation_issues
        if issues:
            self.state = EditorState.EDITING
            self.status_message = issues[0].message
            return False
        self.state = EditorState.PLANNING
        self.status_message = "Planning all five targets..."
        return True

    def randomize(
        self,
        *,
        seed: int | None = None,
        require_solvable: bool = False,
        retry_limit: int = DEFAULT_RANDOM_RETRY_LIMIT,
    ) -> RandomScenarioOutcome:
        self._require_editable()
        if isinstance(retry_limit, bool) or not isinstance(retry_limit, int) or retry_limit <= 0:
            raise ValueError("retry_limit must be a positive integer")
        self._random_request_count += 1
        source = random.Random(seed) if seed is not None else self._rng
        request = self._random_request_count
        identifier = f"explicit-seed={seed}" if seed is not None else "persistent-unseeded-stream"
        print(f"Random {'solvable' if require_solvable else 'raw'} request #{request}")
        print(f"RNG: {identifier}")
        print(f"Attempt limit: {retry_limit}")
        if not require_solvable:
            arena = generate_random_task1_arena(self.config, rng=source)
            self._replace_arena(arena)
            self._last_random_arena = arena
            self.status_message = f"Raw random scenario ready; signature {scenario_signature(arena)}"
            print(f"Scenario signature: {scenario_signature(arena)}")
            return RandomScenarioOutcome(arena, seed, request, 1, False)

        previous_arena = self._arena
        diagnostics: list[RandomAttemptDiagnostic] = []
        last_result: PlanningResult | None = None
        for attempt in range(1, retry_limit + 1):
            for _ in range(100):
                arena = generate_command_reachable_task1_arena(self.config, rng=source)
                if seed is not None:
                    break
                signature = scenario_signature(arena)
                repeated = signature in self._accepted_random_signatures
                too_similar = (
                    self._last_random_arena is not None
                    and _coordinate_difference_count(arena, self._last_random_arena) < 2
                )
                if not repeated and not too_similar:
                    break
            else:
                raise RuntimeError("could not generate a diverse solvable proposal")
            started = time.perf_counter()
            result = self._plan_arena(arena, solvable_generation=True)
            elapsed = time.perf_counter() - started
            diagnostic = _attempt_diagnostic(attempt, arena, result, elapsed, self.config)
            diagnostics.append(diagnostic)
            _print_attempt_diagnostic(diagnostic, arena)
            last_result = result
            if result.status is PlanningStatus.SUCCESS:
                self._replace_arena(arena)
                self._accept_result(arena, result)
                self._last_random_arena = arena
                self._accepted_random_signatures.add(scenario_signature(arena))
                self.random_attempts = attempt
                self.last_random_diagnostics = tuple(diagnostics)
                self.status_message = f"Random solvable plan accepted on attempt {attempt}"
                print(f"Accepted on attempt {attempt}")
                return RandomScenarioOutcome(
                    arena, seed, request, attempt, True, result, tuple(diagnostics)
                )
        self.random_attempts = retry_limit
        self.last_random_diagnostics = tuple(diagnostics)
        self.status_message = (
            f"RANDOM SOLVABLE GENERATION FAILED after {retry_limit} attempts; "
            "previous scenario preserved; press Shift+F5 to try again"
        )
        print(f"Failed after {retry_limit} attempts")
        return RandomScenarioOutcome(
            previous_arena, seed, request, retry_limit, True, last_result, tuple(diagnostics)
        )

    def play_pause(self) -> None:
        if self.simulator is None:
            return
        if self.state is EditorState.PLAYING:
            self.simulator.pause()
            self.state = EditorState.PAUSED
        elif self.state in {EditorState.PLAN_READY, EditorState.PAUSED}:
            self.simulator.play()
            self.state = EditorState.PLAYING

    def step_primitive(self) -> bool:
        if self.simulator is None or self.state is EditorState.PLAYING:
            return False
        advanced = self.simulator.step_primitive()
        self._sync_playback_state()
        return advanced

    def advance(self, elapsed_s: float) -> None:
        if self.simulator is not None:
            self.simulator.advance(elapsed_s)
            self._sync_playback_state()

    def reset_playback(self) -> None:
        if self.simulator is not None:
            self.simulator.reset()
            self.state = EditorState.PLAN_READY
            self.status_message = "Playback reset; plan remains valid"

    def preview_simulator(self) -> HeadlessSimulator:
        if self.simulator is not None:
            return self.simulator
        groups = generate_arena_observation_candidates(self._arena, self.config)
        return HeadlessSimulator(self._arena, groups, (), planned_path=(self._arena.start_pose,))

    def _accept_result(self, arena: ArenaInput, result: PlanningResult) -> None:
        self.planning_result = result
        if result.status is not PlanningStatus.SUCCESS or result.route is None:
            self._set_failed_result(result)
            return
        route = result.route
        groups = generate_arena_observation_candidates(arena, self.config)
        self.simulator = HeadlessSimulator(
            arena,
            groups,
            simulation_steps_from_execution(route.execution_steps, self.config),
            planned_path=route.sampled_poses,
            target_order=route.target_order,
            selected_candidates=tuple(zip(route.target_order, route.selected_candidate_kinds)),
        )
        self.state = EditorState.PLAN_READY
        self.status_message = "Plan ready; press Space to play"

    def _plan_arena(
        self,
        arena: ArenaInput,
        *,
        solvable_generation: bool = False,
    ) -> PlanningResult:
        planner = self._random_solvable_planner if solvable_generation else self._planner
        if isinstance(planner, Task1Planner):
            return planner.plan(
                arena,
                objective=CostMetric.ESTIMATED_TIME,
                routing_mode=RoutingMode.FEASIBILITY,
            )
        return planner.plan(arena, objective=CostMetric.ESTIMATED_TIME)

    def _set_failed_result(self, result: PlanningResult) -> None:
        self.planning_result = result
        self.simulator = None
        self.state = EditorState.NO_ROUTE
        self.status_message = result.issues[0].message

    def _make_arena(self) -> ArenaInput:
        return ArenaInput(
            default_start_pose(self.config.robot),
            tuple(sorted(self._obstacles.values(), key=lambda item: item.obstacle_id)),
        )

    def _replace_one(self, obstacle: Obstacle) -> None:
        updated = dict(self._obstacles)
        updated[obstacle.obstacle_id] = obstacle
        self._replace_obstacles(updated)

    def _replace_arena(self, arena: ArenaInput) -> None:
        self._replace_obstacles({item.obstacle_id: item for item in arena.obstacles})

    def _replace_obstacles(self, obstacles: dict[int, Obstacle]) -> None:
        previous = self._obstacles
        self._obstacles = obstacles
        try:
            arena = self._make_arena()
        except Exception:
            self._obstacles = previous
            raise
        self._arena = arena
        self._invalidate_plan()

    def _invalidate_plan(self) -> None:
        self.planning_result = None
        self.simulator = None
        self._refresh_editing_state()

    def _refresh_editing_state(self) -> None:
        issues = self.validation_issues
        if issues:
            self.state = EditorState.EDITING
            self.status_message = issues[0].message
        else:
            self.state = EditorState.READY_TO_PLAN
            self.status_message = "Scenario changed; press Enter to plan"

    def _required_obstacle(self, obstacle_id: int) -> Obstacle:
        try:
            return self._obstacles[obstacle_id]
        except KeyError as exc:
            raise KeyError(f"unknown target ID {obstacle_id}") from exc

    def _require_editable(self) -> None:
        if not self.can_edit:
            raise RuntimeError("stop active playback before editing the arena")

    def _sync_playback_state(self) -> None:
        if self.simulator is None:
            return
        state = self.simulator.state.playback_state
        if state is PlaybackState.COMPLETE:
            self.state = EditorState.COMPLETE
            self.status_message = "Complete: all planned captures executed"
        elif state is PlaybackState.PLAYING:
            self.state = EditorState.PLAYING
        elif state is PlaybackState.PAUSED:
            self.state = EditorState.PAUSED
        else:
            self.state = EditorState.PLAN_READY


def _coordinate_difference_count(first: ArenaInput, second: ArenaInput) -> int:
    second_cells = {item.obstacle_id: item.cell for item in second.obstacles}
    return sum(item.cell != second_cells.get(item.obstacle_id) for item in first.obstacles)


def _print_attempt_diagnostic(diagnostic: RandomAttemptDiagnostic, arena: ArenaInput) -> None:
    print(f"Attempt {diagnostic.attempt}: {diagnostic.signature}")
    print(
        "  Obstacles:",
        ", ".join(
            f"{item.obstacle_id}=({item.cell.x},{item.cell.y},{item.face.value})"
            for item in arena.obstacles
        ),
    )
    geometric = dict(diagnostic.geometric_candidates)
    reachable = dict(diagnostic.reachable_candidates)
    for target_id in sorted(geometric):
        print(
            f"  Target {target_id}: geometric={geometric[target_id]} "
            f"reachable={reachable[target_id]}"
        )
    print(
        f"  Status={diagnostic.status.value} category={diagnostic.category.value} "
        f"pairwise={diagnostic.pairwise_searches} time={diagnostic.planning_time_s:.3f}s"
    )


__all__ = [
    "DEFAULT_RANDOM_RETRY_LIMIT",
    "AssessmentBenchmarkReport",
    "EditorState",
    "RandomAttemptDiagnostic",
    "RandomBatchReport",
    "RandomFailureCategory",
    "RandomScenarioOutcome",
    "PerturbationBenchmarkReport",
    "Task1EditorController",
    "analyze_random_scenarios",
    "benchmark_assessment_like_scenarios",
    "benchmark_one_cell_perturbations",
    "classify_planning_result",
    "generate_command_reachable_task1_arena",
    "generate_assessment_like_task1_arena",
    "generate_random_task1_arena",
    "scenario_signature",
    "task1_editor_config",
    "validate_editor_arena",
]
