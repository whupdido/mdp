from dataclasses import replace

import pytest

from algorithm.config import CameraGeometry, RobotGeometry, UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import CostMetric, Direction, PlanningStatus
from algorithm.models import ArenaInput, GridCell, Obstacle, Pose
from algorithm.simulator.objective_diagnostic import (
    DiagnosticRun,
    assess_distance_switch,
    compare_objectives,
    seeded_valid_benchmark_arenas,
)


def _arena():
    start = Pose(20.0, 20.0, 0.0)
    return ArenaInput(start, (Obstacle(1, GridCell(5, 2), Direction.WEST),))


def test_comparison_reports_correct_objective_costs_and_command_counts():
    config = replace(
        UNCALIBRATED_SIMULATION_CONFIG,
        robot=RobotGeometry(2.0, 2.0),
        camera=CameraGeometry(0.0, 0.0, 10.0),
        observation_lateral_offsets_cm=(0.0,),
        guaranteed_max_candidates_per_target=1,
    )
    runs = compare_objectives(_arena(), config, candidate_limits=(1,))
    by_objective = {run.objective: run for run in runs}
    estimated = by_objective[CostMetric.ESTIMATED_TIME]
    distance = by_objective[CostMetric.DISTANCE]

    for run in (estimated, distance):
        assert run.status is PlanningStatus.SUCCESS
        assert sum(count for _, count in run.command_counts) == len(run.command_sequence)
        assert sum(run.count(command) for command in ("FW", "BW")) == run.straight_commands
        assert sum(run.count(command) for command in ("FL", "FR", "BL", "BR")) == run.turn_commands
        assert len(run.steps) == len(run.command_sequence)
        assert run.materialized_step_cost == pytest.approx(run.steps[-1].cumulative_objective_cost)

    for step in distance.steps:
        assert step.transition_cost == pytest.approx(step.geometric_length_cm)
    for step in estimated.steps:
        assert step.transition_cost == pytest.approx(step.estimated_primitive_time_s)


def test_comparison_does_not_mutate_input_config_and_reports_candidate_caps():
    config = replace(
        UNCALIBRATED_SIMULATION_CONFIG,
        robot=RobotGeometry(2.0, 2.0),
        camera=CameraGeometry(0.0, 0.0, 10.0),
        observation_lateral_offsets_cm=(0.0,),
        guaranteed_max_candidates_per_target=1,
    )
    runs = compare_objectives(_arena(), config)
    assert tuple(run.candidate_limit for run in runs) == (1, 1, 3, 3)
    assert config.guaranteed_max_candidates_per_target == 1
    assert config.observation_lateral_offsets_cm == (0.0,)


def test_five_seeded_arenas_are_reproducible_and_have_valid_observation_candidates():
    config = replace(
        UNCALIBRATED_SIMULATION_CONFIG,
        robot=RobotGeometry(2.0, 2.0),
        camera=CameraGeometry(0.0, 0.0, 10.0),
        observation_lateral_offsets_cm=(0.0,),
        guaranteed_max_candidates_per_target=3,
    )
    first = seeded_valid_benchmark_arenas(config)
    second = seeded_valid_benchmark_arenas(config)
    assert first == second
    assert len(first) == 5
    assert all(arena.task1_issues() == () for _, arena in first)


def _synthetic_run(name, objective, *, distance, time, turns, steering, commands, status=PlanningStatus.SUCCESS):
    sequence = tuple(["FW"] * commands)
    return DiagnosticRun(
        arena_name=name,
        candidate_limit=3,
        objective=objective,
        status=status,
        target_order=(1,),
        selected_candidates=(),
        command_sequence=sequence,
        command_counts=(("FW", commands), ("BW", 0), ("FL", 0), ("FR", 0), ("BL", 0), ("BR", 0)),
        turn_commands=turns,
        straight_commands=commands - turns,
        direction_changes=0,
        steering_changes=steering,
        geometric_distance_cm=distance,
        estimated_execution_time_s=time,
        objective_cost=time if objective is CostMetric.ESTIMATED_TIME else distance,
        materialized_step_cost=0.0,
        planning_time_s=time,
        nodes_expanded=1,
        steps=(),
    )


def test_switch_gate_accepts_three_smoothness_wins_and_both_median_limits():
    names = ("open", "awkward_20cm", "meaningful_turning", "task1_example")
    runs = []
    for index, name in enumerate(names):
        runs.extend((
            _synthetic_run(name, CostMetric.ESTIMATED_TIME, distance=100, time=10, turns=5, steering=4, commands=10),
            _synthetic_run(name, CostMetric.DISTANCE, distance=105, time=12, turns=4 if index < 3 else 5, steering=4, commands=10),
        ))
    assessment = assess_distance_switch(runs)
    assert assessment.should_switch
    assert assessment.distance_successes == 4
    assert assessment.smoothness_wins == 3
    assert assessment.median_distance_change_percent == pytest.approx(5.0)
    assert assessment.median_planning_time_change_percent == pytest.approx(20.0)


def test_switch_gate_rejects_failure_distance_and_planning_time_regressions():
    names = ("open", "awkward_20cm", "meaningful_turning", "task1_example")
    runs = []
    for name in names:
        runs.extend((
            _synthetic_run(name, CostMetric.ESTIMATED_TIME, distance=100, time=10, turns=5, steering=4, commands=10),
            _synthetic_run(name, CostMetric.DISTANCE, distance=106, time=13, turns=4, steering=3, commands=9,
                           status=PlanningStatus.NO_FEASIBLE_ROUTE if name == "open" else PlanningStatus.SUCCESS),
        ))
    assessment = assess_distance_switch(runs)
    assert not assessment.should_switch
    assert assessment.distance_successes == 3
    assert len(assessment.reasons) == 3


def test_switch_gate_requires_smoothness_improvement_in_three_core_arenas():
    names = ("open", "awkward_20cm", "meaningful_turning", "task1_example")
    runs = []
    for index, name in enumerate(names):
        runs.extend((
            _synthetic_run(name, CostMetric.ESTIMATED_TIME, distance=100, time=10, turns=5, steering=4, commands=10),
            _synthetic_run(name, CostMetric.DISTANCE, distance=100, time=10, turns=4 if index < 2 else 5,
                           steering=4, commands=10),
        ))
    assessment = assess_distance_switch(runs)
    assert not assessment.should_switch
    assert assessment.smoothness_wins == 2
    assert "smoothness improved on 2/4 core arenas" in assessment.reasons
