"""Headless simulator API. Importing this package never imports Pygame."""

from .headless import (
    HeadlessSimulator,
    PlaybackState,
    SimulationState,
    SimulationStep,
    simulation_steps_from_execution,
    simulation_steps_from_poses,
    simulation_steps_from_primitives,
)
from .viewport import WorldViewport
from .task1_editor_model import (
    DEFAULT_RANDOM_RETRY_LIMIT,
    EditorState,
    RandomAttemptDiagnostic,
    RandomBatchReport,
    RandomFailureCategory,
    RandomScenarioOutcome,
    Task1EditorController,
    analyze_random_scenarios,
    generate_command_reachable_task1_arena,
    generate_random_task1_arena,
    scenario_signature,
    task1_editor_config,
    validate_editor_arena,
)

__all__ = [
    "HeadlessSimulator",
    "DEFAULT_RANDOM_RETRY_LIMIT",
    "EditorState",
    "PlaybackState",
    "RandomAttemptDiagnostic",
    "RandomBatchReport",
    "RandomFailureCategory",
    "RandomScenarioOutcome",
    "SimulationState",
    "SimulationStep",
    "WorldViewport",
    "Task1EditorController",
    "analyze_random_scenarios",
    "generate_command_reachable_task1_arena",
    "generate_random_task1_arena",
    "scenario_signature",
    "simulation_steps_from_execution",
    "simulation_steps_from_poses",
    "simulation_steps_from_primitives",
    "task1_editor_config",
    "validate_editor_arena",
]
