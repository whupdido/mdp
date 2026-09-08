import math
import subprocess
import sys

import pytest

from algorithm.config import UNCALIBRATED_SIMULATION_CONFIG
from algorithm.enums import Direction
from algorithm.geometry import propagate_motion
from algorithm.models import ArenaInput, GridCell, Obstacle, Pose
from algorithm.simulator import (
    HeadlessSimulator,
    PlaybackState,
    SimulationStep,
    WorldViewport,
    simulation_steps_from_poses,
    simulation_steps_from_primitives,
)
from algorithm.simulator.demo import DEMO_COMMAND_SEQUENCE, build_demo_simulator
from algorithm.targets import generate_arena_observation_candidates


CONFIG = UNCALIBRATED_SIMULATION_CONFIG
START = Pose.from_direction(100.0, 100.0, Direction.NORTH)


def primitive(command: str):
    return CONFIG.motion.primitives_for(command)[0]


def simulator_with(
    *steps: SimulationStep,
    arena: ArenaInput | None = None,
) -> HeadlessSimulator:
    actual_arena = arena or ArenaInput(START)
    groups = generate_arena_observation_candidates(actual_arena, CONFIG)
    return HeadlessSimulator(actual_arena, groups, tuple(steps))


def test_initial_snapshot_preserves_input_and_separates_route_layers():
    arena = ArenaInput(START, (Obstacle(1, GridCell(17, 17), Direction.WEST),))
    groups = generate_arena_observation_candidates(arena, CONFIG)
    destination = Pose(100.0, 110.0, START.heading_rad)
    simulator = HeadlessSimulator(
        arena,
        groups,
        (SimulationStep.motion(destination, 1.0, "FW", ends_primitive=True),),
    )

    assert simulator.state.arena is arena
    assert simulator.state.candidate_groups == groups
    assert simulator.state.initial_pose == START
    assert simulator.state.robot_pose == START
    assert simulator.state.planned_path == (START, destination)
    assert simulator.state.executed_path == (START,)
    assert simulator.state.playback_state is PlaybackState.READY


def test_play_pause_resume_and_reset_are_deterministic():
    destination = Pose(100.0, 110.0, START.heading_rad)
    simulator = simulator_with(
        SimulationStep.motion(destination, 1.0, "FW", ends_primitive=True)
    )

    simulator.advance(0.5)
    assert simulator.state.simulation_time_s == 0.0
    simulator.play()
    simulator.advance(0.4)
    simulator.pause()
    simulator.advance(10.0)
    assert simulator.state.simulation_time_s == pytest.approx(0.4)
    assert simulator.state.robot_pose == START
    simulator.play()
    simulator.advance(0.6)
    assert simulator.state.robot_pose == destination
    assert simulator.state.playback_state is PlaybackState.COMPLETE

    simulator.reset()
    assert simulator.state.robot_pose == START
    assert simulator.state.executed_path == (START,)
    assert simulator.state.simulation_time_s == 0.0
    assert simulator.state.playback_state is PlaybackState.READY


def test_advance_result_does_not_depend_on_frame_partitioning():
    poses = (Pose(100.0, 101.0, 0.0), Pose(100.0, 102.0, 0.0), Pose(100.0, 103.0, 0.0))
    steps = simulation_steps_from_poses(poses, 0.2, motion_command="FW")
    single = simulator_with(*steps)
    partitioned = simulator_with(*steps)
    single.play()
    partitioned.play()

    single.advance(0.6)
    for _ in range(6):
        partitioned.advance(0.1)

    assert partitioned.state.robot_pose == single.state.robot_pose
    assert partitioned.state.current_step_index == single.state.current_step_index
    assert partitioned.state.executed_path == single.state.executed_path
    assert partitioned.state.simulation_time_s == pytest.approx(single.state.simulation_time_s)


def test_step_primitive_finishes_one_whole_sampled_motion():
    first = primitive("FW")
    second = primitive("FR")
    steps = simulation_steps_from_primitives(START, (first, second), CONFIG)
    simulator = simulator_with(*steps)

    assert simulator.step_primitive()
    first_end = propagate_motion(START, first, CONFIG)
    assert simulator.state.robot_pose == first_end
    assert simulator.state.current_step_index == 10
    assert simulator.state.playback_state is PlaybackState.PAUSED
    assert simulator.state.current_motion_command == "FR"

    assert simulator.step_primitive()
    assert simulator.state.robot_pose == propagate_motion(first_end, second, CONFIG)
    assert simulator.state.playback_state is PlaybackState.COMPLETE


def test_executed_trail_grows_and_reset_preserves_planned_trail():
    steps = simulation_steps_from_primitives(START, (primitive("FW"),), CONFIG)
    simulator = simulator_with(*steps)
    planned_path = simulator.state.planned_path

    assert simulator.state.executed_path == (START,)
    simulator.step_once()
    assert len(simulator.state.executed_path) == 2
    assert simulator.state.planned_path == planned_path
    assert simulator.state.executed_path != simulator.state.planned_path

    simulator.reset()
    assert simulator.state.executed_path == (START,)
    assert simulator.state.planned_path == planned_path


@pytest.mark.parametrize("command", ["FW", "BW", "FL", "FR", "BL", "BR"])
def test_sampled_playback_reaches_authoritative_primitive_endpoint(command):
    motion = primitive(command)
    steps = simulation_steps_from_primitives(START, (motion,), CONFIG)
    simulator = simulator_with(*steps)
    simulator.play()
    simulator.advance(100.0)

    assert simulator.state.robot_pose == propagate_motion(START, motion, CONFIG)
    assert simulator.state.executed_path == simulator.state.planned_path
    assert simulator.state.playback_state is PlaybackState.COMPLETE


def test_capture_marks_target_visited_once_and_only_when_executed():
    arena = ArenaInput(START, (Obstacle(1, GridCell(17, 17), Direction.WEST),))
    simulator = simulator_with(
        SimulationStep.capture(1),
        SimulationStep.capture(1),
        arena=arena,
    )
    assert simulator.state.visited_target_ids == ()

    simulator.step_primitive()
    assert simulator.state.visited_target_ids == (1,)
    simulator.step_primitive()
    assert simulator.state.visited_target_ids == (1,)

    simulator.reset()
    assert simulator.state.visited_target_ids == ()


def test_backward_playback_restores_motion_and_capture_state_deterministically():
    arena = ArenaInput(START, (Obstacle(1, GridCell(17, 17), Direction.WEST),))
    # Use a short explicit timeline so the capture boundary is unambiguous.
    destination = Pose(100.0, 110.0, START.heading_rad)
    simulator = simulator_with(
        SimulationStep.motion(destination, 1.0, "FW", ends_primitive=True),
        SimulationStep.capture(1),
        SimulationStep.motion(START, 1.0, "BW", ends_primitive=True),
        arena=arena,
    )

    assert not simulator.step_backward()
    assert simulator.step_primitive()  # motion
    before_capture = simulator.state
    assert simulator.step_primitive()  # capture
    after_capture = simulator.state
    assert after_capture.visited_target_ids == (1,)
    assert simulator.step_backward()
    assert simulator.state.robot_pose == before_capture.robot_pose
    assert simulator.state.simulation_time_s == before_capture.simulation_time_s
    assert simulator.state.visited_target_ids == ()
    assert simulator.state.executed_path == before_capture.executed_path
    assert simulator.step_primitive()
    assert simulator.state == after_capture

    assert simulator.step_primitive()  # final motion -> COMPLETE
    assert simulator.state.playback_state is PlaybackState.COMPLETE
    assert simulator.step_backward()
    assert simulator.state.playback_state is PlaybackState.PAUSED
    assert simulator.state.visited_target_ids == (1,)
    assert simulator.step_primitive()
    assert simulator.state.playback_state is PlaybackState.COMPLETE


def test_unknown_capture_target_is_rejected():
    with pytest.raises(ValueError, match="unknown obstacles"):
        simulator_with(SimulationStep.capture(99))


def test_invalid_time_and_step_contracts_are_rejected():
    simulator = simulator_with()
    with pytest.raises(ValueError, match="non-negative"):
        simulator.advance(-0.1)
    with pytest.raises(ValueError, match="exactly one"):
        SimulationStep(0.0)
    with pytest.raises(ValueError, match="exactly one"):
        SimulationStep(0.0, pose=START, capture_obstacle_id=1)


def test_viewport_round_trip_and_vertical_axis_inversion():
    viewport = WorldViewport(200.0, 40.0, 20.0, 600.0)
    assert viewport.world_to_screen(0.0, 0.0) == (40.0, 620.0)
    assert viewport.world_to_screen(200.0, 200.0) == (640.0, 20.0)
    world = viewport.screen_to_world(*viewport.world_to_screen(42.5, 137.25))
    assert world == pytest.approx((42.5, 137.25))


def test_demo_is_complete_collision_checked_and_exercises_all_commands_and_captures():
    simulator, config = build_demo_simulator()
    assert all(
        1 <= coordinate <= 18
        for obstacle in simulator.state.arena.obstacles
        for coordinate in (obstacle.cell.x, obstacle.cell.y)
    )
    assert {step.motion_command for step in simulator.steps if step.motion_command} == {
        "FW", "BW", "FL", "FR", "BL", "BR"
    }
    assert [
        step.capture_obstacle_id
        for step in simulator.steps
        if step.capture_obstacle_id is not None
    ] == [2, 2]
    simulator.play()
    simulator.advance(1000.0)
    assert simulator.state.playback_state is PlaybackState.COMPLETE
    assert simulator.state.visited_target_ids == (2,)
    assert 1 not in simulator.state.visited_target_ids
    assert 3 not in simulator.state.visited_target_ids
    assert all(group.has_valid_candidate for group in simulator.state.candidate_groups)
    assert all(group.has_valid_candidate for group in simulator.state.candidate_groups)

    pose = simulator.state.initial_pose
    capture_poses = {}
    for step in simulator.steps:
        if step.pose is not None:
            pose = step.pose
        elif step.capture_obstacle_id is not None:
            capture_poses[step.capture_obstacle_id] = pose
    groups_by_id = {group.obstacle_id: group for group in simulator.state.candidate_groups}
    for obstacle_id, capture_pose in capture_poses.items():
        distances = (
            math.hypot(
                capture_pose.x_cm - candidate.pose.x_cm,
                capture_pose.y_cm - candidate.pose.y_cm,
            )
            for candidate in groups_by_id[obstacle_id].valid_candidates
            if candidate.pose.heading_rad == capture_pose.heading_rad
        )
        assert min(distances) <= config.goal_position_tolerance_cm


@pytest.mark.parametrize("command", ["FW", "BW", "FL", "FR", "BL", "BR"])
def test_demo_command_sequence_contains_required_b1_movement(command):
    assert command in DEMO_COMMAND_SEQUENCE


def test_core_simulator_import_does_not_load_pygame():
    script = "import sys; import algorithm.simulator; assert 'pygame' not in sys.modules"
    subprocess.run([sys.executable, "-c", script], check=True)


def test_pygame_renderer_smoke_with_dummy_video_driver(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    from algorithm.geometry import robot_footprint
    from algorithm.simulator.renderer import (
        PygameRenderer,
        RenderOptions,
        simulator_legend_items,
    )

    simulator, config = build_demo_simulator()
    state_before_render = simulator.state
    footprint_before_render = robot_footprint(simulator.state.robot_pose, config.robot)
    expected_legend_labels = {
        "Nominal candidate",
        "Left fallback",
        "Right fallback",
        "Valid candidate",
        "Invalid candidate",
        "Camera ray clear/blocked",
        "Safety footprint",
        "Planned path",
        "Executed path",
        "Visited target",
        "Target image face",
        "Rear axle / heading",
    }
    assert {item.label for item in simulator_legend_items()} == expected_legend_labels

    renderer = PygameRenderer(config, width_px=960, height_px=700)
    try:
        surface = renderer.initialize()
        options = RenderOptions(show_debug_nodes=True)
        renderer.render(
            simulator.state,
            options,
            debug_nodes=(Pose(80.0, 80.0, 0.0),),
        )
        pygame.display.flip()
        assert surface.get_size() == (960, 700)
        assert simulator.state is state_before_render
        assert robot_footprint(simulator.state.robot_pose, config.robot) == footprint_before_render
    finally:
        renderer.shutdown()
    assert not pygame.get_init()


def test_renderer_dashboard_sections_fit_default_and_compact_windows():
    from algorithm.simulator.renderer import PygameRenderer

    config = build_demo_simulator()[1]
    for width, height in ((1600, 900), (1200, 720), (1920, 1080)):
        renderer = PygameRenderer(config, width_px=width, height_px=height)
        panel = renderer.panel_rect()
        for has_route in (False, True):
            sections = renderer._sidebar_sections(panel, has_route=has_route, has_editor=True)
            ordered = tuple(sections.values())
            assert all(section.rect.left >= panel.left for section in ordered)
            assert all(section.rect.right <= panel.right for section in ordered)
            assert all(section.rect.top >= panel.top for section in ordered)
            assert all(section.rect.bottom <= panel.bottom for section in ordered)
            for previous, current in zip(ordered, ordered[1:]):
                assert previous.rect.bottom <= current.rect.top
            footer_height = 18 if panel.height < 700 else 20
            assert sections["controls"].rect.bottom <= panel.bottom - footer_height


def test_renderer_resize_keeps_square_arena_and_rebuilds_layout(monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    pygame = pytest.importorskip("pygame")
    from algorithm.simulator.renderer import PygameRenderer, RenderOptions

    simulator, config = build_demo_simulator()
    renderer = PygameRenderer(config)
    try:
        renderer.initialize()
        flags = pygame.display.get_surface().get_flags()
        assert flags & pygame.RESIZABLE
        assert not flags & pygame.FULLSCREEN
        assert not flags & getattr(pygame, "NOFRAME", 0)
        for width, height in ((1280, 760), (1920, 1080), (1200, 720)):
            renderer.resize(width, height)
            assert renderer.width_px == width
            assert renderer.height_px == height
            flags = pygame.display.get_surface().get_flags()
            assert flags & pygame.RESIZABLE
            assert not flags & pygame.FULLSCREEN
            assert not flags & getattr(pygame, "NOFRAME", 0)
            assert renderer.viewport.size_px > 0
            assert renderer.viewport.size_px == min(height - 80, width - 480)
            renderer.render(simulator.state, RenderOptions())
        renderer.resize(800, 500)
        assert (renderer.width_px, renderer.height_px) == renderer.MIN_WINDOW_SIZE
        flags = pygame.display.get_surface().get_flags()
        assert flags & pygame.RESIZABLE
        assert not flags & pygame.FULLSCREEN
        assert not flags & getattr(pygame, "NOFRAME", 0)
    finally:
        renderer.shutdown()
