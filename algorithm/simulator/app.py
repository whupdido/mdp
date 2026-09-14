"""Interactive Pygame event loop for headless simulation snapshots."""

from __future__ import annotations

import pygame

from algorithm.config import PlanningConfig
from algorithm.models.pose import Pose
from algorithm.models.planning import PlanningResult

from .headless import HeadlessSimulator, PlaybackState
from .renderer import PygameRenderer, RenderOptions


def run_simulator(
    simulator: HeadlessSimulator,
    config: PlanningConfig,
    *,
    debug_nodes: tuple[Pose, ...] = (),
    show_debug_nodes: bool = False,
    planning_result: PlanningResult | None = None,
) -> None:
    renderer = PygameRenderer(config)
    renderer.initialize()
    clock = pygame.time.Clock()
    options = RenderOptions(show_debug_nodes=show_debug_nodes)
    playback_speed = 1.0
    running = True
    try:
        while running:
            elapsed_s = clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.VIDEORESIZE or event.type in {
                    getattr(pygame, "WINDOWSIZECHANGED", -1),
                    getattr(pygame, "WINDOWRESIZED", -1),
                }:
                    width = getattr(event, "w", 0) or pygame.display.get_window_size()[0]
                    height = getattr(event, "h", 0) or pygame.display.get_window_size()[1]
                    renderer.resize(width, height)
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif event.key == pygame.K_SPACE:
                        if simulator.state.playback_state is PlaybackState.PLAYING:
                            simulator.pause()
                        else:
                            simulator.play()
                    elif event.key == pygame.K_r:
                        simulator.reset()
                    elif event.key == pygame.K_RIGHT or event.key == pygame.K_n:
                        simulator.step_primitive()
                    elif event.key == pygame.K_LEFT:
                        simulator.step_backward()
                    elif event.key in (pygame.K_PLUS, pygame.K_KP_PLUS, pygame.K_EQUALS):
                        playback_speed = min(8.0, playback_speed * 2.0)
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        playback_speed = max(0.25, playback_speed / 2.0)
                    elif event.key == pygame.K_g:
                        options.show_grid_labels = not options.show_grid_labels
                    elif event.key == pygame.K_c:
                        options.show_candidates = not options.show_candidates
                    elif event.key == pygame.K_l:
                        options.show_camera_rays = not options.show_camera_rays
                    elif event.key == pygame.K_f:
                        options.show_footprint = not options.show_footprint
                    elif event.key == pygame.K_p:
                        options.show_planned_path = not options.show_planned_path
                    elif event.key == pygame.K_e:
                        options.show_executed_path = not options.show_executed_path
                    elif event.key == pygame.K_d:
                        options.show_debug_nodes = not options.show_debug_nodes
            simulator.advance(elapsed_s * playback_speed)
            renderer.render(
                simulator.state,
                options,
                playback_speed=playback_speed,
                debug_nodes=debug_nodes,
                planning_result=planning_result,
            )
            pygame.display.flip()
    finally:
        renderer.shutdown()


__all__ = ["run_simulator"]
