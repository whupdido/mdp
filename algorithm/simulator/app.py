"""Interactive Pygame event loop for headless simulation snapshots."""

from __future__ import annotations

import pygame

from algorithm.config import PlanningConfig

from .headless import HeadlessSimulator, PlaybackState
from .renderer import PygameRenderer, RenderOptions


def run_simulator(simulator: HeadlessSimulator, config: PlanningConfig) -> None:
    renderer = PygameRenderer(config)
    renderer.initialize()
    clock = pygame.time.Clock()
    options = RenderOptions()
    playback_speed = 1.0
    running = True
    try:
        while running:
            elapsed_s = clock.tick(60) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
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
                    elif event.key in (pygame.K_n, pygame.K_RIGHT):
                        simulator.step_primitive()
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
            renderer.render(simulator.state, options, playback_speed=playback_speed)
            pygame.display.flip()
    finally:
        renderer.shutdown()


__all__ = ["run_simulator"]
