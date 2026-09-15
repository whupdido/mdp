from __future__ import annotations

import pygame

from .renderer import PygameRenderer, RenderOptions
from .task2_editor_model import Task2EditorController


class Task2EditorApp:
    """Pygame frontend for editing, planning, and playing Fastest Car."""

    def __init__(self, controller: Task2EditorController) -> None:
        self.controller = controller

        self.renderer = PygameRenderer(
            controller.config,
            width_px=1340,
            height_px=740,
            title="MDP Fastest Car Scenario Editor",
            fastest_car_mode=True,
        )

        self.options = RenderOptions(
            show_candidates=False,
            show_camera_rays=False,
            show_planned_path=True,
            show_executed_path=True,
        )

        self.running = True
        self.selected_field = 0

        self._font: pygame.font.Font | None = None
        self._small_font: pygame.font.Font | None = None

    def run(self) -> None:
        self.renderer.initialize()

        self._font = pygame.font.Font(None, 24)
        self._small_font = pygame.font.Font(None, 19)

        clock = pygame.time.Clock()

        try:
            while self.running:
                delta_s = clock.tick(60) / 1000.0

                for event in pygame.event.get():
                    self.handle_event(event)

                self.update(delta_s)
                self.render()

                pygame.display.flip()

        finally:
            self.renderer.shutdown()

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False

        elif event.type == pygame.KEYDOWN:
            self._handle_key(event)

    def _handle_key(self, event: pygame.event.Event) -> None:
        if event.key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False

        # ------------------------------------------------------------
        # Planning
        # ------------------------------------------------------------

        elif event.key == pygame.K_RETURN:
            self.controller.rerun()

        # ------------------------------------------------------------
        # Playback
        # ------------------------------------------------------------

        elif event.key == pygame.K_SPACE:
            simulator = self.controller.preview_simulator()

            if simulator.state.playback_state.name == "PLAYING":
                simulator.pause()
            else:
                simulator.play()

        elif event.key in (pygame.K_RIGHT, pygame.K_n):
            simulator = self.controller.preview_simulator()
            simulator.step_primitive()

        elif event.key == pygame.K_BACKSPACE:
            simulator = self.controller.preview_simulator()
            simulator.reset()

        # ------------------------------------------------------------
        # Geometry editing
        # ------------------------------------------------------------

        elif event.key == pygame.K_r:
            self.controller.randomize()

        elif event.key == pygame.K_TAB:
            self.selected_field = (self.selected_field + 1) % 4

        elif event.key in (pygame.K_UP, pygame.K_w):
            self._change_selected_field(+5)

        elif event.key in (pygame.K_DOWN, pygame.K_s):
            self._change_selected_field(-5)

        # ------------------------------------------------------------
        # Obstacle arrows
        # ------------------------------------------------------------

        elif event.key == pygame.K_1:
            self.controller.toggle_obstacle_1_direction()

        elif event.key == pygame.K_2:
            self.controller.toggle_obstacle_2_direction()

    def update(self, delta_s: float) -> None:
        """Advance robot playback."""

        simulator = self.controller.preview_simulator()

        simulator.advance(delta_s)

    def _change_selected_field(self, amount: int) -> None:
        setup = self.controller.setup

        if self.selected_field == 0:
            self.controller.set_top_clearance(
                max(
                    50,
                    min(
                        120,
                        setup.top_clearance_cm + amount,
                    ),
                )
            )

        elif self.selected_field == 1:
            self.controller.set_bottom_clearance(
                max(
                    50,
                    min(
                        120,
                        setup.bottom_clearance_cm + amount,
                    ),
                )
            )

        elif self.selected_field == 2:
            self.controller.set_carpark_to_obstacle_1(
                max(
                    50,
                    min(
                        130,
                        setup.carpark_to_obstacle_1_cm + amount,
                    ),
                )
            )

        elif self.selected_field == 3:
            self.controller.set_obstacle_1_to_obstacle_2(
                max(
                    50,
                    min(
                        130,
                        setup.obstacle_1_to_obstacle_2_cm + amount,
                    ),
                )
            )

    def render(self) -> None:
        simulator = self.controller.preview_simulator()

        self.renderer.render(
            simulator.state,
            self.options,
        )

        self._draw_editor_panel()

    def _draw_editor_panel(self) -> None:
        screen = self.renderer.screen

        if (
            screen is None
            or self._font is None
            or self._small_font is None
        ):
            return

        panel_left = round(
            self.renderer.viewport.left_px
            + self.renderer.viewport.size_px
            + 25
        )

        panel = pygame.Rect(
            panel_left,
            round(self.renderer.viewport.top_px),
            self.renderer.width_px - panel_left - 25,
            round(self.renderer.viewport.size_px),
        )

        pygame.draw.rect(
            screen,
            (27, 33, 44),
            panel,
            border_radius=10,
        )

        pygame.draw.rect(
            screen,
            (52, 62, 78),
            panel,
            1,
            border_radius=10,
        )

        x = panel.left + 20
        y = panel.top + 20

        self._text(
            "FASTEST CAR",
            x,
            y,
        )

        y += 35

        self._small(
            "EDIT + PLANNING + PLAYBACK",
            x,
            y,
        )

        y += 45

        setup = self.controller.setup

        fields = (
            (
                "Top clearance",
                setup.top_clearance_cm,
                "50–120 cm",
            ),
            (
                "Bottom clearance",
                setup.bottom_clearance_cm,
                "50–120 cm",
            ),
            (
                "Carpark → O1",
                setup.carpark_to_obstacle_1_cm,
                "50–130 cm",
            ),
            (
                "O1 → O2",
                setup.obstacle_1_to_obstacle_2_cm,
                "50–130 cm",
            ),
        )

        for index, (label, value, limits) in enumerate(fields):
            selected = index == self.selected_field

            prefix = "> " if selected else "  "

            self._text(
                f"{prefix}{label}",
                x,
                y,
            )

            y += 25

            self._small(
                f"{value} cm    ({limits})",
                x + 20,
                y,
            )

            y += 45

        self._text(
            "ARROWS",
            x,
            y,
        )

        y += 27

        self._small(
            f"1         O1: {setup.obstacle_1_direction.name}",
            x,
            y,
        )

        y += 23

        self._small(
            f"2         O2: {setup.obstacle_2_direction.name}",
            x,
            y,
        )

        y += 35

        self._text(
            "PLANNING",
            x,
            y,
        )

        y += 27

        self._small(
            "ENTER     Run planner",
            x,
            y,
        )

        y += 23

        self._small(
            "R         Randomize setup",
            x,
            y,
        )

        y += 35

        self._text(
            "ROBOT",
            x,
            y,
        )

        y += 27

        self._small(
            "SPACE     Play / pause",
            x,
            y,
        )

        y += 23

        self._small(
            "RIGHT     Step one primitive",
            x,
            y,
        )

        y += 23

        self._small(
            "BACKSPACE Reset robot",
            x,
            y,
        )

        y += 35

        self._text(
            "EDIT",
            x,
            y,
        )

        y += 27

        self._small(
            "TAB       Select parameter",
            x,
            y,
        )

        y += 23

        self._small(
            "UP / DOWN Change by 5 cm",
            x,
            y,
        )

        y += 23

        self._small(
            "1 / 2     Toggle obstacle arrow",
            x,
            y,
        )

        y += 23

        self._small(
            "Q / ESC   Exit",
            x,
            y,
        )

        y += 35

        simulator = self.controller.preview_simulator()

        state = simulator.state

        self._small(
            f"Playback: {state.playback_state.name}",
            x,
            y,
        )

        y += 23

        self._small(
            f"Step: {state.current_step_index}/{state.total_steps}",
            x,
            y,
        )

        y += 23

        self._small(
            f"Time: {state.simulation_time_s:.1f}s",
            x,
            y,
        )

        y += 35

        self._small(
            self.controller.status_message,
            x,
            y,
        )

    def _text(self, value: str, x: int, y: int) -> None:
        assert self.renderer.screen is not None
        assert self._font is not None

        self.renderer.screen.blit(
            self._font.render(
                value,
                True,
                (239, 242, 247),
            ),
            (x, y),
        )

    def _small(self, value: str, x: int, y: int) -> None:
        assert self.renderer.screen is not None
        assert self._small_font is not None

        self.renderer.screen.blit(
            self._small_font.render(
                value,
                True,
                (164, 174, 190),
            ),
            (x, y),
        )


def run_task2_editor(controller: Task2EditorController) -> None:
    Task2EditorApp(controller).run()


__all__ = [
    "Task2EditorApp",
    "run_task2_editor",
]