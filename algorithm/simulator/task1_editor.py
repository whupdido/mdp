"""Optional Pygame frontend for editing and replanning five-target arenas."""

from __future__ import annotations

import math

import pygame

from algorithm.enums import Direction, PlanningStatus
from algorithm.models import GridCell

from .renderer import PygameRenderer, RenderOptions
from .task1_editor_model import EditorState, Task1EditorController


class Task1EditorApp:
    """Thin input/rendering shell around the Pygame-free editor controller."""

    def __init__(self, controller: Task1EditorController) -> None:
        self.controller = controller
        self.renderer = PygameRenderer(
            controller.config,
            width_px=1340,
            height_px=740,
            title="MDP Task 1 Scenario Editor",
        )
        self.options = RenderOptions()
        self.selected_obstacle_id: int | None = None
        self.playback_speed = 1.0
        self.running = True
        self._font: pygame.font.Font | None = None
        self._small_font: pygame.font.Font | None = None

    def run(self) -> None:
        self.renderer.initialize()
        self._font = pygame.font.Font(None, 22)
        self._small_font = pygame.font.Font(None, 17)
        clock = pygame.time.Clock()
        try:
            while self.running:
                elapsed_s = clock.tick(60) / 1000.0
                for event in pygame.event.get():
                    self.handle_event(event)
                self.controller.advance(elapsed_s * self.playback_speed)
                self.render()
                pygame.display.flip()
        finally:
            self.renderer.shutdown()

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            self._handle_mouse(event.button, event.pos)
        elif event.type == pygame.KEYDOWN:
            self._handle_key(event)

    def render(self) -> None:
        simulator = self.controller.preview_simulator()
        self.renderer.render(
            simulator.state,
            self.options,
            playback_speed=self.playback_speed,
        )
        self._draw_selected_cell()
        self._draw_editor_panel()

    def _handle_mouse(self, button: int, position: tuple[int, int]) -> None:
        cell = self._grid_cell_at(position)
        if cell is None:
            return
        obstacle = next((item for item in self.controller.obstacles if item.cell == cell), None)
        try:
            if button == 1:
                if obstacle is not None:
                    self.selected_obstacle_id = obstacle.obstacle_id
                elif self.selected_obstacle_id is not None:
                    self.controller.move_obstacle(self.selected_obstacle_id, cell)
                else:
                    obstacle_id = next(
                        value
                        for value in range(1, 6)
                        if all(item.obstacle_id != value for item in self.controller.obstacles)
                    )
                    self.controller.add_obstacle(obstacle_id, cell, Direction.NORTH)
                    self.selected_obstacle_id = obstacle_id
            elif button == 3 and obstacle is not None:
                self.controller.remove_obstacle(obstacle.obstacle_id)
                if self.selected_obstacle_id == obstacle.obstacle_id:
                    self.selected_obstacle_id = None
        except (KeyError, RuntimeError, ValueError, StopIteration) as exc:
            self.controller.status_message = str(exc)

    def _handle_key(self, event: pygame.event.Event) -> None:
        if event.key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif event.key == pygame.K_RETURN:
            try:
                self.controller.announce_planning()
                self.render()
                pygame.display.flip()
                self.controller.plan()
            except RuntimeError as exc:
                self.controller.status_message = str(exc)
        elif event.key == pygame.K_SPACE:
            self.controller.play_pause()
        elif event.key == pygame.K_RIGHT:
            self.controller.step_primitive()
        elif event.key == pygame.K_r:
            self.controller.reset_playback()
        elif event.key == pygame.K_n:
            self.options.show_candidates = not self.options.show_candidates
        elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
            if self.selected_obstacle_id is not None:
                try:
                    self.controller.remove_obstacle(self.selected_obstacle_id)
                    self.selected_obstacle_id = None
                except (KeyError, RuntimeError, ValueError) as exc:
                    self.controller.status_message = str(exc)
        elif event.key in (pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d):
            direction = {
                pygame.K_w: Direction.NORTH,
                pygame.K_a: Direction.WEST,
                pygame.K_s: Direction.SOUTH,
                pygame.K_d: Direction.EAST,
            }[event.key]
            if self.selected_obstacle_id is not None:
                try:
                    self.controller.change_face(self.selected_obstacle_id, direction)
                except (KeyError, RuntimeError, ValueError) as exc:
                    self.controller.status_message = str(exc)
        elif event.key == pygame.K_F5:
            solvable = bool(event.mod & pygame.KMOD_SHIFT)
            try:
                self.render()
                pygame.display.flip()
                self.controller.randomize(require_solvable=solvable)
                self.selected_obstacle_id = None
            except RuntimeError as exc:
                self.controller.status_message = str(exc)
        elif event.key in (pygame.K_PLUS, pygame.K_KP_PLUS, pygame.K_EQUALS):
            self.playback_speed = min(8.0, self.playback_speed * 2.0)
        elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.playback_speed = max(0.25, self.playback_speed / 2.0)

    def _grid_cell_at(self, position: tuple[int, int]) -> GridCell | None:
        x_cm, y_cm = self.renderer.viewport.screen_to_world(*position)
        arena_size = self.controller.config.arena_size_cm
        if not (0.0 <= x_cm < arena_size and 0.0 <= y_cm < arena_size):
            return None
        cell_size = self.controller.config.cell_size_cm
        return GridCell(math.floor(x_cm / cell_size), math.floor(y_cm / cell_size))

    def _draw_selected_cell(self) -> None:
        if self.selected_obstacle_id is None or self.renderer.screen is None:
            return
        obstacle = next(
            (
                item
                for item in self.controller.obstacles
                if item.obstacle_id == self.selected_obstacle_id
            ),
            None,
        )
        if obstacle is None:
            return
        cell_size = self.controller.config.cell_size_cm
        left, top = self.renderer.viewport.world_to_screen(
            obstacle.cell.x * cell_size,
            (obstacle.cell.y + 1) * cell_size,
        )
        right, bottom = self.renderer.viewport.world_to_screen(
            (obstacle.cell.x + 1) * cell_size,
            obstacle.cell.y * cell_size,
        )
        pygame.draw.rect(
            self.renderer.screen,
            (255, 204, 79),
            pygame.Rect(round(left), round(top), round(right - left), round(bottom - top)),
            3,
        )

    def _draw_editor_panel(self) -> None:
        screen = self.renderer.screen
        if screen is None or self._font is None or self._small_font is None:
            return
        left = round(self.renderer.viewport.left_px + self.renderer.viewport.size_px + 25)
        panel = pygame.Rect(left + 10, 345, self.renderer.width_px - left - 45, 345)
        pygame.draw.rect(screen, (22, 28, 38), panel, border_radius=8)
        pygame.draw.rect(screen, (75, 88, 108), panel, 1, border_radius=8)
        x = panel.left + 15
        y = panel.top + 12
        self._text(f"EDITOR STATE: {self.controller.state.value.replace('_', ' ').upper()}", x, y)
        y += 25
        self._small(self.controller.status_message, x, y, (236, 190, 82))
        y += 25

        selected = next(
            (
                item
                for item in self.controller.obstacles
                if item.obstacle_id == self.selected_obstacle_id
            ),
            None,
        )
        if selected is None:
            self._small("Selected obstacle: none", x, y)
        else:
            self._small(
                f"Obstacle {selected.obstacle_id}  Grid: ({selected.cell.x}, {selected.cell.y})  "
                f"Image face: {selected.face.name.title()}",
                x,
                y,
            )
        y += 23

        result = self.controller.planning_result
        if result is not None:
            self._small(f"Planning status: {result.status.value.upper()}", x, y)
            y += 20
            reachability = result.metrics.target_reachability
            if reachability:
                self._small(
                    "Target reachability: "
                    + "  ".join(
                        f"{item.target_id}:{item.reachable_candidates}/{item.geometric_candidates}"
                        f" (active {item.activated_candidates})"
                        for item in reachability
                    ),
                    x,
                    y,
                )
                y += 19
            if result.status is PlanningStatus.SUCCESS and result.route is not None:
                route = result.route
                selected_text = ", ".join(
                    f"{target}: {_candidate_description(kind)}"
                    for target, kind in zip(route.target_order, route.selected_candidate_kinds)
                )
                self._small(f"Order: {' > '.join(map(str, route.target_order))}", x, y)
                y += 19
                self._small(f"Selected: {selected_text}", x, y)
                y += 19
                self._small(
                    f"Distance: {route.metrics.geometric_distance_cm:.1f} cm   "
                    f"Provisional time: {route.metrics.estimated_time_s:.2f} s",
                    x,
                    y,
                )
                y += 19
                metrics = result.metrics
                self._small(
                    f"Timing candidate/pairwise/global/total: "
                    f"{metrics.candidate_generation_time_s:.3f} / "
                    f"{metrics.pairwise_planning_time_s:.3f} / "
                    f"{metrics.global_routing_time_s:.3f} / "
                    f"{metrics.total_planning_time_s:.3f} s",
                    x,
                    y,
                )
                y += 19
                self._small(
                    f"Tiers: {metrics.candidate_tiers_activated}  considered: "
                    f"{metrics.candidate_count_considered}  cache H/M: "
                    f"{metrics.cache_hits}/{metrics.pairwise_cache_misses}",
                    x,
                    y,
                )
                y += 19
                self._small(
                    f"Hybrid retries/recoveries: {metrics.hybrid_astar_retries}/"
                    f"{metrics.hybrid_astar_retry_recoveries}  expanded: "
                    f"{metrics.total_nodes_expanded}",
                    x,
                    y,
                )
                y += 19
            else:
                for issue in result.issues[:3]:
                    self._small(issue.message, x, y, (238, 126, 126))
                    y += 18

        y = max(y + 8, panel.top + 185)
        controls = (
            "Left click: select/add; selected + empty cell: move (grid snapped)",
            "Right click or Delete: remove selected obstacle",
            "W / A / S / D: image face North / West / South / East",
            "N: show/hide observation candidates (10/20/30 C/L/R)",
            "Enter: PLAN TASK 1   Space: play/pause   Right Arrow: step",
            "R: reset playback only   F5: raw random   Shift+F5: random solvable",
            "Candidate marker center = rear-axle reference pose",
        )
        for line in controls:
            self._small(line, x, y, (180, 190, 207))
            y += 19

    def _text(self, value: str, x: int, y: int, color=(239, 242, 247)) -> None:
        assert self.renderer.screen is not None and self._font is not None
        self.renderer.screen.blit(self._font.render(value, True, color), (x, y))

    def _small(self, value: str, x: int, y: int, color=(224, 229, 237)) -> None:
        assert self.renderer.screen is not None and self._small_font is not None
        self.renderer.screen.blit(self._small_font.render(value, True, color), (x, y))


def run_task1_editor(controller: Task1EditorController) -> None:
    Task1EditorApp(controller).run()


def _candidate_description(label: str) -> str:
    lateral = {"C": "CENTER", "L": "LEFT", "R": "RIGHT", "O": "OFFSET"}
    if len(label) >= 2 and label[-1] in lateral:
        return f"{label[:-1]}cm {lateral[label[-1]]}"
    return label.upper()


__all__ = ["Task1EditorApp", "run_task1_editor"]
