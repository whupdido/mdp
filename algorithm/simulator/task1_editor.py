"""Optional Pygame frontend for editing and replanning five-target arenas."""

from __future__ import annotations

import math
import queue
import threading
from datetime import datetime, timezone
from pathlib import Path

import pygame

from algorithm.enums import Direction, PlanningStatus
from algorithm.models import ArenaInput, GridCell, PlanningResult

from .renderer import EditorPanelData, PygameRenderer, RenderOptions
from .task1_editor_model import EditorState, Task1EditorController


class Task1EditorApp:
    """Thin input/rendering shell around the Pygame-free editor controller."""

    def __init__(self, controller: Task1EditorController) -> None:
        self.controller = controller
        self.renderer = PygameRenderer(
            controller.config,
            width_px=PygameRenderer.DEFAULT_WINDOW_SIZE[0],
            height_px=PygameRenderer.DEFAULT_WINDOW_SIZE[1],
            title=("MDP Task 1 Production Simulator" if getattr(controller, "production_mode", False)
                   else "MDP Task 1 Scenario Editor"),
        )
        self.options = RenderOptions()
        self.selected_obstacle_id: int | None = None
        self.playback_speed = 1.0
        self.running = True
        self._planning_thread: threading.Thread | None = None
        self._planning_results: queue.Queue[
            tuple[ArenaInput, PlanningResult | None, BaseException | None]
        ] = queue.Queue()
        self._font: pygame.font.Font | None = None
        self._small_font: pygame.font.Font | None = None

    @property
    def planning_in_progress(self) -> bool:
        return self._planning_thread is not None and self._planning_thread.is_alive()

    def run(self) -> None:
        self.renderer.initialize()
        self._font = pygame.font.Font(None, 22)
        self._font.set_bold(True)
        self._small_font = pygame.font.Font(None, 17)
        clock = pygame.time.Clock()
        try:
            while self.running:
                elapsed_s = clock.tick(60) / 1000.0
                for event in pygame.event.get():
                    self.handle_event(event)
                self._poll_planning_result()
                self.controller.advance(elapsed_s * self.playback_speed)
                self.render()
                pygame.display.flip()
        finally:
            self.renderer.shutdown()

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.VIDEORESIZE or event.type in {
            getattr(pygame, "WINDOWSIZECHANGED", -1),
            getattr(pygame, "WINDOWRESIZED", -1),
        }:
            width = getattr(event, "w", 0) or pygame.display.get_window_size()[0]
            height = getattr(event, "h", 0) or pygame.display.get_window_size()[1]
            self.renderer.resize(width, height)
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
            planning_result=self.controller.planning_result,
            editor_data=self._editor_panel_data(),
        )
        self._draw_selected_cell()

    def _editor_panel_data(self) -> EditorPanelData:
        selected = next(
            (item for item in self.controller.obstacles if item.obstacle_id == self.selected_obstacle_id),
            None,
        )
        selected_text = "none"
        if selected is not None:
            selected_text = (
                f"Obstacle {selected.obstacle_id} | Grid ({selected.cell.x}, {selected.cell.y}) | "
                f"Face {selected.face.name.title()}"
            )
        return EditorPanelData(
            state_label=self.controller.state.value.replace("_", " ").upper(),
            status_message=self.controller.status_message,
            selected_obstacle=selected_text,
        )

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
                        for value in range(1, self.controller.target_count_range[1] + 1)
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
            self._start_planning()
            if self.controller.state is EditorState.PLANNING:
                self.render()
                pygame.display.flip()
        elif event.key == pygame.K_s and event.mod & pygame.KMOD_CTRL:
            self._save_physical_run()
        elif event.key == pygame.K_m and event.mod & pygame.KMOD_CTRL:
            self._enter_actual_capture_pose()
        elif event.key == pygame.K_SPACE:
            self.controller.play_pause()
        elif event.key == pygame.K_RIGHT:
            self.controller.step_primitive()
        elif event.key == pygame.K_LEFT:
            self.controller.step_backward()
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

    def _save_physical_run(self) -> None:
        result = self.controller.planning_result
        try:
            from copy import deepcopy
            from .scenarios import build_run_record, build_scenario_record, save_run
            config = self.controller.config
            calibration = getattr(self.controller, "calibration_metadata", None)
            if result is not None and result.route is not None:
                record = build_run_record(self.controller.arena, config, result, calibration=calibration,
                                          actual_capture_poses=getattr(self.controller, "actual_capture_poses", None),
                                          routing_mode=self.controller.routing_mode)
            elif getattr(self.controller, "loaded_scenario_record", None) is not None:
                record = deepcopy(self.controller.loaded_scenario_record)
            else:
                record = build_scenario_record(self.controller.arena, config, calibration=calibration,
                                               routing_mode=self.controller.routing_mode)
            output = Path("task1_runs") / f"task1_run_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}.json"
            save_run(output, record)
            self.controller.status_message = f"Saved scenario/run to {output}"
            print(f"Saved scenario/run: {output}")
        except (OSError, ValueError, KeyError) as exc:
            self.controller.status_message = f"Save failed: {exc}"

    def _enter_actual_capture_pose(self) -> None:
        result = self.controller.planning_result
        loaded_record = getattr(self.controller, "loaded_scenario_record", None)
        if (result is None or result.route is None) and not loaded_record:
            self.controller.status_message = "Load or plan a route before entering measured capture poses"
            return
        try:
            target = int(input("Capture target ID: "))
            x_cm = float(input("Actual rear-axle x (cm): "))
            y_cm = float(input("Actual rear-axle y (cm): "))
            heading_deg = float(input("Actual heading (degrees): "))
            from algorithm.models import Pose
            from .scenarios import attach_measured_capture_pose, compare_capture_poses, pose_from_record
            if result is not None and result.route is not None:
                planned = next(item.pose for item in result.route.observation_poses if item.obstacle_id == target)
            else:
                planned = pose_from_record(loaded_record["planned_capture_poses"][str(target)])
            measured = Pose(x_cm, y_cm, math.radians(heading_deg))
            measured_poses = getattr(self.controller, "actual_capture_poses", {})
            measured_poses[target] = measured
            self.controller.actual_capture_poses = measured_poses
            if loaded_record:
                attach_measured_capture_pose(loaded_record, target, measured)
            error = compare_capture_poses(planned, measured)
            self.controller.status_message = f"Target {target}: physical position error {error['position_error_cm']:.1f} cm"
            print(f"Capture {target} physical-vs-planned: {error}")
        except (ValueError, StopIteration, EOFError) as exc:
            self.controller.status_message = f"Measured pose not recorded: {exc}"

    def _start_planning(self) -> None:
        """Start exactly one background planner for an immutable arena copy."""
        if self.planning_in_progress or self.controller.state is EditorState.PLANNING:
            self.controller.status_message = "Planning already in progress"
            return
        try:
            if not self.controller.announce_planning():
                return
        except RuntimeError as exc:
            self.controller.status_message = str(exc)
            return
        arena = self.controller.arena
        self._planning_results = queue.Queue()
        self._planning_thread = threading.Thread(
            target=self._planning_worker,
            args=(arena,),
            name="task1-planner",
            daemon=True,
        )
        self._planning_thread.start()

    def _planning_worker(self, arena: ArenaInput) -> None:
        try:
            result = self.controller.plan_snapshot(arena)
        except Exception as exc:  # worker failures must return to the UI
            self._planning_results.put((arena, None, exc))
        else:
            self._planning_results.put((arena, result, None))

    def _poll_planning_result(self) -> None:
        """Apply one completed worker result on the Pygame thread."""
        try:
            arena, result, error = self._planning_results.get_nowait()
        except queue.Empty:
            return
        self._planning_thread = None
        if error is not None:
            self.controller.apply_planning_exception(arena, error)
        else:
            assert result is not None
            self.controller.apply_planning_result(arena, result)
            if getattr(self.controller, "production_mode", False):
                _print_production_route_diagnostics(self.controller, result)

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
        panel = pygame.Rect(left + 10, 330, self.renderer.width_px - left - 45, 370)
        pygame.draw.rect(screen, (22, 28, 38), panel, border_radius=8)
        pygame.draw.rect(screen, (75, 88, 108), panel, 1, border_radius=8)
        x = panel.left + 15
        y = panel.top + 12
        state_label = self.controller.state.value.replace("_", " ").upper()
        self._text("PRODUCTION SIMULATOR" if getattr(self.controller, "production_mode", False)
                   else "TASK 1 EDITOR", x, y)
        self._small(f"STATE  {state_label}", x + 185, y + 4, (121, 202, 239))
        y += 27
        status = self.controller.status_message
        if len(status) > 82:
            status = status[:79] + "..."
        self._small(status, x, y, (236, 190, 82))
        y += 22

        selected = next(
            (
                item
                for item in self.controller.obstacles
                if item.obstacle_id == self.selected_obstacle_id
            ),
            None,
        )
        if selected is None:
            self._small("Selected obstacle: none", x, y, (180, 190, 207))
        else:
            self._small(
                f"Obstacle {selected.obstacle_id}  Grid: ({selected.cell.x}, {selected.cell.y})  "
                f"Image face: {selected.face.name.title()}",
                x,
                y,
                (180, 190, 207),
            )
        y += 25

        result = self.controller.planning_result
        content_top = y
        column_gap = 18
        column_width = (panel.width - 30 - column_gap) // 2
        right_x = x + column_width + column_gap
        self._section("ROUTE SUMMARY", x, content_top, column_width)
        self._section("PLANNER DIAGNOSTICS", right_x, content_top, column_width)
        route_y = content_top + 24
        diag_y = content_top + 24
        if result is None:
            self._row("Status", "READY TO PLAN", x, route_y, column_width)
            route_y += 18
            self._row("Planning time", "-", x, route_y, column_width)
            route_y += 18
            self._row("Order", "-", x, route_y, column_width)
            route_y += 18
            self._small("Press Enter to plan the current arena", x, route_y, (180, 190, 207))
        else:
            self._row("Status", result.status.value.upper(), x, route_y, column_width)
            route_y += 18
            if result.status is PlanningStatus.SUCCESS and result.route is not None:
                route = result.route
                self._row(
                    "Planning time",
                    f"{result.metrics.total_planning_time_s:.3f} s",
                    x,
                    route_y,
                    column_width,
                    emphasize=True,
                )
                route_y += 18
                self._row("Route time", f"{route.metrics.estimated_time_s:.2f} s", x, route_y, column_width)
                route_y += 18
                self._row("Distance", f"{route.metrics.geometric_distance_cm:.1f} cm", x, route_y, column_width)
                route_y += 18
                self._row("Order", ">".join(map(str, route.target_order)), x, route_y, column_width)
                route_y += 18
                selected_text = ", ".join(
                    f"{target}:{_candidate_description(kind)}"
                    for target, kind in zip(route.target_order, route.selected_candidate_kinds)
                ) or "-"
                self._row("Candidates", selected_text, x, route_y, column_width)
                route_y += 18
                self._row("Primitives", _primitive_summary(route), x, route_y, column_width)
            else:
                for issue in result.issues[:3]:
                    self._small(issue.message[:62], x, route_y, (238, 126, 126))
                    route_y += 17

        if result is not None:
            metrics = result.metrics
            self._row(
                "Timing",
                f"C {metrics.candidate_generation_time_s:.2f}  P {metrics.pairwise_planning_time_s:.2f}",
                right_x, diag_y, column_width,
            )
            diag_y += 18
            self._row(
                "Global/total",
                f"{metrics.global_routing_time_s:.2f} / {metrics.total_planning_time_s:.2f} s",
                right_x, diag_y, column_width,
            )
            diag_y += 18
            self._row(
                "Cache H/M",
                f"{metrics.cache_hits}/{metrics.pairwise_cache_misses}  tiers {metrics.candidate_tiers_activated}",
                right_x, diag_y, column_width,
            )
            diag_y += 18
            self._row(
                "Retries",
                f"{metrics.hybrid_astar_retries}/{metrics.hybrid_astar_retry_recoveries} recoveries",
                right_x, diag_y, column_width,
            )
            diag_y += 18
            self._row("Expanded", str(metrics.total_nodes_expanded), right_x, diag_y, column_width)
            diag_y += 18
            reachability = "  ".join(
                f"{item.target_id}:{item.reachable_candidates}/{item.geometric_candidates}"
                for item in metrics.target_reachability
            ) or "-"
            self._row("Reachable", reachability, right_x, diag_y, column_width)
        else:
            self._small("Planning metrics appear after Enter", right_x, diag_y, (180, 190, 207))

        controls_y = panel.bottom - 112
        self._section("CONTROLS", x, controls_y, panel.width - 30)
        controls = (
            "Left click select/add/move   Right click/Delete remove",
            "W/A/S/D image face North/West/South/East   N candidates",
            "Enter plan   Space play/pause   Left/Right navigate   R reset",
            ("Ctrl+S save run   Ctrl+M measured capture pose" if getattr(self.controller, "production_mode", False)
             else "F5 random   Shift+F5 verified solvable random"),
            ("rear axle marker = planned robot reference" if getattr(self.controller, "production_mode", False)
             else "marker = rear axle; arrows never send STM commands"),
        )
        for index, line in enumerate(controls):
            self._small(line, x, controls_y + 23 + index * 17, (180, 190, 207))

    def _section(self, title: str, x: int, y: int, width: int) -> None:
        assert self.renderer.screen is not None
        self._text(title, x, y, (239, 242, 247))
        pygame.draw.line(self.renderer.screen, (75, 88, 108), (x + min(205, width - 10), y + 10), (x + width, y + 10), 1)

    def _row(self, label: str, value: str, x: int, y: int, width: int, *, emphasize: bool = False) -> None:
        self._small(label.upper(), x, y, (164, 174, 190))
        max_chars = max(14, (width - 84) // 7)
        if len(value) > max_chars:
            value = value[: max_chars - 1] + "..."
        self._small(value, x + 82, y, (255, 237, 151) if emphasize else (224, 229, 237))

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


def _primitive_summary(route) -> str:
    counts = {command: 0 for command in ("FW", "BW", "FL", "FR", "BL", "BR")}
    for primitive in route.primitives:
        if primitive.command in counts:
            counts[primitive.command] += 1
    return " ".join(f"{command}={counts[command]}" for command in counts)


def _print_production_route_diagnostics(controller, result) -> None:
    from .scenarios import build_run_record
    if result.route is None:
        print(f"Production plan: {result.status.value}: " + "; ".join(issue.message for issue in result.issues))
        return
    record = build_run_record(controller.arena, controller.config, result,
                              calibration=getattr(controller, "calibration_metadata", None),
                              routing_mode=controller.routing_mode)
    route = record["planned_route"]
    movements = route["movements"]
    obstacles = [item["minimum_obstacle_clearance_cm"] for item in movements]
    boundaries = [item["minimum_arena_clearance_cm"] for item in movements]
    print("PRODUCTION ROUTE DIAGNOSTICS")
    print(f"  target order={result.route.target_order} candidates={result.route.selected_candidate_kinds}")
    print(f"  movement commands={len(movements)} distance={route['geometric_distance_cm']:.1f} cm "
          f"estimated={route['estimated_execution_time_s']:.2f}s planning={result.metrics.total_planning_time_s:.3f}s")
    primitives = result.route.primitives
    print(f"  straight={sum(p.steering.value == 'straight' for p in primitives)} "
          f"turns={sum(p.steering.value != 'straight' for p in primitives)} "
          f"reverse={sum(p.gear.value == 'reverse' for p in primitives)} "
          f"reverse_turns={sum(p.gear.value == 'reverse' and p.steering.value != 'straight' for p in primitives)} "
          f"steering_changes={result.route.metrics.steering_changes} "
          f"direction_changes={result.route.metrics.direction_changes}")
    print(f"  minimum obstacle clearance={min(obstacles, default=float('inf')):.2f} cm "
          f"minimum arena clearance={min(boundaries, default=float('inf')):.2f} cm "
          f"expanded={result.metrics.total_nodes_expanded}")
    for move in movements:
        clearance = move["minimum_obstacle_clearance_cm"]
        boundary = move["minimum_arena_clearance_cm"]
        warning_tags = [f"OBSTACLE<{threshold}cm" for threshold in (8, 5, 3) if clearance < threshold]
        if boundary < 5.0:
            warning_tags.append("ARENA<5cm")
        warning = " WARNING " + ",".join(warning_tags) if warning_tags else ""
        print(f"  {move['sequence']:03d} {move['command']} {move['start_pose']} -> {move['end_pose']} "
              f"target={move['target_obstacle']} candidate={move['candidate']} gear={move['gear']} "
              f"obstacle_clear={clearance:.2f}cm arena_clear={boundary:.2f}cm "
              f"closest={move['closest_obstacle_id']} cumulative={move['cumulative_distance_cm']:.1f}cm{warning}")
    for capture in route["captures"]:
        print(f"  CAPTURE target={capture['target_obstacle']} face={capture['image_face']} "
              f"rear_axle={capture['rear_axle_pose']} camera={capture['planned_camera_pose']} "
              f"face_gap={capture['camera_to_image_face_distance_cm']:.2f}cm "
              f"lateral_error={capture['lateral_alignment_error_cm']:.2f}cm "
              f"candidate={capture['selected_candidate']} standoff={capture['standoff_cm']} "
              f"motion_before={capture['motion_before_capture']}")


__all__ = ["Task1EditorApp", "run_task1_editor"]
