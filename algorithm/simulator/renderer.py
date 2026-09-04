"""Optional Pygame rendering for the independent headless simulator."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import pygame

from algorithm.config import PlanningConfig
from algorithm.constants import START_ZONE_SIZE_CM
from algorithm.enums import Direction
from algorithm.geometry import obstacle_bounds, robot_footprint
from algorithm.models.pose import Pose
from algorithm.targets.geometry import camera_world_position
from algorithm.targets.models import ObservationCandidateKind

from .headless import SimulationState
from .viewport import WorldViewport


Color = tuple[int, int, int]


@dataclass(slots=True)
class RenderOptions:
    show_grid_labels: bool = True
    show_candidates: bool = True
    show_camera_rays: bool = True
    show_footprint: bool = True
    show_planned_path: bool = True
    show_executed_path: bool = True
    show_debug_nodes: bool = False


@dataclass(frozen=True, slots=True)
class LegendItem:
    """One renderer-owned legend entry with no planning significance."""

    label: str
    color: Color
    style: str


NOMINAL_COLOR: Color = (142, 103, 219)
LEFT_COLOR: Color = (49, 158, 211)
RIGHT_COLOR: Color = (232, 147, 64)
INVALID_COLOR: Color = (220, 75, 75)
CLEAR_RAY_COLOR: Color = (67, 168, 118)
BLOCKED_RAY_COLOR: Color = (220, 75, 75)
FOOTPRINT_COLOR: Color = (68, 124, 218)
PLANNED_PATH_COLOR: Color = (65, 139, 225)
EXECUTED_PATH_COLOR: Color = (246, 177, 66)
VISITED_COLOR: Color = (48, 166, 105)
IMAGE_FACE_COLOR: Color = (238, 74, 79)
AXLE_COLOR: Color = (239, 242, 246)


def simulator_legend_items() -> tuple[LegendItem, ...]:
    """Return the stable visual vocabulary used by the Phase 4 renderer."""
    return (
        LegendItem("Nominal candidate", NOMINAL_COLOR, "diamond"),
        LegendItem("Left fallback", LEFT_COLOR, "triangle"),
        LegendItem("Right fallback", RIGHT_COLOR, "square"),
        LegendItem("Valid candidate", (234, 238, 244), "filled"),
        LegendItem("Invalid candidate", INVALID_COLOR, "invalid"),
        LegendItem("Camera ray clear/blocked", CLEAR_RAY_COLOR, "ray"),
        LegendItem("Safety footprint", FOOTPRINT_COLOR, "box"),
        LegendItem("Planned path", PLANNED_PATH_COLOR, "dashed"),
        LegendItem("Executed path", EXECUTED_PATH_COLOR, "line"),
        LegendItem("Visited target", VISITED_COLOR, "visited"),
        LegendItem("Target image face", IMAGE_FACE_COLOR, "face"),
        LegendItem("Rear axle / heading", AXLE_COLOR, "axle"),
    )


class PygameRenderer:
    """Draw immutable simulator snapshots; it never advances playback state."""

    BACKGROUND: Color = (17, 21, 29)
    PANEL: Color = (27, 33, 44)
    PANEL_BORDER: Color = (52, 62, 78)
    ARENA: Color = (244, 245, 240)
    GRID: Color = (207, 212, 207)
    TEXT: Color = (239, 242, 247)
    MUTED_TEXT: Color = (164, 174, 190)
    DARK_TEXT: Color = (38, 43, 48)

    def __init__(
        self,
        config: PlanningConfig,
        *,
        width_px: int = 1180,
        height_px: int = 740,
        title: str = "MDP Task 1 Simulator",
    ) -> None:
        if width_px < 640 or height_px < 520:
            raise ValueError("renderer window must be at least 640x520 pixels")
        self.config = config
        self.width_px = width_px
        self.height_px = height_px
        self.title = title
        arena_px = float(min(height_px - 80, width_px - 480))
        self.viewport = WorldViewport(config.arena_size_cm, 40.0, 40.0, arena_px)
        # The stylized body uses the same authoritative footprint transform,
        # but removes safety margin for display only. Collision code continues
        # to use ``config.robot`` unchanged.
        self._physical_body_geometry = replace(config.robot, safety_margin_cm=0.0)
        self.screen: pygame.Surface | None = None
        self._title_font: pygame.font.Font | None = None
        self._font: pygame.font.Font | None = None
        self._small_font: pygame.font.Font | None = None
        self._tiny_font: pygame.font.Font | None = None

    def initialize(self) -> pygame.Surface:
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((self.width_px, self.height_px))
        pygame.display.set_caption(self.title)
        self._title_font = pygame.font.Font(None, 31)
        self._font = pygame.font.Font(None, 24)
        self._small_font = pygame.font.Font(None, 19)
        self._tiny_font = pygame.font.Font(None, 16)
        return self.screen

    def shutdown(self) -> None:
        pygame.quit()
        self.screen = None

    def render(
        self,
        state: SimulationState,
        options: RenderOptions,
        *,
        playback_speed: float = 1.0,
        debug_nodes: tuple[Pose, ...] = (),
    ) -> None:
        if any(font is None for font in (self._title_font, self._font, self._small_font, self._tiny_font)):
            raise RuntimeError("initialize() must be called before render()")
        assert self.screen is not None
        self.screen.fill(self.BACKGROUND)
        self._draw_arena(options)
        self._draw_start_zone()
        self._draw_obstacles(state)
        if options.show_camera_rays:
            self._draw_camera_rays(state)
        if options.show_candidates:
            self._draw_candidates(state)
        if options.show_debug_nodes:
            self._draw_debug_nodes(debug_nodes)
        if options.show_planned_path:
            self._draw_dashed_path(state.planned_path, PLANNED_PATH_COLOR, 2)
        if options.show_executed_path:
            self._draw_path(state.executed_path, EXECUTED_PATH_COLOR, 4)
        self._draw_robot(state.robot_pose, options.show_footprint)
        self._draw_sidebar(state, playback_speed)

    def _screen_point(self, x_cm: float, y_cm: float) -> tuple[int, int]:
        x_px, y_px = self.viewport.world_to_screen(x_cm, y_cm)
        return round(x_px), round(y_px)

    def _draw_arena(self, options: RenderOptions) -> None:
        assert self.screen is not None
        rect = pygame.Rect(
            round(self.viewport.left_px),
            round(self.viewport.top_px),
            round(self.viewport.size_px),
            round(self.viewport.size_px),
        )
        shadow = rect.move(5, 6)
        pygame.draw.rect(self.screen, (8, 11, 16), shadow, border_radius=3)
        pygame.draw.rect(self.screen, self.ARENA, rect)
        cell_count = round(self.config.arena_size_cm / self.config.cell_size_cm)
        for index in range(cell_count + 1):
            coordinate = index * self.config.cell_size_cm
            x1, y1 = self._screen_point(coordinate, 0.0)
            x2, y2 = self._screen_point(coordinate, self.config.arena_size_cm)
            pygame.draw.line(self.screen, self.GRID, (x1, y1), (x2, y2), 1)
            x1, y1 = self._screen_point(0.0, coordinate)
            x2, y2 = self._screen_point(self.config.arena_size_cm, coordinate)
            pygame.draw.line(self.screen, self.GRID, (x1, y1), (x2, y2), 1)
            if options.show_grid_labels and index < cell_count:
                x_px, bottom = self._screen_point(coordinate + self.config.cell_size_cm / 2.0, 0.0)
                left, y_px = self._screen_point(0.0, coordinate + self.config.cell_size_cm / 2.0)
                self._blit_text(str(index), (x_px - 4, bottom + 5), self.MUTED_TEXT, tiny=True)
                self._blit_text(str(index), (left - 22, y_px - 6), self.MUTED_TEXT, tiny=True)
        pygame.draw.rect(self.screen, (77, 87, 100), rect, 2)
        north_x = rect.right - 19
        self._blit_text("N", (north_x - 4, rect.top + 8), self.DARK_TEXT, small=True)
        pygame.draw.line(self.screen, self.DARK_TEXT, (north_x, rect.top + 34), (north_x, rect.top + 20), 2)
        pygame.draw.polygon(
            self.screen,
            self.DARK_TEXT,
            ((north_x, rect.top + 17), (north_x - 4, rect.top + 23), (north_x + 4, rect.top + 23)),
        )

    def _draw_start_zone(self) -> None:
        assert self.screen is not None
        top_left = self._screen_point(0.0, START_ZONE_SIZE_CM)
        bottom_right = self._screen_point(START_ZONE_SIZE_CM, 0.0)
        rect = pygame.Rect(top_left, (bottom_right[0] - top_left[0], bottom_right[1] - top_left[1]))
        overlay = pygame.Surface(rect.size, pygame.SRCALPHA)
        overlay.fill((54, 177, 112, 52))
        self.screen.blit(overlay, rect.topleft)
        pygame.draw.rect(self.screen, (45, 143, 91), rect, 2)
        self._blit_text("START", (rect.left + 7, rect.bottom - 20), (38, 111, 73), tiny=True)

    def _draw_obstacles(self, state: SimulationState) -> None:
        assert self.screen is not None
        visited = set(state.visited_target_ids)
        for obstacle in state.arena.obstacles:
            bounds = obstacle_bounds(obstacle, self.config.cell_size_cm)
            top_left = self._screen_point(bounds.min_x_cm, bounds.max_y_cm)
            bottom_right = self._screen_point(bounds.max_x_cm, bounds.min_y_cm)
            rect = pygame.Rect(top_left, (bottom_right[0] - top_left[0], bottom_right[1] - top_left[1]))
            is_visited = obstacle.obstacle_id in visited
            color = VISITED_COLOR if is_visited else (91, 101, 115)
            pygame.draw.rect(self.screen, (45, 51, 60), rect.move(2, 3), border_radius=2)
            pygame.draw.rect(self.screen, color, rect, border_radius=2)
            pygame.draw.rect(self.screen, (39, 45, 54), rect, 2, border_radius=2)
            self._blit_text(str(obstacle.obstacle_id), (rect.centerx - 4, rect.centery - 8), self.TEXT, small=True)
            if is_visited:
                pygame.draw.circle(self.screen, (213, 250, 229), (rect.right - 5, rect.top + 5), 5)
                pygame.draw.line(self.screen, VISITED_COLOR, (rect.right - 7, rect.top + 5), (rect.right - 5, rect.top + 7), 2)
                pygame.draw.line(self.screen, VISITED_COLOR, (rect.right - 5, rect.top + 7), (rect.right - 2, rect.top + 2), 2)
            if obstacle.face is not None:
                self._draw_image_face(rect, obstacle.face)

    def _draw_image_face(self, rect: pygame.Rect, face: Direction) -> None:
        assert self.screen is not None
        start, end = self._face_line(rect, face)
        pygame.draw.line(self.screen, (255, 235, 235), start, end, 7)
        pygame.draw.line(self.screen, IMAGE_FACE_COLOR, start, end, 4)
        midpoint = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
        outward = {
            Direction.NORTH: (0, -1),
            Direction.EAST: (1, 0),
            Direction.SOUTH: (0, 1),
            Direction.WEST: (-1, 0),
        }[face]
        tip = (midpoint[0] + outward[0] * 8, midpoint[1] + outward[1] * 8)
        pygame.draw.line(self.screen, IMAGE_FACE_COLOR, midpoint, tip, 2)

    @staticmethod
    def _face_line(rect: pygame.Rect, face: Direction) -> tuple[tuple[int, int], tuple[int, int]]:
        return {
            Direction.NORTH: (rect.topleft, rect.topright),
            Direction.EAST: (rect.topright, rect.bottomright),
            Direction.SOUTH: (rect.bottomleft, rect.bottomright),
            Direction.WEST: (rect.topleft, rect.bottomleft),
        }[face]

    def _draw_camera_rays(self, state: SimulationState) -> None:
        for group in state.candidate_groups:
            for candidate in group.candidates:
                color = CLEAR_RAY_COLOR if candidate.line_of_sight_clear else BLOCKED_RAY_COLOR
                self._draw_dashed_line(
                    self._screen_point(candidate.camera_position.x_cm, candidate.camera_position.y_cm),
                    self._screen_point(candidate.target_point.x_cm, candidate.target_point.y_cm),
                    color,
                    width=1,
                    dash_px=5.0,
                    gap_px=4.0,
                )

    def _draw_candidates(self, state: SimulationState) -> None:
        kind_colors = {
            ObservationCandidateKind.NOMINAL: NOMINAL_COLOR,
            ObservationCandidateKind.LEFT: LEFT_COLOR,
            ObservationCandidateKind.RIGHT: RIGHT_COLOR,
            ObservationCandidateKind.ALTERNATIVE: (137, 143, 153),
        }
        selected = set(state.selected_candidates)
        for group in state.candidate_groups:
            for candidate in group.candidates:
                if selected and (group.obstacle_id, candidate.display_label) not in selected:
                    continue
                pose = candidate.observation_pose.pose
                center = self._screen_point(pose.x_cm, pose.y_cm)
                color = kind_colors[candidate.kind] if candidate.valid else INVALID_COLOR
                self._draw_candidate_marker(center, candidate.kind, color, candidate.valid)
                pygame.draw.line(self.screen, color, center, self._heading_endpoint(pose, 8.0), 2)
                label = f"{group.obstacle_id}:{candidate.display_label}"
                self._blit_text(label, (center[0] + 7, center[1] - 14), self.DARK_TEXT, tiny=True)

    def _draw_candidate_marker(
        self,
        center: tuple[int, int],
        kind: ObservationCandidateKind,
        color: Color,
        valid: bool,
    ) -> None:
        assert self.screen is not None
        x, y = center
        if not valid:
            pygame.draw.circle(self.screen, color, center, 6, 2)
            pygame.draw.line(self.screen, color, (x - 4, y - 4), (x + 4, y + 4), 2)
            pygame.draw.line(self.screen, color, (x - 4, y + 4), (x + 4, y - 4), 2)
            return
        if kind is ObservationCandidateKind.NOMINAL:
            pygame.draw.polygon(self.screen, color, ((x, y - 7), (x + 7, y), (x, y + 7), (x - 7, y)))
        elif kind is ObservationCandidateKind.LEFT:
            pygame.draw.polygon(self.screen, color, ((x, y - 7), (x + 7, y + 6), (x - 7, y + 6)))
        elif kind is ObservationCandidateKind.RIGHT:
            pygame.draw.rect(self.screen, color, pygame.Rect(x - 5, y - 5, 11, 11), border_radius=1)
        else:
            pygame.draw.circle(self.screen, color, center, 6)

    def _draw_debug_nodes(self, nodes: tuple[Pose, ...]) -> None:
        assert self.screen is not None
        for pose in nodes:
            pygame.draw.circle(self.screen, (181, 124, 218), self._screen_point(pose.x_cm, pose.y_cm), 2)

    def _draw_path(self, path: tuple[Pose, ...], color: Color, width: int) -> None:
        assert self.screen is not None
        if len(path) >= 2:
            points = [self._screen_point(pose.x_cm, pose.y_cm) for pose in path]
            pygame.draw.lines(self.screen, (107, 72, 18), False, points, width + 2)
            pygame.draw.lines(self.screen, color, False, points, width)

    def _draw_dashed_path(self, path: tuple[Pose, ...], color: Color, width: int) -> None:
        assert self.screen is not None
        if len(path) < 2:
            return
        points = [self._screen_point(pose.x_cm, pose.y_cm) for pose in path]
        draw_dash = True
        pattern_remaining = 7.0
        for start, end in zip(points, points[1:]):
            delta_x = end[0] - start[0]
            delta_y = end[1] - start[1]
            distance = math.hypot(delta_x, delta_y)
            if distance == 0.0:
                continue
            unit_x, unit_y = delta_x / distance, delta_y / distance
            offset = 0.0
            while offset < distance:
                chunk = min(pattern_remaining, distance - offset)
                if draw_dash:
                    pygame.draw.line(
                        self.screen,
                        color,
                        (round(start[0] + unit_x * offset), round(start[1] + unit_y * offset)),
                        (
                            round(start[0] + unit_x * (offset + chunk)),
                            round(start[1] + unit_y * (offset + chunk)),
                        ),
                        width,
                    )
                offset += chunk
                pattern_remaining -= chunk
                if pattern_remaining <= 1e-9:
                    draw_dash = not draw_dash
                    pattern_remaining = 7.0 if draw_dash else 5.0

    def _draw_dashed_line(
        self,
        start: tuple[int, int],
        end: tuple[int, int],
        color: Color,
        *,
        width: int,
        dash_px: float,
        gap_px: float,
    ) -> None:
        assert self.screen is not None
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        distance = math.hypot(delta_x, delta_y)
        if distance == 0.0:
            return
        unit_x, unit_y = delta_x / distance, delta_y / distance
        offset = 0.0
        while offset < distance:
            dash_end = min(distance, offset + dash_px)
            pygame.draw.line(
                self.screen,
                color,
                (round(start[0] + unit_x * offset), round(start[1] + unit_y * offset)),
                (round(start[0] + unit_x * dash_end), round(start[1] + unit_y * dash_end)),
                width,
            )
            offset += dash_px + gap_px

    def _draw_robot(self, pose: Pose, show_footprint: bool) -> None:
        assert self.screen is not None
        if show_footprint:
            footprint = [
                self._screen_point(point.x_cm, point.y_cm)
                for point in robot_footprint(pose, self.config.robot)
            ]
            overlay = pygame.Surface((self.width_px, self.height_px), pygame.SRCALPHA)
            pygame.draw.polygon(overlay, (*FOOTPRINT_COLOR, 58), footprint)
            self.screen.blit(overlay, (0, 0))
            pygame.draw.polygon(self.screen, FOOTPRINT_COLOR, footprint, 2)

        body = [
            self._screen_point(point.x_cm, point.y_cm)
            for point in robot_footprint(pose, self._physical_body_geometry)
        ]
        pygame.draw.polygon(self.screen, (17, 45, 82), body)
        pygame.draw.polygon(self.screen, (102, 168, 238), body, 2)

        half_length = self.config.robot.length_cm / 2.0
        half_width = self.config.robot.width_cm / 2.0
        windshield = self._local_polygon(
            pose,
            forward_min=half_length * 0.08,
            forward_max=half_length * 0.62,
            left_min=-half_width * 0.68,
            left_max=half_width * 0.68,
        )
        pygame.draw.polygon(self.screen, (65, 139, 194), windshield)
        pygame.draw.polygon(self.screen, (146, 211, 241), windshield, 1)

        for forward in (-half_length * 0.48, half_length * 0.48):
            for left in (-half_width * 1.02, half_width * 1.02):
                wheel = self._local_polygon(
                    pose,
                    forward_min=forward - 2.2,
                    forward_max=forward + 2.2,
                    left_min=left - 1.2,
                    left_max=left + 1.2,
                )
                pygame.draw.polygon(self.screen, (25, 28, 34), wheel)

        front_left = self._screen_point(*pose.translated_local(half_length, half_width * 0.78))
        front_right = self._screen_point(*pose.translated_local(half_length, -half_width * 0.78))
        pygame.draw.line(self.screen, (110, 222, 241), front_left, front_right, 3)

        axle = self._screen_point(pose.x_cm, pose.y_cm)
        axle_left = self._screen_point(*pose.translated_local(0.0, half_width * 0.72))
        axle_right = self._screen_point(*pose.translated_local(0.0, -half_width * 0.72))
        pygame.draw.line(self.screen, AXLE_COLOR, axle_left, axle_right, 2)
        pygame.draw.circle(self.screen, (13, 25, 42), axle, 5)
        pygame.draw.circle(self.screen, AXLE_COLOR, axle, 5, 2)

        heading_end = self._heading_endpoint(pose, half_length + 7.0)
        pygame.draw.line(self.screen, (110, 222, 241), axle, heading_end, 3)
        camera = camera_world_position(pose, self.config.camera)
        camera_screen = self._screen_point(camera.x_cm, camera.y_cm)
        pygame.draw.circle(self.screen, (255, 225, 230), camera_screen, 5)
        pygame.draw.circle(self.screen, (230, 66, 91), camera_screen, 4)

    def _local_polygon(
        self,
        pose: Pose,
        *,
        forward_min: float,
        forward_max: float,
        left_min: float,
        left_max: float,
    ) -> tuple[tuple[int, int], ...]:
        local_corners = (
            (forward_max, left_max),
            (forward_min, left_max),
            (forward_min, left_min),
            (forward_max, left_min),
        )
        return tuple(self._screen_point(*pose.translated_local(forward, left)) for forward, left in local_corners)

    def _heading_endpoint(self, pose: Pose, length_cm: float) -> tuple[int, int]:
        x_cm = pose.x_cm + length_cm * math.cos(pose.heading_rad)
        y_cm = pose.y_cm + length_cm * math.sin(pose.heading_rad)
        return self._screen_point(x_cm, y_cm)

    def _draw_sidebar(self, state: SimulationState, playback_speed: float) -> None:
        assert self.screen is not None
        panel_left = round(self.viewport.left_px + self.viewport.size_px + 25)
        panel = pygame.Rect(
            panel_left,
            round(self.viewport.top_px),
            self.width_px - panel_left - 25,
            round(self.viewport.size_px),
        )
        pygame.draw.rect(self.screen, self.PANEL, panel, border_radius=10)
        pygame.draw.rect(self.screen, self.PANEL_BORDER, panel, 1, border_radius=10)
        x = panel.left + 20
        self._blit_text("TASK 1 SIMULATOR", (x, panel.top + 18), self.TEXT, title=True)
        self._blit_text("Deterministic command-aligned playback", (x, panel.top + 48), self.MUTED_TEXT, small=True)

        state_colors = {
            "ready": (94, 166, 231),
            "playing": (53, 190, 120),
            "paused": (238, 177, 62),
            "complete": (151, 112, 220),
        }
        state_text = state.playback_state.value.upper()
        badge = pygame.Rect(x, panel.top + 78, max(72, len(state_text) * 10 + 20), 25)
        pygame.draw.rect(self.screen, state_colors[state.playback_state.value], badge, border_radius=12)
        self._blit_text(state_text, (badge.left + 10, badge.top + 5), (18, 25, 32), tiny=True)

        command_descriptions = {
            "FW": "forward straight",
            "BW": "reverse straight",
            "FL": "forward left",
            "FR": "forward right",
            "BL": "reverse left",
            "BR": "reverse right",
        }
        command = state.current_motion_command or "-"
        pose = state.robot_pose
        heading_degrees = math.degrees(pose.heading_rad) % 360.0
        total_targets = len(state.arena.obstacles)
        visited_text = ", ".join(str(item) for item in state.visited_target_ids) or "none"
        target_order = " > ".join(str(item) for item in state.target_order) or "-"
        selected_candidates = ", ".join(
            f"{obstacle_id}:{kind.upper()}"
            for obstacle_id, kind in state.selected_candidates
        ) or "-"
        try:
            heading_cardinal = Direction.from_heading_rad(pose.heading_rad).value
        except ValueError:
            heading_cardinal = "NON-CARDINAL"
        status_lines = (
            ("Command", f"{command}  {command_descriptions.get(command, '')}".rstrip()),
            ("Sample", f"{state.current_step_index} / {state.total_steps}"),
            ("Logical time", f"{state.simulation_time_s:.2f} s"),
            ("Playback", f"{playback_speed:g}x"),
            ("Pose", f"({pose.x_cm:.1f}, {pose.y_cm:.1f}) cm"),
            ("Heading", f"{heading_degrees:.0f} deg ({heading_cardinal})"),
            ("Visited", f"{len(state.visited_target_ids)}/{total_targets}  [{visited_text}]"),
            ("Order", target_order),
            ("Selected", selected_candidates),
        )
        y = panel.top + 116
        for label, value in status_lines:
            self._blit_text(label.upper(), (x, y), self.MUTED_TEXT, tiny=True)
            self._blit_text(value, (x + 91, y - 1), self.TEXT, small=True)
            y += 21

        legend_y = y + 5
        self._draw_section_title("LEGEND", x, legend_y, panel.width - 40)
        self._draw_legend(x, legend_y + 23, panel.width - 40)

        controls_y = legend_y + 165
        self._draw_section_title("CONTROLS", x, controls_y, panel.width - 40)
        self._draw_controls(x, controls_y + 23, panel.width - 40)

        footer_y = panel.bottom - 24
        self._blit_text("World: 200 x 200 cm  |  Grid: 20 x 20", (x, footer_y), self.MUTED_TEXT, tiny=True)

    def _draw_section_title(self, title: str, x: int, y: int, width: int) -> None:
        assert self.screen is not None
        self._blit_text(title, (x, y), self.MUTED_TEXT, tiny=True)
        pygame.draw.line(self.screen, self.PANEL_BORDER, (x + 65, y + 7), (x + width, y + 7), 1)

    def _draw_legend(self, x: int, y: int, width: int) -> None:
        column_width = max(145, width // 2)
        for index, item in enumerate(simulator_legend_items()):
            column = index // 6
            row = index % 6
            item_x = x + column * column_width
            item_y = y + row * 21
            self._draw_legend_symbol(item, (item_x + 7, item_y + 7))
            self._blit_text(item.label, (item_x + 19, item_y), self.TEXT, tiny=True)

    def _draw_legend_symbol(self, item: LegendItem, center: tuple[int, int]) -> None:
        assert self.screen is not None
        x, y = center
        if item.style == "diamond":
            pygame.draw.polygon(self.screen, item.color, ((x, y - 5), (x + 5, y), (x, y + 5), (x - 5, y)))
        elif item.style == "triangle":
            pygame.draw.polygon(self.screen, item.color, ((x, y - 5), (x + 5, y + 4), (x - 5, y + 4)))
        elif item.style in {"square", "filled"}:
            pygame.draw.rect(self.screen, item.color, pygame.Rect(x - 4, y - 4, 9, 9), border_radius=1)
        elif item.style == "invalid":
            pygame.draw.circle(self.screen, item.color, center, 5, 1)
            pygame.draw.line(self.screen, item.color, (x - 3, y - 3), (x + 3, y + 3), 1)
            pygame.draw.line(self.screen, item.color, (x - 3, y + 3), (x + 3, y - 3), 1)
        elif item.style == "ray":
            pygame.draw.line(self.screen, CLEAR_RAY_COLOR, (x - 7, y), (x - 1, y), 2)
            pygame.draw.line(self.screen, BLOCKED_RAY_COLOR, (x + 2, y), (x + 7, y), 2)
        elif item.style == "dashed":
            pygame.draw.line(self.screen, item.color, (x - 6, y), (x - 1, y), 2)
            pygame.draw.line(self.screen, item.color, (x + 2, y), (x + 7, y), 2)
        elif item.style == "line":
            pygame.draw.line(self.screen, item.color, (x - 7, y), (x + 7, y), 3)
        elif item.style == "face":
            pygame.draw.line(self.screen, item.color, (x - 6, y), (x + 6, y), 4)
        elif item.style == "axle":
            pygame.draw.circle(self.screen, item.color, center, 4, 1)
            pygame.draw.line(self.screen, item.color, center, (x + 8, y), 2)
        elif item.style == "visited":
            pygame.draw.rect(self.screen, item.color, pygame.Rect(x - 5, y - 4, 11, 9), border_radius=1)
        else:
            pygame.draw.rect(self.screen, item.color, pygame.Rect(x - 5, y - 4, 11, 9), 1)

    def _draw_controls(self, x: int, y: int, width: int) -> None:
        controls = (
            ("Space", "play / pause"),
            ("N / Right", "step primitive/event"),
            ("R", "reset"),
            ("+ / -", "playback speed"),
            ("G", "grid labels"),
            ("C / L", "candidates / rays"),
            ("F", "safety footprint"),
            ("P / E", "planned / executed"),
            ("D", "debug nodes"),
            ("Q / Esc", "quit"),
        )
        column_width = max(145, width // 2)
        for index, (key, action) in enumerate(controls):
            column = index // 5
            row = index % 5
            item_x = x + column * column_width
            item_y = y + row * 21
            self._blit_text(key, (item_x, item_y), (118, 199, 239), tiny=True)
            self._blit_text(action, (item_x + 49, item_y), self.TEXT, tiny=True)

    def _blit_text(
        self,
        text: str,
        position: tuple[int, int],
        color: Color,
        *,
        title: bool = False,
        small: bool = False,
        tiny: bool = False,
    ) -> None:
        assert self.screen is not None
        assert self._title_font is not None and self._font is not None
        assert self._small_font is not None and self._tiny_font is not None
        font = self._title_font if title else self._tiny_font if tiny else self._small_font if small else self._font
        self.screen.blit(font.render(text, True, color), position)


__all__ = ["LegendItem", "PygameRenderer", "RenderOptions", "simulator_legend_items"]
