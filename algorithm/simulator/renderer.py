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
from algorithm.models.planning import PlanningResult
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


@dataclass(frozen=True, slots=True)
class EditorPanelData:
    """Display-only editor details supplied by the Pygame frontend."""

    state_label: str
    status_message: str
    selected_obstacle: str = "none"


@dataclass(frozen=True, slots=True)
class PanelSection:
    """A bounded dashboard section calculated from the current panel size."""

    name: str
    rect: pygame.Rect


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

    # Keep the initial window comfortable on a normal laptop display.  The
    # surface is deliberately a standard decorated OS window: RESIZABLE is
    # the only display flag, so no fullscreen, borderless, or scaled mode is
    # requested.
    DEFAULT_WINDOW_SIZE = (1600, 900)
    MIN_WINDOW_SIZE = (1200, 720)
    DISPLAY_FLAGS = pygame.RESIZABLE

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
        width_px: int = DEFAULT_WINDOW_SIZE[0],
        height_px: int = DEFAULT_WINDOW_SIZE[1],
        title: str = "MDP Task 1 Simulator",
    ) -> None:
        if width_px < 640 or height_px < 520:
            raise ValueError("renderer window must be at least 640x520 pixels")
        self.config = config
        self.width_px = width_px
        self.height_px = height_px
        self.title = title
        self.viewport = self._viewport_for_size(width_px, height_px)
        # The stylized body uses the same authoritative footprint transform,
        # but removes safety margin for display only. Collision code continues
        # to use ``config.robot`` unchanged.
        self._physical_body_geometry = replace(config.robot, safety_margin_cm=0.0)
        self.screen: pygame.Surface | None = None
        self._title_font: pygame.font.Font | None = None
        self._section_font: pygame.font.Font | None = None
        self._font: pygame.font.Font | None = None
        self._small_font: pygame.font.Font | None = None
        self._tiny_font: pygame.font.Font | None = None

    def initialize(self) -> pygame.Surface:
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((self.width_px, self.height_px), self.DISPLAY_FLAGS)
        pygame.display.set_caption(self.title)
        self._title_font = pygame.font.Font(None, 31)
        self._section_font = pygame.font.Font(None, 20)
        self._section_font.set_bold(True)
        self._font = pygame.font.Font(None, 24)
        self._small_font = pygame.font.Font(None, 19)
        self._tiny_font = pygame.font.Font(None, 16)
        return self.screen

    def resize(self, width_px: int, height_px: int) -> pygame.Surface:
        """Apply a real window resize and recompute all world/layout geometry."""
        width_px = max(int(width_px), self.MIN_WINDOW_SIZE[0])
        height_px = max(int(height_px), self.MIN_WINDOW_SIZE[1])
        self.width_px = width_px
        self.height_px = height_px
        self.viewport = self._viewport_for_size(width_px, height_px)
        # Pygame may already have installed the resized display surface (for
        # WINDOWSIZECHANGED/WINDOWRESIZED).  Reusing it avoids tearing down a
        # normal decorated Windows window on every resize notification.  A
        # VIDEORESIZE event, or a resize clamped to our usable minimum, still
        # gets one explicit set_mode call with the same RESIZABLE flags.
        current = pygame.display.get_surface()
        if current is not None and current.get_size() == (width_px, height_px):
            self.screen = current
        else:
            self.screen = pygame.display.set_mode((width_px, height_px), self.DISPLAY_FLAGS)
        pygame.display.set_caption(self.title)
        return self.screen

    def _viewport_for_size(self, width_px: int, height_px: int) -> WorldViewport:
        arena_px = float(max(1, min(height_px - 80, width_px - 480)))
        return WorldViewport(self.config.arena_size_cm, 40.0, 40.0, arena_px)

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
        planning_result: PlanningResult | None = None,
        editor_data: EditorPanelData | None = None,
    ) -> None:
        if any(font is None for font in (self._title_font, self._section_font, self._font, self._small_font, self._tiny_font)):
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
        self._draw_sidebar(state, playback_speed, planning_result, editor_data)

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

    def panel_rect(self) -> pygame.Rect:
        """Return the right-hand panel rectangle for the current window."""
        panel_left = round(self.viewport.left_px + self.viewport.size_px + 25)
        return pygame.Rect(
            panel_left,
            round(self.viewport.top_px),
            max(1, self.width_px - panel_left - 25),
            round(self.viewport.size_px),
        )

    def _sidebar_sections(
        self,
        panel: pygame.Rect,
        *,
        has_route: bool,
        has_editor: bool,
    ) -> dict[str, PanelSection]:
        """Budget the complete dashboard before drawing any row content.

        Header, playback, route summary, editor state, legend, controls, and
        the footer receive predictable space first.  Diagnostics are the flex
        section and are compressed when a planned route needs more rows.
        """
        compact = panel.height < 700 or panel.width < 500
        padding = 12 if compact else 20
        gap = 5 if compact else 8
        footer_height = 18 if compact else 20
        if compact:
            preferred = {
                "header": 40,
                "live": 96,
                "route": 125 if has_route else 75,
                "diagnostics": 88,
                "editor": 48,
                "legend": 30,
                "controls": 115,
            }
        else:
            preferred = {
                "header": 60,
                "live": 145,
                "route": 175 if has_route else 90,
                "diagnostics": 92,
                "editor": 76,
                "legend": 30,
                "controls": 95,
            }
        names = ["header", "live", "route", "diagnostics"]
        if has_editor:
            names.append("editor")
        names.extend(("legend", "controls"))
        available = panel.height - 2 * padding - footer_height - gap * (len(names) - 1)
        fixed_names = [name for name in names if name != "diagnostics"]
        fixed_height = sum(preferred[name] for name in fixed_names)
        diagnostics_min = 66 if compact else 78
        heights = {name: preferred[name] for name in fixed_names}
        heights["diagnostics"] = max(diagnostics_min, available - fixed_height)

        # Supported windows have enough room for the minimum diagnostics area.
        # If an even smaller surface is supplied, retain the priority order by
        # compressing diagnostics only; the footer remains outside the section
        # stack and controls are never pushed below it.
        total = sum(heights.values())
        if total > available:
            heights["diagnostics"] = max(1, heights["diagnostics"] - (total - available))
        sections: dict[str, PanelSection] = {}
        y = panel.top + padding
        for name in names:
            rect = pygame.Rect(panel.left + padding, y, panel.width - 2 * padding, heights[name])
            sections[name] = PanelSection(name, rect)
            y += heights[name] + gap
        return sections

    def _draw_sidebar(
        self,
        state: SimulationState,
        playback_speed: float,
        planning_result: PlanningResult | None = None,
        editor_data: EditorPanelData | None = None,
    ) -> None:
        assert self.screen is not None
        panel = self.panel_rect()
        sections = self._sidebar_sections(
            panel,
            has_route=planning_result is not None
            or editor_data is not None and editor_data.state_label == "PLANNING",
            has_editor=editor_data is not None,
        )
        pygame.draw.rect(self.screen, self.PANEL, panel, border_radius=10)
        pygame.draw.rect(self.screen, self.PANEL_BORDER, panel, 1, border_radius=10)
        self._draw_header(sections["header"], editor_data is not None)
        self._draw_live_section(sections["live"], state, playback_speed)
        planning = editor_data is not None and editor_data.state_label == "PLANNING"
        self._draw_route_section(sections["route"], state, planning_result, planning=planning)
        self._draw_diagnostics_section(sections["diagnostics"], planning_result, planning=planning)
        if editor_data is not None:
            self._draw_editor_section(sections["editor"], editor_data)
        self._draw_legend_section(sections["legend"], compact=sections["legend"].rect.height < 130)
        self._draw_controls_section(
            sections["controls"],
            compact=sections["controls"].rect.height < 90,
            editor=editor_data is not None,
        )
        footer = "World: 200 x 200 cm  |  Grid: 20 x 20"
        self._blit_text(footer, (panel.left + 16, panel.bottom - 18), self.MUTED_TEXT, tiny=True)

    def _draw_header(self, section: PanelSection, editor: bool) -> None:
        rect = section.rect
        x, y = rect.left, rect.top
        self._blit_text("TASK 1 EDITOR" if editor else "TASK 1 SIMULATOR", (x, y), self.TEXT, title=True)
        self._draw_robot_reference_legend(rect)
        subtitle = "Edit, plan, and play a five-target route" if editor else "Deterministic command-aligned playback"
        self._blit_text(self._fit_text(subtitle, self._small_font, rect.width), (x, y + 30), self.MUTED_TEXT, small=True)

    def _draw_robot_reference_legend(self, section: PanelSection) -> None:
        """Draw the actual camera and rear-axle marker vocabulary inline."""
        assert self.screen is not None and self._tiny_font is not None
        full_labels = ("Camera/front", "Rear axle")
        short_labels = ("Camera", "Axle")
        gap = 14

        def width_for(labels: tuple[str, str]) -> int:
            return sum(20 + self._tiny_font.size(label)[0] for label in labels) + gap

        labels = full_labels if width_for(full_labels) + 20 <= section.width else short_labels
        total_width = width_for(labels)
        if total_width + 20 > section.width:
            labels = ("", "")
            total_width = 42
        start_x = section.right - total_width
        center_y = section.top + 12
        entries = (
            ((230, 66, 91), labels[0]),
            (AXLE_COLOR, labels[1]),
        )
        cursor = start_x
        for index, (color, label) in enumerate(entries):
            if index == 0:
                pygame.draw.circle(self.screen, (255, 225, 230), (cursor + 6, center_y), 5)
                pygame.draw.circle(self.screen, color, (cursor + 6, center_y), 4)
            else:
                pygame.draw.circle(self.screen, (13, 25, 42), (cursor + 6, center_y), 5)
                pygame.draw.circle(self.screen, color, (cursor + 6, center_y), 5, 2)
            if label:
                self._blit_text(label, (cursor + 16, section.top + 6), self.MUTED_TEXT, tiny=True)
            cursor += 20 + self._tiny_font.size(label)[0]
            if index == 0:
                cursor += gap

    def _draw_live_section(self, section: PanelSection, state: SimulationState, playback_speed: float) -> None:
        rect = section.rect
        self._draw_section_title("LIVE PLAYBACK", section)
        command_descriptions = {
            "FW": "forward", "BW": "reverse", "FL": "forward left", "FR": "forward right",
            "BL": "reverse left", "BR": "reverse right",
        }
        command = state.current_motion_command or "-"
        pose = state.robot_pose
        try:
            heading_cardinal = Direction.from_heading_rad(pose.heading_rad).value
        except ValueError:
            heading_cardinal = "NON-CARDINAL"
        total_targets = len(state.arena.obstacles)
        visited = ", ".join(str(item) for item in state.visited_target_ids) or "none"
        rows = (
            ("Status", state.playback_state.value.upper()),
            ("Command", f"{command} ({command_descriptions.get(command, 'idle')})"),
            ("Sample", f"{state.current_step_index} / {state.total_steps}"),
            ("Logical time", f"{state.simulation_time_s:.2f} s"),
            ("Playback", f"{playback_speed:g}x"),
            ("Pose", f"({pose.x_cm:.1f}, {pose.y_cm:.1f}) cm"),
            ("Heading", f"{math.degrees(pose.heading_rad) % 360.0:.0f} deg ({heading_cardinal})"),
            ("Visited", f"{len(state.visited_target_ids)}/{total_targets} [{visited}]"),
        )
        if rect.height < 120:
            rows = (
                ("Status", state.playback_state.value.upper()),
                ("Command", f"{command} ({command_descriptions.get(command, 'idle')})"),
                ("Sample", f"{state.current_step_index} / {state.total_steps}"),
                ("Logical", f"{state.simulation_time_s:.2f}s  speed {playback_speed:g}x"),
                ("Pose", f"({pose.x_cm:.1f}, {pose.y_cm:.1f})  {math.degrees(pose.heading_rad) % 360.0:.0f} deg"),
                ("Visited", f"{len(state.visited_target_ids)}/{total_targets} [{visited}]"),
            )
        self._draw_rows(section, rows, emphasize_first=True)

    def _draw_route_section(
        self,
        section: PanelSection,
        state: SimulationState,
        planning_result: PlanningResult | None,
        *,
        planning: bool = False,
    ) -> None:
        rect = section.rect
        self._draw_section_title("ROUTE SUMMARY", section)
        route = planning_result.route if planning_result is not None else None
        if planning:
            rows = (
                ("Status", "PLANNING"),
                ("Planning time", "in progress"),
                ("Route time", "-"),
                ("Distance", "-"),
                ("Order", "-"),
                ("Candidates", "-"),
                ("Primitives", "-"),
            )
        elif route is not None:
            selected = ", ".join(
                f"{target}:{kind.upper()}"
                for target, kind in zip(route.target_order, route.selected_candidate_kinds)
            ) or "-"
            rows = (
                ("Status", planning_result.status.value.upper()),
                ("Planning time", f"{planning_result.metrics.total_planning_time_s:.3f} s"),
                ("Route time", f"{route.metrics.estimated_time_s:.2f} s"),
                ("Distance", f"{route.metrics.geometric_distance_cm:.1f} cm"),
                ("Order", " -> ".join(map(str, route.target_order)) or "-"),
                ("Candidates", selected),
                ("Primitives", _primitive_summary(route)),
            )
        else:
            target_order = " -> ".join(map(str, state.target_order)) or "-"
            selected = ", ".join(
                f"{target}:{kind.upper()}" for target, kind in state.selected_candidates
            ) or "-"
            if planning_result is not None:
                rows = (
                    ("Status", planning_result.status.value.upper()),
                    ("Planning time", f"{planning_result.metrics.total_planning_time_s:.3f} s"),
                    ("Route time", "-"),
                    ("Distance", "-"),
                    ("Order", target_order),
                    ("Candidates", selected),
                    ("Primitives", "-"),
                )
            else:
                rows = (("Status", "PLAYBACK ONLY"), ("Order", target_order), ("Candidates", selected))
        self._draw_rows(section, rows, emphasize_labels={"Planning time", "Status"})

    def _draw_diagnostics_section(
        self,
        section: PanelSection,
        result: PlanningResult | None,
        *,
        planning: bool = False,
    ) -> None:
        rect = section.rect
        self._draw_section_title("PLANNER DIAGNOSTICS", section)
        if planning:
            rows = (("Status", "planning in background"),)
        elif result is None:
            rows = (("Mode", "headless route playback"),)
        else:
            metrics = result.metrics
            reachability = " ".join(
                f"{item.target_id}:{item.reachable_candidates}/{item.geometric_candidates}"
                for item in metrics.target_reachability
            ) or "-"
            rows = (
                ("Timing", f"C {metrics.candidate_generation_time_s:.2f} | P {metrics.pairwise_planning_time_s:.2f} | G {metrics.global_routing_time_s:.2f} | T {metrics.total_planning_time_s:.2f}"),
                ("Search", f"H/M {metrics.cache_hits}/{metrics.pairwise_cache_misses} | retries {metrics.hybrid_astar_retries}/{metrics.hybrid_astar_retry_recoveries} | nodes {metrics.total_nodes_expanded:,}"),
                ("Reachable", reachability),
            )
        self._draw_rows(section, rows)

    def _draw_editor_section(self, section: PanelSection, data: EditorPanelData) -> None:
        rect = section.rect
        self._draw_section_title("EDITOR STATE", section)
        if rect.height < 70:
            rows = (
                ("State", data.state_label),
                ("Selected", f"{data.selected_obstacle} | {data.status_message}"),
            )
        else:
            rows = (
                ("State", data.state_label),
                ("Selected", data.selected_obstacle),
                ("Message", data.status_message),
            )
        self._draw_rows(section, rows)

    def _draw_legend_section(self, section: PanelSection, *, compact: bool) -> None:
        rect = section.rect
        self._draw_section_title("LEGEND", section)
        if compact:
            self._blit_text(
                self._fit_text("C center | L left | R right | dashed planned | solid executed", self._tiny_font, rect.width - 76),
                (rect.left + 76, rect.top + 3),
                self.TEXT,
                tiny=True,
            )
            return
        self._draw_legend(rect.left, rect.top + 27, rect.width, compact=False)

    def _draw_controls_section(self, section: PanelSection, *, compact: bool, editor: bool) -> None:
        rect = section.rect
        self._draw_section_title("CONTROLS", section)
        if editor:
            controls = (
                ("Enter", "plan route"), ("Space", "play / pause"),
                ("Left", "previous"), ("Right", "next"),
                ("R", "reset playback"), ("F5", "random arena"), ("Shift+F5", "verified random"),
                ("N", "toggle candidates"), ("W/A/S/D", "change image face"),
                ("Left click", "select / add / move"), ("Right click", "remove obstacle"),
                ("+ / -", "playback speed"), ("Q / Esc", "quit"),
            )
        else:
            controls = (
                ("Space", "play / pause"), ("Left", "previous"), ("Right", "next"), ("R", "reset playback"),
                ("+ / -", "playback speed"), ("N", "candidates"), ("G", "grid labels"),
                ("C / L", "candidates / rays"), ("F", "footprint"), ("P / E", "planned / executed"),
                ("D", "debug nodes"), ("Q / Esc", "quit"),
            )
        columns = 2 if rect.width < 520 else (2 if compact else 3)
        row_height = max(15, min(23, (rect.height - 28) // max(1, (len(controls) + columns - 1) // columns)))
        for index, (key, action) in enumerate(controls):
            column = index % columns
            row = index // columns
            x = rect.left + column * (rect.width // columns)
            y = rect.top + 24 + row * row_height
            self._blit_text(key, (x, y), (118, 199, 239), tiny=True)
            key_width = self._tiny_font.size(key)[0] if self._tiny_font is not None else 52
            action_x = x + max(52, key_width + 10)
            self._blit_text(
                self._fit_text(action, self._tiny_font, rect.width // columns - (action_x - x)),
                (action_x, y),
                self.TEXT,
                tiny=True,
            )

    def _draw_section_title(self, title: str, section: PanelSection) -> None:
        rect = section.rect
        self._blit_text(title, (rect.left, rect.top), self.TEXT, section=True)
        line_start = min(rect.left + 160, rect.right - 12)
        pygame.draw.line(self.screen, self.PANEL_BORDER, (line_start, rect.top + 9), (rect.right, rect.top + 9), 1)

    def _draw_rows(
        self,
        section: PanelSection,
        rows: tuple[tuple[str, str], ...],
        *,
        emphasize_first: bool = False,
        emphasize_labels: set[str] | None = None,
    ) -> None:
        rect = section.rect
        compact = rect.height < 150
        top = rect.top + 23
        row_height = max(10 if compact else 12, min(25, (rect.height - 25) // max(1, len(rows))))
        label_width = min(126 if not compact else 92, max(70, rect.width // 3))
        for index, (label, value) in enumerate(rows):
            y = top + index * row_height
            emphasized = label in (emphasize_labels or set()) or (emphasize_first and index == 0)
            status_color = {
                "READY": (94, 166, 231),
                "READY TO PLAN": (94, 166, 231),
                "PLANNING": (246, 190, 82),
                "SUCCESS": (83, 207, 132),
                "COMPLETE": (177, 137, 239),
                "PLAYING": (83, 207, 132),
                "PAUSED": (246, 190, 82),
                "NO_PATH": (239, 112, 112),
                "NO_FEASIBLE_ROUTE": (239, 112, 112),
                "FAILURE": (239, 112, 112),
            }.get(value.upper())
            label_color = (255, 237, 151) if emphasized else self.MUTED_TEXT
            self._blit_text(label.upper(), (rect.left, y), label_color, tiny=True)
            value_font = self._tiny_font if compact else self._small_font
            value_color = status_color or ((255, 237, 151) if emphasized else self.TEXT)
            max_width = rect.width - label_width - 6
            self._blit_text(
                self._fit_text(value, value_font, max_width),
                (rect.left + label_width, y - 1),
                value_color,
                tiny=compact,
                small=not compact,
            )

    @staticmethod
    def _fit_text(text: str, font: pygame.font.Font | None, max_width: int) -> str:
        """Return text that fits one row, adding an ellipsis when needed."""
        if font is None or max_width <= 0:
            return ""
        if font.size(text)[0] <= max_width:
            return text
        ellipsis = "..."
        if font.size(ellipsis)[0] >= max_width:
            return ellipsis
        trimmed = text
        while trimmed and font.size(trimmed + ellipsis)[0] > max_width:
            trimmed = trimmed[:-1]
        return trimmed.rstrip() + ellipsis

    def _draw_legend(self, x: int, y: int, width: int, *, compact: bool = False) -> None:
        items = simulator_legend_items()
        if compact:
            items = tuple(item for item in items if item.label in {
                "Nominal candidate", "Invalid candidate", "Planned path", "Executed path",
                "Visited target", "Rear axle / heading",
            })
        column_width = max(145, width // 2)
        rows = 3 if compact else 6
        for index, item in enumerate(items):
            column = index // rows
            row = index % rows
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
        section: bool = False,
        small: bool = False,
        tiny: bool = False,
    ) -> None:
        assert self.screen is not None
        assert self._title_font is not None and self._section_font is not None and self._font is not None
        assert self._small_font is not None and self._tiny_font is not None
        font = (
            self._title_font if title
            else self._section_font if section
            else self._tiny_font if tiny
            else self._small_font if small
            else self._font
        )
        self.screen.blit(font.render(text, True, color), position)


def _human_candidate_label(label: str) -> str:
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


__all__ = [
    "EditorPanelData",
    "LegendItem",
    "PanelSection",
    "PygameRenderer",
    "RenderOptions",
    "simulator_legend_items",
]
