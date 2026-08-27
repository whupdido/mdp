"""Camera, image-face, and viewing-ray geometry."""

from __future__ import annotations

from algorithm.config import CameraGeometry, PlanningConfig
from algorithm.enums import Direction
from algorithm.geometry import obstacle_bounds, segment_intersects_rectangle
from algorithm.geometry.shapes import Point
from algorithm.models.arena import ArenaInput
from algorithm.models.obstacle import Obstacle
from algorithm.models.pose import Pose


def image_face_target_point(
    obstacle: Obstacle,
    face: Direction,
    cell_size_cm: float,
) -> Point:
    """Return the center of the annotated obstacle face."""
    bounds = obstacle_bounds(obstacle, cell_size_cm)
    center_x = (bounds.min_x_cm + bounds.max_x_cm) / 2.0
    center_y = (bounds.min_y_cm + bounds.max_y_cm) / 2.0
    return {
        Direction.NORTH: Point(center_x, bounds.max_y_cm),
        Direction.EAST: Point(bounds.max_x_cm, center_y),
        Direction.SOUTH: Point(center_x, bounds.min_y_cm),
        Direction.WEST: Point(bounds.min_x_cm, center_y),
    }[face]


def desired_camera_position(
    obstacle: Obstacle,
    face: Direction,
    lateral_offset_cm: float,
    config: PlanningConfig,
) -> Point:
    """Place the camera at configured normal gap and face-tangent offset.

    The offset is measured along the face tangent obtained by rotating the
    outward face normal counter-clockwise. The configured default ordering
    ``(0, -10, +10)`` therefore represents nominal, robot-left, robot-right.
    """
    target = image_face_target_point(obstacle, face, config.cell_size_cm)
    normal_x, normal_y = face.grid_vector
    tangent_x, tangent_y = -normal_y, normal_x
    return Point(
        target.x_cm + normal_x * config.camera.image_gap_cm + tangent_x * lateral_offset_cm,
        target.y_cm + normal_y * config.camera.image_gap_cm + tangent_y * lateral_offset_cm,
    )


def camera_world_position(pose: Pose, camera: CameraGeometry) -> Point:
    """Transform the configured rear-axle-to-camera offset into world space."""
    x_cm, y_cm = pose.translated_local(camera.forward_offset_cm, camera.left_offset_cm)
    return Point(x_cm, y_cm)


def rear_axle_pose_for_camera(
    camera_position: Point,
    heading: Direction,
    camera: CameraGeometry,
) -> Pose:
    """Solve the rear-axle pose that places the camera at a desired point."""
    camera_pose = Pose.from_direction(camera_position.x_cm, camera_position.y_cm, heading)
    rear_x, rear_y = camera_pose.translated_local(-camera.forward_offset_cm, -camera.left_offset_cm)
    return Pose.from_direction(rear_x, rear_y, heading)


def has_clear_line_of_sight(
    camera_position: Point,
    target_point: Point,
    target_obstacle_id: int,
    arena: ArenaInput,
    config: PlanningConfig,
) -> bool:
    """Check the camera-to-face segment against every non-target obstacle."""
    return not any(
        obstacle.obstacle_id != target_obstacle_id
        and segment_intersects_rectangle(
            camera_position,
            target_point,
            obstacle_bounds(obstacle, config.cell_size_cm),
        )
        for obstacle in arena.obstacles
    )
