"""Versioned physical-run records and exact historical-route playback."""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from algorithm.config import PlanningConfig
from algorithm.enums import CostMetric, Direction, Gear, RoutingMode, Steering
from algorithm.geometry import sample_motion
from algorithm.geometry.footprint import robot_footprint
from algorithm.geometry.shapes import Point
from algorithm.geometry.collision import polygons_intersect
from algorithm.models import ArenaInput, GridCell, Obstacle, Pose
from algorithm.models.motion import CaptureStep, MotionPrimitive, MoveStep
from algorithm.serialization import serialize_planning_result, serialize_stm_command
from algorithm.targets.geometry import camera_world_position, image_face_target_point

from .headless import HeadlessSimulator, SimulationStep

SCHEMA_VERSION = 1


def pose_record(pose: Pose) -> dict[str, float]:
    return {"x_cm": pose.x_cm, "y_cm": pose.y_cm, "heading_rad": pose.heading_rad}


def pose_from_record(data: dict[str, Any]) -> Pose:
    return Pose(float(data["x_cm"]), float(data["y_cm"]), float(data["heading_rad"]))


def compare_capture_poses(planned: Pose, actual: Pose) -> dict[str, float]:
    heading = math.atan2(math.sin(actual.heading_rad-planned.heading_rad),
                         math.cos(actual.heading_rad-planned.heading_rad))
    return {
        "x_error_cm": actual.x_cm-planned.x_cm,
        "y_error_cm": actual.y_cm-planned.y_cm,
        "position_error_cm": math.hypot(actual.x_cm-planned.x_cm, actual.y_cm-planned.y_cm),
        "heading_error_deg": math.degrees(heading),
    }


def arena_record(arena: ArenaInput) -> dict[str, Any]:
    return {
        "start": pose_record(arena.start_pose),
        "obstacles": [
            {"id": item.obstacle_id, "x": item.cell.x, "y": item.cell.y,
             "face": item.face.value if item.face else None, "image_id": item.image_id}
            for item in arena.obstacles
        ],
    }


def arena_from_record(data: dict[str, Any]) -> ArenaInput:
    return ArenaInput(
        pose_from_record(data["start"]),
        tuple(Obstacle(int(item["id"]), GridCell(int(item["x"]), int(item["y"])),
                       Direction.from_token(item["face"]) if item.get("face") else None,
                       item.get("image_id")) for item in data["obstacles"]),
    )


def config_record(config: PlanningConfig, *, objective: CostMetric = CostMetric.ESTIMATED_TIME,
                  routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION) -> dict[str, Any]:
    return {
        "arena_size_cm": config.arena_size_cm,
        "cell_size_cm": config.cell_size_cm,
        "robot": asdict(config.robot), "camera": asdict(config.camera),
        "safety_margin_cm": config.robot.safety_margin_cm,
        "objective": objective.value,
        "routing_mode": routing_mode.value,
        "turn_angles_deg": list(config.turn_angles_deg),
        "search_turn_angles_deg": list(config.search_turn_angles_deg),
        "candidate_policy": {
            "lateral_offsets_cm": list(config.observation_lateral_offsets_cm),
            "standoff_distances_cm": list(config.observation_standoff_distances_cm),
            "activation_tiers": [list(tier) for tier in config.candidate_activation_tiers],
            "candidate_limit": config.guaranteed_max_candidates_per_target,
        },
        "budgets": {
            "max_expanded_nodes": config.max_expanded_nodes,
            "adaptive_initial_expansions": config.adaptive_initial_expansions,
            "adaptive_max_expansions": config.adaptive_max_expansions,
            "local_timeout_s": config.local_planning_timeout_s,
            "overall_timeout_s": config.overall_planning_timeout_s,
        },
        "search_geometry": {
            "position_bin_cm": config.position_bin_cm,
            "heading_bin_rad": config.heading_bin_rad,
            "goal_position_tolerance_cm": config.goal_position_tolerance_cm,
            "goal_heading_tolerance_rad": config.goal_heading_tolerance_rad,
            "collision_translation_step_cm": config.collision_translation_step_cm,
            "collision_arc_step_rad": config.collision_arc_step_rad,
        },
        "motion": {
            "straight_speed_cm_s": config.motion.straight_speed_cm_s,
            "straight_fixed_time_s": config.motion.straight_fixed_time_s,
            "straight_deceleration_cm": config.motion.straight_deceleration_cm,
            "straight_settle_s": config.motion.straight_settle_s,
            "serial_overhead_s": config.motion.serial_overhead_s,
            "capture_delay_s": config.motion.capture_delay_s,
            "turn_radii_cm": {
                primitive.command: primitive.radius_cm
                for primitive in config.motion.primitives if primitive.radius_cm is not None
            },
            "primitives": [
                {
                    "command": primitive.command,
                    "travel_cm": primitive.travel_cm,
                    "turn_angle_rad": primitive.turn_angle_rad,
                    "radius_cm": primitive.radius_cm,
                    "estimated_duration_s": primitive.estimated_duration_s,
                }
                for primitive in config.motion.primitives
            ],
            "penalties_s": {
                "turn": config.motion.turn_penalty_s,
                "steering_change": config.motion.steering_change_penalty_s,
                "direction_change": config.motion.direction_change_penalty_s,
            },
        },
    }


def config_fingerprint(config: PlanningConfig, *, objective: CostMetric = CostMetric.ESTIMATED_TIME,
                       routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION) -> str:
    canonical = json.dumps(config_record(config, objective=objective, routing_mode=routing_mode),
                           sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _repo_metadata() -> dict[str, Any]:
    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True,
                             check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], capture_output=True,
                                    text=True, check=True).stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        return {"commit_sha": None, "working_tree_dirty": None}
    return {"commit_sha": sha, "working_tree_dirty": dirty}


def _segment_distance(a: Point, b: Point, c: Point, d: Point) -> float:
    def orient(p, q, r):
        return (q.x_cm-p.x_cm)*(r.y_cm-p.y_cm) - (q.y_cm-p.y_cm)*(r.x_cm-p.x_cm)
    def within(p, q, r):
        return min(p.x_cm, q.x_cm) <= r.x_cm <= max(p.x_cm, q.x_cm) and min(p.y_cm, q.y_cm) <= r.y_cm <= max(p.y_cm, q.y_cm)
    o1, o2, o3, o4 = orient(a,b,c), orient(a,b,d), orient(c,d,a), orient(c,d,b)
    if o1*o2 <= 0 and o3*o4 <= 0:
        return 0.0
    def point_dist(p, x, y):
        vx, vy = y.x_cm-x.x_cm, y.y_cm-x.y_cm
        denom = vx*vx + vy*vy
        t = 0.0 if denom == 0 else max(0.0, min(1.0, ((p.x_cm-x.x_cm)*vx+(p.y_cm-x.y_cm)*vy)/denom))
        return math.hypot(p.x_cm-(x.x_cm+t*vx), p.y_cm-(x.y_cm+t*vy))
    return min(point_dist(a,c,d), point_dist(b,c,d), point_dist(c,a,b), point_dist(d,a,b))


def pose_clearance(pose: Pose, arena: ArenaInput, config: PlanningConfig) -> tuple[float, float, int | None]:
    footprint = robot_footprint(pose, config.robot)
    boundary = min(min(p.x_cm, p.y_cm, config.arena_size_cm-p.x_cm,
                       config.arena_size_cm-p.y_cm) for p in footprint)
    closest = float("inf")
    closest_id = None
    for obstacle in arena.obstacles:
        x0, y0 = obstacle.cell.x*config.cell_size_cm, obstacle.cell.y*config.cell_size_cm
        box = (Point(x0,y0), Point(x0+config.cell_size_cm,y0),
               Point(x0+config.cell_size_cm,y0+config.cell_size_cm), Point(x0,y0+config.cell_size_cm))
        if polygons_intersect(footprint, box):
            return 0.0, boundary, obstacle.obstacle_id
        distance = min(_segment_distance(footprint[i], footprint[(i+1)%4], box[j], box[(j+1)%4])
                       for i in range(4) for j in range(4))
        if distance < closest:
            closest, closest_id = distance, obstacle.obstacle_id
    return closest, boundary, closest_id


def build_run_record(arena: ArenaInput, config: PlanningConfig, result,
                     *, calibration: dict[str, Any] | None = None,
                     actual_capture_poses: dict[int, Pose] | None = None,
                     routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION) -> dict[str, Any]:
    if result.route is None:
        raise ValueError("only successful plans can be saved as physical runs")
    route = result.route
    movements = []
    captures = []
    sequence = 0
    cumulative_cm = 0.0
    turn_count = reverse_count = reverse_turn_count = 0
    selected = dict(zip(route.target_order, route.selected_candidate_kinds))
    selected_poses = {candidate.obstacle_id: candidate.pose for candidate in route.observation_poses}
    candidate_by_target = {(item.observation_pose.obstacle_id, item.display_label): item
                           for item in route_candidate_groups(arena, config)}
    effective_candidate_policy = config_record(config, objective=route.objective,
                                              routing_mode=routing_mode)["candidate_policy"]
    segment_target = {
        (segment.primitive.command, segment.start, segment.end): target_id
        for target_id, path in zip(route.target_order, route.local_paths)
        for segment in path.segments
    }
    timeline: list[dict[str, Any]] = []
    for step in route.execution_steps:
        if isinstance(step, MoveStep):
            sequence += 1
            primitive = step.segment.primitive
            samples = sample_motion(step.segment.start, primitive, config)
            clearances = [pose_clearance(pose, arena, config) for pose in samples]
            obstacle_clear, obstacle_id = min((entry[0], entry[2]) for entry in clearances)
            arena_clear = min(entry[1] for entry in clearances)
            cumulative_cm += primitive.geometric_length_cm
            is_turn = primitive.steering is not Steering.STRAIGHT
            is_reverse = primitive.gear is Gear.REVERSE
            turn_count += int(is_turn)
            reverse_count += int(is_reverse)
            reverse_turn_count += int(is_turn and is_reverse)
            movements.append({
                "sequence": sequence,
                "command": serialize_stm_command(step),
                "start_pose": pose_record(step.segment.start), "end_pose": pose_record(step.segment.end),
                "heading_before_rad": step.segment.start.heading_rad,
                "heading_after_rad": step.segment.end.heading_rad,
                "primitive": {"command": primitive.command, "travel_cm": primitive.travel_cm,
                              "turn_angle_rad": primitive.turn_angle_rad, "radius_cm": primitive.radius_cm,
                              "estimated_duration_s": primitive.estimated_duration_s},
                "samples": [pose_record(pose) for pose in samples],
                "target_obstacle": segment_target.get((primitive.command, step.segment.start, step.segment.end)),
                "candidate": selected.get(segment_target.get((primitive.command, step.segment.start, step.segment.end))),
                "gear": primitive.gear.value, "motion": "turn" if is_turn else "straight",
                "minimum_obstacle_clearance_cm": obstacle_clear,
                "closest_obstacle_id": obstacle_id,
                "minimum_arena_clearance_cm": arena_clear,
                "cumulative_distance_cm": cumulative_cm,
                "cumulative_turns": turn_count, "cumulative_reverse_commands": reverse_count,
                "cumulative_reverse_turns": reverse_turn_count,
            })
            timeline.append({"type": "move", "sequence": sequence})
        elif isinstance(step, CaptureStep):
            pose = selected_poses[step.obstacle_id]
            obstacle = next(item for item in arena.obstacles if item.obstacle_id == step.obstacle_id)
            candidate = candidate_by_target.get((step.obstacle_id, selected.get(step.obstacle_id)))
            camera = camera_world_position(pose, config.camera)
            face_center = image_face_target_point(obstacle, obstacle.face, config.cell_size_cm)
            normal_x, normal_y = obstacle.face.grid_vector
            lateral_error = ((camera.x_cm-face_center.x_cm) * -normal_y
                             + (camera.y_cm-face_center.y_cm) * normal_x)
            captures.append({"target_obstacle": step.obstacle_id, "image_face": obstacle.face.value,
                             "rear_axle_pose": pose_record(pose),
                             "planned_camera_pose": {"x_cm": camera.x_cm, "y_cm": camera.y_cm},
                             "camera_to_image_face_distance_cm": math.hypot(camera.x_cm-face_center.x_cm,
                                                                            camera.y_cm-face_center.y_cm),
                             "lateral_alignment_error_cm": lateral_error,
                             "candidate_policy": effective_candidate_policy,
                             "selected_candidate": selected.get(step.obstacle_id),
                             "standoff_cm": getattr(candidate, "standoff_cm", None),
                             "motion_before_capture": {
                                 "commands_executed": sequence, "turns": turn_count,
                                 "reverse_commands": reverse_count, "reverse_turns": reverse_turn_count,
                                 "geometric_distance_cm": cumulative_cm,
                             },
                             "actual_pose": pose_record(actual_capture_poses[step.obstacle_id])
                             if actual_capture_poses and step.obstacle_id in actual_capture_poses else None})
            if actual_capture_poses and step.obstacle_id in actual_capture_poses:
                captures[-1]["physical_vs_planned_error"] = compare_capture_poses(
                    pose, actual_capture_poses[step.obstacle_id])
            timeline.append({"type": "capture", "obstacle_id": step.obstacle_id})
    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "original_arena_payload": arena_record(arena),
        "repository": _repo_metadata(),
        "planner_config_fingerprint": config_fingerprint(config, objective=route.objective, routing_mode=routing_mode),
        "effective_planner_settings": config_record(config, objective=route.objective, routing_mode=routing_mode),
        "stm_calibration": calibration or {"source": "effective config radii", "turn_radii_cm": config_record(config)["motion"]["turn_radii_cm"]},
        "robot_geometry": asdict(config.robot), "camera_geometry": asdict(config.camera),
        "safety_margin_cm": config.robot.safety_margin_cm, "objective": route.objective.value,
        "turn_vocabulary_deg": list(config.search_turn_angles_deg),
        "candidate_policy": config_record(config)["candidate_policy"],
        "target_order": list(route.target_order),
        "selected_candidates": {str(key): value for key, value in selected.items()},
        "planned_route": {"sampled_poses": [pose_record(pose) for pose in route.sampled_poses],
                          "movements": movements, "captures": captures, "timeline": timeline,
                          "serialized_commands": serialize_planning_result(result)["steps"],
                          "geometric_distance_cm": route.metrics.geometric_distance_cm,
                          "estimated_execution_time_s": route.metrics.estimated_time_s},
        "planned_capture_poses": {str(key): pose_record(value) for key, value in selected_poses.items()},
        "actual_capture_poses": {str(key): pose_record(value) for key, value in (actual_capture_poses or {}).items()},
    }


def build_scenario_record(arena: ArenaInput, config: PlanningConfig, *,
                          calibration: dict[str, Any] | None = None,
                          routing_mode: RoutingMode = RoutingMode.FULL_OPTIMIZATION) -> dict[str, Any]:
    """Save an editable arena and effective settings before planning."""
    settings = config_record(config, routing_mode=routing_mode)
    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "original_arena_payload": arena_record(arena),
        "repository": _repo_metadata(),
        "planner_config_fingerprint": config_fingerprint(config, routing_mode=routing_mode),
        "effective_planner_settings": settings,
        "stm_calibration": calibration or {
            "source": "effective config radii", "turn_radii_cm": settings["motion"]["turn_radii_cm"]
        },
        "robot_geometry": asdict(config.robot), "camera_geometry": asdict(config.camera),
        "safety_margin_cm": config.robot.safety_margin_cm,
        "objective": CostMetric.ESTIMATED_TIME.value,
        "turn_vocabulary_deg": list(config.search_turn_angles_deg),
        "candidate_policy": settings["candidate_policy"],
        "target_order": [], "selected_candidates": {},
        "planned_route": {"sampled_poses": [], "movements": [], "captures": [],
                          "timeline": [], "serialized_commands": []},
        "planned_capture_poses": {}, "actual_capture_poses": {},
    }


def attach_measured_capture_pose(record: dict[str, Any], target_id: int, pose: Pose) -> None:
    """Attach a physical rear-axle observation to a saved route record."""
    planned_data = record.get("planned_capture_poses", {}).get(str(target_id))
    if planned_data is None:
        raise ValueError(f"target {target_id} has no recorded planned capture pose")
    planned = pose_from_record(planned_data)
    record.setdefault("actual_capture_poses", {})[str(target_id)] = pose_record(pose)
    for capture in record.get("planned_route", {}).get("captures", ()):
        if int(capture["target_obstacle"]) == target_id:
            capture["actual_pose"] = pose_record(pose)
            capture["physical_vs_planned_error"] = compare_capture_poses(planned, pose)
            break


def route_candidate_groups(arena, config):
    from algorithm.targets import generate_arena_observation_candidates
    groups = generate_arena_observation_candidates(arena, config)
    return tuple(candidate for group in groups for candidate in group.candidates)


def save_run(path: Path, record: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_run(path: Path) -> dict[str, Any]:
    record = json.loads(path.read_text(encoding="utf-8"))
    if record.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported Task 1 scenario schema: {record.get('schema_version')}")
    if "original_arena_payload" not in record and "arena" in record:
        record["original_arena_payload"] = record["arena"]
    if "original_arena_payload" not in record:
        raise ValueError("scenario is missing its original arena payload")
    return record


def historical_simulator(record: dict[str, Any], current_config: PlanningConfig) -> HeadlessSimulator:
    """Construct playback only from recorded samples; this never invokes a planner."""
    arena = arena_from_record(record["original_arena_payload"])
    route = record["planned_route"]
    steps: list[SimulationStep] = []
    movement_by_sequence = {item["sequence"]: item for item in route.get("movements", ())}
    for entry in route.get("timeline", ()):
        if entry["type"] == "move":
            movement = movement_by_sequence[entry["sequence"]]
            samples = [pose_from_record(item) for item in movement.get("samples", ())]
            for index, pose in enumerate(samples[1:], start=1):
                steps.append(SimulationStep.motion(
                    pose, 0.08, movement["command"], ends_primitive=index == len(samples)-1,
                ))
        elif entry["type"] == "capture":
            steps.append(SimulationStep.capture(int(entry["obstacle_id"])))
    poses = tuple(pose_from_record(item) for item in route["sampled_poses"])
    from algorithm.targets import generate_arena_observation_candidates
    from .headless import HeadlessSimulator
    return HeadlessSimulator(arena, generate_arena_observation_candidates(arena, current_config),
                             tuple(steps), planned_path=poses,
                             target_order=tuple(record.get("target_order", ())),
                             selected_candidates=tuple((int(k), v) for k,v in record.get("selected_candidates", {}).items()))


__all__ = ["SCHEMA_VERSION", "arena_from_record", "arena_record", "attach_measured_capture_pose", "build_run_record", "build_scenario_record", "compare_capture_poses", "config_fingerprint",
           "config_record", "historical_simulator", "load_run", "save_run"]
