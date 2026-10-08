"""Task 1 route-planning server.

Wraps algorithm.routing.planner.Task1Planner behind the same length-prefixed
JSON socket protocol server/utils.py already uses for yolo_task1.py, so this
runs alongside it with the same start/stop pattern -- no new dependency
(Flask, requests, ...) on either end.

Run on the PC (same machine as server/yolo_task1.py):
    python server/algo_server.py

The RPi side is rpi/algo_client.py's plan_route().
"""

import math
import socket

from algorithm.config import task1_robot_config
from algorithm.coordinates import android_cell_to_planner_pose
from algorithm.enums import Direction, PlanningStatus, Steering
from algorithm.models.arena import ArenaInput
from algorithm.models.motion import CaptureStep, MoveStep
from algorithm.models.obstacle import Obstacle
from algorithm.models.pose import GridCell, Pose
from algorithm.routing.planner import Task1Planner

from server.utils import recv_json, send_json

HOST = "0.0.0.0"
PORT = 5002

# Android/PROTOCOL.md: "the robot starts at (1,1) facing N".
DEFAULT_START = {"x": 1, "y": 1, "face": "N"}

# How the robot is put down at the start.
#
# True: facing the way the tablet says, with its back touching the arena edge
# behind it, centred on the start cell's column (facing N or S) or row (facing
# E or W). For the default start, (1,1) facing N, that is the back on the
# bottom edge and the centre line 15 cm from the left edge. The route then
# opens with a short forward move (FW004 for that start) that takes the robot
# to where the planner starts it: the body centre over the start cell.
#
# False: put the body centre over the start cell by hand, as before, and the
# route starts straight away.
START_WITH_BACK_ON_EDGE = True


def _start_cell(payload: dict):
    start_raw = payload.get("start") or DEFAULT_START
    return (
        GridCell(int(start_raw["x"]), int(start_raw["y"])),
        Direction.from_token(start_raw["face"]),
    )


def _start_pose(start_cell, start_face, config):
    """Return (planner start pose, lead-in move command or None).

    The tablet's start cell is where the body centre should be; the planner
    works from the rear axle, which sits behind the centre on the real car.
    """
    geometry = config.robot
    target = android_cell_to_planner_pose(start_cell, start_face, geometry)
    if not START_WITH_BACK_ON_EDGE:
        return target, None

    # Rear axle when the back touches the edge behind the robot.
    back_to_axle = geometry.length_cm / 2.0 - geometry.rear_axle_to_body_center_forward_cm
    far_side = config.arena_size_cm - back_to_axle
    edge_x, edge_y = {
        Direction.NORTH: (target.x_cm, back_to_axle),
        Direction.SOUTH: (target.x_cm, far_side),
        Direction.EAST: (back_to_axle, target.y_cm),
        Direction.WEST: (far_side, target.y_cm),
    }[start_face]

    # Forward distance from there to the target, in whole centimetres since
    # that is what the STM takes. Rounding up keeps the back clear of the
    # edge by at least the planner's safety margin. The planner then starts
    # from exactly where that move ends, so the rounding costs nothing.
    step_x, step_y = start_face.grid_vector
    gap_cm = (target.x_cm - edge_x) * step_x + (target.y_cm - edge_y) * step_y
    lead_cm = max(0, math.ceil(gap_cm - 1e-6))
    start = Pose.from_direction(edge_x + lead_cm * step_x, edge_y + lead_cm * step_y, start_face)
    return start, (f"FW{lead_cm:03d}" if lead_cm else None)


def _build_arena(payload: dict, start_pose=None) -> ArenaInput:
    if start_pose is None:
        start_cell, start_face = _start_cell(payload)
        start_pose = Pose.from_direction(*start_cell.center_cm(), start_face)

    obstacles = tuple(
        Obstacle(
            obstacle_id=int(o["id"]),
            cell=GridCell(int(o["x"]), int(o["y"])),
            face=Direction.from_token(o["face"]) if o.get("face") else None,
        )
        for o in payload["obstacles"]
    )
    return ArenaInput(start_pose=start_pose, obstacles=obstacles)


def _stm_command(move: MoveStep) -> str:
    """Turn one MoveStep into the exact 5-char string a1_bridge.py/STM expect
    (Android/PROTOCOL.md "Motion commands": two-letter verb + 3 digits)."""
    primitive = move.segment.primitive
    if primitive.steering is Steering.STRAIGHT:
        magnitude = round(primitive.travel_cm)
    else:
        magnitude = round(abs(math.degrees(primitive.turn_angle_rad)))
    return f"{primitive.command}{magnitude:03d}"


def _serialize_result(result) -> dict:
    if result.status is not PlanningStatus.SUCCESS:
        return {
            "status": result.status.value,
            "issues": [
                {"code": issue.code, "message": issue.message, "obstacle_id": issue.obstacle_id}
                for issue in result.issues
            ],
        }

    steps = []
    for step in result.route.execution_steps:
        if isinstance(step, MoveStep):
            steps.append({"type": "move", "command": _stm_command(step)})
        elif isinstance(step, CaptureStep):
            steps.append({"type": "capture", "obstacle_id": step.obstacle_id})
    return {"status": "success", "steps": steps}


def _plan_payload(payload: dict):
    """Plan one request. Returns (planning result, lead-in command or None)."""
    config = task1_robot_config()
    start_pose, lead_in = _start_pose(*_start_cell(payload), config)
    arena = _build_arena(payload, start_pose)
    return Task1Planner(config).plan(arena), lead_in


def handle_client(conn: socket.socket) -> None:
    try:
        payload = recv_json(conn)
        if payload is None:
            return
        print(f"[ALGO] Received {len(payload.get('obstacles', []))} obstacle(s)")

        try:
            result, lead_in = _plan_payload(payload)
        except (KeyError, TypeError, ValueError) as exc:
            send_json(conn, {"status": "invalid_input", "issues": [
                {"code": "malformed_request", "message": str(exc), "obstacle_id": None}
            ]})
            return

        response = _serialize_result(result)
        if lead_in and response["status"] == "success":
            # Drive from the edge to the planner's start before anything else.
            response["steps"].insert(0, {"type": "move", "command": lead_in})
        print(f"[ALGO] Planning result: {response['status']}"
              f" ({len(response.get('steps', []))} steps)" if response["status"] == "success"
              else f"[ALGO] Planning result: {response['status']} -- {response['issues']}")
        send_json(conn, response)
    except Exception as exc:
        print(f"[ALGO] Error handling client: {exc}")


def serve() -> None:
    # The Android protocol places the robot centre at (1, 1), i.e. 15 cm
    # from each lower arena edge.  The general uncalibrated profile expands
    # the 23 cm body by 5 cm per side, which puts that documented start pose
    # outside the arena.  The bounded Task 1 profile uses the repository's
    # validated 3 cm integration margin and conservative search settings.
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    sock.listen(1)
    print(f"[ALGO] Listening on {(HOST, PORT)}")

    try:
        while True:
            conn, addr = sock.accept()
            print(f"[ALGO] Connected from {addr}")
            try:
                handle_client(conn)
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\n[ALGO] Server stopped by user")
    finally:
        sock.close()


if __name__ == "__main__":
    serve()
