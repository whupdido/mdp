"""Task 1 route-planning server.

Wraps algorithm.routing.planner.Task1Planner behind the same length-prefixed
JSON socket protocol server/utils.py already uses for yolo_task1.py, so this
runs alongside it with the same start/stop pattern -- no new dependency
(Flask, requests, ...) on either end.

Run on the PC (same machine as server/yolo_task1.py):
    python server/algo_server.py

The RPi side is rpi/algo_client.py's plan_route().
"""

import socket

from algorithm.config import task1_robot_config
from algorithm.enums import Direction
from algorithm.models.arena import ArenaInput
from algorithm.models.obstacle import Obstacle
from algorithm.models.pose import GridCell, Pose
from algorithm.routing.planner import Task1Planner
from algorithm.serialization import serialize_planning_result, serialize_stm_command

from server.utils import recv_json, send_json

HOST = "0.0.0.0"
PORT = 5002

# Android/PROTOCOL.md: "the robot starts at (1,1) facing N".
DEFAULT_START = {"x": 1, "y": 1, "face": "N"}


def _build_arena(payload: dict) -> ArenaInput:
    start_raw = payload.get("start") or DEFAULT_START
    start_cell = GridCell(int(start_raw["x"]), int(start_raw["y"]))
    start_pose = Pose.from_direction(
        *start_cell.center_cm(), Direction.from_token(start_raw["face"])
    )

    obstacles = tuple(
        Obstacle(
            obstacle_id=int(o["id"]),
            cell=GridCell(int(o["x"]), int(o["y"])),
            face=Direction.from_token(o["face"]) if o.get("face") else None,
        )
        for o in payload["obstacles"]
    )
    return ArenaInput(start_pose=start_pose, obstacles=obstacles)


_stm_command = serialize_stm_command
_serialize_result = serialize_planning_result


def _plan_payload(payload: dict):
    arena = _build_arena(payload)
    return Task1Planner(task1_robot_config()).plan(arena)


def handle_client(conn: socket.socket) -> None:
    try:
        payload = recv_json(conn)
        if payload is None:
            return
        print(f"[ALGO] Received {len(payload.get('obstacles', []))} obstacle(s)")

        try:
            result = _plan_payload(payload)
        except (KeyError, TypeError, ValueError) as exc:
            send_json(conn, {"status": "invalid_input", "issues": [
                {"code": "malformed_request", "message": str(exc), "obstacle_id": None}
            ]})
            return

        response = _serialize_result(result)
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
