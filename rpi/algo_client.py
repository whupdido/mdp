"""Client for server/algo_server.py -- gets a Task 1 route (a list of STM
move commands and image-capture steps) for the obstacles a1_bridge.py has
recorded from Android.

Runs ON THE RPI. Uses its own length-prefixed JSON framing rather than
importing server.utils, same reasoning as capture_and_report.py: this needs
to keep working standalone on the Pi's Python even if it ever diverges from
the server side's.
"""

from __future__ import annotations

import json
import os
import socket
import struct

# Laptop's IP on the shared WiFi, running `python server/algo_server.py`.
# Find it with `ipconfig getifaddr en0` on the Mac. Same value as
# capture_and_report.DETECTION_SERVER_IP whenever both servers run on the same
# laptop, and the same MDP_LAPTOP_IP override.
ALGO_SERVER_IP = os.environ.get("MDP_LAPTOP_IP", "192.168.4.40")
ALGO_SERVER_PORT = 5002


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Algo server closed the connection")
        buf += chunk
    return buf


def _send_json(sock: socket.socket, obj: dict) -> None:
    blob = json.dumps(obj).encode("utf-8")
    sock.sendall(struct.pack(">I", len(blob)))
    sock.sendall(blob)


def _recv_json(sock: socket.socket) -> dict:
    (length,) = struct.unpack(">I", _recv_exact(sock, 4))
    return json.loads(_recv_exact(sock, length).decode("utf-8"))


def server_reachable(timeout: float = 2.0) -> bool:
    """Can we open a socket to the laptop's algo server right now?

    Zhenxi: added for run_task1.py's pre-flight, on the PLAN press.

    The point is to find out during the preparation window -- where time is
    free -- that nobody started `python -m server.algo_server` on the laptop,
    rather than discovering it when the clock is already running and the run
    is already lost. Opens and closes a connection and sends nothing, so it
    cannot disturb a server that is up.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((ALGO_SERVER_IP, ALGO_SERVER_PORT))
        return True
    except OSError as exc:
        print(f"[ALGO] {ALGO_SERVER_IP}:{ALGO_SERVER_PORT} unreachable: {exc}")
        return False
    finally:
        sock.close()


def plan_route(obstacles: list[dict], start: dict | None = None) -> list[dict] | None:
    """obstacles: [{"id": int, "x": int, "y": int, "face": "N"/"E"/"S"/"W"}, ...]
    start: optional {"x": int, "y": int, "face": "N"/"E"/"S"/"W"}, defaults to
    (1, 1, N) server-side, per Android/PROTOCOL.md's start zone.

    Returns the ordered step list on success --
    [{"type": "move", "command": "FW030"}, {"type": "capture", "obstacle_id": 2}, ...]
    -- or None (after printing why) if planning failed."""
    payload = {"obstacles": obstacles}
    if start is not None:
        payload["start"] = start

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ALGO_SERVER_IP, ALGO_SERVER_PORT))
    try:
        _send_json(sock, payload)
        response = _recv_json(sock)
    finally:
        sock.close()

    if response["status"] != "success":
        print(f"[ALGO CLIENT] Planning failed: {response['status']} -- {response.get('issues')}")
        return None

    return response["steps"]
