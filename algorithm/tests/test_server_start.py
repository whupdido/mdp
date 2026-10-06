"""Start placement: where the planner starts the robot, and the lead-in move."""

import socket

import pytest

from algorithm.config import task1_robot_config
from algorithm.enums import Direction
from algorithm.geometry import is_pose_collision_free
from algorithm.models.arena import ArenaInput
from algorithm.models.pose import GridCell
from server import algo_server
from server.utils import recv_json, send_json


def _config():
    return task1_robot_config(emit=lambda _message: None)


@pytest.mark.parametrize(
    "face, along, across",
    [(Direction.NORTH, "y_cm", "x_cm"), (Direction.EAST, "x_cm", "y_cm")],
)
def test_back_on_edge_start_opens_with_a_lead_in_to_the_planner_start(monkeypatch, face, along, across):
    monkeypatch.setattr(algo_server, "START_WITH_BACK_ON_EDGE", True)
    config = _config()
    start, lead_in = algo_server._start_pose(GridCell(1, 1), face, config)

    assert lead_in is not None and lead_in.startswith("FW")
    lead_cm = int(lead_in[2:])
    back_to_axle = config.robot.length_cm / 2.0 - config.robot.rear_axle_to_body_center_forward_cm
    # The planner starts exactly where the lead-in ends...
    assert getattr(start, along) == pytest.approx(back_to_axle + lead_cm)
    assert getattr(start, across) == pytest.approx(15.0)
    assert start.heading_rad == pytest.approx(face.heading_rad)
    # ...which puts the body centre over the start cell, at most 1 cm past it.
    centre = getattr(start, along) + config.robot.rear_axle_to_body_center_forward_cm
    assert 15.0 <= centre < 16.0
    assert is_pose_collision_free(start, ArenaInput(start, ()), config)


def test_hand_placed_start_has_no_lead_in(monkeypatch):
    monkeypatch.setattr(algo_server, "START_WITH_BACK_ON_EDGE", False)
    config = _config()
    start, lead_in = algo_server._start_pose(GridCell(1, 1), Direction.NORTH, config)

    assert lead_in is None
    assert start.y_cm + config.robot.rear_axle_to_body_center_forward_cm == pytest.approx(15.0)


def test_lead_in_is_the_first_step_sent_to_the_pi(monkeypatch):
    monkeypatch.setattr(algo_server, "_plan_payload", lambda payload: (object(), "FW004"))
    monkeypatch.setattr(
        algo_server,
        "_serialize_result",
        lambda result: {
            "status": "success",
            "steps": [{"type": "move", "command": "FR030"}, {"type": "capture", "obstacle_id": 1}],
        },
    )
    ours, servers = socket.socketpair()
    try:
        send_json(ours, {"obstacles": []})
        algo_server.handle_client(servers)
        response = recv_json(ours)
    finally:
        ours.close()
        servers.close()

    assert [step.get("command") for step in response["steps"][:2]] == ["FW004", "FR030"]


def test_failed_plans_get_no_lead_in(monkeypatch):
    monkeypatch.setattr(algo_server, "_plan_payload", lambda payload: (object(), "FW004"))
    monkeypatch.setattr(
        algo_server,
        "_serialize_result",
        lambda result: {"status": "no_feasible_route", "issues": []},
    )
    ours, servers = socket.socketpair()
    try:
        send_json(ours, {"obstacles": []})
        algo_server.handle_client(servers)
        response = recv_json(ours)
    finally:
        ours.close()
        servers.close()

    assert response == {"status": "no_feasible_route", "issues": []}
