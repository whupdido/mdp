import math
import socket

import pytest

from algorithm.enums import Gear, PlanningStatus, Steering
from algorithm.models import (
    CaptureStep,
    MotionPrimitive,
    MotionSegment,
    ObservationPose,
    PathMetrics,
    PlanningIssue,
    PlanningResult,
    Pose,
    RoutePlan,
    MoveStep,
)
from server.algo_server import _serialize_result, _stm_command
from server.utils import recv_json, send_json


START = Pose(100.0, 100.0, 0.0)


def motion_step(command: str, *, angle_deg: float = 0.0, travel_cm: float = 0.0) -> MoveStep:
    if command in {"FW", "BW"}:
        primitive = MotionPrimitive(
            command,
            Gear.FORWARD if command == "FW" else Gear.REVERSE,
            Steering.STRAIGHT,
            travel_cm=travel_cm,
        )
    else:
        gear = Gear.FORWARD if command[0] == "F" else Gear.REVERSE
        steering = Steering.LEFT if command[1] == "L" else Steering.RIGHT
        sign = 1.0 if command in {"FL", "BR"} else -1.0
        primitive = MotionPrimitive(
            command,
            gear,
            steering,
            turn_angle_rad=sign * math.radians(angle_deg),
            radius_cm=30.0,
        )
    segment = MotionSegment(primitive, START, START)
    return MoveStep(segment, command)


@pytest.mark.parametrize(
    ("command", "angle_deg", "expected"),
    (
        ("FL", 30.0, "FL030"),
        ("FR", 90.0, "FR090"),
        ("BL", 45.0, "BL045"),
        ("BR", 60.0, "BR060"),
    ),
)
def test_turn_commands_use_absolute_degrees_and_preserve_direction(command, angle_deg, expected):
    assert _stm_command(motion_step(command, angle_deg=angle_deg)) == expected


@pytest.mark.parametrize(
    ("command", "travel_cm", "expected"),
    (("FW", 10.0, "FW010"), ("BW", 20.0, "BW020")),
)
def test_straight_commands_use_travel_distance(command, travel_cm, expected):
    assert _stm_command(motion_step(command, travel_cm=travel_cm)) == expected


def successful_result():
    capture_pose = Pose(120.0, 100.0, 0.0)
    observation = ObservationPose(7, 0, capture_pose, nominal=True)
    steps = (
        motion_step("FL", angle_deg=30.0),
        CaptureStep(7, capture_pose),
        motion_step("FW", travel_cm=10.0),
    )
    route = RoutePlan(
        start=START,
        target_order=(7,),
        observation_poses=(observation,),
        local_paths=(),
        execution_steps=steps,
        metrics=PathMetrics(),
    )
    return PlanningResult(PlanningStatus.SUCCESS, route=route)


def test_successful_result_serializes_ordered_move_and_capture_steps():
    assert _serialize_result(successful_result()) == {
        "status": "success",
        "steps": [
            {"type": "move", "command": "FL030"},
            {"type": "capture", "obstacle_id": 7},
            {"type": "move", "command": "FW010"},
        ],
    }


def test_non_success_result_preserves_status_and_issues():
    issue = PlanningIssue("planning_timeout", "planning exceeded the time bound", obstacle_id=7)
    result = PlanningResult(PlanningStatus.PLANNING_TIMEOUT, issues=(issue,))

    assert _serialize_result(result) == {
        "status": "planning_timeout",
        "issues": [
            {
                "code": "planning_timeout",
                "message": "planning exceeded the time bound",
                "obstacle_id": 7,
            }
        ],
    }


def test_existing_length_prefixed_json_protocol_round_trips_serialized_result():
    payload = _serialize_result(successful_result())
    sender, receiver = socket.socketpair()
    try:
        send_json(sender, payload)
        assert recv_json(receiver) == payload
    finally:
        sender.close()
        receiver.close()
