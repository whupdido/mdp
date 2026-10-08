"""Robot requests must use the same candidate fallback policy as the editor."""

from algorithm.enums import PlanningStatus
from algorithm.geometry import is_motion_collision_free
from algorithm.models.motion import CaptureStep, MoveStep
from server.algo_server import _build_arena, _plan_payload, _serialize_result


def test_robot_server_routes_eight_target_layout_requiring_camera_fallbacks():
    payload = {
        "start": {"x": 1, "y": 1, "face": "N"},
        "obstacles": [
            {"id": obstacle_id, "x": x, "y": y, "face": face}
            for obstacle_id, x, y, face in (
                (1, 0, 13, "S"),
                (2, 8, 13, "S"),
                (3, 19, 19, "W"),
                (4, 11, 13, "S"),
                (5, 14, 9, "S"),
                (6, 19, 9, "W"),
                (7, 19, 2, "W"),
                (8, 8, 9, "E"),
            )
        ],
    }

    result, lead_in = _plan_payload(payload)

    assert result.status is PlanningStatus.SUCCESS, result.issues
    assert lead_in == "FW004"
    route = result.route
    assert route is not None
    assert set(route.target_order) == set(range(1, 9))
    assert len(route.target_order) == 8
    assert result.metrics.candidate_tiers_activated > 1
    assert any(not pose.nominal for pose in route.observation_poses)

    # Verify the robot-bound sequence preserves continuous, collision-free moves.
    from algorithm.config import task1_robot_config

    arena = _build_arena(payload, route.start)
    config = task1_robot_config(emit=lambda _message: None)
    current = route.start
    captures = []
    for step in route.execution_steps:
        if isinstance(step, MoveStep):
            assert step.segment.start == current
            assert is_motion_collision_free(current, step.segment.primitive, arena, config)
            current = step.segment.end
        elif isinstance(step, CaptureStep):
            assert step.pose == current
            captures.append(step.obstacle_id)

    assert tuple(captures) == route.target_order
    response = _serialize_result(result)
    assert response["status"] == "success"
    assert [step["obstacle_id"] for step in response["steps"] if step["type"] == "capture"] == captures
    assert all(len(step["command"]) == 5 for step in response["steps"] if step["type"] == "move")
