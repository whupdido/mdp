import math

from algorithm.config import FASTEST_CAR_CONFIG, fastest_car_config
from algorithm.enums import Direction
from algorithm.geometry import is_pose_collision_free
from algorithm.models import ArenaInput, Pose, RectangleObstacle
from algorithm.simulator.task2_demo import build_fastest_car_demo, fastest_car_demo_arena


def test_fastest_car_config():
    assert FASTEST_CAR_CONFIG.grid_display is True
    assert FASTEST_CAR_CONFIG.arena_size_cm == 500.0
    assert FASTEST_CAR_CONFIG.arena_height_cm == 350.0
    assert FASTEST_CAR_CONFIG.robot.length_cm == 30.0
    assert FASTEST_CAR_CONFIG.robot.width_cm == 30.0


def test_rectangle_obstacle_collision_uses_continuous_coordinates():
    config = fastest_car_config(arena_width_cm=500.0, arena_height_cm=400.0)
    arena = ArenaInput(
        Pose(100.0, 200.0, 0.0),
        (RectangleObstacle(1, 200.0, 180.0, 260.0, 220.0, Direction.WEST),),
        500.0,
        400.0,
    )
    assert is_pose_collision_free(Pose(100.0, 100.0, 0.0), arena, config)
    assert not is_pose_collision_free(Pose(225.0, 200.0, 0.0), arena, config)


def test_fastest_car_demo_plans():
    scenario = build_fastest_car_demo()
    assert scenario.planning_result.route is not None
    assert scenario.planning_result.route.target_order == (1,2)
    assert scenario.config.grid_display is True
    assert all(isinstance(obstacle, RectangleObstacle) for obstacle in fastest_car_demo_arena().obstacles)
