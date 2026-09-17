import pytest

from algorithm.enums import PlanningStatus, Steering
from algorithm.simulator.task2_demo import build_fastest_car_demo
from algorithm.simulator.task2_editor_model import FastestCarSetup,random_fastest_car_setup
import random

@pytest.mark.parametrize(
    "obstacle_1_direction, obstacle_2_direction",
    [
        (Steering.LEFT, Steering.LEFT),
        (Steering.LEFT, Steering.RIGHT),
        (Steering.RIGHT, Steering.LEFT),
        (Steering.RIGHT, Steering.RIGHT),
    ],
)
def test_task2_arrow_combinations(
    obstacle_1_direction: Steering,
    obstacle_2_direction: Steering,
) -> None:
    setup = FastestCarSetup(
        obstacle_1_direction=obstacle_1_direction,
        obstacle_2_direction=obstacle_2_direction,
    )

    scenario = build_fastest_car_demo(setup)

    assert scenario.planning_result.status is PlanningStatus.SUCCESS

@pytest.mark.parametrize(
    "obstacle_1_direction, obstacle_2_direction",
    [
        (Steering.LEFT, Steering.LEFT),
        (Steering.LEFT, Steering.RIGHT),
        (Steering.RIGHT, Steering.LEFT),
        (Steering.RIGHT, Steering.RIGHT),
    ],
)
@pytest.mark.parametrize(
    "top_clearance_cm,bottom_clearance_cm,carpark_to_obstacle_1_cm,obstacle_1_to_obstacle_2_cm",
    [
        (50, 50, 50, 50),       # minimum geometry
        (120, 120, 130, 130),   # maximum geometry
    ],
)
def test_task2_geometry_boundaries(
    obstacle_1_direction: Steering,
    obstacle_2_direction: Steering,
    top_clearance_cm: int,
    bottom_clearance_cm: int,
    carpark_to_obstacle_1_cm: int,
    obstacle_1_to_obstacle_2_cm: int,
) -> None:
    setup = FastestCarSetup(
        top_clearance_cm=top_clearance_cm,
        bottom_clearance_cm=bottom_clearance_cm,
        carpark_to_obstacle_1_cm=carpark_to_obstacle_1_cm,
        obstacle_1_to_obstacle_2_cm=obstacle_1_to_obstacle_2_cm,
        obstacle_1_direction=obstacle_1_direction,
        obstacle_2_direction=obstacle_2_direction,
    )

    scenario = build_fastest_car_demo(setup)

    assert scenario.planning_result.status is PlanningStatus.SUCCESS

def test_task2_random_setups() -> None:
    rng = random.Random(73)

    for i in range(100):
        setup = random_fastest_car_setup(rng)

        try:
            scenario = build_fastest_car_demo(setup)
        except RuntimeError as exc:
            pytest.fail(
                f"Random case {i} failed.\n"
                f"Setup: {setup}\n"
                f"Reason: {exc}"
            )

        assert scenario.planning_result.status is PlanningStatus.SUCCESS