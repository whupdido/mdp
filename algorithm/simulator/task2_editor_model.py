from __future__ import annotations

from dataclasses import dataclass, replace
import random
from random import Random

from algorithm.config import FASTEST_CAR_CONFIG, PlanningConfig
from algorithm.enums import Steering
from algorithm.models import ArenaInput

from .headless import HeadlessSimulator


@dataclass(frozen=True, slots=True)
class FastestCarSetup:
    """DEFAULT, Editable geometry and arrow directions for Fastest Car Task."""

    # Arena / route geometry
    arena_width_cm: float = 500.0
    arena_height_cm: float = 350.0
    top_clearance_cm: int = 100
    bottom_clearance_cm: int = 100
    carpark_to_obstacle_1_cm: int = 100
    obstacle_1_to_obstacle_2_cm: int = 100

    # Obstacle / carpark dimensions
    obstacle_width_cm: float = 40.0
    obstacle_height_cm: float = 60.0
    carpark_width_cm: float = 60.0

    # Robot starting position
    start_x_cm: float = 45.0

    # Arena wall geometry
    wall_width_cm: float = 20.0

    # O2 loop geometry
    loop_clearance_cm: float = 10.0
    loop_exit_lane_offset_cm: float = 30.0
    loop_exit_x_offset_cm: float = 20.0

    # Obstacle arrow directions
    obstacle_1_direction: Steering = Steering.LEFT
    obstacle_2_direction: Steering = Steering.RIGHT


FASTEST_CAR_LIMITS = {
    "top_clearance_cm": (50, 120),
    "bottom_clearance_cm": (50, 120),
    "carpark_to_obstacle_1_cm": (50, 130),
    "obstacle_1_to_obstacle_2_cm": (50, 130),
}

def random_fastest_car_setup(
    rng: random.Random | None = None,
) -> FastestCarSetup:
    """Generate a random Fastest Car setup."""

    rng = rng or random

def random_fastest_car_setup(
    rng: Random | None = None,
) -> FastestCarSetup:
    """Generate a random Fastest Car setup."""

    if rng is None:
        rng = Random()

    return FastestCarSetup(
        top_clearance_cm=rng.randint(
            *FASTEST_CAR_LIMITS["top_clearance_cm"]
        ),
        bottom_clearance_cm=rng.randint(
            *FASTEST_CAR_LIMITS["bottom_clearance_cm"]
        ),
        carpark_to_obstacle_1_cm=rng.randint(
            *FASTEST_CAR_LIMITS["carpark_to_obstacle_1_cm"]
        ),
        obstacle_1_to_obstacle_2_cm=rng.randint(
            *FASTEST_CAR_LIMITS["obstacle_1_to_obstacle_2_cm"]
        ),
        obstacle_1_direction=rng.choice(
            (Steering.LEFT, Steering.RIGHT)
        ),
        obstacle_2_direction=rng.choice(
            (Steering.LEFT, Steering.RIGHT)
        ),
    )

class Task2EditorController:
    """Controls the editable Fastest Car geometry and replanning."""

    def __init__(
        self,
        config: PlanningConfig = FASTEST_CAR_CONFIG,
    ) -> None:
        self.config = config
        self.setup = FastestCarSetup()

        self._simulator: HeadlessSimulator | None = None
        self.status_message = "Edit the geometry, then press ENTER to plan."

    def build_arena(self) -> ArenaInput:
        """Build an ArenaInput from the current setup."""

        # Local import avoids a circular import:
        # task2_demo -> task2_editor_model
        # task2_editor_model -> task2_demo
        from .task2_demo import fastest_car_demo_arena

        return fastest_car_demo_arena(
            self.setup,
            self.config,
        )

    def preview_simulator(self) -> HeadlessSimulator:
        """Return the current simulator, creating a geometry-only preview if needed."""

        if self._simulator is not None:
            return self._simulator

        arena = self.build_arena()

        self._simulator = HeadlessSimulator(
            arena,
            candidate_groups=(),
            steps=(),
            planned_path=(arena.start_pose,),
        )

        return self._simulator

    def rerun(self) -> None:
        """Run Fastest Car planning using the current editable setup."""

        from .task2_demo import build_fastest_car_demo

        try:
            scenario = build_fastest_car_demo(self.setup)

            self._simulator = scenario.simulator
            self.status_message = "Path planned successfully."

        except RuntimeError as exc:
            self._simulator = None
            self.status_message = f"Planning failed: {exc}"

    def randomize(self) -> None:
        """Replace the current setup with a random setup."""

        self.setup = random_fastest_car_setup()

        # Immediately show the new geometry, but don't automatically
        # run the relatively expensive path planner.
        self._simulator = None
        self.status_message = (
            "Randomized geometry. Press ENTER to plan."
        )

    def set_top_clearance(self, value_cm: int) -> None:
        """Change the top wall clearance."""

        self.setup = replace(
            self.setup,
            top_clearance_cm=value_cm,
        )
        self._simulator = None

    def set_bottom_clearance(self, value_cm: int) -> None:
        """Change the bottom wall clearance."""

        self.setup = replace(
            self.setup,
            bottom_clearance_cm=value_cm,
        )
        self._simulator = None

    def set_carpark_to_obstacle_1(self, value_cm: int) -> None:
        """Change the carpark-to-obstacle-1 distance."""

        self.setup = replace(
            self.setup,
            carpark_to_obstacle_1_cm=value_cm,
        )
        self._simulator = None

    def set_obstacle_1_to_obstacle_2(self, value_cm: int) -> None:
        """Change the obstacle-1-to-obstacle-2 distance."""

        self.setup = replace(
            self.setup,
            obstacle_1_to_obstacle_2_cm=value_cm,
        )
        self._simulator = None

    def toggle_obstacle_1_direction(self) -> None:
        """Toggle Obstacle 1's required arrow direction."""

        self.setup = replace(
            self.setup,
            obstacle_1_direction=(
                Steering.RIGHT
                if self.setup.obstacle_1_direction is Steering.LEFT
                else Steering.LEFT
            ),
        )
        self._simulator = None

    def toggle_obstacle_2_direction(self) -> None:
        """Toggle Obstacle 2's required arrow direction."""

        self.setup = replace(
            self.setup,
            obstacle_2_direction=(
                Steering.RIGHT
                if self.setup.obstacle_2_direction is Steering.LEFT
                else Steering.LEFT
            ),
        )
        self._simulator = None


__all__ = [
    "FastestCarSetup",
    "Task2EditorController",
    "random_fastest_car_setup",
    "FASTEST_CAR_LIMITS",
]