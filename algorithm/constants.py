"""Stable arena and protocol constants.

Physical and planner tuning values belong in :mod:`algorithm.config`.  This
module only contains values fixed by the Android integration contract.
"""

# Arena
ARENA_SIZE_CM = 200
GRID_SIZE = 20
CELL_SIZE_CM = ARENA_SIZE_CM // GRID_SIZE
START_CELL_X = 1
START_CELL_Y = 1

# Robot
ROBOT_SIZE_CM = 30

# Obstacles
OBSTACLE_SIZE_CM = 10

# Published image identifier range used by Android.
MIN_IMAGE_ID = 11
MAX_IMAGE_ID = 40
