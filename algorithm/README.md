# MDP Algorithm Module

This module contains the path planning software for the MDP robot. The project is organised into three main components.

```
algorithm/
│
├── models/
├── simulation/
├── pathfinding/
└── constants.py
```

---

## models/

Represents the state of the world. These classes contain data only.

Files:

- `arena.py` – Represents the 2.0m × 2.0m arena.
- `robot.py` – Represents the robot's position and orientation.
- `obstacle.py` – Represents an obstacle and its associated image face.

---

## simulation/

Contains the software simulator used for development and debugging.

Responsibilities include:

- Displaying the arena
- Displaying the robot
- Displaying obstacles
- Visualising robot movement
- Testing path planning algorithms before deployment to the physical robot

---

## pathfinding/

Contains all path planning algorithms.

Examples include:

- Hamiltonian path
- Hybrid A\*
- Dubins path
- Route optimisation

---

## constants.py

Stores project-wide constants such as:

- Arena dimensions
- Grid size
- Robot dimensions
- Obstacle dimensions
- Turning radius
