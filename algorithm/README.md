# MDP Task 1 Algorithm and Simulator

This package owns the Task 1 planning pipeline and its independent simulator.
It does not own Bluetooth, serial communication, the RPi bridge, STM32 motion
control, Android, or image recognition.

## Status

Phases 1 and 2 are implemented: validated immutable models, coordinate
transforms, physical/planner configuration, configurable motion primitives,
structured planning results, oriented footprint geometry, pose collision, and
swept-motion validation.

The following phases remain incomplete: observation-pose generation, the
simulator, Hybrid A*, pairwise planning, global routing, route serialization,
and transport adapters. Empty `pathfinding` and `simulator` packages are
retained for those phases.

## Architecture

```text
Arena input
    -> viewing-pose generation
    -> collision-aware local planning
    -> directed pairwise path cache
    -> target-order and observation-pose optimization
    -> route composition
    -> headless playback / Pygame visualization
    -> structured execution steps
```

Local path planning and global target ordering are deliberately separate. The
eventual exhaustive optimizer will provide the **optimal target ordering and
observation pose chain with respect to cached local path costs**. This is not a
claim of globally optimal physical execution.

Current package responsibilities:

- `config.py`: immutable physical, timing, primitive, and search configuration.
- `coordinates.py`: Android cell/body-center to rear-axle transformations.
- `geometry/`: Pygame-free footprint, obstacle, collision, and swept-motion logic.
- `models/`: input, pose, motion, path, route, and result contracts.
- `pathfinding/`: reserved for the configurable command-aligned Hybrid A*.
- `simulator/`: reserved for headless playback and its optional Pygame UI.

## Coordinate System and Units

- The arena is 200x200 cm and divided into 20x20 cells.
- Android cell `(0,0)` is the bottom-left cell.
- `x` increases East and `y` increases North.
- An Android cell `(x,y)` has physical center
  `((x+0.5)*10, (y+0.5)*10)` cm.
- Android robot coordinates refer to the body-center cell.
- Continuous planner poses refer to the rear-axle center.
- Headings use radians: East `0`, North `pi/2`, counter-clockwise positive.
- The documented start cell `(1,1,N)` converts to `(15 cm,15 cm,pi/2)` under
  the bundled zero axle/body-center-offset simulation profile.

Use `algorithm.coordinates` for integration conversions. Do not duplicate the
cell-center or rear-axle transformation in planners or renderers.

## Configuration and Calibration

`UNCALIBRATED_SIMULATION_CONFIG` allows deterministic development while making
no physical-readiness claim. Its current values are:

- 23.0x18.8 cm body with a 5 cm collision margin.
- Body center temporarily coincident with rear axle.
- Front-centered camera 11.5 cm ahead of the axle.
- 20 cm camera-to-image gap.
- Viewing offsets of 0, -10, and +10 cm.
- Forward/reverse straight primitives of 10 cm.
- FL/FR/BL/BR 90-degree radii of 26.1/31.8/24.6/30.3 cm.
- Provisional 90-degree durations of 2.4/2.9/2.3/2.8 seconds.

The primitive set is **command aligned and configurable**. Ninety-degree arcs
are the v1 hardware constraint, not a limitation of Hybrid A*. Multiple
primitive definitions may share an STM32 command verb, allowing later profiles
to add calibrated finer-angle successors.

The reverse convention currently assumes BL turns the nose right and BR turns
it left. Their semantics and radii must be floor-tested before routes depending
on them are considered physically ready.

## Models and Validation

Inputs use `ArenaInput`, `Pose`, `GridCell`, and `Obstacle`. An arena rejects
duplicate obstacle identifiers and duplicate occupied cells. Obstacles may be
stored before their image face arrives, but `ArenaInput.task1_issues()` reports
every missing face as a structured validation issue.

Planner outputs use `PairwisePath`, `PathMetrics`, `ObservationPose`,
`RoutePlan`, `PlanningResult`, and typed movement/capture execution steps.
Expected invalid-input and infeasible-route outcomes are represented as data,
not transport-layer errors.

## Dependencies and Development Setup

Development environment: Python 3.11.

Supported Python baseline: Python 3.10+.

Phases 1 and 2 use only the supported standard library. The repository
currently has no established Python dependency manifest, so these phases do
not create one. Code must continue avoiding features that require a newer
baseline unless the team deliberately changes the project requirement.

A local environment may be created without committing it:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install pytest
python -m pytest algorithm/tests
```

Before later phases add dependency metadata, re-check for a repository-wide
convention and reuse it. Pygame will be a simulator-only dependency: planning,
geometry, routing, integration contracts, and headless playback must import and
run without it.

## Checklist Mapping

- **B.1:** will be satisfied by headless route playback plus a simulator-only
  Pygame renderer showing the full grid, start zone, obstacles, faces, robot,
  movements, routes, and visited targets.
- **B.2:** will be satisfied by validated viewing poses, swept-footprint
  collision checking, configurable Hybrid A*, and exactly one capture event per
  requested target.
- **B.3:** will be satisfied by provisional execution-time costs, cached
  directed paths, exhaustive target ordering with observation-pose selection,
  and nearest-neighbour comparison.

Phases 1 and 2 supply the shared contracts and collision validation needed by
all three requirements but do not claim that any requirement is complete yet.
