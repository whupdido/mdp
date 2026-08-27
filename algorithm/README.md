# MDP Task 1 Algorithm and Simulator

This package owns the Task 1 planning pipeline and its independent simulator.
It does not own Bluetooth, serial communication, the RPi bridge, STM32 motion
control, Android, or image recognition.

## Status

Phases 1 through 5 are implemented: validated immutable models, coordinate
transforms, physical/planner configuration, configurable motion primitives,
structured planning results, oriented footprint geometry, pose collision, and
swept-motion validation, plus grouped observation-pose generation with camera
line-of-sight checks, deterministic headless playback, and an optional Pygame
visualizer, followed by configurable command-aligned Hybrid A* local planning.

Phase 4.1 adds presentation polish and manual B.1 verification guidance without
changing simulator state, geometry, collision, target generation, coordinates,
or motion semantics.

The following phases remain incomplete: directed pairwise planning and caching,
global routing, route serialization, and transport adapters. Phase 5 solves
only one directed local `start -> goal` query at a time.

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
- `pathfinding/`: configurable command-aligned Hybrid A* and local path costs.
- `simulator/`: deterministic headless playback and its optional Pygame UI.
- `targets/`: camera transforms, image-face geometry, line of sight, and
  grouped observation candidates.

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

The initial search bookkeeping uses 5 cm position bins, cardinal heading bins,
a 5 cm goal-position tolerance, exact cardinal goal heading within numerical
tolerance, and a 50,000-expanded-node safeguard. These are replaceable
configuration values suitable for the 200 cm development arena; they are not a
claim of physically optimal resolution.

## Observation Pose Generation

Each annotated obstacle produces ordered nominal, robot-left, and robot-right
candidates from the configured lateral offsets. The image target is the center
of the annotated 10 cm obstacle face. Candidate camera position is calculated
as:

```text
face center + outward normal * image gap + face tangent * lateral offset
```

The generator then inverts the configured rear-axle-to-camera transform to
obtain the rear-axle `Pose`. The robot heading is the opposite of the image
face, so it faces the obstacle side. The configured image gap is the
perpendicular distance from the face plane; fallback viewing rays may be
diagonal because of their lateral displacement.

Every candidate uses the Phase 2 authoritative pose collision check and a
continuous camera-to-face-center segment test against all other obstacle
rectangles. The target obstacle is excluded from line-of-sight blockers so its
own face endpoint does not reject the ray. Invalid candidates remain grouped
with their rejection reason for later simulation/debugging; routing will
receive only geometrically valid alternatives.

Geometric validity does not imply Hybrid A* reachability. Reachability is
evaluated separately by the Phase 5 local planner described below.

## Hybrid A* Local Planning

`HybridAStarPlanner` accepts a continuous start pose, continuous requested goal
pose, `ArenaInput`, configured physical/motion model, cost objective, and an
optional debug-data flag. It returns `LocalPlanningResult`, whose status is one
of `SUCCESS`, `NO_PATH`, `INVALID_START`, `INVALID_GOAL`, or
`SEARCH_LIMIT_REACHED`. Normal infeasibility is returned as data rather than a
generic exception.

The planner state retains continuous `(x, y, heading)` values. Every successor
is produced by the existing Phase 2 motion propagation and remains continuous;
it is never snapped to a grid. Closed-set bookkeeping uses a separate
`HybridSearchKey` containing position bins, a heading bin, and the previous
gear/steering state needed by configured transition penalties.

The successor set comes directly from `PlanningConfig.motion.primitives` in its
configured deterministic order. The initial profile therefore uses:

- FW and BW: 10 cm forward and reverse straights.
- FL and FR: 90-degree forward arcs with 26.1 and 31.8 cm radii.
- BL and BR: 90-degree reverse arcs with 24.6 and 30.3 cm radii.

These are command-aligned v1 hardware constraints, not inherent Hybrid A*
restrictions. A calibrated configuration can supply different distances,
angles, or radii without rewriting the search. Every attempted transition is
validated by the authoritative Phase 2 swept-footprint collision check; valid
endpoints with colliding intermediate motion are rejected.

The search uses a standard-library priority queue and conventional A* values:

```text
f(n) = g(n) + h(n)
```

`g(n)` is actual accumulated selected-objective cost. For distance search,
`h(n)` is Euclidean position distance. For provisional-time search, it is
Euclidean distance divided by the configured maximum speed bound, which is the
documented 27.3 cm/s straight speed in the initial profile. Neither heuristic
adds a heading penalty. Since any physical path is at least the Euclidean
distance and configured change penalties are non-negative, these are
conservative lower bounds. Standard symmetric Reeds-Shepp is deferred because
the configured radii are asymmetric and BL/BR semantics remain physically
unverified.

A state reaches the requested goal when position and heading are within the
configured tolerances and the reached pose remains collision free. The goal is
not forced to exact floating-point equality. The default 5 cm positional
tolerance matches the current observation-planning contract; the default
heading tolerance preserves the cardinal camera-facing direction.

Distance and provisional execution time remain separate. Distance cost uses
primitive geometric length. Provisional time uses:

```text
straight fixed time
    + max(0, length - deceleration distance) / straight speed
    + straight settle time
    + serial command overhead
```

Turns use their configured primitive duration plus command overhead. Configured
direction-change and steering-change penalties are added consistently and are
zero by default. These are uncoalesced local-search estimates, not final STM
serialization or a physical timing guarantee.

On success, `HybridPath` contains the requested and reached poses, original
generating `MotionPrimitive` objects, ordered `MotionSegment` objects, key
poses, the complete sampled path, monotonic cumulative objective costs, and
`PathMetrics`. Metrics include geometric distance, provisional time,
forward/reverse distance, turns, direction/steering changes, command count,
expanded/generated nodes, collision checks, and runtime. Parent links preserve
the actual generating primitive rather than inferring commands from pose
differences.

Optional `HybridSearchDebug` contains plain tuples of expanded and generated
continuous poses. It has no Pygame dependency and can feed the simulator's
existing debug-node overlay. Equal-priority heap entries use an insertion
counter, configured primitives are expanded in tuple order, and states reopen
only for a strictly lower cost, making path and search counts deterministic for
identical inputs.

A collision-free Phase 3 candidate can still be unreachable under these
command-aligned motions. Phase 5 reports that as local planning failure without
altering Phase 3 candidate validity or selecting a different candidate.

Run the Phase 5 single-query demonstration from the repository root:

```powershell
python -m algorithm.simulator --hybrid-demo
```

It uses a dedicated Phase 5 arena, selects a real valid Phase 3 observation
candidate, runs estimated-time Hybrid A*, prints the local result summary,
displays expanded nodes by default, and adapts the resulting primitives and
sampled path to the unchanged headless simulator. The original `--demo` B.1
scenario remains deterministic and separate.

## Simulator

`HeadlessSimulator` owns logical playback state and consumes sampled poses,
configured motion primitives, capture events, or future execution steps. Its
immutable snapshots keep the full planned trail separate from the trail that
has actually executed. Logical time advances only through `advance(delta_s)`,
so results do not depend on rendering frame rate. A target becomes visited only
when its capture event executes; reset clears both progress and visited state.

The optional renderer shows the 20x20 arena, 3x3 start zone, obstacles and
annotated faces, valid/invalid observation candidates, camera rays, the
safety-expanded robot footprint, rear axle, heading, camera, planned/executed
trails, current command, and visited targets. It uses `WorldViewport` to invert
screen y while preserving the planner's bottom-left world origin. Rendering
never mutates or advances simulator state.

Install Pygame locally and run the collision-checked demonstration from the
repository root:

```powershell
python -m pip install pygame
python -m algorithm.simulator --demo
```

### Visual language

The dark control panel contains the live status, legend, and controls while the
arena remains neutral and uncluttered. Nominal candidates use purple diamonds,
left fallbacks use blue triangles, and right fallbacks use orange squares.
Valid candidates are filled; invalid candidates use a red ring and cross.
Compact labels use `obstacle ID:candidate kind`, such as `2:L`.
The primary B.1 demo keeps its obstacles comfortably inside the arena and shows
valid candidates. Near-boundary invalid-candidate behavior remains covered by
the automated target-generation tests instead of making the main scenario look
physically suspect.

Clear camera rays are green and blocked rays are red. The planned route is a
thin dashed blue trail, while the executed route is a solid orange trail that
grows during playback. Obstacles turn green and receive a check indicator after
capture; their image face remains a strong red edge marker.

The robot has two independent visual layers. The translucent blue rectangle is
the authoritative safety-expanded Phase 2 footprint. The top-down body,
windshield, wheels, front bar, rear axle, heading line, and camera marker are a
decorative Pygame overlay derived from the same pose and configured geometry.
The decorative layer is never used for collision checking or planning. The
legend also documents the footprint, paths, visited targets, target face, rear
axle, and heading.

### Demo sequence and B.1 movements

The actual deterministic motion sequence is:

```text
FW -> FW -> FW -> FW -> FW -> FR -> FW -> FW -> FL -> BW -> BL -> BR
```

The demo inserts `CAPTURE(2)` twice after FL. The repeated capture exists solely
to demonstrate that duplicate capture events do not double-count a visited
target. Targets 1 and 3 remain unvisited so both visual states remain available.

The B.1 demo intentionally demonstrates simulator rendering, deterministic
playback, controls, movement types, capture state, and diagnostic overlays
only. It is not expected to visit every target and does not represent globally
planned Task 1 execution. Global target ordering and complete route composition
belong to later phases.

- `FW`: forward straight movement.
- `BW`: reverse straight movement.
- `FL`: forward movement with left steering.
- `FR`: forward movement with right steering.
- `BL`: reverse movement with left steering; the provisional geometry changes
  the nose to the right.
- `BR`: reverse movement with right steering; the provisional geometry changes
  the nose to the left.

All demo movement uses the authoritative Phase 2 sampler and swept-footprint
collision checker. Use 0.25x playback or primitive/event stepping to inspect a
short movement closely.

### Controls

| Key | Action |
| --- | --- |
| Space | Play or pause |
| N | Single-step one primitive or event |
| Right Arrow | Single-step one primitive or event |
| R | Reset |
| + | Increase playback speed |
| - | Decrease playback speed |
| G | Toggle grid labels |
| C | Toggle observation candidates |
| L | Toggle camera rays |
| F | Toggle authoritative robot footprint |
| P | Toggle planned path |
| E | Toggle executed path |
| D | Toggle future debug-node overlay |
| Q | Quit |
| Escape | Quit |

Because the documented `(15,15,N)` start pose conflicts with the default 5 cm
safety-expanded body boundary, the demo explicitly derives a simulation-only
3 cm margin. It does not alter `UNCALIBRATED_SIMULATION_CONFIG` or claim
physical calibration.

## Manual B.1 Verification

From the repository root, activate the Python 3.11 virtual environment and run:

```powershell
.\.venv\Scripts\Activate.ps1
python -m algorithm.simulator --demo
```

Use this checklist for simulator-level B.1 sign-off. Items 1-17 verify the B.1
arena, robot, targets, and six movement types. Items 18-35 verify the playback,
state separation, controls, and diagnostic overlays used to demonstrate them.

1. A 2 m by 2 m arena is displayed.
2. The arena is represented as a 20 by 20 grid.
3. The start zone is visible in the lower-left region.
4. Multiple obstacles are visible.
5. Target obstacles clearly show the annotated image-facing side.
6. Observation candidates can be displayed.
7. Nominal, left fallback, and right fallback candidates are visually distinguishable.
8. Main-demo candidates are valid; near-boundary invalid candidates remain verified by automated target tests.
9. Camera rays can be displayed.
10. The decorative robot is aligned with the authoritative oriented rectangular footprint.
11. Robot heading is visually identifiable.
12. Forward straight movement can be observed (`FW`).
13. Reverse straight movement can be observed (`BW`).
14. Forward left movement can be observed (`FL`).
15. Forward right movement can be observed (`FR`).
16. Reverse left movement can be observed (`BL`).
17. Reverse right movement can be observed (`BR`).
18. Planned path can be toggled with P.
19. Executed path can be toggled with E.
20. Executed path grows as playback progresses.
21. Planned and executed trails remain logically and visually separate.
22. Space pauses playback.
23. Space resumes playback.
24. N and Right Arrow each advance exactly one primitive or capture event.
25. R restores the simulation to its initial state while retaining the planned trail.
26. + increases playback speed, up to 8x.
27. - decreases playback speed, down to 0.25x.
28. The first `CAPTURE(2)` marks target 2 visited while targets 1 and 3 remain unvisited.
29. Single-step the repeated `CAPTURE(2)` and confirm the visited count does not increase.
30. R clears all visited-target state.
31. G toggles grid labels.
32. C toggles the candidate overlay.
33. L toggles the camera-ray overlay.
34. F toggles the authoritative footprint without removing the decorative robot.
35. D toggles the empty Phase 5 debug-node overlay without affecting normal rendering.

**B.1 is considered complete at the simulator demonstration level once the
Manual B.1 Verification checklist passes.** This does not claim physical
calibration, B.2 completion, or B.3 completion.

An interactive scenario editor is intentionally deferred until the local
planner is operational. Future work may allow setting the start pose, adding or
removing obstacles, changing image faces, loading arbitrary Task 1 scenarios,
and invoking the planner from the simulator. None of those editor features are
part of Phase 4.1.

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

The planning core, Hybrid A*, and headless simulator use only the supported standard
library. The repository currently has no established Python dependency
manifest, so Phases 4 and 5 do not create one. Code must continue avoiding features
that require a newer baseline unless the team deliberately changes the project
requirement.

A local environment may be created without committing it:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install pytest pygame
python -m pytest algorithm/tests
```

Before later phases add dependency metadata, re-check for a repository-wide
convention and reuse it. Pygame is a simulator-only dependency: planning,
geometry, routing, integration contracts, and headless playback import and run
without it. Renderer smoke tests skip cleanly when Pygame is unavailable.

## Checklist Mapping

- **B.1:** Phase 4 supplies headless route playback plus a simulator-only
  Pygame renderer showing the full grid, start zone, obstacles, faces, robot,
  movement types, route layers, and target progress. Final integrated-route
  demonstration remains pending later planning phases.
- **B.2:** Phase 5 now supplies a collision-free local route from one pose to
  one reachable observation pose. Pairwise candidate planning, global target
  ordering, route composition, and one capture per target remain incomplete.
- **B.3:** Phase 5 supplies directed local geometric and provisional-time costs.
  Directed caching, exhaustive target ordering, observation-pose-chain
  selection, and nearest-neighbour comparison remain incomplete.

Phases 1 through 5 supply shared contracts, collision validation, observation
targets, B.1 visualization/playback, and one-to-one local planning. They do not
yet claim complete Task 1 routing or physical readiness.
