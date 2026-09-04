# MDP Task 1 Algorithm and Simulator

This package owns the Task 1 planning pipeline and its independent simulator.
It does not own Bluetooth, serial communication, the RPi bridge, STM32 motion
control, Android, or image recognition.

## Status

Phases 1 through 6.2 are implemented: validated immutable models, coordinate
transforms, physical/planner configuration, configurable motion primitives,
structured planning results, oriented footprint geometry, pose collision, and
swept-motion validation, plus grouped observation-pose generation with camera
line-of-sight checks, deterministic headless playback, and an optional Pygame
visualizer, configurable command-aligned Hybrid A\* local planning, directed
pairwise caching, exact global target ordering, candidate-chain optimization,
complete route composition, and editable/seeded five-target assessment arenas.

Phase 4.1 adds presentation polish and manual B.1 verification guidance without
changing simulator state, geometry, collision, target generation, coordinates,
or motion semantics.

The following phases remain incomplete: final route-command serialization and
transport/integration adapters. Phase 5 remains the one-query local planner;
Phase 6 builds the complete Task 1 layer above it without changing Hybrid A\*.

## Architecture

```text
obstacles
    -> observation candidates
    -> pairwise Hybrid A*
    -> directed cost cache
    -> exhaustive target order
    -> candidate-chain DP
    -> complete route
    -> capture events
```

Local path planning and global target ordering are deliberately separate. The
exhaustive optimizer provides the **optimal target order and observation-pose
chain with respect to the cached directed Hybrid A\* pairwise costs**. This is not a
claim of globally optimal physical execution.

Current package responsibilities:

- `config.py`: immutable physical, timing, primitive, and search configuration.
- `coordinates.py`: Android cell/body-center to rear-axle transformations.
- `geometry/`: Pygame-free footprint, obstacle, collision, and swept-motion logic.
- `models/`: input, pose, motion, path, route, and result contracts.
- `pathfinding/`: configurable command-aligned Hybrid A\* and local path costs.
- `routing/`: directed cache, exact/baseline global optimizers, and Task 1 façade.
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
- FL/FR/BL/BR radii of 26.1/31.8/24.6/30.3 cm.
- Provisional 90-degree durations of 2.4/2.9/2.3/2.8 seconds; partial-turn
  durations are proportional simulation estimates only.

The primitive set is **command aligned and configurable**. The planning profile
uses bounded 30, 45, 60, and 90 degree arcs; STM confirmation now indicates
arbitrary turn angles are executable. The approximately 40 cm radius comment is
unverified and was not adopted. Multiple primitive definitions share an STM32
command verb, allowing later calibration without planner changes.

The reverse convention currently assumes BL turns the nose right and BR turns
it left. Their semantics and radii must be floor-tested before routes depending
on them are considered physically ready.

The initial search bookkeeping uses 5 cm position bins and 15 degree heading
bins,
a 5 cm goal-position tolerance, exact cardinal goal heading within numerical
tolerance, and a 50,000-expanded-node safeguard. These are replaceable
configuration values suitable for the 200 cm development arena; they are not a
claim of physically optimal resolution.

## Observation Pose Generation

Each annotated obstacle produces a bounded deterministic product of configured
camera-to-image standoffs and lateral offsets. The simulation profile orders
`20C, 20L, 20R, 10C, 10L, 10R, 30C, 30L, 30R`: 20 cm is the preferred
recognition distance, while 10 cm and 30 cm are simulation fallbacks pending
physical image-recognition validation. Because the configured body center and
rear axle coincide and the camera is 11.5 cm forward, these correspond to
body-center-to-face distances of 31.5 cm (preferred), 21.5 cm, and 41.5 cm.
No supported midpoint-distance range is present in the checkout, so the two
fallbacks remain configurable rather than claimed physically valid. The image target is the center
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
with their rejection reason for later simulation/debugging; routing receives
only geometrically valid alternatives.

Geometric validity does not imply Hybrid A\* reachability. Reachability is
evaluated separately by the Phase 6 directed pairwise layer; geometric flags
and rejection diagnostics are never mutated by routing.

## Hybrid A\* Local Planning

The complete configurable turn vocabulary remains 30/45/60/90 degrees, while
`search_turn_angles_deg` selects a bounded runtime profile. The editor and
local diagnostics use 30-degree arcs to reduce branching; repeated partial arcs
retain arbitrary-angle capability without implying a hardware limitation.

`HybridAStarPlanner` accepts a continuous start pose, continuous requested goal
pose, `ArenaInput`, configured physical/motion model, cost objective, and an
optional debug-data flag. It returns `LocalPlanningResult`, whose status is one
of `SUCCESS`, `NO_PATH`, `INVALID_START`, `INVALID_GOAL`, or
`SEARCH_LIMIT_REACHED`, or `PLANNING_TIMEOUT`. Normal infeasibility is returned as data rather than a
generic exception.

At the Task 1 façade, a route is reported as `NO_FEASIBLE_ROUTE` only when all
considered local transitions were conclusively infeasible. Expansion-budget and
wall-clock exhaustion remain distinct `SEARCH_LIMIT_REACHED` or
`PLANNING_TIMEOUT` outcomes and are retryable/diagnostic rather than proof of
geometric impossibility.

The planner state retains continuous `(x, y, heading)` values. Every successor
is produced by the existing Phase 2 motion propagation and remains continuous;
it is never snapped to a grid. Closed-set bookkeeping uses a separate
`HybridSearchKey` containing position bins, a heading bin, and the previous
gear/steering state needed by configured transition penalties.

The successor set comes directly from `PlanningConfig.motion.primitives` in its
configured deterministic order. The initial profile therefore uses:

- FW and BW: 10 cm forward and reverse straights.
- FL and FR: bounded 30/45/60/90-degree forward arcs with 26.1 and 31.8 cm radii.
- BL and BR: bounded 30/45/60/90-degree reverse arcs with 24.6 and 30.3 cm radii.

These are command-aligned configurable successors, not inherent Hybrid A\*
restrictions. A calibrated configuration can supply different distances,
angles, or radii without rewriting the search. Every attempted transition is
validated by the authoritative Phase 2 swept-footprint collision check; valid
endpoints with colliding intermediate motion are rejected.

The search uses a standard-library priority queue and conventional A\* values:

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
candidate, runs estimated-time Hybrid A\*, prints the local result summary,
displays expanded nodes by default, and adapts the resulting primitives and
sampled path to the unchanged headless simulator. The original `--demo` B.1
scenario remains deterministic and separate.

## Complete Task 1 Routing

`Task1Planner.plan(arena, objective=...)` is the Phase 6 facade. It generates
the existing grouped Phase 3 candidates, rejects a required target with no
geometrically valid candidate, plans every canonical directed local edge once,
and passes the resulting graph to a separate route optimizer. Hybrid A\*
remains responsible only for `pose A -> pose B`; local pathfinding is not
duplicated.

The pairwise graph contains `start -> candidate` edges and candidate edges
between different targets. It intentionally omits same-target candidate edges
because Task 1 chooses exactly one observation pose per obstacle. Edges are
directed: heading and asymmetric forward/reverse primitives mean `A -> B` and
`B -> A` may have different costs, paths, or reachability statuses.

`DirectedPairwisePathCache` stores successes and structured `NO_PATH`,
`INVALID_START`, `INVALID_GOAL`, and `SEARCH_LIMIT_REACHED` results. Its
immutable key contains the arena, full physical/planner configuration,
selected objective, endpoint identities, and exact endpoint poses. Candidate
index is therefore part of identity, and changing motion primitives, collision
geometry, discretization, objective, start, or goal cannot reuse a stale
entry. Repeated valid keys are cache hits and do not invoke Hybrid A\* again.

Only geometrically valid candidates enter the graph. Pairwise results then
provide a separate reachability layer: one unreachable candidate does not
invalidate its target when another candidate participates in a complete chain.
If no complete chain visits every required target, the result remains
`NO_FEASIBLE_ROUTE`; required targets are never silently skipped. Diagnostics
distinguish no geometric candidate, no incoming reachable candidate, search
limits, and the general absence of a complete directed chain.

The exact optimizer enumerates every stable target permutation. For each fixed
order it applies layered dynamic programming instead of enumerating the
candidate Cartesian product:

```text
best(next candidate)
    = min over reachable previous candidates:
        best(previous candidate) + cached directed edge cost
```

The start layer uses cached `start -> candidate` costs. The lowest complete
layer across every permutation selects both target order and one observation
candidate per target. Cost ties prefer fewer fallback candidates, then stable
target and candidate identity. This gives the precise guarantee: **optimal
target order and observation-pose chain with respect to the canonical cached
directed Hybrid A\* pairwise costs**. Continuous route materialization described
below may change the realized execution cost, so this is not a guarantee of a
globally optimal physical trajectory or calibrated shortest execution time.

Both `DISTANCE` and `ESTIMATED_TIME` minimize the sum of their corresponding
pairwise objective values without mixing units. Route metrics continue to
report total geometric distance and provisional execution time separately.
The estimated-time objective retains the Phase 5 timing and BL/BR calibration
warnings.

The initial profile charges the documented 50 ms overhead for every 10 cm
straight primitive and every turn primitive. Forward and reverse straights use
the same provisional formula; FL/FR/BL/BR use their configured durations.
Direction-change and steering-change penalties remain configurable but are
zero because no measured delay is available. Direction changes are therefore
counted but not additionally penalized in the current profile.

There is a known Phase 7 costing boundary: adjacent equal-direction straight
primitives may later serialize as one coalesced STM command, while Phase 5
currently charges overhead per search primitive. Correcting that exactly would
require search state and cost to track the length of the current coalescible
run. Phase 6 does not guess future serializer behaviour, so estimated-time
ordering remains explicitly provisional.

`NearestNeighbourRouteOptimizer` is a comparison baseline. At each step it
greedily chooses the cheapest reachable cached edge to any remaining target
candidate; equal edges prefer nominal, then stable target/candidate identity.
It does not backtrack and can fail or cost more than the exact result. It is
never selected as the execution route by `Task1Planner`.

The 5 cm local goal tolerance means a canonical pairwise leg can legally end a
few centimetres from its ideal Phase 3 candidate. Starting the next cached leg
at the ideal candidate would reset the physical pose and create a discontinuity.
After target/candidate selection, Phase 6 therefore materializes the selected
chain sequentially: the first leg starts at the arena start, and every later
leg is requested through the same directed cache from the preceding leg's
actual reached pose. At most one additional selected-edge query is needed per
boundary. Materialization failure returns structured `NO_FEASIBLE_ROUTE` data.

Composition then enforces exact shared physical boundary poses, removes only
genuine duplicate poses, and never inserts a correction or teleport. Each
original primitive becomes a structured move step, followed by one
`CaptureStep` at the actual accepted reached pose. Capture does not reset the
robot to the ideal candidate. The route retains target ID, candidate index, and
candidate kind, starts at the configured start, and ends after the final
capture. `optimized_candidate_chain_cost` records the canonical optimization
cost; `selected_route_cost` and route totals describe the continuous
materialized route. No return-to-start edge is added. Final STM protocol
strings remain Phase 7 work.

Run the complete deterministic five-target demonstration from the repository
root:

```powershell
python -m algorithm.simulator --task1-demo
```

It performs 25 canonical directed Hybrid A\* queries plus four continuous
selected-boundary queries for five active nominal candidates, evaluates all
120 target orders, and prints the initial pose, grouped primitive legs,
forward/reverse counts, direction changes, costs, distance, time, and runtime.
The sidebar displays the selected order and candidates.
Playback finishes `COMPLETE` with `VISITED 5/5`, and captures come only from
the structured composed route. The demo uses the existing provisional 3 cm
simulation margin so the safety footprint at the authoritative `(1,1,N)`
Android start maps to rear-axle pose `(15,15,pi/2)` and fits completely inside
the displayed 40 cm (4x4-cell) start zone. It uses one active candidate per target for
bounded startup;
automated routing tests cover expanded candidate layers without Cartesian-product search.
`--demo` remains the simulator-only B.1 scenario and `--hybrid-demo` remains
the single local Phase 5 query.

## Task 1 Scenario Editor and Random Arenas

### Phase 6.5 maneuverability diagnostics

The current editor profile retains the deterministic `10/20/30 C/L/R`
observation candidates (20 cm centered is preferred) and adds bounded partial
Ackermann arcs at 30, 45, 60, and 90 degrees. Each arc uses its existing
direction-specific radius; only the angle and proportional provisional timing
vary. Heading bins are 15 degrees for bookkeeping while propagated poses stay
continuous. `N` toggles candidate markers in the editor; `C` remains the
candidate toggle in the general simulator. The teammate-confirmed arbitrary
turn-angle capability is not a radius calibration: the informal ~40 cm radius
estimate remains unverified and is not used.

Phase 6.2 adds a Pygame-free `Task1EditorController` and a thin optional Pygame
frontend. Editing produces a new validated `ArenaInput`; it does not implement
pathfinding or collision logic. Planning continues through the existing Phase
3 candidate generator, `Task1Planner`, directed cache, exact optimizer,
continuous composition, capture events, and headless playback.

Launch the editor with the deterministic reference layout loaded but not yet
planned:

```powershell
python -m algorithm.simulator --task1-editor
```

Editor controls:

| Input | Action |
| --- | --- |
| Left-click obstacle | Select it |
| Left-click empty cell with a selection | Move the selected obstacle to that snapped grid cell |
| Left-click empty cell without a selection | Add the next available target ID with a North image face |
| Right-click obstacle, Delete, or Backspace | Remove it |
| W / A / S / D | Set image face to North / West / South / East |
| N | Show or hide 10/20/30 cm C/L/R observation markers |
| Enter | Validate and plan all five targets |
| Space | Play or pause a successful plan |
| Right Arrow | Step one primitive or capture event |
| R | Reset playback execution state while retaining the valid plan and obstacles |
| F5 | Generate an arbitrary raw five-target arena; it may be unsolvable |
| Shift+F5 | Generate a new planner-verified solvable arena with bounded retries |
| + / - | Change playback speed |
| Q / Escape | Quit |

The selected obstacle panel reports its target ID, grid `(x,y)`, and full image
face name. All mouse placement snaps to the existing zero-based 20x20
`GridCell` convention. The start remains the authoritative `(1,1,N)` Android
cell / `(15,15,pi/2)` rear-axle pose and cannot be randomized.

The editor requires exactly five unique target IDs and cells, a North/East/
South/West image face for every target, an unobstructed initial robot pose, and
no obstacle in the protected 4x4 start zone. No course briefing artifact is
stored in this checkout; the 40 cm value follows the supplied current-course
requirement. The project documentation does not
state a broader placement rule, so this protection is deliberately limited to
the assessment start zone. Domain constructors reject out-of-arena cells,
duplicate cells, and invalid face types; the authoritative collision checker
validates the start footprint.

Any position or image-face edit immediately discards the old planning result,
route, candidates, and playback state. The next Enter press constructs the new
arena and replans it. Editing is blocked during active or paused playback, so
obstacles cannot teleport through an executing route. The directed cache may
be retained internally, but its key includes the complete arena and therefore
cannot reuse an incompatible edge.

The editor distinguishes three concepts explicitly:

- **Image face:** North, East, South, or West on the obstacle.
- **Candidate identity:** standoff plus lateral class, such as `20C`, `20L`, or `30R`.
- **Robot heading:** degrees plus the cardinal value, for example `90 deg (N)`.

After planning, the panel shows structured success/failure status, target
order, candidate type per target, distance, provisional time, visited count,
per-target geometric/reachable candidate counts, and candidate-generation/
pairwise/global/total planning times. A failed map retains its specific
`PlanningIssue`; it is never converted into partial Task 1 success. Console
diagnostics distinguish invalid input, missing geometric candidates, local
reachability failures, global-chain failures, and search-limit exhaustion.

Raw seeded generation is reproducible but does not promise a complete route:

```powershell
python -m algorithm.simulator --task1-random --seed 42
```

It creates exactly five unique interior cells with valid random image faces.
Press Enter to test the resulting map. Planner-verified solvable generation is
separate:

```powershell
python -m algorithm.simulator --task1-random --seed 1 --solvable --retry-limit 50
```

Normal Shift+F5 requests advance one persistent, unseeded RNG stream, so each
press continues from new random state. An explicit CLI `--seed` creates a
reproducible stream for tests and bug reports only. Every request prints its
request number, RNG identifier, attempt limit, stable scenario signature,
coordinates/faces, per-target geometric and reachable candidate counts,
planning status, failure category, pairwise request count, and timing.

Solvable proposals are not derived by cosmetically editing the fixed demo.
They construct obstacle coordinates from randomized executable command walks,
snap only the resulting targets to the existing grid, require at least three
cells of center separation, protect the start zone, require valid camera poses,
and randomize target-ID assignment. These are generator heuristics, not relaxed
collision or planning rules. The complete Task 1 planner still accepts or
rejects every proposal. Consecutive unseeded acceptances must differ in at
least two target coordinates and cannot repeat an earlier accepted signature.

The default bounded retry limit is 50. A failed Shift+F5 request preserves the
current edited/planned arena and reports `RANDOM SOLVABLE GENERATION FAILED`;
it never loads the deterministic demo or the last known solvable proposal as a
fallback. Press Shift+F5 again to continue the persistent random stream.

An open-looking grid can still be unreachable: the robot cannot rotate in
place, has finite asymmetric turning radii, must finish at the required image-
facing cardinal heading, and currently uses command-aligned 10 cm straights
and 90-degree arcs. `NO_PATH` and `SEARCH_LIMIT_REACHED` therefore remain
different diagnostic outcomes.

### Phase 6.3 robustness profile

`generate_assessment_like_task1_arena` samples independently of planner motion.
It uses only constraints established in this checkout or supplied for the
current course: a 200 cm square / 20x20-cell arena, 10 cm obstacles, exactly
five unique targets with cardinal image faces, the `(1,1,N)` start, and no
obstacle in the lower-left 40 cm / 4x4-cell start zone. No supported minimum
inter-obstacle distance, outer-boundary exclusion, or guaranteed image-access
rule was found locally, so the generator does not invent one. Such maps may be
physically inaccessible and are intentionally distinct from Shift+F5's
planner-biased command-walk proposals.

The physical body is 23.0 by 18.8 cm. The standard uncalibrated collision
profile adds 5 cm on every side for an effective 33.0 by 28.8 cm footprint;
the B.1, fixed Task 1, and live-editor demonstration profile preserves its
existing 3 cm margin for an effective 29.0 by 24.8 cm footprint. The legacy
`ROBOT_SIZE_CM = 30` integration constant is the only repository-local
recommended planning-size evidence. These values are reported separately and
must not be changed merely to improve route success.

The recorded Phase 6.3 independent benchmark uses seed 6300 and 100 maps on
the bounded live 20C profile. It found 91 maps with at least one target having
no geometric candidate and 9 maps whose directed searches reached the 20-node
limit; none completed 5/5. The run issued 225 pairwise searches (5 successful,
220 failed), averaged 0.331 s/map, and used an explicitly shallow 57,600-byte
cache-entry estimate. Generating the same 100 maps with the full nine-pose
geometry reduced no-candidate maps from 91 to 69 (2,679 of 4,500 poses valid),
but an eager full five-target graph can require 1,665 searches and exceeded 90
seconds on the fixed arena, so a full nine-candidate route benchmark is not
represented as completed evidence.

For the 20 valid one-cell perturbations of the deterministic five-target map,
7 remained solvable, 11 exhausted the search limit, and 2 lost complete global
connectivity; none lost exactly one active 20C candidate. Raising a diagnostic
copy from 20 to 100 expansions increased representative per-map runtime from
about 3 seconds to 14--16 seconds and improved some candidate reachability,
but did not produce a complete route in the three sampled cases. The
production limit therefore remains unchanged.

Phase 6.4 lets the live editor generate all nine candidates without eagerly
planning all 1,665 possible directed edges. B.2 feasibility mode activates
`20C`, then `20L/20R`, then `10C/30C`, and finally the remaining lateral
fallbacks. Each exact target-permutation/candidate-layer DP attempt asks the
directed cache for an edge only when it needs that edge, and all prior tier
results remain cached. It stops at the first tier yielding a complete route.
That is a feasibility result, not a claim of optimality across inactive
candidates.

`RoutingMode.FULL_OPTIMIZATION` activates the complete configured candidate
set in one exact optimization pass. Only that mode preserves the claim of an
optimal target order and observation-pose chain with respect to every cached
directed local cost in the configured full set. It can be substantially slower
and is not the live-editor default.

Local B.2 queries use bounded adaptive budgets of 20 then 100 expansions, a
1.5-second local timeout, and a 15-second overall Task 1 guard. A
`SEARCH_LIMIT_REACHED` result is retryable and carries its budget, attempts,
expanded nodes, and runtime; `SUCCESS` and genuine `NO_PATH` remain final.
The open-set keeps Euclidean `f` admissible and uses heading mismatch only as
a deterministic equal-`f` tie-break. Equal-length immediate `FW/BW` inverses
are pruned, while turns and legitimate reverse manoeuvres remain available.
The 5 cm/cardinal search buckets are unchanged. Previous gear/steering are
included in dominance keys only when configured change penalties make that
history affect future cost.

The same seed-6300 assessment-like benchmark used in Phase 6.3 still produced
0/100 complete routes in Phase 6.4. The breakdown changed from 91 no-candidate
and 9 search-limit maps to 69 no-candidate and 31 search-limit maps because the
full geometric candidate set is now available. Lazy planning issued 368 local
requests, recovered 29 searches during 341 larger-budget retries, found 36
successful local paths, and expanded 33,197 nodes. Average/median/p95 planning
times were 4.768/0.0067/15.844 seconds. Thus adaptive search materially
improves local reachability, but not enough to form a complete route in this
unconstrained arbitrary-map sample under the 15-second guard.

The identical 20 one-cell perturbations improved from 7/20 to 13/20 complete
routes. Two still ended at search limits and five lacked a complete global
chain. The run made 331 retries, recovered 95 local searches, and expanded
37,624 nodes. All 13 successes used tier 1, so this improvement is attributable
to adaptive search/dominance rather than fallback observation poses. A
separate end-to-end editor regression makes every `20C` goal unreachable and
proves that tier 2 fallbacks can produce a continuous five-capture route.

For the recorded 100-map raw batch (seed 6200), 76 maps lacked a geometric
candidate for at least one target and 24 entered pairwise planning but exhausted
the configured local search bound; 0 were falsely accepted. Average planning
time was 1.435 s, median time was 0.00086 s, and the batch made 600 directed
pairwise requests. Raw F5 is therefore useful for failure/debug demonstrations,
while Shift+F5 is the assessment-facing planner-verified action.

Recommended B.2 live workflow:

1. Launch `--task1-editor`.
2. Select and move an obstacle to another grid cell.
3. Select an obstacle and change its image face.
4. Press Enter and show the new target order and candidate types.
5. Press Space and play the planner-generated route.
6. Confirm five unique capture events and final `COMPLETE`, `VISITED 5/5`.

A complete B.2 success requires all five image targets to be reached and
captured exactly once. B.2 is demonstrable on both the deterministic five-target
assessment scenario and user-modified or planner-verified random valid
five-target scenarios. Unsolvable maps are valid failure demonstrations, not
B.2 successes.

Capture remains only the Algorithm subsystem's request at a valid observation
pose. Actual image classification belongs to the image-recognition subsystem
during integrated execution; Phase 6.2 adds no computer vision or networking.

## Simulator

`HeadlessSimulator` owns logical playback state and consumes sampled poses,
configured motion primitives, capture events, or future execution steps. Its
immutable snapshots keep the full planned trail separate from the trail that
has actually executed. Logical time advances only through `advance(delta_s)`,
so results do not depend on rendering frame rate. A target becomes visited only
when its capture event executes; reset clears both progress and visited state.

The optional renderer shows the 20x20 arena, 4x4 start zone, obstacles and
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
Compact labels use unambiguous `obstacle ID:distance/lateral` text, such as
`2:20C` or `2:30L`. Each glyph is centered on the stored rear-axle pose; its
tip is not another destination.
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
are demonstrated separately by `--task1-demo`.

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

| Key         | Action                               |
| ----------- | ------------------------------------ |
| Space       | Play or pause                        |
| N           | Single-step one primitive or event   |
| Right Arrow | Single-step one primitive or event   |
| R           | Reset                                |
| +           | Increase playback speed              |
| -           | Decrease playback speed              |
| G           | Toggle grid labels                   |
| C           | Toggle observation candidates        |
| L           | Toggle camera rays                   |
| F           | Toggle authoritative robot footprint |
| P           | Toggle planned path                  |
| E           | Toggle executed path                 |
| D           | Toggle future debug-node overlay     |
| Q           | Quit                                 |
| Escape      | Quit                                 |

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

The Phase 6.2 editor intentionally keeps the start pose fixed. Arbitrary start
editing and dynamic obstacle replanning during playback remain out of scope.

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

The planning core, Hybrid A\*, and headless simulator use only the supported standard
library. The repository currently has no established Python dependency
manifest, so Phases 4 through 6 do not create one. Code must continue avoiding features
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
  movement types, route layers, and target progress. Phase 6 adds a separate
  integrated five-target route demonstration without changing the B.1 demo.
- **B.2:** Phase 6.2 is complete at the software-planning/simulation level: every
  required demo target has a selected reachable pose, every selected cached leg
  is swept-collision-checked, the route starts at the configured start, every
  target occurs exactly once, capture follows its accepted observation leg,
  and playback finishes with all five targets visited. Physical execution is
  not validated. The same pipeline is demonstrable with the fixed reference,
  user-edited, and planner-verified seeded random valid five-target arenas.
- **B.3:** Phase 6 supplies directed caching, exhaustive target ordering,
  exact candidate-layer selection, and nearest-neighbour comparison. The
  result is optimal only with respect to cached directed local costs; physical
  shortest-time claims remain blocked by provisional timings and calibration.

Phases 1 through 6.2 supply shared contracts, collision validation, observation
targets, B.1 visualization/playback, local planning, and complete software-level
Task 1 routing. They do not claim physical readiness.
