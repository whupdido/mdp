# Historical senior references: static review

Reviewed the six repository-local files independently. The Python-like files
were parsed as text only; none was executed because their imports include
unavailable modules. `equivalent_notes.txt` is plain, partially obfuscated
notes rather than Python. All hardware dimensions, motion deltas, map scales,
timings, and calibration values below are historical evidence only.

The six distinct SHA-256 digests confirm six distinct local byte streams:

| File | SHA-256 | Apparent role and visible design |
| --- | --- | --- |
| `planner_2_python.txt` | `3788629007ed5233f099dc946cc77b23af01b31c56fb08e91682e176d52c7430` | Grid/state search and robot control implementation. Uses an 80-cell map axis and coordinate conversion multiplied by four. Heading/state logic and command-specific movement branches are visible; imported transform and cost helpers are missing, so units and exact action semantics remain uncertain. |
| `procedure_tuned_python.txt` | `46f1215037ac3862a53829562a4605ad3ba5e23933bee7c91f7f44eafd14472e` | Tuned planning/control variant. It contains the same 80-axis and ×4 map-coordinate pattern, directional movement transforms, and search/controller structure. Search state/cost dependencies are imported, so the effective quantization, complete collision model, and action-to-cost mapping cannot all be established from this file alone. |
| `procedure_sturdy_python.txt` | `5144b28154f2425ebe01822104ebf20bba51b9a3eb49623a942f48def5264b1d` | Robust planner with explicit search, edge handling, route-cache, and target-order code. The visible closed-state key rounds x/y at a fixed scale and uses 45° heading bins. It includes directed source/target path entries and enumerates target permutations over cached costs. Two substantial planner sections/revisions appear concatenated in the same file. |
| `baseline_python.txt` | `01ea42da5fa6474be18efcaf860ef08635f533696f92c98fe85bb92568e01714` | Scenario/benchmark harness importing several planner variants. It lists 4-, 6-, and 8-obstacle layouts and prints a three-column runtime comparison. The imported implementations and benchmark environment are unavailable, so those times are historical claims, not comparable measurements. |
| `equivalent_notes.txt` | `0b230ba079a186510f9915ac728e2f285b1c31a4778cf808e979b3baa2f6c37c` | Partially obfuscated prose/benchmark notes with scenario arrays and a 4/6/8 timing table. It has no executable state, heading, collision, heuristic, or cache implementation to inspect. Its bytes differ from `emulation_python.txt`. |
| `emulation_python.txt` | `910dff7c3e5dfd0205c2b818f8bfc57b1142f6a7403ae54510f1d3ca9a96c1b3` | Pygame-style visual/control emulator. It maintains a visual pose and heading and issues discrete control actions through unavailable modules. It is not a byte duplicate of `equivalent_notes.txt`; imported model/controller behavior prevents confirmation of exact physical deltas and collision handling here. |

## Per-file implementation detail

"Not visible" means the file does not define the behavior locally, or it calls
an unavailable imported helper. It is not evidence that the historical system
lacked that behavior.

| File | State / heading / map | Motion and collision | Cost / heuristic | Targets, cache, ordering, performance | Calibration and transferable idea |
| --- | --- | --- | --- | --- | --- |
| `planner_2_python.txt` | Uses bounded 80-by-80 map coordinates and a visible fourfold obstacle-coordinate conversion. Heading/action handling is present, but parts of the state key and transform functions are imported. | Command-specific movement branches are visible. Occupancy helper is imported; exact footprint inflation and swept collision are not established. | Cost and heuristic helpers are imported; no reliable local cost ranking can be concluded. | Target generation and pairwise cache/order behavior are not established from this file. | Preserve the idea of separating command-specific measured motion from search. Its scale and deltas are hardware-specific. |
| `procedure_tuned_python.txt` | Defines an 80-axis map and visible `* 4` coordinate transforms. The effective search key/heading resolution depends on imported functions. | Direction-specific transforms are defined. An obstacle-clearance helper and near-edge exclusion are visible, but exact collision semantics depend on unavailable helpers. | Search and cost portions depend on imports; no exact cost/heuristic conclusion. | Target/path ordering is not sufficiently self-contained to verify. | A tuned variant shows that motion transforms can be centralized/configurable. Do not import its parameters. |
| `procedure_sturdy_python.txt` | Visible closed-state key rounds x/y and quantizes heading in 45-degree steps. Uses an 80-by-80 map. | Command-specific asymmetric deltas are visible. Obstacle inflation uses a helper over offsets derived from `7**2 + 7**2`; outer cells are reserved. Full footprint/sweep cannot be recovered. | A*-style `f=g+h`; visible Euclidean-plus-heading heuristic. Two actions have unit cost and four have cost 30, but obfuscated labels prevent proving which correspond to turns. | Directed pairwise paths are cached; target permutations are evaluated using those cached edges. This is an exact-ordering approach with factorial growth. | Repeated command endpoint measurement and conservative edge reserve are transferable experiments; all numeric values and action ratios are hardware-specific. |
| `baseline_python.txt` | No planner state implementation; it imports planner variants. | No self-contained motion/collision model. | No self-contained objective or heuristic. | Harness describes 4-, 6-, and 8-obstacle layouts and prints comparative runtime values; dependencies and protocol are unavailable. | Keep the idea of recording comparable scenario sizes and runtimes. Reported timings are not transferable. |
| `equivalent_notes.txt` | No executable state, heading, or map implementation; the text is partially obfuscated notes. | Not visible. | Not visible. | Contains separate 4/6/8 scenario arrays and timing notes; no complete harness to reproduce them. | Historical scenario records can guide test coverage only; its measurements remain unverified. |
| `emulation_python.txt` | Visual emulator/controller code tracks a pose and heading, but exact model behavior is delegated to unavailable imports. | Issues discrete command actions; exact command-specific displacement, collision checks, and boundaries are not verifiable here. | Not enough self-contained planner code to identify objective/heuristic. | No verifiable pairwise cache or target-order optimizer in the visible emulator flow. | A visual command replay is useful for workflow concepts. It does not establish physical fidelity or planner correctness. |

## Evidence by topic

- **Map and scale:** the planner files visibly use dimensions/range checks of 80 and convert obstacle or pose coordinates with a `* 4` factor. This supports an approximately 80×80 internal grid and fourfold coordinate scaling in those variants. It does not establish equivalence to our 200 cm planner grid.
- **Inflation and edges:** the sturdy code calls an unavailable helper using squared offsets derived from `7**2 + 7**2`, then excludes cells within roughly 3.5 grid cells of each outer edge. This supports conservative obstacle inflation and explicit edge reservation; the helper's exact footprint rule cannot be reconstructed safely.
- **Heading/state:** the sturdy search key includes quantized x/y and heading, with a visible 45° heading interval. The other planner variants contain eight-direction/45° heading operations, but some of their key/cost helpers are external. Do not assume they share an identical state key.
- **Primitives:** visible direction-specific branches assign different x/y displacements for commands and diagonals. This is a useful precedent for empirically calibrated command-specific movement models, not evidence that those deltas fit this robot.
- **Collision/boundaries:** occupancy checks include obstacle exclusion/inflation and edge restrictions. The exact footprint, corner sweep, and collision safety of missing helper modules cannot be verified. No historical collision claim replaces our sampled continuous footprint checker.
- **Costs and heuristic:** sturdy search has A*-style `f=g+h`; visible heuristic combines Euclidean distance and a heading-bin term. Two action labels have a unit cost while four have a cost of 30. Their action names are obfuscated at the cost site, so the interpretation “straight cheaper than turns” is plausible from neighboring movement branches but not proven solely by those labels. Do not transplant this ratio.
- **Targets and ordering:** visible target-position data associates observation positions with image/face information. The sturdy variant precomputes directed target-to-target paths, stores costs and paths, then searches target permutations. This supports directed pairwise caching and cached-cost order optimization conceptually. Candidate reachability/fallback details depend on imported helpers.
- **Performance:** precomputation plus a directed cache can avoid repeating local search across route permutations; exact permutation search still grows factorially with target count. The benchmark harness reports apparent speedups but provides no reproducible machine, versions, warmup/repetition protocol, or complete dependency chain.
- **Calibration:** command-specific displacements and asymmetric turn branches appear in the planner/emulator material. Their numeric values, units, and hardware differ; do not copy them. A future measured per-command endpoint model should be considered only after our robot's repeated endpoint measurements demonstrate material radius-model error.

## Requested hypothesis checks

| Hypothesis | Finding |
| --- | --- |
| Approximately 80×80 occupancy map | Supported in planner code by 80-cell coordinate bounds. |
| Obstacle coordinates scaled by approximately four | Supported by visible `* 4` conversion expressions. |
| Conservative obstacle inflation | Supported in sturdy code; exact extent relies on a missing helper. |
| Explicit edge reservation | Supported by visible edge-index exclusion near 3.5 cells. |
| Headings quantized in 45° increments | Supported for the sturdy search key; broader consistency is not established for every file. |
| Command-specific asymmetric motion deltas | Supported structurally; numeric values are hardware-specific and helper semantics are incomplete. |
| Straight actions substantially cheaper than turns | Suggested by 1-versus-30 action costs, but action labels are obfuscated; not asserted as verified mapping. |
| One principal image observation pose per obstacle | Suggested by fixed target pose/face associations in the planner material; imported target helpers prevent proving no alternatives exist. |
| Directed pairwise path caching | Supported in the sturdy variant. |
| Target ordering over cached pair costs | Supported in the sturdy variant through permutation evaluation and path reconstruction. |
| 4-, 6-, and 8-obstacle timing notes | Present in `baseline_python.txt` and separately in `equivalent_notes.txt`; not independently reproducible. |

## Transferable ideas

Keep our authoritative continuous collision checker and measured STM radii.
The historical material suggests future experiments with measured
command-specific endpoints, conservative clearance review, explicit boundary
reserve, directed-pair reuse, and profiling target-order optimization apart
from local path search. The current repository already has directed caching
and route-order optimization; benchmark their actual cost before adding more
layers. No historical dimensions, margins, displacements, 45° vocabulary,
calibration, or benchmark figures are adopted as production values.
