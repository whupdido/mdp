# MDP Android Remote Controller

The tablet app. Draws the arena, places obstacles, drives the robot, and shows
what the robot reports back.

**Checklist C.1 – C.10: all signed off.** The module is complete, so this
document is written for the people integrating against it rather than for
whoever is building it.

| | | | | |
|---|---|---|---|---|
| `C.1` send / receive over BT | ✅ | | `C.6` place, drag, delete obstacles | ✅ |
| `C.2` scan and connect | ✅ | | `C.7` annotate the target face | ✅ |
| `C.3` interactive robot control | ✅ | | `C.8` survive a dropped link | ✅ |
| `C.4` filtered status box | ✅ | | `C.9` show the target ID | ✅ |
| `C.5` 2D arena with robot | ✅ | | `C.10` move the robot from a message | ✅ |

---

## Contents

- [If you are integrating, read this](#if-you-are-integrating-read-this)
- [The message protocol](#the-message-protocol)
- [Starting a run](#starting-a-run)
  - [Task 1 is three presses](#task-1-is-three-presses)
  - [Task 2 is one press](#task-2-is-one-press)
  - [Everything that has to be running](#everything-that-has-to-be-running)
- [Where the robot starts](#where-the-robot-starts)
- [Quick start](#quick-start)
- [Using the app](#using-the-app)
- [Testing without a robot](#testing-without-a-robot)
- [Architecture](#architecture)
- [Running the tests](#running-the-tests)
- [An emulator that matches the real tablet](#an-emulator-that-matches-the-real-tablet)
- [Known limits and gotchas](#known-limits-and-gotchas)
- [Where to change things](#where-to-change-things)

---

## If you are integrating, read this

The coordinate contract is in [`PROTOCOL.md`](PROTOCOL.md) and is shared with
the planner — `algorithm/coordinates.py` and `algorithm/README.md` agree with
it. The short version:

| | |
|---|---|
| Arena | 20 × 20 cells, 10 cm each, 200 × 200 cm |
| Origin | `(0,0)` is the **bottom-left** cell; x East, y North |
| Obstacle | one cell; its `(x,y)` **is** that cell |
| Robot | 3 × 3 cells; its `(x,y)` is the **body-centre** cell, not a corner |
| Legal robot centres | `1..18` in both axes |
| Start zone | `x,y ∈ 0..3` (40 cm), matching `algorithm/constants.py` |
| Start pose | `(1,1,N)` |
| Image IDs | `11..40` |

The one that catches people is the robot coordinate: it is the centre of the
3 × 3 footprint. If your side treats it as a corner, both modules look correct
in isolation and the error only appears once they are connected.

**Per module:**

- **Raspberry Pi** — SPP, UUID `00001101-0000-1000-8000-00805F9B34FB`. While
  disconnected the app dials *and* listens at the same time, which is what makes
  C.8 work: it recovers whether we redial or you reconnect. `rpi/a1_bridge.py`
  already routes correctly — motion commands to the board, `ADD`/`SUB`/`FACE`
  acknowledged with `STATUS,MAP,…` and not forwarded, anything else rejected.
  Verify changes with `python3 rpi/test_a1_bridge.py`, no hardware needed.
- **Path planning** — send `ROBOT,<x>,<y>,<D>` as the robot moves and the map
  follows live with a trail. Obstacles reach you as `ADD,B<n>,(<x>,<y>)` on
  finger lift.
- **Image recognition** — send `TARGET,<obstacle>,<id>[,<face>]`. `<id>` must be
  11–40 and `<obstacle>` must already exist on the map, or the message is
  dropped.

  The rules also require the RAW captures, with their bounding boxes, tiled in
  one window at the end of a run — on *either* the tablet or the laptop. **This
  is done on the laptop, for both tasks** (Denzel, 25 Sep; Task 2 since #40),
  so the tablet deliberately does not have that page. If that ever changes, it
  lands here and it is not a small job; do not assume the tablet already
  covers it.
- **STM board** — the drive controls emit your format unchanged. Every string
  the app can send lives in one file,
  [`protocol/Outbound.kt`](app/src/main/java/com/example/androidapp/protocol/Outbound.kt).

---

## The message protocol

### The app sends

| When | String | Item |
|---|---|---|
| Obstacle placed, or moved and finger lifted | `ADD,B<n>,(<x>,<y>)` | C.6 |
| Obstacle dragged off the arena | `SUB,B<n>` | C.6 |
| Target face annotated | `FACE,B<n>,<D>` | C.7 |
| Drive controls | `FW<ddd>` `BW<ddd>` `FL<ddd>` `FR<ddd>` `BL<ddd>` `BR<ddd>` `STOP` | C.3 |
| SETUP pressed | `ROBOT,<x>,<y>,<D>`, the whole map, then `COMPUTE` | rules |
| PLAN pressed | `ARM` | rules |
| START pressed, Task 1 | `START` | rules |
| START pressed, Task 2 | `START2` | rules |

Two-letter verb, three zero-padded digits: `FW010` is forward 10 cm, `FL090` a
90° forward-left turn. Everything outbound is newline-terminated, and exactly
one message goes out per gesture, on finger lift — never a stream while
dragging.

### Starting a run

The rules are strict about this: the run must be started **from a button on the
tablet**, and during the attempt the team may not touch anything else — not the
laptop, not the robot. (Pressing Enter in the Pi's SSH session still works, for
testing off the clock. Don't use it on the day.)

#### First: start the right program on the Pi

**This is the easiest thing to get wrong.** Two programs listen on the same
Bluetooth link and neither does the other's job, so the wrong one makes the
buttons look dead:

| On the Pi | Handles | Use for |
|---|---|---|
| `python3 run_task1.py` | `COMPUTE`, `ARM`, `START`, the map and start pose | **Task 1 only** |
| `python3 a1_bridge.py` | drive commands, `STOP`, `START2` | **Task 2**, manual driving, checklist demos |

Each one now says so if it receives the other's trigger, rather than ignoring
it — silence is indistinguishable from a broken button.

#### Task 1 is three presses

All three happen in the **two-minute preparation window**, except the last.
Nothing below costs run time until START.

| | Press | Tablet sends | Pi answers |
|---|---|---|---|
| 1 | **SETUP** | `ROBOT,<x>,<y>,<D>`, the whole map, `COMPUTE` | `STATUS,PLAN,WORKING` → `READY,<moves>` or `FAILED,<why>` |
| 2 | **PLAN** | `ARM` | `STATUS,PLAN,CHECKING` → `ARMED,<secs>` → `SET` |
| 3 | **START** | `START` | drives |

**SETUP** plans. Planning is slow and the six minutes are precious; the rules
give two minutes of preparation and that is the budget it should come from.
The tablet shows *PATH FOUND — n moves* and the button becomes PLAN.

**PLAN** pre-flights. A zero-length move asks whether the board is awake
without turning a wheel, and a socket probe asks whether anyone actually
started `server/algo_server.py` on the laptop. Both are free here and both
cost the whole run if you discover them later. START goes green on `SET`.

**START** drives, immediately. `ARMING_DELAY_SECONDS` in `run_task1.py` is
**0**: a hold only ever existed to make planning look like it happened inside
the run window, and it does not need to — Prof Smitha confirmed the setup time
may overlap the execution time. Anywhere after START it comes straight out of
the six minutes, and FAQ 14 separates equal scores on timing. It is still a
variable if someone wants a second or two before the robot lurches.

Only one press is live at a time, so they cannot be taken out of order under
pressure. **Any edit to the map or the start pose sends you back to SETUP** —
running a route planned for a different layout is exactly the kind of mistake
the rules give no second chance for, and it would otherwise be invisible.

The map goes out in full on SETUP — every `ADD` and `FACE`, one line every
50 ms — even though edits also go out as they are made. `run_task1.py` only
starts collecting obstacles once it is running, so anything keyed in before
someone launched it was being dropped. Repeating is safe: `ADD` and `FACE`
overwrite by obstacle number on the Pi rather than accumulate.

#### What a run looks like on the wire

```
-- sending start pose and 3 obstacle(s) --
TX  ROBOT,1,1,N
TX  ADD,B1,(5,13)    TX  FACE,B1,W
TX  ADD,B2,(5,7)     TX  FACE,B2,S
TX  ADD,B3,(12,9)    TX  FACE,B3,E
TX  COMPUTE                          <- press 1, SETUP
RX  STATUS,PLAN,READY,42             <- "PATH FOUND"
TX  ARM                              <- press 2, PLAN
RX  STATUS,PLAN,CHECKING
RX  STATUS,PLAN,SET                  <- START goes green
TX  START                            <- press 3
RX  ROBOT,1,4,N                      <- pose updates as it drives
RX  TARGET,1,35,W                    <- image ID, live on the map
...
RX  MSG,Task 1 route complete
```

#### When a move goes wrong mid-run

`run_task1.py` does not abandon the route on the first problem — each
unvisited obstacle is worth ten points.

| Board says | What happens |
|---|---|
| `BLOCKED` | IR stopped it short, robot is safe. Backs off 10 cm, re-plans over the obstacles still to do, continues. Twice at most. |
| `STALL`, `TIMEOUT` | May be wedged. Stops: driving more is how a run loses the ability to stop itself. |
| `BUSY`, `ERR` | Protocol fault. Stops rather than flooding the board. |

An obstacle already photographed is never read again — a second look can
detect something different, and a wrong image ID is minus ten points (FAQ 4).

#### Task 2 is one press

Task 2's obstacles go down only *after* the preparation time, and the rules
forbid entering anything about them into the system (rule 2). So there is
nothing to key in, nothing to plan and no map to send: tap **TASK 2**, wait
for the supervisor, press **START**. The clock counts down from 3:00.

| | Press | Tablet sends | What comes back |
|---|---|---|---|
| 1 | **START** | `START2` | `STM,ACK`; at each obstacle `STM,SCAN` and an arrow `MSG`; then `STM,PARKED` |

`a1_bridge.py` hands `START2` to the board, and `task_2()` drives the whole
course by itself. At each obstacle the board stops and says `SCAN`. The Pi
photographs the arrow, asks the laptop what it is, and answers `IM038` (go
round the right) or `IM039` (go round the left) inside the board's 3 s window.
A miss is retried, up to four scans per obstacle; after four misses the board
dodges **left** by default. A wrong side is a disqualification (FAQ 10), so
the default is a last resort, not a plan.

**The clock stops on `PARKED`.** The board sends it the moment the car is in
the carpark and stopped, which is exactly when the rules stop timing (rule 6).
`DONE` follows when the routine returns, and still stops the clock on firmware
from before `PARKED`.

**The images go to the laptop afterwards.** Once the routine has returned —
so it costs no run time — the Pi sends one frame per obstacle back to the
laptop, which boxes and tiles them like Task 1's (rule 8):
`yolo_logs/task1_collage_task2-<time>.jpg`. Open it after the run, let the
supervisor photograph it, and email it to them (rule 10).

What the real `a1_bridge.py` sends the tablet, captured against a scripted
board where obstacle 1 was read on its second scan:

```
TX  START2                                     <- the one press
RX  STATUS,SENT,START2
RX  STM,ACK                                    "Robot accepted Task 2."
RX  STM,SCAN                                   "Robot stopped to read an arrow."
RX  MSG,No arrow seen - board will use its default     <- it retries first
RX  STM,SCAN
RX  MSG,Arrow LEFT (39)                        obstacle 1: round the left
RX  STM,SCAN
RX  MSG,Arrow RIGHT (38)                       obstacle 2: round the right
RX  STM,PARKED                                 "Parked in 1:52." Clock stops
RX  STM,DONE                                   "Robot has finished Task 2."
RX  MSG,Task 2 images on the laptop (2 of 2)
```

**Retry** (rule 7): press **RUN AGAIN**, then **START**. **Stopping by hand**:
**STOP RUN**, then confirm. That scores as incomplete; the board stops where
it is and does not send `PARKED`.

The robot icon does not move during Task 2. The route is the board's own and
nothing reports poses — and the rules only ask for a live map in Task 1.
Because `START2` blocks for up to three minutes, the bridge waits on
`TASK2_TIMEOUT_SECONDS` (200 s) rather than the per-move timeout. The full
board-side contract is the `START2` section of
[`stm32/STM32_motion_spec.md`](../stm32/STM32_motion_spec.md).

#### Everything that has to be running

```bash
# Laptop
python -m server.yolo_task1       # image detection, port 5001 -- both tasks
python -m server.algo_server      # route planning, port 5002 -- Task 1 only

# Pi -- once; Denzel's autostart picks the program and restarts it if it dies
cd rpi
./install_autostart.sh <tablet-mac> task1 <laptop-ip>   # Task 1: run_task1.py
./install_autostart.sh <tablet-mac> task2 <laptop-ip>   # Task 2: a1_bridge.py
#   switch task: run it again with the other one.  Logs: journalctl -u mdp -f
#   by hand instead: sudo rfcomm bind 0 <tablet-mac>, then the program

# Robot, during prep, inside the carpark
long-press the board button to zero the gyro (rule 1 allows calibration)

# Tablet
Connect -> pick the Pi -> green "Connected" -> TASK 1 or TASK 2
```

Forget the laptop's algo server and **PLAN will tell you** before the clock
starts. That is the whole reason it is a separate press.

**Task 2 has no such check.** If `server.yolo_task1` is not running, every
scan misses and the board dodges left at both obstacles — a disqualification
the moment an arrow says right. Look at the laptop terminal before START.

### Where the robot starts

The planner is **told** the start pose rather than assuming `(1,1,N)`. The
robot starts in the carpark, but which cell of it and facing which way is the
supervisor's call on the day.

On the map, the robot uses the same two gestures obstacles already do: **drag
it** to move it, **tap it** for the compass. It is sent as
`ROBOT,<x>,<y>,<D>` — the same shape as the inbound line, because it means the
same thing in both directions — and `rpi/run_task1.py` passes it to
`algo_client.plan_route(obstacles, start=…)`.

The app warns before planning if the robot's 3 × 3 body is not **wholly inside
the carpark**: leaving it during preparation is a disqualification (FAQ 9).

**Task 2 sends no start pose and no map** — the route is the board's own. See
[Task 2 is one press](#task-2-is-one-press).

Obstacle numbers are **never reused while an obstacle is alive**. Deleting B2
leaves B3 called B3, because the robot has already been told about B3.

### The app understands

| String | Effect | Item |
|---|---|---|
| `ROBOT,<x>,<y>,<D>` | moves and turns the robot, drops a breadcrumb | C.10 |
| `TARGET,<n>,<id>` | block `n` shows `<id>` in large white text | C.9 |
| `TARGET,<n>,<id>,<D>` | as above, plus a coloured bar on face `<D>` | C.9 |
| `MSG,[text]` | one line in the status box | C.4 |
| `STATUS,<text>` | one line in the status box | — |
| `STATUS,SENT,<cmd>` | "Sent `<cmd>` to the robot." | — |
| `STATUS,MAP,<msg>` | receipt for one of our own map edits; Traffic only | — |
| `STATUS,PLAN,WORKING` | planning started, after SETUP | rules |
| `STATUS,PLAN,READY,<moves>` | route exists — "PATH FOUND". Enables **PLAN**, not START | rules |
| `STATUS,PLAN,CHECKING` | pre-flight running, after PLAN | rules |
| `STATUS,PLAN,ARMED,<secs>` | pre-flight passed; the Pi is holding this long (0 by default) | rules |
| `STATUS,PLAN,SET` | hold over — **this is what makes START live** | rules |
| `STATUS,PLAN,FAILED,<why>` | no route, or pre-flight failed; the reason is shown | rules |
| `STM,<reply>` | relayed board reply | — |
| `STM,PARKED` | Task 2 over: **stops the clock**. Bare `PARKED` and `STATUS,PARKED` work too | rules |
| `STM,[WARN] <text>` | board diagnostic; a warning, not a status line | — |
| `ERR,<reason>` | warning — something we sent was refused | — |

`STM,<reply>` covers `READY` `DONE` `ACK` `BUSY` `STALL` `TIMEOUT` `BLOCKED`
`SCAN` `ERR` `NO_REPLY`. **`STALL`, `TIMEOUT` and `BLOCKED` raise a visible
warning**, because the STM spec says position is unknown after any of them —
the robot drawn on the map is wrong until something re-references it.

**`[WARN] COLLISION …` is treated the same way.** Since the IR sensors went in,
the board stops short of an obstacle rather than hit it, and announces that
with a `[WARN]` line before the reply code (`stm32/Core/Src/control.c`,
`move_straight_mm`). On current firmware the code is `BLOCKED`; on firmware
from before `command.c` reported *how* a move ended it was a plain `DONE`,
which on its own said the move succeeded. The app reads the warning line as
"position lost" so it is right either way, and prints "Move ended early" for
the code that follows instead of "Move complete". Any other `[WARN]` line is
shown as a warning verbatim.

### Parsing is deliberately forgiving

The written checklist and the ARCM briefing slides disagree about the `TARGET`
format, and a supervisor typing into the AMD tool by hand will produce whichever
they remember. So `B2` and `2` both work, whitespace is trimmed, the keyword and
direction letter are case-insensitive, coordinates may be `(10,6)` or `10,6`, and
a trailing empty field is ignored.

Anything unrecognised is logged and dropped. Anything out of range, or naming an
obstacle that does not exist, is dropped with a short on-screen note. **Nothing
throws and nothing crashes the app** — you can send it garbage all day.

---

## Quick start

**You need:** Android Studio and the Android SDK with platform 37.

```bash
git clone https://github.com/whupdido/mdp.git
cd mdp/Android
```

Open the **`Android` folder** in Android Studio — not the repo root, the Gradle
project is one level down — and press Run.

**`local.properties` does not come from Git.** It holds a machine-specific path
so it is gitignored. Android Studio writes it on first open; if a command-line
build says *SDK location not found*, create it yourself:

```properties
sdk.dir=C:/path/to/your/Android/Sdk
```

```bash
./gradlew installDebug     # build and push to a connected device
./gradlew assembleDebug    # just build the APK
./gradlew test             # 94 unit tests, no device needed
```

**Toolchain:** AGP 9.3.1, Gradle 9.5, JDK 25, `compileSdk` 37, `minSdk` 24.
Kotlin comes from AGP's built-in support, which is why there is deliberately no
Kotlin plugin in `build.gradle.kts`. This combination works — please don't
accept Android Studio's upgrade prompts without telling the team, because a
version bump breaks the build for everyone at once.

---

## Using the app

One screen, landscape, tablet-first. No drawer and no tabs: during a timed run
nobody should have to navigate.

```
┌────────────────────────────┬────────────┬──────────────────────┐
│                            │ RUN        │  ROBOT LINK          │
│                            │ [ TASK 1 ] │  ● Connected to …    │
│        ARENA               │ [ TASK 2 ] │  [Connect][Disconn]  │
│        20 × 20             │            │  Simulator       [ ] │
│                            │    6:00    ├──────────────────────┤
│   tap    → add obstacle    │            │  STATUS  or  TRAFFIC │
│   drag   → move it         │ IMAGES 0/5 │                      │
│   off    → delete it       │            ├──────────────────────┤
│   tap it → face compass    │ hint       │  DRIVE        [Pad]  │
│                            │ [ SETUP  ] │  [F-L][FWD][F-R]     │
│   robot: drag → move       │            │  [B-L][BCK][B-R]     │
│          tap  → facing     │ [ START  ] │  [-][+]  [  STOP  ]  │
│            ROBOT (1,1) N   │            ├──────────────────────┤
│                            │            │[Undo][Clear][Demo][⟲]│
└────────────────────────────┴────────────┴──────────────────────┘
```

**Run** is the only column that matters during an attempt. Pick **TASK 1**
(6:00) or **TASK 2** (3:00) and the clock shows that budget.

| Task 1 | Task 2 |
|---|---|
| **SETUP → PLAN → START**, one live at a time | **START** only — no SETUP button, nothing to plan |
| **IMAGES** counts the IDs found | no tally — Task 2 is scored on time |
| ends when every obstacle has an ID, or the Pi says the route is done | ends on `PARKED` |

While a run is live, START turns red and reads **STOP RUN** (it asks first: a
hand stop scores as incomplete). DRIVE and the bottom row disappear, so nothing
else can be pressed by accident — the rules allow only the start button during
an attempt. When the run ends the clock freezes and the button turns green:
**RUN AGAIN**, which clears the clock for the retry.

**Arena.** Tap an empty cell to add an obstacle; it takes the lowest free
number. Drag to move, drag past the edge to delete. Tap a placed obstacle to
open a magnified four-quadrant compass and pick N/E/S/W — an obstacle is one
cell out of twenty across, far too small to hit an edge directly, so the compass
is the touch interaction that satisfies C.7.

**Status** shows selected information only, never the raw stream. That is a
checklist requirement (C.4), not a style choice. Everything on the wire goes to
**Traffic** instead; `⌄` swaps between them.

**Drive** sends one command per tap, never auto-repeat: the board replies `BUSY`
and discards anything sent mid-move. `−`/`+` change the distance step; tapping
the readout cycles the turn angle.

**Pad** swaps the buttons for a gesture control — drag out from the centre,
release to fire. Release in the middle for `STOP`, outside the ring to cancel,
so a stray touch costs nothing. There is no plain "left" or "right" because the
car cannot turn on the spot.

**⟲ Replay** scrubs back through the run. Every robot pose and target report is
recorded automatically, so you never have to decide in advance that a run was
worth keeping.

### Reading the arena

The palette carries meaning, so the screen reads at a glance mid-run:

| | |
|---|---|
| **Amber** | something you did or can do |
| **Cyan** | something the robot is telling you |
| **Red** | a target |
| **Green** | the link is healthy |

Obstacles are drawn as raised blocks with a lit top and a cast shadow, because
they *are* 10 cm cubes standing on a floor. It stays a true plan view, so
coordinates still read correctly.

When a target is recognised the block shows the **ID in large white text** — the
C.9 requirement — and a chip beside it carrying the **character actually printed
on that image** (`A`, `7`, `↑`). Mapped from the briefing's image pool in
`Arena.glyphFor()`, so there are no image assets to manage.

The robot does not teleport between poses. It travels, and a turn leaves along
the heading it was already facing before curving into the new one. Right turns
animate wider than left because they *are* wider — `FR` 352 mm against `FL`
282 mm, the radii the firmware runs (`stm32/Core/Inc/calib.h`, tape measured
07-Oct with the front weights on). The tables in `stm32/STM32_motion_spec.md`
are the older 25-Sep measurements (`FR` 366, `FL` 272); **`calib.h` is the
source of truth**. A forward 90° turn carries the car 2.8–3.5 cells along. If the map ever appears to pivot the
robot on the spot, it is lying about the robot.

---

## Testing without a robot

You do not need hardware, Bluetooth or even a tablet.

1. Turn on **Simulator**. The Traffic drawer opens automatically.
2. Press **Connect** and pick "Simulated robot".
3. Type an inbound message into the field and press **Inject**.

```
ROBOT,7,2,W          robot moves to (7,2) facing West
TARGET,B2,11,N       obstacle 2 shows a large 11, red bar on its north face
target,b3,25,e       same thing — the parser is not fussy
MSG,[Looking for target 3]
STM,DONE
ROBOT,99,99,N        ignored, with a warning — out of bounds
!!!                  ignored silently, logged to Traffic
```

A Task 2 run, after tapping TASK 2 and START:

```
STM,ACK              "Robot accepted Task 2."
STM,SCAN             "Robot stopped to read an arrow."
MSG,Arrow LEFT (39)  shown as is
STM,PARKED           the clock stops; START becomes RUN AGAIN
STM,DONE             "Robot has finished Task 2."
```

While the clock is running, `uiautomator dump` cannot read the screen (it
waits for an idle UI that never comes), so scripted emulator tests have to tap
by coordinate mid-run.

Everything the app *sends* is logged in Traffic too, so you can confirm your own
module will receive what it expects before wiring anything together.

---

## Architecture

```
com.example.androidapp/
├─ MainActivity.kt        wiring only — no decisions, no drawing
├─ MdpViewModel.kt        every decision, both halves of the app
├─ arena/
│   ├─ ArenaModel.kt      pure Kotlin: grid, obstacles, robot, transitions
│   └─ ArenaView.kt       custom View: draws and hit-tests, nothing else
├─ control/
│   └─ GesturePad.kt      the C.3 gesture control
├─ protocol/
│   ├─ Inbound.kt         pure Kotlin: tolerant parser
│   └─ Outbound.kt        pure Kotlin: every string we send
└─ link/
    ├─ Link.kt            the interface between the map and the radio
    ├─ BluetoothLink.kt   real SPP — C.1, C.2, C.8
    └─ FakeLink.kt        simulator, same interface
```

**Two rules make this work.**

`ArenaModel`, `Inbound` and `Outbound` import nothing from Android. They are
plain Kotlin, so they run as JVM tests in about a second — no emulator, no
tablet. Getting `ROBOT,7,2,W` parsing right via a JUnit test is a one-second
loop; doing it by installing an APK is ninety seconds. Please keep them clean.

`Link` is the seam. Everything about Bluetooth lives behind it; everything about
the arena lives in front and never imports `android.bluetooth`.

```kotlin
interface Link {
    val state: StateFlow<LinkState>       // Disconnected / Listening / Connecting / Connected / Failed
    val incoming: SharedFlow<String>      // one complete message per emission
    fun send(line: String): Boolean       // false if not connected; never throws
    fun connect(device: RemoteDevice?)    // null = listen for an incoming connection
    fun disconnect()
    // plus discovery: pairedDevices(), startScan(), stopScan(), discovered, scanning
}
```

Need a third transport — WiFi to the PC, say? Implement `Link` and nothing else
in the app has to change.

---

## Running the tests

```bash
cd Android && ./gradlew test
```

**94 JVM tests, no device needed.**

- `ProtocolTest` (39) — every inbound format, both `TARGET` spellings, the Pi
  bridge's whole vocabulary including `STATUS,PLAN,…` and `PARKED`, and a pile
  of garbage that must not crash it.
- `ArenaModelTest` (21) — obstacle numbering and reuse, collisions, bounds, the
  3 × 3 footprint, target ID range, breadcrumbs, headings, and the image-pool
  glyph map.
- `RunStateTest` (34) — both tasks' budgets, clocks and start strings, which
  button is live in every run phase, where the start pose may go, and
  re-running the same map without last run's IDs counting.

The other end of the link has its own offline tests, no hardware needed:

```bash
python3 rpi/test_a1_bridge.py     # Task 2 and manual driving: START2, SCAN, PARKED, the image sheet
python3 rpi/test_run_task1.py     # Task 1 runner
python3 rpi/test_task1_chain.py   # Task 1, SETUP to the last image, with fakes
```

Add to these when you change parsing or the model. They are fast, and they are
the reason the protocol survived contact with the real bridge.

---

## An emulator that matches the real tablet

The assigned tablet is a **Samsung Galaxy Tab A7 Lite (SM-T220)**: 800 × 1340 at
213 dpi, about 1007 × 601 dp in landscape. That is a tight vertical budget and it
has already caused real layout bugs — a phone-shaped emulator will not catch
them. Create an AVD with exactly those values (Device Manager → New → resolution
800 × 1340, density 213), API 33, landscape.

**No emulator has a Bluetooth controller.** `adb shell service check bluetooth`
reports *not found*; discovery returns nothing and connecting always fails.
Confusingly, `pm list features` still claims `android.hardware.bluetooth`. Use
Simulator mode on the emulator, and the real tablet for anything with a radio.

---

## Known limits and gotchas

- **No Bluetooth on emulators.** C.1, C.2 and C.8 can only be tested on the
  tablet.
- **The Pi does one thing at a time.** While a move runs (up to 25 s), or the
  whole of Task 2 (up to 200 s), only `STOP` is handled straight away; anything
  else the tablet sends waits its turn and is then handled in order.
- **Task 2 has no pre-flight.** Nothing checks the laptop before START, so a
  dead detection server shows up as "No arrow seen" at the first obstacle —
  already too late. Check the laptop first.
- **The tablet's clock starts at the START press.** If preparation overruns
  two minutes, the rules add the extra to the run time (FAQ 8); the tablet
  does not know about that. The supervisor's watch is the official one.
- **The robot cannot turn on the spot.** Ackermann steering: a 90° turn carries
  the car 2.5–3.6 cells along, and right turns need ~25 % more space than left
  going forward, ~40 % in reverse (`calib.h`, 07-Oct: FL 282, FR 352, BL 254,
  BR 357 mm).
- **`STALL` and `TIMEOUT` invalidate the map.** After either, the drawn position
  is stale until something re-references it.
- **Pair the tablet and the Pi in Android Settings first**, not in code. If they
  are not bonded, nothing else works.
- **Start zone is 40 cm here and in the planner**, but neither of us has checked
  it against the physical arena.

---

## Where to change things

| To change | Edit |
|---|---|
| Any string the app transmits | `protocol/Outbound.kt` |
| How an inbound message is understood | `protocol/Inbound.kt` |
| What a message *does* to the map | `MdpViewModel.kt` |
| How the arena looks | `arena/ArenaView.kt` (colours at the bottom) |
| Grid size, robot footprint, start pose, image glyphs | `arena/ArenaModel.kt` |
| Bluetooth behaviour, reconnect strategy | `link/BluetoothLink.kt` |
| The gesture pad | `control/GesturePad.kt` |
| Screen layout | `res/layout/activity_main.xml` |

Keep decisions out of `ArenaView`, and keep Android out of `ArenaModel` and
`protocol/`. That separation is what keeps the tests fast and the merges small.
