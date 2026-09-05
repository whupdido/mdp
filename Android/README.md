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

Two-letter verb, three zero-padded digits: `FW010` is forward 10 cm, `FL090` a
90° forward-left turn. Everything outbound is newline-terminated, and exactly
one message goes out per gesture, on finger lift — never a stream while
dragging.

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
| `STM,<reply>` | relayed board reply | — |
| `ERR,<reason>` | warning — something we sent was refused | — |

`STM,<reply>` covers `READY` `DONE` `ACK` `BUSY` `STALL` `TIMEOUT` `ERR`
`NO_REPLY`. **`STALL` and `TIMEOUT` raise a visible warning**, because the STM
spec says position is unknown after either — the robot drawn on the map is wrong
until something re-references it.

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
./gradlew test             # 45 unit tests, no device needed
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
┌──────────────────────────────┬──────────────────────┐
│                              │  ROBOT LINK          │
│                              │  ● Connected to …    │
│        ARENA                 │  [Connect][Disconn]  │
│        20 × 20               │  Simulator       [ ] │
│                              ├──────────────────────┤
│   tap    → add obstacle      │  STATUS  or  TRAFFIC │
│   drag   → move it           │                      │
│   off    → delete it         ├──────────────────────┤
│   tap it → face compass      │  DRIVE        [Pad]  │
│                              │  [F-L][FWD][F-R]     │
│                              │  [B-L][BCK][B-R]     │
│              ROBOT (7,2) W   │  [-][+]  [  STOP  ]  │
├──────────────────────────────┼──────────────────────┤
│                              │[Undo][Clear][Demo][⟲]│
└──────────────────────────────┴──────────────────────┘
```

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
animate wider than left because they *are* wider — `FR` 413 mm against `FL`
317 mm, per the 31-Aug re-measurement in `stm32/STM32_motion_spec.md`. A 90°
turn carries the car 3.1–4.2 cells along. If the map ever appears to pivot the
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

Everything the app *sends* is logged in Traffic too, so you can confirm your own
module will receive what it expects before wiring anything together.

For AMDTool testing using the laptop as client:

First, scan for the tablet via the AMDTool's interface. Then pair it.
Secondly, use the Android App (Client) on the Tablet to initiate connection with the AMDTool (Server)



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

**45 JVM tests, no device needed.**

- `ProtocolTest` — every inbound format, both `TARGET` spellings, the Pi
  bridge's whole vocabulary, and a pile of garbage that must not crash it.
- `ArenaModelTest` — obstacle numbering and reuse, collisions, bounds, the 3 × 3
  footprint, target ID range, breadcrumbs, and the image-pool glyph map.

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
- **The Pi bridge blocks while waiting for the board.** For up to 25 s after
  forwarding a motion command it reads nothing from Android, so obstacle edits
  made mid-move queue in the RFCOMM buffer. Fine for a demo, worth remembering
  during a timed run.
- **The robot cannot turn on the spot.** Ackermann steering: a 90° turn carries
  the car 3.1–4.2 cells along, and right turns need ~30 % more space than left
  going forward, 35 % in reverse.
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

