# MDP Android Remote Controller

The tablet app. Draws the arena, places obstacles, drives the robot, and shows
what the robot reports back.

**Checklist status: C.1 – C.10 all signed off.**

| | | |
|---|---|---|
| C.1 | Send and receive text over the Bluetooth serial link | ✅ |
| C.2 | Scan, select and connect to a Bluetooth device | ✅ |
| C.3 | Interactive control of robot movement | ✅ |
| C.4 | Status box showing selected remote updates | ✅ |
| C.5 | 2D arena with numbered obstacles and the robot | ✅ |
| C.6 | Place, drag and delete obstacles by touch | ✅ |
| C.7 | Annotate which obstacle face holds the target image | ✅ |
| C.8 | Survives a dropped link and reconnects by itself | ✅ |
| C.9 | Show the target ID on an obstacle block | ✅ |
| C.10 | Move and turn the robot from an inbound message | ✅ |

---

## Contents

- [Quick start](#quick-start)
- [Using the app](#using-the-app)
- [Testing without a robot](#testing-without-a-robot)
- [The message protocol](#the-message-protocol) ← **start here if you are integrating**
- [Integrating with your module](#integrating-with-your-module)
- [Architecture](#architecture)
- [Running the tests](#running-the-tests)
- [An emulator that matches the real tablet](#an-emulator-that-matches-the-real-tablet)
- [Known limits and gotchas](#known-limits-and-gotchas)
- [Where to change things](#where-to-change-things)

---

## Quick start

**You need:** Android Studio, and the Android SDK with platform 37.

```bash
git clone https://github.com/whupdido/mdp.git
cd mdp/Android
```

Open the `Android` folder in Android Studio (not the repo root — the Gradle
project lives one level down) and press Run.

**`local.properties` does not come from Git.** It is gitignored because it
holds a machine-specific path. Android Studio writes it on first open. If a
command-line build fails with *SDK location not found*, create
`Android/local.properties` yourself:

```properties
sdk.dir=C:/path/to/your/Android/Sdk
```

From the command line, with `JAVA_HOME` pointing at Android Studio's bundled
JDK:

```bash
./gradlew installDebug     # build and push to a connected device
./gradlew assembleDebug    # just build the APK
./gradlew test             # run the unit tests
```

**Toolchain:** AGP 9.3.1, Gradle 9.5, JDK 25, `compileSdk` 37, `minSdk` 24,
Kotlin via AGP's built-in support (there is deliberately no Kotlin plugin in
`build.gradle.kts` — AGP 9 handles it). This combination works; please don't
accept Android Studio's upgrade prompts without telling the team, because a
version bump breaks the build for everyone at once.

---

## Using the app

One screen, landscape, tablet-first. No drawer and no tabs — during a timed run
nobody should have to navigate.

```
┌──────────────────────────────┬─────────────────────┐
│                              │  ROBOT LINK         │
│                              │  ● Connected to …   │
│        ARENA                 │  [Connect][Disconn] │
│        20 × 20               │  Simulator      [ ] │
│                              ├─────────────────────┤
│   tap    → add obstacle      │  STATUS  (or)       │
│   drag   → move it           │  TRAFFIC            │
│   off    → delete it         │                     │
│   tap it → face compass      ├─────────────────────┤
│                              │  DRIVE              │
│                              │  [Fwd-L][Fwd][Fwd-R]│
│                              │  [Bck-L][Bck][Bck-R]│
│              ROBOT (7,2) W   │  [-][+]  [  STOP  ] │
├──────────────────────────────┼─────────────────────┤
│                              │ [Undo][Clear][Demo] │
└──────────────────────────────┴─────────────────────┘
```

**Arena**
- Tap an empty cell to add an obstacle. It takes the lowest free number.
- Drag an obstacle to move it. Drag it past the edge to delete it.
- Messages go out **once, on finger lift** — not continuously while dragging.
- Tap a placed obstacle to open a magnified four-quadrant compass, then tap N,
  E, S or W to mark the face holding the target image. An obstacle is one cell
  out of twenty across, far too small to hit an edge directly, so the compass
  is the touch interaction that satisfies C.7.
- Obstacle numbers are **never reused while an obstacle is alive**. Deleting B2
  leaves B3 called B3, because the robot has already been told about B3.

**Robot link**
- **Connect** lists paired and nearby devices. Pick one.
- **Simulator** swaps the real radio for a fake one — see below.

**Status** shows selected information only, never the raw stream. That is a
checklist requirement (C.4), not a style choice. Everything on the wire, both
directions, goes in **Traffic** instead. The `⌄` button swaps between them.

**Drive** issues one command per tap. Not auto-repeat: the STM board replies
`BUSY` and discards anything sent while it is already moving. `−`/`+` change
the distance step; tapping the readout cycles the turn angle.

---

## Testing without a robot

You do not need hardware, Bluetooth or even a tablet to work on this.

1. Turn on **Simulator**. The Traffic drawer opens automatically.
2. Press **Connect** and pick "Simulated robot".
3. Type any inbound message into the field at the bottom and press **Inject**.

```
ROBOT,7,2,W          robot moves to (7,2) facing West
TARGET,B2,11,N       obstacle 2 shows a large 11, red bar on its north face
target,b3,25,e       same thing — the parser is not fussy
MSG,[Looking for target 3]
STM,DONE
ROBOT,99,99,N        ignored, with a warning — out of bounds
!!!                  ignored silently, logged to Traffic
```

Everything the app *sends* is logged in Traffic too, so you can confirm your
own module will receive what it expects before you wire anything together.

---

## The message protocol

Full detail, including the coordinate convention, is in
[`PROTOCOL.md`](PROTOCOL.md). Summary:

### The app sends

| When | String | Item |
|---|---|---|
| Obstacle placed, or moved and finger lifted | `ADD,B<n>,(<x>,<y>)` | C.6 |
| Obstacle dragged off the arena | `SUB,B<n>` | C.6 |
| Target face annotated | `FACE,B<n>,<D>` | C.7 |
| Drive buttons | `FW<ddd>` `BW<ddd>` `FL<ddd>` `FR<ddd>` `BL<ddd>` `BR<ddd>` `STOP` | C.3 |

Motion commands are Kush's STM format: two-letter verb, three digits,
zero-padded. `FW010` is forward 10 cm; `FL090` is a 90° forward-left turn.

All outbound messages are newline-terminated.

### The app understands

| String | Effect | Item |
|---|---|---|
| `ROBOT,<x>,<y>,<D>` | Moves and turns the robot, drops a breadcrumb | C.10 |
| `TARGET,<n>,<id>` | Block `n` shows `<id>` in large white text | C.9 |
| `TARGET,<n>,<id>,<D>` | As above, plus a coloured bar on face `<D>` | C.9 |
| `MSG,[text]` | One line in the status box | C.4 |
| `STATUS,<text>` | One line in the status box | — |
| `STATUS,SENT,<cmd>` | "Sent `<cmd>` to the robot." | — |
| `STATUS,MAP,<msg>` | Receipt for one of our own map edits; Traffic only | — |
| `STM,<reply>` | Relayed board reply, see below | — |
| `ERR,<reason>` | Warning toast — something we sent was refused | — |

`STM,<reply>` handles `READY`, `DONE`, `ACK`, `BUSY`, `STALL`, `TIMEOUT`,
`ERR` and `NO_REPLY`. **`STALL` and `TIMEOUT` raise a visible warning**, because
the STM spec says position is unknown after either — the robot drawn on the map
is wrong until something re-references it.

### Parsing is deliberately forgiving

The written checklist and the ARCM briefing slides disagree about the `TARGET`
format, and a supervisor typing into the AMD tool by hand will produce whichever
they remember. So:

- `B2` and `2` are both accepted as obstacle numbers
- whitespace around any field is trimmed — `TARGET, 2, 11` works
- the keyword and direction letter are case-insensitive
- coordinates may be `(10,6)` or `10,6`
- a trailing empty field is ignored

Anything unrecognised is logged to Traffic and dropped. Anything out of range,
or naming an obstacle that doesn't exist, is dropped with a short on-screen
note. **Nothing throws and nothing crashes the app.** You can send it garbage
all day.

---

## Integrating with your module

### Raspberry Pi (Kenneth)

The tablet connects over Bluetooth SPP, UUID
`00001101-0000-1000-8000-00805F9B34FB`. While disconnected the app runs a
client retry *and* a server accept at the same time, so it recovers whether we
redial or you reconnect from your side — that is what makes C.8 work.

`rpi/a1_bridge.py` already routes correctly:

- motion commands → forwarded to the STM board
- `ADD` / `SUB` / `FACE` → acknowledged with `STATUS,MAP,…`, **not** forwarded
- anything else → `ERR,INVALID_COMMAND`

Verify any change to those patterns without hardware:

```bash
python3 rpi/test_a1_bridge.py
```

### Path planning (Faheem, WX)

Read the coordinate section of [`PROTOCOL.md`](PROTOCOL.md) before anything
else. The one most likely to bite:

> **The robot's `(x,y)` is the centre of its 3 × 3 footprint, not a corner.**
> Legal centres are 1–18.

Send `ROBOT,<x>,<y>,<D>` as the robot moves and the map follows it live,
leaving a breadcrumb trail behind. Obstacle positions reach you as
`ADD,B<n>,(<x>,<y>)` whenever the user finishes placing one.

### Image recognition (Denzel)

Send `TARGET,<obstacle>,<id>` or `TARGET,<obstacle>,<id>,<face>`.

- `<id>` must be **11–40**. Anything outside that range is ignored — that is
  the published image pool, 11–19 digits, 20–35 letters, 36–40 arrows and stop.
- `<obstacle>` must be a block the user has actually placed, otherwise the
  message is dropped with a note.
- `<face>` is optional and draws a coloured bar on that side.

### STM board (Kush, Wen Rong)

The drive pad emits your format unchanged. Every string the app can send lives
in one file, [`protocol/Outbound.kt`](app/src/main/java/com/example/androidapp/protocol/Outbound.kt),
so if the agreed vocabulary changes it is a one-file edit.

---

## Architecture

```
com.example.androidapp/
├─ MainActivity.kt        wiring only — no decisions, no drawing
├─ MdpViewModel.kt        every decision, both halves of the app
├─ arena/
│   ├─ ArenaModel.kt      pure Kotlin: grid, obstacles, robot, transitions
│   └─ ArenaView.kt       custom View: draws and hit-tests, nothing else
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
plain Kotlin, so they run as JVM unit tests in about a second — no emulator, no
tablet. Getting `ROBOT,7,2,W` parsing right via a JUnit test is a one-second
loop; doing it by installing an APK is ninety seconds. Please keep them clean.

`Link` is the seam. Everything about Bluetooth lives behind it; everything
about the arena lives in front of it and never imports `android.bluetooth`.
Swapping the real radio for the simulator is one line, which is why the map
could be built and demonstrated before the radio worked.

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

If you need a third transport — WiFi to the PC, say — implement `Link` and
nothing else in the app has to change.

---

## Running the tests

```bash
cd Android
./gradlew test
```

40 JVM tests, no device needed.

- `ProtocolTest` — every inbound format, both `TARGET` spellings, the Pi
  bridge's whole vocabulary, and a pile of garbage that must not crash it.
- `ArenaModelTest` — obstacle numbering and reuse, collisions, bounds, the
  3 × 3 robot footprint, target ID range, breadcrumbs.

Add to these when you change parsing or the model. They are fast and they are
the reason the protocol survived contact with the real bridge.

---

## An emulator that matches the real tablet

The assigned tablet is a **Samsung Galaxy Tab A7 Lite (SM-T220)**: 800 × 1340 at
213 dpi, which is about 1007 × 601 dp in landscape — a tight vertical budget
that has already caused real layout bugs. Testing on a phone-shaped emulator
will not catch them.

Create an AVD with those exact values (Android Studio → Device Manager → New →
set resolution 800 × 1340 and density 213), API 33, landscape.

**The emulator has no Bluetooth controller at all** — `adb shell service check
bluetooth` reports *not found*. Discovery returns nothing and connecting always
fails, no matter what the code does. Confusingly `pm list features` still
claims `android.hardware.bluetooth`. Use Simulator mode on the emulator, and
the real tablet for anything involving the radio.

---

## Known limits and gotchas

- **No Bluetooth on emulators.** As above. C.1, C.2 and C.8 can only be tested
  on the tablet.
- **The Pi bridge blocks while waiting for the board.** For up to 25 s after
  forwarding a motion command it reads nothing from Android, so obstacle edits
  made mid-move queue in the RFCOMM buffer until the move finishes. Fine for a
  demo, worth remembering during a timed run.
- **The robot cannot turn on the spot.** Ackermann steering: a 90° turn also
  carries the car 2.5–3.2 cells forward or back. That is why the buttons say
  "Fwd-left" rather than "Left". If the map ever animates a turn as a pivot in
  place, it is lying about the robot.
- **`STALL` and `TIMEOUT` invalidate the map.** After either, the robot's real
  position is unknown and the drawn position is stale until something
  re-references it.
- **Pair the tablet and the Pi in Android Settings first.** Not in code. If
  they are not bonded, nothing else works.

---

## Where to change things

| To change | Edit |
|---|---|
| Any string the app transmits | `protocol/Outbound.kt` |
| How an inbound message is understood | `protocol/Inbound.kt` |
| What a message *does* to the map | `MdpViewModel.kt` |
| How the arena looks | `arena/ArenaView.kt` (colours at the bottom) |
| Grid size, robot footprint, start pose, target ID range | `arena/ArenaModel.kt` |
| Bluetooth behaviour, reconnect strategy | `link/BluetoothLink.kt` |
| Screen layout | `res/layout/activity_main.xml` |

Keep decisions out of `ArenaView` and Android out of `ArenaModel` and
`protocol/` — that separation is what keeps the tests fast and the merges
small.
