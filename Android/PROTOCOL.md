# Android ↔ Robot protocol

Owner: Zhenxi (map) and Peter (transport). Anyone changing this file tells the
other five people in the team, because every module carries these coordinates.

Status: **proposed** — needs agreeing at the whiteboard, then this line becomes
"agreed <date>".

## Coordinate convention

This is the part that silently breaks integration, so it is written down rather
than remembered.

| | |
|---|---|
| Grid | 20 × 20 cells, 10 cm per cell, arena 2.0 m × 2.0 m |
| Origin | `(0,0)` is the **bottom-left** cell |
| Axes | `x` grows East, `y` grows North |
| Obstacle | occupies exactly one cell; its `(x,y)` **is** that cell |
| Robot | occupies 3 × 3 cells; its `(x,y)` is the **centre** cell |
| Legal robot centres | `1..18` in both axes — the footprint must stay inside |
| Start zone | `x,y ∈ 0..2`; the robot starts at `(1,1)` facing `N` |
| Direction | one of `N` `E` `S` `W` |
| Target IDs | `11..40` (11–19 digits, 20–35 letters, 36–40 arrows and stop) |

The car is 23.0 × 18.8 cm, which is why the footprint is three cells rather
than two.

## Android → robot

| When | String | Checklist |
|---|---|---|
| Obstacle placed, or moved and the finger lifted | `ADD,B<n>,(<x>,<y>)` | C.6 |
| Obstacle dragged off the arena | `SUB,B<n>` | C.6 |
| Target face annotated | `FACE,B<n>,<D>` | C.7 |

Exactly one `ADD` is sent per gesture, on `ACTION_UP` — never one per frame of
a drag.

Obstacle numbers are assigned lowest-free-first and are **never reassigned**
while the obstacle lives. Deleting B2 leaves B3 called B3, because the robot has
already been told about B3 by that name.

## Robot → Android

| String | Effect | Checklist |
|---|---|---|
| `ROBOT,<x>,<y>,<D>` | move and turn the robot on the map | C.10 |
| `TARGET,<n>,<id>` | block `n` shows `<id>` in large white text | C.9 |
| `TARGET,<n>,<id>,<D>` | as above, plus a thick line on face `<D>` | C.9 |
| `MSG,[text]` | one line in the status box | C.4 |

### Parsing is deliberately tolerant

The written checklist gives the target format as
`TARGET, <Obstacle Number>, <Target ID>`; the ARCM briefing slides give it as
`TARGET,B2,11`. A supervisor typing into the AMD tool by hand will produce
whichever they remember. The app accepts both, and:

- surrounding whitespace on every field is trimmed
- the keyword and the direction letter are case-insensitive
- obstacle numbers may be written `B2` or `2`
- coordinates may be written `(10,6)` or `10,6`
- a trailing empty field is ignored

Anything unrecognised is logged to the Traffic drawer and dropped. Anything out
of range, or naming an obstacle that does not exist, is dropped with a short
on-screen note. Nothing throws, and nothing reaches the status box that the
checklist would call "the whole incoming stream".

## Motion commands

Taken from `STM32_motion_spec.md` on the `main` branch. Two-letter verb, three
digits, one command at a time — the board replies `BUSY` and discards anything
sent while a move is running, so the drive pad is single-shot rather than
auto-repeat.

| Command | Meaning | Argument |
|---|---|---|
| `FW<ddd>` | forward | cm |
| `BW<ddd>` | backward | cm |
| `FL<ddd>` `FR<ddd>` | forward-left / forward-right | degrees |
| `BL<ddd>` `BR<ddd>` | reverse-left / reverse-right | degrees |
| `STOP` | abort the current move | — |

The car uses Ackermann steering and **cannot turn on the spot**: a 90° turn also
carries it 2.5–3.2 cells along. The buttons are labelled "Fwd-left" rather than
"Left" for that reason.

## Framing

Messages are newline-terminated on the way out. On the way in the reader accepts
both: it emits on `\n` or `\r`, and also flushes a complete buffer when nothing
further is waiting, because the AMD tool sends one message per write and does
not always terminate it.

## Transport

Bluetooth Serial Port Profile, UUID `00001101-0000-1000-8000-00805F9B34FB`.

While disconnected the app dials the chosen device *and* listens for an
incoming connection at the same time. C.8 asks the app to recover when the
remote side reconnects, so it has to be listening, not only retrying.
